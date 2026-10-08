use std::path::Path;

use anyhow::{Context, Result, anyhow};
use aws_sdk_s3::{Client, primitives::ByteStream};
use sha2::{Digest, Sha256};
use tokio::{fs::File, io::AsyncWriteExt};

use crate::model::{InputObject, OutputArtifact, OutputObject, SliceResult};

#[derive(Debug, thiserror::Error)]
pub enum TransferError {
    #[error("{0}")]
    Rejected(String),
    #[error(transparent)]
    Unavailable(#[from] anyhow::Error),
}

#[derive(Clone)]
pub struct Storage {
    client: Client,
    output_bucket: String,
}

impl Storage {
    pub fn new(client: Client, output_bucket: String) -> Self {
        Self {
            client,
            output_bucket,
        }
    }

    pub async fn download_verified(
        &self,
        object: &InputObject,
        destination: &Path,
        max_bytes: u64,
    ) -> std::result::Result<u64, TransferError> {
        let mut head = self
            .client
            .head_object()
            .bucket(&object.bucket)
            .key(&object.key);
        if let Some(version_id) = &object.version_id {
            head = head.version_id(version_id);
        }
        let head = head.send().await.map_err(|error| {
            TransferError::Unavailable(anyhow!(error).context("reading input object metadata"))
        })?;
        let content_length = head.content_length().unwrap_or_default();
        if content_length <= 0 {
            return Err(TransferError::Rejected("input object is empty".into()));
        }
        let content_length = u64::try_from(content_length).map_err(|error| {
            TransferError::Unavailable(anyhow!(error).context("negative input object size"))
        })?;
        if content_length > max_bytes {
            return Err(TransferError::Rejected(format!(
                "input object is {content_length} bytes; limit is {max_bytes} bytes"
            )));
        }

        let mut get = self
            .client
            .get_object()
            .bucket(&object.bucket)
            .key(&object.key);
        if let Some(version_id) = &object.version_id {
            get = get.version_id(version_id);
        }
        let response = get.send().await.map_err(|error| {
            TransferError::Unavailable(anyhow!(error).context("downloading input object"))
        })?;
        let mut reader = response.body.into_async_read();
        let mut output = File::create(destination).await.map_err(|error| {
            TransferError::Unavailable(
                anyhow!(error).context(format!("creating {}", destination.display())),
            )
        })?;
        let copied = tokio::io::copy(&mut reader, &mut output)
            .await
            .map_err(|error| {
                TransferError::Unavailable(anyhow!(error).context("streaming input object to disk"))
            })?;
        output.flush().await.map_err(|error| {
            TransferError::Unavailable(anyhow!(error).context("flushing downloaded input"))
        })?;
        if copied != content_length {
            return Err(TransferError::Rejected(format!(
                "downloaded {copied} bytes but S3 reported {content_length} bytes"
            )));
        }

        let actual_sha256 = sha256_file(destination)
            .await
            .map_err(TransferError::Unavailable)?;
        if !constant_time_hex_eq(&actual_sha256, &object.sha256) {
            return Err(TransferError::Rejected(format!(
                "input SHA-256 mismatch: expected {}, received {actual_sha256}",
                object.sha256
            )));
        }
        Ok(copied)
    }

    pub async fn upload_output(
        &self,
        job_id: &str,
        destination: &OutputObject,
        source: &Path,
        max_bytes: u64,
    ) -> std::result::Result<OutputArtifact, TransferError> {
        let metadata = tokio::fs::metadata(source).await.map_err(|error| {
            TransferError::Unavailable(
                anyhow!(error).context(format!("reading {} metadata", source.display())),
            )
        })?;
        let size = metadata.len();
        if size == 0 {
            return Err(TransferError::Rejected(
                "slicer produced an empty output file".into(),
            ));
        }
        if size > max_bytes {
            return Err(TransferError::Rejected(format!(
                "output is {size} bytes; limit is {max_bytes} bytes"
            )));
        }
        let sha256 = sha256_file(source)
            .await
            .map_err(TransferError::Unavailable)?;
        let body = ByteStream::from_path(source).await.map_err(|error| {
            TransferError::Unavailable(anyhow!(error).context("opening output file for upload"))
        })?;
        let response = self
            .client
            .put_object()
            .bucket(&destination.bucket)
            .key(&destination.key)
            .content_type("text/x-gcode")
            .metadata("job-id", job_id)
            .metadata("sha256", &sha256)
            .body(body)
            .send()
            .await
            .map_err(|error| {
                TransferError::Unavailable(anyhow!(error).context("uploading sliced G-code"))
            })?;

        Ok(OutputArtifact {
            bucket: destination.bucket.clone(),
            key: destination.key.clone(),
            size_bytes: size,
            sha256,
            etag: response.e_tag().map(ToOwned::to_owned),
            version_id: response.version_id().map(ToOwned::to_owned),
        })
    }

    pub async fn load_result(&self, key: &str) -> Result<Option<SliceResult>> {
        let head = self
            .client
            .head_object()
            .bucket(&self.output_bucket)
            .key(key)
            .send()
            .await;
        let head = match head {
            Ok(head) => head,
            Err(error)
                if error
                    .as_service_error()
                    .is_some_and(|service| service.is_not_found()) =>
            {
                return Ok(None);
            }
            Err(error) => return Err(anyhow!(error).context("checking stored slice result")),
        };
        if head.content_length().unwrap_or_default() > 1024 * 1024 {
            return Err(anyhow!("stored slice result exceeds 1 MiB"));
        }
        let response = self
            .client
            .get_object()
            .bucket(&self.output_bucket)
            .key(key)
            .send()
            .await
            .context("loading stored slice result")?;
        let bytes = response
            .body
            .collect()
            .await
            .context("reading stored slice result")?
            .into_bytes();
        if bytes.len() > 1024 * 1024 {
            return Err(anyhow!("stored slice result exceeds 1 MiB"));
        }
        let result = serde_json::from_slice(&bytes).context("parsing stored slice result")?;
        Ok(Some(result))
    }

    pub async fn store_result(&self, key: &str, result: &SliceResult) -> Result<()> {
        let body = serde_json::to_vec(result).context("serializing slice result")?;
        if body.len() > 1024 * 1024 {
            return Err(anyhow!("slice result exceeds 1 MiB"));
        }
        self.client
            .put_object()
            .bucket(&self.output_bucket)
            .key(key)
            .content_type("application/json")
            .metadata("job-id", &result.job_id)
            .body(ByteStream::from(body))
            .send()
            .await
            .context("storing slice result")?;
        Ok(())
    }
}

pub async fn sha256_file(path: &Path) -> Result<String> {
    let mut file = File::open(path)
        .await
        .with_context(|| format!("opening {} for hashing", path.display()))?;
    let mut hasher = Sha256::new();
    let mut buffer = vec![0u8; 1024 * 1024];
    loop {
        let count = tokio::io::AsyncReadExt::read(&mut file, &mut buffer)
            .await
            .context("hashing file")?;
        if count == 0 {
            break;
        }
        hasher.update(&buffer[..count]);
    }
    Ok(hex::encode(hasher.finalize()))
}

fn constant_time_hex_eq(actual: &str, expected: &str) -> bool {
    use subtle::ConstantTimeEq;
    actual.len() == expected.len()
        && bool::from(
            actual
                .as_bytes()
                .ct_eq(expected.to_ascii_lowercase().as_bytes()),
        )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hash_comparison_is_case_insensitive_and_exact() {
        assert!(constant_time_hex_eq("abcdef", "ABCDEF"));
        assert!(!constant_time_hex_eq("abcdef", "abcdee"));
        assert!(!constant_time_hex_eq("abcdef", "abcdef00"));
    }
}
