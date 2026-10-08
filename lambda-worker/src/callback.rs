use std::{time::Duration, time::SystemTime};

use anyhow::{Context, Result, anyhow, bail};
use hmac::{Hmac, Mac};
use reqwest::{Client, Url};
use sha2::Sha256;
use tokio::time::sleep;

use crate::model::SliceResult;

const SIGNATURE_HEADER: &str = "x-helio-signature";
const TIMESTAMP_HEADER: &str = "x-helio-timestamp";
const EVENT_HEADER: &str = "x-helio-event";
const EVENT_NAME: &str = "slice.finished";

type HmacSha256 = Hmac<Sha256>;

#[derive(Clone)]
pub struct CallbackClient {
    client: Client,
    url: Url,
    secret: Vec<u8>,
    attempts: usize,
}

impl CallbackClient {
    pub fn new(url: Url, secret: Vec<u8>, attempts: usize, timeout: Duration) -> Result<Self> {
        if attempts == 0 {
            bail!("callback attempts must be greater than zero");
        }
        let client = Client::builder()
            .timeout(timeout)
            .user_agent(concat!("slicer-lambda/", env!("CARGO_PKG_VERSION")))
            .build()
            .context("building callback HTTP client")?;
        Ok(Self {
            client,
            url,
            secret,
            attempts,
        })
    }

    pub async fn send(&self, result: &SliceResult) -> Result<()> {
        let body = serde_json::to_vec(result).context("serializing callback payload")?;
        let mut last_error = None;

        for attempt in 0..self.attempts {
            let timestamp = unix_timestamp()?;
            let signature = sign(&self.secret, timestamp, &body)?;
            let response = self
                .client
                .post(self.url.clone())
                .header("content-type", "application/json")
                .header(EVENT_HEADER, EVENT_NAME)
                .header(TIMESTAMP_HEADER, timestamp.to_string())
                .header(SIGNATURE_HEADER, format!("v1={signature}"))
                .header("idempotency-key", &result.job_id)
                .body(body.clone())
                .send()
                .await;

            match response {
                Ok(response) if response.status().is_success() => return Ok(()),
                Ok(response) => {
                    let status = response.status();
                    last_error = Some(anyhow!("callback returned HTTP {status}"));
                }
                Err(error) => last_error = Some(anyhow!(error).context("sending callback")),
            }

            if attempt + 1 < self.attempts {
                sleep(Duration::from_millis(250 * (1u64 << attempt.min(4)))).await;
            }
        }

        Err(last_error.unwrap_or_else(|| anyhow!("callback failed without an error")))
    }
}

pub(crate) fn sign(secret: &[u8], timestamp: u64, body: &[u8]) -> Result<String> {
    let mut mac = HmacSha256::new_from_slice(secret).context("initializing callback HMAC")?;
    mac.update(timestamp.to_string().as_bytes());
    mac.update(b".");
    mac.update(body);
    Ok(hex::encode(mac.finalize().into_bytes()))
}

fn unix_timestamp() -> Result<u64> {
    Ok(SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .context("system clock is before the Unix epoch")?
        .as_secs())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signature_is_stable_and_covers_timestamp_and_body() {
        let signature = sign(
            b"0123456789abcdef0123456789abcdef",
            1_700_000_000,
            br#"{"jobId":"job-1"}"#,
        )
        .unwrap();
        assert_eq!(
            signature,
            "821cf633699eb2374baa61ee528c44a63d05384de2ca72a093bc79440667b202"
        );
        assert_ne!(
            signature,
            sign(
                b"0123456789abcdef0123456789abcdef",
                1_700_000_001,
                br#"{"jobId":"job-1"}"#
            )
            .unwrap()
        );
    }

    #[tokio::test]
    async fn posts_authenticated_idempotent_callback() {
        use tokio::{
            io::{AsyncReadExt, AsyncWriteExt},
            net::TcpListener,
        };

        use crate::model::{Engine, SliceStatus};

        let secret = b"0123456789abcdef0123456789abcdef".to_vec();
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let expected_secret = secret.clone();
        let server = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            let mut received = Vec::new();
            let mut chunk = [0u8; 4096];
            let (header_end, content_length) = loop {
                let count = stream.read(&mut chunk).await.unwrap();
                assert!(count > 0);
                received.extend_from_slice(&chunk[..count]);
                if let Some(header_end) = received.windows(4).position(|value| value == b"\r\n\r\n")
                {
                    let header_end = header_end + 4;
                    let headers = String::from_utf8_lossy(&received[..header_end]);
                    let content_length = headers
                        .lines()
                        .find_map(|line| {
                            let (name, value) = line.split_once(':')?;
                            name.eq_ignore_ascii_case("content-length")
                                .then(|| value.trim().parse::<usize>().unwrap())
                        })
                        .unwrap();
                    break (header_end, content_length);
                }
            };
            while received.len() < header_end + content_length {
                let count = stream.read(&mut chunk).await.unwrap();
                assert!(count > 0);
                received.extend_from_slice(&chunk[..count]);
            }
            let headers = String::from_utf8_lossy(&received[..header_end]);
            assert!(headers.contains("x-helio-event: slice.finished"));
            assert!(headers.contains("idempotency-key: job-1"));
            let timestamp = headers
                .lines()
                .find_map(|line| {
                    let (name, value) = line.split_once(':')?;
                    name.eq_ignore_ascii_case(TIMESTAMP_HEADER)
                        .then(|| value.trim().parse::<u64>().unwrap())
                })
                .unwrap();
            let supplied_signature = headers
                .lines()
                .find_map(|line| {
                    let (name, value) = line.split_once(':')?;
                    name.eq_ignore_ascii_case(SIGNATURE_HEADER)
                        .then(|| value.trim().strip_prefix("v1=").unwrap().to_owned())
                })
                .unwrap();
            let body = &received[header_end..header_end + content_length];
            assert_eq!(
                supplied_signature,
                sign(&expected_secret, timestamp, body).unwrap()
            );
            let callback: SliceResult = serde_json::from_slice(body).unwrap();
            assert_eq!(callback.job_id, "job-1");
            stream
                .write_all(b"HTTP/1.1 204 No Content\r\ncontent-length: 0\r\n\r\n")
                .await
                .unwrap();
        });

        let callback = CallbackClient::new(
            Url::parse(&format!("http://{address}/callback")).unwrap(),
            secret,
            1,
            Duration::from_secs(2),
        )
        .unwrap();
        callback
            .send(&SliceResult {
                schema_version: 1,
                job_id: "job-1".into(),
                status: SliceStatus::Failed,
                engine: Engine::Bambu,
                duration_ms: 10,
                engine_version: None,
                engine_git_sha: None,
                deployment_version: None,
                output: None,
                diagnostics: vec![],
                error: None,
            })
            .await
            .unwrap();
        server.await.unwrap();
    }
}
