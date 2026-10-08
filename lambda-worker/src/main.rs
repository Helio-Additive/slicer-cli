mod callback;
mod config;
mod model;
mod slicer;
mod storage;

use std::{path::Path, sync::Arc};

use anyhow::{Context, Result, anyhow};
use aws_lambda_events::event::sqs::{SqsBatchResponse, SqsEvent, SqsMessage};
use lambda_runtime::{Error as LambdaError, LambdaEvent, service_fn};
use serde::Deserialize;
use tracing::{error, info, warn};

use crate::{
    callback::CallbackClient,
    config::WorkerConfig,
    model::{SliceRequest, SliceStatus},
    slicer::{EngineIdentity, execute, validate_request},
    storage::Storage,
};

struct State {
    config: WorkerConfig,
    storage: Storage,
    callback: CallbackClient,
    identity: EngineIdentity,
}

#[tokio::main]
async fn main() -> Result<(), LambdaError> {
    tracing_subscriber::fmt()
        .json()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into()),
        )
        .with_target(false)
        .without_time()
        .init();

    let config = WorkerConfig::from_env()?;
    let aws = aws_config::load_defaults(aws_config::BehaviorVersion::latest()).await;
    let storage = Storage::new(aws_sdk_s3::Client::new(&aws), config.output_bucket.clone());
    let callback = CallbackClient::new(
        config.callback_url.clone(),
        config.callback_hmac_secret.clone(),
        config.callback_attempts,
        config.callback_timeout,
    )?;
    let identity = load_engine_identity(&config.slicer_root).await;
    let state = Arc::new(State {
        config,
        storage,
        callback,
        identity,
    });

    lambda_runtime::run(service_fn(move |event| {
        let state = Arc::clone(&state);
        async move { handle_event(event, state).await }
    }))
    .await
}

async fn handle_event(
    event: LambdaEvent<SqsEvent>,
    state: Arc<State>,
) -> Result<SqsBatchResponse, LambdaError> {
    let mut response = SqsBatchResponse::default();
    for message in event.payload.records {
        let message_id = message
            .message_id
            .clone()
            .unwrap_or_else(|| "missing-message-id".to_owned());
        if let Err(error) = process_message(&message, &state).await {
            error!(message_id, error = %error, "slice message failed and will be retried");
            response.add_failure(message_id);
        }
    }
    Ok(response)
}

async fn process_message(message: &SqsMessage, state: &State) -> Result<()> {
    let body = message.body.as_deref().context("SQS message has no body")?;
    let request: SliceRequest =
        serde_json::from_str(body).context("parsing slice request from SQS")?;
    let request_is_valid = validate_request(&state.config, &request).is_ok();
    let result_key = format!("{}.result.json", request.output.key);

    if request_is_valid && let Some(stored) = state.storage.load_result(&result_key).await? {
        validate_stored_result(&stored, &request)?;
        info!(
            job_id = request.job_id,
            "delivering previously stored slice result"
        );
        state.callback.send(&stored).await?;
        return Ok(());
    }

    let result = execute(&state.config, &state.storage, &state.identity, &request).await?;
    if request_is_valid {
        state.storage.store_result(&result_key, &result).await?;
    } else {
        warn!(
            job_id = request.job_id,
            "invalid request cannot use its untrusted result key; callback will not be checkpointed"
        );
    }
    state.callback.send(&result).await?;
    match result.status {
        SliceStatus::Succeeded => info!(job_id = request.job_id, "slice completed"),
        SliceStatus::Failed => warn!(job_id = request.job_id, "slice failed permanently"),
    }
    Ok(())
}

fn validate_stored_result(
    stored: &crate::model::SliceResult,
    request: &SliceRequest,
) -> Result<()> {
    if stored.schema_version != crate::model::SCHEMA_VERSION
        || stored.job_id != request.job_id
        || stored.engine != request.engine
    {
        return Err(anyhow!(
            "stored slice result identity does not match the request"
        ));
    }
    match stored.status {
        SliceStatus::Succeeded => {
            let output = stored
                .output
                .as_ref()
                .context("stored successful result has no output")?;
            if output.bucket != request.output.bucket
                || output.key != request.output.key
                || stored.error.is_some()
            {
                return Err(anyhow!(
                    "stored successful result has inconsistent output metadata"
                ));
            }
        }
        SliceStatus::Failed => {
            if stored.output.is_some() || stored.error.is_none() {
                return Err(anyhow!(
                    "stored failed result has inconsistent failure metadata"
                ));
            }
        }
    }
    Ok(())
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct PackageManifest {
    version: Option<String>,
    git_sha: Option<String>,
}

async fn load_engine_identity(root: &Path) -> EngineIdentity {
    let path = root.join("bin/manifest.json");
    match tokio::fs::read(&path).await {
        Ok(bytes) => match serde_json::from_slice::<PackageManifest>(&bytes) {
            Ok(manifest) => EngineIdentity {
                version: manifest.version,
                git_sha: manifest.git_sha,
            },
            Err(error) => {
                warn!(path = %path.display(), error = %error, "cannot parse slicer package manifest");
                EngineIdentity::default()
            }
        },
        Err(error) => {
            warn!(path = %path.display(), error = %error, "cannot read slicer package manifest");
            EngineIdentity::default()
        }
    }
}
