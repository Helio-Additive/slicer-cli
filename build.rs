use std::{env, fs, path::PathBuf};

#[allow(dead_code)]
mod job_config {
    include!("src/job_config.rs");
}

use job_config::JobConfig;

fn main() {
    println!("cargo:rerun-if-changed=src/job_config.rs");

    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));
    let exported_dir = manifest_dir.join("schemas/json/exported");
    fs::create_dir_all(&exported_dir).expect("create exported schema directory");

    let schema = schemars::schema_for!(JobConfig);
    let schema = serde_json::to_string_pretty(&schema).expect("serialize config schema");
    let exported_schema = exported_dir.join("config.schema.json");
    let existing = fs::read_to_string(&exported_schema).ok();
    if existing.as_deref() != Some(schema.as_str()) {
        fs::write(exported_schema, schema).expect("write exported config schema");
    }
}
