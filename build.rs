use std::{env, fs, path::PathBuf};

#[allow(dead_code)]
mod job_config {
    include!("src/job_config.rs");
}

use job_config::{
    FilamentProfile, JobConfig, MachineProfile, ProcessProfile, ProfileDefinition,
    ProfileReference, ProfileTriple, SlicerConfig, StringList,
};

fn main() {
    println!("cargo:rerun-if-changed=src/job_config.rs");

    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));
    let exported_dir = manifest_dir.join("schemas/json/exported");
    fs::create_dir_all(&exported_dir).expect("create exported schema directory");

    write_schema::<JobConfig>(&exported_dir, "config.schema.json", None);
    write_schema::<SlicerConfig>(&exported_dir, "slicer-config.schema.json", None);
    write_schema::<MachineProfile>(&exported_dir, "machine.schema.json", Some("MachineProfile"));
    write_schema::<FilamentProfile>(
        &exported_dir,
        "filament.schema.json",
        Some("FilamentProfile"),
    );
    write_schema::<ProcessProfile>(&exported_dir, "process.schema.json", Some("ProcessProfile"));
    write_schema::<ProfileReference>(&exported_dir, "profile-reference.schema.json", None);
    write_schema::<ProfileTriple>(&exported_dir, "profile-triple.schema.json", None);
    write_schema::<ProfileDefinition>(&exported_dir, "profile-definition.schema.json", None);
    write_schema::<StringList>(&exported_dir, "string-list.schema.json", None);
}

fn write_schema<T>(exported_dir: &PathBuf, filename: &str, title: Option<&str>)
where
    T: schemars::JsonSchema,
{
    let schema = schemars::schema_for!(T);
    let mut schema = serde_json::to_value(schema).expect("serialize schema value");
    if let Some(title) = title {
        schema["title"] = title.into();
    }
    let schema = serde_json::to_string_pretty(&schema).expect("serialize schema");
    let exported_schema = exported_dir.join(filename);
    let existing = fs::read_to_string(&exported_schema).ok();
    if existing.as_deref() != Some(schema.as_str()) {
        fs::write(exported_schema, schema).expect("write exported schema");
    }
}
