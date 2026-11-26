//! Printer selection in the Bambu root CLI (main.cpp 2258–2272, 2439–2499, 9696–9790).
use slicer::app_slice::NozzleVariantRefusal;
use slicer::config::ConfigOptionResolver;
use slicer::print_config::config_def::DynamicPrintConfig;
use std::{
    fs,
    path::{Path, PathBuf},
};

pub(super) fn profiles_dir() -> Option<PathBuf> {
    let exe = std::env::current_exe().ok()?.canonicalize().ok()?;
    let dir = exe.parent()?;
    [
        "../references/BambuStudio/resources/profiles",
        "../../references/BambuStudio/resources/profiles",
        "../resources/profiles",
        "resources/profiles",
    ]
    .iter()
    .map(|path| dir.join(path))
    .find(|path| path.is_dir())
}

struct PrinterPreset {
    name: String,
    variant: String,
}

fn shipped_printer_presets(profiles: &Path, model: &str) -> Vec<PrinterPreset> {
    let mut presets = Vec::new();
    if model.is_empty() {
        return presets;
    }
    let Ok(vendors) = fs::read_dir(profiles) else {
        return presets;
    };
    for vendor in vendors.flatten() {
        let Ok(files) = fs::read_dir(vendor.path().join("machine")) else {
            continue;
        };
        for entry in files.flatten() {
            let path = entry.path();
            if path.extension().and_then(|s| s.to_str()) != Some("json") {
                continue;
            }
            let Some(name) = path.file_stem().and_then(|s| s.to_str()) else {
                continue;
            };
            let Ok(file) = fs::File::open(&path) else {
                continue;
            };
            let Ok(preset) =
                serde_json::from_reader::<_, serde_json::Value>(std::io::BufReader::new(file))
            else {
                continue;
            };
            if preset["type"] == "machine_model" || preset["printer_model"].as_str() != Some(model)
            {
                continue;
            }
            presets.push(PrinterPreset {
                name: name.to_owned(),
                variant: preset["printer_variant"]
                    .as_str()
                    .unwrap_or_default()
                    .to_owned(),
            });
        }
    }
    presets.sort_by(|a, b| a.name.cmp(&b.name));
    presets
}

pub(super) fn check_nozzle_variant(
    config: &DynamicPrintConfig,
    given: &str,
) -> Option<NozzleVariantRefusal> {
    let wanted = slicer::config::complete_finite_double(given)?;
    let current = config
        .option("nozzle_diameter")?
        .vector_values_any()?
        .downcast_ref::<Vec<f64>>()?;
    if current.iter().all(|v| (v - wanted).abs() < 1e-6) {
        return None;
    }
    let id = |key| {
        config
            .option(key)
            .and_then(|opt| opt.single_value_any()?.downcast_ref::<String>())
            .map(String::as_str)
            .unwrap_or_default()
    };
    let model = id("printer_model");
    let variant = id("printer_variant");
    let mut preset = id("printer_settings_id");
    let shipped = profiles_dir()
        .map(|path| shipped_printer_presets(&path, model))
        .unwrap_or_default();
    let by_variant = |v: &str| {
        shipped
            .iter()
            .find(|p| !v.is_empty() && p.variant == v)
            .map(|p| p.name.as_str())
    };
    if !shipped.iter().any(|p| p.name == preset) {
        preset = by_variant(variant).unwrap_or(preset);
    }
    let format_mm = |v| slicer::locales_utils::float_to_string_decimal_point(v, -1);
    let requested = format_mm(wanted);
    let instead = if let Some(name) = by_variant(&requested) {
        format!("--printer-preset \"{name}\"")
    } else if !model.is_empty() {
        format!("--printer-preset \"{model} {requested} nozzle\"")
    } else {
        format!("the printer preset of a {requested} mm nozzle")
    };
    let printer = if preset.is_empty() {
        "this printer".into()
    } else {
        format!("'{preset}'")
    };
    let has = current
        .iter()
        .map(|v| format_mm(*v))
        .collect::<Vec<_>>()
        .join(", ");
    let mut message = format!("--nozzle {given} does not change the printer: {printer} is built for a {has} mm nozzle. The desktop app changes the nozzle by choosing the printer variant, so give the printer preset of that nozzle instead: {instead}.");
    let names: Vec<_> = shipped.iter().take(4).map(|p| p.name.as_str()).collect();
    if let Some((last, others)) = names.split_last() {
        message.push_str(" This engine ships ");
        if !others.is_empty() {
            message.push_str(&others.join(", "));
            message.push_str(" and ");
        }
        message.push_str(last);
        message.push('.');
    }
    Some(NozzleVariantRefusal {
        given: given.to_owned(),
        printer_variant: variant.to_owned(),
        printer_preset: preset.to_owned(),
        message,
    })
}
