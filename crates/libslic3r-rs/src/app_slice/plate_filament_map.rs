use super::DynamicPrintConfig;
use anyhow::Result;

/// main.cpp 10430–10518. A saved plate map is authoritative in a manual
/// mode. In an automatic mode ToolOrdering assigns the actual nozzle later.
pub(super) fn apply_plate_filament_map(
    plate: &crate::format::bbs_3mf::PlateData,
    config: &mut DynamicPrintConfig,
    settings: &mut serde_json::Value,
) -> Result<()> {
    use crate::config::{ConfigBase, ConfigOptionEnum, ConfigOptionInts, ConfigOptionResolver};
    use crate::print_config::enums::FilamentMapMode::*;
    let mut mode = crate::print_config::enums::FilamentMapMode::from_i32(
        crate::gcode::wipe_tower::cfg_enum(config, "filament_map_mode"),
    );
    if let Some(plate_mode) = plate.config.option("filament_map_mode") {
        let plate_mode =
            crate::print_config::enums::FilamentMapMode::from_i32(plate_mode.get_int()?);
        if plate_mode != FmmDefault {
            mode = plate_mode;
            config.set_key_value(
                "filament_map_mode",
                Box::new(ConfigOptionEnum::with_value(mode)),
            );
        }
    }
    if !plate.filament_maps.is_empty() && matches!(mode, FmmManual | FmmNozzleManual) {
        let maps = &plate.filament_maps;
        config.set_key_value(
            "filament_map",
            Box::new(ConfigOptionInts::from_values(maps.clone())),
        );
        if let Some(volume) = plate.config.opt::<ConfigOptionInts>("filament_volume_map") {
            if volume.values.len() == maps.len() {
                config.set_key_value(
                    "filament_volume_map",
                    Box::new(ConfigOptionInts::from_values(volume.values.clone())),
                );
            }
        }
        let mirror = maps.iter().map(|id| id - 1).collect();
        config.set_key_value(
            "filament_map_2",
            Box::new(ConfigOptionInts::from_values(mirror)),
        );
        let rack = crate::gcode::wipe_tower::cfg_ints(config, "extruder_max_nozzle_count")
            .iter()
            .any(|&n| n > 1);
        mode = if rack { FmmNozzleManual } else { FmmManual };
        config.set_key_value(
            "filament_map_mode",
            Box::new(ConfigOptionEnum::with_value(mode)),
        );
    }
    let count = crate::g_code::front_end_project_filament_count(config);
    crate::g_code::front_end_align_per_filament_maps(config, count);
    settings["filament_map_mode"] = config.opt_serialize("filament_map_mode")?.into();
    for key in [
        "filament_map",
        "filament_map_2",
        "filament_volume_map",
        "filament_nozzle_map",
    ] {
        if let Some(opt) = config.opt::<ConfigOptionInts>(key) {
            settings[key] = opt
                .values
                .iter()
                .map(i32::to_string)
                .collect::<Vec<_>>()
                .into();
        }
    }
    Ok(())
}

/// main.cpp 3767–3985. The model has been scoped to this plate.
fn arrange_plate_extruders(
    model: &crate::model::Model,
    config: &DynamicPrintConfig,
    plate_index: i32,
) -> Vec<i32> {
    let features = crate::model::GlobalFeatureFilaments::from_full_config(config);
    let mut used = Vec::new();
    for object in &model.objects {
        if !object.instances.iter().any(|instance| instance.printable) {
            continue;
        }
        for volume in &object.volumes {
            used.extend(volume.get_extruders(object, &features));
        }
        for range in object.layer_config_ranges.values() {
            used.extend(
                [
                    "extruder",
                    "wall_filament",
                    "sparse_infill_filament",
                    "solid_infill_filament",
                ]
                .iter()
                .filter_map(|key| range.option(key)?.get_int().ok())
                .filter(|&id| id > 0),
            );
        }
        let support = object
            .get_config_value(config, "enable_support")
            .and_then(|v| v.get_bool().ok())
            .unwrap_or(false);
        let raft = object
            .get_config_value(config, "raft_layers")
            .and_then(|v| v.get_int().ok())
            .unwrap_or(0);
        if support || raft > 0 {
            for key in ["support_interface_filament", "support_filament"] {
                let id = object
                    .config
                    .option(key)
                    .and_then(|v| v.get_int().ok())
                    .filter(|&id| id != 0)
                    .unwrap_or_else(|| config.opt_int(key));
                if id != 0 {
                    used.push(id);
                }
            }
        }
    }
    if let Some(changes) = model.plates_custom_gcodes.get(&plate_index) {
        let count = crate::gcode::wipe_tower::cfg_strings(config, "filament_colour").len() as i32;
        used.extend(
            changes
                .gcodes
                .iter()
                .filter(|g| {
                    g.gcode_type == crate::custom_g_code::Type::ToolChange && g.extruder <= count
                })
                .map(|g| g.extruder),
        );
    }
    used.sort_unstable();
    used.dedup();
    // PartPlate.cpp 1414-1426 expands each mixed slot into its physical filaments.
    if let Some(mixed) = config.opt::<crate::config::ConfigOptionBools>("filament_is_mixed") {
        if crate::filament_mixer::has_any_mixed_filament(&mixed.values) {
            let components =
                crate::gcode::wipe_tower::cfg_strings(config, "filament_mixed_components");
            let slots: Vec<u32> = used
                .iter()
                .filter(|&&id| id >= 1)
                .map(|id| (id - 1) as u32)
                .collect();
            return crate::filament_mixer::expand_mixed_filaments(
                &slots,
                &mixed.values,
                &components,
            )
            .into_iter()
            .map(|slot| slot as i32 + 1)
            .collect();
        }
    }
    used
}

#[derive(Debug, thiserror::Error)]
#[error("{0}")]
pub struct ManualMapRefusal(String);

/// cli_nozzle_options.cpp 257–284 and main.cpp 8180–8194.
pub(super) fn check_plate_extruders(
    config: &DynamicPrintConfig,
    model: &crate::model::Model,
    plate_index: i32,
) -> Result<()> {
    use crate::gcode::wipe_tower::{cfg_enum, cfg_floats, cfg_ints, cfg_strings};
    use crate::print_config::enums::FilamentMapMode::{self, FmmManual, FmmNozzleManual};
    if !matches!(
        FilamentMapMode::from_i32(cfg_enum(config, "filament_map_mode")),
        FmmManual | FmmNozzleManual
    ) {
        return Ok(());
    }
    let heads = cfg_floats(config, "nozzle_diameter").len().max(1);
    let maps = cfg_ints(config, "filament_map");
    let masks = cfg_ints(config, "filament_printable");
    let types = cfg_strings(config, "filament_type");
    for id in arrange_plate_extruders(model, config, plate_index) {
        if id < 1 {
            continue;
        }
        let index = (id - 1) as usize;
        let (Some(&head), Some(&mask)) = (maps.get(index), masks.get(index)) else {
            continue;
        };
        if head < 1
            || head as usize > heads
            || mask.checked_shr((head - 1) as u32).unwrap_or_default() & 1 != 0
        {
            continue;
        }
        let suffix = |name: &str| {
            if name.is_empty() {
                String::new()
            } else {
                format!(" ({name})")
            }
        };
        let kind = suffix(types.get(index).map_or("", String::as_str));
        let side = match (heads, head) {
            (2, 1) => " (left)",
            (2, 2) => " (right)",
            _ => "",
        };
        let printer = suffix(config.opt_string("printer_settings_id"));
        return Err(ManualMapRefusal(format!("Filament {id}{kind} cannot print with extruder {head}{side} of this printer{printer}, which the filament map names for it. Give it another extruder in --filament-map, or an automatic --filament-map-mode.")).into());
    }
    Ok(())
}
