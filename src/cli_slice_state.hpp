#pragma once

#include "cli_load_settings.hpp"
#include "cli_plate_plan.hpp"
#include "cli_result_json.hpp"
#include <boost/filesystem/path.hpp>
#include "libslic3r/Model.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/Semver.hpp"

#include <algorithm>
#include <optional>

namespace slicer_cli {

/// One file a run writes, as check_run_outputs compares it (see there).
struct RunOutput {
    bool        folder = false;   // a folder the run writes into (--export-slicedata)
    bool        stl    = false;   // an object STL file of --export-stl / --export-stls
    std::string flag;             // "--export-3mf NAME"; empty for the run's own files
    boost::filesystem::path path;
    std::string what;             // "the run's own result.json in --outputdir"
    std::string is_what;          // what follows "is" in the same-name sentence
};

/// One plate of the run: its grid origin in the scene and its instances in
/// the run's model (PartPlate's origin and obj_to_instance_set). `members`
/// empty with `all` set: every instance of the model is on this plate (a
/// load of one plate, or one plate of model files).
struct RunPlate {
    Slic3r::Vec2d origin = Slic3r::Vec2d::Zero();
    bool all = true;
    std::vector<Slic3r::ModelInstance*> members;
    bool is_member(const Slic3r::ModelInstance* inst) const {
        return all || std::find(members.begin(), members.end(), inst) != members.end();
    }
};

struct LoadedProject {
    bool loaded = false;      // the inputs are in `model` and `base`
    Slic3r::Model model;
    Slic3r::PlateDataPtrs plate_data;
#ifdef ENGINE_SNAPMAKER
    // The 3MF printer switch remaps the shared model's paint once per run: every
    // plate pass trims its own copy of the settings, but the model is the run's.
    bool switch_paint_remapped = false;
#endif
    // Load-time facts every later step reads.
    bool is_bbl_3mf = false;
    Slic3r::Semver file_version;
    // Each plate's grid origin in the project (plate_grid_origin), for
    // the plates --downward-check loads alone; empty when not a project.
    std::vector<Slic3r::Vec2d> plate_origins;   // the file's grid, for --downward-check
    // What the official CLI reads from a Bambu-made 3MF, and its embedded
    // presets, for --load-settings and friends (cli_load_settings.cpp).
    slicer_cli::ProjectFacts project_facts;
    std::vector<Slic3r::Preset> project_presets_kept;
    bool project_loaded_like_desktop = false;   // the project was loaded over the system presets
};

struct RunSettings {
    // m_print_config: the project's (or the profiles') settings with the
    // settings merge and the flags applied, before any plate's own.
    Slic3r::DynamicPrintConfig base;
    // The settings given as flags; the settings merge takes filament_colour
    // out of them once it has used it, as the official does.
    Slic3r::DynamicPrintConfig extra;
    bool explicit_config_supplied_nozzle_map = false;
    slicer_cli::SettingsMerge settings_merge;
    std::string merged_print_sequence;
    int shrink_to_new_bed = 0;
    size_t project_filaments = 0;
};

struct RunPlatePlan {
    std::vector<RunPlate> plates;   // index = plate id - 1
    ProjectPlan plan;
    // A project 3MF with model files after it (ProjectPlan): the file's own
    // settings and version, for the plate list's size, and its filament
    // count, the used filaments of the official's tower step.
    std::optional<Slic3r::DynamicPrintConfig> plan_file_config;
    Slic3r::Semver plan_file_version;
    int plan_file_filaments = 0;
    // --slice: the plates the input declares before any plan (run_slice_mode).
    int file_plate_count = 1;
    // Plate `plate_id` of the run; a run of one plate (model files, a 3MF
    // without plates) answers with that plate whatever the id.
    RunPlate& plate(int plate_id) {
        if (plates.empty())
            plates.assign(1, RunPlate());
        if (plates.size() == 1 || plate_id < 1 || plate_id > (int)plates.size())
            return plates.front();
        return plates[plate_id - 1];
    }
};

struct PlatePreparation {
    bool prepared = false;    // the steps before the plate loop ran
    int prepared_plate = 0;   // the plate the run-level steps ran for (0: every plate)
    // The settings the steps before the plate loop read and changed (the
    // arrange's tower): `prepared_plate`'s own, or the project's for every plate.
    Slic3r::DynamicPrintConfig run_config;
    // What the load and the run-level steps record for every plate's result.
    PlateOutcome carry;
};

struct PlateWarnings {
    // The settings check's warnings, per plate (InvalidValuesNoted).
    std::map<int, nlohmann::json> noted;
    // --slice 0's check pass runs every plate before any is sliced: the
    // warnings it finds are announced only once every plate passed (the
    // warning promises a run that slices), so it holds them here.
    bool hold_noted = false;
    std::vector<nlohmann::json> noted_held;
};

struct RunPipeProgress {
    // --pipe: steps 2 and 3 go out once per run, before any plate is
    // checked (BambuStudio.cpp 4923-4926 before the transforms, 6455-6458 as
    // the slice action starts, after pre_check is set at 6441; OrcaSlicer.cpp
    // 4185 and 5580, after 5565).
    bool pipe_files_loaded_sent = false;
    bool pipe_prepare_sent = false;
};

struct RunActionOutputs {
    // The model actions run once per run (BambuStudio.cpp 6336; OrcaSlicer.cpp 5470).
    bool actions_done = false;
    // The outputs were checked before the first write (check_run_outputs).
    bool outputs_checked = false;
    // The outputs the actions given before --slice wrote: the check after the
    // calibration pattern replaced the model reserves them (see
    // check_run_outputs' `reserved`).
    std::vector<RunOutput> reserved_outputs;
};

/// The run's one model and one settings base, as the official CLI keeps them:
/// every input is loaded once into one model (BambuStudio.cpp 1855-2163,
/// 3947-3963; OrcaSlicer.cpp 1537-1810, 3441-3457), every object at its
/// place in the scene, and m_print_config is built once from the file, the
/// profiles, the settings merge and the flags (BambuStudio.cpp 4091;
/// OrcaSlicer.cpp 3542). Each plate is then sliced from that model with the
/// plate's origin on its Print (PartPlate::get_print, set_plate_origin;
/// BambuStudio.cpp 6506; OrcaSlicer.cpp 5623) and a copy of the settings with
/// the plate's own laid on top (BambuStudio.cpp 6902-6904; OrcaSlicer.cpp
/// 5905-5907).
struct RunState {
    LoadedProject project;
    RunSettings settings;
    RunPlatePlan layout;
    PlatePreparation preparation;
    PlateWarnings warnings;
    RunPipeProgress progress;
    RunActionOutputs outputs;

    ~RunState() { Slic3r::release_PlateData_list(project.plate_data); }
};

} // namespace slicer_cli
