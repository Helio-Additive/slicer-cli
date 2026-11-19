// libslic3r_standalone - CLI tool for slicing with libslic3r
// Supports loading BambuStudio JSON config files (machine, filament, process)

#include <iostream>
#include <limits>
#include <stdexcept>
#include <fstream>
#include <string>
#include <memory>
#include <map>
#include <vector>
#include <optional>
#include <iterator>
#include <set>
#include <cstdint>
#include <cstdlib>
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#ifdef _WIN32
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#else
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>
#endif
#include <functional>

// Core libslic3r headers
// Bambu 2.9.1 Log.hpp occupies boost/log/trivial.hpp's include guard, so boost's
// severity types must be declared before any engine header (as the engine's own Log.cpp does).
#include <boost/log/trivial.hpp>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Utils.hpp"   // get_process_name/get_current_pid: the engine's own folder-ownership test
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Thread.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Format/STL.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/AppConfig.hpp"
#include "libslic3r/miniz_extension.hpp"
#include "libslic3r/ModelArrange.hpp"
#include "libslic3r/Arrange.hpp"
#include "libslic3r/GCode/WipeTower.hpp"
#ifdef ENGINE_BAMBU
#include "libslic3r/FilamentMixer.hpp"
#endif
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/Semver.hpp"
#include "libslic3r_version.h"
#include "cli_snapmaker.hpp"
#include "cli_gcode_timing.hpp"

// For JSON parsing (using libslic3r's built-in nlohmann/json)
#include <nlohmann/json.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem.hpp>
#include <boost/nowide/args.hpp>
#include <boost/nowide/convert.hpp>
#include <boost/nowide/fstream.hpp>
// boost::nowide::nowide_filesystem: <boost/nowide/filesystem.hpp> since Boost
// 1.73 (OrcaSlicer.cpp 44); the older bundled Nowide calls the header
// integration/filesystem.hpp (BambuStudio.cpp 43).
#if __has_include(<boost/nowide/filesystem.hpp>)
#include <boost/nowide/filesystem.hpp>
#else
#include <boost/nowide/integration/filesystem.hpp>
#endif

#include <csignal>
#include "diagnostic_output.hpp"

#include "calib_args.hpp"
#include "layout_plan.hpp"
#include "cli_options.hpp"
#include "cli_command_line.hpp"
#include "cli_flag_support.hpp"
#include "desktop_presets.hpp"
#include "cli_model_load.hpp"
#include "cli_failure_reason.hpp"
#include "cli_run_steps.hpp"
#include "cli_simplify.hpp"
#include "cli_list_flags.hpp"
#include "cli_engine_info.hpp"
#include "cli_load_settings.hpp"
#include "cli_repetitions.hpp"
#include "cli_assemble.hpp"
#include "cli_layer_ranges.hpp"
#include "cli_nozzle_diameter.hpp"
#include "cli_filament_map.hpp"
#include "cli_nozzle_options.hpp"
#include "cli_project_load.hpp"
#include "cli_pipe.hpp"
#include "cli_file_info.hpp"
#include "cli_plate_plan.hpp"
#include "cli_project_values.hpp"
#include "cli_named_presets.hpp"
#include "cli_plate_arrange.hpp"
#include "cli_plate_report.hpp"
#include "cli_run_cleanup.hpp"
#include "cli_result_json.hpp"
#include "cli_run_context.hpp"
#include "cli_project_settings.hpp"
#include "cli_percent_widths.hpp"
#include "cli_preset_staging.hpp"
#include "cli_arrange_geometry.hpp"
#include "cli_setting_overrides.hpp"
#include "cli_list_settings.hpp"
#include "cli_printer_catalog.hpp"
#include "cli_project_archive.hpp"
#include "cli_filament_config.hpp"
#include "cli_json_config.hpp"
#include "cli_legacy_gcode.hpp"
#include "cli_engine_events.hpp"

#ifdef __APPLE__
#include <mach-o/dyld.h>   // _NSGetExecutablePath
#include <climits>         // PATH_MAX
#endif
using slicer_cli::CliRunContext;
using slicer_cli::PlateOutcome;
using slicer_cli::set_outcome_failure;
using slicer_cli::assemble_list_file;
using slicer_cli::cli_error_sentence;
using slicer_cli::named_input_files;
using slicer_cli::write_result_json;
using slicer_cli::emit_progress;
using slicer_cli::emit_run_progress;
using slicer_cli::RunBackupDetach;
using slicer_cli::sweep_stale_staging_folders;
using slicer_cli::run_backup_path;
using slicer_cli::set_run_backup_path;
using slicer_cli::create_archive_dirs;
using slicer_cli::record_plate_statistics;
using slicer_cli::record_plate_for_export;
using slicer_cli::post_slice_checks;
using slicer_cli::mm_text;
using slicer_cli::check_objects_inside_bed;
using slicer_cli::downward_plate_size;
using slicer_cli::move_to_new_bed;
using slicer_cli::assemble_tower_inside_plate;
using slicer_cli::tower_reach_box;
using slicer_cli::arrange_on_bed;
using slicer_cli::tower_default_corner;
using slicer_cli::assemble_tower_default_corner;
using slicer_cli::TowerWipeParams;
using slicer_cli::tower_wipe_params;
using slicer_cli::estimate_tower_polygon_global;
using slicer_cli::arrange_assemble_plate;
using slicer_cli::this_engine_profiles_dir;
using slicer_cli::apply_file_inherits;
using slicer_cli::config_id_text;
#ifdef ENGINE_ORCA
using slicer_cli::config_value_text;
#endif
using slicer_cli::project_presets_shipped;
using slicer_cli::load_system_presets;
using slicer_cli::stage_named_presets_for_project;
using slicer_cli::desktop_plate_type_for_printer;
using slicer_cli::resolve_named_presets;
using slicer_cli::run_list_presets;
#ifdef ENGINE_ORCA
using slicer_cli::BambuValueAction;
#endif
#ifdef ENGINE_ORCA
using slicer_cli::filament_keys_split_reads_empty;
#endif
#ifdef ENGINE_ORCA
using slicer_cli::bambu_value_actions;
#endif
using slicer_cli::json_scalar_text;
#ifdef ENGINE_ORCA
using slicer_cli::apply_bambu_actions;
#endif
using slicer_cli::ProjectPlan;
using slicer_cli::plate_grid_origin;
using slicer_cli::plate_scene_box;
using slicer_cli::trailing_models_port;
using slicer_cli::plan_project_plates;
using slicer_cli::emit_project_plan_event;
using slicer_cli::run_info;
using json = nlohmann::json;
using slicer_cli::project_settings_json;
#ifdef ENGINE_ORCA
using slicer_cli::RawValues;
using slicer_cli::FileObjectSettings;
using slicer_cli::project_object_settings;
using slicer_cli::maker_changed_key;
using slicer_cli::file_states_number;
using slicer_cli::file_states_text;
#endif

#ifdef ENGINE_BAMBU
using slicer_cli::PercentRewrite;
using slicer_cli::rewrite_percent_line_widths;
#endif

using slicer_cli::staging_owner_marker;
using slicer_cli::write_owner_marker;
using slicer_cli::remove_staging_folder;
using slicer_cli::make_owned_temp_dir;
using slicer_cli::remove_owned_temp_dir;
using slicer_cli::stage_file_copy;
using slicer_cli::stage_vendor_dir;
using slicer_cli::configure_engine_resources;

using slicer_cli::arrange_opt;
using slicer_cli::arrange_exclude_boxes;
using slicer_cli::arrange_add_exclude_areas;
using slicer_cli::arrange_plate_extruders;
using slicer_cli::arrange_estimate_wipe_tower_size;

using slicer_cli::apply_command_line_overrides;

using slicer_cli::run_list_settings;

using slicer_cli::EngineFit;
using slicer_cli::ShippedPrinterPreset;
using slicer_cli::engine_executable_dir;
using slicer_cli::engine_profiles_dir;
using slicer_cli::orca_profiles_dir;
using slicer_cli::preset_printer_model;
using slicer_cli::engine_fit_for;
using slicer_cli::engine_mismatch_sentence;
using slicer_cli::shipped_printer_presets;
using slicer_cli::shipped_printer_preset_of_variant;
using slicer_cli::preset_name_list;

using slicer_cli::FileVersionVerdict;
using slicer_cli::input_is_stl;
using slicer_cli::input_is_3mf;
using slicer_cli::read_zip_member;
using slicer_cli::zip_member_readable;
using slicer_cli::count_3mf_plates;
using slicer_cli::model_metadata;
using slicer_cli::engine_file_version;
using slicer_cli::engine_version_text;
using slicer_cli::file_version_verdict;

using slicer_cli::project_filament_count;
#ifdef ENGINE_BAMBU
using slicer_cli::apply_explicit_nozzle_mapping;
using slicer_cli::set_default_config;
using slicer_cli::ensure_vector_config_sizes;
using slicer_cli::is_filament_setting_key;
using slicer_cli::align_per_filament_maps;
using slicer_cli::align_per_filament_config_vectors;
#endif
using slicer_cli::normalize_filament_config_vectors;

using slicer_cli::json_file_has_key;
using slicer_cli::load_json_config;
#ifdef ENGINE_BAMBU
using slicer_cli::bbs_3mf_config_contains_nozzle_map;
#endif

using slicer_cli::normalize_legacy_gcode_tokens;
#ifdef ENGINE_ORCA
using slicer_cli::bambu_only_gcode_placeholder;
#endif

using slicer_cli::slicing_notification_tag;
using slicer_cli::warning_level_tag;
using slicer_cli::emit_event;
using slicer_cli::emit_config_substitutions;
using slicer_cli::emit_gcode_result_diagnostics;
using slicer_cli::install_engine_log_bridge;
using slicer_cli::emit_status_warning;
using slicer_cli::emit_validation_event;

// Read all of an fd into a string, polling the cancellation flag between reads
// so SIGINT/Ctrl+C during a slow stream (stdin, FIFO, or pipe) is honoured
// with a bounded exit. Returns false if cancelled.
// 0 = ok, 1 = cancelled, 2 = hard read error (not a cancel)
static int read_all_cancellable(int fd, std::string& out) {
    char buf[4096];
    while (true) {
        if (layout_plan::is_cancelled()) return 1;
#ifdef _WIN32
        // Windows: a redirected pipe can block in _read even after the CRT
        // handler runs. PeekNamedPipe before each read; when the pipe is empty,
        // wait a bounded interval and re-check the flag.
        HANDLE h = reinterpret_cast<HANDLE>(_get_osfhandle(fd));
        if (h != INVALID_HANDLE_VALUE && GetFileType(h) == FILE_TYPE_PIPE) {
            DWORD avail = 0;
            if (PeekNamedPipe(h, nullptr, 0, nullptr, &avail, nullptr) && avail == 0) {
                Sleep(50);
                continue;
            }
        }
        int n = ::_read(fd, buf, static_cast<unsigned>(sizeof buf));
#else
        ssize_t n = ::read(fd, buf, sizeof buf);
        if (n < 0 && errno == EINTR) continue;  // sigaction has no SA_RESTART
#endif
        if (n < 0) return 2;       // hard error (not a cancel)
        if (n == 0) return 0;      // EOF, ok
        out.append(buf, static_cast<size_t>(n));
    }
}

void print_usage(const char* prog_name) {
    slicer_cli::print_help(std::cout, prog_name);
}

// Parse a numeric CLI argument, failing fast with usage instead of letting
// std::stod/std::stoi throw an uncaught exception (which would SIGABRT). Used by
// the --calib-* flags so a typo like `--calib-start abc` exits 1 cleanly.
// ── Per-plate outcome ───────────────────────────────────────────────────
// The command line itself is CliOptions (cli_options.hpp), filled by
// cli_command_line.cpp.














#ifdef ENGINE_ORCA
#endif







/// One file a run writes, as check_run_outputs compares it (see there).
struct RunOutput {
    bool        folder = false;   // a folder the run writes into (--export-slicedata)
    bool        stl    = false;   // an object STL file of --export-stl / --export-stls
    std::string flag;             // "--export-3mf NAME"; empty for the run's own files
    boost::filesystem::path path;
    std::string what;             // "the run's own result.json in --outputdir"
    std::string is_what;          // what follows "is" in the same-name sentence
};

/// One plate's objects, loaded alone from the project 3MF, for
/// --downward-check: the official sizes every plate of the project
/// (check_plate_wipe_tower per plate, BambuStudio.cpp 4645-4658; OrcaSlicer.cpp
/// 3986-3996), whichever plate is sliced. Plate 0 is the whole project;
/// `plate_members`, when given, receives each plate's (object, instance)
/// pairs in the loaded model (PlateData::objects_and_instances), and
/// `file_config` the project's settings as the file holds them.
static bool load_plate_objects(CliRunContext& context, const std::string& input_file, int plate_id, Slic3r::Model& model,
                               std::vector<std::vector<std::pair<int, int>>>* plate_members = nullptr,
                               Slic3r::DynamicPrintConfig* file_config = nullptr) {
    using namespace Slic3r;
    DynamicPrintConfig scratch;
    scratch.apply(FullPrintConfig::defaults(), true);
    ConfigSubstitutionContext subst(ForwardCompatibilitySubstitutionRule::Enable);
    PlateDataPtrs plates;
    std::vector<Preset*> presets;
    bool is_bbl = false;
    Semver version;
    std::string load_path = input_file;
#ifdef ENGINE_BAMBU
    PercentRewrite percent_rewrite;
    rewrite_percent_line_widths(input_file, percent_rewrite);
    if (!percent_rewrite.refusal.empty())
        return false;
    if (percent_rewrite.changed)
        load_path = percent_rewrite.temp_path;
#endif
    const auto strategy = LoadStrategy::LoadModel | LoadStrategy::LoadConfig | LoadStrategy::AddDefaultInstances;
    // A writable backup folder of this run's own, as the slice's own load
    // sets (never the loader's default /bamboo_model, and never one shared
    // with another run: run_backup_path), with the archive's directories in it
    // so the extraction finds them, and detached again when this function
    // ends: this model is a temporary one, and its destruction must not remove
    // the folder the run's own load extracts into (detach_run_backup_path).
    set_run_backup_path(context, model);
    create_archive_dirs(load_path, run_backup_path(context));
    const RunBackupDetach detach_model{model};
    bool loaded = false;
    try {
#ifdef ENGINE_ORCA
#ifdef ENGINE_SNAPMAKER
        // Snapmaker's load_bbs_3mf takes no is_orca_3mf flag (Format/bbs_3mf.hpp
        // at bac6fd5), like BambuStudio's.
        loaded = load_bbs_3mf(load_path.c_str(), &scratch, &subst, &model, &plates, &presets, &is_bbl,
                              &version, nullptr, strategy, nullptr, plate_id);
#else
        bool is_orca = false;
        loaded = load_bbs_3mf(load_path.c_str(), &scratch, &subst, &model, &plates, &presets, &is_bbl, &is_orca,
                              &version, nullptr, strategy, nullptr, plate_id);
#endif
#else
        loaded = load_bbs_3mf(load_path.c_str(), &scratch, &subst, &model, &plates, &presets, &is_bbl,
                              &version, nullptr, strategy, nullptr, plate_id);
#endif
    } catch (const std::exception&) {
        loaded = false;
    }
    if (plate_members) {
        // The loader keeps a plate's instances only as obj_inst_map and
        // stamps each loaded instance with its identify_id (bbs_3mf.cpp
        // 2411-2445 at 5873b5f); an instance tied to no plate goes to the
        // first plate its box meets (PartPlateList::reload_all_objects ->
        // PartPlate::intersect_instance, PartPlate.cpp 5668-5710, 2651-2677),
        // as export_sliced_3mf finds them.
        plate_members->assign(plates.size(), {});
        std::set<std::pair<int, int>> placed;
        for (size_t k = 0; loaded && k < plates.size(); ++k) {
            if (plates[k] == nullptr) continue;
            std::set<int> identify_ids;
            for (const auto& entry : plates[k]->obj_inst_map)
                if (entry.second.second > 0) identify_ids.insert(entry.second.second);
            for (size_t oi = 0; oi < model.objects.size(); ++oi)
                for (size_t ii = 0; ii < model.objects[oi]->instances.size(); ++ii) {
                    const int id = model.objects[oi]->instances[ii]->loaded_id;
                    if (id > 0 && identify_ids.count(id) && placed.insert({int(oi), int(ii)}).second)
                        (*plate_members)[k].emplace_back(int(oi), int(ii));
                }
        }
        for (size_t oi = 0; loaded && oi < model.objects.size(); ++oi)
            for (size_t ii = 0; ii < model.objects[oi]->instances.size(); ++ii) {
                if (placed.count({int(oi), int(ii)})) continue;
                const BoundingBoxf3 box = model.objects[oi]->instance_convex_hull_bounding_box(ii);
                for (size_t k = 0; k < plates.size(); ++k)
                    if (plate_scene_box(scratch, version, int(k), int(plates.size())).intersects(box)) {
                        (*plate_members)[k].emplace_back(int(oi), int(ii));
                        break;
                    }
            }
    }
    if (file_config)
        *file_config = scratch;
    release_PlateData_list(plates);
    for (Preset* p : presets)
        delete p;
    return loaded;
}

/// The selected plate's own settings, as the official CLI lays them over the
/// project's per plate (see the call site): the plate's PlateData::config
/// without the filament-map keys (handled by the plate filament-map blocks
/// of each build: mode always, map in a manual mode) and without keys this
/// engine does not define. Empty
/// without a named plate (plate_id 0 is a whole-file load, not one plate).
static Slic3r::DynamicPrintConfig plate_own_settings(const Slic3r::PlateDataPtrs& plate_data, int plate_id) {
    Slic3r::DynamicPrintConfig plate_config;
    if (plate_id <= 0 || (int)plate_data.size() < plate_id || plate_data[plate_id - 1] == nullptr)
        return plate_config;
    plate_config = plate_data[plate_id - 1]->config;
    for (const char* key : {"filament_map_mode", "filament_map", "filament_volume_map"})
        plate_config.erase(key);
    for (const std::string& key : plate_config.keys())
        if (Slic3r::print_config_def.get(key) == nullptr)
            plate_config.erase(key);
    return plate_config;
}

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
    bool loaded = false;      // the inputs are in `model` and `base`
    bool prepared = false;    // the steps before the plate loop ran
    int prepared_plate = 0;   // the plate the run-level steps ran for (0: every plate)
    Slic3r::Model model;
    // m_print_config: the project's (or the profiles') settings with the
    // settings merge and the flags applied, before any plate's own.
    Slic3r::DynamicPrintConfig base;
    // The settings the steps before the plate loop read and changed (the
    // arrange's tower): `prepared_plate`'s own, or the project's for every plate.
    Slic3r::DynamicPrintConfig run_config;
    Slic3r::PlateDataPtrs plate_data;
#ifdef ENGINE_SNAPMAKER
    // The 3MF printer switch remaps the shared model's paint once per run: every
    // plate pass trims its own copy of the settings, but the model is the run's.
    bool switch_paint_remapped = false;
#endif
    ~RunState() { Slic3r::release_PlateData_list(plate_data); }
    std::vector<RunPlate> plates;   // index = plate id - 1
    ProjectPlan plan;
    // What the load and the run-level steps record for every plate's result.
    PlateOutcome carry;
    // The settings check's warnings, per plate (InvalidValuesNoted).
    std::map<int, json> noted;
    // --slice 0's check pass runs every plate before any is sliced: the
    // warnings it finds are announced only once every plate passed (the
    // warning promises a run that slices), so it holds them here.
    bool hold_noted = false;
    std::vector<json> noted_held;
    // Load-time facts every later step reads.
    bool is_bbl_3mf = false;
    Slic3r::Semver file_version;
    std::vector<Slic3r::Vec2d> plate_origins;   // the file's grid, for --downward-check
    std::optional<Slic3r::DynamicPrintConfig> plan_file_config;
    Slic3r::Semver plan_file_version;
    int plan_file_filaments = 0;
    slicer_cli::ProjectFacts project_facts;
    std::vector<Slic3r::Preset> project_presets_kept;
    bool project_loaded_like_desktop = false;   // the project was loaded over the system presets
    Slic3r::DynamicPrintConfig extra;
    bool explicit_config_supplied_nozzle_map = false;
    slicer_cli::SettingsMerge settings_merge;
    std::string merged_print_sequence;
    int shrink_to_new_bed = 0;
    size_t project_filaments = 0;
    // --pipe: steps 2 and 3 go out once per run, before any plate is
    // checked (BambuStudio.cpp 4923-4926 before the transforms, 6455-6458 as
    // the slice action starts, after pre_check is set at 6441; OrcaSlicer.cpp
    // 4185 and 5580, after 5565).
    bool pipe_files_loaded_sent = false;
    bool pipe_prepare_sent = false;
    // The model actions run once per run (BambuStudio.cpp 6336; OrcaSlicer.cpp 5470).
    bool actions_done = false;
    // The outputs were checked before the first write (check_run_outputs).
    bool outputs_checked = false;
    // The outputs the actions given before --slice wrote: the check after the
    // calibration pattern replaced the model reserves them (see
    // check_run_outputs' `reserved`).
    std::vector<RunOutput> reserved_outputs;
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

/// Events held while it lives (diagnostics::held_events): dropped when the
/// step they come from passes (drop()), sent when it stops early.
class HeldEventsScope {
public:
    explicit HeldEventsScope(bool active) : m_active(active) {
        if (!m_active) return;
        m_prev = slicer_cli::diagnostics::held_events;
        slicer_cli::diagnostics::held_events = &m_held;
    }
    void drop() { m_held.clear(); }
    ~HeldEventsScope() {
        if (!m_active) return;
        slicer_cli::diagnostics::held_events = m_prev;
        for (const std::string& line : m_held)
            slicer_cli::diagnostics::write_event(line);
    }
    HeldEventsScope(const HeldEventsScope&) = delete;
    HeldEventsScope& operator=(const HeldEventsScope&) = delete;
private:
    bool m_active = false;
    std::vector<std::string>* m_prev = nullptr;
    std::vector<std::string> m_held;
};

static void pipe_files_loaded(RunState& rs) {
    if (!slicer_cli::pipe_started() || rs.pipe_files_loaded_sent)
        return;
    rs.pipe_files_loaded_sent = true;
    slicer_cli::pipe_update(2, "Loading files finished");
}
static void pipe_prepare_slicing(RunState& rs) {
    pipe_files_loaded(rs);
    if (!slicer_cli::pipe_started() || rs.pipe_prepare_sent)
        return;
    rs.pipe_prepare_sent = true;
    slicer_cli::pipe_update(3, "Prepare slicing");
}

/// One plate of the run's model in its own frame, for the steps the official
/// CLI runs on one plate in the plate's coordinates (the move to a new bed,
/// BambuStudio.cpp 4487-4533; the one-plate arrange, 5730-6176, with
/// PartPlateList::preprocess_arrange_polygon taking the plate origin off and
/// postprocess_arrange_polygon putting it back, PartPlate.cpp 5831-5871,
/// 6107-6129; the bed check against the plate's shape, 6527): while the
/// scope lives the model holds only this plate's instances, each moved by
/// minus the plate's origin. On exit every instance goes back to the scene
/// with the move the step made (an instance the step did not move keeps its
/// exact offset), the other instances come back, and instances or objects
/// the step added (--repetitions) are kept, on the plate. Instances are known
/// by their ObjectID, which a copy of the model keeps (Model::assign_copy), so
/// a step that puts back a saved copy of the model finds its instances again.
/// Only a scope that holds the whole model may let its step replace the model's
/// objects: nothing this model reaches through a scope may replace them (the
/// repetitions search runs on a copy of the model for that reason,
/// cli_repetitions.cpp).
class PlateScope {
public:
    PlateScope(Slic3r::Model& model, const RunPlate& plate) : m_model(model), m_origin(plate.origin) {
        m_whole = plate.all;
        if (!m_whole) {
            m_whole = true;
            for (const Slic3r::ModelObject* object : model.objects)
                for (const Slic3r::ModelInstance* inst : object->instances)
                    m_whole = m_whole && plate.is_member(inst);
        }
        if (!m_whole) {
            m_all_objects = model.objects;
            std::vector<Slic3r::ModelObject*> kept;
            for (Slic3r::ModelObject* object : model.objects) {
                std::vector<Slic3r::ModelInstance*> mine;
                for (Slic3r::ModelInstance* inst : object->instances)
                    if (plate.is_member(inst))
                        mine.push_back(inst);
                if (mine.empty())
                    continue;
                m_instances[object] = object->instances;
                object->instances = mine;
                kept.push_back(object);
            }
            model.objects = kept;
        }
        for (Slic3r::ModelObject* object : model.objects)
            for (Slic3r::ModelInstance* inst : object->instances) {
                const Slic3r::Vec3d scene = inst->get_offset();
                const Slic3r::Vec3d local = scene - Slic3r::Vec3d(m_origin.x(), m_origin.y(), 0.);
                m_scene[inst->id().id] = scene;
                m_local[inst->id().id] = local;
                m_members.insert(inst);
                if (m_origin.x() != 0. || m_origin.y() != 0.)
                    inst->set_offset(local);
            }
    }
    ~PlateScope() {
        const Slic3r::Vec3d origin(m_origin.x(), m_origin.y(), 0.);
        for (Slic3r::ModelObject* object : m_model.objects) {
            for (Slic3r::ModelInstance* inst : object->instances) {
                const auto it = m_scene.find(inst->id().id);
                if (it == m_scene.end())
                    inst->set_offset(inst->get_offset() + origin);   // added in the scope
                else if (inst->get_offset() != m_local[it->first])
                    inst->set_offset(it->second + (inst->get_offset() - m_local[it->first]));
                else
                    inst->set_offset(it->second);
            }
            object->invalidate_bounding_box();
        }
        if (m_whole)
            return;
        // The other plates' instances back in their places, the plate's own
        // (and any the step added) where they were.
        for (auto& [object, all] : m_instances) {
            std::vector<Slic3r::ModelInstance*> now = object->instances;
            std::vector<Slic3r::ModelInstance*> restored;
            for (Slic3r::ModelInstance* inst : all)
                if (!m_members.count(inst) || std::find(now.begin(), now.end(), inst) != now.end())
                    restored.push_back(inst);
            for (Slic3r::ModelInstance* inst : now)
                if (std::find(all.begin(), all.end(), inst) == all.end())
                    restored.push_back(inst);
            object->instances = restored;
        }
        std::vector<Slic3r::ModelObject*> now = m_model.objects;
        std::vector<Slic3r::ModelObject*> restored = m_all_objects;
        for (Slic3r::ModelObject* object : now)
            if (std::find(m_all_objects.begin(), m_all_objects.end(), object) == m_all_objects.end())
                restored.push_back(object);
        m_model.objects = restored;
    }
    PlateScope(const PlateScope&) = delete;
    PlateScope& operator=(const PlateScope&) = delete;

private:
    Slic3r::Model& m_model;
    Slic3r::Vec2d m_origin;
    bool m_whole = true;
    std::vector<Slic3r::ModelObject*> m_all_objects;
    std::map<Slic3r::ModelObject*, std::vector<Slic3r::ModelInstance*>> m_instances;
    std::set<const Slic3r::ModelInstance*> m_members;
    std::map<size_t, Slic3r::Vec3d> m_scene, m_local;
};

/// --skip-objects on the placed objects (BambuStudio.cpp 6541-6581,
/// CLI_NO_SUITABLE_OBJECTS_AFTER_SKIP when nothing is left; OrcaSlicer.cpp
/// 5645-5720): inside the slice action, after the model actions. Its caller
/// runs --ensure-on-bed first, before the actions (BambuStudio.cpp 6188-6194
/// then the action loop at 6336; OrcaSlicer.cpp 5443-5455 then 5470).
/// --mtcpp follows the bed check (plate_triangle_limit).
static bool plate_object_steps(const CliOptions& o, Slic3r::Model& model, int plate, PlateOutcome& outcome) {
    std::vector<int> skipped;
    if (slicer_cli::apply_skip_objects(o, model, skipped) < 0) {
        const std::string why = "Every object on plate " + std::to_string(plate) + " is in --skip-objects.";
        std::cerr << "Error: " << why << "\n";
        set_outcome_failure(outcome, CLI_NO_SUITABLE_OBJECTS_AFTER_SKIP, why);
        return false;
    }
    outcome.skipped_objects = skipped;
    return true;
}

/// --mtcpp, once the bed check has marked each instance inside or not
/// (BambuStudio.cpp 6541-6597; OrcaSlicer.cpp 5657-5720).
static bool plate_triangle_limit(const CliOptions& o, const Slic3r::Model& model, int plate, PlateOutcome& outcome) {
    const slicer_cli::StepResult tri = slicer_cli::check_triangle_limit(o, model, plate);
    if (tri.code != 0) {
        std::cerr << "Error: " << tri.message << "\n";
        set_outcome_failure(outcome, tri.code, tri.message);
        return false;
    }
    return true;
}

/// The noted-value warning (InvalidValuesNoted) as the run announces it: the
/// event and its line on stderr. --slice 0's check pass holds them until every
/// plate has passed (RunState::noted_held), so the warning's "the run slices
/// with them" is said only of a run that slices.
static void announce_noted_warning(const json& announcement) {
    emit_event(announcement);
    std::cerr << "Warning: " << announcement.value("message", std::string()) << "\n";
}

/// One slice of one plate from the run's one model and settings (RunState):
/// the first call of a run loads the inputs and builds the settings base
/// (`rs.loaded`) and runs the steps the official CLI runs once before its
/// plate loop (`rs.prepared`: the transforms, the arrange, the actions before
/// --slice); every call then lays the plate's own settings over a copy of the
/// base and slices the plate with its origin, as the official plate loop does
/// (BambuStudio.cpp 6437-7260; OrcaSlicer.cpp 5561-6200). The default call
/// (`file [--plate N] -o out.gcode`) is one call; `--slice` makes a prepare
/// call (outcome.prepare_pass, which stops before the plate checks) and then
/// one call per plate. The return value is the process exit code of the
/// default call; `outcome` carries what `--slice` mode reports on top.
static int check_run_outputs(CliRunContext& context, const CliOptions& o, const std::vector<int>& plates, const Slic3r::Model* model,
                             std::string* detail, bool after_slice_actions_only = false,
                             const std::vector<RunOutput>* reserved = nullptr,
                             std::vector<RunOutput>* reserve = nullptr);

/// The settings check's warnings (InvalidValuesNoted) of the plate this
/// result is for, once: any the outcome already holds (a carry from another
/// plate's call) are replaced.
static void take_noted_warnings(PlateOutcome& outcome, const std::map<int, json>& noted, int plate_id) {
    json& warnings = outcome.warnings;
    warnings.erase(std::remove_if(warnings.begin(), warnings.end(),
                                  [](const json& w) { return w.value("tag", "") == "InvalidValuesNoted"; }),
                   warnings.end());
    const int plate = outcome.plate_id > 0 ? outcome.plate_id : (plate_id > 0 ? plate_id : 1);
    if (const auto it = noted.find(plate); it != noted.end())
        for (const json& w : it->second)
            warnings.push_back(w);
}


#ifndef ENGINE_SNAPMAKER
/// --filament-map-mode and --filament-map: the command line's mode and map win
/// over the plate's and the project's, as in the official CLIs: the mode from
/// the command line before the plate's (BambuStudio.cpp 7095-7099 at
/// v02.09.01.70; OrcaSlicer.cpp 5851-5854 at 8500fcd), the map set on the plate
/// (7208-7249; 5889-5929) and laid over the print config (7434; 5989). In
/// Manual each engine prints a filament with the extruder its map names (Bambu
/// ToolOrdering.cpp 2653-2680; Orca ToolOrdering.cpp 1288-1305, which regroups
/// only in an automatic mode). --filament-map with no mode means Manual: the
/// official CLI would keep the plate's mode and, in an automatic one, regroup
/// over the map. On the Bambu build a manual mode also runs the official
/// rack checks and fills (cli_nozzle_options.cpp), and on a printer with one
/// extruder and a nozzle rack the map is Bambu Studio's dialog grouping.
/// Snapmaker Orca's plate carries no map at bac6fd5, so that build refuses
/// both flags (cli_flag_support.cpp).
/// ModeOnly: the mode is set and the plate's own map still applies.
enum class FilamentMapFlag { NotGiven, Applied, ModeOnly, Refused };
static FilamentMapFlag apply_filament_map_flag(const CliOptions& o, Slic3r::DynamicPrintConfig& extra,
                                               Slic3r::DynamicPrintConfig& config, RunState& rs,
                                               std::optional<Slic3r::DynamicPrintConfig>& project_settings,
                                               size_t project_filaments, int plate_id, PlateOutcome& outcome) {
    // The checks and the maps are cli_filament_map.cpp's; this applies what
    // they decided to the run's own records.
    const slicer_cli::FilamentMapFlagStep step =
        slicer_cli::filament_map_flag_step(extra, config, project_filaments, !(o.slice_mode && o.slice_plate == 0));
    using Kind = slicer_cli::FilamentMapFlagStep::Kind;
    // The exported plate states the volume and nozzle maps the flags gave (or
    // the step filled), not the ones the plate was saved with.
    if (step.kind != Kind::Refused && o.slice_mode && !o.export_3mf.empty())
        for (const char* key : {"filament_volume_map", "filament_nozzle_map"})
            if (const auto* map = extra.option<Slic3r::ConfigOptionInts>(key))
                outcome.flag_plate_maps[key] = map->values;
    if (step.kind == Kind::NotGiven)
        return FilamentMapFlag::NotGiven;
    if (step.kind == Kind::Refused) {
        std::cerr << "Error: " << step.refusal << "\n";
        set_outcome_failure(outcome, CLI_INVALID_PARAMS, step.refusal);
        return FilamentMapFlag::Refused;
    }
    // The project's settings state the mode with the map they already carry:
    // --export-settings writes them (BambuStudio.cpp 6366-6370; OrcaSlicer.cpp
    // 5499-5503), and a map saved with an automatic mode is regrouped on the
    // next slice.
    Slic3r::ConfigSubstitutionContext subst(Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
    // With the mode go the maps the slice runs with: the map as the step
    // normalised it (a nozzle rack's grouping is extruder 1 for every
    // filament) and the nozzle and volume maps it filled.
    const auto state_maps = [&](Slic3r::DynamicPrintConfig& settings) {
        settings.set_deserialize("filament_map_mode", step.mode_name, subst);
        if (step.kind != Kind::Applied || !step.manual)
            return;
        settings.option<Slic3r::ConfigOptionInts>("filament_map", true)->values = step.map;
        for (const char* key : {"filament_volume_map", "filament_nozzle_map"})
            if (const Slic3r::ConfigOption* map = extra.option(key))
                settings.set_key_value(key, map->clone());
    };
    state_maps(rs.base);
    if (project_settings)
        state_maps(*project_settings);
    // The exported project's plate states the mode, and in a manual mode the
    // map, the slice used (store_bbs_3mf writes a plate's mode and map,
    // bbs_3mf.cpp).
    if (o.slice_mode && !o.export_3mf.empty()) {
        outcome.derived_filament_map_mode = step.mode_name;
        if (step.kind == Kind::Applied && step.manual)
            outcome.derived_filament_map = step.map;
        else
            outcome.derived_filament_map.clear();
    }
    if (step.kind == Kind::ModeOnly) {
        emit_event({{"event","config_normalized"}, {"tag","FilamentMapModeFromCommandLine"},
                    {"plate_id", plate_id}, {"filament_map_mode", step.mode_name}, {"message", step.message}});
        return FilamentMapFlag::ModeOnly;
    }
    emit_event({{"event","config_normalized"}, {"tag","FilamentMapFromCommandLine"},
                {"plate_id", plate_id}, {"filament_map", step.map}, {"filament_map_mode", step.mode_name},
                {"message", step.message}});
    return FilamentMapFlag::Applied;
}
#endif // ENGINE_SNAPMAKER

#ifdef ENGINE_BAMBU
/// The official check of a manual map against filament_printable
/// (cli_nozzle_options.cpp), on the objects the model holds now: the caller
/// scopes it to the plate and runs it after the object exclusions.
static bool manual_map_extruders_fit(const Slic3r::DynamicPrintConfig& config, const Slic3r::Model& model, int plate_id,
                                     PlateOutcome& outcome) {
    const std::string why =
        slicer_cli::manual_map_extruder_refusal(config, arrange_plate_extruders(model, config, std::max(0, plate_id - 1)));
    if (why.empty())
        return true;
    std::cerr << "Error: " << why << "\n";
    set_outcome_failure(outcome, CLI_FILAMENTS_NOT_SUPPORTED_BY_EXTRUDER, why);
    return false;
}
#endif

static int slice_one_plate(CliRunContext& context, const CliOptions& o, Slic3r::Calib_Params& calib_params, RunState& rs,
                           int plate_id, const std::string& output_file, PlateOutcome& outcome) {
    const std::string& input_file      = o.input_file;
    const std::string& machine_config  = o.machine_config;
    const std::string& filament_config = o.filament_config;
    const std::string& process_config  = o.process_config;
    const std::string& bundle_config   = o.bundle_config;
    const bool verbose                 = o.verbose;
    const bool normalize_legacy_gcode  = o.normalize_legacy_gcode;
    const auto& overrides              = o.overrides;
#ifdef ENGINE_SNAPMAKER
    // The engine writes debug SVGs into data_dir()/"SVG" on some slicing paths
    // (references/SnapmakerOrca/src/libslic3r/utils.cpp:302-320 at bac6fd5, where
    // create_directory throws if the parent is gone). The data directory can be
    // the named-preset staging folder, which is removed once staging ends, so a
    // run that switched printer stopped there. Give such a run a data directory
    // that exists for as long as the slice does.
    slicer_cli::snapmaker_keep_data_dir(run_backup_path(context));
#endif
    const bool calib_enabled       = calib_params.mode != Slic3r::CalibMode::Calib_None;
    const bool calib_self_geometry = slicer_cli::calib_mode_generates_geometry(calib_params.mode);
    const auto plate_started = std::chrono::steady_clock::now();
    // The first call of the run loads and prepares; the others slice only.
    const bool first_call = !rs.prepared;
    if (!first_call) {
        // What the load and the run-level steps recorded, for this plate's result.
        PlateOutcome carried = rs.carry;
        carried.plate_id    = outcome.plate_id;
        carried.plate_index = outcome.plate_index;
        carried.plate_count = outcome.plate_count;
        carried.gcode_path  = outcome.gcode_path;
        carried.pre_check   = outcome.pre_check;
        if (plate_id != rs.prepared_plate) {
            // Another plate than the one the run-level steps ran for: its own
            // move to the new bed and none of that plate's arrange results.
            carried.derived_filament_map.clear();
            carried.derived_filament_map_mode.clear();
            carried.flag_plate_maps.clear();
        }
        outcome = carried;
        // The carry is the prepare call's outcome, with that plate's settings
        // warnings: this plate gets its own instead.
        take_noted_warnings(outcome, rs.noted, plate_id);
    }
    try {
        // Create configuration BEFORE model loading so load_bbs_3mf can populate it
        // from the embedded Metadata/project_settings.config JSON.
        Slic3r::DynamicPrintConfig config;
        Slic3r::Model& model = rs.model;
        bool& is_bbl_3mf = rs.is_bbl_3mf;
        // Values this engine has no word for (unknown enum values), reported
        // together with the out-of-range values at the value check below.
        std::vector<std::string> unknown_values;
        json unknown_items = json::array();
        Slic3r::PlateDataPtrs& plate_data = rs.plate_data;
        // Each plate's grid origin in the project (plate_grid_origin), for
        // the plates --downward-check loads alone; empty when not a project.
        std::vector<Slic3r::Vec2d>& plate_origins = rs.plate_origins;
        // A project 3MF with model files after it (ProjectPlan): the file's own
        // settings and version, for the plate list's size, and its filament
        // count, the used filaments of the official's tower step.
        std::optional<Slic3r::DynamicPrintConfig>& plan_file_config = rs.plan_file_config;
        Slic3r::Semver& plan_file_version = rs.plan_file_version;
        int& plan_file_filaments = rs.plan_file_filaments;
        // What the official CLI reads from a Bambu-made 3MF, and its embedded
        // presets, for --load-settings and friends (cli_load_settings.cpp).
        slicer_cli::ProjectFacts& project_facts = rs.project_facts;
        std::vector<Slic3r::Preset>& project_presets_kept = rs.project_presets_kept;
        // The settings given as flags; the settings merge takes filament_colour
        // out of them once it has used it, as the official does.
        Slic3r::DynamicPrintConfig& extra = rs.extra;
#ifdef ENGINE_BAMBU
        bool& explicit_config_supplied_nozzle_map = rs.explicit_config_supplied_nozzle_map;
#endif

        // Geometry files (every model file this engine reads that is not a
        // project 3MF) load the official CLI's way, placed further down once
        // the bed is known. Any other file keeps the unsupported-format
        // refusal below.
        const bool geometry_input = !input_file.empty() && slicer_cli::is_loadable_model_file(input_file) &&
            (!input_is_3mf(input_file) ||
             slicer_cli::classify_3mf(input_file) == slicer_cli::ThreeMfKind::GeometryOnly);
        const bool geometry_only_3mf = geometry_input && input_is_3mf(input_file);
        // A project 3MF with model files after it, run as the official CLI
        // runs it (ProjectPlan): the run loads every plate (plate_id 0) and
        // the model files into the one model and arranges them; each plate's
        // slice then takes its instances from the plan.
        const bool trailing_port = !geometry_input && context.g_assemble == nullptr && trailing_models_port(o);
        // The plate filter of the one load (Model::read_from_file(...,
        // plate_to_slice), BambuStudio.cpp 1889; OrcaSlicer.cpp 1571): the
        // plate the run slices, or 0 for every plate.
        const int load_plate_id = plate_id;
        // --load-assemble-list: the plates' objects from the list built
        // before the plate loop, and every plate's own settings (plate_params)
        // as its PlateData config, where the official keeps them
        // (BambuStudio.cpp 1066-1078; OrcaSlicer.cpp 877-889).
        const bool assemble_input = context.g_assemble != nullptr;
        // The settings base the plate's own are laid over (below), and the
        // run-level part of the settings and of the model: once per run.
        std::optional<Slic3r::DynamicPrintConfig> project_settings;
        int& shrink_to_new_bed = rs.shrink_to_new_bed;
        slicer_cli::SettingsMerge& settings_merge = rs.settings_merge;
        std::string& merged_print_sequence = rs.merged_print_sequence;
#ifdef ENGINE_BAMBU
        size_t& project_filaments = rs.project_filaments;
#endif
        if (!rs.loaded) {
        std::cout << "\nConfiguring print settings...\n";
#ifdef ENGINE_BAMBU
        // Start with BambuStudio's full defaults (ensures all keys exist with correct
        // types) PLUS the BBS-specific extruder-variant normalization that coaxes the
        // Bambu engine into accepting a non-Bambu (e.g. Snapmaker U1) printer config.
        set_default_config(config);
#else
        // OrcaSlicer handles non-Bambu printers (U1/Prusa/Voron/…) natively, so none of
        // the BBS variant-array normalization is needed.  Just seed every key with the
        // engine's defaults; load_bbs_3mf (next) overlays the 3MF's project_settings.config.
        config.apply(Slic3r::FullPrintConfig::defaults(), true);
#endif
        extra = o.extra_config;

        // Load model
        std::cout << "Loading model: " << input_file << "\n";
        // Pre-set backup_path to a writable temp dir of this run's own so the
        // backup manager never touches the read-only /bamboo_model network
        // path, and parallel runs never share the loader's _temp_N.config
        // files (run_backup_path), with the input's own directories in it (the
        // OrcaSlicer loader does not create them: create_archive_dirs).
        set_run_backup_path(context, model);
        create_archive_dirs(input_file, run_backup_path(context));
        // The run's model is detached when this block ends, so no copy of it
        // (each plate's own, or any temporary) can take the folder down while
        // a later load still writes into it (detach_run_backup_path).
        const RunBackupDetach detach_run_model{model};
        if (assemble_input) {
            // The list's plates the run slices: plate N, or every plate. Each
            // plate's objects go to its place in the plate grid once the bed
            // is known (below).
            if (plate_id > (int)context.g_assemble->plates.size() || plate_id < 0) {
                set_outcome_failure(outcome, CLI_INVALID_PARAMS, "The assemble list has no plate " + std::to_string(plate_id) + ".");
                return 1;
            }
            rs.plates.assign(context.g_assemble->plates.size(), RunPlate());
            for (size_t k = 0; k < context.g_assemble->plates.size(); ++k) {
                rs.plates[k].all = false;
                if (plate_id > 0 && int(k) != plate_id - 1)
                    continue;
                const size_t before = model.objects.size();
                for (const Slic3r::ModelObject* object : context.g_assemble->plates[k].loaded_obj_list)
                    model.add_object(*object);
                for (size_t oi = before; oi < model.objects.size(); ++oi)
                    for (Slic3r::ModelInstance* inst : model.objects[oi]->instances)
                        rs.plates[k].members.push_back(inst);
                emit_event({{"event","model_loaded"}, {"tag","AssemblePlateBuilt"}, {"plate_id", int(k) + 1},
                            {"objects", model.objects.size() - before},
                            {"message","Plate " + std::to_string(k + 1) + " of the assemble list holds " +
                                       std::to_string(model.objects.size() - before) + " object(s)"}});
            }
            for (size_t k = 0; k < context.g_assemble->plates.size(); ++k) {
                auto* pd = new Slic3r::PlateData();
                pd->plate_index = int(k);
                pd->plate_name  = context.g_assemble->plates[k].plate_name;
                pd->config      = context.g_assemble->plate_configs[k];
                plate_data.push_back(pd);
            }
        } else if (geometry_input) {
            const slicer_cli::ModelLoadResult loaded = slicer_cli::load_geometry_files(o, o.input_files, 0, model);
            if (loaded.code != 0 && o.input_files.size() == 1 && input_is_stl(input_file) &&
                (loaded.code == CLI_DATA_FILE_ERROR || loaded.code == CLI_FILE_NOTFOUND)) {
                // One STL that cannot be read: the refusal slicer-cli has
                // always given for it.
                std::cerr << "Failed to load STL file\n";
                set_outcome_failure(outcome, CLI_DATA_FILE_ERROR);
                return 1;
            }
            slicer_cli::add_failure_reason(outcome.error_reason, loaded.failure_reason);
            if (loaded.code != 0) {
                emit_event({{"event","load_error"}, {"tag","ModelLoadFailed"}, {"path", input_file},
                            {"message", loaded.message}});
                std::cerr << "Error: " << loaded.message << "\n";
                set_outcome_failure(outcome, loaded.code, loaded.message);
                return 1;
            }
        } else if (input_is_3mf(input_file)) {
#ifdef ENGINE_BAMBU
            Slic3r::ConfigOptionInts accepted_3mf_nozzle_map(
                config.option<Slic3r::ConfigOptionInts>("filament_nozzle_map", true)->values);
            // The file's own filament_nozzle_map is what an earlier slice grouped
            // (written back from Print::get_filament_nozzle_maps, BambuStudio.cpp
            // 7107-7108), not a placement anyone asked for: the official CLI
            // reads a nozzle map only from its command line, and only in a
            // manual mode (BambuStudio.cpp 6789, 6836-6838). So it is kept as
            // the file's value but never makes the mapping explicit; only a
            // --config/--machine/--process/--filament overlay can.
            bbs_3mf_config_contains_nozzle_map(input_file, accepted_3mf_nozzle_map);
#endif
            // The path the engine's 3MF loader reads: the input itself, or on
            // the Bambu build a converted copy when the file states line widths
            // as percentages (see rewrite_percent_line_widths).
            std::string load_path = input_file;
#ifdef ENGINE_BAMBU
            PercentRewrite percent_rewrite;
            rewrite_percent_line_widths(input_file, percent_rewrite);
            if (!percent_rewrite.refusal.empty()) {
                emit_event({{"event","config_refused"},
                            {"tag","PercentLineWidthUnusable"},
                            {"message", percent_rewrite.refusal}});
                std::cerr << "Error: " << percent_rewrite.refusal << "\n";
                set_outcome_failure(outcome, CLI_INVALID_VALUES_IN_3MF, percent_rewrite.refusal);
                return 1;
            }
            if (percent_rewrite.changed) {
                load_path = percent_rewrite.temp_path;
                emit_event({{"event","config_normalized"},
                            {"tag","PercentLineWidthConverted"},
                            {"nozzle_diameter", percent_rewrite.nozzle_mm},
                            {"converted", percent_rewrite.converted},
                            {"message","Converted " + std::to_string(percent_rewrite.converted.size()) +
                                       " line width(s) stated as a percentage of the " +
                                       mm_text(percent_rewrite.nozzle_mm) + " mm nozzle into mm"}});
            }
#endif
            Slic3r::ConfigSubstitutionContext config_subst(Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
            std::vector<Slic3r::Preset*> presets;
            struct PresetsRelease {
                std::vector<Slic3r::Preset*>& presets;
                ~PresetsRelease() { for (Slic3r::Preset* p : presets) delete p; presets.clear(); }
            } presets_release{presets};
            Slic3r::Semver file_version;

            // Pass &config so load_bbs_3mf extracts project_settings.config from the
            // 3MF archive and applies it via config.load_from_json().  Previously this
            // was nullptr which caused load_bbs_3mf to return false immediately
            // (bbs_3mf.cpp:8966 checks config == nullptr).
            //
            // LoadStrategy must include LoadModel (parse geometry) and LoadConfig
            // (parse Metadata/project_settings.config).  AddDefaultInstances ensures
            // each object gets at least one instance.
            auto strategy = Slic3r::LoadStrategy::LoadModel
                          | Slic3r::LoadStrategy::LoadConfig
                          | Slic3r::LoadStrategy::AddDefaultInstances;
#ifdef ENGINE_ORCA
#ifdef ENGINE_SNAPMAKER
            // Snapmaker's load_bbs_3mf has no is_orca_3mf flag (Format/bbs_3mf.hpp
            // at bac6fd5), so this uses the BambuStudio-shaped call.
            bool result = Slic3r::load_bbs_3mf(
                load_path.c_str(),
                &config,
                &config_subst,
                &model,
                &plate_data,
                &presets,
                &is_bbl_3mf,
                &file_version,
                nullptr,   // proFn (progress callback)
                strategy,
                nullptr,   // BBLProject
                load_plate_id   // 0 = all plates, >0 = specific plate
            );
#else
            // OrcaSlicer's load_bbs_3mf inserts a bool* is_orca_3mf between is_bbl_3mf
            // and file_version (and drops the two trailing Bambu-only params).
            bool is_orca_3mf = false;
            bool result = Slic3r::load_bbs_3mf(
                load_path.c_str(),
                &config,
                &config_subst,
                &model,
                &plate_data,
                &presets,
                &is_bbl_3mf,
                &is_orca_3mf,
                &file_version,
                nullptr,   // proFn (progress callback)
                strategy,
                nullptr,   // BBLProject
                load_plate_id   // 0 = all plates, >0 = specific plate
            );
#endif // ENGINE_SNAPMAKER
#else
            bool result = Slic3r::load_bbs_3mf(
                load_path.c_str(),
                &config,
                &config_subst,
                &model,
                &plate_data,
                &presets,
                &is_bbl_3mf,
                &file_version,
                nullptr,   // proFn (progress callback)
                strategy,
                nullptr,   // BBLProject
                load_plate_id   // 0 = all plates, >0 = specific plate
            );
#endif
            // Surface what the engine silently changed or ignored while
            // deserializing Metadata/project_settings.config. This is the
            // channel the GUI renders as "incompatible settings were
            // substituted", and it is also where an unknown key (a setting the
            // host wrote that this engine has no definition for) is recorded.
#ifdef ENGINE_SNAPMAKER
            slicer_cli::convert_flow_type_spelling(config, config_subst, "3mf:project_settings.config");
#endif
            emit_config_substitutions(config_subst, "3mf:project_settings.config");
            outcome.unknown_settings = config_subst.unrecogized_keys;   // sic — upstream spelling
#ifdef ENGINE_SNAPMAKER
            if (result)
                slicer_cli::warn_newer_project_schema(config);
#endif

            if (!result) {
                emit_event({{"event","load_error"},
                            {"tag","ThreeMfLoadFailed"},
                            {"path", input_file},
                            {"message","Failed to load 3MF file"}});
                std::cerr << "Failed to load 3MF file\n";
                set_outcome_failure(outcome, CLI_DATA_FILE_ERROR);
                return 1;
            }
            // --normative-check, on by default in the official CLI for a
            // Bambu-made 3MF (BambuStudio.cpp 1948-1965; OrcaSlicer.cpp
            // 1609-1619). Part of --slice here, or asked for by name, so the
            // default call slices the same files it always has.
            if (is_bbl_3mf && (o.slice_mode || o.given_flag("normative_check"))) {
                const slicer_cli::StepResult nc = slicer_cli::normative_check(o, config);
                if (nc.code != 0) {
                    emit_event({{"event","config_refused"}, {"tag","NormativeCheckFailed"}, {"message", nc.message}});
                    std::cerr << "Error: " << nc.message << "\n";
                    set_outcome_failure(outcome, nc.code, nc.message);
                    return 1;
                }
            }
            // The settings exactly as the file states them, and whether they
            // were kept: the rebase below (Orca build) replaces `config` with
            // the preset-rebased settings, and the steps that describe the FILE
            // read these instead (read_project_facts).
            Slic3r::DynamicPrintConfig project_file_config;
            bool                       project_file_config_kept = false;
#ifdef ENGINE_ORCA
            // ── A Bambu Studio project is rebased onto its system presets ────
            // The desktop does not slice a project's settings as the file
            // states them. It loads this engine's system presets
            // (PresetBundle::load_presets, Plater.cpp 6040-6160 at 31f6803),
            // hands the file's settings to PresetBundle::load_config_model
            // (PresetBundle.hpp 388 -> PresetBundle.cpp 4239-4567), which loads
            // the project's printer, process and filament presets over the ones
            // this engine ships and, for every key the file does not list in
            // different_settings_to_system, takes the system preset's value
            // (Preset.cpp 2446-2457 load_external_preset ->
            // update_non_diff_values_to_base_config), and slices
            // PresetBundle::full_config(false) (Plater.cpp 7959-7963). Every
            // Bambu-only encoding of a key the maker did not touch -- the
            // 0-based filament indices, tree_support_wall_count -1,
            // raft_first_layer_expansion -1, ensure_vertical_shell_thickness
            // "enabled" -- therefore never reaches the engine. Upstream main's
            // CLI rebases the same way for its --uptodate path (OrcaSlicer
            // 94266c28, OrcaSlicer.cpp 3075-3108 and 2697-2726 on main).
            // wipe_tower_x/y are the plate's own positions and are put back
            // after the load (Plater.cpp 6380-6389).
            // This scope test is the one place the scope is decided.
#ifdef ENGINE_SNAPMAKER
            // Snapmaker's loader reports no "made by OrcaSlicer" flag (Format/bbs_3mf.hpp
            // at bac6fd5), so the maker is read from the file's own tags: a Snapmaker Orca
            // project (Application "Snapmaker_Orca-<version>", Format/bbs_3mf.cpp:6645-6647)
            // or an OrcaSlicer one. The maker decides only the Bambu Studio walls
            // warning below; the rebase covers every project.
            bool is_orca_3mf = false;
            {
                std::string model_xml;
                if (read_zip_member(input_file, "3D/3dmodel.model", model_xml))
                    is_orca_3mf = slicer_cli::snapmaker_engine_made(model_metadata(model_xml));
            }
#endif
            // Every project one of the desktops made is rebased, this engine's
            // own too: the desktop's load has no test on who made the file
            // (OrcaSlicer Plater.cpp 6063-6110 lets a From_Orca project load
            // its config, 6415 load_config_model, Preset.cpp 2516-2535;
            // Snapmaker_Orca Plater.cpp 12103, Preset.cpp 2121-2148).
            const bool bambu_made_project = is_bbl_3mf;
#if defined(ENGINE_SNAPMAKER) || defined(ENGINE_ORCA)
            context.g_file_made_by_bambustudio = is_bbl_3mf && !is_orca_3mf;
#endif
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
            // The loader's own is_orca_3mf flag does not recognise every
            // OrcaSlicer-made file (it is false for an "OrcaSlicer-0.1.0-dev"
            // project), so the walls warning reads the maker from the file's own
            // tags, the way --engine-info reads it. The rebase scope above stays
            // the loader's flag: it is the OrcaSlicer build's behaviour on main,
            // which this change leaves as it is.
            {
                std::string maker_xml;
                if (read_zip_member(input_file, "3D/3dmodel.model", maker_xml)) {
                    const auto maker_meta = model_metadata(maker_xml);
                    const auto maker_app  = maker_meta.find("Application");
                    const bool orca_made  = maker_meta.count("OrcaSlicer") != 0 ||
                                            (maker_app != maker_meta.end() &&
                                             boost::starts_with(maker_app->second, "OrcaSlicer-"));
                    context.g_file_made_by_bambustudio = is_bbl_3mf && !orca_made;
                }
            }
#endif
#else
            // The BambuStudio build loads every Bambu Studio project the same
            // way: Bambu Studio's own Plater does (load_config_model,
            // Plater.cpp 12155 at v02.09.01.70 -> load_config_file_config,
            // PresetBundle.cpp 5265 -> load_external_preset, Preset.cpp:
            // update_non_diff_values_to_base_config for every key the file
            // does not list in different_settings_to_system). A project saved
            // by an older version then slices with today's start G-code, and a
            // setting the maker never changed takes today's preset value, as
            // it does on the desktop. --project-settings-as-saved keeps the
            // file's own values, which is what the official command line does.
            const bool bambu_made_project = is_bbl_3mf;
#endif
            if (bambu_made_project && !o.project_settings_as_saved) {
                // The settings as the file states them, for read_project_facts
                // below: the facts describe the file (its printer, bed, plate
                // membership and the maker's own changed-key list), not the
                // preset-rebased settings this run slices with.
                project_file_config      = config;
                project_file_config_kept = true;
                Slic3r::DynamicPrintConfig rebased;
                std::string                unavailable;
                bool                       rebase = true;
                try {
                    // The profiles tree staged as <data_dir>/system, the same
                    // staging the named-preset path uses (load_system_presets):
                    // the bundle reads system presets from there and writes
                    // nothing back (load_external_preset saves no file).
                    const boost::filesystem::path staging = make_owned_temp_dir("slicer_cli_project-");
                    struct StagingCleanup {
                        boost::filesystem::path dir;
                        ~StagingCleanup() { remove_owned_temp_dir(dir); }
                    } staging_cleanup;
                    staging_cleanup.dir = staging;
                    Slic3r::PresetBundle bundle;
                    if (!load_system_presets(o.argv0, bundle, staging, unavailable))
                        rebase = false;
                    else {
                        // The names the FILE states, not the ones the config
                        // carries: this engine's own 3MF loader substitutes a
                        // preset name it does not know with its default before
                        // the front end sees the config, and a project whose
                        // preset is not shipped is the one the rebase must skip
                        // (loading it over the default is a different print, and
                        // a project whose process preset the bundle cannot
                        // resolve is loaded as an external preset that
                        // Print::apply then aborts on).
                        const json  file_settings  = project_settings_json(input_file);
                        std::string file_printer   = json_scalar_text(file_settings, "printer_settings_id");
                        std::string file_process   = json_scalar_text(file_settings, "print_settings_id");
                        if (file_printer.empty())
                            file_printer = config_id_text(config, "printer_settings_id");
                        if (file_process.empty())
                            file_process = config_id_text(config, "print_settings_id");
#ifdef ENGINE_BAMBU
                        // The presets saved inside the project and the file's
                        // own nozzle rack, as the desktop keeps them
                        // (desktop_presets.cpp).
                        const Slic3r::DynamicPrintConfig file_nozzle_keys =
                            slicer_cli::load_project_embedded_presets(bundle, presets, config);
#endif
                        if (!project_presets_shipped(bundle, file_printer, file_process)) {
                            unavailable = "this engine ships no '" + file_printer + "' / '" + file_process +
                                          "' system preset";
                            rebase = false;
                        } else {
                        // The tower's plate positions, as the desktop keeps
                        // them across the load (Plater.cpp 6380-6389).
                        Slic3r::ConfigOptionFloats file_wipe_tower_x, file_wipe_tower_y;
                        bool                       has_wipe_tower_x = false, has_wipe_tower_y = false;
                        if (const auto* opt = config.option<Slic3r::ConfigOptionFloats>("wipe_tower_x")) {
                            file_wipe_tower_x = *opt;
                            has_wipe_tower_x  = true;
                        }
                        if (const auto* opt = config.option<Slic3r::ConfigOptionFloats>("wipe_tower_y")) {
                            file_wipe_tower_y = *opt;
                            has_wipe_tower_y  = true;
                        }
                        // load_config_model takes the config by value and
                        // consumes it: the bundle keeps what it parsed. Handed
                        // the caller's own config, an exception part-way (a
                        // preset this engine cannot resolve) would leave the
                        // flat path with a moved-from config, and Print::apply
                        // then reads settings that are no longer there. So the
                        // load gets a copy, and `config` is replaced only when
                        // the rebase itself succeeded.
                        Slic3r::DynamicPrintConfig file_config_for_bundle = config;
                        bundle.load_config_model(input_file, std::move(file_config_for_bundle), file_version);
#ifdef ENGINE_BAMBU
                        // The file's nozzles win over the selected printer's
                        // (NozzleInventory::open_project, Plater.cpp 12160).
                        bundle.nozzle_inventory.open_project(bundle.printers, file_nozzle_keys);
#endif
#ifdef ENGINE_SNAPMAKER
                        rebased = bundle.full_config();   // see above: full_fff_config at bac6fd5
#else
                        rebased = bundle.full_config(false);
#endif
#ifdef ENGINE_BAMBU
                        // extruder_ams_count: a newly started app takes the
                        // file's (PresetBundle.cpp 5362-5366: the bundle's own
                        // counts are empty then), and so does this run.
#endif
                        // A project whose preset name this bundle cannot
                        // resolve is loaded as an external preset built from the
                        // file, and the config that comes back can describe a
                        // different filament roster than the file does;
                        // Print::apply then aborts on the mismatch (it derives
                        // its extruder count from filament_diameter,
                        // PrintApply.cpp:1504). The rebase is kept only when it
                        // describes the roster the file states.
                        const size_t file_filaments    = project_filament_count(config);
                        const size_t rebased_filaments = project_filament_count(rebased);
                        if (file_filaments != 0 && rebased_filaments != file_filaments) {
                            unavailable = "the system presets the project names describe " +
                                          std::to_string(rebased_filaments) + " filament(s) where the file states " +
                                          std::to_string(file_filaments);
                            rebase = false;
                        }
                        // The same for the extruders: a file that states
                        // another number of extruders than the printer preset
                        // it names is not that preset's project, and its
                        // settings are sliced as the file states them.
                        const auto extruder_count = [](const Slic3r::DynamicPrintConfig& c) {
                            const auto* d = c.option<Slic3r::ConfigOptionFloats>("nozzle_diameter");
                            return d ? d->values.size() : size_t(0);
                        };
                        if (rebase && extruder_count(config) != 0 && extruder_count(rebased) != extruder_count(config)) {
                            unavailable = "the printer preset the project names has " +
                                          std::to_string(extruder_count(rebased)) + " extruder(s) where the file states " +
                                          std::to_string(extruder_count(config));
                            rebase = false;
                        }
                        if (has_wipe_tower_x)
                            rebased.set_key_value("wipe_tower_x", new Slic3r::ConfigOptionFloats(file_wipe_tower_x));
                        if (has_wipe_tower_y)
                            rebased.set_key_value("wipe_tower_y", new Slic3r::ConfigOptionFloats(file_wipe_tower_y));
                        }
                    }
                } catch (const std::exception& e) {
                    unavailable = e.what();
                    rebase      = false;
                    // A filament setting the preset split cannot read. The
                    // desktop calls the same load_config_model (Plater.cpp 6391
                    // at 31f6803) inside a catch that shows the error and skips
                    // the file (6639-6642), so the project is not widened here:
                    // the reason is named, and any refusal of the plate says
                    // which key it is.
#ifdef ENGINE_ORCA
                    const auto empty = filament_keys_split_reads_empty(config, project_settings_json(input_file));
                    if (!empty.empty()) {
                        std::string names;
                        bool        single = true;
                        for (const auto& key : empty) {
                            names += (names.empty() ? "" : ", ") + key.first;
                            single = single && key.second;
                        }
                        const bool  one  = empty.size() == 1;
                        const char* what = single ? (one ? " is a single value" : " are single values")
                                                  : (one ? " has no entries" : " have no entries");
                        unavailable += ": the file's " + names + what + " where OrcaSlicer expects one per filament";
                        outcome.refusal_note =
                            "This project's " + names + what +
                            " where OrcaSlicer expects one per filament, so its settings could not be matched to the "
                            "desktop presets; slice it with slicer_cli (BambuStudio), or re-save it with one " +
                            names + " entry per filament.";
                    }
#endif
                }
                if (rebase) {
                    outcome.settings_replaced_by_presets = slicer_cli::settings_replaced_by_presets(config, rebased);
                    // Only what the file states: a key it leaves out was
                    // never the file's value.
                    const json file_settings = project_settings_json(input_file);
                    for (const std::string& key : outcome.settings_replaced_by_presets)
                        if (file_settings.contains(key)) {
                            auto& values = outcome.replaced_file_values[key];
                            values.push_back(config.opt_serialize(key));
                            if (file_settings[key].is_string())
                                values.push_back(file_settings[key].get<std::string>());
                        }
                    config = std::move(rebased);
                    rs.project_loaded_like_desktop = true;
                    emit_event({{"event","config_normalized"},
                                {"tag","ProjectRebasedOnSystemPresets"},
                                {"printer_settings_id", config_id_text(config, "printer_settings_id")},
                                {"print_settings_id", config_id_text(config, "print_settings_id")},
                                {"replaced_settings", outcome.settings_replaced_by_presets},
                                {"message","The project was loaded over this engine's system presets, as the desktop loads it: "
                                           "every key the maker did not change takes the preset's value"}});
                } else {
                    // The engine ships no preset to load the project over, so
                    // the file's own settings are sliced, as before.
                    project_file_config.clear();
                    project_file_config_kept = false;
                    emit_event({{"event","config_normalized"},
                                {"tag","ProjectPresetRebaseSkipped"},
                                {"reason", unavailable},
                                {"message","The project keeps the values the file states: " + unavailable}});
                }
            }
#ifdef ENGINE_ORCA
            // ── The file's own words, in this engine's values ───────────────
            // The engine's own legacy pass (PrintConfigDef::handle_legacy,
            // PrintConfig.cpp 7867-8151 at 31f6803) runs per key while a config
            // loads: it cannot see another key's value, and it knows only some
            // of BambuStudio's words. The rest are converted here, on the
            // settings the run slices with, from the words the file states --
            // for the project and for every object's and part's own settings,
            // which the file keeps in Metadata/model_settings.config and which
            // bbs_3mf loads over the same legacy pass (bbs_3mf.cpp 2130, 5115,
            // 5263 at 31f6803). One set of rules (bambu_value_actions) decides
            // for all three scopes.
            {
                const json project_settings = project_settings_json(input_file);
                RawValues  project_raw;
                for (const char* key : {"tree_support_wall_count", "raft_first_layer_expansion", "top_one_wall_type",
                                        "ensure_vertical_shell_thickness"})
                    project_raw[key] = json_scalar_text(project_settings, key);
                std::vector<BambuValueAction> conversions;
                std::vector<BambuValueAction> warnings;
                std::string                   refusal;
                if (!bambu_value_actions(context, project_raw, config_value_text(config, "support_type"),
                                         maker_changed_key(project_file_config, "raft_first_layer_expansion"),
                                         conversions, warnings, refusal)) {
                    emit_event({{"event","config_refused"},
                                {"tag","BambuValueNotCarryable"},
                                {"opt_key", project_raw.count("raft_first_layer_expansion") ? "raft_first_layer_expansion"
                                                                                           : "ensure_vertical_shell_thickness"},
                                {"message", refusal}});
                    std::cerr << "Error: " << refusal << "\n";
                    set_outcome_failure(outcome, CLI_INVALID_VALUES_IN_3MF, refusal);
                    return 1;
                }
                // A word this pass converted is not a substitution: the engine's
                // record of not knowing it would make --slice mode refuse an
                // encoding difference. The loader keeps one substitution context
                // for the whole file, object and part settings included, so the
                // record names the key, not the scope it came from.
                const auto drop_substitution = [&](const char* key) {
                    config_subst.substitutions.erase(
                        std::remove_if(config_subst.substitutions.begin(), config_subst.substitutions.end(),
                                       [key](const Slic3r::ConfigSubstitution& s) {
                                           return s.opt_def != nullptr && s.opt_def->opt_key == key;
                                       }),
                        config_subst.substitutions.end());
                };
                apply_bambu_actions(
                    conversions, warnings,
                    [&](const char* key, const std::string& value) {
                        Slic3r::ConfigSubstitutionContext context(
                            Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
                        config.set_deserialize(key, value, context);
                    },
                    drop_substitution, "scope", std::string());

                // Each object's and each part's own settings, matched to the
                // loaded model by the names the file states (the file's numeric
                // ids are not kept on the loaded objects). An object's settings
                // are the maker's own words by definition, so the "the maker
                // changed this key" test the project needs does not apply here.
                const std::vector<FileObjectSettings>          file_objects = project_object_settings(input_file);
                std::map<std::string, std::vector<const FileObjectSettings*>> by_name;
                for (const FileObjectSettings& settings : file_objects)
                    by_name[settings.name].push_back(&settings);
                std::map<std::string, size_t> taken;
                for (Slic3r::ModelObject* object : model.objects) {
                    if (object == nullptr)
                        continue;
                    const auto& candidates = by_name[object->name];
                    const size_t index     = taken[object->name]++;
                    if (index >= candidates.size())
                        continue;
                    const FileObjectSettings& raw_object = *candidates[index];
                    const std::string project_support = config_value_text(config, "support_type");
                    const std::string object_support  = object->config.option("support_type") != nullptr
                                                            ? object->config.option("support_type")->serialize()
                                                            : project_support;
                    auto convert_scope = [&](const RawValues&                            raw,
                                             const std::string&                          support_type,
                                             const std::string&                          scope,
                                             const std::string&                          scope_name,
                                             const std::function<void(const char*, const std::string&)>& write) {
                        std::vector<BambuValueAction> scope_conversions;
                        std::vector<BambuValueAction> scope_warnings;
                        std::string                   scope_refusal;
                        if (!bambu_value_actions(context, raw, support_type, /*maker_set_raft=*/true, scope_conversions,
                                                 scope_warnings, scope_refusal)) {
                            const std::string sentence = scope_refusal + " (in " + scope + " \"" + scope_name + "\")";
                            emit_event({{"event","config_refused"},
                                        {"tag","BambuValueNotCarryable"},
                                        {scope, scope_name},
                                        {"message", sentence}});
                            std::cerr << "Error: " << sentence << "\n";
                            set_outcome_failure(outcome, CLI_INVALID_VALUES_IN_3MF, sentence);
                            return false;
                        }
                        apply_bambu_actions(scope_conversions, scope_warnings, write, drop_substitution, scope,
                                            scope_name);
                        return true;
                    };
                    const bool object_ok = convert_scope(
                        raw_object.values, object_support, "object", object->name,
                        [&](const char* key, const std::string& value) {
                            Slic3r::ConfigSubstitutionContext context(
                                Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
                            object->config.set_deserialize(key, value, context);
                        });
                    if (!object_ok)
                        return 1;
                    std::map<std::string, size_t> parts_taken;
                    for (Slic3r::ModelVolume* volume : object->volumes) {
                        if (volume == nullptr)
                            continue;
                        size_t part_index = 0;
                        bool   matched    = false;
                        for (const auto& part : raw_object.parts) {
                            if (part.first != volume->name)
                                continue;
                            if (part_index++ < parts_taken[volume->name])
                                continue;
                            parts_taken[volume->name]++;
                            const std::string part_support = volume->config.option("support_type") != nullptr
                                                                 ? volume->config.option("support_type")->serialize()
                                                                 : object_support;
                            if (!convert_scope(part.second, part_support, "part", volume->name,
                                               [&](const char* key, const std::string& value) {
                                                   Slic3r::ConfigSubstitutionContext context(
                                                       Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
                                                   volume->config.set_deserialize(key, value, context);
                                               }))
                                return 1;
                            matched = true;
                            break;
                        }
                        (void)matched;
                    }
                }
            }
            // OrcaSlicer v2.4.2 numbers every feature-filament index from 0,
            // where 0 is "Default" — the active object's or part's own
            // filament (outer_wall_filament_id, PrintConfig.cpp 5011-5018, min
            // 0, default 0) — and both apps' project files now carry those
            // keys by that name: handle_legacy renames the old 1-based
            // wall_filament / sparse_infill_filament / solid_infill_filament
            // to their *_id spelling and maps a legacy "1" to "0"
            // (PrintConfig.cpp 8038-8058). A Bambu Studio file's own 0 is
            // therefore already a value this engine reads, and nothing has to
            // be moved for it. Before the bump this block carried that 0 into
            // this engine's 1-based range; v2.4.2 makes that half redundant.
            //
            // What the desktop still does on load, and this run now does on
            // the settings it slices with, before the value check below: an
            // index outside [0, N] falls back to 0 for the six feature
            // filaments (PresetBundle.cpp 4127-4136), and support_filament /
            // support_interface_filament / wipe_tower_filament clamp to
            // [0, N] (PresetBundle.cpp 4119-4125), with N the filament count.
            // Any other out-of-range value is left as it is and still
            // refuses, exactly as the desktop leaves it (it does not clamp
            // them either, e.g. a BambuStudio -1, its "auto").
            {
                const size_t filament_count = std::max<size_t>(1, project_filament_count(config));
                json reset = json::object();
                json clamped = json::object();
                for (const char* key : {"outer_wall_filament_id", "inner_wall_filament_id",
                                        "sparse_infill_filament_id", "internal_solid_filament_id",
                                        "top_surface_filament_id", "bottom_surface_filament_id"}) {
                    auto* opt = dynamic_cast<Slic3r::ConfigOptionInt*>(config.option(key, false));
                    if (opt == nullptr || (opt->value >= 0 && opt->value <= int(filament_count)))
                        continue;
                    reset[key] = {{"from", opt->value}, {"to", 0}};
                    opt->value = 0;
                }
                for (const char* key : {"support_filament", "support_interface_filament", "wipe_tower_filament"}) {
                    auto* opt = dynamic_cast<Slic3r::ConfigOptionInt*>(config.option(key, false));
                    if (opt == nullptr)
                        continue;
                    const int value = std::min(std::max(opt->value, 0), int(filament_count));
                    if (value == opt->value)
                        continue;
                    clamped[key] = {{"from", opt->value}, {"to", value}};
                    opt->value = value;
                }
                if (!reset.empty() || !clamped.empty()) {
                    std::string names;
                    for (const auto& [key, change] : reset.items())
                        names += (names.empty() ? "" : ", ") + key + " " + change["from"].dump() + " -> " +
                                 change["to"].dump();
                    for (const auto& [key, change] : clamped.items())
                        names += (names.empty() ? "" : ", ") + key + " " + change["from"].dump() + " -> " +
                                 change["to"].dump();
                    emit_event({{"event","config_normalized"},
                                {"tag","FilamentIndexOutOfRangeReset"},
                                {"reset", reset},
                                {"clamped", clamped},
                                {"filament_count", int(filament_count)},
                                {"message","The 3mf states filament index value(s) outside this engine's range for its " +
                                           std::to_string(filament_count) + " filament(s): " + names +
                                           " (0 is Default; the desktop resets the same keys on load)"}});
                }
            }
#endif
#ifdef ENGINE_SNAPMAKER
            // The desktop's rule for filament indices out of range on load,
            // after the switch's paint remap (cli_snapmaker.hpp).
            slicer_cli::fix_out_of_range_filaments(model, config, project_filament_count(config), context.g_filament_id_remap,
                                                   context.g_filament_id_total, context.g_filament_count_changed);
#endif
            // Engine fit: a project for a printer this engine does not have is
            // refused before anything is sliced. Without the printer's presets
            // the engine slices the flat file against its own defaults (a
            // 200 x 200 bed for a U1 project on the Bambu build) and reports
            // success. Keyed on printer_model, not on the app that made the
            // file: OrcaSlicer ships Bambu Lab printers too, so a Bambu
            // printer's project is valid on both builds.
            // printer_model as the file states it: a loader that does not take
            // the file for its own app's project (the Bambu loader with an
            // "OrcaSlicer-" Application) leaves the setting at the default.
            std::string file_printer_model;
            {
                std::string text;
                if (read_zip_member(input_file, "Metadata/project_settings.config", text)) {
                    try {
                        const json settings = json::parse(text);
                        if (settings.is_object() && settings.contains("printer_model") &&
                            settings["printer_model"].is_string())
                            file_printer_model = settings["printer_model"].get<std::string>();
                    } catch (...) {
                    }
                }
                if (file_printer_model.empty() && config.has("printer_model"))
                    file_printer_model = config.opt_string("printer_model");
            }
            {
                const std::string& printer_model = file_printer_model;
                if (!printer_model.empty()) {
                    const EngineFit fit = engine_fit_for(o.argv0, printer_model);
                    if (fit.this_catalog_found && !fit.this_has) {
                        const std::string sentence = engine_mismatch_sentence(fit, printer_model);
                        json fits = json::array();
                        if (fit.other_has) fits.push_back(fit.other_binary);
                        emit_event({{"event","engine_mismatch"},
                                    {"tag","PrinterModelNotInEngine"},
                                    {"printer_model", printer_model},
                                    {"engine", fit.this_binary},
                                    {"fits", fits},
                                    {"message", sentence}});
                        std::cerr << "Error: " << sentence << "\n";
                        set_outcome_failure(outcome, CLI_3MF_NOT_SUPPORT_MACHINE_CHANGE, sentence);
                        return 1;
                    }
                }
            }
            // The official version gate (BambuStudio.cpp 1906-1911 at 5873b5f,
            // OrcaSlicer.cpp 1588-1592 at 31f6803): a file saved by a newer
            // major.minor than this engine is refused unless --allow-newer-file.
            // On the Orca build the maker decides which number the file's
            // version is in (G2, see below): a Bambu Studio project is only
            // warned about, an OrcaSlicer project is refused.
            // The refusal sentence adds what the file says about its maker: a
            // Bambu Studio 02.07 project on the Orca 2.4 build is newer only in
            // the other app's numbering, and the binary to use is the maker's.
            if (!o.allow_newer_file && file_version.maj() + file_version.min() > 0) {
                std::string model_xml;
                std::map<std::string, std::string> meta;
                if (read_zip_member(input_file, "3D/3dmodel.model", model_xml))
                    meta = model_metadata(model_xml);
                const auto app = meta.find("Application");
                // G2 (file_version_verdict): a Bambu Studio project on the
                // Orca build is compared with the Bambu base and only warned
                // about -- a BambuStudio 02.05 project is not newer than
                // OrcaSlicer 2.4, it is newer than the base both apps share,
                // and the desktop loads it.
                const FileVersionVerdict verdict = file_version_verdict(file_version, meta);
                if (verdict == FileVersionVerdict::WarnedNewerThanBase)
                    emit_event({{"event","warning"},
                                {"tag","FileNewerThanEngineBase"},
                                {"file_version", file_version.to_string()},
                                {"application", app != meta.end() ? json(app->second) : json(nullptr)},
                                {"base_version", SLIC3R_VERSION},
                                {"unknown_keys", outcome.unknown_settings},
                                {"message","The file is version " + file_version.to_string() +
                                           (app != meta.end() ? " (" + app->second + ")" : std::string()) +
                                           ", newer than the " + SLIC3R_VERSION +
                                           " Bambu base this engine is built on. It is sliced as it is; the "
                                           "setting(s) this engine has no definition for are ignored"}});
                if (verdict == FileVersionVerdict::Refused) {
                    std::string detail = "The file is version " + file_version.to_string() +
                                         (app != meta.end() ? " (" + app->second + ")" : std::string()) +
                                         "; this engine is " + engine_version_text() + "." +
                                         " Pass --allow-newer-file to slice it anyway; the settings this engine"
                                         " does not know are then ignored.";
                    emit_event({{"event","config_refused"},
                                {"tag","FileVersionNewerThanEngine"},
                                {"file_version", file_version.to_string()},
                                {"engine_version", engine_version_text()},
                                {"message", cli_error_sentence(CLI_FILE_VERSION_NOT_SUPPORTED) + " " + detail}});
                    std::cerr << "Error: " << cli_error_sentence(CLI_FILE_VERSION_NOT_SUPPORTED) << " " << detail << "\n";
                    set_outcome_failure(outcome, CLI_FILE_VERSION_NOT_SUPPORTED, detail);
                    return 1;
                    }
            }
            // A setting whose value this engine has no meaning for is a
            // different print from the one the file states, so it is refused,
            // naming the setting, the value and the values this engine has.
            // Only enum
            // substitutions: those are the ones where the engine swapped in
            // its own default for a word it does not know. --allow-substitution
            // keeps the old behaviour (slice with the engine's substitute).
            // A project moved from OrcaSlicer to a Bambu printer can still
            // carry Orca words (ensure_all, rectilinear) that the default call
            // has always substituted; the default call keeps doing so, and the
            // refusal is part of the --slice mode only.
            if (o.slice_mode && !o.allow_substitution) {
                std::vector<std::string>& refused = unknown_values;
                json& items = unknown_items;
                for (const auto& sub : config_subst.substitutions) {
                    if (!sub.opt_def) continue;
                    const auto type = sub.opt_def->type;
                    if (type != Slic3r::coEnum && type != Slic3r::coEnums) continue;
                    std::string allowed;
                    if (sub.opt_def->enum_keys_map)
                        for (const auto& [name, value] : *sub.opt_def->enum_keys_map)
                            allowed += (allowed.empty() ? "" : ", ") + name;
                    // BambuStudio's "partial" skips one shell pass (BambuStudio
                    // PrintObject.cpp 1944, 1975 at 5873b5f) and this engine has
                    // no shell pass to skip, so the value is refused with the
                    // choice to make rather than the engine's generic sentence.
                    // "enabled"/"disabled" are the same meaning in another
                    // encoding and are converted by handle_legacy
                    // (libslic3r/orcaslicer/libslic3r/PrintConfig.cpp).
                    const bool partial_shell =
                        sub.opt_def->opt_key == "ensure_vertical_shell_thickness" && sub.old_value == "partial";
                    refused.push_back(partial_shell
                        ? "'ensure_vertical_shell_thickness' is 'partial', BambuStudio's skip-one-shell-pass mode, "
                          "which this engine has no value for (its values: " + allowed +
                          "); pick the shell coverage you want, e.g. ensure_all for BambuStudio's 'enabled'"
                        : "'" + sub.opt_def->opt_key + "' is '" + sub.old_value +
                          "', which this engine does not have (its values: " + allowed + ")");
                    items.push_back(json{{"opt_key", sub.opt_def->opt_key}, {"value", sub.old_value},
                                         {"allowed", allowed}});
                }
            }
#ifdef ENGINE_BAMBU
            // The importer may partially mutate a vector before rejecting it.
            // Retain the last usable input map, or the original driver default
            // when no usable map was supplied. This does not establish provenance.
            config.set_key_value("filament_nozzle_map", accepted_3mf_nozzle_map.clone());
#endif
            if (is_bbl_3mf)
                // The file's own settings when the rebase kept them apart: the
                // facts describe the file (its printer, bed, plate membership
                // and the maker's changed-key list), and the printer change
                // reads them to find the system preset the file's process
                // inherits from. full_fff_config erases
                // different_settings_to_system and writes a new inherits_group,
                // so the rebased settings would answer a different question.
                slicer_cli::read_project_facts(project_file_config_kept ? project_file_config : config,
                                               project_facts);
            // is_bbl_3mf ends false once a model file follows the project
            // (BambuStudio.cpp 1871; OrcaSlicer.cpp 1553): the settings steps
            // for a Bambu-made project do not run.
            if (trailing_port)
                project_facts.is_bbl_3mf = false;
            rs.file_version = file_version;
            for (const Slic3r::Preset* p : presets)
                if (p) project_presets_kept.push_back(*p);
            // Validate --plate against actual plate count
            if (plate_id > 0 && (int)plate_data.size() < plate_id) {
                emit_event({{"event","input_error"},
                            {"tag","PlateOutOfRange"},
                            {"requested_plate", plate_id},
                            {"plate_count", (int)plate_data.size()},
                            {"message","--plate " + std::to_string(plate_id) +
                                       " but the 3MF has only " +
                                       std::to_string(plate_data.size()) + " plate(s)"}});
                std::cerr << "Error: --plate " << plate_id
                          << " but 3MF only has " << plate_data.size() << " plate(s)\n";
                set_outcome_failure(outcome, CLI_INVALID_PARAMS);
                return 1;
            }

            // ── Multi-plate coordinate translation ───────────────────────
            // A Bambu/Orca 3MF stores every object at its scene position: plate
            // i (0-based) sits at the grid origin the desktop's PartPlateList
            // gives it, and the CLI slices that plate with its print origin set
            // there.  Port of that rule (pins 5873b5f / 31f6803):
            //   - plate size = (int)(printable_area[2] - printable_area[0]) from
            //     the FILE's config (BambuStudio.cpp 2054-2056), plus
            //     bed3d_ax3s_default_tip_radius (2.5 * 0.5 mm) for files older
            //     than 1.5.9 (BambuStudio.cpp 1911-1914, 4316-4318; OrcaSlicer.cpp
            //     3727-3729);
            //   - cols = compute_colum_count(plate count) (PartPlate.hpp:38);
            //   - origin(i) = (col * w * 1.2, -row * d * 1.2) with
            //     LOGICAL_PART_PLATE_GAP = 1/5 (PartPlate.cpp:55, 4343-4355, 5240-5254).
            // The objects stay at their scene places in the one model, and
            // each plate is sliced in its own frame (the plate's instances
            // moved by minus its origin, PlateScope; see the Print's origin at
            // print.apply). The objects stay where the maker put them on their
            // plate: no snap to the bed
            // corner, no plate_N.json bbox guess (both moved parts, and on beds
            // with an exclusion area the corner snap pushed them into it).
            if (is_bbl_3mf)
                for (int index = 0; index < (int)plate_data.size(); ++index)
                    plate_origins.push_back(plate_grid_origin(config, file_version, index, (int)plate_data.size()));
            // The plates of the run: the one plate loaded (every instance of
            // the model is on it), or with every plate loaded each plate's
            // instances as the file states them (PlateData::obj_inst_map's
            // identify ids, else the first plate an instance's box meets:
            // PartPlateList::reload_all_objects -> PartPlate::intersect_instance,
            // PartPlate.cpp 5668-5710, 2651-2677).
            if (is_bbl_3mf && plate_id > 0) {
                rs.plates.assign(plate_data.size(), RunPlate());
                for (int index = 0; index < (int)plate_data.size(); ++index) {
                    rs.plates[index].origin = plate_origins[index];
                    rs.plates[index].all = index == plate_id - 1;
                }
            } else if (is_bbl_3mf && plate_data.size() > 1 && !trailing_port) {
                rs.plates.assign(plate_data.size(), RunPlate());
                std::set<const Slic3r::ModelInstance*> placed;
                for (int index = 0; index < (int)plate_data.size(); ++index) {
                    rs.plates[index].origin = plate_origins[index];
                    rs.plates[index].all = false;
                    if (plate_data[index] == nullptr) continue;
                    std::set<int> identify_ids;
                    for (const auto& entry : plate_data[index]->obj_inst_map)
                        if (entry.second.second > 0) identify_ids.insert(entry.second.second);
                    for (Slic3r::ModelObject* object : model.objects)
                        for (Slic3r::ModelInstance* inst : object->instances)
                            if (inst->loaded_id > 0 && identify_ids.count(int(inst->loaded_id)) && placed.insert(inst).second)
                                rs.plates[index].members.push_back(inst);
                }
                for (Slic3r::ModelObject* object : model.objects)
                    for (size_t ii = 0; ii < object->instances.size(); ++ii) {
                        if (placed.count(object->instances[ii])) continue;
                        const Slic3r::BoundingBoxf3 box = object->instance_convex_hull_bounding_box(ii);
                        for (int index = 0; index < (int)plate_data.size(); ++index)
                            if (plate_scene_box(config, file_version, index, (int)plate_data.size()).intersects(box)) {
                                rs.plates[index].members.push_back(object->instances[ii]);
                                break;
                            }
                    }
            }
            if (trailing_port || plate_id == 0) {
                plan_file_config = config;
                plan_file_version = file_version;
                if (const auto* ids = config.option<Slic3r::ConfigOptionStrings>("filament_settings_id"))
                    plan_file_filaments = (int)ids->values.size();
            }
            // A plate prints by object when its own print_sequence says so,
            // else when the project's does (get_print_sequence,
            // BambuStudio.cpp 4403-4416; the plate's own settings go over the
            // project's per plate, 6902-6904). plate_N.json's is_seq_print is
            // what the desktop's last slice found, an output, not a setting.
            if (plate_id > 0 && is_bbl_3mf && !model.objects.empty()) {
                const Slic3r::Vec2d origin = plate_grid_origin(config, file_version, plate_id - 1, (int)plate_data.size());
                if (verbose)
                    std::cout << "Plate " << plate_id << " of " << plate_data.size()
                              << ": grid origin (" << origin.x() << ", " << origin.y() << ")\n";
            }

#ifdef ENGINE_BAMBU
            // ── Rebuild config via PresetBundle ──────────────────────────
            // The 3MF's project_settings.config is a merged flat file with
            // multi-element arrays from the printer profile's variant support.
            // BambuStudio desktop builds config from individual presets via
            // PresetBundle::full_config(), producing correctly-sized vectors.
            // Replicate that here: load system presets → select by name →
            // full_config() → overlay any 3MF-only keys.
            //
            // If this fails (profiles not found, names don't match, etc.)
            // we silently fall back to the flat 3MF config from load_bbs_3mf.
            {
                // Same executable dir and profiles tree that
                // configure_engine_resources() resolved at startup, including
                // the package-root layout. exe_dir is kept for the
                // preset_resolution_failed diagnostic below.
                const boost::filesystem::path exe_dir = engine_executable_dir(o.argv0.c_str());
                const boost::filesystem::path profiles_dir = engine_profiles_dir(exe_dir);

                bool preset_loaded = false;
                if (!profiles_dir.empty()) {
                    try {
                        // Prepare data_dir/system/ with symlinks to vendor dirs.
                        // Always start fresh to avoid stale cached copies of
                        // vendor JSON files that can cause extruder variant
                        // lookup failures (see update_values_to_printer_extruders).
                        // One staging directory per process: two packaged slices
                        // running at once must not delete each other's vendor
                        // files mid-load (a shared fixed path with remove_all did).
                        //
                        // The cleanup guard lives outside the fatal try below so
                        // the staged tree survives the PresetBundle load; it gets
                        // its path before create_directories so a half-created
                        // staging dir is still removed.
                        boost::filesystem::path tmpdir;
                        boost::filesystem::path sysdir;
                        struct StagingCleanup {
                            boost::filesystem::path dir;
                            ~StagingCleanup() { remove_owned_temp_dir(dir); }
                        } staging_cleanup;
                        // A staging step that fails — creating the staging dir
                        // or a copy that even the stream-copy fallback cannot
                        // complete — is fatal: the same event + exit-code shape
                        // as the other preset failures below, except the slice
                        // stops here instead of running on the flat 3MF config
                        // with no presets at all.
                        try {
                            tmpdir = make_owned_temp_dir("slicer_cli_presets-");
                            sysdir = tmpdir / "system";
                            staging_cleanup.dir = tmpdir;
                            boost::filesystem::create_directories(sysdir);
                            for (auto& entry : boost::filesystem::directory_iterator(profiles_dir)) {
                                auto dst = sysdir / entry.path().filename();
                                if (boost::filesystem::is_directory(entry.path()))
                                    stage_vendor_dir(entry.path(), dst);
                                else if (entry.path().extension() == ".json")
                                    stage_file_copy(entry.path(), dst);
                            }
                        } catch (const std::exception& e) {
                            emit_event({{"event","preset_error"},
                                        {"tag","PresetStagingFailed"},
                                        {"exe_dir", exe_dir.empty() ? std::string{} : exe_dir.string()},
                                        {"profiles_dir", profiles_dir.string()},
                                        {"message", std::string("Preset staging failed: ") + e.what()}});
                            std::cerr << "  Preset staging failed: " << e.what() << "\n";
                            set_outcome_failure(outcome, CLI_ENVIRONMENT_ERROR, e.what());
                            return 1;
                        }
                        // resources_dir() already points at profiles_dir/.. — set
                        // once for both engines by configure_engine_resources().
                        Slic3r::set_data_dir(tmpdir.string());

                        Slic3r::PresetBundle preset_bundle;

                        // Load ALL vendors (BBL, Creality, Qidi, …)
                        // First vendor uses LoadSystem (resets bundle),
                        // subsequent vendors just load without reset.
                        bool first_vendor = true;
                        for (auto& dir_entry : boost::filesystem::directory_iterator(sysdir)) {
                            if (dir_entry.path().extension() != ".json")
                                continue;
                            std::string vname = dir_entry.path().stem().string();
                            if (vname == "blacklist") continue;
                            try {
                                if (first_vendor) {
                                    preset_bundle.load_vendor_configs_from_json(
                                        sysdir.string(), vname,
                                        Slic3r::PresetBundle::LoadSystem,
                                        Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
                                    first_vendor = false;
                                } else {
                                    // load_vendor_configs_from_json without LoadSystem
                                    // won't reset — it appends to existing presets
                                    preset_bundle.load_vendor_configs_from_json(
                                        sysdir.string(), vname,
                                        Slic3r::PresetBundle::LoadConfigBundleAttributes(),
                                        Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
                                }
                            } catch (...) {
                                // Skip vendors that fail to load
                            }
                        }

                        // Read preset names from the 3MF config
                        std::string printer_name = config.opt_string("printer_settings_id");
                        std::string print_name   = config.opt_string("print_settings_id");
                        std::string filament_name;
                        if (auto* fsi = config.option<Slic3r::ConfigOptionStrings>("filament_settings_id"))
                            if (!fsi->values.empty())
                                filament_name = fsi->values[0];

                        std::cout << "  Preset lookup: printer='" << printer_name
                                  << "' print='" << print_name
                                  << "' filament='" << filament_name << "'\n";

                        if (!printer_name.empty() && !print_name.empty() && !filament_name.empty()) {
                            bool ok_printer  = preset_bundle.printers.select_preset_by_name(printer_name, true);
                            bool ok_print    = preset_bundle.prints.select_preset_by_name(print_name, true);
                            bool ok_filament = preset_bundle.filaments.select_preset_by_name(filament_name, true);

                            std::cout << "  Preset match: printer=" << ok_printer
                                      << " (resolved='" << preset_bundle.printers.get_edited_preset().name << "')"
                                      << " print=" << ok_print
                                      << " (resolved='" << preset_bundle.prints.get_edited_preset().name << "')"
                                      << " filament=" << ok_filament
                                      << " (resolved='" << preset_bundle.filaments.get_edited_preset().name << "')\n";

                            // Only use PresetBundle config if ALL three presets were found
                            // (not just the default fallbacks)
                            if (ok_printer && ok_print && ok_filament
                                && preset_bundle.printers.get_edited_preset().name == printer_name
                                && preset_bundle.prints.get_edited_preset().name == print_name
                                && preset_bundle.filaments.get_edited_preset().name == filament_name)
                            {
                                // Same flow as BambuStudio desktop:
                                // full_config() builds defaults → printer → process → filament
                                // then apply project_settings.config on top.
                                Slic3r::DynamicPrintConfig base_config = preset_bundle.full_config();
                                base_config.apply(config, /*ignore_nonexistent=*/true);
                                config = std::move(base_config);
                                preset_loaded = true;

                                if (verbose)
                                    std::cout << "  Presets: " << printer_name
                                              << " / " << print_name
                                              << " / " << filament_name << "\n";
                            }
                        }
                    } catch (const std::exception& e) {
                        emit_event({{"event","preset_error"},
                                    {"tag","PresetBundleException"},
                                    {"message", std::string("PresetBundle exception: ") + e.what()}});
                        std::cerr << "  PresetBundle exception: " << e.what() << "\n";
                    }
                }
                if (!preset_loaded) {
                    // The slice is about to run on the flat, merged 3MF config
                    // instead of a preset-resolved one. The desktop app can
                    // never be in this state, so nothing downstream expects it;
                    // it must not stay a plain stdout line.
                    emit_event({{"event","preset_resolution_failed"},
                                {"tag","FlatThreeMfConfigFallback"},
                                {"exe_dir", exe_dir.empty() ? std::string{} : exe_dir.string()},
                                {"profiles_dir", profiles_dir.empty() ? std::string{} : profiles_dir.string()},
                                {"message","Presets were not resolved; slicing from the flat 3MF config"}});
                    std::cout << "  WARNING: Using flat 3MF config (presets not resolved)\n";
                    if (!exe_dir.empty())
                        std::cout << "  exe_dir: " << exe_dir << "\n";
                    else
                        std::cout << "  exe_dir: <empty — could not determine executable path>\n";
                    if (!profiles_dir.empty())
                        std::cout << "  profiles_dir: " << profiles_dir << "\n";
                    else
                        std::cout << "  profiles_dir: <not found>\n";
                }
            }
#endif // ENGINE_BAMBU — PresetBundle preset-resolution + staging symlinks.
       // OrcaSlicer slices directly from the flat 3MF project_settings.config.
        } else if (calib_self_geometry && input_file.empty()) {
            std::cout << "No --input: pressure_advance_pattern will synthesize a handle cube.\n";
        } else {
            // The kinds this engine's loaders read (cli_model_load.cpp).
            std::cerr << "Unsupported file format. This engine loads: "
                      << slicer_cli::loadable_model_extensions() << "\n";
            set_outcome_failure(outcome, CLI_DATA_FILE_ERROR);
            return 1;
        }

        // For pressure_advance_pattern the model is generated later by
        // apply_pa_pattern (before print.apply), so an empty model here is fine.
        if (model.objects.empty() && !calib_self_geometry) {
            std::cerr << "No objects found in model\n";
            set_outcome_failure(outcome, CLI_NO_SUITABLE_OBJECTS);
            return 1;
        }

        // Model files after a project 3MF: loaded the same way as geometry
        // (the official CLI reads them after the first file, BambuStudio.cpp
        // 1855-2163) and placed further down with the other new objects.
        if (!geometry_input && o.input_files.size() > 1) {
            const std::vector<std::string> rest(o.input_files.begin() + 1, o.input_files.end());
            const slicer_cli::ModelLoadResult loaded = slicer_cli::load_geometry_files(o, rest, 1, model);
            slicer_cli::add_failure_reason(outcome.error_reason, loaded.failure_reason);
            if (loaded.code != 0) {
                emit_event({{"event","load_error"}, {"tag","ModelLoadFailed"}, {"path", rest.front()},
                            {"message", loaded.message}});
                std::cerr << "Error: " << loaded.message << "\n";
                set_outcome_failure(outcome, loaded.code, loaded.message);
                return 1;
            }
        }
        // A run of one plate (model files, a 3MF sliced as one plate): every
        // instance is on it, at the origin.
        if (rs.plates.empty())
            rs.plates.assign(1, RunPlate());
#ifdef ENGINE_BAMBU
        // Bambu Studio puts each plate's saved tower back inside the plate
        // when it opens a project (cli_project_load.hpp). Here, before the
        // flags are laid on the settings: a position a flag gives stays as
        // given, and the project's settings, which the exports write, hold
        // the corrected one. Every plate whose objects are loaded (one plate
        // with --slice N or --plate N). --arrange places the tower itself.
        if (rs.project_loaded_like_desktop && !o.arrange_forced())
            if (const auto* area = config.option<Slic3r::ConfigOptionPoints>("printable_area");
                area != nullptr && area->values.size() >= 3) {
                const Slic3r::BoundingBoxf reach = tower_reach_box(config, *area);
                const double height = config.has("printable_height") ? config.opt_float("printable_height") : 0.;
                const Slic3r::BoundingBoxf3 plate_box(Slic3r::Vec3d(reach.min.x(), reach.min.y(), 0.),
                                                      Slic3r::Vec3d(reach.max.x(), reach.max.y(), height));
                const TowerWipeParams wipe = tower_wipe_params(config);
                const auto exclude = arrange_exclude_boxes(config);
                for (int index = 0; index < (int)rs.plates.size(); ++index) {
                    if (plate_id > 0 && index != plate_id - 1)
                        continue;
                    PlateScope scope(model, rs.plates[index]);
                    slicer_cli::project_load_tower_inside_plate(
                        config, index, (int)rs.plates.size(), reach,
                        (int)arrange_plate_extruders(model, config, index).size(),
                        [&](int filaments) {
                            return arrange_estimate_wipe_tower_size(model, config, wipe.width, wipe.volume,
                                                                    wipe.extruder_count, filaments, wipe.wrapping,
                                                                    plate_box, exclude, nullptr,
                                                                    /*legacy_behavior=*/false);
                        });
                }
            }
#else
        // OrcaSlicer and Snapmaker Orca keep a project's saved tower position
        // when they open it (cli_project_load.hpp), and their slice reads a
        // plate the list holds no position for as the first plate's
        // (get_at, Config.hpp 627 at 8500fcd; 438 at bac6fd5).
        if (rs.project_loaded_like_desktop && !o.arrange_forced())
            slicer_cli::project_load_tower_positions_per_plate(config, (int)rs.plates.size());
#endif
        const bool has_unplaced_objects = std::any_of(model.objects.begin(), model.objects.end(),
            [](const Slic3r::ModelObject* obj) { return obj->instances.empty(); });

        // Ensure all objects have at least one instance
        // Also set use_loaded_id_for_label so that the identify_id from
        // model_settings.config is used for OBJECT_ID labels in G-code
        // (matches BambuStudio desktop behavior at BambuStudio.cpp:6196).
        // Geometry still to be placed gets its instance from the placement.
        for (auto* obj : model.objects) {
            if (obj->instances.empty() && !has_unplaced_objects) {
                obj->add_instance();
            }
            for (auto* inst : obj->instances) {
                inst->use_loaded_id_for_label = true;
            }
        }

        std::cout << "Model loaded successfully:\n";
        for (const auto* obj : model.objects) {
            std::cout << "  - " << obj->name << " (" << obj->volumes.size()
                      << " volumes, " << obj->instances.size() << " instances)\n";
        }

        // Mesh auto-repair census. The GUI puts a warning icon beside every
        // object whose mesh admesh had to repair, with a "(Repair)" hyperlink
        // (GUI_ObjectList.cpp:516/521). The CLI repairs silently, so an agent
        // cannot tell a clean input from one that was rewritten under it.
        for (auto* obj : model.objects) {
            const int repaired = obj->get_repaired_errors_count();
            if (repaired <= 0) continue;
            emit_event({{"event","mesh_repaired"},
                        {"tag","RepairedMeshErrors"},
                        {"object", obj->name},
                        {"repaired_error_count", repaired},
                        {"message","Mesh errors were auto-repaired on load for '" + obj->name + "'"}});
        }

        // Load config files in order (later ones can override earlier ones)
        // A profile that fails to load leaves the slice running on whatever was
        // already in `config` — silently, with only a stderr line. The agent
        // that chose that profile must be told it did not take effect.
        auto load_profile = [&](const std::string& path, const char* kind) {
            if (path.empty()) return;
            if (load_json_config(path, config, verbose,
                                 std::string("profile:") + kind + ":" + path,
#ifdef ENGINE_BAMBU
                                 &explicit_config_supplied_nozzle_map
#else
                                 nullptr
#endif
                                 )) {
                // A shipped preset holds only its differences from its
                // parents: read the rest of the chain in, so the file means
                // the same here as it does in the desktop app.
                apply_file_inherits(o.argv0, path, kind, config, verbose);
                return;
            }
            emit_event({{"event","config_load_failed"},
                        {"tag","ProfileLoadFailed"},
                        {"kind", kind},
                        {"path", path},
                        {"message", std::string("Failed to load ") + kind +
                                    " config; slicing continues with the previously resolved settings"}});
            std::cerr << "Warning: Failed to load " << kind << " config\n";
            slicer_cli::add_failure_reason(outcome.error_reason, std::string("The ") + kind + " file " + path +
                                                                 " could not be read, so its settings were not used.");
        };
        // Presets by name: the desktop app's full_config() for the named
        // printer, process and filaments, over the engine defaults.
        if (context.g_preset_config)
            config.apply(*context.g_preset_config);
        // Capture preset names before raw settings files can replace their
        // id fields. A settings file is configuration input, not proof that
        // its arbitrary ids were selected from this engine's preset catalog.
        // Project inputs, including named project switches, are deliberately
        // omitted: an embedded id can be arbitrary and switch defaults are
        // decided later in the settings merge.
        if (context.g_preset_config) {
            if (const auto* id = config.option<Slic3r::ConfigOptionString>("printer_settings_id"))
                outcome.printer_preset = id->value;
            if (const auto* id = config.option<Slic3r::ConfigOptionString>("print_settings_id"))
                outcome.process_preset = id->value;
            if (const auto* ids = config.option<Slic3r::ConfigOptionStrings>("filament_settings_id"))
                outcome.filament_presets = ids->values;
        }
        load_profile(bundle_config,   "bundle");
        load_profile(machine_config,  "machine");
        load_profile(process_config,  "process");
        load_profile(filament_config, "filament");
        // --load-settings, --load-filaments, --uptodate*: the official
        // settings merge (cli_load_settings.cpp), on the settings loaded so far.
        // (rs.settings_merge; rs.merged_print_sequence: the project's
        // print_sequence after the merge, the official m_print_config's.)
#ifdef ENGINE_BAMBU
        // An assemble list's OBJ colours become filaments in the merge, which
        // needs the filaments loaded (BambuStudio.cpp 2574-2580).
        if (assemble_input && !context.g_assemble->colours.empty()) {
            settings_merge.input_obj_colours.assign(context.g_assemble->colours.begin(), context.g_assemble->colours.end());
            if (!slicer_cli::wants_settings_merge(o)) {
                const std::string why = "The assemble list's OBJ files carry colours, which become filaments: "
                                        "give the filaments with --load-filaments.";
                std::cerr << "Error: " << why << "\n";
                set_outcome_failure(outcome, CLI_INVALID_PARAMS, why);
                return 1;
            }
        }
#endif
        if (slicer_cli::wants_settings_merge(o)) {
            // Whether the settings loaded so far carry filament_colour (the
            // official m_print_config holds no engine defaults yet; see
            // SettingsMerge::project_has_filament_colour): a project 3MF's
            // own settings, the named presets' full config, or a profile file
            // that names it.
            bool has_colours = (!input_file.empty() && input_is_3mf(input_file) && !geometry_input && !assemble_input) ||
                               context.g_preset_config != nullptr;
            for (const std::string* path : {&bundle_config, &machine_config, &process_config, &filament_config})
                has_colours = has_colours || json_file_has_key(*path, "filament_colour");
            settings_merge.project_has_filament_colour = has_colours;
            std::vector<Slic3r::Preset*> preset_ptrs;
            for (Slic3r::Preset& p : project_presets_kept)
                preset_ptrs.push_back(&p);
            // The official merges into m_print_config, which holds the
            // project's print_sequence, not a plate's own (get_print_sequence
            // reads the plate's first, BambuStudio.cpp 4403-4416).
            const slicer_cli::StepResult merged =
                slicer_cli::merge_loaded_settings(o, project_facts, this_engine_profiles_dir(o.argv0).string(), config, extra,
                                                  preset_ptrs, settings_merge);
            if (const Slic3r::ConfigOption* seq = config.option("print_sequence"))
                merged_print_sequence = seq->serialize();
            if (merged.code != 0) {
                emit_event({{"event","config_refused"}, {"tag","SettingsFilesRefused"}, {"message", merged.message}});
                std::cerr << "Error: " << merged.message << "\n";
                set_outcome_failure(outcome, merged.code, merged.message);
                return 1;
            }
            slicer_cli::update_object_configs_after_switch(project_facts, settings_merge, config, model);
            outcome.settings_merge = std::make_shared<slicer_cli::SettingsMerge>(settings_merge);
            if (slicer_cli::disable_tower_after_mapping(o, settings_merge, config, extra))
                emit_event({{"event","config_normalized"}, {"tag","PrimeTowerOffOneFilament"},
                            {"message","Every filament slot holds the same filament: the prime tower is off"}});
        }
        // --load-custom-gcodes, --skip-modified-gcodes (cli_load_settings.cpp).
        {
            const slicer_cli::StepResult g = slicer_cli::apply_custom_gcodes(o, o.slice_mode ? o.slice_plate : plate_id, model);
            if (g.code != 0) {
                std::cerr << "Error: " << g.message << "\n";
                set_outcome_failure(outcome, g.code, g.message);
                return 1;
            }
        }
        // Every setting given as a flag, over the loaded settings: the
        // official m_print_config.apply(m_extra_config, true)
        // (BambuStudio.cpp 4091 at 5873b5f; OrcaSlicer.cpp 3542 at 31f6803).
        // The settings before any flag: the settings check's desktop baseline.
        const Slic3r::DynamicPrintConfig settings_before_flags = config;
#ifdef ENGINE_BAMBU
        // The filament switcher and the smart filament assignment that needs
        // it are off unless a flag states them, whatever the project or its
        // plate saved: the official CLI's rule (BambuStudio.cpp 4436-4439 at
        // v02.09.01.70). They describe hardware fitted to the maker's
        // printer, which a headless run cannot see.
        for (const char* key : {"enable_filament_dynamic_map", "has_filament_switcher"})
            if (!extra.has(key))
                extra.set_key_value(key, new Slic3r::ConfigOptionBool(false));
        // --estimate-mode slices with the automatic grouping and a full
        // Standard rack (BambuStudio.cpp 4441-4474 at v02.09.01.70).
        if (o.given_flag("estimate_mode") && o.cli.opt_bool("estimate_mode")) {
            const std::string why = slicer_cli::estimate_mode_nozzle_settings(extra, config);
            if (!why.empty()) {
                std::cerr << "Error: " << why << "\n";
                set_outcome_failure(outcome, CLI_INVALID_PARAMS, why);
                return 1;
            }
        }
#endif
        if (!extra.empty())
            config.apply(extra, true);

        // Two inputs the desktop app decides by choosing a printer preset, not
        // by editing a value: the nozzle and the plate type. Both are settled
        // here, once, where the settings files and the flags have landed and
        // before any plate is sliced.
        {
            // A string setting as the file states it, empty when the key is not
            // there (the engine's own printer_model / printer_variant /
            // printer_settings_id are plain strings in both engines).
            const auto id_text = [](const Slic3r::DynamicPrintConfig& c, const char* key) {
                const auto* opt = c.option<Slic3r::ConfigOptionString>(key);
                return opt != nullptr ? opt->value : std::string();
            };
            const std::string printer_model   = id_text(config, "printer_model");
            const std::string printer_variant = id_text(config, "printer_variant");
            const bool nozzle_given = overrides.count("nozzle_diameter") > 0;
            // The plate type the desktop app sets when a printer is picked
            // (Sidebar::update_all_preset_comboboxes -> set_bed_type_accord_combox,
            // BambuStudio Plater.cpp 3340-3414 and 3703-3715 at 5873b5f;
            // OrcaSlicer Plater.cpp 2527-2556 and 2798-2806 at 31f6803), for a
            // run whose printer comes from settings files: those state no plate
            // type, so the printer preset's own is taken. Without this the
            // engine's own default is used (curr_bed_type's ConfigDef default,
            // btPC "Cool Plate": BambuStudio PrintConfig.cpp 1162 at 5873b5f),
            // which is never the printer's. The named-preset path already sets
            // it (resolve_named_presets -> desktop_bed_type).
            bool plate_chosen = o.given_flag("curr_bed_type");
            for (const std::string* path : {&bundle_config, &machine_config, &process_config, &filament_config})
                plate_chosen = plate_chosen || (!path->empty() && json_file_has_key(*path, "curr_bed_type"));
            const bool plate_needed = !plate_chosen && context.g_preset_config == nullptr &&
                                      (!input_is_3mf(input_file) || geometry_only_3mf);
            if (nozzle_given || plate_needed) {
                // The printer preset this run's settings belong to: the name
                // the settings state, else the preset this engine ships for the
                // same printer model and nozzle variant. A settings file the
                // desktop exported (Export current configs) carries no
                // printer_settings_id at all, and a shipped leaf file carries
                // one that is empty.
                const std::vector<ShippedPrinterPreset> shipped =
                    shipped_printer_presets(this_engine_profiles_dir(o.argv0), printer_model);
                const std::string printer_settings = id_text(config, "printer_settings_id");
                std::string       printer_preset   = printer_settings;
                if (printer_preset.empty() ||
                    std::none_of(shipped.begin(), shipped.end(),
                                 [&](const ShippedPrinterPreset& p) { return p.name == printer_preset; }))
                    if (const std::string by_variant = shipped_printer_preset_of_variant(shipped, printer_variant);
                        !by_variant.empty())
                        printer_preset = by_variant;

                // ── --nozzle ────────────────────────────────────────────────
                // The desktop app changes a printer's nozzle by choosing the
                // printer variant: the printer list is rebuilt per nozzle
                // variant and the process, the line widths and the start G-code
                // come from the chosen preset
                // (Sidebar::update_all_preset_comboboxes, BambuStudio Plater.cpp
                // 3340-3414 at 5873b5f; OrcaSlicer Plater.cpp 2527-2556 at
                // 31f6803). Editing nozzle_diameter alone leaves the 0.4 line
                // widths and the 0.4 process in place, so a value the printer
                // preset is not built for is refused, naming the preset to
                // choose instead; a value it already has is what it already is.
                if (nozzle_given) {
                    const std::string& given = overrides.find("nozzle_diameter")->second;
                    std::vector<double> current;
                    if (const auto* nd = config.option<Slic3r::ConfigOptionFloats>("nozzle_diameter"))
                        current = nd->values;
                    double wanted = 0.;
                    bool   number = false;
                    try {
                        size_t used = 0;
                        wanted = std::stod(given, &used);
                        number = used == given.size() && std::isfinite(wanted);
                    } catch (...) {
                    }
                    if (number && !current.empty() &&
                        !std::all_of(current.begin(), current.end(),
                                     [&](double v) { return std::abs(v - wanted) < 1e-6; })) {
                        std::ostringstream has;
                        for (size_t i = 0; i < current.size(); ++i) has << (i ? ", " : "") << current[i];
                        std::ostringstream variant;
                        variant << wanted;
                        const std::string wanted_preset =
                            shipped_printer_preset_of_variant(shipped, variant.str());
                        std::string instead;
                        if (!wanted_preset.empty())
                            instead = "--printer-preset \"" + wanted_preset + "\"";
                        else if (!printer_model.empty())
                            instead = "--printer-preset \"" + printer_model + " " + variant.str() + " nozzle\"";
                        else
                            instead = "the printer preset of a " + variant.str() + " mm nozzle";
                        std::string sentence =
                            "--nozzle " + given + " does not change the printer: " +
                            (printer_preset.empty() ? std::string("this printer") : "'" + printer_preset + "'") +
                            " is built for a " + has.str() + " mm nozzle. The desktop app changes the nozzle by "
                            "choosing the printer variant, so give the printer preset of that nozzle instead: " +
                            instead + ".";
                        if (!shipped.empty())
                            sentence += " This engine ships " + preset_name_list(shipped, 4) + ".";
                        emit_event({{"event","config_refused"}, {"tag","NozzleFlagNotAPrinterVariant"},
                                    {"nozzle", given}, {"printer_variant", printer_variant},
                                    {"printer_preset", printer_preset}, {"message", sentence}});
                        std::cerr << "Error: " << sentence << "\n";
                        set_outcome_failure(outcome, CLI_INVALID_PARAMS, sentence);
                        return 1;
                    }
                }

                // ── the plate type ──────────────────────────────────────────
                // The same desktop_bed_type the named-preset path applies, so a
                // run whose printer comes from settings files cannot land on a
                // different plate than --printer-preset does. The rule it
                // ports: the desktop sets the plate when a printer is picked
                // (OrcaSlicer Plater.cpp 2527-2563 at 31f6803, both sides of
                // its is_bbl_vendor || support_multi_bed_types test, through
                // Preset::get_default_bed_type, Preset.cpp 931-958 at 31f6803
                // — for the Snapmaker U1 the file's stated NAME "Textured PEI
                // Plate" reads as no number, which gives btPEI, the High Temp
                // Plate — and BambuStudio Plater.cpp 3340-3414, 3703-3715 at
                // 5873b5f). Without this the engine's own default is used
                // (curr_bed_type's ConfigDef default, btPC "Cool Plate":
                // BambuStudio PrintConfig.cpp 1162 at 5873b5f), which is no
                // printer's — and on the U1 it is a plate whose bed
                // temperature the shipped filament chain leaves at 0, so the
                // run is refused as unprintable at the first layer.
                if (plate_needed && !printer_preset.empty()) {
                    Slic3r::BedType bed = Slic3r::btPC;
                    std::string     why;
                    if (desktop_plate_type_for_printer(o, printer_preset, bed, why)) {
                        config.set_key_value("curr_bed_type", new Slic3r::ConfigOptionEnum<Slic3r::BedType>(bed));
                        emit_event({{"event","presets_resolved"}, {"tag","PrinterDefaultPlate"},
                                    {"printer", printer_preset},
                                    {"curr_bed_type", slicer_cli::bed_type_name(bed)},
                                    {"curr_bed_type_reason", why},
                                    {"message","The plate type is the printer preset's own: '" +
                                                   slicer_cli::bed_type_name(bed) + "' (" + why +
                                                   "); give --curr-bed-type to choose another"}});
                    }
                }
            }
        }

        // The official value check (OrcaSlicer.cpp 3574-3581 at 31f6803,
        // BambuStudio.cpp 4134-4141 at 5873b5f): DynamicPrintConfig::validate(true)
        // over the settings as the file and profiles state them, before this
        // driver pads any vector. Out-of-range values are named with the
        // engine's own sentence ("tree_support_wall_count: -1 not in range
        // [0,2]"); which of them refuse and which only warn is below.
        // The official check runs after the command line is applied
        // (m_print_config.apply(m_extra_config) at BambuStudio.cpp 4091,
        // OrcaSlicer.cpp 3542), so a command-line override is range-checked
        // too. The overrides themselves still land where they always have
        // (further down); here they go onto a copy for the check only, and
        // without overrides the check reads `config` as before.
        if (!calib_self_geometry) {
            // The plate's own settings are checked the same way, in the order
            // they are applied below (plate, then command line). The official
            // CLI checks m_print_config (BambuStudio.cpp 4134) before it lays
            // the plate over it (6903), so it never range-checks a plate
            // value; checking the settings a plate is sliced with is stricter
            // and changes nothing for a plate that states none.
            // Every plate the run slices: plate N, or each plate for --slice 0.
            std::vector<int> checked_plates;
            if (plate_id > 0 || plate_data.size() <= 1)
                checked_plates.push_back(plate_id);
            else
                for (int k = 1; k <= (int)plate_data.size(); ++k)
                    checked_plates.push_back(k);
            for (const int checked : checked_plates) {
            const Slic3r::DynamicPrintConfig plate_settings = plate_own_settings(plate_data, checked);
            std::unique_ptr<Slic3r::DynamicPrintConfig> with_overrides;
            if (!overrides.empty() || !plate_settings.empty()) {
                with_overrides = std::make_unique<Slic3r::DynamicPrintConfig>(config);
                with_overrides->apply(plate_settings, true);
                with_overrides->apply(extra, true);
                apply_command_line_overrides(*with_overrides, overrides, /*report_rejections=*/false);
            }
            const std::map<std::string, std::string> found =
                (with_overrides ? *with_overrides : config).validate(true);
            // The desktop app runs the same check on the settings a 3MF
            // brings (config.validate(), not under the CLI) and only shows a
            // notice, "Invalid values found in the 3MF" (OrcaSlicer
            // Plater.cpp 6283-6297 at v2.4.2; BambuStudio Plater.cpp
            // 8627-8640 at 926a719); its slice is gated by Print::validate,
            // which checks each region against its own nozzle (OrcaSlicer
            // Print.cpp 1676-1690). Those findings are a warning here too, and
            // the slice's own Print::validate below stays the gate. Still
            // refused: what only the CLI check finds (the spiral-vase checks,
            // under_cli only: OrcaSlicer PrintConfig.cpp 10456-10458,
            // BambuStudio PrintConfig.cpp 9574-9576; the desktop asks the user
            // there), and what a command-line value brings (the desktop's
            // own fields refuse a value out of range as it is typed). The
            // baseline is the settings before any flag or override, with the
            // plate's own laid over: a finding there is the file's and only
            // warned; a finding the command line brings (on the flag's own
            // setting or another, as --nozzle-diameter makes a file's
            // bridge_line_width too wide) is refused.
            std::map<std::string, std::string> validity;   // refused
            std::map<std::string, std::string> noted;      // warned, as the desktop notice
            if (!found.empty()) {
                Slic3r::DynamicPrintConfig file_only(settings_before_flags);
                file_only.apply(plate_settings, true);
                const std::map<std::string, std::string> desktop = file_only.validate(false);
                for (const auto& [key, why] : found) {
                    const auto it = desktop.find(key);
                    // A setting given on the command line is the command
                    // line's value even when the file's own was out of range
                    // the same way: refused, as the desktop's field refuses it.
                    const bool given_as_flag = extra.has(key) || overrides.count(key) > 0;
                    (!given_as_flag && it != desktop.end() && it->second == why ? noted : validity)[key] = why;
                }
            }
            // Each plate's findings are its own: recorded once per plate
            // (every call of a --slice 0 run checks the plates again) and
            // given to that plate's result below.
            const int noted_plate = checked > 0 ? checked : 1;
            const bool noted_first_time = rs.noted.count(noted_plate) == 0;
            json& noted_warnings = rs.noted[noted_plate];
            noted_warnings = json::array();
            // Noted only when the run goes on: a refusal below says why it
            // does not, and the warning's "the run slices" would not hold.
            if (!noted.empty() && validity.empty() && unknown_values.empty()) {
                std::string sentence = "Invalid values found in the 3MF/config: ";
                json items = json::object();
                std::string flags;
                for (const auto& [key, why] : noted) {
                    sentence += (items.empty() ? "" : "; ") + key + ": " + why;
                    items[key] = why;
                    std::string flag = "--" + key;
                    std::replace(flag.begin(), flag.end(), '_', '-');
                    flags += (flags.empty() ? "" : ", ") + flag;
                }
                sentence += ". The run slices with them as they are, and the slice's own check of each "
                            "object against its nozzle and plate still applies. Give " + flags +
                            " a value in range to change it.";
                if (noted_first_time) {
                    const json announcement = {{"event","warning"},
                                               {"tag","InvalidValuesNoted"},
                                               {"plate_id", noted_plate},
                                               {"settings", items},
                                               {"message", sentence}};
                    // Held while the check pass runs every plate: it is
                    // announced once they all passed (announce_noted_warning).
                    if (rs.hold_noted)
                        rs.noted_held.push_back(announcement);
                    else
                        announce_noted_warning(announcement);
                }
                noted_warnings.push_back(json{{"message", sentence},
                                              {"level", "warning"},
                                              {"tag", "InvalidValuesNoted"}});
            }
            if (!validity.empty() || !unknown_values.empty()) {
                // One refusal naming every value at once: unknown enum
                // values first, then the engine's range findings.
#ifdef ENGINE_SNAPMAKER
                std::string sentence = "Snapmaker Orca cannot print this file as it is: ";
#elif defined(ENGINE_ORCA)
                std::string sentence = "OrcaSlicer cannot print this file as it is: ";
#else
                std::string sentence = "BambuStudio cannot print this file as it is: ";
#endif
                bool first = true;
                for (const std::string& u : unknown_values) {
                    sentence += (first ? "" : "; ") + u;
                    first = false;
                }
                json items = json::object();
                for (const auto& [key, why] : validity) {
                    sentence += (first ? "" : "; ") + key + ": " + why;
                    first = false;
                    items[key] = why;
                }
                sentence += ".";
                // The official sentence above stays the first part of the
                // message, so anything that matches on it still matches; the
                // caller is told how to get past the refusal (a command-line
                // override is applied on top of the file's value).
                if (!items.empty()) {
                    std::string flags;
                    for (auto it = items.begin(); it != items.end(); ++it) {
                        std::string flag = "--" + it.key();
                        std::replace(flag.begin(), flag.end(), '_', '-');
                        flags += (flags.empty() ? "" : ", ") + flag;
                    }
                    sentence += " Give " + flags + " a value in range to override it.";
                }
                sentence += " --allow-substitution lets the engine use its own default for a value it does"
                            " not know instead.";
                emit_event({{"event","config_refused"},
                            {"tag", unknown_values.empty() ? "InvalidValues" : "InvalidOrUnknownValues"},
                            {"settings", items},
                            {"unknown_values", unknown_items},
                            {"message", sentence}});
                std::cerr << "Param values in 3mf/config error: " << sentence << "\n";
                set_outcome_failure(outcome, CLI_INVALID_VALUES_IN_3MF, sentence);
                return 1;
            }
            }
            // This result's plate gets its own findings, once.
            take_noted_warnings(outcome, rs.noted, plate_id);
        }

        // A project on a printer with another bed: each plate's objects and
        // tower move onto it (move_to_new_bed; translate_models runs on every
        // plate, BambuStudio.cpp 4497-4659; OrcaSlicer.cpp 3885-3980), after
        // the settings given as flags apply (BambuStudio.cpp 4091; OrcaSlicer.cpp
        // 3542), so a flag's tower position or prime tower moves too. Each
        // plate moves in its own frame (PlateScope).
        if (project_facts.is_bbl_3mf && !assemble_input && !model.objects.empty()) {
            const int first = plate_id > 0 ? plate_id : 1;
            const int last  = plate_id > 0 ? plate_id : (int)rs.plates.size();
            for (int moved_plate = first; moved_plate <= last; ++moved_plate) {
                const int index = moved_plate - 1;
                RunPlate& run_plate = rs.plate(moved_plate);
                // The plate's own print sequence, else the project's
                // (get_print_sequence, BambuStudio.cpp 4403-4416).
                bool is_sequence = false;
                if (const auto* seq = config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence"))
                    is_sequence = seq->value == Slic3r::PrintSequence::ByObject;
                if (index < (int)plate_data.size() && plate_data[index] != nullptr)
                    if (const auto* own = plate_data[index]->config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence");
                        own && own->value != Slic3r::PrintSequence::ByDefault)
                        is_sequence = own->value == Slic3r::PrintSequence::ByObject;
                Slic3r::Vec3d moved;
                {
                    PlateScope scope(model, run_plate);
                    shrink_to_new_bed = move_to_new_bed(context, project_facts, settings_merge.machine_switch, model, config,
                                                        index, is_sequence, moved);
                }
                if (moved.x() != 0. || moved.y() != 0.) {
                    std::ostringstream text;
                    text << std::fixed << std::setprecision(2) << "The plate's objects and prime tower moved by (" << moved.x()
                         << ", " << moved.y() << ") mm onto this printer's bed";
                    emit_event({{"event","arranged"}, {"tag","MovedToNewBed"}, {"shrink_to_new_bed", shrink_to_new_bed},
                                {"message", text.str()}});
                    // The moved tower is the project's (the official moves it in
                    // m_print_config, which it exports, BambuStudio.cpp 4659,
                    // 8156-8157); the flags laid over the plate below
                    // (6902-6904) reach only the slice.
                }
            }
            // A plate moved onto the new printer also moves to its place in
            // the grid of the new bed size (compute_origin_using_new_size,
            // BambuStudio.cpp 4534; OrcaSlicer.cpp 3919; then reset_size):
            // translate_models moves every plate when the printer changes
            // (BambuStudio.cpp 4516) or, on Orca, when the bed changes
            // (OrcaSlicer.cpp 3902). The plate keeps the same place on itself.
#ifdef ENGINE_ORCA
            const bool translated = shrink_to_new_bed > 0;
#else
            const bool translated = settings_merge.machine_switch;
#endif
            const auto* new_area = config.option<Slic3r::ConfigOptionPoints>("printable_area");
            if (translated && new_area && new_area->values.size() >= 4 && !plate_data.empty())
                for (int index = 0; index < (int)rs.plates.size() && index < (int)plate_data.size(); ++index) {
                    RunPlate& run_plate = rs.plates[index];
                    const Slic3r::Vec2d new_origin =
                        plate_grid_origin(config, Slic3r::Semver(), index, (int)plate_data.size());
                    const Slic3r::Vec3d delta(new_origin.x() - run_plate.origin.x(), new_origin.y() - run_plate.origin.y(), 0.);
                    if (delta == Slic3r::Vec3d::Zero())
                        continue;
                    for (Slic3r::ModelObject* object : model.objects)
                        for (Slic3r::ModelInstance* inst : object->instances)
                            if (run_plate.is_member(inst))
                                inst->set_offset(inst->get_offset() + delta);
                    run_plate.origin = new_origin;
                }
        }

#ifdef ENGINE_BAMBU
        // Filament roster the project declares, captured before the BBS
        // normalization below reshapes any vector option.  It is taken from the
        // five identity keys alone, and it is the only count that stays the
        // project's: the plate overlay and the nozzle-map derivation further down
        // can rewrite filament_map, and the final alignment grows the per-filament
        // arrays to this count, so asking one of those arrays for its own length
        // later answers a different question.  The identity keys are themselves
        // never padded to the machine's nozzle count
        // (is_per_filament_config_key), so this is also the count the engine
        // derives once the config is applied.
        project_filaments = project_filament_count(config);

        // extruder_nozzle_stats: how many nozzles of each flow type each
        // extruder holds; automatic grouping reads it. The desktop writes it
        // into every project (NozzleInventory::export_to, NozzleInventory.cpp
        // 785-808 at v02.09.01.70, through full_config, PresetBundle.cpp 5061;
        // re-resolved on a printer change, select_printer 642-646), and a
        // named preset carries it (desktop_presets.cpp). Input without it
        // (settings files, an STL with --machine/--process/--filament) gets
        // the official CLI's: the printer's extruder_max_nozzle_count nozzles
        // of the nozzle_volume_type flow per extruder, a Hybrid slot taking
        // the preset's per-slot defaults (on_printer_model_change_cli,
        // BambuStudio.cpp 4536-4553), then each extruder switched to the
        // plate's flow type, the per-slot nozzle keys derived from the counts,
        // and the stats written to m_print_config on a printer with several
        // extruders or nozzles (BambuStudio.cpp 7088, 7170-7191;
        // new_nozzle_volume_type from 3804-3825). Stats the input carries are
        // read as they are (extruder_nozzle_stats_new first, 4496, 4555-4559)
        // unless the printer changed, when the official recomputes them from
        // the new printer (4530). A 2.8.x project carries the stats but none
        // of the per-slot keys, so the per-slot derivation below runs for
        // carried stats too (7173-7189).
        {
            const auto has_values = [](const Slic3r::ConfigOptionStrings* opt) {
                return opt != nullptr && std::any_of(opt->values.begin(), opt->values.end(),
                                                     [](const std::string& s) { return !s.empty(); });
            };
            const auto* stats_new = config.option<Slic3r::ConfigOptionStrings>("extruder_nozzle_stats_new");
            const auto* stats     = config.option<Slic3r::ConfigOptionStrings>("extruder_nozzle_stats");
            const Slic3r::ConfigOptionStrings* carried =
                rs.settings_merge.machine_switch ? nullptr : has_values(stats_new) ? stats_new : has_values(stats) ? stats : nullptr;
            const auto* max_opt = config.option<Slic3r::ConfigOptionIntsNullable>("extruder_max_nozzle_count");
            const auto* nozzles = config.option<Slic3r::ConfigOptionFloats>("nozzle_diameter");
            if (max_opt != nullptr && !max_opt->values.empty() && nozzles != nullptr) {
                const std::vector<int> max_nozzle_count = max_opt->values;
                const size_t slot_count = max_nozzle_count.size();
                const int new_extruder_count = int(nozzles->values.size());
                const auto* opt_nvt = dynamic_cast<const Slic3r::ConfigOptionEnumsGeneric*>(config.option("nozzle_volume_type"));
                Slic3r::ExtruderNozzleStat nozzle_stats_obj;
                if (carried != nullptr) {
                    // BambuStudio.cpp 4555-4559.
                    nozzle_stats_obj.set_raw_stat(Slic3r::get_extruder_nozzle_stats(carried->values));
                } else {
                    // BambuStudio.cpp 4536-4553.
                    std::vector<int> curr_volume_map_value(slot_count, static_cast<int>(Slic3r::NozzleVolumeType::nvtStandard));
                    if (opt_nvt && !opt_nvt->values.empty())
                        for (size_t idx = 0; idx < slot_count; ++idx)
                            curr_volume_map_value[idx] = idx < opt_nvt->values.size() ? opt_nvt->values[idx] : opt_nvt->values.back();
                    nozzle_stats_obj.on_printer_model_change_cli(curr_volume_map_value, max_nozzle_count);
                    // Hybrid only summarises mixed slots, so take the real mix
                    // from the preset's per-slot defaults (4544-4553).
                    for (size_t eid = 0; eid < slot_count; ++eid) {
                        if (curr_volume_map_value[eid] != int(Slic3r::NozzleVolumeType::nvtHybrid))
                            continue;
                        bool first = true;
                        for (const auto& [vt, cnt] : Slic3r::count_default_nozzle_volume_types(config, int(eid), max_nozzle_count[eid])) {
                            nozzle_stats_obj.set_extruder_nozzle_count(int(eid), vt, cnt, first);
                            first = false;
                        }
                    }
                }
                // BambuStudio.cpp 3804-3825: the flow type per extruder.
                std::vector<Slic3r::NozzleVolumeType> new_nozzle_volume_type;
                if (opt_nvt && opt_nvt->values.size() >= static_cast<size_t>(new_extruder_count)) {
                    for (int i = 0; i < new_extruder_count; i++)
                        new_nozzle_volume_type.push_back(Slic3r::NozzleVolumeType(opt_nvt->values[i]));
                } else {
                    if (!rs.settings_merge.machine_switch)
                        for (int v : project_facts.current_nozzle_volume_type)
                            new_nozzle_volume_type.push_back(Slic3r::NozzleVolumeType(v));
                    new_nozzle_volume_type.resize(new_extruder_count, Slic3r::NozzleVolumeType::nvtStandard);
                }
                // BambuStudio.cpp 3804 and 7088, 7170-7191.
                const bool support_multi_nozzle = std::any_of(max_nozzle_count.begin(), max_nozzle_count.end(),
                                                              [](int v) { return v > 1; });
                if (new_extruder_count > 1 || support_multi_nozzle) {
                    // An input without the full per-slot layout (extruder_nozzle_count,
                    // _volume_type, _diameter, _slot) gets it from the counts: a
                    // single-extruder nozzle rack keeps its count of the target
                    // flow type, any other extruder keeps its total and takes the
                    // plate's flow type (7173-7189).
                    // The rack the command line describes wins over the input's
                    // (7103-7172, cli_nozzle_options.cpp).
                    const bool from_flags = slicer_cli::command_line_nozzle_rack(extra, config, new_extruder_count, nozzle_stats_obj);
                    slicer_cli::sync_ams_count_with_command_line(extra, config);
                    // A project moved to another printer keeps its nozzles
                    // only where they hold the printer's diameter, as the
                    // app keeps them (cli_nozzle_options.hpp). The official
                    // command line keeps a complete per-slot layout from the
                    // file as it is (BambuStudio.cpp 7173-7174). The
                    // printer changes whenever the run names another one;
                    // machine_switch is set only when the project's process
                    // does not suit it.
                    const bool printer_changed = !rs.settings_merge.new_printer_name.empty() &&
                                                 rs.settings_merge.new_printer_name != project_facts.current_printer_name;
                    if (!from_flags && printer_changed &&
                        slicer_cli::fit_project_nozzles_to_printer(config))
                        emit_event({{"event","config_normalized"}, {"tag","ProjectNozzlesFitToPrinter"},
                                    {"message","The project's nozzles do not hold the new printer's diameter; "
                                               "those extruders take the printer's default nozzles, as the app does"}});
                    const bool derived = !from_flags && !Slic3r::per_slot_nozzle_options_consistent(config);
                    if (derived) {
                        for (size_t eid = 0; eid < new_nozzle_volume_type.size(); ++eid) {
                            if (Slic3r::is_single_ext_multi_nozzle(config))
                                nozzle_stats_obj.on_volume_type_switch(int(eid), new_nozzle_volume_type[eid]);
                            else if (new_nozzle_volume_type[eid] != Slic3r::NozzleVolumeType::nvtHybrid)
                                nozzle_stats_obj.set_extruder_nozzle_count(int(eid), new_nozzle_volume_type[eid],
                                                                           nozzle_stats_obj.get_extruder_nozzle_count(int(eid), std::nullopt), true);
                        }
                        const Slic3r::ExtruderFullStat full_stat = Slic3r::build_extruder_full_stat_from_nozzle_stat(
                            nozzle_stats_obj, max_nozzle_count, config_id_text(config, "printer_variant"));
                        Slic3r::sync_per_slot_nozzle_options_from_full_stat(config, full_stat);
                    }
                    // The official re-serialises the stats every time (7191), so
                    // a project whose newer extruder_nozzle_stats_new differs
                    // from its older key (a rack with a TPU nozzle) slices with
                    // the newer one.
                    config.option<Slic3r::ConfigOptionStrings>("extruder_nozzle_stats", true)->values =
                        Slic3r::save_extruder_nozzle_stats_to_string(nozzle_stats_obj.get_raw_stat());
                }
            }
        }
#endif

#ifdef ENGINE_BAMBU
        // Provenance is accumulated while each input/profile layer is parsed,
        // before normalization pads vector options. A supplied [1] is explicit
        // even when it equals the driver's seed value.
        if (verbose)
            std::cout << "Nozzle-map provenance: explicit config map="
                      << (explicit_config_supplied_nozzle_map ? "yes" : "no") << "\n";

        // ── BBS toolchanger / per-extruder normalizations ───────────────────
        // Everything from here to the matching #endif exists to make the Bambu
        // engine accept a non-Bambu printer config (vector-array padding/collapse,
        // explicit nozzle-map derivation, master-nozzle reassignment, prime-tower
        // disable heuristics).  OrcaSlicer handles all of this natively, so the
        // Orca driver skips the whole region and slices from the flat 3MF config.
        //
        // Pad empty vector config options and preserve 3MF multi-element values.
        //
        // The 3MF project_settings.config contains per-extruder arrays from the
        // printer profile (e.g. machine_max_acceleration_e = ["5000","5000"]).
        // BambuStudio desktop keeps these intact through Print::apply() which
        // processes them via update_values_to_printer_extruders.  The CLI must
        // NOT truncate them — the pipeline handles multi-element arrays via
        // get_at() which clamps to the last element for single-filament prints.
        //
        // Pad empty vectors to extruder_count (from nozzle_diameter size).
        {
            // Determine actual extruder count from nozzle_diameter vector.
            size_t extruder_count = 1;
            if (auto* nd = config.option<Slic3r::ConfigOptionFloats>("nozzle_diameter", false)) {
                if (!nd->values.empty())
                    extruder_count = nd->values.size();
            }

            // Keys that must NOT be touched (polygon/group semantics)
            static const std::unordered_set<std::string> skip_pad = {
                "printable_area",           // bed shape polygon (4+ points)
                "bed_exclude_area",         // exclusion zones
                "thumbnails",
                "extruder_printable_area",  // per-extruder bed polygons

                // The two per-filament INDEX maps.  is_per_filament_config_key()
                // returns false for them — they are the "special" spellings that
                // get their own alignment, not plain value arrays — so the
                // predicate below does not reach them on its own.  They are per
                // filament all the same and must not be padded to the nozzle
                // count: padding is exactly how a one-filament project on a
                // two-nozzle machine grows filament_map to [1, 0], whose LENGTH
                // the engine then reads as two filaments and fans every
                // per-filament array out to (PrintConfig.cpp:8134).
                // align_per_filament_maps() extends and repairs them against the
                // roster instead.  (filament_nozzle_map, the third index map, is a
                // plain per-filament key, so the predicate already covers it.)
                "filament_map",
                "filament_map_2",

                // A list of script paths, not a value per extruder. Padding it
                // names an empty script, and a project exported with that entry
                // is refused by the official command line's normative check
                // (BambuStudio.cpp 1948-1956).
                "post_process",

                // Not one value per extruder either; the official CLI leaves
                // each as the settings give it, and a padded 0 is read:
                //  - wipe_tower_x / wipe_tower_y: one entry per plate, read
                //    with get_at(plate index) (Print.cpp 998, 2213, 2662),
                //    which falls back to the first entry (Config.hpp 681-685). A 0 put plate
                //    2's tower at (0, 0).
                //  - flush_multiplier / flush_multiplier_fast: one per nozzle;
                //    when the official sizes them it fills 1.0 / 1.2
                //    (BambuStudio.cpp 3864-3868; OrcaSlicer.cpp 3360-3361),
                //    and get_at() gives a short list's first value. A 0 is no
                //    flush on the second nozzle.
                //  - first_layer_print_sequence / other_layers_print_sequence:
                //    filament orders; more than one entry is read as a custom
                //    first-layer order (ToolOrdering.cpp 467-468, 597-610).
                "wipe_tower_x",
                "wipe_tower_y",
                "flush_multiplier",
                "flush_multiplier_fast",
                "first_layer_print_sequence",
                "other_layers_print_sequence",
            };

            // Per-filament arrays are excluded from this padding altogether: the
            // plain ones by the same predicate the final alignment uses, the two
            // index maps by skip_pad above.  Their LENGTH is the
            // engine's filament count, not the head count: Print::apply() reads its
            // num_extruders from filament_diameter (PrintApply.cpp:1438),
            // ToolOrdering sizes its filament loops and the per-nozzle flush
            // geometry from filament_colour / filament_diameter (ToolOrdering.cpp:509,
            // 2503; prepare_flush_matrices:1421), Print::has_wipe_tower()
            // (Print.cpp:3113) tests filament_diameter.size() > 1 directly, and the
            // multi-filament fan-out reads filament_map's length
            // (PrintConfig.cpp:8134).  Padding a one-filament project's
            // per-filament arrays up to a two-nozzle machine's nozzle count
            // invents a filament slot the project never declared — with a zero
            // diameter, no colour and no type — and everything sized from that
            // count downstream (wipe tower, per-nozzle flush matrix, G-code
            // metadata) is then sized for it.  Left at their declared length, the
            // engine sees the roster the project actually loaded, and the final
            // central alignment extends them to exactly that count.
            //
            // A zero spliced in here would also be sticky: the alignment only ever
            // duplicates the FRONT value, so a padded [22, 0] becomes [22, 0, 22]
            // for a three-filament roster instead of [22, 22, 22].
            //
            // The variant-slot arrays (filament_extruder_variant,
            // filament_self_index) are not per-filament and keep the extruder-count
            // padding they have always had — they belong to the printer-side slot
            // table (PrintConfig.cpp:7592).
            //
            // Only pad empty vectors to extruder_count; do NOT truncate.
            // Variant-expanded arrays (retraction_length, machine_max_*, speed, etc.)
            // are collapsed by PrintApply.cpp's update_values_to_printer_extruders()
            // when (extruder_count > 1) || different_extruder.
            // support_different_extruders() returns true when extruder_variant_list
            // has multiple comma-separated variants (e.g. "Direct Drive Standard,
            // Direct Drive High Flow"), which is the normal case for BBL printers.
            // Truncating here would bypass that collapsing and produce wrong sizes.
            for (const auto& key : config.keys()) {
                if (skip_pad.count(key))
                    continue;

                // Every filament setting (is_filament_setting_key, which holds
                // all the keys the final alignment treats as a roster) is left
                // alone, so this pass can never pad an array the alignment then
                // treats as a roster.  The index maps the predicate excludes are
                // in skip_pad above.
                if (is_filament_setting_key(key))
                    continue;

                auto* opt = config.option(key, false);
                if (!opt) continue;

#define NORM_VEC(Type, zero_val) \
                if (auto v = dynamic_cast<Slic3r::Type*>(opt)) { \
                    while (v->values.size() < extruder_count) v->values.push_back(zero_val); \
                } else
                NORM_VEC(ConfigOptionBoolsNullable,          (unsigned char)0)
                NORM_VEC(ConfigOptionBools,                  (unsigned char)0)
                NORM_VEC(ConfigOptionIntsNullable,           0)
                NORM_VEC(ConfigOptionInts,                   0)
                NORM_VEC(ConfigOptionFloatsNullable,         0.0)
                NORM_VEC(ConfigOptionFloats,                 0.0)
                NORM_VEC(ConfigOptionPercentsNullable,       0.0)
                NORM_VEC(ConfigOptionPercents,               0.0)
                NORM_VEC(ConfigOptionFloatsOrPercentsNullable, (Slic3r::FloatOrPercent{0.0, false}))
                NORM_VEC(ConfigOptionFloatsOrPercents,       (Slic3r::FloatOrPercent{0.0, false}))
                NORM_VEC(ConfigOptionStrings,                std::string{})
                NORM_VEC(ConfigOptionEnumsGenericNullable,   0)
                NORM_VEC(ConfigOptionEnumsGeneric,           0)
                { /* ConfigOptionPoints, ConfigOptionPointsGroups, etc. — leave alone */ }
#undef NORM_VEC
            }

            // extruder_variant_list contains comma-separated variant names per slot
            // (e.g. "Direct Drive Standard,Direct Drive High Flow").
            // support_different_extruders() splits these and detects multiple variants,
            // triggering update_values_to_printer_extruders() in PrintApply.cpp.
            // Do NOT truncate this list — it must remain as-is for correct collapsing.

            // Clamp master_extruder_id to extruder_count (1-indexed).
            // ToolOrdering::get_recommended_filament_maps() uses master_extruder_id-1
            // as an index into nozzle_list, which has extruder_count entries.
            if (auto* mid = config.option<Slic3r::ConfigOptionInt>("master_extruder_id", false)) {
                if (mid->value < 1) mid->value = 1;
                if (mid->value > (int)extruder_count) mid->value = (int)extruder_count;
            }
        }

        // Ensure vector options after loading JSON configs and forcing single-extruder
        // JSON configs may have left some vectors empty or missing
        ensure_vector_config_sizes(config);
#endif // ENGINE_BAMBU — the run's settings base

        // The run's settings base, m_print_config (see RunState): everything
        // above runs once per run.
        rs.base = config;
        rs.loaded = true;
        } // !rs.loaded

        // This plate's settings: a copy of the base with the plate's own and
        // the flags laid on top (BambuStudio.cpp 6902-6904; OrcaSlicer.cpp
        // 5905-5907). The plate the run-level steps ran for keeps the settings
        // those steps read and changed (the arrange's tower).
        const bool fresh_plate_config = !(rs.prepared && plate_id == rs.prepared_plate);
        config = fresh_plate_config ? rs.base : rs.run_config;
        // --export-settings writes the project's settings, not this plate's:
        // the official action saves m_print_config before the plate loop
        // applies *part_plate->config() (BambuStudio.cpp 6366-6370 and
        // 6902-6904; OrcaSlicer.cpp 5499-5503 and 5905-5907).
        if (slicer_cli::has_model_actions(o))
            project_settings = rs.base;
        if (fresh_plate_config) {
        // The prepare call of a run of several plates lays no plate's own
        // settings: its settings serve the run-level steps only, and each
        // plate reports its own when it is sliced.
        HeldEventsScope run_config_events(outcome.prepare_pass && plate_id == 0 &&
                                          (trailing_port || plate_data.size() > 1));
#ifdef ENGINE_BAMBU
        // If the fully-overlaid config carries an explicit per-filament nozzle assignment,
        // apply it directly to config's filament_map.  This reproduces the
        // BambuStudio desktop behaviour where the user has already constrained the
        // mapping in the GUI ("Nozzle Manual" mode) and saved it in the plate metadata.
        //
        // Explicit is detected when EITHER:
        //   (a) plate filament_maps have multiple distinct values (e.g. [2,1]), OR
        //   (b) the project config says "Nozzle Manual" (handles uniform maps like [2,2])
        //
        // When the maps are uniform AND mode is "Auto For Flush" (e.g. [1,1]),
        // we fall through to apply_explicit_nozzle_mapping() which re-derives the map
        // from filament_nozzle_map using the physical_extruder_map inverse.
        int plate_data_idx = (plate_id > 0 && (int)plate_data.size() >= plate_id) ? plate_id - 1 : 0;
        bool explicit_plate_mapping_applied = false;
        // --filament-map-mode and --filament-map (apply_filament_map_flag).
        // --filament-map alone is Manual, not Nozzle Manual, on the H2C as
        // well: Manual groups by filament_map and checks the result against it
        // (ToolOrdering.cpp 2653-2680 at v02.09.01.70), Nozzle Manual reads
        // filament_nozzle_map instead (2565-2623).
        bool cli_mode_given = false;
        switch (apply_filament_map_flag(o, extra, config, rs, project_settings, project_filaments, plate_id, outcome)) {
        case FilamentMapFlag::Refused:
            return 1;
        case FilamentMapFlag::Applied: {
            // filament_map_2, the 0-based mirror, follows the map.
            const std::vector<int>& pm = config.option<Slic3r::ConfigOptionInts>("filament_map")->values;
            auto* fm2 = config.option<Slic3r::ConfigOptionInts>("filament_map_2", true);
            fm2->values.resize(pm.size());
            for (size_t i = 0; i < pm.size(); ++i)
                fm2->values[i] = pm[i] - 1;
            explicit_plate_mapping_applied = true;
            cli_mode_given = true;
            break;
        }
        case FilamentMapFlag::ModeOnly:
            cli_mode_given = true;
            break;
        case FilamentMapFlag::NotGiven:
            break;
        }
        if (!explicit_plate_mapping_applied && !plate_data.empty() && plate_data[plate_data_idx] != nullptr) {
            const auto& pm = plate_data[plate_data_idx]->filament_maps;
            // The plate's own mode, else the project's
            // (PartPlate::get_real_filament_map_mode, PartPlate.cpp 278-289;
            // BambuStudio.cpp 6661-6665), applied over the print config as the
            // official CLI applies the plate config (BambuStudio.cpp 6905-6907).
            // The stored plate map is used only in a manual mode: under an
            // automatic mode the engine groups the filaments itself
            // (ToolOrdering.cpp 1897-1914), whatever map an earlier slice saved.
            Slic3r::FilamentMapMode real_mode = Slic3r::FilamentMapMode::fmmAutoForFlush;
            if (auto* mode_opt = config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode", false))
                real_mode = mode_opt->value;
            if (auto* plate_mode = plate_data[plate_data_idx]->config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode");
                !cli_mode_given && plate_mode && plate_mode->value != Slic3r::FilamentMapMode::fmmDefault) {
                real_mode = plate_mode->value;
                config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode", true)->value = real_mode;
            }
            const bool is_manual_mode = real_mode == Slic3r::FilamentMapMode::fmmManual ||
                                        real_mode == Slic3r::FilamentMapMode::fmmNozzleManual;
            // Any saved map in a manual mode, one slot included: the official
            // CLI slices a manual plate with get_real_filament_maps(), the
            // plate's own map whenever it has one (BambuStudio.cpp 6751;
            // PartPlate.cpp 266-276), whatever its length.
            bool has_explicit = !pm.empty() && is_manual_mode;
            if (has_explicit) {
                auto* fm = config.option<Slic3r::ConfigOptionInts>("filament_map", true);
                fm->values = pm;  // already 1-based per PlateData docs
                // The plate's nozzle-volume map with it, as the official
                // plate config carries both (bbs_3mf.cpp 4612-4624).
                if (const auto* pvm = plate_data[plate_data_idx]->config.option<Slic3r::ConfigOptionInts>("filament_volume_map");
                    pvm && pvm->values.size() == pm.size())
                    config.option<Slic3r::ConfigOptionInts>("filament_volume_map", true)->values = pvm->values;

                // Also sync filament_map_2 (0-based mirror used by some code paths)
                auto* fm2 = config.option<Slic3r::ConfigOptionInts>("filament_map_2", true);
                fm2->values.resize(pm.size());
                for (size_t i = 0; i < pm.size(); ++i)
                    fm2->values[i] = pm[i] - 1;

                // `Nozzle Manual` drives the multi-nozzle grouping branch.  H2D is
                // instead a multi-extruder machine (one nozzle per extruder), where
                // `Manual` is the branch that preserves this already-resolved plate
                // map through ToolOrdering.  Derive the mode from the configured
                // topology rather than a printer identity.
                bool has_multiple_nozzles_per_extruder = false;
                if (auto* counts = config.option<Slic3r::ConfigOptionInts>("extruder_max_nozzle_count", false)) {
                    has_multiple_nozzles_per_extruder = std::any_of(
                        counts->values.begin(), counts->values.end(),
                        [](int count) { return count > 1; });
                }
                // The command line's mode stays as given.
                Slic3r::ConfigSubstitutionContext subst(Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
                if (!cli_mode_given)
                    config.set_deserialize(
                        "filament_map_mode",
                        has_multiple_nozzles_per_extruder ? "Nozzle Manual" : "Manual",
                        subst);
                explicit_plate_mapping_applied = true;
            }
        }

        // Fix the cardinality of the per-filament maps BEFORE the derivation reads
        // them.  apply_explicit_nozzle_mapping() takes filament_map's own length as
        // the filament count, so a map still holding the driver's single seed entry
        // — a project whose filament_map is a placeholder while its
        // filament_nozzle_map carries the real cross-nozzle split — would look like
        // a one-filament project and skip a routing the file declares.  This is the
        // same helper the final alignment calls, which is what keeps the maps
        // cardinality-correct after the plate overlay above and the command-line
        // overrides below have had their chance to rewrite them.
        align_per_filament_maps(config, project_filaments);

        // The selected plate already expresses physical routing when it carries
        // a diverse map. Do not re-derive that routing from the general
        // filament_nozzle_map fallback, which is for an otherwise-auto mapping.
        // Nor when the command line names an automatic mode: the engine
        // groups the filaments itself, and a manual map derived here would
        // be the one an exported project states.
        bool cli_automatic_mode = false;
        if (cli_mode_given)
            if (const auto* mode = config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode"))
                cli_automatic_mode = mode->value != Slic3r::FilamentMapMode::fmmManual &&
                                     mode->value != Slic3r::FilamentMapMode::fmmNozzleManual;
        bool nozzle_mapping_derived =
            (explicit_plate_mapping_applied || cli_automatic_mode) ? false : apply_explicit_nozzle_mapping(
                config, explicit_config_supplied_nozzle_map, verbose);

        // The derived routing is the plate's (filament_map + its mode); every
        // object keeps its own filament, as in Bambu Studio and OrcaSlicer,
        // which never move an object to another filament for routing.
        if (nozzle_mapping_derived) {
            // --export-3mf reloads the file, which knows nothing of this
            // routing: keep it so the plate states what the slice used.
            if (o.slice_mode && !o.export_3mf.empty()) {
                outcome.derived_filament_map = config.option<Slic3r::ConfigOptionInts>("filament_map", true)->values;
                outcome.derived_filament_map_mode = config.opt_serialize("filament_map_mode");
            }
        } else if (verbose) {
            const auto* mode = config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>(
                "filament_map_mode", false);
            const std::string mode_name = mode ? mode->serialize() : "<unset>";
            std::cout << "Nozzle-map derivation: skipped mode=" << mode_name << "\n";
        }

        // Disable prime tower when there is no actual multi-material printing.
        //
        // BambuStudio (BambuStudio.cpp:3885-3913) disables prime tower when
        // all loaded filament presets are the same AND all filament colors are
        // the same.  The GUI also disables it when all filament slots map to the
        // same physical extruder (filament_map all equal) and all colors match.
        //
        // A prime tower with no tool changes (ToolOrdering::has_wipe_tower()==false)
        // causes Print::has_wipe_tower()==true to take a different code path in
        // psWipeTower: it builds m_wipe_tower_data.tool_ordering but leaves
        // m_tool_ordering empty.  psSkirtBrim then reads the empty m_tool_ordering,
        // producing initial_extruder_id==-1 and wrong path ordering.  Disabling
        // enable_prime_tower forces the correct code path.
        {
            auto* ept = config.option<Slic3r::ConfigOptionBool>("enable_prime_tower", false);
            if (ept && ept->value) {
                bool disable = false;

                // Single filament slot: trivially no multi-material.  This asks
                // the roster the project declared (filament_colour /
                // settings_id / ids / type / diameter), in the snapshot taken
                // from project_filament_count() before any normalization ran —
                // not one array's own current length, which the alignment below
                // has already grown to that same count.  A project that declares
                // no identity key at all still reads as zero here, not as one.
                if (project_filaments <= 1)
                    disable = true;

                // Multiple slots: disable when all filament colors are the same
                // AND all filament_map values point to the same extruder.
                // This covers: same-preset AMS prints (tests 20-23, 18-19),
                // same-extruder different-preset prints (tests 24-25), etc.
                // Multi-color AMS prints (test 10) have different colors → kept enabled.
                if (!disable) {
                    auto* fc = config.option<Slic3r::ConfigOptionStrings>("filament_colour", false);
                    auto* fm = config.option<Slic3r::ConfigOptionInts>("filament_map", false);
                    bool all_same_color = fc && !fc->values.empty() &&
                        std::all_of(fc->values.begin(), fc->values.end(),
                                    [&](const std::string& c){ return c == fc->values[0]; });
                    bool all_same_extruder = fm && !fm->values.empty() &&
                        std::all_of(fm->values.begin(), fm->values.end(),
                                    [&](int v){ return v == fm->values[0]; });
                    if (all_same_color && all_same_extruder)
                        disable = true;
                }

                if (disable)
                    ept->value = false;
            }
        }
#endif // ENGINE_BAMBU — BBS toolchanger / per-extruder normalizations

        // Toolchanger filament-map (U1/Prusa-XL class) on the Orca build: the
        // flat 3MF stores filament_map_mode="Auto For Flush" with a placeholder
        // filament_map. With is_BBL_printer() initialized deterministically (see
        // below, next to print construction) the engine's native Auto-For-Flush
        // resolution computes the per-filament→tool map from the per-volume
        // extruders we already load, matching golden B — so no driver-side
        // filament_map injection is needed here. Model per-extruder static
        // tables (setExtruderParams/setPrintSpeedTable) are still set after
        // apply(), exactly as Orca's own headless CLI does.

        // The selected plate's own settings over the project's, then the
        // command line over both: the official CLI slices each plate with
        // new_print_config = m_print_config; apply(*part_plate->config());
        // apply(m_extra_config) (BambuStudio.cpp 6902-6904 at 5873b5f;
        // OrcaSlicer.cpp 5905-5907 at 31f6803). A plate states bed type,
        // print sequence, first/other layer order, spiral mode and the
        // filament-switcher flags (bbs_3mf.cpp 4563-4728 / Orca 4419-4563).
        // The filament-map keys are not taken here: on the Bambu build the
        // plate block above applies the plate's mode and, in a manual mode,
        // its maps (an automatic mode regroups whatever map was saved,
        // ToolOrdering.cpp 1897-1914); on the Orca build the block just
        // below does the same. Only with a plate named (--plate N or
        // --slice): a whole-file load is not one plate.
        // The plate wins over the --config/--machine/--process/--filament
        // files: they are the official --load_settings/--load_filaments,
        // which merge into m_print_config (update_full_config, BambuStudio.cpp
        // 3246, 3384; OrcaSlicer.cpp 2781, 2925) before the plate is laid over
        // it. The command-line overrides (m_extra_config) come last.
#ifdef ENGINE_ORCA
        // The plate's filament-map keys on the Orca build. The official Orca
        // CLI lays the whole plate config over the project's
        // (OrcaSlicer.cpp 5905-5907), mode and map included. Here: the
        // plate's mode when it states one (PartPlate::get_real_filament_map_mode,
        // fmmDefault defers to the project), and in the manual mode the
        // plate's own map (get_real_filament_maps, the map whenever the plate
        // has one) and nozzle-volume map. An automatic mode keeps the
        // engine's native Auto-For-Flush resolution with the project's map,
        // as before (the U1 toolchanger case).
        bool cli_filament_map_applied = false, cli_mode_given = false;
#ifndef ENGINE_SNAPMAKER
        // --filament-map-mode and --filament-map win over the plate's mode and
        // map (apply_filament_map_flag).
        switch (apply_filament_map_flag(o, extra, config, rs, project_settings, project_filament_count(config), plate_id,
                                        outcome)) {
        case FilamentMapFlag::Refused:  return 1;
        case FilamentMapFlag::Applied:  cli_filament_map_applied = cli_mode_given = true; break;
        case FilamentMapFlag::ModeOnly: cli_mode_given = true; break;
        case FilamentMapFlag::NotGiven: break;
        }
#endif
        if (!cli_filament_map_applied && plate_id > 0 && (int)plate_data.size() >= plate_id && plate_data[plate_id - 1] != nullptr) {
            const Slic3r::PlateData& plate = *plate_data[plate_id - 1];
            Slic3r::FilamentMapMode real_mode = Slic3r::FilamentMapMode::fmmAutoForFlush;
            if (auto* mode_opt = config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode", false))
                real_mode = mode_opt->value;
            if (auto* plate_mode = plate.config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode");
                !cli_mode_given && plate_mode && plate_mode->value != Slic3r::FilamentMapMode::fmmDefault) {
                real_mode = plate_mode->value;
                config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode", true)->value = real_mode;
            }
#ifndef ENGINE_SNAPMAKER
            // PlateData::filament_maps is OrcaSlicer 2.4 only; Snapmaker's
            // PlateData has no per-plate map at bac6fd5, so a plate's own map
            // cannot be applied.
            if (real_mode == Slic3r::FilamentMapMode::fmmManual && !plate.filament_maps.empty()) {
                config.option<Slic3r::ConfigOptionInts>("filament_map", true)->values = plate.filament_maps;
                if (const auto* pvm = plate.config.option<Slic3r::ConfigOptionInts>("filament_volume_map");
                    pvm && pvm->values.size() == plate.filament_maps.size())
                    config.option<Slic3r::ConfigOptionInts>("filament_volume_map", true)->values = pvm->values;
                emit_event({{"event","config_normalized"}, {"tag","PlateFilamentMapApplied"},
                            {"plate_id", plate_id}, {"filament_map", plate.filament_maps},
                            {"message","Plate " + std::to_string(plate_id) +
                                       " is in Manual mode: its own filament map applies"}});
            }
#endif // ENGINE_SNAPMAKER
        }
#endif
        {
            const Slic3r::DynamicPrintConfig plate_config = plate_own_settings(plate_data, plate_id);
            if (!plate_config.empty()) {
                config.apply(plate_config, true);
                emit_event({{"event","config_normalized"}, {"tag","PlateSettingsApplied"},
                            {"plate_id", plate_id}, {"keys", plate_config.keys()},
                            {"message","Applied plate " + std::to_string(plate_id) + "'s own settings: " +
                                       boost::algorithm::join(plate_config.keys(), ", ")}});
            }
        }

        // The settings given as flags over the plate's own (the official
        // apply(m_extra_config) after apply(*part_plate->config()),
        // BambuStudio.cpp 6902-6904, OrcaSlicer.cpp 5905-5907).
        if (!extra.empty())
            config.apply(extra, true);
        // Apply command-line overrides
        apply_command_line_overrides(config, overrides, /*report_rejections=*/true);
#ifdef ENGINE_BAMBU
        // A manual map, the command line's or the plate's, that puts a filament
        // on an extruder it cannot print with (BambuStudio.cpp 7357-7400).
        // A --slice run checks it after the plate's object exclusions
        // (--skip-objects), with the bed checks below.
        if (!o.slice_mode && !manual_map_extruders_fit(config, model, plate_id, outcome))
            return 1;
#endif
#ifdef ENGINE_ORCA
        // A Bambu Studio project's own start G-code can carry the maker's
        // placeholders, and this engine binds no value for two of them: its
        // PlaceholderParser sets initial_extruder and reads nozzle_diameter
        // from the config, while BambuStudio also sets
        // nozzle_diameter_at_nozzle_id and initial_nozzle_id (BambuStudio
        // GCode.cpp 2656-2660 at 5873b5f; upstream main binds them with its
        // multi-nozzle engine, GCode.cpp 3334-3335 on main). Unbound, the
        // parser throws at export, after the whole run has sliced
        // ("Not a variable name"): refused here, before slicing, naming the
        // placeholder and the value to use instead. The settings are the ones
        // the run slices with, overrides and all.
        {
            std::string       placeholder_key;
            const std::string placeholder = bambu_only_gcode_placeholder(config, placeholder_key);
            if (!placeholder.empty()) {
                const std::string instead = placeholder == "initial_nozzle_id"
                                                ? "{initial_extruder}"
                                                : "{nozzle_diameter[initial_extruder]}";
                const std::string sentence =
                    "The project's " + placeholder_key + " uses {" + placeholder +
                    "}, which this engine has no value for; use " + instead + " instead (OrcaSlicer binds "
                    "the initial extruder as {initial_extruder} and the nozzle diameters as {nozzle_diameter}).";
                emit_event({{"event","config_refused"},
                            {"tag","BambuOnlyGcodePlaceholder"},
                            {"opt_key", placeholder_key},
                            {"placeholder", placeholder},
                            {"replace_with", instead},
                            {"message", sentence}});
                std::cerr << "Error: " << sentence << "\n";
                set_outcome_failure(outcome, CLI_INVALID_VALUES_IN_3MF, sentence);
                return 1;
            }
        }
#endif

        // Display active settings
        std::cout << "\nActive print settings:\n";
        if (config.has("layer_height")) {
            std::cout << "  Layer height: " << config.opt_float("layer_height") << "mm\n";
        }
        if (config.has("perimeters")) {
            std::cout << "  Perimeters: " << config.opt_int("perimeters") << "\n";
        }
        if (config.has("sparse_infill_density")) {
            auto percent_opt = config.option<Slic3r::ConfigOptionPercent>("sparse_infill_density");
            if (percent_opt) {
                std::cout << "  Infill: " << percent_opt->value << "%\n";
            }
        }
        if (config.has("nozzle_diameter")) {
            auto nozzles = config.option<Slic3r::ConfigOptionFloats>("nozzle_diameter");
            if (nozzles && !nozzles->values.empty()) {
                std::cout << "  Nozzle: " << nozzles->values[0] << "mm\n";
            }
        }
        run_config_events.drop();
        } // fresh_plate_config
        // A plate sliced after the run-level steps ran for every plate: the
        // towers those steps placed (the official's m_print_config after its
        // arrange, BambuStudio.cpp 5693-5700, 5856; OrcaSlicer.cpp 4954-4961).
        if (fresh_plate_config && rs.prepared)
            for (const char* key : {"wipe_tower_x", "wipe_tower_y"})
                if (const Slic3r::ConfigOption* opt = rs.run_config.option(key))
                    config.set_key_value(key, opt->clone());

        // A plate's print sequence: its own, or else the project's
        // (get_print_sequence, BambuStudio.cpp 4403-4416), for the plates
        // other than this one (--downward-check, and the whole project of
        // the model actions for --slice 0).
        std::optional<bool> project_sequence_read;
        const auto project_sequence = [&]() {
            if (project_sequence_read)
                return *project_sequence_read;
            std::string project_seq;
            if (const Slic3r::ConfigOption* opt = extra.option("print_sequence"))
                project_seq = opt->serialize();
            else if (!merged_print_sequence.empty())
                // --load-settings and friends: the merged settings
                // (m_print_config, BambuStudio.cpp 4785).
                project_seq = merged_print_sequence;
            else if (assemble_input) {
                if (const auto* seq = config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence"))
                    project_seq = seq->serialize();
            } else {
                std::string text;
                if (read_zip_member(input_file, "Metadata/project_settings.config", text)) {
                    try {
                        const json settings = json::parse(text);
                        if (settings.contains("print_sequence") && settings["print_sequence"].is_string())
                            project_seq = settings["print_sequence"].get<std::string>();
                    } catch (...) {
                    }
                }
            }
            project_sequence_read = project_seq == "by object";
            return *project_sequence_read;
        };
        const auto plate_sequence = [&](int index) {
            if (index < (int)plate_data.size() && plate_data[index] != nullptr)
                if (const auto* seq = plate_data[index]->config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence");
                    seq && seq->value != Slic3r::PrintSequence::ByDefault)
                    return seq->value == Slic3r::PrintSequence::ByObject;
            return project_sequence();
        };

        // --downward-check, before the transforms as the official runs it
        // (BambuStudio.cpp 4669-4914; OrcaSlicer.cpp 4006-4175). The
        // official checks the plates of a project (plate_obj_size_infos
        // exists only then, BambuStudio.cpp 4646-4658), every one of them.
        if (o.given_flag("downward_check") && o.cli.opt_bool("downward_check") && context.g_downward_run.done) {
            // Every plate was checked by the run's first call.
            outcome.downward_checked = true;
            outcome.downward_printers = context.g_downward_run.printers;
            outcome.downward_failed = context.g_downward_run.failed;
            outcome.sequence_plate = context.g_downward_run.sequence_plate;
            outcome.multi_nozzle_diameters = context.g_downward_run.multi_nozzle_diameters;
        } else if (o.given_flag("downward_check") && o.cli.opt_bool("downward_check")) {
            if (plate_data.empty()) {
                const std::string why = "--downward-check checks the plates of a 3MF project; this input has none";
                std::cerr << "Error: " << why << "\n";
                set_outcome_failure(outcome, CLI_INVALID_PARAMS, why);
                return 1;
            }
            std::vector<slicer_cli::DownwardPrinter> printers;
            const slicer_cli::StepResult loaded =
                slicer_cli::load_downward_printers(o, project_facts, this_engine_profiles_dir(o.argv0).string(), printers);
            if (loaded.code != 0) {
                std::cerr << "Error: " << loaded.message << "\n";
                set_outcome_failure(outcome, loaded.code, loaded.message);
                return 1;
            }
            // Every plate of the project, whichever is sliced (BambuStudio.cpp
            // 4779-4790; OrcaSlicer.cpp 4104-4117): each plate's objects, and
            // its own print sequence or else the project's (get_print_sequence,
            // BambuStudio.cpp 4403-4416). A plate in the run's model is read
            // from it, in its own frame (PlateScope); a plate the run did not
            // load (--plate N / --slice N load one plate) from the project.
            struct DownwardPlate {
                int index = 0;
                std::unique_ptr<Slic3r::Model> loaded;
                const Slic3r::Model* model = nullptr;
                bool is_sequence = false;
            };
            std::vector<DownwardPlate> downward_plates;
            const int project_plates = (int)plate_data.size();
            for (int index = 0; index < project_plates; ++index) {
                DownwardPlate plate;
                plate.index = index;
                const bool in_model = index < (int)rs.plates.size() &&
                                      (rs.plates[index].all || !rs.plates[index].members.empty());
                if (project_plates == 1 || index == plate_id - 1 || in_model) {
                    {
                        PlateScope scope(model, rs.plate(index + 1));
                        plate.loaded = std::make_unique<Slic3r::Model>(model);
                    }
                    plate.model = plate.loaded.get();
                    if (project_plates == 1 || index == plate_id - 1) {
                        bool is_sequence = false;
                        if (const auto* seq = config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence"))
                            is_sequence = seq->value == Slic3r::PrintSequence::ByObject;
                        plate.is_sequence = is_sequence;
                    } else {
                        plate.is_sequence = plate_sequence(index);
                    }
                } else {
                    plate.loaded = std::make_unique<Slic3r::Model>();
                    if (assemble_input) {
                        for (const Slic3r::ModelObject* object : context.g_assemble->plates[index].loaded_obj_list)
                            plate.loaded->add_object(*object);
                    } else if (!load_plate_objects(context, input_file, index + 1, *plate.loaded)) {
                        const std::string why = "--downward-check could not read plate " + std::to_string(index + 1) +
                                                " of " + input_file + ".";
                        std::cerr << "Error: " << why << "\n";
                        set_outcome_failure(outcome, CLI_DATA_FILE_ERROR, why);
                        return 1;
                    } else if (verbose) {
                        std::cout << "--downward-check: read plate " << index + 1 << " from the project\n";
                    }
                    if (!assemble_input && index < (int)plate_origins.size()) {
                        // The plate at its own origin, as the sliced plate is:
                        // the official adds the plate's origin to its tower
                        // position instead (plate->get_origin(), BambuStudio.cpp
                        // 4486-4489; OrcaSlicer.cpp 3875-3878).
                        const Slic3r::Vec2d origin = plate_origins[index];
                        for (Slic3r::ModelObject* object : plate.loaded->objects)
                            for (Slic3r::ModelInstance* inst : object->instances) {
                                const Slic3r::Vec3d off = inst->get_offset();
                                inst->set_offset(Slic3r::Vec3d(off.x() - origin.x(), off.y() - origin.y(), off.z()));
                            }
                    }
                    plate.model = plate.loaded.get();
                    plate.is_sequence = plate_sequence(index);
                }
                downward_plates.push_back(std::move(plate));
            }
            // A Bambu-made 3MF with support on anywhere (BambuStudio.cpp 2197-2210).
            bool has_support = false;
            if (is_bbl_3mf && !trailing_port) {
                const auto* support = config.option<Slic3r::ConfigOptionBool>("enable_support");
                has_support = support && support->value;
                for (const DownwardPlate& plate : downward_plates)
                    for (const Slic3r::ModelObject* object : plate.model->objects) {
                        const auto* obj_support = object->config.get().option<Slic3r::ConfigOptionBool>("enable_support");
                        has_support |= obj_support && obj_support->value;
                    }
            }
            outcome.downward_checked = true;
            for (const slicer_cli::DownwardPrinter& p : printers)
                outcome.downward_printers.push_back(p.name);
#ifdef ENGINE_BAMBU
            // Mixed nozzle diameters (BambuStudio.cpp 5218-5258 at v02.09.01.70):
            // every diameter the project's filaments select needs a nozzle of
            // its own, so a printer that mounts fewer nozzles than that fails.
            // With no plate using a filament, the whole map counts.
            if (Slic3r::multi_nozzle_diameter_mode_enabled(config)) {
                std::vector<std::vector<int>> plates_filaments;
                for (const DownwardPlate& plate : downward_plates)
                    plates_filaments.push_back(arrange_plate_extruders(*plate.model, config, plate.index));
                const std::set<std::string> used_diameters = slicer_cli::project_nozzle_diameters(config, plates_filaments);
                outcome.multi_nozzle_diameters = used_diameters.size() > 1;
                if (used_diameters.size() > 1)
                    for (const slicer_cli::DownwardPrinter& p : printers)
                        if ((int)used_diameters.size() > p.max_nozzle_count &&
                            std::find(outcome.downward_failed.begin(), outcome.downward_failed.end(), p.name) ==
                                outcome.downward_failed.end()) {
                            emit_event({{"event","warning"},
                                        {"tag","PrinterNozzleCountTooSmall"},
                                        {"printer", p.name},
                                        {"max_nozzle_count", p.max_nozzle_count},
                                        {"nozzle_diameters", std::vector<std::string>(used_diameters.begin(), used_diameters.end())},
                                        {"message", p.name + " mounts " + std::to_string(p.max_nozzle_count) +
                                                    " nozzle(s); the project uses " + std::to_string(used_diameters.size()) +
                                                    " nozzle diameters"}});
                            outcome.downward_failed.push_back(p.name);
                        }
            }
#endif
            for (const DownwardPlate& plate : downward_plates) {
                outcome.sequence_plate |= plate.is_sequence;
                if (printers.empty())
                    continue;
                const std::vector<std::string> failed = slicer_cli::downward_failures(
                    printers, downward_plate_size(context, *plate.model, config, plate.index, plate.is_sequence),
                    plate.is_sequence, config, project_facts, has_support);
                for (const std::string& name : failed)
                    if (std::find(outcome.downward_failed.begin(), outcome.downward_failed.end(), name) ==
                        outcome.downward_failed.end())
                        outcome.downward_failed.push_back(name);
            }
            context.g_downward_run.done = true;
            context.g_downward_run.printers = outcome.downward_printers;
            context.g_downward_run.failed = outcome.downward_failed;
            context.g_downward_run.sequence_plate = outcome.sequence_plate;
            context.g_downward_run.multi_nozzle_diameters = outcome.multi_nozzle_diameters;
        }

        // --pipe: the files are loaded (BambuStudio.cpp 4923-4926;
        // OrcaSlicer.cpp 4185), before the transforms.
        pipe_files_loaded(rs);

        // The transforms on the command line, in its order, then orient
        // (cli_run_steps.cpp). Before any placement, as the official CLI
        // runs them before its arrange.
        // A plate of the plan has its objects transformed already (the plan
        // pass ran the transforms on every object, as the official does).
        // The plates of an assemble list at their places in the plate grid of
        // this printer's bed (PartPlateList::compute_origin, PartPlate.cpp
        // 4306-4341), now that the bed is known.
        if (first_call && assemble_input) {
            const int count = (int)context.g_assemble->plates.size();
            for (int k = 0; k < count && k < (int)rs.plates.size(); ++k) {
                const Slic3r::Vec2d origin = plate_grid_origin(config, Slic3r::Semver(), k, count);
                rs.plates[k].origin = origin;
                for (Slic3r::ModelInstance* inst : rs.plates[k].members)
                    inst->set_offset(inst->get_offset() + Slic3r::Vec3d(origin.x(), origin.y(), 0.));
            }
        }
        if (first_call && !o.transforms.empty()) {
            int duplicate_count = 0;
            const int plate_count = std::max<int>(1, (int)plate_data.size());
            const slicer_cli::StepResult t = slicer_cli::apply_transforms(
                o, model, &settings_merge.load_process_config, o.slice_mode ? o.slice_plate : plate_id, plate_count,
                duplicate_count);
            if (t.code != 0) {
                std::cerr << "Error: " << t.message << "\n";
                set_outcome_failure(outcome, t.code, t.message);
                return 1;
            }
            outcome.duplicate_count = duplicate_count;
        }
        // --simplify: the desktop Simplify tool, after the transforms and
        // before the objects are placed, so the placement, the bed check,
        // --mtcpp, every export and the slice see the reduced meshes. The
        // desktop's "more than 1M triangles" suggestion follows, on the
        // meshes as they now stand (cli_simplify.cpp).
        if (first_call) {
            // The ids --simplify-objects and --skip-objects may name
            // (cli_engine_info.hpp).
            const std::set<int> known_object_ids = slicer_cli::known_object_ids(
                o.input_files, model,
                [](const std::string& archive, const std::string& member, std::string& out) {
                    return read_zip_member(archive, member, out);
                });
            // --skip-objects keeps the official meaning (an unknown id is
            // ignored, BambuStudio.cpp 6541-6581), so the caller is told once
            // by an event, where --simplify-objects, a new flag, refuses.
            if (const std::string why = slicer_cli::unknown_object_ids_refusal(
                    "--skip-objects", o.cli.option<Slic3r::ConfigOptionInts>("skip_objects")->values, known_object_ids);
                !why.empty())
                emit_event({{"event", "warning"},
                            {"tag", "SkipObjectsUnknownIds"},
                            {"message", why + ". The id is ignored and the slice goes on, as the official command line does"}});
            const slicer_cli::StepResult sm = slicer_cli::simplify_meshes(o, model, known_object_ids);
            if (sm.code != 0) {
                std::cerr << "Error: " << sm.message << "\n";
                set_outcome_failure(outcome, sm.code, sm.message);
                return 1;
            }
            slicer_cli::warn_large_meshes(model);
        }
        {
            const slicer_cli::StepResult sd = slicer_cli::check_slicedata_flags(o, outcome.duplicate_count);
            if (sd.code != 0) {
                std::cerr << "Error: " << sd.message << "\n";
                set_outcome_failure(outcome, sd.code, sd.message);
                return 1;
            }
        }
        const bool needs_placement = first_call && (geometry_input ||
            std::any_of(model.objects.begin(), model.objects.end(),
                        [](const Slic3r::ModelObject* obj) { return obj->instances.empty(); }));

        // Geometry placement, with the bed now known. --arrange 1 or 0: the
        // official CLI's default instances (read_from_file with
        // AddDefaultInstances, BambuStudio.cpp 1884), then --arrange 1
        // arranges below. Otherwise the desktop's placement on load
        // (cli_model_load.cpp). A project 3MF's own objects keep their places.
        if (first_call && geometry_only_3mf && o.arrange_auto())
            slicer_cli::desktop_center_geometry_3mf(model, config);
        if (needs_placement) {
            if (!o.arrange_auto())
                model.add_default_instances();
            else
                slicer_cli::desktop_place_on_bed(model, config);
            for (auto* obj : model.objects)
                for (auto* inst : obj->instances)
                    inst->use_loaded_id_for_label = true;
        }

        // Actions that work on the loaded model and settings (--info,
        // --export-settings, --export-stl, --export-stls). The official runs
        // every action in one loop in command-line order, the first that fails
        // stopping the rest (BambuStudio.cpp 6336, 6366-6401; OrcaSlicer.cpp
        // 5470, 5499-5534), --slice among them (6437; 5561) with each plate's
        // bed check and slice inside it. So with --slice the actions given
        // before it run before the first plate's bed check, and those given
        // after it once the last plate has sliced; without --slice they are the
        // whole run. A failure is recorded with plate 0, as the official records
        // these (record_exit_reson(..., CLI_EXPORT_STL_ERROR, 0, ...),
        // BambuStudio.cpp 6391, 6399).
        // A project 3MF with model files after it: the plan (ProjectPlan) once
        // the transforms ran, before any action, as the official arranges
        // before its action loop; the actions then see every object at its
        // arranged place.
        // --slice 0 --arrange 1 on a project of several plates: the official
        // arranges every plate's objects together across the plates (the
        // global arrange, BambuStudio.cpp 5628-5728, 5886-6006; OrcaSlicer.cpp
        // 4887-4990, 5142-5262: duplicate_count 0 and plate_to_slice 0), adding
        // or dropping plates, as for model files after the project.
        // A run of model actions only is a run without --slice, whose
        // plate_to_slice stays 0 (BambuStudio.cpp 1677; OrcaSlicer.cpp 1368):
        // the official arranges every plate for it too.
        const bool global_arrange = (o.slice_mode ? o.slice_plate == 0 : slicer_cli::model_actions_only(o)) &&
                                    o.arrange_forced() && is_bbl_3mf &&
                                    !assemble_input && !trailing_port && plate_data.size() > 1;
        if (first_call && (trailing_port || global_arrange) && !calib_self_geometry) {
            std::set<int> used_filaments;
#ifndef ENGINE_ORCA
            for (int f = 1; f <= plan_file_filaments; ++f)
                used_filaments.insert(f);
#endif
            const auto& filament_ids = o.cli.option<Slic3r::ConfigOptionInts>("load_filament_ids")->values;
            for (size_t index = 1; index < o.input_files.size() && index < filament_ids.size(); ++index)
                if (filament_ids[index] > 0)
                    used_filaments.insert(filament_ids[index]);
            ProjectPlan& plan = rs.plan;
            const Slic3r::DynamicPrintConfig& file_config = plan_file_config ? *plan_file_config : config;
            if (!plan_project_plates(context, model, config, file_config, plan_file_version, plate_data, (int)used_filaments.size(),
                                     global_arrange || o.arrange != 0, plan, outcome)) {
                std::cerr << "Error: " << outcome.error_string << "\n";
                return 1;
            }
            // The plates as the plan left them; a plate the arrange added is a
            // fresh plate with no settings of its own (PartPlateList::create_plate).
            rs.plates.assign(plan.plate_count, RunPlate());
            for (int k = 0; k < plan.plate_count; ++k) {
                rs.plates[k].origin  = plan.origins[k];
                rs.plates[k].all     = false;
                rs.plates[k].members = plan.members[k];
            }
            while ((int)plate_data.size() < plan.plate_count) {
                auto* pd = new Slic3r::PlateData();
                pd->plate_index = int(plate_data.size());
                plate_data.push_back(pd);
            }
            emit_project_plan_event(plan, trailing_port);
        }

        // The actions read the run's one model, every object at its place in
        // the scene (the official's m_models, BambuStudio.cpp 6371-6401;
        // OrcaSlicer.cpp 5504-5534), and --export-settings the project's
        // settings (m_print_config, BambuStudio.cpp 6366-6370; OrcaSlicer.cpp
        // 5499-5503).
        const auto run_model_action_step = [&](slicer_cli::ActionPhase phase) -> bool {
            slicer_cli::StepResult a;
            // --export-settings writes the project's settings, not this
            // plate's (see project_settings above). The command line is
            // already in them; the setting flags go on as on `config`.
            std::optional<Slic3r::DynamicPrintConfig> settings_to_export = project_settings;
            if (settings_to_export)
                apply_command_line_overrides(*settings_to_export, overrides, /*report_rejections=*/false);
#ifdef ENGINE_SNAPMAKER
            // The switch's filament list, as each plate slices with it.
            if (settings_to_export && slicer_cli::switch_trims_filaments(o.preset_switch, o.printer_preset))
                slicer_cli::trim_project_roster_for_switch(*settings_to_export);
#endif
            {
                // The actions run once per run, also from the check pass:
                // their events are not held.
                std::vector<std::string>* const held = slicer_cli::diagnostics::held_events;
                slicer_cli::diagnostics::held_events = nullptr;
                a = slicer_cli::run_model_actions(o, model, settings_to_export ? *settings_to_export : config, nullptr,
                                                  phase);
                slicer_cli::diagnostics::held_events = held;
            }
            if (a.code != 0) {
                std::cerr << "Error: " << a.message << "\n";
                set_outcome_failure(outcome, a.code, a.message);
                outcome.run_step_failed = true;
                return false;
            }
            return true;
        };
        // Every output of the run, checked once the model is loaded (the
        // object STL names are known) and before the first write: the
        // actions, then the slice (check_run_outputs). The plates are the
        // ones already known; --slice checks them all again once its plan is
        // final (run_slice_mode).
        const auto outputs_ok = [&](bool after_slice_actions_only = false) -> bool {
            if (rs.outputs_checked)
                return true;
            rs.outputs_checked = true;
            // Every plate the run could write: plate N for --slice N, else
            // every plate of the plan, or of the input when there is no plan
            // yet (a superset is fine: the check after the plan stays).
            std::vector<int> known_plates;
            if (o.slice_mode) {
                if (o.slice_plate > 0)
                    known_plates.push_back(o.slice_plate);
                const int count = rs.plan.done ? std::max(rs.plan.plate_count, rs.file_plate_count)
                                               : rs.file_plate_count;
                if (o.slice_plate <= 0)
                    for (int p = 1; p <= std::max(1, count); ++p)
                        known_plates.push_back(p);
            }
            std::string detail;
            if (const int refused = check_run_outputs(context, o, known_plates, &model, &detail, after_slice_actions_only,
                                                      after_slice_actions_only ? &rs.reserved_outputs : nullptr,
                                                      after_slice_actions_only ? nullptr : &rs.reserved_outputs)) {
                set_outcome_failure(outcome, refused, detail);
                outcome.run_step_failed = true;
                return false;
            }
            return true;
        };

        // --slice: the official per-plate gate before apply (BambuStudio.cpp
        // 6527-6567 at 5873b5f; OrcaSlicer.cpp 5645-5697 at 31f6803). An
        // object partly over the bed edge is refused, and a plate with no
        // object fully inside is refused; objects wholly outside are left
        // out of the print, as the official CLI's apply() leaves them out.
        // --slice, once per run before the plate loop: the arrange, the
        // --ensure-on-bed lift and the actions given before --slice
        // (BambuStudio.cpp 5237-6194, 6336; OrcaSlicer.cpp 4498-5455, 5470),
        // for the plate the run names (in its own frame, PlateScope) or for
        // every plate. A run of model actions only takes the same steps: the
        // official arranges before its action loop with or without --slice
        // (need_arrange for model files, BambuStudio.cpp 2077, 5558, then
        // 6336; OrcaSlicer.cpp 1724, 4817, then 5470).
        const bool actions_only = slicer_cli::model_actions_only(o);
        if ((o.slice_mode || actions_only) && !calib_self_geometry && first_call) {
            // The plate's towers before the arrange: only the entries the
            // arrange changes go into the project below. The rest are the
            // flags laid over this plate (BambuStudio.cpp 6902-6904), which
            // reach only the slice.
            std::map<std::string, std::vector<double>> tower_before;
            for (const char* key : {"wipe_tower_x", "wipe_tower_y"})
                if (const auto* opt = config.option<Slic3r::ConfigOptionFloats>(key))
                    tower_before[key] = opt->values;
            // The plate's filament count from its last slice (slice_info), as
            // the official reads plate_data_src[plate]->slice_filaments_info
            // (BambuStudio.cpp 5794; OrcaSlicer.cpp 5050); 0 for an STL.
            size_t sliced_filament_count = 0;
            if (plate_id > 0 && (int)plate_data.size() >= plate_id && plate_data[plate_id - 1] != nullptr)
                sliced_filament_count = plate_data[plate_id - 1]->slice_filaments_info.size();
            const int arrange_plate_index = plate_id > 0 ? plate_id - 1 : 0;
            // Several objects from model files and --arrange automatic (not 0
            // or 1, or not given): the official
            // CLI arranges model files (need_arrange is set for them,
            // BambuStudio.cpp 2077 / OrcaSlicer.cpp 1724, and --arrange only
            // overrides it, 5066-5081 / 4327-4342), where the desktop's
            // placement on load could stack them. One object keeps the
            // desktop's placement (the bed centre either way).
            int model_file_instances = 0;
            if (geometry_input)
                for (const Slic3r::ModelObject* object : model.objects)
                    for (const Slic3r::ModelInstance* inst : object->instances)
                        model_file_instances += inst->printable ? 1 : 0;
            const bool auto_arrange_model_files = o.arrange_auto() && model_file_instances > 1;
            // A printer change that changes the extruder clearances re-arranges
            // a plate printed by object (BambuStudio.cpp 5275-5288: only on a
            // change of printer, the distance to the rod counted; OrcaSlicer.cpp
            // 4538-4549: any change of the clearances), unless the bed changed
            // (shrink_to_new_bed) or the plate is arranged anyway.
            bool clearance_arrange = false;
            {
                const auto* seq = config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence");
                const bool is_seq_print = seq && seq->value == Slic3r::PrintSequence::ByObject;
                const bool need_arrange = o.arrange_forced() || auto_arrange_model_files || outcome.duplicate_count > 0 ||
                                          assemble_input;
                auto opt_or_zero = [&](const char* key) {
                    return config.option<Slic3r::ConfigOptionFloat>(key) ? float(config.opt_float(key)) : 0.f;
                };
                const float height_to_rod = opt_or_zero("extruder_clearance_height_to_rod");
                const float height_to_lid = opt_or_zero("extruder_clearance_height_to_lid");
#ifdef ENGINE_ORCA
                const float clearance_radius = opt_or_zero("extruder_clearance_radius");
                const bool printer_changed = true;
                const bool rod_distance_changed = false;
#else
                const float clearance_radius = opt_or_zero("extruder_clearance_max_radius");
                const bool printer_changed = !settings_merge.new_printer_system_name.empty() &&
                                             settings_merge.new_printer_system_name != project_facts.current_printer_system_name;
                const float distance_to_rod = opt_or_zero("extruder_clearance_dist_to_rod");
                const bool rod_distance_changed = project_facts.old_distance_to_rod != 0.f &&
                                                  project_facts.old_distance_to_rod != distance_to_rod;
#endif
                const bool clearances_changed =
                    (project_facts.old_height_to_rod != 0.f && project_facts.old_height_to_rod != height_to_rod) ||
                    (project_facts.old_height_to_lid != 0.f && project_facts.old_height_to_lid != height_to_lid) ||
                    (project_facts.old_max_radius != 0.f && project_facts.old_max_radius != clearance_radius) ||
                    rod_distance_changed;
                // plate_to_slice > 0: one plate named, never --slice 0; 0
                // without --slice (BambuStudio.cpp 1677).
                const int plate_to_slice = o.slice_mode ? o.slice_plate : 0;
                clearance_arrange = !need_arrange && is_bbl_3mf && !trailing_port && shrink_to_new_bed == 0 && plate_to_slice > 0 &&
                                    printer_changed && clearances_changed && is_seq_print;
                if (clearance_arrange)
                    emit_event({{"event","arranged"}, {"tag","ClearanceArrange"},
                                {"message","The printer's extruder clearances differ from the project's: the by-object plate is arranged again"}});
            }
            // --repetitions: a spiral vase is copied only when printing by
            // object (BambuStudio.cpp 5259-5269; OrcaSlicer.cpp 4519-4529).
            if (!assemble_input && outcome.duplicate_count > 0) {
                const auto* spiral = config.option<Slic3r::ConfigOptionBool>("spiral_mode");
                const auto* seq    = config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence");
                if (spiral && spiral->value && !(seq && seq->value == Slic3r::PrintSequence::ByObject)) {
                    emit_event({{"event","arranged"}, {"tag","RepetitionsSkippedSpiral"},
                                {"message","--repetitions: a spiral vase is copied only when printing by object; the plate is printed once"}});
                    outcome.duplicate_count = 0;
                }
            }
            // --repetitions arranges the copies and sets the tower in `config`
            // (arrange_repetitions); the project keeps that tower.
            if (assemble_input) {
                // An assemble list's plates take the list's own arrange in
                // place of --arrange and --repetitions (BambuStudio.cpp
                // 5326-5557, before the general arrange's else-branch;
                // OrcaSlicer.cpp 4582-4816), every plate of the run in its
                // own frame.
                for (int arranged_plate = plate_id > 0 ? plate_id : 1;
                     arranged_plate <= (plate_id > 0 ? plate_id : (int)context.g_assemble->plates.size()); ++arranged_plate) {
                const int arrange_plate_index = arranged_plate - 1;
                const slicer_cli::AssemblePlate& plate = context.g_assemble->plates[arrange_plate_index];
                // A plate of --slice 0 reads its own settings (plate_params)
                // over the run's, as its own slice does.
                Slic3r::DynamicPrintConfig plate_settings_copy;
                if (plate_id == 0) {
                    plate_settings_copy = config;
                    plate_settings_copy.apply(plate_own_settings(plate_data, arranged_plate), true);
                    if (!extra.empty())
                        plate_settings_copy.apply(extra, true);
                    apply_command_line_overrides(plate_settings_copy, overrides, /*report_rejections=*/false);
                }
                Slic3r::DynamicPrintConfig& config_k = plate_id == 0 ? plate_settings_copy : config;
                PlateScope scope(model, rs.plate(arranged_plate));
                if (plate.need_arrange) {
                    if (!arrange_assemble_plate(context, model, config_k, arrange_plate_index, plate.filaments_count,
                                                context.g_assemble->model, outcome)) {
                        std::cerr << "Error: " << outcome.error_string << "\n";
                        return 1;
                    }
                } else {
                    // The objects keep the list's positions; a plate of several
                    // filaments gets the tower's default corner unless a
                    // position was given (has_wipe_tower_position, BambuStudio.cpp
                    // 4098: wipe_tower_x/y are in no preset, so only a settings
                    // file or a flag states them for a run without a 3MF). The
                    // corner is the desktop's own: clamped inside the plate and
                    // its extruder areas, as the desktop clamps a new plate's
                    // (set_default_wipe_tower_pos_for_plate, Orca PartPlate.cpp
                    // 4115-4190; Bambu PartPlate.cpp 4615-4702) — the official
                    // CLI's corner alone put the tower off the bed and refused
                    // this run -104.
                    const auto* seq = config_k.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence");
                    const bool is_seq_print = seq && seq->value == Slic3r::PrintSequence::ByObject;
                    const auto states_tower = [](const Slic3r::DynamicPrintConfig& c) {
                        return c.has("wipe_tower_x") && c.has("wipe_tower_y");
                    };
                    const bool has_wipe_tower_position =
                        states_tower(settings_merge.load_process_config) || states_tower(o.extra_config);
                    if (!is_seq_print && plate.filaments_count > 1 && !has_wipe_tower_position)
                        assemble_tower_inside_plate(context, config_k, model, arrange_plate_index, /*keep_existing=*/false);
                }
                // The plate's tower entry goes into the run's settings.
                if (plate_id == 0)
                    for (const char* key : {"wipe_tower_x", "wipe_tower_y"})
                        if (const auto* placed = config_k.option<Slic3r::ConfigOptionFloats>(key);
                            placed && (int)placed->values.size() > arrange_plate_index) {
                            Slic3r::ConfigOptionFloat entry(placed->get_at(arrange_plate_index));
                            config.option<Slic3r::ConfigOptionFloats>(key, true)->set_at(&entry, arrange_plate_index, 0);
                        }
                }
            } else if (outcome.duplicate_count > 0) {
                PlateScope scope(model, rs.plate(plate_id));
                std::set<int> skip_ids;
                for (int id : o.cli.option<Slic3r::ConfigOptionInts>("skip_objects")->values)
                    skip_ids.insert(id);
                model.add_default_instances();
                const slicer_cli::RepetitionsResult reps = slicer_cli::arrange_repetitions(
                    model, config, arrange_plate_index, outcome.duplicate_count, skip_ids,
                    [&](Slic3r::Model& m, Slic3r::DynamicPrintConfig& c, int& landed) {
                        PlateOutcome scratch = outcome;
                        return arrange_on_bed(context, m, c, arrange_plate_index, sliced_filament_count, scratch, &landed);
                    });
                // Copies the loop left off the plate stand wholly outside it
                // and are left out by the inside check below.
                outcome.duplicate_count = reps.copies_kept;
            } else if ((o.arrange_forced() || auto_arrange_model_files || clearance_arrange) && !rs.plan.done) {
                PlateScope scope(model, rs.plate(plate_id));
                if (!arrange_on_bed(context, model, config, arrange_plate_index, sliced_filament_count, outcome)) {
                    std::cerr << "Error: " << outcome.error_string << "\n";
                    return 1;
                }
            }
            // --ensure-on-bed before the actions given before --slice, as the
            // official lifts the objects before its action loop
            // (BambuStudio.cpp 6188-6194 then 6336; OrcaSlicer.cpp 5443-5455
            // then 5470); --skip-objects stays in the slice step after them.
            slicer_cli::ensure_on_bed_if_asked(o, model);
            // The towers the arrange placed are the project's from here on:
            // the official arrange writes them into m_print_config
            // (BambuStudio.cpp 5693-5700, 5856, 6075; OrcaSlicer.cpp
            // 4954-4961, 5112, 5335), which --export-settings, result.json and
            // --export-3mf then read (6366-6370, 6905-6910, 8156-8157).
            // Only the entries the arrange set: a tower given as a flag stays
            // the plate's, and the project keeps its own (moved) value.
            for (const char* key : {"wipe_tower_x", "wipe_tower_y"}) {
                const auto* placed = config.option<Slic3r::ConfigOptionFloats>(key);
                if (placed == nullptr)
                    continue;
                const std::vector<double>& before = tower_before[key];
                auto* project_tower = rs.base.option<Slic3r::ConfigOptionFloats>(key);
                if (project_tower == nullptr || project_tower->values.empty()) {
                    rs.base.set_key_value(key, placed->clone());
                    continue;
                }
                for (size_t k = 0; k < placed->values.size(); ++k) {
                    if (k < before.size() && before[k] == placed->values[k])
                        continue;
                    Slic3r::ConfigOptionFloat entry(placed->values[k]);
                    project_tower->set_at(&entry, k, 0);
                }
            }
            if (slicer_cli::has_model_actions(o))
                project_settings = rs.base;
            if (!outputs_ok())
                return 1;
            if (slicer_cli::has_model_actions(o) && !rs.actions_done) {
                rs.actions_done = true;
                if (!run_model_action_step(o.slice_mode ? slicer_cli::ActionPhase::BeforeSlice
                                                        : slicer_cli::ActionPhase::All))
                    return 1;
            }
            // Model actions only: the actions were the whole run.
            if (actions_only) {
                outcome.actions_only = true;
                return 0;
            }
            // --pipe: the slice action starts, before the first plate's
            // checks (BambuStudio.cpp 6455-6458; OrcaSlicer.cpp 5580).
            pipe_prepare_slicing(rs);
        }
        // The run-level steps are done: the prepare call stops here, before
        // any plate is checked.
        if (first_call && o.slice_mode) {
            rs.prepared = true;
            rs.prepared_plate = plate_id;
            rs.run_config = config;
            rs.carry = outcome;
            if (outcome.prepare_pass)
                return 0;
        }

        // --slice: the official per-plate gate before apply (BambuStudio.cpp
        // 6527-6567 at 5873b5f; OrcaSlicer.cpp 5645-5697 at 31f6803). An
        // object partly over the bed edge is refused, and a plate with no
        // object fully inside is refused; objects wholly outside are left
        // out of the print, as the official CLI's apply() leaves them out.
        if (o.slice_mode && !calib_self_geometry) {
            RunPlate& run_plate = rs.plate(plate_id);
            pipe_prepare_slicing(rs);
            // A plate with none of the run's instances: the refusal a load of
            // that plate alone gave (above, "No objects found in model").
            if (!run_plate.all && run_plate.members.empty()) {
                std::cerr << "No objects found in model\n";
                set_outcome_failure(outcome, CLI_NO_SUITABLE_OBJECTS);
                return 1;
            }
            const int step_plate = plate_id > 0 ? plate_id : std::max(1, o.slice_plate);
            {
                // The plate's instances in the plate's own frame: the bed check
                // reads the plate's shape (PartPlate::get_shape, the bed at the
                // plate's origin, BambuStudio.cpp 6527).
                PlateScope scope(model, run_plate);
                if (!plate_object_steps(o, model, step_plate, outcome))
                    return 1;
#ifdef ENGINE_BAMBU
                if (!manual_map_extruders_fit(config, model, step_plate, outcome))
                    return 1;
#endif
                if (!check_objects_inside_bed(model, config, outcome, o.arrange_forced())) {
                    std::cerr << "Error: " << outcome.error_string << "\n";
                    return 1;
                }
                if (!plate_triangle_limit(o, model, step_plate, outcome))
                    return 1;
            }
            // The other plates' instances are outside this plate, so this
            // plate's Print leaves them out (ModelInstance::is_printable,
            // Model.hpp 1466 / 1344; PrintApply.cpp 145-155).
            for (Slic3r::ModelObject* object : model.objects)
                for (Slic3r::ModelInstance* inst : object->instances)
                    if (!run_plate.is_member(inst))
                        inst->print_volume_state = Slic3r::ModelInstancePVS_Fully_Outside;
        } else if (!calib_self_geometry) {
            if (first_call)
                slicer_cli::ensure_on_bed_if_asked(o, model);
            // --skip-objects and --mtcpp belong to the slice action
            // (BambuStudio.cpp 6541-6597; OrcaSlicer.cpp 5645-5720): a run of
            // model actions only never slices, so it skips nothing.
            if (!slicer_cli::model_actions_only(o) &&
                (!plate_object_steps(o, model, std::max(1, plate_id), outcome) ||
                 !plate_triangle_limit(o, model, std::max(1, plate_id), outcome)))
                return 1;
        }

        // Without --slice, or for the calibration prints that make their own
        // model, the actions run here; with --slice those before it ran before
        // the bed check above. The outputs are checked first (without --slice
        // this is the check before the actions and the -o G-code).
        if (!outputs_ok())
            return 1;
        if (slicer_cli::has_model_actions(o) && !rs.actions_done) {
            rs.actions_done = true;
            if (!run_model_action_step(o.slice_mode ? slicer_cli::ActionPhase::BeforeSlice : slicer_cli::ActionPhase::All))
                return 1;
            if (slicer_cli::model_actions_only(o)) {
                outcome.actions_only = true;
                return 0;
            }
        }

        // Initialize print
        std::cout << "\nInitializing print...\n";
        Slic3r::Print print;

        // The plate's own per-layer custom G-code (pauses, colour changes,
        // custom lines): the loader keys them by plate (plate_id - 1,
        // bbs_3mf.cpp 3446/3474), --load-custom-gcodes by the plate sliced,
        // and the Print reads the entry of the model's current plate
        // (Print.cpp 517-518; PrintApply.cpp 1558-1569 copies the index), so
        // the official CLI sets it per plate before slicing it
        // (BambuStudio.cpp 6493; OrcaSlicer.cpp 5617). Left at 0, every plate
        // took plate 1's. The pressure-advance pattern writes its entry at
        // the same index (calib.cpp 655).
        model.curr_plate_index = std::max(0, plate_id - 1);

        // Enable BBL printer features (M981 spaghetti detector, M1003 powerlost
        // recovery, etc.) for a printer of the Bambu Lab vendor.
#ifdef ENGINE_BAMBU
        // set_BBL_Printer is a BambuStudio-only Print method (enables M981/M1003
        // BBL printer features).  OrcaSlicer's Print has no such method.
        // The desktop decides from the printer preset's vendor
        // (BackgroundSlicingProcess.cpp:205, Preset::is_bbl_vendor_preset), and
        // the official CLI from the plate config's printer_model, else the
        // loaded or the project's printer name (BambuStudio.cpp 7055-7070 at
        // 5873b5f): the same rule for a 3MF, a named preset or settings files.
        {
            const std::string printer_model_string = config.opt_string("printer_model", true);
            bool is_bbl_vendor_preset = false;
            if (!printer_model_string.empty())
                is_bbl_vendor_preset = printer_model_string.compare(0, 9, "Bambu Lab") == 0;
            else if (!rs.settings_merge.new_printer_name.empty())
                is_bbl_vendor_preset = rs.settings_merge.new_printer_name.compare(0, 9, "Bambu Lab") == 0;
            else if (!rs.project_facts.current_printer_system_name.empty())
                is_bbl_vendor_preset = rs.project_facts.current_printer_system_name.compare(0, 9, "Bambu Lab") == 0;
            print.set_BBL_Printer(is_bbl_vendor_preset);
        }
#endif
#ifdef ENGINE_ORCA
        // Orca's Print::m_isBBLPrinter (Print.hpp:1143) has NO default initializer and
        // no set_BBL_Printer() method. Left uninitialized it is read with an
        // indeterminate value (UB) across the slice/export phases — driving both the
        // wipe_tower_type() Type1/Type2 choice (Print.hpp:1072), the GCode dialect
        // (GCode.cpp:2046), and the ;Z_HEIGHT vs ;Z: comment (GCode.cpp:4486). The
        // inconsistent reads make process()'s wipe-tower tool ordering disagree with the
        // sequence read at export → "append_tcr ... toolchange it didn't expect".
        // Mirror OrcaSlicer's own headless CLI (OrcaSlicer.cpp:5972-5986): the flag is
        // true iff the printer_model preset begins with "Bambu Lab" — false for the
        // Snapmaker U1, which selects Type2 + the Orca/Prusa-style gcode dialect (golden B).
        {
            const std::string pm = config.opt_string("printer_model", true);
            print.is_BBL_printer() = (pm.rfind("Bambu Lab", 0) == 0);
        }
#endif

        // The plate's Print in the plate's own frame. The official CLI slices
        // each plate of the scene model with a Print whose origin is the
        // plate's (PartPlate::get_print and the plate's BuildVolume,
        // BambuStudio.cpp 6506-6529; OrcaSlicer.cpp 5623-5646; the origin from
        // set_plate_origin, PartPlate.cpp 2376/2517 at 5873b5f, 2311/2433 at
        // 31f6803). Here the Print's model is the plate's instances moved by
        // minus the plate's origin (PlateScope at print.apply below) and the
        // Print's origin stays 0: the same placement, and the G-code this
        // command line has always written, which the product relies on. The
        // two differ in bytes on a plate past the first: the engine fits arcs
        // on the Print's own coordinates (arc centres round differently) and
        // starts the first layer's moves from (0,0) of that frame (the first
        // moves and the object order change). Measured on the product corpus
        // for PR #35 with the scene-frame build kept as pkg-origin: object
        // order changed on 11 of 50 plates past the first, moves on 16 beyond
        // the engine's run-to-run noise; extents and settings unchanged.
        const Slic3r::Vec2d plate_origin = Slic3r::Vec2d::Zero();   // the Print's origin
        print.set_plate_origin(Slic3r::Vec3d(0.0, 0.0, 0.0));
        // The plate's own per-plate values (wipe_tower_x/y are one entry per
        // plate, read with get_at(m_plate_index): Print.cpp 2213, GCode.cpp 5006)
        // need the plate's index, as the desktop sets it (PartPlate.cpp:2299).
        // Left at 0, plate N's prime tower stood where plate 1's does.
        if (plate_id > 0)
            print.set_plate_index(plate_id - 1);

        // Alias the unbound `initial_no_support_filament_id` placeholder to the
        // engine-bound `initial_no_support_extruder` across custom-gcode keys, BEFORE
        // print.apply snapshots the config. Without this the PlaceholderParser throws at
        // export when a 3MF carries the legacy token in its custom gcode. Always on;
        // suppressed with --no-normalize-legacy-gcode.
        if (normalize_legacy_gcode) {
            normalize_legacy_gcode_tokens(config);
        }

        // Calibration: whether the active printer speaks the Bambu G-code dialect.
        // Mirrors the is_BBL_printer() derivation used below for the engine.
        const bool calib_is_bbl_machine =
            config.opt_string("printer_model", true).rfind("Bambu Lab", 0) == 0;

        // Calibration: pressure_advance_pattern generates its own geometry + per-layer
        // custom G-code. This MUST run before print.apply (apply snapshots the
        // model + config and reads plates_custom_gcodes). `calib_params` is a
        // main-scope local, so the reference held by model.calib_pa_pattern
        // stays valid through the whole slice.
        if (calib_params.mode == Slic3r::CalibMode::Calib_PA_Pattern) {
            std::cout << "Generating pressure-advance pattern geometry...\n";
            slicer_cli::apply_pa_pattern(calib_params, config, model, calib_is_bbl_machine);
            // The pattern replaced the model, so the object STL files an
            // --export-stl after --slice writes carry its object's name
            // (obj_1_pa_pattern_handle.stl): the outputs still to be written
            // are checked again. One given before --slice has written the
            // loaded model's files already.
            rs.outputs_checked = false;
            if (!outputs_ok(/*after_slice_actions_only=*/true))
                return 1;
        }

#ifdef ENGINE_BAMBU
        // Align every per-filament array with the filament roster the project
        // loaded (see align_per_filament_config_vectors above), using the count
        // captured before the normalization pass padded vector options.  This is
        // the one and only alignment, and it is the last config write before
        // print.apply() snapshots the config, so every layer that can reshape a
        // per-filament array is already in place: the plate map overlay and the
        // nozzle-map derivation, the legacy-gcode token aliasing, the calibration
        // pattern, and the command-line overrides — an override such as --temp
        // rewrites nozzle_temperature down to a single value, so aligning any
        // earlier would leave that array short again.
        align_per_filament_config_vectors(config, project_filaments, verbose);
#endif // ENGINE_BAMBU

        // Automatic filament grouping on a machine with several extruders sizes
        // each nozzle's group from extruder_ams_count (FilamentGroupUtils.cpp
        // calc_max_group_size, both engines). The desktop app takes that count
        // from the printer it is connected to, else from the project it opened,
        // and builds no AMS slots without a printer (BambuStudio PresetBundle.cpp
        // 3826-3831 and 3547, Plater.cpp 10810-10826 at 02.08.02.61; OrcaSlicer
        // PresetBundle.cpp 4375-4379 and 4162 at 2.4.2). Headless there is no
        // printer, so the input's own count stands, and input without one groups
        // as the desktop does with an empty count. The official CLIs instead
        // give every nozzle an estimated 4-slot AMS (BambuStudio.cpp 6911-6950,
        // OrcaSlicer.cpp 5914-5951), which regroups a project the maker sliced
        // with another count.

        try {
            std::cout << "Applying configuration...\n";
#ifdef ENGINE_ORCA
            // Last write before Print::apply snapshots the config: size every
            // filament vector to the roster the project declares, the way the
            // desktop's Preset::normalize does. Print::get_physical_unprintable_
            // filaments reads filament_printable unchecked
            // (Print.cpp:3229 at v2.4.2); the Bambu build's own alignment
            // (align_per_filament_config_vectors above) already ran on its path.
            if (size_t grown = normalize_filament_config_vectors(config); grown > 0 && verbose)
                std::cout << "Filament vectors: " << grown << " array(s) sized to the roster\n";
#endif
#ifdef ENGINE_SNAPMAKER
            // The plate's default tower corner for a run from a model file
            // (cli_snapmaker.hpp: when nothing else placed the tower).
            // wipe_tower_x/y are official setting flags, so a value from the
            // command line shows up in the given-flag set, not in `overrides`.
            const bool user_set_tower = o.given_flag("wipe_tower_x") || o.given_flag("wipe_tower_y");
            if (slicer_cli::snapmaker_tower_default_applies(config, !boost::algorithm::iends_with(input_file, ".3mf"),
                                                            user_set_tower, o.given_flag("arrange"))) {
                // The same writer the assemble and fixed-plate paths use, so the
                // corner and its margin live in one place.
                assemble_tower_default_corner(context, config, std::max(0, plate_id - 1),
                                              WIPE_TOWER_MARGIN + arrange_opt(config, "prime_tower_width")->getFloat());
            }
#endif
#ifdef ENGINE_SNAPMAKER
            // A switch to a printer that is not a Snapmaker U1 trims the filament
            // list to that printer's extruder count and leaves the painted ids
            // clamped to it (references/SnapmakerOrca/src/slic3r/GUI/Tab.cpp:2247-2270
            // and Plater.cpp:22281-22290 at bac6fd5: on_filaments_change runs
            // remap_extruder_ids, else update_extruder_count). The staged 3MF path
            // never reaches resolve_named_presets' setters, so the trim happens
            // here, on the settings this plate slices with, with the same rule.
            if (slicer_cli::switch_trims_filaments(o.preset_switch, o.printer_preset))
                slicer_cli::trim_filaments_for_switch(config, model, o.printer_preset, plate_id, rs.switch_paint_remapped);
#endif
#ifdef ENGINE_SNAPMAKER
            // The desktop's flow-type sync and the U1's plate types change the
            // settings the slice runs with, so they come before Print::apply,
            // which snapshots the config.
            {
                // The filaments this plate's objects use, as the dialog lists
                // them (FilamentGroupDialog.cpp 352-372).
                std::vector<int> plate_filaments;
                {
                    PlateScope scope(model, rs.plate(plate_id));
                    plate_filaments = arrange_plate_extruders(model, config, std::max(0, plate_id - 1));
                }
                const std::string why = slicer_cli::sync_filament_volume_types(
                    config, plate_id, std::set<int>(plate_filaments.begin(), plate_filaments.end()),
                    extra.has("filament_volume_type"));
                if (!why.empty()) {
                    std::cerr << "Error: " << why << "\n";
                    set_outcome_failure(outcome, CLI_INVALID_PARAMS, why);
                    return 1;
                }
            }
            slicer_cli::force_u1_bed_type(config, plate_id);
#endif
            {
                // The plate's instances only, on the plate (see the Print's
                // origin above). Print::apply copies the model it is given
                // (PrintApply.cpp 1553 / 1298), so the run's model stays in
                // the scene.
                PlateScope scope(model, rs.plate(plate_id));
                // Every height range carries a layer height before the engine
                // builds the layer height profile: it reads one out of each range
                // without a check (Slicing.cpp layer_height_profile_from_ranges,
                // `it_range->second.option("layer_height")->getFloat()`), and a
                // range from an assemble list or a 3MF that states
                // only its settings would crash the slice. A range that states none
                // keeps the object's height, so nothing about the print changes.
                // Filled for this plate's objects with this plate's settings, and
                // taken out again when the scope ends: Print::apply copies the
                // model, so the print keeps the heights, while the run's model
                // stays as it was read for the next plate and the next pass.
                struct RangeFill {
                    std::vector<slicer_cli::FilledRangeHeight> filled;
                    ~RangeFill() { slicer_cli::clear_filled_range_layer_heights(filled); }
                } range_fill{slicer_cli::fill_range_layer_heights(model, config)};
                const auto mm = [](double v) { std::ostringstream s; s << v; return s.str(); };
                for (const slicer_cli::FilledRangeHeight& filled : range_fill.filled)
                    emit_event({{"event", "config_normalized"}, {"tag", "RangeLayerHeightFilled"},
                                {"object", filled.object},
                                {"min_z", filled.min_z}, {"max_z", filled.max_z},
                                {"layer_height", filled.layer_height},
                                {"message", "The height range " + mm(filled.min_z) + "-" + mm(filled.max_z) +
                                                " mm of '" + filled.object +
                                                "' states no layer height; it keeps the object's " +
                                                mm(filled.layer_height) + " mm."}});
                print.apply(model, config);
#ifdef ENGINE_ORCA
                // The Orca desktop applies again before Slice (Plater.cpp 15855/15893 at v2.4.2).
                // On Bambu a second apply moves the output away from the desktop's own G-code (7
                // corpus files with the desktop's G-code keep enable_prime_tower=1), so not there.
                // Applied again, as the desktop does before every slice: its
                // background-process timer applies (OrcaSlicer Plater.cpp
                // 5035-5038, 7983 at v2.4.2), then Slice applies once more
                // (reslice, 15855 -> 15893). Print::apply counts the used
                // filaments (PrintApply.cpp 1131, 1620) before it builds the
                // regions (1730), so on the first apply a filament used only
                // through a feature setting (top_surface_filament_id and the
                // like, PrintRegion.cpp 89-90) is not counted and
                // normalize_fdm_2 turns the prime tower off (PrintConfig.cpp
                // 8692); on the second the regions exist and the tower stays
                // as the settings state it. The official CLI applies once
                // (OrcaSlicer.cpp 6040) and drops the tower. Nothing else of
                // the model or the settings changes between the two calls.
                // What the second call changes in the settings the print runs
                // with is told (PrintReapplied); when it changes nothing, the
                // print is the one the first call made.
                const Slic3r::PrintConfig first_config = print.config();
                print.apply(model, config);
                const Slic3r::t_config_option_keys reapplied = first_config.diff(print.config());
                if (!reapplied.empty()) {
                    json values = json::object();
                    for (const std::string& key : reapplied)
                        values[key] = print.config().opt_serialize(key);
                    emit_event({{"event","config_normalized"}, {"tag","PrintReapplied"},
                                {"plate_id", plate_id}, {"settings", values},
                                {"message","The settings applied a second time before slicing, as the desktop "
                                           "app does, changed " + boost::algorithm::join(reapplied, ", ") +
                                           ": the plate's filaments are counted with its regions"}});
                }
#endif
                // What the load replaced and the print, after its last apply,
                // used other than the file's value (cli_project_load.hpp). The
                // setting flags count by the settings they write: a short flag
                // writes other keys than its own name (--infill writes
                // sparse_infill_density, --bed-temp the plate type's keys).
                if (!outcome.settings_replaced_by_presets.empty()) {
                    std::set<std::string> command_line;
                    for (const std::string& key : rs.extra.keys())
                        command_line.insert(key);
                    Slic3r::DynamicPrintConfig written;
                    if (const Slic3r::ConfigOption* bed = config.option("curr_bed_type"))
                        written.set_key_value("curr_bed_type", bed->clone());
                    apply_command_line_overrides(written, o.overrides, /*report_rejections=*/false);
                    for (const std::string& key : written.keys())
                        if (key != "curr_bed_type" || o.overrides.count(key) != 0)
                            command_line.insert(key);
                    for (const auto& override_value : o.overrides)
                        command_line.insert(override_value.first);
                    slicer_cli::keep_replaced_printed_differently(outcome.settings_replaced_by_presets,
                                                                  outcome.replaced_file_values,
                                                                  print.full_print_config(), command_line);
                }
            }
#ifdef ENGINE_ORCA
            // OrcaSlicer's WipeTowerData::height is read on a path that never
            // writes it, and a value out of Clipper's range then reaches the
            // skirt. Nothing in the engine at 31f6803 initialises it: the struct
            // declares `float height;` with no initialiser (Print.hpp:763) and
            // its constructor, `WipeTowerData(ToolOrdering &) : tool_ordering(...)
            // { clear(); }` (Print.hpp:784), calls a clear() that resets depth,
            // brim_width and rib_offset but not height (Print.hpp:767-778); the
            // only writer, Print::_make_wipe_tower, sets it in its WipeTower2
            // (Type-2) branch alone (Print.cpp:3449), and a Bambu Lab printer
            // always takes the Type-1 branch (wipe_tower_type() is
            // is_BBL_printer() ? Type1 : m_config.wipe_tower_type.value,
            // Print.hpp:1072; is_wipe_tower_type2 at Print.cpp:3196, the
            // `if (!is_wipe_tower_type2)` branch at 3242, which sets depth,
            // brim_width, bbx and rib_offset at 3338-3342 and nothing else). No
            // other writer exists: OrcaSlicer.cpp never touches it, and the
            // desktop only reads it (GLCanvas3D.cpp:2874, 2894).
            // 0 is the value upstream itself gives this field: OrcaSlicer main's
            // WipeTowerData::clear() sets `height = 0.f;` beside the other
            // floats, which is exactly what our pin's clear() is missing. Set
            // here because the engine submodule cannot be patched.
            // Print::first_layer_wipe_tower_corners() reads it unconditionally
            // (Print.cpp:2838) as the height of the stabilization cone — a
            // WipeTower2-only feature, whose base radius is
            // tan(deg2rad(wipe_tower_cone_angle/2)) * height
            // (WipeTower2.cpp:2161-2175) — and adds the ring of points at that
            // radius to the skirt's hull (Print.cpp:2840-2844, appended at
            // 2647), so an indeterminate height puts a point past Clipper's
            // range and Print::_make_skirt's offset() throws
            // clipperException "Coordinate outside allowed range"
            // (Print.cpp:2695, ClipperBase::AddPathInternal). The Type-1 tower
            // (GCode/WipeTower.cpp) has no cone at all, so 0 is also the radius
            // its path means: the ring collapses onto the tower's centre and the
            // tower's own bbox, already in the hull, is unchanged. Measured on
            // the packaged macOS Orca engine of run 37538132874: the E4 project's
            // plate 2 either slices or dies with that exception run to run, and
            // setting wipe_tower_cone_angle to 0 makes every failing input slice.
#ifdef ENGINE_SNAPMAKER
            // Snapmaker Orca has no WipeTowerType/wipe_tower_type() (absent at
            // bac6fd5) and always builds the Type-2 tower
            // (`bUseWipeTower2 = is_BBL_printer() ? false : true`,
            // references/SnapmakerOrca/src/libslic3r/Print.cpp:3221), whose
            // branch writes WipeTowerData::height — so the workaround below,
            // which is for the Type-1 path, does not apply.
#else
            if (print.has_wipe_tower() && print.wipe_tower_type() == Slic3r::WipeTowerType::Type1)
                const_cast<Slic3r::WipeTowerData&>(print.wipe_tower_data()).height = 0.f;
#endif
#endif

            // Calibration: install the calibration params after apply (which resets print
            // state) and before validate/process so the engine's per-layer calib
            // emission (GCode.cpp) and PA-line/pattern paths see the mode.
            if (calib_enabled) {
                print.set_calib_params(calib_params);
                std::cout << "Calibration mode: "
                          << slicer_cli::calib_mode_name(calib_params.mode)
                          << " [start=" << calib_params.start
                          << " end=" << calib_params.end
                          << " step=" << calib_params.step << "]\n";
            }

            // Install the structured-warning emitter BEFORE validate() / process()
            // so the TS host sees pre-slice diagnostics in the same JSON-line
            // stream as slicing-time warnings. Pure stdout side-effect — does
            // not change print state, exit codes, or G-code output.
            // --pipe: the steps before the first plate (BambuStudio.cpp
            // 1765-1767, 4923-4926, 6455-6458: 1 at the start, 2 once the
            // files are loaded, 3 before the plates), then this plate
            // (7034-7046: its index and the run's plate count, 4 to begin),
            // and every status the slice reports (cli_status_callback).
            const bool piped = slicer_cli::pipe_started();
            // The check pass sends no plate step: the official reports a plate
            // only once it slices it (BambuStudio.cpp 7034-7046, after the
            // check pass's continue at 7027).
            if (piped && !outcome.pre_check) {
                pipe_prepare_slicing(rs);   // once per run: here only for a run that skipped the plate checks
                slicer_cli::pipe_set_plate_info(std::max(1, outcome.plate_id), std::max(1, outcome.plate_count));
                slicer_cli::pipe_update(4, "Slicing begins");
            }
            if (o.progress || piped) {
                print.set_status_callback([&outcome, &o, piped](const Slic3r::PrintBase::SlicingStatus& status) {
                    if (piped)
                        slicer_cli::pipe_update(status.percent, status.text, status.warning_step);
                    emit_status_warning(status);
                    if (!o.progress)
                        return;
                    using FB = Slic3r::PrintBase::SlicingStatus::FlagBits;
                    const bool is_warning =
                        (status.flags & (FB::UPDATE_PRINT_STEP_WARNINGS | FB::UPDATE_PRINT_OBJECT_STEP_WARNINGS)) != 0;
                    if (status.warning_step != -1 || is_warning) {
                        // The official CLI keeps every warning (g_slicing_warnings,
                        // cli_status_callback) and reports the last one per plate.
                        if (!status.text.empty() &&
                            status.message_type != Slic3r::PrintStateBase::SlicingDefaultNotification) {
                            outcome.warning_message = status.text;
                            outcome.warnings.push_back(json{{"message", status.text},
                                                            {"level", warning_level_tag(status.warning_level)},
                                                            {"tag", slicing_notification_tag(static_cast<int>(status.message_type))}});
                        }
                        return;
                    }
                    if (status.percent >= 0)
                        emit_progress(outcome, status.percent, status.text);
                });
            } else {
                print.set_status_callback(emit_status_warning);
            }

            // The per-filament and speed tables the brim width reads
            // (Brim.cpp, through Model::extruderParamsMap and
            // printSpeedMap): both official CLIs fill them between apply()
            // and process() (BambuStudio.cpp 7074-7075 at 5873b5f;
            // OrcaSlicer.cpp 6065-6066 at 31f6803). Left empty, the brim
            // takes no account of the filament (TPU narrower, PETG/PCTG
            // wider) or of the print speed.
            {
                int filament_count = 1;
                if (auto* fc = config.option<Slic3r::ConfigOptionStrings>("filament_colour", false))
                    filament_count = std::max<int>(1, (int)fc->values.size());
                Slic3r::Model::setExtruderParams(config, filament_count);
                Slic3r::Model::setPrintSpeedTable(config, print.config());
#ifdef ENGINE_ORCA
                std::cout << "  [orca] setExtruderParams/setPrintSpeedTable (filaments="
                          << filament_count << ")\n";
#endif
            }
            // --no-check and --allow-mix-temp (BambuStudio.cpp 6979-6982;
            // OrcaSlicer.cpp 5968-5971); only when given, so the default
            // stays the engine's own.
            const slicer_cli::PlateLoopSwitches switches = slicer_cli::plate_loop_switches(o);
            outcome.no_check = switches.no_check;
            if (o.given_flag("no_check"))
                print.set_no_check_flag(switches.no_check);
#ifdef ENGINE_SNAPMAKER
            // Snapmaker Orca has no setter for this check (no
            // set_check_multi_filaments_compatibility in its Print at
            // bac6fd5, Print.hpp:1020), so --allow_mix_temp has no engine hook.
#elif defined(ENGINE_BAMBU)
            // Bambu 2.9.1 dropped the plate-level high/low temperature mix check and
            // its setter (no set_check_multi_filaments_compatibility in Print at
            // v02.09.01.70; the official CLI parses allow_mix_temp and no longer uses
            // it). The flag is kept for callers that pass it; the run says so once,
            // at its start (AllowMixTempIgnored, run_cli_slice).
#else
            if (o.given_flag("allow_mix_temp"))
                print.set_check_multi_filaments_compatibility(!switches.allow_mix_temp);
#endif
#ifdef ENGINE_SNAPMAKER
            // The desktop's notes after apply and its PLA/PETG warning; neither
            // stops the slice.
            slicer_cli::post_apply_filament_notes(print, config, plate_id);
#endif
#ifdef ENGINE_SNAPMAKER
            // The Snapmaker desktop's own pre-slice blockers
            // (references/SnapmakerOrca/src/slic3r/GUI/Plater.cpp:21485-21530 at
            // bac6fd5): the high/low-temperature mix, the Cool Steel Plate, then
            // the zero flow ratio. They replace the OrcaSlicer CLI's type-based
            // Print::check_multi_filaments_compatibility, which the desktop does
            // not use. --allow-mix-temp lifts the temperature rule only.
            {
                int blocker_code = 0;
                std::string blocker_message;
                // Only this plate's objects count, as the desktop's
                // collect_used_filament_slots_on_plate counts them.
                int blocked = 0;
                {
                    PlateScope scope(model, rs.plate(plate_id));
                    blocked = slicer_cli::snapmaker_pre_slice_check(model, config, plate_id, switches.allow_mix_temp,
                                                                    blocker_code, blocker_message);
                }
                if (blocked != 0) {
                    std::cerr << blocker_message << "\n";
                    set_outcome_failure(outcome, blocker_code, blocker_message);
                    return 1;
                }
            }
#endif

#ifdef ENGINE_BAMBU
            // Mixed nozzle diameters (BambuStudio.cpp 5066-5086 at v02.09.01.70):
            // with multi_nozzle_diameter_mode on (the project's, or the command
            // line's), a plate whose filaments all
            // print with one nozzle diameter is refused (the single-diameter
            // map was refused in the settings merge).
            if (Slic3r::multi_nozzle_diameter_mode_enabled(config)) {
                // Only this plate's objects count, and the plate's tool changes
                // are keyed 0-based (plates_custom_gcodes, bbs_3mf.cpp 3872).
                // A model file has one plate, numbered 1 (plate_id 0 here, as
                // curr_plate_index = max(0, plate_id - 1) below).
                const int plate_number = std::max(1, plate_id);
                std::vector<int> plate_filaments;
                {
                    PlateScope scope(model, rs.plate(plate_id));
                    plate_filaments = arrange_plate_extruders(model, config, plate_number - 1);
                }
                const std::string single_diameter = slicer_cli::plate_single_nozzle_diameter(config, plate_filaments);
                if (!single_diameter.empty()) {
                    const std::string detail = "Plate " + std::to_string(plate_number) + " uses only nozzle diameter " +
                                               single_diameter + " with multi_nozzle_diameter_mode on.";
                    emit_event({{"event","config_refused"},
                                {"tag","PlateSingleNozzleDiameter"},
                                {"plate", plate_number},
                                {"nozzle_diameter", single_diameter},
                                {"message", cli_error_sentence(CLI_MULTI_NOZZLE_DIAMETER_INVALID) + " " + detail}});
                    std::cerr << "Error: " << cli_error_sentence(CLI_MULTI_NOZZLE_DIAMETER_INVALID) << " " << detail << "\n";
                    set_outcome_failure(outcome, CLI_MULTI_NOZZLE_DIAMETER_INVALID, detail);
                    return 1;
                }
            }
#endif
            std::cout << "Validating...\n";
            // Option C — pass a warning out-pointer so BBS routes is_warning-
            // flagged exceptions there instead of into the return value
            // (mirrors `Plater.cpp:9888`'s call signature). Hard validation
            // errors still come back as the return value and remain fatal.
            // Soft warnings get emitted as JSON events and slicing proceeds —
            // matching what BBS GUI does in this case.
            Slic3r::StringObjectException validation_warning;
            // The GUI calls validate() with all three out-params
            // (Plater.cpp:10835) and paints the returned hulls on the plate;
            // a human sees the collision as geometry, with no text anywhere.
            // Passing them here is the only way an agent can learn WHERE a
            // sequential-print clearance violation is, rather than just that
            // one happened.
            Slic3r::Polygons collision_polygons;
            std::vector<std::pair<Slic3r::Polygon, float>> height_polygons;
#ifdef ENGINE_BAMBU
            // Bambu 2.9.1 validate() returns every warning (Print.hpp 948 at
            // v02.09.01.70); the official CLI joins them with newlines (BambuStudio.cpp 7531-7536).
            std::vector<Slic3r::StringObjectException> validation_warnings;
            Slic3r::StringObjectException validation_result =
                print.validate(&validation_warnings, &collision_polygons, &height_polygons);
            for (const Slic3r::StringObjectException& item : validation_warnings) {
                if (validation_warning.string.empty()) validation_warning = item;
                else {
                    if (validation_warning.string.back() != '\n') validation_warning.string += "\n";
                    validation_warning.string += item.string;
                }
            }
#else
            Slic3r::StringObjectException validation_result =
                print.validate(&validation_warning, &collision_polygons, &height_polygons);
#endif
            if (!collision_polygons.empty() || !height_polygons.empty()) {
                json e;
                e["event"] = "clearance_violation";
                e["tag"]   = "SequentialPrintClearance";
                e["collision_hull_count"] = collision_polygons.size();
                e["height_hull_count"]    = height_polygons.size();
                json hulls = json::array();
                for (const auto& poly : collision_polygons) {
                    json points = json::array();
                    for (const auto& p : poly.points)
                        points.push_back(json{{"x_mm", Slic3r::unscaled<double>(p.x()) - plate_origin.x()},
                                              {"y_mm", Slic3r::unscaled<double>(p.y()) - plate_origin.y()}});
                    hulls.push_back(points);
                }
                if (!hulls.empty()) e["collision_hulls"] = hulls;
                json heights = json::array();
                for (const auto& [poly, height] : height_polygons) {
                    json points = json::array();
                    for (const auto& p : poly.points)
                        points.push_back(json{{"x_mm", Slic3r::unscaled<double>(p.x()) - plate_origin.x()},
                                              {"y_mm", Slic3r::unscaled<double>(p.y()) - plate_origin.y()}});
                    heights.push_back(json{{"height_mm", height},
                                           {"point_count", poly.points.size()},
                                           {"points", points}});
                }
                if (!heights.empty()) e["height_hulls"] = heights;
                e["message"] = "Object clearance hulls overlap; the GUI would paint these regions red on the plate";
                emit_event(e);
            }
            if (!validation_warning.string.empty()) {
                emit_validation_event(validation_warning);
                std::cout << "Validation warning: " << validation_warning.string << "\n";
            }
            // --no-check lets a layer height over the printer's limit pass
            // as a warning (BambuStudio.cpp 6985-6987; OrcaSlicer.cpp 5992-5994).
            if (!validation_result.string.empty() && switches.no_check &&
                validation_result.type == Slic3r::STRING_EXCEPT_LAYER_HEIGHT_EXCEEDS_LIMIT) {
                emit_validation_event(validation_result);
                std::cout << "Validation warning: " << validation_result.string << "\n";
                validation_result.string.clear();
            }
            if (!validation_result.string.empty()) {
                emit_validation_event(validation_result);
                std::cerr << "Validation error: " << validation_result.string << "\n";
                std::cerr << "\nTip: You may need to specify proper config files:\n";
                std::cerr << "  --machine, --filament, and --process options\n";
                // The official CLI's mapping of validate() errors to result
                // codes (BambuStudio.cpp 7094-7111 at 5873b5f).
                int validate_error = CLI_VALIDATE_ERROR;
                switch (validation_result.type) {
                    case Slic3r::STRING_EXCEPT_FILAMENT_NOT_MATCH_BED_TYPE:      validate_error = CLI_FILAMENT_NOT_MATCH_BED_TYPE; break;
#ifndef ENGINE_SNAPMAKER
                    case Slic3r::STRING_EXCEPT_FILAMENTS_DIFFERENT_TEMP:         validate_error = CLI_FILAMENTS_DIFFERENT_TEMP; break;
#else
                    // Snapmaker Orca calls it STRING_EXCEPT_FILAMENTS_MIXING_TEMP
                    // (references/SnapmakerOrca/src/libslic3r/PrintBase.hpp:1-12
                    // at bac6fd5); the same situation, the same code.
                    case Slic3r::STRING_EXCEPT_FILAMENTS_MIXING_TEMP:            validate_error = CLI_FILAMENTS_DIFFERENT_TEMP; break;
#endif
                    case Slic3r::STRING_EXCEPT_OBJECT_COLLISION_IN_SEQ_PRINT:    validate_error = CLI_OBJECT_COLLISION_IN_SEQ_PRINT; break;
                    case Slic3r::STRING_EXCEPT_OBJECT_COLLISION_IN_LAYER_PRINT:  validate_error = CLI_OBJECT_COLLISION_IN_LAYER_PRINT; break;
                    default: break;
                }
                // The engine's own sentence names the symptom ("Add \"G92 E0\"
                // to layer_gcode"). When the run has no layer change G-code at
                // all, the cause is that one: the settings files state none, so
                // the merge wrote the key empty (load_default_gcodes_to_config,
                // BambuStudio.cpp 685-745 at 5873b5f / OrcaSlicer.cpp 548-604 at
                // 31f6803 makes every G-code key the file leaves out an empty
                // string). Name it, so the refusal says what to give instead of
                // what to edit in a file the user does not have.
                std::string message = validation_result.string;
                if (message.rfind("Relative extruder addressing", 0) == 0) {
                    const auto* layer_gcode = config.option<Slic3r::ConfigOptionString>("layer_change_gcode");
                    if (layer_gcode == nullptr || layer_gcode->value.empty())
                        message += " This run has no layer change G-code at all: the machine settings given state no"
                                   " layer_change_gcode, so the settings merge wrote it empty. Give a machine settings"
                                   " file that states its layer_change_gcode, or the printer preset"
                                   " (--printer-preset).";
                }
                // The engine's own sentence names the plate and the filament
                // and stops there ("Plate 1: Cool Plate does not support
                // filament 2"). Its check reads the plate's own bed temperature
                // key with get_at and reads 0 as "not this filament's plate", for every
                // filament the run prints with (Print.cpp 1700-1726 at 31f6803;
                // Print.cpp 1650-1672 at 5873b5f; the filament list is
                // Print::extruders(), Print.cpp 1265 / 1307; get_bed_temp_key,
                // OrcaSlicer PrintConfig.hpp 466-490 / BambuStudio
                // PrintConfig.hpp 438-456). Name the plates whose key is not 0
                // for every one of them — the plates this run can actually
                // print on — so the refusal says what to give instead.
                if (validation_result.type == Slic3r::STRING_EXCEPT_FILAMENT_NOT_MATCH_BED_TYPE) {
                    const std::vector<unsigned int> used = print.extruders();
                    const Slic3r::ConfigOptionDef*    bed_def = Slic3r::print_config_def.get("curr_bed_type");
                    std::vector<std::string>          plates;
                    if (!used.empty() && bed_def != nullptr && bed_def->enum_keys_map != nullptr) {
                        // enum_values is the plate list in the definition's own
                        // order (PrintConfig.cpp 1049-1053 at 31f6803,
                        // 1150-1154 at 5873b5f), so the list is stable.
                        for (const std::string& plate : bed_def->enum_values) {
                            const auto named = bed_def->enum_keys_map->find(plate);
                            if (named == bed_def->enum_keys_map->end())
                                continue;
                            const std::string temp_key =
                                Slic3r::get_bed_temp_key(static_cast<Slic3r::BedType>(named->second));
                            if (temp_key.empty())
                                continue;
                            const auto* temps = config.option<Slic3r::ConfigOptionInts>(temp_key);
                            if (temps == nullptr || temps->values.empty())
                                continue;
                            // Read as the check reads it: get_at, so a list with
                            // one value is that value for every filament.
                            bool every_filament = true;
                            for (unsigned int filament : used)
                                if (temps->get_at(filament) == 0) {
                                    every_filament = false;
                                    break;
                                }
                            if (every_filament)
                                plates.push_back(plate);
                        }
                    }
                    if (plates.empty())
                        message += " No plate this engine knows gives every filament of this run a bed temperature:"
                                   " check the plate temperatures in the filament presets (--filament-preset).";
                    else {
                        message += " Pick a plate this filament supports: --curr-bed-type \"" + plates.front() + "\"";
                        if (plates.size() > 1) {
                            message += " (or";
                            for (size_t i = 1; i < plates.size(); i++)
                                message += (i > 1 ? ", " : " ") + std::string("\"") + plates[i] + "\"";
                            message += ")";
                        }
                        message += ".";
                    }
                }
                set_outcome_failure(outcome, validate_error, message);
                return 1;
            }
            // --slice 0 on several plates: the check pass ends here for this
            // plate, once its bed check, apply and validate passed
            // (BambuStudio.cpp 7027-7028; OrcaSlicer.cpp 6034-6035).
            if (outcome.pre_check)
                return 0;

            // --load-slicedata: the plate's saved slicing, else a normal
            // slice (BambuStudio.cpp 7076-7099; OrcaSlicer.cpp 6067-6090).
            const int cache_plate = plate_id > 0 ? plate_id : 1;
            const std::string load_dir = slicer_cli::slicedata_dir(o, "load_slicedata", cache_plate);
#ifdef ENGINE_SNAPMAKER
            // Again here: the project rebase and the desktop plate type load
            // presets after the call at the top of this function, and each
            // points data_dir() at a staging folder its scope then removes.
            slicer_cli::snapmaker_keep_data_dir(run_backup_path(context));
#endif
            std::cout << "Slicing...\n";
            if (!load_dir.empty() && print.load_cached_data(load_dir) == 0) {
                emit_event({{"event","model_loaded"}, {"tag","SliceDataLoaded"}, {"path", load_dir},
                            {"message","Loaded the saved slicing of plate " + std::to_string(cache_plate)}});
                slicer_cli::pipe_update(69, "Cache data loaded");   // BambuStudio.cpp 7087-7090
                print.process(nullptr, true);
            } else {
                if (!load_dir.empty())
                    emit_event({{"event","model_loaded"}, {"tag","SliceDataNotLoaded"}, {"path", load_dir},
                                {"message","No usable saved slicing for plate " + std::to_string(cache_plate) + "; slicing it normally"}});
                print.process();
            }

            std::cout << "\n✓ Slicing complete!\n";

            // --slice: toolpath conflicts stop the plate before any G-code
            // is written, as the official CLI checks them right after
            // process() and exits before export_gcode (BambuStudio.cpp
            // 7111-7116 at 5873b5f). Print::process() sets the conflict
            // result in both engines (Print.cpp 2396 / 2553).
            if (o.slice_mode) {
                const std::string conflict = print.get_conflict_string();
                if (!conflict.empty()) {
                    set_outcome_failure(outcome, CLI_GCODE_PATH_CONFLICTS, conflict + ".");
                    emit_event({{"event","plate_error"}, {"tag","GcodePathConflicts"}, {"message", conflict}});
                    return 1;
                }
            }

            // Attempt G-code export
            std::cout << "\nExporting G-code to: " << output_file << "\n";

            try {
                /// Export G-code with a result object (must not be nullptr — GCode.cpp:1738 dereferences it)
                /// Print.hpp:848
                /// C++: std::string export_gcode(const std::string &path, GCodeProcessorResult* result, ThumbnailsGeneratorCallback thumbnail_cb);
                Slic3r::GCodeProcessorResult gcode_result;
                // GCode::do_export makes the file's folder when it is missing and
                // throws on a bare file name, whose folder is "" (GCode.cpp
                // 1793-1797 at 5873b5f): the current folder is named for it.
                const std::string export_path = boost::filesystem::path(output_file).has_parent_path()
                                                    ? output_file : (boost::filesystem::path(".") / output_file).string();
                print.export_gcode(export_path, &gcode_result, nullptr);

                // The result object is fully populated by the export. Drain
                // every check the GUI would show a human before reporting
                // success — a run can exit 0 and still carry warnings,
                // toolpath conflicts or printable-area failures.
                emit_gcode_result_diagnostics(gcode_result);
                if (o.slice_mode) {
                    record_plate_statistics(print, model, config, rs.base, gcode_result, outcome);
                    outcome.sliced_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - plate_started).count();
                    // --mstpp: a plate that took longer ends the run, listed with
                    // its note (BambuStudio.cpp 7364-7373, a limit in seconds;
                    // OrcaSlicer.cpp 6202-6211 compares the milliseconds with the
                    // value as given).
                    if (switches.max_slicing_time_s != 0) {
#ifdef ENGINE_ORCA
                        const long long limit = switches.max_slicing_time_s;
#else
                        const long long limit = (long long)switches.max_slicing_time_s * 1000;
#endif
                        if (outcome.sliced_time_ms > limit) {
                            outcome.warning_message = "plate " + std::to_string(cache_plate) + "'s slice time " +
                                                      std::to_string(outcome.sliced_time_ms) + " exceeds the limit " +
                                                      std::to_string(limit) + ", return error.";
                            outcome.exported = true;
                            set_outcome_failure(outcome, CLI_SLICING_TIME_EXCEEDS_LIMIT);
                            return 1;
                        }
                    }
                    if (!post_slice_checks(print, model, gcode_result, outcome))
                        return 1;
                    outcome.exported = true;
                    if (!o.export_3mf.empty())
                        record_plate_for_export(context, print, model, config, gcode_result, output_file, outcome,
                                                plate_origin);
                    if (o.progress)
                        emit_progress(outcome, 100, "Slicing finished");
                    slicer_cli::pipe_update(100, "Slicing finished");   // BambuStudio.cpp 7242-7245
                }
                {
                    // --export-slicedata (BambuStudio.cpp 7247-7262; OrcaSlicer.cpp 6183-6199).
                    if (const std::string export_dir = slicer_cli::slicedata_dir(o, "export_slicedata", cache_plate);
                        !export_dir.empty()) {
                        const bool with_space = Slic3r::get_logging_level() >= 4;
                        // Print::export_cached_data makes the plate's folder
                        // only, so the folder given on the command line is made
                        // first (the official fails with -53 when it is missing).
                        {
                            boost::system::error_code mk;
                            boost::filesystem::create_directories(boost::filesystem::path(export_dir).parent_path(), mk);
                        }
#ifdef ENGINE_ORCA
                        const int ret = print.export_cached_data(export_dir, with_space);
#else
                        int obj_cached_cnt = 0;
                        const int ret = print.export_cached_data(export_dir, obj_cached_cnt, with_space);
#endif
                        if (ret != 0) {
                            boost::system::error_code rm;
                            boost::filesystem::remove_all(export_dir, rm);
                            set_outcome_failure(outcome, ret, "Could not write the slicing data to " + export_dir + ".");
                            return 1;
                        }
                        emit_event({{"event","export"}, {"tag","SliceDataExported"}, {"path", export_dir},
                                    {"message","Wrote the slicing data of plate " + std::to_string(cache_plate) + " to " + export_dir}});
                    }
                }

                std::cout << "✓ G-code export complete!\n";
                std::cout << "\nOutput file: " << output_file << "\n";

                // The actions given after --slice, once the last plate has
                // sliced (see run_model_action_step).
                if (o.slice_mode && outcome.plate_index == outcome.plate_count &&
                    slicer_cli::has_model_actions(o, slicer_cli::ActionPhase::AfterSlice) &&
                    !run_model_action_step(slicer_cli::ActionPhase::AfterSlice))
                    return 1;

            } catch (const std::bad_alloc&) {
                // Out of memory while slicing or exporting. The official CLI has
                // this outcome of its own -- CLI_OUT_OF_MEMORY (-14, Utils.hpp:39
                // at 31f6803) with the sentence that says what to do
                // (OrcaSlicer.cpp 126, BambuStudio.cpp 132) -- and it is not a
                // slicing failure: a run that ran out of memory must never
                // report CLI_SLICING_ERROR (-100).
                emit_event({{"event","slicing_error"},{"phase","export_gcode"},{"kind","bad_alloc"},
                            {"message","std::bad_alloc"}});
                std::cerr << "\nOut of memory during slicing.\n";
                set_outcome_failure(outcome, CLI_OUT_OF_MEMORY);
                return 1;
            } catch (const Slic3r::RuntimeError& e) {
                emit_event({{"event","slicing_error"},{"phase","export_gcode"},{"kind","RuntimeError"},{"message",std::string(e.what())}});
                slicer_cli::add_engine_reason(outcome.error_reason, e.what());
                std::cerr << "\n❌ G-code export failed (RuntimeError): " << e.what() << "\n";
                std::cerr << "\nSlicing succeeded, but export needs additional configuration.\n";
                return 1;
            } catch (const std::length_error& e) {
                emit_event({{"event","slicing_error"},{"phase","export_gcode"},{"kind","length_error"},{"message",std::string(e.what())}});
                slicer_cli::add_engine_reason(outcome.error_reason, e.what());
                std::cerr << "\n❌ G-code export failed (length_error): " << e.what() << "\n";
                return 1;
            } catch (const std::exception& e) {
                emit_event({{"event","slicing_error"},{"phase","export_gcode"},{"kind",typeid(e).name()},{"message",std::string(e.what())}});
                slicer_cli::add_engine_reason(outcome.error_reason, e.what());
                std::cerr << "\n❌ G-code export failed (" << typeid(e).name() << "): " << e.what() << "\n";
                std::cerr << "\nSlicing succeeded, but export needs additional configuration.\n";
                return 1;
            } catch (...) {
                emit_event({{"event","slicing_error"},{"phase","export_gcode"},{"kind","unknown"},{"message","unknown exception"}});
                std::cerr << "\n❌ G-code export failed (unknown exception)\n";
                return 1;
            }

        } catch (const std::bad_alloc&) {
            // The slicing step ran out of memory (see the export step above):
            // this engine's own CLI has CLI_OUT_OF_MEMORY for it, never -100.
            emit_event({{"event","slicing_error"},{"phase","process"},{"kind","bad_alloc"},
                        {"message","std::bad_alloc"}});
            std::cerr << "\nOut of memory during slicing.\n";
            set_outcome_failure(outcome, CLI_OUT_OF_MEMORY);
            return 1;
        } catch (const Slic3r::RuntimeError& e) {
            emit_event({{"event","slicing_error"},{"phase","process"},{"kind","RuntimeError"},{"message",std::string(e.what())}});
            slicer_cli::add_engine_reason(outcome.error_reason, e.what());
            std::cerr << "Slicing RuntimeError: " << e.what() << "\n";
            std::cerr << "\nThis may indicate missing or incomplete configuration.\n";
            std::cerr << "Try using BambuStudio config files with --machine, --filament, --process\n";
            return 1;
        } catch (const std::length_error& e) {
            emit_event({{"event","slicing_error"},{"phase","process"},{"kind","length_error"},{"message",std::string(e.what())}});
            slicer_cli::add_engine_reason(outcome.error_reason, e.what());
            std::cerr << "Slicing length_error: " << e.what() << "\n";
            return 1;
        } catch (const std::exception& e) {
            emit_event({{"event","slicing_error"},{"phase","process"},{"kind",typeid(e).name()},{"message",std::string(e.what())}});
            slicer_cli::add_engine_reason(outcome.error_reason, e.what());
            std::cerr << "Slicing error (" << typeid(e).name() << "): " << e.what() << "\n";
            std::cerr << "\nThis may indicate missing or incomplete configuration.\n";
            std::cerr << "Try using BambuStudio config files with --machine, --filament, --process\n";
            return 1;
        } catch (...) {
            emit_event({{"event","slicing_error"},{"phase","process"},{"kind","unknown"},{"message","unknown exception"}});
            std::cerr << "Slicing error: unknown exception\n";
            return 1;
        }

        return 0;

    } catch (const std::bad_alloc&) {
        // Out of memory before or around the slicing steps (the load of a large
        // project, the arrange, the plate cleanup). The official outcome is
        // CLI_OUT_OF_MEMORY with its sentence (Utils.hpp:39, OrcaSlicer.cpp 126
        // at 31f6803), never CLI_SLICING_ERROR (-100): this is the run that must
        // tell the user to lower the geometry resolution, not report a slicing
        // failure.
        emit_event({{"event","slicing_error"},{"phase","load_or_prepare"},{"kind","bad_alloc"},
                    {"message","std::bad_alloc"}});
        std::cerr << "\nOut of memory during slicing.\n";
        set_outcome_failure(outcome, CLI_OUT_OF_MEMORY);
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}



/// --export-3mf: the sliced project, written with store_bbs_3mf as the
/// official CLI writes it: export_project(&m_models[0], plate_data_list,
/// project_presets, ..., &m_print_config, minimum_save, plate_to_slice - 1)
/// (BambuStudio.cpp 8156-8157, CLI::export_project 8468-8505 at 5873b5f:
/// Silence | WithGcode | SplitModel | UseLoadedId | ShareMesh; OrcaSlicer.cpp
/// 6985). The model is the run's one model as it was sliced: the transforms,
/// the arrange, the move to a new bed, --repetitions' copies and
/// --skip-objects' unprintable instances are in it. With --slice N it holds
/// plate N's objects only, as the official loads only that plate
/// (BambuStudio.cpp 1889). The plates are the file's (or the run's), each
/// listing its instances, and each sliced plate gets its G-code and slice
/// facts. No thumbnails are rendered (the CLI has no OpenGL), so a project
/// keeps the plate pictures it came with.
static int export_sliced_3mf(CliRunContext& context, const CliOptions& o, const boost::filesystem::path& outdir, RunState& rs,
                             std::vector<PlateOutcome>& outcomes, const Slic3r::DynamicPrintConfig& project,
                             const std::set<std::string>* given_settings, std::string& error) {
    using namespace Slic3r;
    const std::string path = (outdir / o.export_3mf).string();
    Model& model = rs.model;
    // store_bbs_3mf stages the project's files in the model's backup folder
    // (_add_project_config_file_to_archive writes <backup>/_temp_1.config,
    // bbs_3mf.cpp). The run's folder (once slicer_cli_backup, shared by every
    // slicer_cli run on the host, and removed by a run that ends: ~Model,
    // Model.cpp remove_backup) was not the export's, so a run exporting
    // beside another lost its files. The export stages in a folder of its
    // own, as ba0dcfb's export model did. "detach" first: set_backup_path
    // removes the folder it replaces (Model::set_backup_path), which is the
    // load's own (run_backup_path).
    model.set_backup_path("detach");
    const std::string export_backup =
        context.run_staging.add((boost::filesystem::temp_directory_path() /
                         boost::filesystem::unique_path("slicer_cli_export-%%%%%%%%")).string());
    model.set_backup_path(export_backup);   // removed with the run's other staging folders
    // Detached again when this function ends, so this model's destruction does
    // not queue a removal of a folder the run removes itself
    // (detach_run_backup_path).
    const RunBackupDetach detach_export_model{model};
    PlateDataPtrs plates;
    struct Release {
        PlateDataPtrs& plates;
        ~Release() { release_PlateData_list(plates); }
    } release{plates};
    if (outcomes.empty() || !outcomes.front().plate_data) {
        error = "Nothing was sliced to export.";
        return CLI_EXPORT_3MF_ERROR;
    }
    const bool geometry_input = !input_is_3mf(o.input_file) ||
        slicer_cli::classify_3mf(o.input_file) == slicer_cli::ThreeMfKind::GeometryOnly;

    // The plates (PartPlateList::store_to_3mf_structure, BambuStudio.cpp
    // 7483-7484): the file's, with any the arrange added, or one plate for
    // model files and a 3MF without plates.
    const int count = std::max<int>({1, (int)rs.plates.size(), (int)rs.plate_data.size()});
    for (int k = 0; k < count; ++k) {
        PlateData* pd = k < (int)rs.plate_data.size() && rs.plate_data[k] != nullptr ? new PlateData(*rs.plate_data[k])
                                                                                    : new PlateData();
        pd->plate_index = k;
        pd->objects_and_instances.clear();
        plates.push_back(pd);
    }
    // Each plate's instances (PartPlate::obj_to_instance_set, which the
    // writer reads as objects_and_instances, bbs_3mf.cpp 8208, 8328-8350).
    for (size_t oi = 0; oi < model.objects.size(); ++oi)
        for (size_t ii = 0; ii < model.objects[oi]->instances.size(); ++ii) {
            const ModelInstance* inst = model.objects[oi]->instances[ii];
            for (int k = 0; k < count; ++k)
                if (rs.plate(k + 1).is_member(inst) && (k < (int)rs.plates.size() || count == 1)) {
                    plates[k]->objects_and_instances.emplace_back(int(oi), int(ii));
                    break;
                }
        }
    // Each sliced plate's slice result (BambuStudio.cpp 7483-7532).
    for (const PlateOutcome& out : outcomes) {
        if (!out.plate_data) continue;
        const int idx = geometry_input && !context.g_assemble ? 0 : out.plate_id - 1;
        if (idx < 0 || idx >= count) continue;
        PlateData* pd = plates[idx];
        const PlateData& sliced = *out.plate_data;
        pd->gcode_file               = sliced.gcode_file;
        pd->is_sliced_valid          = true;
        pd->gcode_prediction         = sliced.gcode_prediction;
        pd->gcode_weight             = sliced.gcode_weight;
        pd->toolpath_outside         = sliced.toolpath_outside;
        pd->timelapse_warning_code   = sliced.timelapse_warning_code;
        pd->is_label_object_enabled  = sliced.is_label_object_enabled;
#ifndef ENGINE_SNAPMAKER
        // Not in Snapmaker's PlateData at bac6fd5 (see record_plate_for_export).
        pd->limit_filament_maps      = sliced.limit_filament_maps;
        pd->layer_filaments          = sliced.layer_filaments;
        pd->filament_change_sequence = sliced.filament_change_sequence;
        pd->nozzle_change_sequence   = sliced.nozzle_change_sequence;
        pd->optimal_assignment       = sliced.optimal_assignment;
        pd->filament_maps            = sliced.filament_maps;
#endif
        pd->is_support_used          = sliced.is_support_used;
        pd->slice_filaments_info     = sliced.slice_filaments_info;
        pd->skipped_objects          = sliced.skipped_objects;
        pd->printer_model_id         = sliced.printer_model_id;
        pd->nozzle_diameters         = sliced.nozzle_diameters;
#ifndef ENGINE_SNAPMAKER
        // The routing the slice derived from a supplied nozzle map, or the
        // command line's --filament-map, on this plate only: store_bbs_3mf writes a plate's filament_map_mode and
        // filament_maps from its config (bbs_3mf.cpp 8260-8279 at 5873b5f;
        // PartPlate::set_filament_maps keeps the map there, PartPlate.cpp
        // 3860-3863), and a plate mode other than Default wins over the
        // project's (PartPlate.cpp 278-289). The objects keep their own
        // filaments, so an object with copies on other plates stays as it is.
        if (!out.derived_filament_map.empty())
            pd->config.option<ConfigOptionInts>("filament_map", true)->values = out.derived_filament_map;
        if (!out.derived_filament_map_mode.empty()) {
            ConfigSubstitutionContext mode_subst(ForwardCompatibilitySubstitutionRule::Enable);
            pd->config.set_deserialize("filament_map_mode", out.derived_filament_map_mode, mode_subst);
        }
        for (const auto& [key, values] : out.flag_plate_maps)
            pd->config.option<ConfigOptionInts>(key, true)->values = values;
#endif
    }
    // printer_model_id per plate after a printer change (BambuStudio.cpp 7524-7525).
    if (!rs.settings_merge.printer_model_id.empty())
        for (PlateData* pd : plates)
            pd->printer_model_id = rs.settings_merge.printer_model_id;
    if (context.g_assemble)
        for (int k = 0; k < count && k < (int)context.g_assemble->plates.size(); ++k)
            plates[k]->plate_name = context.g_assemble->plates[k].plate_name;

    // The presets the project carries, and the "(auto)" process a printer
    // change makes (BambuStudio.cpp 3094).
    std::vector<Preset*> project_presets;
    for (Preset& p : rs.project_presets_kept)
        project_presets.push_back(&p);
    std::unique_ptr<Preset> auto_process;
    if (rs.settings_merge.new_preset) {
        auto_process = std::make_unique<Preset>(*rs.settings_merge.new_preset);
        project_presets.push_back(auto_process.get());
    }

    // The project's settings are the run's, m_print_config (the official
    // export_project(..., &m_print_config, ...), BambuStudio.cpp 8156-8157;
    // OrcaSlicer.cpp 6985), the ones --export-settings writes and
    // result.json reads, with the towers the arrange placed. A plate's own
    // settings stay the plate's (its PlateData config). The G-code header
    // reads the plate's copy, which also holds this command line's own
    // per-plate adjustments (the prime tower turned off when every filament
    // is the same, the per-filament lists filled to the filament count on the
    // Bambu build): those keys can differ from the project's, as the official
    // app never writes them.
    DynamicPrintConfig config = project;
    // The slice ran its custom G-code with the legacy placeholder aliased
    // (normalize_legacy_gcode_tokens, before print.apply); the project
    // states the same templates. The slice already reported the alias.
    if (!geometry_input && !context.g_assemble && o.normalize_legacy_gcode)
        normalize_legacy_gcode_tokens(config, /*report_event=*/false);
    // The settings this run changed, listed as the desktop lists them when it
    // saves (desktop_presets.hpp): a project is opened over the presets and
    // keeps only what is listed. A project whose presets the package does not
    // ship is opened with its saved values, so its list stays as it is.
    {
        std::string unavailable;
        try {
            const boost::filesystem::path staging = make_owned_temp_dir("slicer_cli_project-");
            struct StagingCleanup {
                boost::filesystem::path dir;
                ~StagingCleanup() { remove_owned_temp_dir(dir); }
            } staging_cleanup{staging};
            PresetBundle bundle;
            if (load_system_presets(o.argv0, bundle, staging, unavailable)) {
#ifdef ENGINE_BAMBU
                slicer_cli::load_project_embedded_presets(bundle, project_presets, config);
#endif
                if (project_presets_shipped(bundle, config_id_text(config, "printer_settings_id"),
                                            config_id_text(config, "print_settings_id")))
                    slicer_cli::record_changed_settings(bundle, path, config, given_settings, unavailable);
            }
        } catch (const std::exception& e) {
            unavailable = e.what();
        }
        if (!unavailable.empty())
            emit_event({{"event","export_note"}, {"tag","ChangedSettingsNotListed"}, {"path", path},
                        {"reason", unavailable},
                        {"message","The project does not list the settings this run changed: " + unavailable}});
    }

    // Each sliced plate's Metadata/plate_N.json; an empty box for the others
    // (BambuStudio.cpp 7976-7988).
    std::vector<std::unique_ptr<PlateBBoxData>> bbox_storage;
    std::vector<PlateBBoxData*> plate_bboxes;
    for (size_t i = 0; i < plates.size(); ++i) {
        PlateBBoxData* found = nullptr;
        for (const PlateOutcome& out : outcomes)
            if (out.plate_bbox && (geometry_input && !context.g_assemble ? i == 0 : out.plate_id - 1 == int(i)))
                found = out.plate_bbox.get();
        bbox_storage.push_back(std::make_unique<PlateBBoxData>(found ? *found : PlateBBoxData()));
        plate_bboxes.push_back(bbox_storage.back().get());
    }
    // --makerlab-*, --metadata-* (BambuStudio.cpp 8137-8154).
    slicer_cli::apply_model_metadata(o, model);

    StoreParams store;
    store.path = path.c_str();
    store.model = &model;
    store.plate_data_list = plates;
    store.project_presets = project_presets;
    store.config = &config;
    store.id_bboxes = plate_bboxes;
    store.strategy = SaveStrategy::Silence | SaveStrategy::WithGcode | SaveStrategy::SplitModel |
                     SaveStrategy::UseLoadedId | SaveStrategy::ShareMesh;
    // --min-save: the project without its model parts (BambuStudio.cpp 8493-8494).
    if (o.given_flag("min_save") && o.cli.opt_bool("min_save"))
        store.strategy = store.strategy | SaveStrategy::SkipModel;
    store.export_plate_idx = o.slice_plate - 1;
    if (!store_bbs_3mf(store)) {
        error = "Writing " + path + " failed.";
        return CLI_EXPORT_3MF_ERROR;
    }
    emit_event({{"event","exported_3mf"}, {"tag","SlicedProjectWritten"}, {"path", path},
                {"message","Wrote the sliced project " + path}});
    // The official CLI renders plate pictures with its GUI's OpenGL code;
    // slicer-cli makes none, so the project has no plate pictures. Said
    // once, here, rather than left for a reader to find missing.
    emit_event({{"event","export_note"}, {"tag","NoPlatePictures"}, {"path", path},
                {"message","The project has no plate pictures: slicer-cli does not render them"}});
    return 0;
}

/// Every file this run writes, checked before the run writes any of them,
/// whatever flags are given: no two of them may be one file (the one written
/// last would replace the other, and the run would report success with a
/// requested output lost), and none may be a file the run reads.
///
/// The outputs: result.json and each sliced plate's plate_<n>.gcode in
/// --outputdir (--slice), the G-code of -o (the single-plate call),
/// --export-3mf (joined onto --outputdir, as the official joins it:
/// export_3mf_file = outfile_dir + "/" + export_3mf_file, BambuStudio.cpp
/// 7508-7510 at 5873b5f; OrcaSlicer.cpp 6302-6304 at 31f6803), the
/// --export-settings file, the object STL files of --export-stl /
/// --export-stls (each object's own name, object_stl_file, read from the
/// loaded `model`) and the plate folders under --export-slicedata.
///
/// It runs twice in a --slice run: once the model is loaded and before the
/// first action writes a file (every output but the plates', whose plan may
/// not be final yet; `plates` holds the ones already known), and again once
/// the plate plan is final, before the first plate is sliced, with every
/// plate. A run without --slice checks once, before its actions and its -o
/// G-code.
///
/// Two outputs are compared by name, the link chain followed and without
/// case on every system (they may not exist yet; a name that differs from
/// another only by letter case is refused too, as an odd combination), and
/// two existing names for one file by fs::equivalent. An output overwrites an
/// input only when it is that file, and the inputs exist, so that check asks
/// the file system (fs::equivalent: links, hard links and the volume's own
/// case rule); an output that does not exist overwrites nothing.
///
/// Returns 0, or CLI_INVALID_PARAMS once the refusal is reported (-2:
/// ExportOverwritesInput for an input, ExportNameTaken for two outputs). With
/// `detail` the sentence is handed back for the caller's result; without it,
/// result.json is written here.
///
/// `reserved` holds outputs already written by actions given before --slice
/// (the check the calibration pattern runs after it, once the model is the
/// pattern): their names are taken, so every output still to come is called
/// out against them, and they are not checked or run again. `reserve` hands
/// the outputs of those earlier actions back for that second check.
static int check_run_outputs(CliRunContext& context, const CliOptions& o, const std::vector<int>& plates, const Slic3r::Model* model,
                             std::string* detail, bool after_slice_actions_only,
                             const std::vector<RunOutput>* reserved,
                             std::vector<RunOutput>* reserve) {
    namespace fs = boost::filesystem;
    const fs::path outdir = o.outputdir.empty() ? fs::path(".") : fs::path(o.outputdir);
    const auto key = [](const fs::path& p) {
        // A link whose target does not exist yet (result.json before the run
        // writes it) is not resolved by weakly_canonical: follow the chain by
        // hand first.
        fs::path q = fs::absolute(p);
        boost::system::error_code ec;
        for (int hops = 0; hops < 40 && fs::is_symlink(q, ec) && !ec; ++hops) {
            const fs::path link = fs::read_symlink(q, ec);
            if (ec) break;
            q = link.is_absolute() ? link : q.parent_path() / link;
        }
        ec.clear();
        const fs::path resolved = fs::weakly_canonical(q, ec);
        return (ec ? q : resolved).lexically_normal().generic_string();
    };
    const auto lower = [](const std::string& s) { return boost::algorithm::to_lower_copy(s); };

    // ── What the run writes ─────────────────────────────────────────────
    std::vector<RunOutput> outputs;
    const auto add = [&](bool folder, bool stl, const std::string& flag, const fs::path& path,
                         const std::string& what, const std::string& is_what, bool before_slice = false) {
        RunOutput out;
        out.folder  = folder;
        out.stl     = stl;
        out.flag    = flag;
        out.path    = path;
        out.what    = what;
        out.is_what = is_what;
        outputs.push_back(std::move(out));
        // An action given before --slice writes its files before the slice:
        // the caller keeps them as reserved for the check the calibration
        // pattern runs after it (see the second check in slice_one_plate).
        if (before_slice && reserve != nullptr)
            reserve->push_back(outputs.back());
    };
    // The outputs a flag names come first: a clash is told from that flag.
    if (!o.export_3mf.empty() && o.slice_mode)
        add(false, false, "--export-3mf " + o.export_3mf, outdir / o.export_3mf,
            "the project --export-3mf " + o.export_3mf + " writes", "the project --export-3mf " + o.export_3mf + " writes");
    // With after_slice_actions_only, the actions given before --slice have
    // run already (their files are written): only those after it are still
    // to write.
    const size_t slice_index = size_t(std::find(o.actions.begin(), o.actions.end(), std::string("slice")) -
                                      o.actions.begin());
    for (size_t action_index = 0; action_index < o.actions.size(); ++action_index) {
        const std::string& action = o.actions[action_index];
        if (after_slice_actions_only && !(slice_index < o.actions.size() && action_index > slice_index))
            continue;
        const bool before_slice = slice_index < o.actions.size() && action_index < slice_index;
        if (action == "export_settings") {
            const std::string file = o.cli.opt_string("export_settings");
            if (!file.empty())
                add(false, false, "--export-settings " + file, fs::path(file),
                    "the file --export-settings " + file + " writes", "the file --export-settings " + file + " writes",
                    before_slice);
        } else if ((action == "export_stl" || action == "export_stls") && model != nullptr) {
            const std::string dir  = action == "export_stls" ? o.cli.opt_string("export_stls") : std::string();
            const std::string flag = action == "export_stls" ? "--export-stls " + dir : std::string("--export-stl");
            unsigned index = 1;
            for (const Slic3r::ModelObject* object : model->objects) {
                const std::string file = slicer_cli::object_stl_file(o, *object, index++, dir);
                const std::string what = "the object STL file " + file + " " + flag + " writes";
                add(false, true, flag, fs::path(file), what, what, before_slice);
            }
        }
    }
    // The outputs of the actions given before --slice were written before the
    // check that follows the calibration pattern: they keep the names they
    // took, in front of the ones still to come (the sentence names the flag
    // that wrote first). They were checked before they ran, and running them
    // again is not this check's to do.
    if (reserved != nullptr && !reserved->empty())
        outputs.insert(outputs.begin(), reserved->begin(), reserved->end());
    if (o.slice_mode)
        for (int p : plates) {
            const std::string folder = slicer_cli::slicedata_dir(o, "export_slicedata", p);
            if (!folder.empty())
                add(true, false, "--export-slicedata " + folder, fs::path(folder),
                    "the folder --export-slicedata writes (" + folder + ")",
                    "inside the folder --export-slicedata writes (" + folder + ")");
        }
    if (o.slice_mode) {
        add(false, false, "", outdir / "result.json", "the run's own result.json in --outputdir",
            "the run's own result.json in --outputdir");
        for (int p : plates) {
            const std::string name = "plate_" + std::to_string(p) + ".gcode";
            add(false, false, "", outdir / name, "the run's own " + name + " in --outputdir",
                "the run's own " + name + " in --outputdir");
        }
    } else if (!slicer_cli::model_actions_only(o) && !o.output_file.empty()) {
        add(false, false, "-o " + o.output_file, fs::path(o.output_file), "the G-code -o " + o.output_file + " writes",
            "the G-code -o " + o.output_file + " writes");
    }

    // ── What the run reads ──────────────────────────────────────────────
    // With the STL and OBJ files the assemble list names, as its loader opens
    // them (the path as written, construct_assemble_list): they were read into
    // the model, and an output over one would replace the user's part.
    std::vector<std::string> inputs = named_input_files(context, o);
    // The files the loaders read beside those models (an OBJ's mtllib file,
    // its textures, a glTF's buffers), as each loader resolves them: the
    // user's files too, named only inside the model.
    {
        std::vector<std::string> side;
        for (const std::string& file : o.input_files)
            for (std::string& s : slicer_cli::loader_side_files(file, true))
                side.push_back(std::move(s));
        if (context.g_assemble)
            for (const std::string& file : context.g_assemble->sources)
                for (std::string& s : slicer_cli::loader_side_files(file, false))
                    side.push_back(std::move(s));
        inputs.insert(inputs.end(), side.begin(), side.end());
    }

    // ── The checks ──────────────────────────────────────────────────────
    const auto refuse = [&](const char* tag, const std::string& sentence) {
        if (detail != nullptr)
            *detail = sentence;
        else if (o.slice_mode)
            write_result_json(context, outdir.string(), CLI_INVALID_PARAMS, o.slice_plate,
                              cli_error_sentence(CLI_INVALID_PARAMS) + " " + sentence, {}, 0, 0);
        std::cerr << "Error: " << sentence << "\n";
        emit_event({{"event","input_error"}, {"tag", tag}, {"message", sentence}});
        return CLI_INVALID_PARAMS;
    };
    // 0: apart; 1: one name, or two names of one file; 2: names that differ
    // only by letter case.
    const auto same = [&](const fs::path& a, const fs::path& b) {
        boost::system::error_code ec;
        if (fs::equivalent(a, b, ec) && !ec)
            return 1;
        const std::string ka = key(a), kb = key(b);
        if (ka == kb)
            return 1;
        return lower(ka) == lower(kb) ? 2 : 0;
    };
    const auto inside = [&](const fs::path& a, const fs::path& folder) {
        const std::string ka = key(a), kf = key(folder) + "/";
        if (boost::algorithm::starts_with(ka, kf))
            return 1;
        return boost::algorithm::starts_with(lower(ka), lower(kf)) ? 2 : 0;
    };
    // How two outputs meet (0, 1 or 2 as above).
    const auto meet = [&](const RunOutput& a, const RunOutput& b) -> int {
        if (a.folder && b.folder)
            return 0;   // --export-slicedata's own plate folders, one per plate
        if (a.folder || b.folder) {
            const RunOutput& folder = a.folder ? a : b;
            const RunOutput& other  = a.folder ? b : a;
            if (const int c = same(other.path, folder.path))
                return c;
            return inside(other.path, folder.path);
        }
        return same(a.path, b.path);
    };
    for (const RunOutput& out : outputs)
        for (const std::string& input : inputs) {
            boost::system::error_code ec;
            const fs::path in(input);
            bool hit = false;
            if (!out.folder)
                hit = fs::exists(out.path, ec) && fs::equivalent(out.path, in, ec) && !ec;
            else   // the cache files of a plate folder: any input in it
                hit = fs::exists(out.path, ec) && fs::exists(in, ec) && inside(in, out.path) != 0;
            if (!hit)
                continue;
            if (out.flag.empty())   // result.json or a plate's G-code
                return refuse("ExportOverwritesInput", "--slice would overwrite the input file '" + input + "' with " +
                                                           out.what + "; choose another --outputdir.");
            const std::string who = out.flag.rfind("--export-3mf ", 0) == 0 ? std::string("--export-3mf") : out.flag;
            return refuse("ExportOverwritesInput",
                          who + " would overwrite the input file '" + input + "'; give it another name.");
        }
    // --export-slicedata into the folder --load-slicedata reads, or one inside
    // the other: the export would write over the cache the run loads from.
    if (o.slice_mode)
        for (int p : plates) {
            const std::string out_dir = slicer_cli::slicedata_dir(o, "export_slicedata", p);
            const std::string in_dir  = slicer_cli::slicedata_dir(o, "load_slicedata", p);
            if (out_dir.empty() || in_dir.empty())
                continue;
            if (same(fs::path(out_dir), fs::path(in_dir)) || inside(fs::path(out_dir), fs::path(in_dir)) ||
                inside(fs::path(in_dir), fs::path(out_dir)))
                return refuse("ExportOverwritesInput", "--export-slicedata " + out_dir +
                                                           " would overwrite the folder --load-slicedata " + in_dir +
                                                           " reads; give it another folder.");
        }
    for (size_t i = 0; i < outputs.size(); ++i)
        for (size_t j = i + 1; j < outputs.size(); ++j) {
            const RunOutput& a = outputs[i];
            const RunOutput& b = outputs[j];
            if (a.flag.empty())
                continue;   // two of the run's own files never clash (result.json, plate_<n>.gcode)
            if (a.flag == b.flag)
                continue;   // one flag's own files: one object STL each, one folder per plate
            const int c = meet(a, b);
            if (c == 0)
                continue;
            if (a.stl && b.stl)
                // --export-stl and --export-stls into one folder write the same
                // obj_<n>_<name>.stl files, each action in its turn (before or
                // after --slice, so from a model the arrange may have moved):
                // the later one replaces the earlier.
                return refuse("ExportNameTaken", a.flag + " and " + b.flag + " write the same object STL file " +
                                                     b.path.generic_string() +
                                                     (c == 1 ? "" : " (the names differ only by letter case)") +
                                                     "; give --export-stls another folder.");
            if (a.folder)   // a folder that would hold one of the run's own files
                return refuse("ExportNameTaken", a.flag + (c == 1 ? " holds " : " differs only by letter case from ") +
                                                     b.what + "; choose another folder.");
            return refuse("ExportNameTaken", c == 1 ? a.flag + " is " + b.is_what + "; choose another name."
                                                    : a.flag + " differs only by letter case from " + b.what +
                                                          "; choose another name.");
        }
    return 0;
}

/// --slice: every plate (0) or plate N of the input, one G-code each as
/// <outputdir>/plate_N.gcode, through the same single-plate path the default
/// call runs. The official loop stops at the first plate that fails
/// (flush_and_exit inside the plate loop, BambuStudio.cpp 6533-7240), and so
/// does this one. The exit status is the official CLI_* code of that failure.
static int run_slice_mode(CliRunContext& context, const CliOptions& o, Slic3r::Calib_Params& calib_params) {
    namespace fs = boost::filesystem;
    const fs::path outdir = o.outputdir.empty() ? fs::path(".") : fs::path(o.outputdir);
    std::vector<PlateOutcome> outcomes;
    int code = 0;
    std::string error_string;

    try {
        fs::create_directories(outdir);
    } catch (const std::exception& e) {
        std::cerr << "Error: cannot create --outputdir " << outdir.string() << ": " << e.what() << "\n";
        return CLI_ENVIRONMENT_ERROR;
    }

    // --load-assemble-list: the list and every part it names, built before
    // the first plate (BambuStudio.cpp 2165-2188; OrcaSlicer.cpp 1812-1834).
    // A refusal names no plate (record_exit_reson(..., 0, ...)).
    const std::string assemble_file = assemble_list_file(o);
    if (!assemble_file.empty()) {
        auto list = std::make_unique<slicer_cli::AssembleList>();
        // The parts are read while the list is built: a refusal here names the
        // ones its loaders read (and the files they read beside them) as the
        // run's own inputs too, so the result.json it writes — here and from
        // this function's caller — lands on none of them. They stay in hand
        // until the list is the run's (g_assemble).
        const slicer_cli::StepResult built = slicer_cli::load_assemble_list(assemble_file, *list);
        context.g_assemble_pending_sources = list->sources;
        if (built.code != 0) {
            write_result_json(context, outdir.string(), built.code, 0, cli_error_sentence(built.code) + " " + built.message, {}, 0, 0);
            std::cerr << "Error: " << built.message << "\n";
            emit_event({{"event","input_error"}, {"tag","AssembleListRefused"}, {"path", assemble_file},
                        {"message", built.message}});
            return built.code;
        }
        context.g_assemble = std::move(list);
        context.g_assemble_pending_sources.clear();
    } else
    // A missing input is the official "not found", not an unparseable model:
    // the official CLI checks every input before loading it
    // (BambuStudio.cpp 1854-1859 at 5873b5f).
    {
        boost::system::error_code exists_ec;
        const bool input_exists = fs::exists(fs::path(o.input_file), exists_ec);
        if (exists_ec && exists_ec != boost::system::errc::no_such_file_or_directory) {
            // The path could not be checked (permissions, I/O): say so, not
            // "not found".
            const std::string detail = "Cannot check the input " + o.input_file + ": " + exists_ec.message();
            write_result_json(context, outdir.string(), CLI_ENVIRONMENT_ERROR, o.slice_plate,
                              cli_error_sentence(CLI_ENVIRONMENT_ERROR) + " " + detail, {}, 0, 0);
            std::cerr << "Error: " << detail << "\n";
            emit_event({{"event","input_error"}, {"tag","InputUnreadable"}, {"path", o.input_file},
                        {"message", detail}});
            return CLI_ENVIRONMENT_ERROR;
        }
        if (!input_exists) {
            write_result_json(context, outdir.string(), CLI_FILE_NOTFOUND, o.slice_plate,
                              cli_error_sentence(CLI_FILE_NOTFOUND), {}, 0, 0);
            std::cerr << "No such file: " << o.input_file << "\n";
            emit_event({{"event","input_error"}, {"tag","InputNotFound"}, {"path", o.input_file},
                        {"message","No such file: " + o.input_file}});
            return CLI_FILE_NOTFOUND;
        }
    }

    // A project with plate metadata slices plate by plate; anything else (an
    // STL, a 3MF without plates) is one plate holding every object — the
    // official CLI resets plate_to_slice to 0 for a non-Bambu 3MF
    // (BambuStudio.cpp 2192-2196).
    int plate_count = 1;
    bool per_plate_load = false;
    if (context.g_assemble) {
        plate_count = int(context.g_assemble->plates.size());
        per_plate_load = true;
    } else if (slicer_cli::classify_3mf(o.input_file) == slicer_cli::ThreeMfKind::Project) {
        // A geometry-only 3MF (no project settings) loads whole as model
        // geometry, whatever plate tags its model_settings.config keeps.
        const int declared = count_3mf_plates(o.input_file);
        if (declared > 0) {
            plate_count = declared;
            per_plate_load = true;
        }
    }
    // The run's one model and settings (RunState): a prepare call loads every
    // input once and runs the steps the official CLI runs before its plate
    // loop (BambuStudio.cpp 1855-6336; OrcaSlicer.cpp 1537-5470), before any
    // plate is checked or sliced. A project 3MF with model files after it
    // prepares first, as its plan (ProjectPlan) decides the plates; every
    // other run prepares once the plates are known (below).
    RunState rs;
    rs.file_plate_count = plate_count;
    int plan_code = 0;
    std::string plan_error;
    const bool trailing = per_plate_load && !context.g_assemble && trailing_models_port(o);
    std::optional<PlateOutcome> prepare_failure;
    const auto prepare_run = [&](int prepare_plate) {
        PlateOutcome prepare_outcome;
        prepare_outcome.prepare_pass = true;
        prepare_outcome.plate_id = prepare_plate;
        const int rc = slice_one_plate(context, o, calib_params, rs, prepare_plate, std::string(), prepare_outcome);
        if (rc == 0 && prepare_outcome.cli_code == 0 && rs.prepared)
            return;
        if (prepare_outcome.cli_code == 0)
            set_outcome_failure(prepare_outcome, CLI_SLICING_ERROR);
        if (prepare_outcome.error_string.empty())
            prepare_outcome.error_string = cli_error_sentence(prepare_outcome.cli_code);
        plan_code = prepare_outcome.cli_code;
        plan_error = prepare_outcome.error_string;
        prepare_failure = prepare_outcome;
    };
    if (trailing) {
        // --slice 0 may check several plates once the plan is known, so the
        // project's noted-value warning found here is held like the check
        // pass's own (released below when the run checks only one plate).
        rs.hold_noted = o.slice_plate == 0;
        prepare_run(0);
        if (plan_code == 0)
            plate_count = rs.plan.plate_count;
    }

    if (plan_code == 0 && o.slice_plate > plate_count) {
        write_result_json(context, outdir.string(), CLI_INVALID_PARAMS, o.slice_plate,
                          cli_error_sentence(CLI_INVALID_PARAMS) + " --slice " + std::to_string(o.slice_plate) +
                          " but the file has only " + std::to_string(plate_count) + " plate(s).",
                          {}, 0, 0);
        std::cerr << "Error: --slice " << o.slice_plate << " but the file has only "
                  << plate_count << " plate(s)\n";
        emit_event({{"event","input_error"},
                    {"tag","PlateOutOfRange"},
                    {"requested_plate", o.slice_plate},
                    {"plate_count", plate_count},
                    {"message","--slice " + std::to_string(o.slice_plate) +
                               " but the file has only " + std::to_string(plate_count) + " plate(s)"}});
        return CLI_INVALID_PARAMS;
    }

    const auto run_started = std::chrono::steady_clock::now();
    if (o.progress) {
        PlateOutcome preparing;
        emit_progress(preparing, 0, "Prepare slicing");
    }

    std::vector<int> plates;
    if (o.slice_plate == 0)
        for (int p = 1; p <= plate_count; ++p) plates.push_back(p);
    else
        plates.push_back(o.slice_plate);

    // A run that checks every plate before slicing any (--slice 0 on several
    // plates) holds each plate's noted-value warning until the whole check
    // pass has passed: the warning says the run slices with the file's values,
    // which a later plate's refusal makes untrue. The prepare call below
    // checks the first plate, so the hold starts before it.
    const bool hold_noted = o.slice_plate == 0 && plates.size() > 1;
    if (rs.hold_noted && !hold_noted) {
        for (const json& announcement : rs.noted_held)
            announce_noted_warning(announcement);
        rs.noted_held.clear();
    }
    rs.hold_noted = hold_noted;

    // --slice 0 on several plates checks every plate before it slices any
    // (pre_check, BambuStudio.cpp 6441, 7027-7028, 7428-7429; OrcaSlicer.cpp
    // 5565, 6034-6035, 6222-6223): each plate runs up to its slice, and the
    // first refusal ends the run with no G-code written. Every event of the
    // pass is held (diagnostics::held_events). A plate that passes is run
    // again by the slice pass below, so its events are dropped. When a later
    // plate is refused they stay dropped: no plate was sliced, and the
    // official's check pass reports only the refused plate (record_exit_reson
    // with its index, BambuStudio.cpp 6533, 6579; OrcaSlicer.cpp 5650). The
    // refused plate's events are sent with its result.
    // Every other run prepares here, for its one plate or for every plate
    // (--slice 0 on several plates).
    if (!trailing && plan_code == 0) {
        prepare_run(plates.size() == 1 && per_plate_load ? plates.front() : 0);
        // --slice 0 --arrange 1 on several plates: the plates the arrange left.
        if (plan_code == 0 && rs.plan.done) {
            plates.clear();
            for (int p = 1; p <= rs.plan.plate_count; ++p) plates.push_back(p);
        }
    }
    // A prepare call that failed ends the run here: with plate 0 for a
    // project with model files after it (its plan), else with the plate it
    // was preparing (the first plate's check, for --slice 0).
    if (plan_code != 0) {
        code = plan_code;
        error_string = plan_error;
        if (!trailing && prepare_failure) {
            prepare_failure->plate_id = plates.size() == 1 ? plates.front() : 1;
            prepare_failure->plate_index = 1;
            prepare_failure->plate_count = int(plates.size());
            // The refused plate starts without a G-code, as in the slice pass.
            prepare_failure->gcode_path =
                (outdir / ("plate_" + std::to_string(prepare_failure->plate_id) + ".gcode")).string();
            boost::system::error_code ignored;
            fs::remove(prepare_failure->gcode_path, ignored);
            outcomes.push_back(*prepare_failure);
        }
    }
    // Every file the run writes (result.json, the plates' G-code, the
    // --export-3mf project, the --export-settings file, the object STL files,
    // the --export-slicedata folders), checked against each other and against
    // the files the run reads (check_run_outputs). The check runs here, once
    // the plate plan is final: the global arrange (--slice 0 --arrange 1) can
    // add plates the pre-arrange count does not know (BambuStudio.cpp
    // 5627-5722; OrcaSlicer.cpp 4887-4983), and a plate_<n>.gcode it then
    // writes would be overwritten. `plates` is the set this run slices.
    if (const int refused = check_run_outputs(context, o, plates, &rs.model, nullptr))
        return refused;

    const bool pre_check = code == 0 && o.slice_plate == 0 && plates.size() > 1;
    for (size_t i = 0; pre_check && i < plates.size(); ++i) {
        PlateOutcome outcome;
        outcome.plate_id    = plates[i];
        outcome.plate_index = int(i) + 1;
        outcome.plate_count = int(plates.size());
        outcome.gcode_path  = (outdir / ("plate_" + std::to_string(plates[i]) + ".gcode")).string();
        outcome.pre_check   = true;
        std::vector<std::string> held;
        slicer_cli::diagnostics::held_events = &held;
        int rc = 0;
        try {
            rc = slice_one_plate(context, o, calib_params, rs, per_plate_load ? plates[i] : 0, outcome.gcode_path, outcome);
        } catch (...) {
            slicer_cli::diagnostics::held_events = nullptr;
            throw;
        }
        slicer_cli::diagnostics::held_events = nullptr;
        if (rc == 0 && outcome.cli_code == 0)
            continue;
        for (const std::string& line : held)
            slicer_cli::diagnostics::write_event_now(line);
        if (outcome.cli_code == 0)
            set_outcome_failure(outcome, CLI_SLICING_ERROR);
        if (outcome.error_string.empty())
            set_outcome_failure(outcome, outcome.cli_code);
        // The refused plate starts without a G-code, as in the slice pass.
        boost::system::error_code ignored;
        fs::remove(outcome.gcode_path, ignored);
        outcomes.push_back(outcome);
        code = outcome.cli_code;
        error_string = outcome.error_string;
        break;
    }
    if (rs.hold_noted) {
        rs.hold_noted = false;
        // Every plate passed: its own warnings are announced now, in plate
        // order.
        if (code == 0)
            for (const json& announcement : rs.noted_held)
                announce_noted_warning(announcement);
        rs.noted_held.clear();
    }

    for (size_t i = 0; code == 0 && i < plates.size(); ++i) {
        PlateOutcome outcome;
        outcome.plate_id    = plates[i];
        outcome.plate_index = int(i) + 1;
        outcome.plate_count = int(plates.size());
        outcome.gcode_path  = (outdir / ("plate_" + std::to_string(plates[i]) + ".gcode")).string();
        // The plate about to be sliced starts without a G-code in --outputdir,
        // so a failure before export_gcode cannot leave an earlier run's
        // plate_N.gcode looking current. Only this plate: a run that stops
        // here leaves the plates it never reached as they were. The official
        // CLI only overwrites (export to <outputdir>/plate_N.gcode,
        // BambuStudio.cpp 7170-7186; flush_and_exit removes backups only,
        // 458-473); result.json stays the verdict, and a G-code written by
        // this run is kept.
        {
            boost::system::error_code ignored;
            fs::remove(outcome.gcode_path, ignored);
        }
        const int rc = slice_one_plate(context, o, calib_params, rs, per_plate_load ? plates[i] : 0,
                                       outcome.gcode_path, outcome);
        // A failure that set no official code (an engine exception in the
        // process or export step) is the official CLI's slicing failure with
        // its sentence (record_exit_reson(..., CLI_SLICING_ERROR, index+1,
        // cli_errors[CLI_SLICING_ERROR], ...), BambuStudio.cpp 7139, 7423);
        // the exception text is in the slicing_error event.
        if (rc != 0 && outcome.cli_code == 0)
            set_outcome_failure(outcome, CLI_SLICING_ERROR);
        if (outcome.cli_code != 0 && outcome.error_string.empty())
            set_outcome_failure(outcome, outcome.cli_code);
        outcomes.push_back(outcome);
        if (outcome.cli_code != 0) {
            code = outcome.cli_code;
            error_string = outcome.error_string;
            break;
        }
    }
    // --pipe: past the plates, the run's own steps (BambuStudio.cpp 7433-7438).
    if (code == 0 && slicer_cli::pipe_started())
        slicer_cli::pipe_set_plate_info(0, int(plates.size()));
    long long export_ms = 0;
    if (code == 0 && !o.export_3mf.empty()) {
        const auto export_started = std::chrono::steady_clock::now();
        if (o.progress) {
            PlateOutcome last = outcomes.empty() ? PlateOutcome() : outcomes.back();
            emit_run_progress(last, 97, "Exporting 3mf");
        }
        slicer_cli::pipe_update(97, "Exporting 3mf");   // BambuStudio.cpp 8128-8133
        std::string export_error;
        try {
            Slic3r::DynamicPrintConfig project = rs.base;
            apply_command_line_overrides(project, o.overrides, /*report_rejections=*/false);
            // The settings the command line gave: the official setting flags
            // and what this command line's own short flags changed. They are
            // the run's only changes when every other value is a named
            // preset's or a project's loaded over its presets; a settings
            // file or a project sliced as saved brings values of its own, and
            // then every setting that differs from the presets is listed.
            std::set<std::string> given_settings;
            for (const std::string& key : rs.extra.keys())
                given_settings.insert(key);
            for (const std::string& key : rs.base.diff(project))
                given_settings.insert(key);
            const bool settings_files = !o.machine_config.empty() || !o.filament_config.empty() ||
                                        !o.process_config.empty() || !o.bundle_config.empty() ||
                                        o.given_flag("load_settings") || o.given_flag("load_filaments");
            const bool presets_only = !settings_files &&
                                      (rs.project_loaded_like_desktop || !input_is_3mf(o.input_file) ||
                                       slicer_cli::classify_3mf(o.input_file) == slicer_cli::ThreeMfKind::GeometryOnly);
#ifdef ENGINE_SNAPMAKER
            // The switch trimmed each plate's filament list and remapped the
            // model's paint to it, so the project carries that list too:
            // otherwise a reopened project reads the paint against the
            // untrimmed numbering.
            if (slicer_cli::switch_trims_filaments(o.preset_switch, o.printer_preset))
                slicer_cli::trim_project_roster_for_switch(project);
#endif
            code = export_sliced_3mf(context, o, outdir, rs, outcomes, project, presets_only ? &given_settings : nullptr,
                                     export_error);
        } catch (const std::exception& e) {
            code = CLI_EXPORT_3MF_ERROR;
            export_error = e.what();
        }
        if (code != 0)
            error_string = cli_error_sentence(code) + (export_error.empty() ? "" : " " + export_error);
        export_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - export_started).count();
    }
    const long long total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - run_started).count();
    long long sliced_ms = 0;
    for (const PlateOutcome& p : outcomes) sliced_ms += p.sliced_time_ms;
    if (code == 0) error_string = cli_error_sentence(CLI_SUCCESS);
    const int reported_plate = (code != 0 && !outcomes.empty())
                                   ? (outcomes.back().run_step_failed ? 0 : outcomes.back().plate_id)
                                   : o.slice_plate;
    const bool result_written = write_result_json(context, outdir.string(), code, reported_plate, error_string, outcomes,
                                                  std::max(0LL, total_ms - sliced_ms - export_ms), export_ms);
    if (code != 0)
        std::cerr << "Error: " << error_string << "\n";
    // No result document is a failed run, with the code this mode already
    // uses when --outputdir cannot be made (CLI_ENVIRONMENT_ERROR above).
    if (code == 0 && !result_written)
        code = CLI_ENVIRONMENT_ERROR;
    // 100 only once everything is written ("All done, Success",
    // BambuStudio.cpp 8237).
    if (code == 0 && o.progress) {
        PlateOutcome last = outcomes.empty() ? PlateOutcome() : outcomes.back();
        emit_run_progress(last, 100, "All done, Success");
    }
    // --pipe: the last step, then the pipe closes (BambuStudio.cpp 8234-8241;
    // a failed run closes it without one, flush_and_exit 457-460).
    if (code == 0)
        slicer_cli::pipe_update(100, "All done, Success");
    slicer_cli::pipe_stop();
    return code;
}

static int run_cli_slice(CliRunContext& context, const CliOptions& o, Slic3r::Calib_Params& calib_params) {
#ifdef ENGINE_BAMBU
    // --allow-mix-temp has no engine hook on Bambu 2.9.1 (no plate-level
    // temperature-mix check to lift); a caller that passes it is told once,
    // on the --slice path and on the single-file call alike.
    if (o.given_flag("allow_mix_temp"))
        emit_event({{"event","warning"},
                    {"tag","AllowMixTempIgnored"},
                    {"message", "--allow-mix-temp has no effect: Bambu Studio 2.9.1 has no plate-level "
                                "temperature-mix check to lift"}});
#endif
    if (o.slice_mode)
        return run_slice_mode(context, o, calib_params);
    PlateOutcome outcome;
    outcome.plate_id = o.plate_id;
    RunState rs;
    const int rc = slice_one_plate(context, o, calib_params, rs, o.plate_id, o.output_file, outcome);
    // --progress on the single-file call ends like --slice: 100 once the
    // G-code is written ("All done, Success", BambuStudio.cpp 8237).
    if (rc == 0 && o.progress)
        emit_run_progress(outcome, 100, "All done, Success");
    if (rc == 0 && slicer_cli::pipe_started()) {
        slicer_cli::pipe_set_plate_info(0, 1);
        slicer_cli::pipe_update(100, "All done, Success");
    }
    slicer_cli::pipe_stop();
    return rc;
}

int main(int argc, char** argv) {
    static CliRunContext context;
    // Name the main thread and save its id first, as the official command
    // lines start (BambuStudio.cpp:1490-1493 at 926a719, OrcaSlicer.cpp
    // 1189-1192 at 8500fcd, Snapmaker_Orca.cpp:1045-1048 at bac6fd5). On
    // Windows the first name sets up the thread-naming API with no lock
    // (Thread.cpp WindowsGetSetThreadNameAPIInitialize: "name the main thread
    // before spawning worker threads"), so it must happen before the worker
    // threads name themselves.
#ifdef ENGINE_SNAPMAKER
    Slic3r::set_current_thread_name("Snapmaker_Orca_main");
#elif defined(ENGINE_ORCA)
    Slic3r::set_current_thread_name("orcaslicer_main");
#else
    Slic3r::set_current_thread_name("bambustu_main");
#endif
    Slic3r::save_main_thread_id();
    // UTF-8 everywhere, as the official command lines start: on Windows the
    // arguments come from the wide command line as UTF-8 (BambuStudio.cpp
    // 8636-8660, the wide entry of BambuStudio_app_msvc.cpp; the same in
    // OrcaSlicer), and boost::filesystem paths take UTF-8 strings
    // (nowide_filesystem, BambuStudio.cpp 1510-1523; OrcaSlicer.cpp 1233-1246).
    // Elsewhere both are no-ops.
    boost::nowide::args utf8_args(argc, argv);
    // Timing needs the same UTF-8 paths and the engine's resources root as the
    // other modes (an Orca-derived import reads info/nozzle_info.json through
    // Print::get_hrc_by_nozzle_type, Print.cpp 3084 at the Orca pin), but
    // returns before logging or any slicing path.  Quiet: its stdout and
    // stderr are the one JSON document and nothing.
    if (slicer_cli::gcode_timing_requested(argc, argv)) {
        try {
            // UTF-8 paths first: resource discovery converts the executable's
            // path with root.string(), which on Windows needs this locale for
            // a non-ASCII install directory (BambuStudio.cpp 1510-1523).
            boost::nowide::nowide_filesystem();
            configure_engine_resources(argv[0], /*quiet=*/true);
        } catch (...) {
            // Timing owns stdout: either step may throw before the command's
            // own boundary, so keep its one-document contract.
            return slicer_cli::write_gcode_timing_unavailable();
        }
        return slicer_cli::run_gcode_timing_command(argc, argv);
    }
    try {
        boost::nowide::nowide_filesystem();
    } catch (const std::runtime_error& ex) {
        std::cerr << "boost::nowide::nowide_filesystem failed; slicer-cli stops here.\n" << ex.what() << "\n";
        return CLI_ENVIRONMENT_ERROR;
    }
    // Initialize libslic3r
    Slic3r::set_logging_level(3); // Info level
    boost::log::core::get()->set_logging_enabled(true);

    // Parse arguments: the official flags through this engine's own CLI
    // definitions, slicer-cli's own flags beside them (cli_command_line.cpp).
    CliOptions o;
    slicer_cli::ModeArgs mode_args;
    const std::string& input_file = o.input_file;
    int& plate_id = o.plate_id;  // 0 = all plates (default); >0 = slice only that plate
    const std::string& layout_json_file = mode_args.layout_json_file;
    const bool& layout_plan_mode = mode_args.layout_plan_mode;
    slicer_cli::CalibOptions& calib_opts = mode_args.calib;

    for (int i = 1; i < argc; ++i)
        context.g_command_line_words.emplace_back(argv[i]);

    // subcommand: slicer_cli layout capabilities --json
    if (argc >= 3 && std::string(argv[1]) == "layout" && std::string(argv[2]) == "capabilities") {
        if (argc != 4 || std::string(argv[3]) != "--json") {
            std::cerr << "Usage: " << argv[0] << " layout capabilities --json\n";
            // With --slice among the other words the refusal still leaves
            // result.json, as every refused --slice command line does (the
            // parse refusal below): the parser's own first pass finds --slice
            // and --outputdir in the words after the subcommand.
            // The subcommand's own --json is not a flag of the slice.
            std::vector<char*> rest{argv[0]};
            for (int i = 3; i < argc; ++i)
                if (!(i == 3 && std::string(argv[i]) == "--json"))
                    rest.push_back(argv[i]);
            CliOptions probe;
            slicer_cli::ModeArgs probe_mode;
            slicer_cli::ParseRefusal probe_refusal;
            (void) slicer_cli::parse_command_line((int)rest.size(), rest.data(), probe, probe_mode, probe_refusal);
            if (probe.slice_mode || probe.slice_given) {
                const std::string dir = probe.outputdir.empty() ? "." : probe.outputdir;
                boost::system::error_code mk;
                boost::filesystem::create_directories(dir, mk);
                context.g_result_json_options = &probe;
                context.g_result_json_mode = &probe_mode;
                write_result_json(context, dir, CLI_INVALID_PARAMS, probe.slice_mode ? std::max(0, probe.slice_plate) : 0,
                                  cli_error_sentence(CLI_INVALID_PARAMS) +
                                      " layout capabilities takes --json and nothing else; run the slice without it.",
                                  {}, 0, 0);
                return CLI_INVALID_PARAMS;
            }
            return 1;
        }
        layout_plan::install_cancellation_handler();  // ignore SIGPIPE so write failures surface as errors
        boost::log::core::get()->set_logging_enabled(false);
        return layout_plan::run_capabilities();
    }

    // A run that is killed cannot remove its staging folders, and each holds
    // up to the input's embedded G-code (the loader extracts
    // Metadata/plate_1.gcode into it: 175 MB on the OrcaSlicer build with the
    // corpus 44e597511c7d5abd.3mf). Bound a slice with `timeout` — SIGTERM —
    // or kill it, and on a tmpfs /tmp that is RAM left behind. The folders of
    // runs that are gone go now, before this run makes its own. No SIGTERM or
    // SIGHUP handler is installed to remove this run's: the run holds ~30
    // threads, and fork() in a handler (what an `rm -rf` child needs) runs
    // glibc's atfork handlers and takes the malloc arena locks, so a signal
    // that lands while any thread holds one blocks the handler on a futex for
    // ever — a plugin that ends a run with SIGTERM would then wait for ever.
    // The default action ends the run with 143/129, the rest of the corpus and
    // leakcheck's SIGTERM row leave the folder to the sweep above, and the
    // ownership marker's PID check reclaims it, exactly as after SIGKILL.
    sweep_stale_staging_folders();

    // Both engines read slice-time resources (info/, flush/, filament_mixing/)
    // relative to this root, which depends on no argument: resolve it once here
    // so the ENGINE_ORCA build gets it too, and so STL and calibration slices
    // stop falling back to the hardcoded tables.
    {
        // --layout-plan owns stderr as JSON documents, so the resources line
        // is left out there. Whether the command line asks for it is the
        // parser's own answer -- every spelling it takes (-layout-plan,
        // --layout-plan, an inline =value) -- from a dry parse; the parse the
        // run uses follows, and reports any refusal.
        CliOptions               probe;
        slicer_cli::ModeArgs     probe_mode;
        slicer_cli::ParseRefusal probe_refusal;
        (void) slicer_cli::parse_command_line(argc, argv, probe, probe_mode, probe_refusal);
        configure_engine_resources(argv[0], probe_mode.layout_plan_mode);
    }

    {
        slicer_cli::ParseRefusal refusal;
        context.g_result_json_options = &o;
        context.g_result_json_mode = &mode_args;
        if (!slicer_cli::parse_command_line(argc, argv, o, mode_args, refusal)) {
            std::cerr << "Error: " << refusal.message << "\n";
            if (refusal.with_usage) {
                std::cerr << "\n";
                print_usage(argv[0]);
            }
            // --slice always leaves result.json, also when its own value is
            // the one refused (--slice foo, --slice=-1): the official exits
            // with CLI_INVALID_PARAMS for a bad command line ("setup params
            // error", BambuStudio.cpp 1553-1558; OrcaSlicer.cpp 1249-1252),
            // and records an early exit's reason (BambuStudio.cpp 1697).
            if (o.slice_mode || o.slice_given) {
                const std::string dir = o.outputdir.empty() ? "." : o.outputdir;
                boost::system::error_code mk;
                boost::filesystem::create_directories(dir, mk);
                write_result_json(context, dir, refusal.code, o.slice_mode ? std::max(0, o.slice_plate) : 0,
                                  cli_error_sentence(refusal.code) + " " + refusal.message, {}, 0, 0);
                return refusal.code;
            }
            return 1;
        }
        context.g_command_line_parsed = true;
    }
    // --help: the flags. With --slice the run goes on: the official prints its
    // help as one action of its loop and runs the others, --slice among them
    // (BambuStudio.cpp 6337-6338; OrcaSlicer.cpp 5471-5472).
    if (mode_args.help) {
        print_usage(argv[0]);
        if (!o.slice_mode)
            return 0;
    }
    // A refusal before anything loads: under --slice it leaves result.json
    // in --outputdir too, as every --slice run does (the official records the
    // reason of an early exit the same way, record_exit_reson,
    // BambuStudio.cpp 1697; OrcaSlicer.cpp 1387). Without --slice: exit 1.
    auto refuse_run = [&](int code, const std::string& sentence) -> int {
        std::cerr << "Error: " << sentence << "\n";
        if (!o.slice_mode)
            return 1;
        const std::string dir = o.outputdir.empty() ? "." : o.outputdir;
        boost::system::error_code mk;
        boost::filesystem::create_directories(dir, mk);
        write_result_json(context, dir, code, o.slice_plate, cli_error_sentence(code) + " " + sentence, {}, 0, 0);
        return code;
    };
    if (o.verbose)
        Slic3r::set_logging_level(5);
    // --debug N: the engine's log level (BambuStudio.cpp 1661-1667; OrcaSlicer.cpp 1352-1359).
    if (o.given_flag("debug"))
        Slic3r::set_logging_level(o.cli.opt_int("debug"));
    // The arrange switches (read in arrange_on_bed).
    {
        auto flag = [&](const char* key, bool fallback) { return o.given_flag(key) ? o.cli.opt_bool(key) : fallback; };
        context.g_arrange_switches.enable_timelapse            = flag("enable_timelapse", false);
        context.g_arrange_switches.allow_rotations             = flag("allow_rotations", false);
        context.g_arrange_switches.allow_multicolor_oneplate   = flag("allow_multicolor_oneplate", true);
        context.g_arrange_switches.avoid_extrusion_cali_region = flag("avoid_extrusion_cali_region", false);
#ifdef ENGINE_BAMBU
        context.g_skip_useless_pick = flag("skip_useless_pick", false);
#endif
    }

    // Calibration: resolve the mode and params up front so a bad --calib-*
    // value fails fast with usage, before any model/config work.
    Slic3r::Calib_Params calib_params;
    try {
        calib_params = slicer_cli::build_calib_params(calib_opts);
    } catch (const std::exception& e) {
        return refuse_run(CLI_INVALID_PARAMS, e.what());
    }
    const bool calib_self_geometry = slicer_cli::calib_mode_generates_geometry(calib_params.mode);

#ifdef ENGINE_ORCA
    // pressure_advance_pattern's geometry generator is ported only for the Bambu
    // engine (Orca's CalibPressureAdvancePattern API differs); reject it cleanly
    // here so the Orca binary fails fast instead of throwing from apply_pa_pattern.
    if (calib_params.mode == Slic3r::CalibMode::Calib_PA_Pattern)
        return refuse_run(CLI_INVALID_PARAMS, "pressure_advance_pattern is not yet supported on the OrcaSlicer "
                                              "engine; use a tower or pressure_advance_line calib mode instead.");
#endif

    // --engine-info and --list-presets (slicer-cli's own, with no official
    // counterpart): one JSON document on stdout, nothing sliced. With --slice
    // they are refused, so a --slice run never ends without slicing or
    // result.json.
    if (o.slice_mode && !mode_args.engine_info_file.empty())
        return refuse_run(CLI_INVALID_PARAMS, "--engine-info describes a file and slices nothing; give it without --slice.");
    if (o.slice_mode && mode_args.list_presets)
        return refuse_run(CLI_INVALID_PARAMS, "--list-presets lists this engine's presets and slices nothing; give it "
                                              "without --slice.");
    if (o.slice_mode && mode_args.list_settings)
        return refuse_run(CLI_INVALID_PARAMS, "--list-settings lists this engine's settings and slices nothing; give "
                                              "it without --slice.");
    if (o.slice_mode && mode_args.list_flags)
        return refuse_run(CLI_INVALID_PARAMS, "--list-flags lists this program's flags and slices nothing; give it "
                                              "without --slice.");
    if (!mode_args.engine_info_file.empty()) {
        boost::log::core::get()->set_logging_enabled(false);
        return run_info(o.argv0, mode_args.engine_info_file, o.printer_preset);
    }

    if (mode_args.list_presets) {
        boost::log::core::get()->set_logging_enabled(false);
        return run_list_presets(o, mode_args.list_printer);
    }
    if (mode_args.list_settings) {
        boost::log::core::get()->set_logging_enabled(false);
        return run_list_settings();
    }
    if (mode_args.list_flags) {
        boost::log::core::get()->set_logging_enabled(false);
#ifdef ENGINE_SNAPMAKER
        const char* engine = "snapmaker";
#elif defined(ENGINE_ORCA)
        const char* engine = "orcaslicer";
#else
        const char* engine = "bambustudio";
#endif
        return slicer_cli::run_list_flags(engine, engine_version_text());
    }
    // --load-assemble-list builds the plates itself: no model files with it
    // (BambuStudio.cpp 1848-1853), and on OrcaSlicer no transforms either
    // (OrcaSlicer.cpp 1530-1535). It is the official plate loop's input, so
    // it runs with --slice; and as for any input that is not a Bambu-made
    // 3MF, every plate is sliced (plate_to_slice reset to 0,
    // BambuStudio.cpp 2192-2196; OrcaSlicer.cpp 1839-1843).
    if (!assemble_list_file(o).empty()) {
#ifdef ENGINE_ORCA
        if (!o.input_files.empty() || !o.transforms.empty())
            return refuse_run(CLI_INVALID_PARAMS, "--load-assemble-list builds the plates from its list; "
                                                  "give no model files and no transforms with it.");
#else
        if (!o.input_files.empty())
            return refuse_run(CLI_INVALID_PARAMS, "--load-assemble-list builds the plates from its list; "
                                                  "give no model files with it.");
#endif
        if (!o.slice_mode)
            return refuse_run(CLI_INVALID_PARAMS, "--load-assemble-list needs --slice: it slices the list's plates "
                                                  "into --outputdir.");
        if (o.slice_plate > 0) {
            emit_event({{"event","config_normalized"}, {"tag","AssembleListSlicesEveryPlate"},
                        {"requested_plate", o.slice_plate},
                        {"message","--load-assemble-list slices every plate of the list, as the official command "
                                   "line does for any input that is not a Bambu-made 3MF; --slice " +
                                   std::to_string(o.slice_plate) + " is taken as --slice 0"}});
            o.slice_plate = 0;
        }
    }
    // A project 3MF with model files after it (ProjectPlan): is_bbl_3mf ends
    // false, so the official slices every plate (plate_to_slice reset to 0,
    // BambuStudio.cpp 2192-2196; OrcaSlicer.cpp 1839-1843), after arranging
    // everything across the plates. --repetitions is then refused by its own
    // step, as for any --slice 0 (BambuStudio.cpp 4960-4964; OrcaSlicer.cpp
    // 4221-4225).
    if (assemble_list_file(o).empty() && trailing_models_port(o)) {
        if (o.slice_mode && o.slice_plate > 0) {
            emit_event({{"event","config_normalized"}, {"tag","ProjectWithModelsSlicesEveryPlate"},
                        {"requested_plate", o.slice_plate},
                        {"message","Model files after a project 3MF: every plate is sliced, as the official command "
                                   "line does once a file that is not a Bambu-made 3MF is loaded; --slice " +
                                   std::to_string(o.slice_plate) + " is taken as --slice 0"}});
            o.slice_plate = 0;
        }
    }
    // What the input is, in one answer: only a .3mf that OPENS and carries
    // Metadata/project_settings.config is a project. A .3mf that cannot be
    // read (missing, or not an archive) is its own state, never a project:
    // upstream checks every input's existence before it loads one, and a
    // failed load is the loader's to report (BambuStudio.cpp 1855-1860 at
    // 5873b5f; OrcaSlicer.cpp 1537-1542 at 31f6803). Calling it a project
    // here would refuse a named preset as CLI_INVALID_PARAMS ("a 3MF carries
    // its own settings") before the run could report CLI_FILE_NOTFOUND or
    // CLI_DATA_FILE_ERROR, as the same call does without a preset.
    const slicer_cli::ThreeMfKind input_3mf = slicer_cli::classify_3mf(input_file);
    const bool project_3mf_input = input_3mf == slicer_cli::ThreeMfKind::Project;
    // A 3MF with geometry only names no printer: the desktop asks for one,
    // the command line needs it named to slice. A run of model actions only
    // (--info, --export-stl, --export-stls, --export-settings without
    // --slice) needs none, as for an STL: the official runs those actions on
    // the loaded model (BambuStudio.cpp 6366-6401; OrcaSlicer.cpp 5499-5534).
    if (input_3mf == slicer_cli::ThreeMfKind::GeometryOnly && !o.uses_presets() &&
        !slicer_cli::model_actions_only(o) &&
        o.machine_config.empty() && o.bundle_config.empty() &&
        o.cli.option<Slic3r::ConfigOptionStrings>("load_settings")->values.empty())
        return refuse_run(CLI_INVALID_PARAMS,
            input_file + " holds geometry only, with no printer or print settings. Name the printer with "
            "--printer-preset (and optionally --process-preset and --filament-preset), or give settings "
            "files with --load-settings.");
    if (o.uses_presets()) {
        int code = 0;
        std::string error;
        bool resolved = false;
        if (project_3mf_input) {
            // --printer-preset on a project 3MF: the desktop app's printer
            // switch (stage_named_presets_for_project). The presets named are
            // written out in full for the merge --load-settings already runs.
            const std::string dir = context.run_staging.add((boost::filesystem::temp_directory_path() /
                                                     boost::filesystem::unique_path("slicer_cli_preset-%%%%%%%%")).string());
            resolved = stage_named_presets_for_project(o, dir, code, error);
        } else {
            context.g_preset_config = std::make_unique<Slic3r::DynamicPrintConfig>();
            resolved = resolve_named_presets(context, o, *context.g_preset_config, code, error);
        }
        if (!resolved) {
            emit_event({{"event","preset_error"}, {"tag","NamedPresetRefused"}, {"message", error}});
            std::cerr << "Error: " << error << "\n";
            if (o.slice_mode) {
                boost::system::error_code mk;
                boost::filesystem::create_directories(o.outputdir.empty() ? "." : o.outputdir, mk);
                write_result_json(context, o.outputdir.empty() ? "." : o.outputdir, code, o.slice_plate,
                                  cli_error_sentence(code) + " " + error, {}, 0, 0);
                return code;
            }
            return 1;
        }
    }
    if (!o.export_3mf.empty() && !o.slice_mode) {
        std::cerr << "Error: --export-3mf needs --slice\n";
        return 1;
    }
    if (const std::string why = slicer_cli::simplify_flags_refusal(o); !why.empty())
        return refuse_run(CLI_INVALID_PARAMS, why);
    if (o.arrange_forced() && !o.slice_mode) {
        std::cerr << "Error: --arrange needs --slice (the official CLI arranges inside its plate loop)\n";
        return 1;
    }
    if (o.slice_mode)
        o.progress = true;   // --slice reports progress like the official --pipe
    if (o.slice_mode && plate_id > 0)
        return refuse_run(CLI_INVALID_PARAMS, "--plate and --slice are mutually exclusive; use --slice N for plate N");

    // Detect conflicting layout flags. Under --slice this refusal must still
    // leave result.json like every other early refusal (refuse_run); without
    // --slice it prints the same sentence and exits 1, unchanged. The owner
    // rule's "what to do" part goes on the result.json path alone: the
    // sentence the no---slice call prints is the one that was always there.
    if (layout_plan_mode && !layout_json_file.empty())
        return refuse_run(CLI_INVALID_PARAMS, o.slice_mode
                              ? "--layout-plan and --layout are mutually exclusive; give one of them"
                              : "--layout-plan and --layout are mutually exclusive");

    // Both layout modes arrange and return, before any slice: like
    // --engine-info and --list-presets they write no result.json, so with
    // --slice they are refused too, and a --slice run never ends without
    // slicing or result.json. Without --slice neither line below is reached.
    if (o.slice_mode && layout_plan_mode)
        return refuse_run(CLI_INVALID_PARAMS, "--layout-plan arranges the input's objects and slices nothing; give "
                                              "it without --slice.");
    if (o.slice_mode && !layout_json_file.empty())
        return refuse_run(CLI_INVALID_PARAMS, "--layout arranges the model files and slices nothing; give it "
                                              "without --slice.");

    // --layout-plan: versioned headless arrange contract
    if (layout_plan_mode) {
        layout_plan::install_cancellation_handler();  // before input read: honor SIGINT during parse
        json raw;
        std::string input_data;
        int fd = -1;
        if (!input_file.empty()) {
            // opening a FIFO blocks until a writer connects; loop on EINTR and
            // treat it as a cancellation check point
            bool cancelled = false;
            for (;;) {
                if (layout_plan::is_cancelled()) { cancelled = true; break; }
#ifdef _WIN32
                // The path is UTF-8 (boost::nowide::args): open it by its wide form.
                fd = ::_wopen(boost::nowide::widen(input_file).c_str(), _O_RDONLY | _O_BINARY);  // binary: no Ctrl+Z EOF, no newline translation
#else
                fd = ::open(input_file.c_str(), O_RDONLY);
                if (fd < 0 && errno == EINTR) {
                    if (layout_plan::is_cancelled()) { cancelled = true; break; }
                    continue;
                }
#endif
                break;
            }
            if (cancelled) {
                std::cerr << json{{"schemaVersion",1},{"error",{{"code","CANCELLED"},{"message","cancelled during input open"}}}}.dump() << std::endl;
                return 5;
            }
            if (fd < 0) {
                std::cerr << json{{"schemaVersion",1},{"error",{{"code","INVALID_INPUT"},{"message","cannot open --input file"}}}}.dump() << std::endl;
                return 3;
            }
        } else {
            fd = 0;  // stdin
        }
        // route --input through the same cancellable fd read loop as stdin so
        // FIFOs/slow streams observe SIGINT/Ctrl+C with a bounded exit
        int rrc = read_all_cancellable(fd, input_data);
        if (fd > 0) {
#ifdef _WIN32
            ::_close(fd);
#else
            ::close(fd);
#endif
        }
        if (rrc == 1) {
            std::cerr << json{{"schemaVersion",1},{"error",{{"code","CANCELLED"},{"message","cancelled during input read"}}}}.dump() << std::endl;
            return 5;
        }
        if (rrc == 2) {
            std::cerr << json{{"schemaVersion",1},{"error",{{"code","INVALID_INPUT"},{"message","failed to read input stream"}}}}.dump() << std::endl;
            return 3;
        }
        try { raw = json::parse(input_data); } catch (const std::exception& e) {
            if (layout_plan::is_cancelled()) {  // SIGINT during the parse → cancel, not parse-error
                std::cerr << json{{"schemaVersion",1},{"error",{{"code","CANCELLED"},{"message","cancelled during input read"}}}}.dump() << std::endl;
                return 5;
            }
            std::cerr << json{{"schemaVersion",1},{"error",{{"code","INVALID_INPUT"},{"message",std::string("JSON parse error: ")+e.what()}}}}.dump() << std::endl;
            return 3;
        }
        if (layout_plan::is_cancelled()) {  // SIGINT during a large parse → CANCELLED, no continued work
            std::cerr << json{{"schemaVersion",1},{"error",{{"code","CANCELLED"},{"message","cancelled during input read"}}}}.dump() << std::endl;
            return 5;
        }
        layout_plan::LayoutProblemV1 problem;
        layout_plan::LayoutErrorV1   parse_err;
        if (!layout_plan::parse_input(raw, problem, parse_err)) {
            json err_json = {
                {"schemaVersion", parse_err.SCHEMA_VERSION},
                {"error", {
                    {"code",    parse_err.error.code},
                    {"message", parse_err.error.message}
                }}
            };
            if (!parse_err.error.object_ids.empty())
                err_json["error"]["object_ids"] = parse_err.error.object_ids;
            std::cerr << err_json.dump() << std::endl;
            return parse_err.error.code == "CANCELLED" ? 5 : 3;
        }
        boost::log::core::get()->set_logging_enabled(false);
        return layout_plan::run_layout_plan(problem);
    }

    // --layout: headless arrange from the older JSON form
    if (!layout_json_file.empty()) {
        boost::nowide::ifstream lf(layout_json_file);
        if (!lf.is_open()) { std::cerr << "Cannot open layout JSON: " << layout_json_file << "\n"; return 1; }
        json lj;
        try { lj = json::parse(lf); } catch (const std::exception& e) {
            std::cerr << "Failed to parse layout JSON: " << e.what() << "\n"; return 1;
        }
        std::string profiles_dir = lj.value("profilesDir", "");
        while (!profiles_dir.empty() && profiles_dir.back() == '/') profiles_dir.pop_back();

        Slic3r::DynamicPrintConfig cfg;
        if (lj.contains("profiles")) {
            if (profiles_dir.empty()) {
                std::cerr << "profilesDir is required when profiles is specified\n";
                return 1;
            }
            for (auto& [kind, path] : lj["profiles"].items()) {
                if (!load_json_config(profiles_dir + "/" + path.get<std::string>(), cfg)) {
                    std::cerr << "Failed to load profile: " << path.get<std::string>() << "\n";
                    return 1;
                }
                // The profile keeps its own "inherits" chain: the arrange reads
                // the printer's clearances, which a shipped machine preset
                // leaves to its parents.
                apply_file_inherits(argv[0] ? argv[0] : "slicer_cli", profiles_dir + "/" + path.get<std::string>(),
                                    kind.c_str(), cfg, false);
            }
        }
        using namespace Slic3r;
        using namespace Slic3r::arrangement;

        ArrangeParams params;
#ifdef ENGINE_ORCA
        params.clearance_radius = cfg.has("extruder_clearance_max_radius") ? cfg.opt_float("extruder_clearance_max_radius") : 1.0f;
        if (params.clearance_radius < 1.0f) params.clearance_radius = lj.value("clearanceRadiusMm", 68.0f);
#else
        params.cleareance_radius = cfg.has("extruder_clearance_max_radius") ? cfg.opt_float("extruder_clearance_max_radius") : 1.0f;
        if (params.cleareance_radius < 1.0f) params.cleareance_radius = lj.value("clearanceRadiusMm", 68.0f);
#endif

        // Load all STLs into a single Model (one ModelObject per STL)
        Model model;
        if (!lj.contains("objects") || !lj["objects"].is_array()) {
            std::cerr << "objects must be a JSON array\n";
            return 1;
        }
        for (auto& obj : lj["objects"]) {
            if (!obj.is_object()) {
                std::cerr << "each object entry must be a JSON object\n";
                return 1;
            }
            std::string stl_path = obj.value("stl", "");
            if (stl_path.empty()) continue;
            try {
                Model m = Model::read_from_file(stl_path);
                for (ModelObject* mo : m.objects) {
                    ModelObject* new_obj = model.add_object(*mo);
                    if (new_obj->instances.empty())
                        new_obj->add_instance();
                }
            } catch (const std::exception& e) {
                std::cerr << "Failed to load " << stl_path << ": " << e.what() << "\n";
                return 1;
            }
        }
        params.min_obj_distance = scaled<coord_t>(lj.value("spacingMm", 10.0));
        params.allow_rotations = lj.value("allowRotations", true);
        params.do_final_align = lj.value("doFinalAlign", true);

        // Use get_arrange_polys -> arrange pipeline (same as GUI ArrangeJob)
        ModelInstancePtrs instances;
        auto input = get_arrange_polys(model, instances);
        // get_arrange_polygon leaves bed_idx at its UNARRANGED (-1) default,
        // which the nester skips as BIN_ID_UNFIT. Seed bed 0 as the desktop
        // does (ModelArrange.cpp:98, `ap.bed_idx = 0;` in get_arrange_poly;
        // same in Orca v2.4.2 src/libslic3r/ModelArrange.cpp:25).
        for (auto& ap : input) ap.bed_idx = 0;

#ifdef ENGINE_ORCA
        update_arrange_params(params, &cfg, input);
        update_selected_items_inflation(input, &cfg, params);
        Points bed_pts = get_shrink_bedpts(&cfg, params);
#else
        update_arrange_params(params, cfg, input);
        update_selected_items_inflation(input, cfg, params);
        Points bed_pts = get_shrink_bedpts(cfg, params);
#endif
        arrangement::arrange(input, {}, bed_pts, params);

        // Apply results back to model instances
        apply_arrange_polys(input, instances, [](ArrangePolygon&) {});

        json out;
        out["engine"] =
        #ifdef ENGINE_SNAPMAKER
            "snapmaker";
        #elif defined(ENGINE_ORCA)
            "orca";
        #else
            "bambu";
        #endif
        out["placements"] = json::array();
        for (auto& ap : input) {
            json p;
            p["name"] = ap.name;
            p["bed_idx"] = ap.bed_idx;
            p["x_mm"] = unscaled<double>(ap.translation.x());
            p["y_mm"] = unscaled<double>(ap.translation.y());
            p["rotation_deg"] = ap.rotation * 180.0 / M_PI;
            BoundingBox bb = ap.transformed_poly().contour.bounding_box();
            p["cx_mm"] = unscaled<double>(bb.min.x() + bb.max.x()) / 2.0;
            p["cy_mm"] = unscaled<double>(bb.min.y() + bb.max.y()) / 2.0;
            out["placements"].push_back(p);
        }
        std::cout << out.dump() << std::endl;
        return 0;

    }
    // ── Structured-diagnostics install point ───────────────────────────────
    // Install only on the slicing path. Capabilities and --layout-plan require
    // one JSON document. Legacy --layout can include engine text before its
    // result; keep its existing output free of structured diagnostic events.
    install_engine_log_bridge();

    if (input_file.empty() && !calib_self_geometry && assemble_list_file(o).empty()) {
        if (o.slice_mode)
            return refuse_run(CLI_INVALID_PARAMS, "No input file specified");
        std::cerr << "Error: No input file specified\n\n";
        print_usage(argv[0]);
        return 1;
    }
    if (calib_self_geometry) {
        // pressure_advance_pattern DISCARDS the loaded model and generates its own
        // geometry, but it still needs a fully-resolved printer/filament/process
        // config AND the 3MF's per-plate custom-gcode scaffolding. Require a real
        // .3mf --input: a profile bundle alone is insufficient (its files load
        // later with only a warning on failure, and no plate metadata is set up,
        // so the pattern would silently slice the default/wrong printer). An STL
        // supplies only geometry, which the pattern throws away.
        //
        // The loader's own test (input_is_3mf: the final extension), so a
        // path like `part.3mf.stl` — which loads as STL — is NOT a 3MF here.
        if (!input_is_3mf(input_file)) {
            const std::string why = "pressure_advance_pattern requires a .3mf --input for its "
                                    "printer/filament config (it discards the model geometry but reads the "
                                    "embedded config + plate setup; a profile bundle or STL is not "
                                    "sufficient)";
            if (o.slice_mode)
                return refuse_run(CLI_INVALID_PARAMS, why);
            std::cerr << "Error: " << why << "\n\n";
            print_usage(argv[0]);
            return 1;
        }
    }

    std::cout << "libslic3r_standalone - Standalone slicing tool\n";
    std::cout << "Based on BambuStudio libslic3r\n\n";

    // --pipe NAME (Linux; BambuStudio.cpp 1758-1769, OrcaSlicer.cpp
    // 1441-1451): the progress pipe, from the start of the run.
    if (o.given_flag("pipe") && !o.cli.opt_string("pipe").empty() && slicer_cli::pipe_supported()) {
        slicer_cli::pipe_start(o.cli.opt_string("pipe"));
        slicer_cli::pipe_update(1, "Start to load files");
    }

    return run_cli_slice(context, o, calib_params);
}
