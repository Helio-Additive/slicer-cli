// calib_args.cpp — see calib_args.hpp.
#include "calib_args.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "libslic3r/TriangleMesh.hpp"

namespace slicer_cli {

using namespace Slic3r;

CalibMode parse_calib_mode(const std::string& s)
{
    if (s.empty() || s == "none")        return CalibMode::Calib_None;
    if (s == "temp_tower")               return CalibMode::Calib_Temp_Tower;
    if (s == "retraction_tower")         return CalibMode::Calib_Retraction_tower;
    if (s == "pressure_advance_line")    return CalibMode::Calib_PA_Line;
    if (s == "pressure_advance_pattern") return CalibMode::Calib_PA_Pattern;
    if (s == "pressure_advance_tower")   return CalibMode::Calib_PA_Tower;
    throw std::invalid_argument("unknown --calib-mode '" + s + "'");
}

std::string calib_mode_name(CalibMode mode)
{
    switch (mode) {
        case CalibMode::Calib_None:             return "none";
        case CalibMode::Calib_Temp_Tower:       return "temp_tower";
        case CalibMode::Calib_Retraction_tower: return "retraction_tower";
        case CalibMode::Calib_PA_Line:          return "pressure_advance_line";
        case CalibMode::Calib_PA_Pattern:       return "pressure_advance_pattern";
        case CalibMode::Calib_PA_Tower:         return "pressure_advance_tower";
        default:                                return "unsupported";
    }
}

bool calib_mode_generates_geometry(CalibMode mode)
{
    return mode == CalibMode::Calib_PA_Pattern;
}

namespace {
// Canonical sweep defaults per mode, used for any of start/end/step omitted on
// the CLI. Mirrors the typical Orca/BBS calibration ranges (see Calib.hpp).
struct SweepDefaults { double start, end, step; };

SweepDefaults default_sweep(CalibMode mode)
{
    switch (mode) {
        // Temperature tower descends from the high temp; the engine steps 5 C
        // per 10 mm of height regardless of `step`, so end/step are advisory.
        case CalibMode::Calib_Temp_Tower:       return {240.0, 190.0, 5.0};
        case CalibMode::Calib_Retraction_tower: return {0.0,   2.0,   0.1};
        case CalibMode::Calib_PA_Line:          return {0.0,   0.10,  0.002};
        case CalibMode::Calib_PA_Pattern:       return {0.0,   0.08,  0.005};
        case CalibMode::Calib_PA_Tower:         return {0.0,   0.10,  0.002};
        default:                                return {0.0,   0.0,   0.0};
    }
}
} // namespace

Calib_Params build_calib_params(const CalibOptions& opts)
{
    Calib_Params params;
    params.mode = parse_calib_mode(opts.mode);
    if (params.mode == CalibMode::Calib_None)
        return params;

    const SweepDefaults def = default_sweep(params.mode);
    params.start = opts.has_start ? opts.start : def.start;
    params.end   = opts.has_end   ? opts.end   : def.end;
    params.step  = opts.has_step  ? opts.step  : def.step;
    params.extruder_id   = std::max(0, opts.extruder_id);
    params.print_numbers = opts.print_numbers;

    // Only pressure_advance_pattern actually consumes Calib_Params::extruder_id
    // (apply_pa_pattern scales widths / picks the wall speed / assigns the handle
    // for it). The other modes go through Print::set_calib_params + the engine's
    // per-layer hooks, which read the CURRENT writer filament — they ignore
    // extruder_id. Accepting a nonzero id there would silently calibrate the
    // wrong extruder, so reject it rather than mislead.
    if (params.extruder_id != 0 && params.mode != CalibMode::Calib_PA_Pattern)
        throw std::invalid_argument(
            "--calib-extruder-id is only honored by pressure_advance_pattern; other modes "
            "calibrate the model's current extruder");

    if (!std::isfinite(params.start) || !std::isfinite(params.end) || !std::isfinite(params.step))
        throw std::invalid_argument("--calib-start/--calib-end/--calib-step must be finite numbers");
    if (params.step <= 0.0)
        throw std::invalid_argument("--calib-step must be > 0");
    if (std::abs(params.end - params.start) < params.step)
        throw std::invalid_argument("--calib range (|end - start|) must be at least one --calib-step");

    // Pressure-advance modes count their patterns/lines with `ceil((end-start)/
    // step + 1)` consumed by an UNSIGNED loop bound (Calib.hpp get_num_patterns /
    // generate_test). A reversed (start > end) range yields a negative count that
    // wraps to a near-unbounded value — so require ascending for PA modes. Temp
    // and retraction towers use per-layer formulas (no count loop) and temp
    // towers are conventionally high-to-low, so descending stays allowed there.
    if ((params.mode == CalibMode::Calib_PA_Line ||
         params.mode == CalibMode::Calib_PA_Pattern ||
         params.mode == CalibMode::Calib_PA_Tower) &&
        params.end <= params.start)
        throw std::invalid_argument(
            "pressure-advance --calib-end must be greater than --calib-start (ascending sweep)");

    // The temperature tower descends from `start` (engine hook: temp = start -
    // floor(z/10.001)*5; `end` is advisory and unused). An ascending range
    // (start < end) would sweep DOWN from the low start, away from the intended
    // high temp — almost always a mistake. Require start > end (high → low).
    if (params.mode == CalibMode::Calib_Temp_Tower && params.start <= params.end)
        throw std::invalid_argument(
            "temp_tower --calib-start must be greater than --calib-end (the tower descends "
            "from the start temperature)");

    return params;
}

void apply_pa_pattern(const Calib_Params& params,
                      DynamicPrintConfig& config,
                      Model& model,
                      bool is_bbl_machine)
{
#ifndef ENGINE_BAMBU
    // OrcaSlicer's CalibPressureAdvancePattern API diverges from BambuStudio's:
    // its ctor + generate_custom_gcodes take a ModelObject (not Model) and return
    // a CustomGCode::Info (calib.hpp:297-308), so the Bambu-shaped port below
    // would not compile against Orca's libslic3r. A separate Orca port is a
    // follow-up; until then this mode is rejected for the Orca engine (the cli
    // also blocks it up front — see main.cpp). Tower + PA-line modes work on both
    // engines via the common Print::set_calib_params path.
    (void) params; (void) config; (void) model; (void) is_bbl_machine;
    throw std::runtime_error(
        "pressure_advance_pattern is not yet supported on the OrcaSlicer engine build");
#else
    // ── Port of CalibUtils::calib_pa_pattern (GUI-free). The device-coupled
    // bits (MachineObject, logical-extruder remap, get_index_for_extruder_
    // parameter) collapse to the CLI's single logical extruder.
    const int extruder_id = std::max(0, params.extruder_id);

    // Use the CALIBRATED extruder's nozzle diameter (not always nozzle 0) so a
    // second, differently-sized nozzle scales its suggested widths correctly.
    float nozzle_diameter = 0.4f;
    if (auto* nd = config.option<ConfigOptionFloatsNullable>("nozzle_diameter"))
        if (!nd->values.empty()) {
            const int ni = (extruder_id < static_cast<int>(nd->values.size())) ? extruder_id : 0;
            nozzle_diameter = static_cast<float>(nd->get_at(ni));
        }

    const SuggestedConfigCalibPAPattern suggested;
    for (const auto& opt : suggested.float_pairs)
        config.set_key_value(opt.first, new ConfigOptionFloat(opt.second));
    for (const auto& opt : suggested.floats_pairs)
        config.set_key_value(opt.first, new ConfigOptionFloatsNullable(opt.second));
    // Apply the suggested calibration line widths BEFORE optimizing the PA speed,
    // so find_optimal_PA_speed uses the width the pattern actually prints at (the
    // loaded profile's line_width can differ and would skew the result).
    for (const auto& opt : suggested.nozzle_ratio_pairs)
        config.set_key_value(opt.first, new ConfigOptionFloat(nozzle_diameter * opt.second / 100));

    const float wall_speed = CalibPressureAdvance::find_optimal_PA_speed(
        config, config.get_abs_value("line_width"), config.get_abs_value("layer_height"),
        extruder_id, 0);
    if (auto* ws = config.option<ConfigOptionFloatsNullable>("outer_wall_speed")) {
        if (!ws->values.empty()) {
            const int idx = (extruder_id < static_cast<int>(ws->values.size())) ? extruder_id : 0;
            ws->values[idx] = wall_speed;
        }
    }

    for (const auto& opt : suggested.int_pairs)
        config.set_key_value(opt.first, new ConfigOptionInt(opt.second));
    config.set_key_value(suggested.brim_pair.first,
                         new ConfigOptionEnum<BrimType>(suggested.brim_pair.second));

    // The pressure_advance_pattern is self-contained: its handle cube + the
    // per-layer pattern G-code ARE the print, so the loaded model (if any) is
    // replaced. Config still comes from --input (3MF / profile bundle) — the
    // engine cannot slice from defaults alone. The pattern constructor
    // dereferences model.objects.front()->volumes.front(), so the synthesized
    // object must exist before construction; final dims come from the pattern.
    model.clear_objects();
    {
        ModelObject* obj = model.add_object("pa_pattern_handle", "", make_cube(1.0, 1.0, 1.0));
        // Assign the handle to the calibrated extruder (1-indexed, mirrors
        // Model.cpp:480) so the pattern writer derives the right filament; without
        // this a nonzero --calib-extruder-id is silently ignored for the geometry.
        obj->config.set_key_value("extruder", new ConfigOptionInt(extruder_id + 1));
        obj->add_instance();
    }

    const Vec3d plate_origin(0, 0, 0);
    CalibPressureAdvancePattern pa_pattern(params, config, is_bbl_machine, model, plate_origin);

    // Bowden extruders emit `M901 P<k>` instead of `M900 K<k>`
    // (GCodeWriter::set_pressure_advance). Mirror CalibUtils.cpp:975-976 /
    // Plater.cpp:17318-17323 — gated on is_bbl_machine (M901 is the BBL bowden
    // dialect). Without this a Bowden printer's PA pattern emits the wrong code.
    if (is_bbl_machine) {
        if (auto* et = config.option<ConfigOptionEnumsGeneric>("extruder_type")) {
            if (extruder_id < static_cast<int>(et->values.size()) &&
                et->values[extruder_id] == static_cast<int>(ExtruderType::etBowden)) {
                pa_pattern.set_bbl_bowden_mode();
            }
        }
    }

    ModelInstance* handle_inst = model.objects.front()->instances.front();
    handle_inst->set_scaling_factor(
        Vec3d(pa_pattern.handle_xy_size(), pa_pattern.handle_xy_size(), pa_pattern.max_layer_z()));

    // Center the pattern on the printable area (mirror CalibUtils:978-983).
    if (auto* pa = config.opt<ConfigOptionPoints>("printable_area")) {
        const auto& bedfs = pa->values;
        if (bedfs.size() >= 4) {
            const double current_width = bedfs[2].x() - bedfs[0].x();
            const double current_depth = bedfs[2].y() - bedfs[0].y();
            const Vec3d  half_size(pa_pattern.print_size_x() / 2, pa_pattern.print_size_y() / 2, 0);
            // Anchor at the bed's MIN corner (bedfs[0]) so a centered-coordinate /
            // offset bed (origin != 0,0) still lands the pattern in the printable
            // area. For a 0-origin bed (the Bambu engine case) this is unchanged.
            const Vec3d  bed_center(bedfs[0].x() + current_width / 2, bedfs[0].y() + current_depth / 2, 0);
            const Vec3d  offset = bed_center - half_size;
            pa_pattern.set_start_offset(offset);
            // Place the handle BELOW the pattern start so it abuts (not overlaps)
            // the pattern frame. This inverts the engine's natural start formula
            // (_refresh_starting_point: start = {bbox.min.x, bbox.max.y +
            // handle_spacing}) — i.e. handle top edge = start.y - handle_spacing,
            // so the handle's min-y corner sits at start.y - spacing - xy_size.
            handle_inst->set_offset(Vec3d(
                offset.x(),
                offset.y() - pa_pattern.handle_spacing() - pa_pattern.handle_xy_size(),
                0));
        }
    }

    pa_pattern.generate_custom_gcodes(config, is_bbl_machine, model, plate_origin);
    model.calib_pa_pattern = std::make_unique<CalibPressureAdvancePattern>(pa_pattern);
#endif  // ENGINE_BAMBU
}

} // namespace slicer_cli
