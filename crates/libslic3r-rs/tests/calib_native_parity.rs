//! `slicer::calib` (the port of Calib.hpp / Calib.cpp) against the C++ at the pin: the
//! configuration and geometry members of `CalibPressureAdvancePattern`, the arithmetic members of
//! `CalibPressureAdvance`, `find_optimal_PA_speed` and `SuggestedConfigCalibPAPattern`.
//!
//! `tests/data/calib/calib.txt` is the output of `gen.cpp` beside it, which links the pin's
//! libslic3r and runs the same cases (a configuration set from text, a model of one box with one
//! instance, Calib_Params); the header of gen.cpp describes the lines. Every line has to be the
//! same, doubles bit for bit.

use slicer::calib::{CalibMode, CalibParams, CalibPressureAdvance, CalibPressureAdvancePattern, FullDynamicPrintConfig, SuggestedConfigCalibPAPattern};
use slicer::config::{ConfigBase, ConfigOptionFloats, ConfigOptionFloatsNullable, ConfigOptionResolverTyped, ConfigSubstitutionContext, ForwardCompatibilitySubstitutionRule};
use slicer::geometry::geometry::Vec3d;
use slicer::geometry::PointF;
use slicer::model::{Model, ModelMesh, ModelVolumeType};
use slicer::triangle_mesh::its_make_cube;

fn hd(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}

fn ht(s: &str) -> String {
    if s.is_empty() {
        return "-".to_string();
    }
    s.bytes().map(|c| format!("{:02x}", c)).collect()
}

struct Case {
    opts: Vec<(&'static str, &'static str)>,
    object_opts: Vec<(&'static str, &'static str)>,
    start: f64,
    end: f64,
    step: f64,
    bx: [f64; 3],
    offset: [f64; 3],
}

fn cases() -> Vec<Case> {
    let base = |nd, lw, ilw, wl, ilh, lh, ils, ows, fmvs| {
        vec![
            ("nozzle_diameter", nd),
            ("line_width", lw),
            ("initial_layer_line_width", ilw),
            ("wall_loops", wl),
            ("initial_layer_print_height", ilh),
            ("layer_height", lh),
            ("initial_layer_speed", ils),
            ("outer_wall_speed", ows),
            ("filament_max_volumetric_speed", fmvs),
        ]
    };
    vec![
        Case {
            opts: base("0.4", "0.45", "0.5", "3", "0.25", "0.2", "30", "200", "21"),
            object_opts: vec![],
            start: 0.0,
            end: 0.1,
            step: 0.002,
            bx: [20.0, 30.0, 1.0],
            offset: [100.0, 120.0, 0.0],
        },
        Case {
            opts: base("0.6", "0.62", "0.75", "2", "0.3", "0.3", "25", "60", "12"),
            object_opts: vec![("wall_loops", "4")],
            start: 0.01,
            end: 0.05,
            step: 0.005,
            bx: [35.5, 12.25, 2.0],
            offset: [-10.5, 33.3, 0.0],
        },
        Case {
            opts: base("0.2", "0.22", "0.28", "3", "0.1", "0.08", "20", "150", "3.5"),
            object_opts: vec![],
            start: 0.0,
            end: 0.04,
            step: 0.004,
            bx: [10.0, 10.0, 0.5],
            offset: [50.0, 50.0, 0.0],
        },
        Case {
            opts: base("0.4", "0.42", "0.5", "1", "0.2", "0.16", "50", "300", "40"),
            object_opts: vec![("layer_height", "0.12")],
            start: 0.02,
            end: 0.08,
            step: 0.0025,
            bx: [60.0, 40.0, 3.0],
            offset: [0.0, 0.0, 0.0],
        },
        Case {
            opts: base("0.8", "0.9", "1", "5", "0.4", "0.4", "35", "80", "30"),
            object_opts: vec![],
            start: 0.1,
            end: 1.0,
            step: 0.1,
            bx: [80.0, 25.0, 1.5],
            offset: [12.75, -8.5, 0.0],
        },
        Case {
            opts: base("0.4", "0.45", "0.5", "3", "0.25", "0.2", "30", "200", "21"),
            object_opts: vec![],
            start: 0.0,
            end: 0.0333,
            step: 0.001111,
            bx: [20.0, 30.0, 1.0],
            offset: [100.0, 120.0, 0.0],
        },
    ]
}

fn floats_at(config: &FullDynamicPrintConfig, key: &str) -> f64 {
    if let Some(o) = config.option_typed::<ConfigOptionFloatsNullable>(key) {
        return *o.get_at(0);
    }
    *config.option_typed::<ConfigOptionFloats>(key).expect(key).get_at(0)
}

#[test]
fn calib_matches_the_pin() {
    let mut lines: Vec<String> = Vec::new();
    let mut out = |name: &str, v: String| lines.push(format!("{name} {v}"));
    for (n, c) in cases().iter().enumerate() {
        out("case", n.to_string());
        let mut config = FullDynamicPrintConfig::full_print_config();
        for (k, v) in &c.opts {
            config.set_deserialize_strict(k, v, false).unwrap();
        }
        let mut model = Model::new();
        {
            let obj = model.add_object_new();
            obj.add_volume_mesh(ModelMesh::from_its(its_make_cube(c.bx[0], c.bx[1], c.bx[2])), ModelVolumeType::ModelPart, true);
            for (k, v) in &c.object_opts {
                let mut ctxt = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Disable);
                obj.config.set_deserialize(k, v, &mut ctxt, false).unwrap();
            }
            obj.add_instance().set_offset(&Vec3d::new(c.offset[0], c.offset[1], c.offset[2]));
        }
        let params = CalibParams { start: c.start, end: c.end, step: c.step, mode: CalibMode::PAPattern, ..CalibParams::default() };
        let mut p = CalibPressureAdvancePattern::new(params, &config, false, &model, &Vec3d::new(5.0, 7.0, 0.0)).unwrap();

        out("handle_xy_size", hd(p.handle_xy_size()));
        out("handle_spacing", hd(p.handle_spacing()));
        let so = p.get_start_offset();
        out("get_start_offset", format!("{} {} {}", hd(so.0), hd(so.1), hd(so.2)));
        out("speed_first_layer", hd(p.speed_first_layer()));
        out("speed_perimeter", hd(p.speed_perimeter()));
        out("line_width_first_layer", hd(p.line_width_first_layer()));
        out("line_width", hd(p.line_width()));
        out("wall_count", p.wall_count().to_string());
        out("height_first_layer", hd(p.height_first_layer()));
        out("height_layer", hd(p.height_layer()));
        out("get_num_patterns", p.get_num_patterns().to_string());
        out("line_spacing", hd(p.line_spacing()));
        out("line_spacing_first_layer", hd(p.line_spacing_first_layer()));
        out("line_spacing_angle", hd(p.line_spacing_angle()));
        out("object_size_x", hd(p.object_size_x()));
        out("object_size_y", hd(p.object_size_y()));
        out("frame_size_y", hd(p.frame_size_y()));
        out("glyph_start_x", (0..6).map(|i| hd(p.glyph_start_x(i))).collect::<Vec<_>>().join(" "));
        out("glyph_length_x", hd(p.glyph_length_x()));
        out("glyph_tab_max_x", hd(p.glyph_tab_max_x()));
        out("max_numbering_height", hd(p.max_numbering_height()));
        out("pattern_shift", hd(p.pattern_shift()));
        out("print_size_x", hd(p.print_size_x()));
        out("print_size_y", hd(p.print_size_y()));
        out("max_layer_z", hd(p.max_layer_z()));
        p.set_start_offset((-3.5, 4.25, 0.5));
        let so = p.get_start_offset();
        out("set_start_offset", format!("{} {} {} {}", hd(so.0), hd(so.1), hd(so.2), hd(p.glyph_tab_max_x())));

        let speed = CalibPressureAdvance::find_optimal_pa_speed(&config, p.line_width(), p.height_layer(), 0, 0).unwrap();
        out("find_optimal_PA_speed", format!("{:08x}", speed.to_bits()));
        let e = p
            .base
            .e_per_mm(
                p.line_width(),
                p.height_layer(),
                floats_at(&config, "nozzle_diameter") as f32,
                floats_at(&config, "filament_diameter") as f32,
                floats_at(&config, "filament_flow_ratio") as f32,
            )
            .unwrap();
        out("e_per_mm", hd(e));
        out("get_distance", hd(p.base.get_distance(PointF::new(c.offset[0], c.offset[1]), PointF::new(c.bx[0], c.bx[1]))));
        out("to_radians", hd(p.base.to_radians(c.bx[0])));
        out("number_spacing", hd(p.base.number_spacing()));
    }

    out("case", "strings".to_string());
    let base = CalibPressureAdvance::new(FullDynamicPrintConfig::default());
    for v in [0.0, 0.002, 0.1, 0.035, 1.0 / 3.0, 12.5, 100.0, -0.004, 0.0333, 1e-7, 2.0000005, 123456.789, -0.0] {
        out("convert_number_to_string", format!("{} {}", hd(v), ht(&base.convert_number_to_string(v))));
    }
    let sc = SuggestedConfigCalibPAPattern::default();
    for d in [0.2, 0.4, 0.2000001, 0.1999995, 0.6] {
        let fp = sc.float_pairs(d);
        out(
            "float_pairs",
            format!(
                "{} {} {} {} {} {}",
                hd(d),
                hd(sc.initial_layer_print_height(d)),
                ht(&fp[0].0),
                hd(fp[0].1),
                ht(&fp[1].0),
                hd(fp[1].1)
            ),
        );
    }

    let path = format!("{}/tests/data/calib/calib.txt", env!("CARGO_MANIFEST_DIR"));
    let want = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let want: Vec<&str> = want.lines().collect();
    assert_eq!(lines.len(), want.len(), "number of lines");
    let bad: Vec<String> = lines
        .iter()
        .zip(want.iter())
        .enumerate()
        .filter(|(_, (g, w))| g != w)
        .map(|(k, (g, w))| format!("line {}:\n  rust {g}\n  c++  {w}", k + 1))
        .collect();
    println!("compared {} lines", lines.len());
    assert!(bad.is_empty(), "{} lines differ:\n{}", bad.len(), bad.join("\n"));
}

/// The G-code the calibration classes write through their GCodeWriter, against
/// `tests/data/calib/calib_gcode.txt.gz` (`gen gcode`): the pattern's custom G-code items, the
/// drawing members on the pattern's writer, and the line test on a GCode generator.
#[test]
fn calib_gcode_matches_the_pin() {
    use slicer::calib::{CalibPressureAdvanceLine, DrawBoxOptArgs, DrawDigitMode};
    use slicer::g_code::gcode_class::GCode;
    use std::io::Read;

    struct BoxArgs(f64, f64, f64, f64, i32, f64, f64, f64, bool);
    let beds = [
        "0x0,256x0,256x256,0x256",
        "0x0,180x0,180x180,0x180",
        "0x0,250x0,250x220,0x220",
        "0x0,350x0,350x320,0x320",
        "-90x0,-64x-64,0x-90,64x-64,90x0,64x64,0x90,-64x64",
        "10x10,110x10,110x80,10x80",
    ];
    let boxes = [
        BoxArgs(30.0, 40.0, 12.0, 5.0, 2, 0.2, 0.45, 3000.0, true),
        BoxArgs(5.0, 6.0, 5.0, 14.0, 3, 0.25, 0.5, 1800.0, true),
        BoxArgs(8.5, 9.25, 8.0, 8.0, 3, 0.2, 0.42, 1200.0, false),
        BoxArgs(0.0, 0.0, 40.0, 3.0, 5, 0.3, 0.62, 2400.0, true),
        BoxArgs(-3.0, 2.0, 2.0, 2.0, 4, 0.2, 0.4, 600.0, true),
        BoxArgs(50.0, 60.0, 23.7, 9.3, 1, 0.16, 0.44, 4800.0, true),
    ];
    let mode_of = |m: i32| if m == 0 { DrawDigitMode::LeftToRight } else { DrawDigitMode::BottomToTop };

    let mut lines: Vec<String> = Vec::new();
    for (n, c) in cases().iter().enumerate() {
        let mut config = FullDynamicPrintConfig::full_print_config();
        for (k, v) in &c.opts {
            config.set_deserialize_strict(k, v, false).unwrap();
        }
        let mut model = Model::new();
        {
            let obj = model.add_object_new();
            obj.add_volume_mesh(ModelMesh::from_its(its_make_cube(c.bx[0], c.bx[1], c.bx[2])), ModelVolumeType::ModelPart, true);
            for (k, v) in &c.object_opts {
                let mut ctxt = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Disable);
                obj.config.set_deserialize(k, v, &mut ctxt, false).unwrap();
            }
            obj.add_instance().set_offset(&Vec3d::new(c.offset[0], c.offset[1], c.offset[2]));
        }
        let params = CalibParams { start: c.start, end: c.end, step: c.step, mode: CalibMode::PAPattern, ..CalibParams::default() };
        let origin = Vec3d::new(5.0, 7.0, 0.0);
        let mut p = CalibPressureAdvancePattern::new(params, &config, false, &model, &origin).unwrap();
        if n == 4 {
            p.base.set_bbl_bowden_mode();
        }
        p.generate_custom_gcodes(&config, false, &mut model, &origin).unwrap();
        let info = &model.plates_custom_gcodes[&model.curr_plate_index];
        lines.push(format!("pattern {} {} {}", n, info.mode as i32, info.gcodes.len()));
        for item in &info.gcodes {
            lines.push(format!("item {} {} {}", hd(item.print_z), item.gcode_type as i32, ht(&item.extra)));
        }

        for mode in 0..2 {
            for ch in "0123456789.x".bytes() {
                let s = p.base.draw_digit(10.5, 20.25, ch, mode_of(mode), 0.45, 0.0331, &mut p.writer);
                lines.push(format!("digit {} {} {}", mode, ch as char, ht(&s)));
            }
        }
        for mode in 0..2 {
            for v in [0.0325, 1.5, 12345.678, 0.0001, 0.1] {
                let s = p.base.draw_number(12.5, 30.25, v, mode_of(mode), 0.42, 0.034, 1800.0, &mut p.writer);
                lines.push(format!("number {} {} {}", mode, hd(v), ht(&s)));
            }
        }
        for b in &boxes {
            let mut args = DrawBoxOptArgs::new(b.4, b.5, b.6, b.7);
            args.is_filled = b.8;
            let s = p.base.draw_box(&mut p.writer, b.0, b.1, b.2, b.3, args).unwrap();
            lines.push(format!("box {}", ht(&s)));
        }
        let s = p.base.draw_line(&mut p.writer, PointF::new(77.125, 13.5), 0.45, 0.2, 2400.0, "comment").unwrap();
        lines.push(format!("line_to {}", ht(&s)));
        let s = p.base.move_to(PointF::new(1.0625, 2.5), &mut p.writer, "Glyph");
        lines.push(format!("move_to {}", ht(&s)));

        let mut line_config = config.clone();
        line_config.set_deserialize_strict("printable_area", beds[n % 6], false).unwrap();
        let mut g = GCode::new();
        g.m_config = line_config.clone();
        g.m_writer.apply_print_config(&line_config);
        g.m_writer.set_extruders(vec![0]);
        g.m_writer.set_extruder(0, 0);
        let mut pa = CalibPressureAdvanceLine::new(&mut g).unwrap();
        pa.set_speed(if n == 1 { 180.7 } else { 100.0 }, if n == 1 { 25.3 } else { 20.0 });
        *pa.draw_numbers() = n % 2 == 0 || n == 5;
        if n == 3 {
            pa.base.set_bbl_bowden_mode();
        }
        let count = (((c.end - c.start) / c.step).ceil().round() as i64 + 1) as i32;
        let s = pa.generate_test(c.start, c.step, count).unwrap();
        lines.push(format!(
            "line {} {} {} {} {} {}",
            n,
            count,
            pa.is_delta() as i32,
            hd(pa.line_width()),
            hd(pa.height_layer()),
            ht(&s)
        ));
    }

    let path = format!("{}/tests/data/calib/calib_gcode.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut want = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut want).expect("gzip");
    let want: Vec<&str> = want.lines().collect();
    assert_eq!(lines.len(), want.len(), "number of lines");
    let bad: Vec<String> = lines
        .iter()
        .zip(want.iter())
        .enumerate()
        .filter(|(_, (g, w))| g != w)
        .map(|(k, (g, w))| format!("line {}:\n  rust {}\n  c++  {}", k + 1, &g[..g.len().min(600)], &w[..w.len().min(600)]))
        .collect();
    println!("compared {} lines", lines.len());
    assert!(bad.is_empty(), "{} lines differ:\n{}", bad.len(), bad.join("\n"));
}
