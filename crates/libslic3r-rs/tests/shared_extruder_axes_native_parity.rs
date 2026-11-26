//! Extruder.cpp 5–6, 36–60, 71–149 and Extruder.hpp 19–40. The constructor
//! and reset operations affect other shared extruders, including across an
//! ordered thread handoff. Each tool has its own filament usage and restart extra.
use slicer::extruder::{Extruder, GCodeConfig};
use std::fmt::Write;

fn snapshots() -> String {
    let mut output = String::new();
    for relative in [false, true] {
        for hotend in [false, true] {
            let mut config = GCodeConfig::new();
            config.use_relative_e_distances = relative;
            config.handle_hotend_as_extruder = hotend;
            config.filament_map.values = vec![1, 1, 2];
            config.filament_nozzle_map.values = vec![4, 3, 4];
            let mut a = Extruder::new(0, &config, true);
            let mut b = Extruder::new(1, &config, true);
            let mut c = Extruder::new(2, &config, true);
            let mut separate = Extruder::new(0, &config, false);
            macro_rules! record {
                ($op:literal, $result:expr) => {{
                    let value: f64 = $result;
                    write!(
                        output,
                        "{} {} {} {:016x}",
                        u8::from(relative),
                        u8::from(hotend),
                        $op,
                        value.to_bits()
                    )
                    .unwrap();
                    for e in [&a, &b, &c, &separate] {
                        write!(
                            output,
                            " {:016x} {:016x} {:016x}",
                            e.e().to_bits(),
                            e.get_share_retracted_length().to_bits(),
                            e.used_filament().to_bits()
                        )
                        .unwrap();
                    }
                    output.push('\n');
                }};
            }
            record!("init", 0.);
            record!("a-extrude", a.extrude(3.25));
            record!("b-negative", b.extrude(-0.5));
            record!("c-retract", c.retract(1.25, 0.25));
            record!("a-shorter", a.retract(0.125, 0.625));
            record!("b-equal", b.retract(0.5, 0.375));
            record!("b-repay", b.unretract());
            record!("a-longer", a.retract(2., 0.375));
            record!("c-repay", c.unretract());
            record!("a-repay", a.unretract());
            record!("separate", separate.extrude(8.));
            separate.reset_e();
            record!("reset-e", 0.);
            std::thread::scope(|scope| {
                scope.spawn(|| {
                    let mut peer = Extruder::new(1, &config, true);
                    peer.extrude(7.);
                    peer.retract(1.75, 0.125);
                });
            });
            record!("worker", 0.);
            record!("b-worker-repay", b.unretract());
            a.reset();
            record!("reset", 0.);
        }
    }
    output
}

#[test]
fn shared_axes_match_native_across_extruders_and_threads() {
    assert_eq!(
        snapshots(),
        include_str!("data/g_code_writer/shared_axes.txt")
    );
    // Keep this check in the same test. Constructing a shared extruder
    // resets the process-wide axes for every writer in this executable.
    let mut config = GCodeConfig::new();
    config.use_relative_e_distances = false;
    config.filament_map.values = vec![1, 1];
    let observer = Extruder::new(0, &config, true);
    let barrier = std::sync::Barrier::new(4);
    std::thread::scope(|scope| {
        for id in [0, 1, 0, 1] {
            let config = &config;
            let barrier = &barrier;
            scope.spawn(move || {
                let mut tool = Extruder::new(id, config, true);
                barrier.wait();
                for _ in 0..512 {
                    tool.extrude(0.125);
                }
                barrier.wait();
                tool.retract(2.0, 0.0);
            });
        }
    });
    assert_eq!(observer.e(), 4.0 * 512.0 * 0.125 - 2.0);
    assert_eq!(observer.get_share_retracted_length(), 2.0);
}
