//! AppConfig::get_mouse_device_lock_horizon (AppConfig.hpp:257-258) and
//! dual_extruder_first_slice_video_app_config_key (AppConfig.hpp:303-315), get_language_code and
//! get_hms_host (AppConfig.cpp:48-82) against the pin's
//! (`tests/data/app_config_keys/gen.cpp`).

use slicer::app_config::{dual_extruder_first_slice_video_app_config_key, AppConfig};

fn unhex(s: &str) -> Vec<u8> {
    if s == "-" {
        return Vec::new();
    }
    (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
}

#[test]
fn app_config_keys_match_the_pin() {
    let text = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/app_config_keys/vectors.txt")).unwrap();
    let mut cfg = AppConfig::new();
    let mut cases = 0;
    for line in text.lines() {
        let f: Vec<&str> = line.split(' ').collect();
        match f[0] {
            "lock" => {
                let v = String::from_utf8(unhex(f[1])).unwrap();
                cfg.set("mouse_device:dev", "lock_horizon", &v);
                let got = cfg.get_mouse_device_lock_horizon("dev");
                let (found, lock) = (f[2] == "1", f[3] == "1");
                assert_eq!(got.is_some(), found, "{v:?}");
                assert_eq!(got.unwrap_or(false), lock, "{v:?}");
            }
            "lock_missing_device" => assert_eq!(cfg.get_mouse_device_lock_horizon("other").is_some(), f[1] == "1"),
            "lock_missing_key" => {
                cfg.set("mouse_device:dev2", "swap_yz", "1");
                assert_eq!(cfg.get_mouse_device_lock_horizon("dev2").is_some(), f[1] == "1");
            }
            "key" => {
                let model = String::from_utf8(unhex(f[1])).unwrap();
                assert_eq!(dual_extruder_first_slice_video_app_config_key(&model).into_bytes(), unhex(f[2]), "{model:?}");
            }
            "lang" => {
                cfg.set("app", "language", &String::from_utf8(unhex(f[1])).unwrap());
                assert_eq!(cfg.get_language_code().into_bytes(), unhex(f[2]), "{line}");
            }
            "hms" => {
                cfg.set("app", "iot_environment", &String::from_utf8(unhex(f[1])).unwrap());
                assert_eq!(cfg.get_hms_host().into_bytes(), unhex(f[2]), "{line}");
            }
            other => panic!("unknown line {other}"),
        }
        cases += 1;
    }
    assert_eq!(cases, 46);
}
