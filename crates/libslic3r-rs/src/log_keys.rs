//! Port of `LogKeys.hpp`: field names for `kv` in log records.
//!
//! C++ Reference:
//! - src/libslic3r/LogKeys.hpp
//!
//! Use these instead of string literals so the same value is always logged under the same name.
//! Values of sensitive fields are masked in public builds; see `crate::log::kv`.

/// LogKeys.hpp:11-58 `namespace Slic3r::log_key`
pub mod log_key {
    // Sensitive: masked in public builds
    /// LogKeys.hpp:14
    pub const DEV_ID: &str = "dev_id";
    /// LogKeys.hpp:15
    pub const SN: &str = "sn";
    /// LogKeys.hpp:16
    pub const DEV_IP: &str = "dev_ip";
    /// LogKeys.hpp:17
    pub const DEV_NAME: &str = "dev_name";
    /// LogKeys.hpp:18
    pub const UID: &str = "uid";
    /// LogKeys.hpp:19
    pub const ACCESS_CODE: &str = "access_code";
    /// LogKeys.hpp:20
    pub const TOKEN: &str = "token";
    /// LogKeys.hpp:21
    pub const PASSWORD: &str = "password";
    /// LogKeys.hpp:22
    pub const AUTHKEY: &str = "authkey";
    /// LogKeys.hpp:23
    pub const TTCODE_ENC: &str = "ttcode_enc";
    /// LogKeys.hpp:24
    pub const URL: &str = "url";
    /// LogKeys.hpp:25
    pub const REGION: &str = "region";
    /// LogKeys.hpp:26
    pub const MODEL_ID: &str = "model_id";

    // Not sensitive
    /// LogKeys.hpp:29
    pub const SEQ: &str = "seq";
    /// LogKeys.hpp:30
    pub const RET: &str = "ret";
    /// LogKeys.hpp:31
    pub const TASK_ID: &str = "task_id";
    /// LogKeys.hpp:32
    pub const SUBTASK_ID: &str = "subtask_id";
    /// LogKeys.hpp:33
    pub const JOB_ID: &str = "job_id";
    /// LogKeys.hpp:34
    pub const PLATE: &str = "plate";
    /// LogKeys.hpp:35
    pub const PLATE_ID: &str = "plate_id";
    /// LogKeys.hpp:36
    pub const NETWORK_AGENT: &str = "network_agent";
    /// LogKeys.hpp:37
    pub const HTTP_CODE: &str = "http_code";
    /// LogKeys.hpp:38
    pub const FILAMENT_ID: &str = "filament_id";
    /// LogKeys.hpp:39
    pub const SETTING_ID: &str = "setting_id";
    /// LogKeys.hpp:40
    pub const BASE_ID: &str = "base_id";
    /// LogKeys.hpp:41
    pub const AMS_ID: &str = "ams_id";
    /// LogKeys.hpp:42
    pub const SLOT_ID: &str = "slot_id";
    /// LogKeys.hpp:43
    pub const TRAY_ID: &str = "tray_id";
    /// LogKeys.hpp:44
    pub const EXTRUDER_ID: &str = "extruder_id";
    /// LogKeys.hpp:45
    pub const NOZZLE_DIAMETER: &str = "nozzle_diameter";
    /// LogKeys.hpp:46
    pub const FILE_COUNT: &str = "file_count";
    /// LogKeys.hpp:47
    pub const THUMBNAIL_COUNT: &str = "thumbnail_count";

    // Environment, logged once at startup and when OpenGL is set up
    /// LogKeys.hpp:50
    pub const APP_VERSION: &str = "app_version";
    /// LogKeys.hpp:51
    pub const BUILD_TIME: &str = "build_time";
    /// LogKeys.hpp:52
    pub const BUILD_COMMIT: &str = "build_commit";
    /// LogKeys.hpp:53
    pub const GL_VERSION: &str = "gl_version";
    /// LogKeys.hpp:54
    pub const GLSL_VERSION: &str = "glsl_version";
    /// LogKeys.hpp:55
    pub const GPU_VENDOR: &str = "gpu_vendor";
    /// LogKeys.hpp:56
    pub const GPU: &str = "gpu";
}
