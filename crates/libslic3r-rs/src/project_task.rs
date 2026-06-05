//! Faithful 1:1 port of `ProjectTask.cpp` / `ProjectTask.hpp` (BambuStudio).
//!
//! C++ Reference:
//! - src/libslic3r/ProjectTask.hpp
//! - src/libslic3r/ProjectTask.cpp
//!
//! Pure data-model + cloud-task JSON parsing. No geometry / coord_t involved.
//!
//! Pointer notes: the C++ types store raw back-pointers to their parent
//! (`BBLProject*`, `BBLProfile*`, `BBLTask*`). Those raw pointers are not
//! load-bearing for G-code generation and cannot be represented as raw
//! pointers in safe/wasm Rust, so they are omitted; the constructors still
//! copy out exactly the same identifier fields the C++ constructors read off
//! the parent (see each `new`).

use serde_json::Value;

// ProjectTask.hpp:23
// enum MachineBedType
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MachineBedType {
    //BED_TYPE_AUTO = 0,
    // ProjectTask.hpp:25
    BedTypePc = 0,
    // ProjectTask.hpp:26
    BedTypePe,
    // ProjectTask.hpp:27
    BedTypePei,
    // ProjectTask.hpp:28
    BedTypePte,
    // ProjectTask.hpp:29
    BedTypeSupertack,
    // ProjectTask.hpp:30
    BedTypeCount,
}

// ProjectTask.hpp:33
// enum MappingResult
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MappingResult {
    // ProjectTask.hpp:34
    MappingResultDefault = 0,
    // ProjectTask.hpp:35
    MappingResultTypeMismatch = 1,
    // ProjectTask.hpp:36
    MappingResultExceed = 2,
}

// ProjectTask.hpp:39
// struct FilamentInfo
#[derive(Debug, Clone)]
pub struct FilamentInfo {
    pub id: i32,         // filament id = extruder id, start with 0.  ProjectTask.hpp:41
    pub type_: String,   // ProjectTask.hpp:42
    pub color: String,   // ProjectTask.hpp:43
    pub filament_id: String, // ProjectTask.hpp:44
    pub brand: String,   // ProjectTask.hpp:45
    pub used_m: f32,     // ProjectTask.hpp:46
    pub used_g: f32,     // ProjectTask.hpp:47
    pub tray_id: i32,    // start with 0  ProjectTask.hpp:48
    pub distance: f32,   // ProjectTask.hpp:49
    pub ctype: i32,      // ProjectTask.hpp:50
    pub colors: Vec<String>, // ProjectTask.hpp:51
    pub mapping_result: i32, // ProjectTask.hpp:52
    pub used_for_support: bool, // ProjectTask.hpp:53
    pub used_for_object: bool,  // ProjectTask.hpp:54

    /*for multi nozzle*/
    pub group_id: Vec<i32>,    // ProjectTask.hpp:57
    pub nozzle_diameter: f64,  // ProjectTask.hpp:58
    pub nozzle_volume_type: String, // ProjectTask.hpp:59

    /*for new ams mapping*/
    pub ams_id: String,  // ProjectTask.hpp:62
    pub slot_id: String, // ProjectTask.hpp:63
}

impl Default for FilamentInfo {
    // ProjectTask.hpp:39 (in-class member initializers)
    fn default() -> Self {
        FilamentInfo {
            id: 0,
            type_: String::new(),
            color: String::new(),
            filament_id: String::new(),
            brand: String::new(),
            used_m: 0.0,
            used_g: 0.0,
            tray_id: 0,
            distance: 0.0,
            ctype: 0,
            colors: Vec::new(),
            mapping_result: 0,
            used_for_support: false,
            used_for_object: false,
            group_id: Vec::new(),
            nozzle_diameter: 0.0,
            nozzle_volume_type: String::new(),
            ams_id: String::new(),
            slot_id: String::new(),
        }
    }
}

impl FilamentInfo {
    // ProjectTask.hpp:66
    pub fn get_ams_id(&self) -> i32 {
        if self.ams_id.is_empty() {
            return -1;
        }

        // try { return stoi(ams_id); } catch (...) {};
        if let Ok(v) = stoi(&self.ams_id) {
            return v;
        }

        -1
    }

    // ProjectTask.hpp:79
    pub fn get_slot_id(&self) -> i32 {
        if self.slot_id.is_empty() {
            return -1;
        }

        // try { return stoi(slot_id); } catch (...) {};
        if let Ok(v) = stoi(&self.slot_id) {
            return v;
        }

        -1
    }

    /*copied from AmsTray::get_display_filament_type()*/
    // ProjectTask.hpp:91
    pub fn get_display_filament_type(&self) -> String {
        if self.type_ == "PLA-S" {
            "Sup.PLA".to_string()
        } else if self.type_ == "PA-S" {
            "Sup.PA".to_string()
        } else if self.type_ == "ABS-S" {
            "Sup.ABS".to_string()
        } else {
            self.type_.clone()
        }
        // return type; (unreachable in C++)
    }
}

/// `std::stoi`-equivalent helper.
///
/// `std::stoi` parses a leading optional sign followed by digits, stops at the
/// first non-digit, throws `std::invalid_argument` when no conversion could be
/// performed, and `std::out_of_range` when the value does not fit in an `int`.
/// We model the "throw" path as `Err(())` so the callers above can mimic
/// `catch (...)`.
fn stoi(s: &str) -> Result<i32, ()> {
    let bytes = s.as_bytes();
    let mut idx = 0;
    // skip leading whitespace (std::stoi uses strtol which skips whitespace)
    while idx < bytes.len() && (bytes[idx] as char).is_whitespace() {
        idx += 1;
    }
    let start = idx;
    if idx < bytes.len() && (bytes[idx] == b'+' || bytes[idx] == b'-') {
        idx += 1;
    }
    let digits_start = idx;
    while idx < bytes.len() && bytes[idx].is_ascii_digit() {
        idx += 1;
    }
    if idx == digits_start {
        // no digits consumed -> std::invalid_argument
        return Err(());
    }
    // parse the matched [start, idx) substring
    match s[start..idx].parse::<i64>() {
        Ok(v) if v >= i32::MIN as i64 && v <= i32::MAX as i64 => Ok(v as i32),
        _ => Err(()), // out_of_range
    }
}

// ProjectTask.hpp:105
// class BBLSliceInfo
#[derive(Debug, Clone)]
pub struct BBLSliceInfo {
    pub filaments_info: Vec<FilamentInfo>, // ProjectTask.hpp:129

    pub index: String,          // plate index, start 1, 2, 3, etc.  ProjectTask.hpp:131
    pub title: String,          // ProjectTask.hpp:132
    pub thumbnail_dir: String,  // ProjectTask.hpp:133
    pub thumbnail_name: String, // ProjectTask.hpp:134
    pub thumbnail_url: String,  // ProjectTask.hpp:135
    pub gcode_name: String,     // ProjectTask.hpp:136
    pub gcode_url: String,      // ProjectTask.hpp:137
    pub gcode_dir: String,      // ProjectTask.hpp:138
    pub config_url: String,     // ProjectTask.hpp:139
    pub weight: f32,            // ProjectTask.hpp:140
    pub prediction: i32,        // ProjectTask.hpp:141
    // BBLProfile* profile_;   ProjectTask.hpp:142 (raw back-pointer; omitted)
}

impl BBLSliceInfo {
    // ProjectTask.hpp:107
    // BBLSliceInfo(BBLProfile* profile = nullptr)
    pub fn new() -> Self {
        BBLSliceInfo {
            // profile_ = profile;
            prediction: 0,
            weight: 0.0,
            filaments_info: Vec::new(),
            index: String::new(),
            title: String::new(),
            thumbnail_dir: String::new(),
            thumbnail_name: String::new(),
            thumbnail_url: String::new(),
            gcode_name: String::new(),
            gcode_url: String::new(),
            gcode_dir: String::new(),
            config_url: String::new(),
        }
    }
    // ProjectTask.hpp:114 copy-constructor: derive(Clone) provides the
    // equivalent field-wise copy.
}

impl Default for BBLSliceInfo {
    fn default() -> Self {
        Self::new()
    }
}

// ProjectTask.hpp:145
// enum TaskUserOptions
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TaskUserOptions {
    // ProjectTask.hpp:146
    OptionsBedLeveling = 0,
    // ProjectTask.hpp:147
    OptionsVibrationCali = 1,
    // ProjectTask.hpp:148
    OptionsFlowCali = 2,
    // ProjectTask.hpp:149
    OptionsLayerInspect = 3,
    // ProjectTask.hpp:150
    OptionsRecordTimelapse = 4,
}

// ProjectTask.hpp:153
// class BBLModelTask
#[derive(Debug, Clone)]
pub struct BBLModelTask {
    pub job_id: i32,           // ProjectTask.hpp:158
    pub design_id: i32,        // ProjectTask.hpp:159
    pub profile_id: i32,       // ProjectTask.hpp:160
    pub instance_id: i32,      // ProjectTask.hpp:161
    pub task_id: String,       // ProjectTask.hpp:162
    pub model_id: String,      // ProjectTask.hpp:163
    pub model_name: String,    // ProjectTask.hpp:164
    pub profile_name: String,  // ProjectTask.hpp:165
}

impl BBLModelTask {
    // ProjectTask.cpp:189
    // BBLModelTask::BBLModelTask()
    pub fn new() -> Self {
        BBLModelTask {
            job_id: -1,    // ProjectTask.cpp:191
            design_id: -1, // ProjectTask.cpp:192
            profile_id: -1, // ProjectTask.cpp:193
            // remaining members default-initialized
            instance_id: 0,
            task_id: String::new(),
            model_id: String::new(),
            model_name: String::new(),
            profile_name: String::new(),
        }
    }
}

impl Default for BBLModelTask {
    fn default() -> Self {
        Self::new()
    }
}

// ProjectTask.hpp:170
// enum BBLSubTask::SubTaskStatus
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SubTaskStatus {
    // ProjectTask.hpp:171
    TaskCreated = 0,
    // ProjectTask.hpp:172
    TaskReady = 1,
    // ProjectTask.hpp:173
    TaskRunning = 2,
    // ProjectTask.hpp:174
    TaskPause = 3,
    // ProjectTask.hpp:175
    TaskFailed = 4,
    // ProjectTask.hpp:176
    TaskFinished = 5,
    // ProjectTask.hpp:177
    TaskUnknown = 6,
}

// ProjectTask.hpp:168
// class BBLSubTask
#[derive(Debug, Clone)]
pub struct BBLSubTask {
    pub task_id: String,            /* plate id */          // ProjectTask.hpp:208
    pub task_model_id: String,      /* model id */          // ProjectTask.hpp:209
    pub task_project_id: String,    /* project id */        // ProjectTask.hpp:210
    pub task_profile_id: String,    /* profile id*/         // ProjectTask.hpp:211
    pub task_name: String,          /* task name, generally filename as task name */ // ProjectTask.hpp:212
    pub task_file: String,          /* local full file path of 3mf or gcode */       // ProjectTask.hpp:213
    pub task_path: std::path::PathBuf, /* local path of 3mf or gcode */              // ProjectTask.hpp:214
    pub task_gcode_in_3mf: String,  /* gcode in 3mf */      // ProjectTask.hpp:215
    pub task_create_time: String,   /* time created by cloud */ // ProjectTask.hpp:216
    pub task_thumbnail_url: String, /* url of task thumbnail */ // ProjectTask.hpp:217
    /* user options */
    pub task_bed_type: String,      /* bed_type of task, enum "auto" "pe", "pc", "pei" */ // ProjectTask.hpp:219
    pub task_bed_leveling: bool,    /* bed leveling of task */    // ProjectTask.hpp:220
    pub task_flow_cali: bool,       /* flow calibration of task */ // ProjectTask.hpp:221
    pub task_vibration_cali: bool,  /* vibration calibration of task */ // ProjectTask.hpp:222
    pub task_layer_inspect: bool,   /* first layer inspection of task */ // ProjectTask.hpp:223
    pub task_record_timelapse: bool, /* record timelapse of task */ // ProjectTask.hpp:224
    pub task_timelapse_use_internal: bool, /* use internal storage for timelapse, cfg bit[2] */ // ProjectTask.hpp:225

    // task of plate info
    pub task_weight: String,        /* weight create by slicer */ // ProjectTask.hpp:228
    pub task_weight_f: f32,         /* weight in task */          // ProjectTask.hpp:229
    pub slice_info: BBLSliceInfo,   /* slice info of subtask */   // ProjectTask.hpp:230
    pub task_partplate_idx: String, /* partplate_idx, start at 1, 2, etc. */ // ProjectTask.hpp:231

    pub task_status: SubTaskStatus, // ProjectTask.hpp:233
    pub task_printer_dev_id: String, /* dev_id of machine */ // ProjectTask.hpp:234
    pub task_progress: i32,         /* task running progress, update by machine */ // ProjectTask.hpp:235
    pub printing_status: String,    /* task status, update by machine */ // ProjectTask.hpp:236
    pub task_url: String,           /* post task to this url */ // ProjectTask.hpp:237
    pub task_url_md5: String,       /* md5 of task file */     // ProjectTask.hpp:238
    // BBLTask* parent_task_;  ProjectTask.hpp:239 (raw back-pointer; omitted)
    pub parent_id: String,          // ProjectTask.hpp:240

    pub job_id: i32,                // ProjectTask.hpp:242
    pub origin_model_name: String,  // ProjectTask.hpp:243
    pub origin_profile_name: String, // ProjectTask.hpp:244
}

impl BBLSubTask {
    // ProjectTask.cpp:56
    // BBLSubTask::BBLSubTask(BBLTask* task)
    pub fn new(task: Option<&BBLTask>) -> Self {
        let mut s = BBLSubTask {
            // members not explicitly set in the ctor are default-initialized;
            // the header in-class initializer task_layer_inspect{true} applies.
            task_id: String::new(),
            task_model_id: String::new(),
            task_project_id: String::new(),
            task_profile_id: String::new(),
            task_name: String::new(),
            task_file: String::new(),
            task_path: std::path::PathBuf::new(),
            task_gcode_in_3mf: String::new(),
            task_create_time: String::new(),
            task_thumbnail_url: String::new(),
            task_bed_type: String::new(),
            task_bed_leveling: false,
            task_flow_cali: false,
            task_vibration_cali: false,
            task_layer_inspect: true, // ProjectTask.hpp:223 in-class initializer {true}
            task_record_timelapse: false,
            task_timelapse_use_internal: false, // ProjectTask.hpp:225 in-class initializer { false }
            task_weight: String::new(),
            task_weight_f: 0.0,
            slice_info: BBLSliceInfo::new(),
            task_partplate_idx: String::new(),
            task_status: SubTaskStatus::TaskCreated,
            task_printer_dev_id: String::new(),
            task_progress: 0,
            printing_status: String::new(),
            task_url: String::new(),
            task_url_md5: String::new(),
            parent_id: String::new(),
            job_id: 0,
            origin_model_name: String::new(),
            origin_profile_name: String::new(),
        };
        // parent_task_ = task;
        if let Some(task) = task {
            // ProjectTask.cpp:60
            s.parent_id = task.task_id.clone();
            // ProjectTask.cpp:61
            s.task_project_id = task.task_project_id.clone();
            // ProjectTask.cpp:62
            s.task_profile_id = task.task_profile_id.clone();
        }
        // ProjectTask.cpp:64
        s.task_progress = 0;
        // ProjectTask.cpp:65
        s.task_record_timelapse = false;
        // ProjectTask.cpp:66
        s.task_bed_type = "auto".to_string();
        s
    }
    // ProjectTask.hpp:182 copy-constructor: derive(Clone) covers it.

    // ProjectTask.cpp:69
    // int BBLSubTask::parse_content_json(std::string json_str)
    pub fn parse_content_json(&mut self, json_str: &str) -> i32 {
        // try {
        let result = (|| -> Result<i32, ()> {
            // json j = json::parse(json_str);
            let j: Value = serde_json::from_str(json_str).map_err(|_| ())?;

            // if (j.contains("info") && !j["info"].is_null())
            if let Some(info) = j.get("info") {
                if !info.is_null() {
                    // if (j["info"].contains("name") && !j["info"]["name"].is_null())
                    if let Some(name) = info.get("name") {
                        if !name.is_null() {
                            // task_name = j["info"]["name"].get<std::string>();
                            self.task_name = name.as_str().ok_or(())?.to_string();
                        }
                    }
                    // if (j["info"].contains("plate_idx") && !j["info"]["plate_idx"].is_null())
                    if let Some(plate_idx) = info.get("plate_idx") {
                        if !plate_idx.is_null() {
                            // if (j["info"]["plate_idx"].is_number())
                            if plate_idx.is_number() {
                                // task_partplate_idx = std::to_string(j["info"]["plate_idx"].get<int>());
                                let v = plate_idx.as_i64().ok_or(())? as i32;
                                self.task_partplate_idx = v.to_string();
                            } else {
                                // task_partplate_idx = j["info"]["plate_idx"].get<std::string>();
                                self.task_partplate_idx = plate_idx.as_str().ok_or(())?.to_string();
                            }
                        }
                    }
                    // if (j["info"].contains("printer") && !j["info"]["printer"].is_null())
                    if let Some(printer) = info.get("printer") {
                        if !printer.is_null() {
                            // task_printer_dev_id = j["info"]["printer"].get<std::string>();
                            self.task_printer_dev_id = printer.as_str().ok_or(())?.to_string();
                        }
                    }
                    // return 0;
                    return Ok(0);
                }
            }
            // fall through (info absent / null): no early return, drop out of try block
            Err(())
        })();

        match result {
            Ok(code) => code,
            Err(()) => {
                // catch (...) { ... return -1; }  and the post-try fallthrough
                // ProjectTask.cpp:89 / ProjectTask.cpp:92
                log::trace!("parse_content_json failed! json={}", json_str);
                -1
            }
        }
    }

    // ProjectTask.cpp:96
    // BBLSubTask::SubTaskStatus BBLSubTask::parse_status(std::string status)
    pub fn parse_status(status: &str) -> SubTaskStatus {
        if status == "CREATED" {
            // ProjectTask.cpp:99
            SubTaskStatus::TaskCreated
        } else if status == "READY" {
            // ProjectTask.cpp:102
            SubTaskStatus::TaskReady
        } else if status == "RUNNING" {
            // ProjectTask.cpp:105
            SubTaskStatus::TaskRunning
        } else if status == "PAUSE" {
            // ProjectTask.cpp:108
            SubTaskStatus::TaskPause
        } else if status == "FAILED" {
            // ProjectTask.cpp:111
            SubTaskStatus::TaskFailed
        } else if status == "FINISHED" {
            // ProjectTask.cpp:114
            SubTaskStatus::TaskFinished
        } else {
            // ProjectTask.cpp:117
            SubTaskStatus::TaskCreated
        }
    }

    // ProjectTask.cpp:121
    // BBLSubTask::SubTaskStatus BBLSubTask::parse_user_service_task_status(int status)
    pub fn parse_user_service_task_status(status: i32) -> SubTaskStatus {
        if status == 1 {
            // ProjectTask.cpp:124
            SubTaskStatus::TaskRunning
        } else if status == 2 {
            // ProjectTask.cpp:126
            SubTaskStatus::TaskFinished
        } else if status == 3 {
            // ProjectTask.cpp:128
            SubTaskStatus::TaskFailed
        } else {
            // ProjectTask.cpp:129
            SubTaskStatus::TaskUnknown
        }
    }
}

// ProjectTask.hpp:253
// enum BBLTask::TaskStatus
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TaskStatus {
    // ProjectTask.hpp:256
    TaskActive = 0,
    // ProjectTask.hpp:257
    TaskInactive = 1,
}

/// typedef std::function<void(BBLModelTask* subtask)> OnGetSubTaskFn;
/// ProjectTask.hpp:251
pub type OnGetSubTaskFn<'a> = Box<dyn FnMut(&mut BBLModelTask) + 'a>;

// ProjectTask.hpp:253
// class BBLTask
#[derive(Debug, Clone)]
pub struct BBLTask {
    /* properties */
    pub task_id: String,            // ProjectTask.hpp:263
    pub task_name: String,          // ProjectTask.hpp:264
    pub task_create_time: String,   // ProjectTask.hpp:265
    pub task_status: TaskStatus,    // ProjectTask.hpp:266
    pub task_file: String,          /* local task file */ // ProjectTask.hpp:267 (std::wstring)
    pub task_url: String,           /* cloud task url */   // ProjectTask.hpp:268
    pub task_url_md5: String,       /* md5 of cloud task url file */ // ProjectTask.hpp:269
    pub task_dst_url: String,       /* put task to dest url in machine */ // ProjectTask.hpp:270 (std::wstring)
    // BBLProfile* profile_;  ProjectTask.hpp:271 (raw back-pointer; omitted)
    pub task_project_id: String,    // ProjectTask.hpp:272
    pub task_model_id: String,      // ProjectTask.hpp:273
    pub task_profile_id: String,    // ProjectTask.hpp:274
    pub subtasks: Vec<BBLSubTask>,  // ProjectTask.hpp:275 (std::vector<BBLSubTask*>)
    pub slice_info: std::collections::BTreeMap<String, BBLSliceInfo>, /* slice info of subtasks, key: plate idx, 1, 2, 3, etc... */ // ProjectTask.hpp:276
}

impl BBLTask {
    // ProjectTask.cpp:46
    // BBLTask::BBLTask(BBLProfile* profile)
    pub fn new(profile: Option<&BBLProfile>) -> Self {
        let mut t = BBLTask {
            task_id: String::new(),
            task_name: String::new(),
            task_create_time: String::new(),
            task_status: TaskStatus::TaskActive,
            task_file: String::new(),
            task_url: String::new(),
            task_url_md5: String::new(),
            task_dst_url: String::new(),
            task_project_id: String::new(),
            task_model_id: String::new(),
            task_profile_id: String::new(),
            subtasks: Vec::new(),
            slice_info: std::collections::BTreeMap::new(),
        };
        // profile_ = nullptr;
        if let Some(profile) = profile {
            // profile_ = profile;
            // ProjectTask.cpp:51
            t.task_profile_id = profile.profile_id.clone();
            // ProjectTask.cpp:52
            t.task_project_id = profile.project_id.clone();
        }
        t
    }

    // ProjectTask.hpp:278
    // std::string task_status_str()
    pub fn task_status_str(&self) -> String {
        if self.task_status == TaskStatus::TaskActive {
            "active".to_string()
        } else if self.task_status == TaskStatus::TaskInactive {
            "inactive".to_string()
        } else {
            "inactive".to_string()
        }
    }

    // ProjectTask.cpp:132
    // int BBLTask::parse_content_json(std::string json)
    pub fn parse_content_json(&mut self, json: &str) -> i32 {
        // try {
        let _ = (|| -> Result<(), ()> {
            // std::stringstream ss(json); pt::ptree root; pt::read_json(ss, root);
            let root: Value = serde_json::from_str(json).map_err(|_| ())?;

            // for (int i = 0; i < subtasks.size(); i++) delete subtasks[i];
            // subtasks.clear();
            self.subtasks.clear();

            // if (root.get_child_optional("subtasks") != boost::none)
            if let Some(subtask_list) = root.get("subtasks") {
                // pt::ptree subtask_list = root.get_child("subtasks");
                // for (auto subtask = subtask_list.begin(); subtask != subtask_list.end(); ++subtask)
                //
                // boost::property_tree iterates children in document order. For a
                // JSON array that is element order; for a JSON object it is the
                // member order. Mirror both shapes.
                let iter: Box<dyn Iterator<Item = &Value>> = if let Some(arr) = subtask_list.as_array() {
                    Box::new(arr.iter())
                } else if let Some(obj) = subtask_list.as_object() {
                    Box::new(obj.values())
                } else {
                    Box::new(std::iter::empty())
                };

                for subtask in iter {
                    // BBLSubTask* new_subtask = new BBLSubTask(this);
                    let mut new_subtask = BBLSubTask::new(Some(self));

                    /* create subtasks */
                    // boost::optional<std::string> subtask_id = subtask->second.get_optional<std::string>("id");
                    // if (subtask_id.has_value()) new_subtask->task_id = subtask_id.value();
                    if let Some(v) = get_optional_string(subtask, "id") {
                        new_subtask.task_id = v;
                    }

                    // subtask_name -> task_name
                    if let Some(v) = get_optional_string(subtask, "name") {
                        new_subtask.task_name = v;
                    }

                    // subtask_create_time -> task_create_time
                    if let Some(v) = get_optional_string(subtask, "create_time") {
                        new_subtask.task_create_time = v;
                    }

                    // subtask_plate_idx -> task_partplate_idx
                    if let Some(v) = get_optional_string(subtask, "plate_idx") {
                        new_subtask.task_partplate_idx = v;
                    }

                    // subtask_printer -> task_printer_dev_id
                    if let Some(v) = get_optional_string(subtask, "printer") {
                        new_subtask.task_printer_dev_id = v;
                    }

                    // subtask_weight -> task_weight
                    if let Some(v) = get_optional_string(subtask, "weight") {
                        new_subtask.task_weight = v;
                    }

                    // subtasks.push_back(new_subtask);
                    self.subtasks.push(new_subtask);
                }
            }
            Ok(())
        })()
        // catch (...) { BOOST_LOG_TRIVIAL(trace) << ...; }
        .map_err(|()| {
            // ProjectTask.cpp:170
            log::trace!("parse_content_json failed! json={}", json);
        });
        // return 0;  (ProjectTask.cpp:172) — always 0
        0
    }
}

/// boost `ptree::get_optional<std::string>` equivalent.
///
/// `boost::property_tree`'s JSON parser stores every scalar as a string and
/// `get_optional<std::string>` returns the value when the key exists and the
/// node holds a (translatable) scalar. We model that by accepting string and
/// numeric/boolean scalars and returning their textual form, and returning
/// `None` when the key is absent or maps to a non-scalar (array/object).
fn get_optional_string(node: &Value, key: &str) -> Option<String> {
    match node.get(key) {
        Some(Value::String(s)) => Some(s.clone()),
        Some(Value::Number(n)) => Some(n.to_string()),
        Some(Value::Bool(b)) => Some(b.to_string()),
        // null / array / object / missing -> no value
        _ => None,
    }
}

// ProjectTask.hpp:293
// class BBLProfile
#[derive(Debug, Clone)]
pub struct BBLProfile {
    pub tasks: Vec<BBLTask>,        // ProjectTask.hpp:298 (std::vector<BBLTask*>)
    pub profile_id: String,         // ProjectTask.hpp:299
    pub profile_name: String,       // ProjectTask.hpp:300
    pub profile_content: String,    // ProjectTask.hpp:301
    pub project_id: String,         /* parent project_id */ // ProjectTask.hpp:302
    pub model_id: String,           /* parent model_id */   // ProjectTask.hpp:303
    pub upload_url: String,         /* url for upload 3mf */ // ProjectTask.hpp:304
    pub upload_ticket: String,      /* ticket for notification */ // ProjectTask.hpp:305
    pub url: String,                /* 3mf url */            // ProjectTask.hpp:306
    pub md5: String,                /* 3mf md5 */            // ProjectTask.hpp:307
    pub filename: String,           /* 3mf filename */       // ProjectTask.hpp:308
    // BBLProject* project_;  ProjectTask.hpp:309 (raw back-pointer; omitted)
    pub slice_info: std::collections::BTreeMap<String, BBLSliceInfo>, /* key: plate_idx, start at 1, 2, 3, etc. */ // ProjectTask.hpp:310
}

impl BBLProfile {
    // ProjectTask.cpp:27
    // BBLProfile::BBLProfile(BBLProject* project)
    pub fn new(project: Option<&BBLProject>) -> Self {
        let mut p = BBLProfile {
            tasks: Vec::new(),
            profile_id: String::new(),
            // profile_name = "N/A";  ProjectTask.cpp:35
            profile_name: "N/A".to_string(),
            profile_content: String::new(),
            project_id: String::new(),
            model_id: String::new(),
            upload_url: String::new(),
            upload_ticket: String::new(),
            url: String::new(),
            md5: String::new(),
            filename: String::new(),
            slice_info: std::collections::BTreeMap::new(),
        };
        // project_ = nullptr;
        if let Some(project) = project {
            // project_ = project;
            // ProjectTask.cpp:32
            p.project_id = project.project_id.clone();
        }
        p
    }

    // ProjectTask.cpp:38
    // BBLSliceInfo* BBLProfile::get_slice_info(std::string plate_idx)
    pub fn get_slice_info(&self, plate_idx: &str) -> Option<&BBLSliceInfo> {
        // std::map<std::string, BBLSliceInfo*>::iterator it = slice_info.find(plate_idx);
        // if (it == slice_info.end()) return nullptr;
        // return it->second;
        self.slice_info.get(plate_idx)
    }
}

// ProjectTask.hpp:314
// class BBLProject
#[derive(Debug, Clone)]
pub struct BBLProject {
    pub project_id: String,         // ProjectTask.hpp:324
    pub project_model_id: String,   /* model id */    // ProjectTask.hpp:325
    pub project_design_id: String,  /* design_id */   // ProjectTask.hpp:326
    pub project_status: String,     // ProjectTask.hpp:327
    pub project_create_time: String, /* created by cloud */ // ProjectTask.hpp:328
    pub project_url: String,        /* url storage on cloud */ // ProjectTask.hpp:329
    pub project_url_md5: String,    /* md5 of project url file */ // ProjectTask.hpp:330
    pub project_name: String,       // ProjectTask.hpp:331
    pub project_3mf_file: String,   // ProjectTask.hpp:332
    pub project_path: std::path::PathBuf, // ProjectTask.hpp:333
    pub project_content: String,    // ProjectTask.hpp:334
    pub project_country_code: String, // ProjectTask.hpp:335

    pub profiles: Vec<BBLProfile>,  // ProjectTask.hpp:338 (std::vector<BBLProfile*>)
}

impl BBLProject {
    // ProjectTask.hpp:316
    // BBLProject() { project_name = "Untitled"; }
    pub fn new() -> Self {
        let mut p = BBLProject::empty();
        /* give a default project name */
        p.project_name = "Untitled".to_string();
        p
    }

    // ProjectTask.hpp:320
    // BBLProject(std::string name) { project_name = name; }
    pub fn with_name(name: String) -> Self {
        let mut p = BBLProject::empty();
        p.project_name = name;
        p
    }

    fn empty() -> Self {
        BBLProject {
            project_id: String::new(),
            project_model_id: String::new(),
            project_design_id: String::new(),
            project_status: String::new(),
            project_create_time: String::new(),
            project_url: String::new(),
            project_url_md5: String::new(),
            project_name: String::new(),
            project_3mf_file: String::new(),
            project_path: std::path::PathBuf::new(),
            project_content: String::new(),
            project_country_code: String::new(),
            profiles: Vec::new(),
        }
    }

    /* deprecated apis */
    // ProjectTask.hpp:341
    // void set_name(std::string name) { project_name = name; }
    pub fn set_name(&mut self, name: String) {
        self.project_name = name;
    }

    // ProjectTask.cpp:175
    // void BBLProject::reset()
    pub fn reset(&mut self) {
        self.project_model_id.clear();   // ProjectTask.cpp:177
        self.project_name.clear();       // ProjectTask.cpp:178
        self.project_id.clear();         // ProjectTask.cpp:179
        self.project_design_id.clear();  // ProjectTask.cpp:180
        self.project_status.clear();     // ProjectTask.cpp:181
        self.project_create_time.clear(); // ProjectTask.cpp:182
        self.project_url.clear();        // ProjectTask.cpp:183
        self.project_url_md5.clear();    // ProjectTask.cpp:184
        self.project_3mf_file.clear();   // ProjectTask.cpp:185
        // fs::path::clear() -> empty the path
        self.project_path = std::path::PathBuf::new(); // ProjectTask.cpp:186
    }
}

impl Default for BBLProject {
    fn default() -> Self {
        Self::new()
    }
}
