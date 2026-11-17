//! Faithful 1:1 port of `MultiNozzleUtils.cpp` / `MultiNozzleUtils.hpp`.
//!
//! C++ Reference:
//! - src/libslic3r/MultiNozzleUtils.hpp
//! - src/libslic3r/MultiNozzleUtils.cpp
//!
//! coord_t->i64, coordf_t->f64. `NozzleVolumeType` is reused from `crate::extruder`.
//! `FilamentInfo` is reused from `crate::project_task` (the .cpp includes ProjectTask.hpp).
//! `format_diameter_to_str` is reused from `crate::utils` (utils.cpp:1346).

// MultiNozzleUtils.cpp:1  #include "MultiNozzleUtils.hpp"
// MultiNozzleUtils.cpp:2  #include "ProjectTask.hpp"
// MultiNozzleUtils.cpp:3  #include "Utils.hpp"
// MultiNozzleUtils.cpp:4  #include "Print.hpp"
// MultiNozzleUtils.cpp:5  #include <chrono>
// MultiNozzleUtils.cpp:6  #include <unordered_map>
// MultiNozzleUtils.cpp:7  #include <unordered_set>
// MultiNozzleUtils.cpp:8  #include <boost/log/trivial.hpp>
use std::collections::{BTreeMap, BTreeSet};
use std::time::{Duration, Instant};

use crate::extruder::{NozzleVolumeType, NVT_MAX_NOZZLE_VOLUME_TYPE};
use crate::project_task::FilamentInfo;
use crate::utils::format_diameter_to_str_default as format_diameter_to_str;

// MultiNozzleUtils.cpp:10  namespace Slic3r { namespace MultiNozzleUtils {

// ==================== PrintConfig.cpp helpers reused for parity ====================
//
// `get_nozzle_volume_type_string` and `ConfigOptionEnum<NozzleVolumeType>::get_enum_values()`
// are defined in PrintConfig.cpp. They are reproduced here exactly (string<->enum maps)
// because the existing `crate::extruder` name table diverges from the C++ key-name table.

// PrintConfig.cpp:489  static const t_config_enum_values s_keys_map_NozzleVolumeType = {
//     { "Standard",  nvtStandard },
//     { "High Flow", nvtHighFlow },
//     { "TPU High Flow", nvtTPUHighFlow },
//     { "Hybrid", nvtHybrid}
// };
// ConfigOptionEnum<NozzleVolumeType>::get_enum_values() returns s_keys_map_NozzleVolumeType.
// (string -> enum value)
fn nozzle_volume_type_enum_values() -> BTreeMap<String, i32> {
    let mut m = BTreeMap::new();
    m.insert("Standard".to_string(), NozzleVolumeType::NvtStandard as i32);
    m.insert("High Flow".to_string(), NozzleVolumeType::NvtHighFlow as i32);
    m.insert(
        "TPU High Flow".to_string(),
        NozzleVolumeType::NvtTPUHighFlow as i32,
    );
    m.insert("Hybrid".to_string(), NozzleVolumeType::NvtHybrid as i32);
    m
}

// PrintConfig.cpp:119  static t_config_enum_names enum_names_from_keys_map(...)
//   inverts s_keys_map_NozzleVolumeType into names[enum_value] = key.
// Result: ["Standard", "High Flow", "Hybrid", "TPU High Flow"]
const S_KEYS_NAMES_NOZZLE_VOLUME_TYPE: [&str; 4] =
    ["Standard", "High Flow", "Hybrid", "TPU High Flow"];

// PrintConfig.cpp:563  std::string get_nozzle_volume_type_string(NozzleVolumeType nozzle_volume_type)
fn get_nozzle_volume_type_string(nozzle_volume_type: NozzleVolumeType) -> String {
    // PrintConfig.cpp:566  if (nozzle_volume_type > nvtMaxNozzleVolumeType) { ... return ""; }
    if (nozzle_volume_type as i32) > NVT_MAX_NOZZLE_VOLUME_TYPE {
        return String::new();
    }
    // PrintConfig.cpp:569  return s_keys_names_NozzleVolumeType[nozzle_volume_type];
    S_KEYS_NAMES_NOZZLE_VOLUME_TYPE[nozzle_volume_type as usize].to_string()
}

// MultiNozzleUtils.hpp:15  struct NozzleInfo
#[derive(Debug, Clone)]
pub struct NozzleInfo {
    pub diameter: String,                  // MultiNozzleUtils.hpp:17
    pub volume_type: NozzleVolumeType,     // MultiNozzleUtils.hpp:18
    pub extruder_id: i32,                  // MultiNozzleUtils.hpp:19  逻辑挤出机id  {-1}
    pub group_id: i32,                     // MultiNozzleUtils.hpp:20  对应逻辑喷嘴id {-1}
}

impl Default for NozzleInfo {
    // MultiNozzleUtils.hpp:15 (in-class member initializers)
    fn default() -> Self {
        NozzleInfo {
            diameter: String::new(),
            volume_type: NozzleVolumeType::NvtStandard,
            extruder_id: -1,
            group_id: -1,
        }
    }
}

impl NozzleInfo {
    // MultiNozzleUtils.hpp:24  bool operator<(const NozzleInfo& other) const
    fn lt(&self, other: &NozzleInfo) -> bool {
        // MultiNozzleUtils.hpp:25  if(group_id != other.group_id) return group_id < other.group_id;
        if self.group_id != other.group_id {
            return self.group_id < other.group_id;
        }
        // MultiNozzleUtils.hpp:26  if(extruder_id != other.extruder_id) return extruder_id < other.extruder_id;
        if self.extruder_id != other.extruder_id {
            return self.extruder_id < other.extruder_id;
        }
        // MultiNozzleUtils.hpp:27  if(volume_type != other.volume_type) return volume_type < other.volume_type;
        if self.volume_type != other.volume_type {
            return (self.volume_type as i32) < (other.volume_type as i32);
        }
        // MultiNozzleUtils.hpp:28  return diameter < other.diameter;
        self.diameter < other.diameter
    }

    // MultiNozzleUtils.cpp:882  std::string NozzleInfo::serialize() const
    pub fn serialize(&self) -> String {
        // MultiNozzleUtils.cpp:884-889
        format!(
            "id=\"{}\" extruder_id=\"{}\" nozzle_diameter=\"{}\" volume_type=\"{}\"",
            self.group_id,
            self.extruder_id + 1,
            self.diameter,
            get_nozzle_volume_type_string(self.volume_type)
        )
    }
}

// MultiNozzleUtils.hpp:34  struct NozzleGroupInfo
#[derive(Debug, Clone)]
pub struct NozzleGroupInfo {
    pub diameter: String,              // MultiNozzleUtils.hpp:36
    pub volume_type: NozzleVolumeType, // MultiNozzleUtils.hpp:37
    pub extruder_id: i32,              // MultiNozzleUtils.hpp:38
    pub nozzle_count: i32,             // MultiNozzleUtils.hpp:39
}

impl Default for NozzleGroupInfo {
    // MultiNozzleUtils.hpp:41  NozzleGroupInfo() = default;
    // (no in-class initializers; members are default-constructed: empty string, enum 0, ints 0)
    fn default() -> Self {
        NozzleGroupInfo {
            diameter: String::new(),
            volume_type: NozzleVolumeType::NvtStandard,
            extruder_id: 0,
            nozzle_count: 0,
        }
    }
}

impl NozzleGroupInfo {
    // MultiNozzleUtils.hpp:43  NozzleGroupInfo(diameter_, volume_type_, extruder_id_, nozzle_count_)
    pub fn new(
        nozzle_diameter_: String,
        volume_type_: NozzleVolumeType,
        extruder_id_: i32,
        nozzle_count_: i32,
    ) -> Self {
        NozzleGroupInfo {
            diameter: nozzle_diameter_,
            volume_type: volume_type_,
            extruder_id: extruder_id_,
            nozzle_count: nozzle_count_,
        }
    }

    // MultiNozzleUtils.hpp:47  inline bool operator<(const NozzleGroupInfo &rhs) const
    fn lt(&self, rhs: &NozzleGroupInfo) -> bool {
        // MultiNozzleUtils.hpp:49  if (extruder_id != rhs.extruder_id) return extruder_id < rhs.extruder_id;
        if self.extruder_id != rhs.extruder_id {
            return self.extruder_id < rhs.extruder_id;
        }
        // MultiNozzleUtils.hpp:50  if (diameter != rhs.diameter) return diameter < rhs.diameter;
        if self.diameter != rhs.diameter {
            return self.diameter < rhs.diameter;
        }
        // MultiNozzleUtils.hpp:51  if (volume_type != rhs.volume_type) return volume_type < rhs.volume_type;
        if self.volume_type != rhs.volume_type {
            return (self.volume_type as i32) < (rhs.volume_type as i32);
        }
        // MultiNozzleUtils.hpp:52  return nozzle_count < rhs.nozzle_count;
        self.nozzle_count < rhs.nozzle_count
    }

    // MultiNozzleUtils.hpp:55  bool is_same_type(const NozzleGroupInfo &rhs) const
    pub fn is_same_type(&self, rhs: &NozzleGroupInfo) -> bool {
        // MultiNozzleUtils.hpp:57
        self.diameter == rhs.diameter
            && self.volume_type == rhs.volume_type
            && self.extruder_id == rhs.extruder_id
    }

    // MultiNozzleUtils.hpp:60  inline bool operator==(const NozzleGroupInfo &rhs) const
    pub fn eq(&self, rhs: &NozzleGroupInfo) -> bool {
        // MultiNozzleUtils.hpp:62
        self.diameter == rhs.diameter
            && self.volume_type == rhs.volume_type
            && self.extruder_id == rhs.extruder_id
            && self.nozzle_count == rhs.nozzle_count
    }

    // MultiNozzleUtils.cpp:895  std::string NozzleGroupInfo::serialize() const
    pub fn serialize(&self) -> String {
        // MultiNozzleUtils.cpp:897-902
        //   oss << extruder_id << "-" << std::setprecision(2) << diameter << "-"
        //       << get_nozzle_volume_type_string(volume_type) << "-" << nozzle_count;
        // diameter is a std::string; setprecision(2) only affects floating-point output,
        // so it has no effect on the string `diameter`.
        format!(
            "{}-{}-{}-{}",
            self.extruder_id,
            self.diameter,
            get_nozzle_volume_type_string(self.volume_type),
            self.nozzle_count
        )
    }

    // MultiNozzleUtils.cpp:905  static std::optional<NozzleGroupInfo> deserialize(const std::string &str)
    pub fn deserialize(str: &str) -> Option<NozzleGroupInfo> {
        // MultiNozzleUtils.cpp:907-911
        //   std::istringstream iss(str); std::string token; std::vector<std::string> tokens;
        //   while (std::getline(iss, token, '-')) { tokens.push_back(token); }
        // std::getline(iss, token, '-') extracts each segment between '-' delimiters.
        // Internal empty segments ("a--b") yield empty tokens, but a trailing delimiter
        // ("a-b-") does NOT yield a final empty token: the last getline reads zero chars,
        // hits EOF, sets failbit and the `while(getline)` loop exits without pushing.
        // Empty input yields no tokens.
        let mut tokens: Vec<String> = Vec::new();
        let bytes = str.as_bytes();
        let mut start = 0usize;
        let mut i = 0usize;
        let mut consumed_delimiter = false;
        while i < bytes.len() {
            if bytes[i] == b'-' {
                tokens.push(str[start..i].to_string());
                start = i + 1;
                consumed_delimiter = true;
            }
            i += 1;
        }
        // Final segment: only emit if there is at least one character after the last
        // delimiter, or if no delimiter was ever consumed and the input is non-empty.
        if start < str.len() || (!consumed_delimiter && !str.is_empty()) {
            tokens.push(str[start..].to_string());
        }

        // MultiNozzleUtils.cpp:913  if (tokens.size() != 4) { return std::nullopt; }
        if tokens.len() != 4 {
            return None;
        }

        // MultiNozzleUtils.cpp:915  try {
        //   int extruder_id = std::stoi(tokens[0]);
        let extruder_id: i32 = match stoi(&tokens[0]) {
            Some(v) => v,
            None => return None, // MultiNozzleUtils.cpp:922  catch (const std::exception &) { return std::nullopt; }
        };
        // MultiNozzleUtils.cpp:917  std::string diameter = tokens[1];
        let diameter = tokens[1].clone();
        // MultiNozzleUtils.cpp:918  NozzleVolumeType volume_type = NozzleVolumeType(s_keys_map_NozzleVolumeType.at(tokens[2]));
        //   .at() throws if the key is absent -> caught -> std::nullopt.
        let enum_values = nozzle_volume_type_enum_values();
        let volume_type = match enum_values.get(&tokens[2]) {
            Some(&v) => NozzleVolumeType::from_i32(v),
            None => return None,
        };
        // MultiNozzleUtils.cpp:919  int nozzle_count = std::stoi(tokens[3]);
        let nozzle_count: i32 = match stoi(&tokens[3]) {
            Some(v) => v,
            None => return None,
        };

        // MultiNozzleUtils.cpp:921  return NozzleGroupInfo(diameter, volume_type, extruder_id, nozzle_count);
        Some(NozzleGroupInfo::new(
            diameter,
            volume_type,
            extruder_id,
            nozzle_count,
        ))
    }
}

// Faithful equivalent of std::stoi: parses a leading optional sign + decimal digits,
// ignoring leading whitespace and any trailing non-numeric characters. Returns None
// when no conversion could be performed (std::invalid_argument) — matching the
// try/catch behaviour at the call sites.
fn stoi(s: &str) -> Option<i32> {
    let bytes = s.as_bytes();
    let mut idx = 0usize;
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
        return None;
    }
    s[start..idx].parse::<i32>().ok()
}

// MultiNozzleUtils.hpp:69  struct FilamentChangeTimeParams
#[derive(Debug, Clone, Copy)]
pub struct FilamentChangeTimeParams {
    pub selector_load_time: f32,    // MultiNozzleUtils.hpp:71  {0.0f}
    pub selector_unload_time: f32,  // MultiNozzleUtils.hpp:72  {0.0f}
    pub standard_load_time: f32,    // MultiNozzleUtils.hpp:73  {0.0f}
    pub standard_unload_time: f32,  // MultiNozzleUtils.hpp:74  {0.0f}
}

impl Default for FilamentChangeTimeParams {
    fn default() -> Self {
        FilamentChangeTimeParams {
            selector_load_time: 0.0,
            selector_unload_time: 0.0,
            standard_load_time: 0.0,
            standard_unload_time: 0.0,
        }
    }
}

// MultiNozzleUtils.cpp:11  // ==================== 工具函数实现 ====================
// MultiNozzleUtils.cpp:12  std::vector<NozzleInfo> build_nozzle_list(std::vector<NozzleGroupInfo> nozzle_groups)
pub fn build_nozzle_list(mut nozzle_groups: Vec<NozzleGroupInfo>) -> Vec<NozzleInfo> {
    // MultiNozzleUtils.cpp:14  std::vector<NozzleInfo> ret;
    let mut ret: Vec<NozzleInfo> = Vec::new();
    // MultiNozzleUtils.cpp:15  std::sort(nozzle_groups.begin(), nozzle_groups.end());
    nozzle_groups.sort_by(|a, b| {
        if a.lt(b) {
            std::cmp::Ordering::Less
        } else if b.lt(a) {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Equal
        }
    });
    // MultiNozzleUtils.cpp:16  int nozzle_id = 0;
    let mut nozzle_id: i32 = 0;
    // MultiNozzleUtils.cpp:17  for (auto& group : nozzle_groups) {
    for group in &nozzle_groups {
        // MultiNozzleUtils.cpp:18  for (int i = 0; i < group.nozzle_count; ++i) {
        for _i in 0..group.nozzle_count {
            // MultiNozzleUtils.cpp:19  NozzleInfo tmp;
            let mut tmp = NozzleInfo::default();
            // MultiNozzleUtils.cpp:20  tmp.diameter = group.diameter;
            tmp.diameter = group.diameter.clone();
            // MultiNozzleUtils.cpp:21  tmp.extruder_id = group.extruder_id;
            tmp.extruder_id = group.extruder_id;
            // MultiNozzleUtils.cpp:22  tmp.volume_type = group.volume_type;
            tmp.volume_type = group.volume_type;
            // MultiNozzleUtils.cpp:23  tmp.group_id = nozzle_id++;
            tmp.group_id = nozzle_id;
            nozzle_id += 1;
            // MultiNozzleUtils.cpp:24  ret.emplace_back(std::move(tmp));
            ret.push(tmp);
        }
    }
    // MultiNozzleUtils.cpp:27  return ret;
    ret
}

// MultiNozzleUtils.cpp:30  std::vector<NozzleInfo> build_nozzle_list(double diameter,
//     const std::vector<int>& filament_nozzle_map, const std::vector<int>& filament_volume_map,
//     const std::vector<int>& filament_map)
pub fn build_nozzle_list_from_maps(
    diameter: f64,
    filament_nozzle_map: &[i32],
    filament_volume_map: &[i32],
    filament_map: &[i32],
) -> Vec<NozzleInfo> {
    // MultiNozzleUtils.cpp:32  std::string diameter_str = format_diameter_to_str(diameter);
    let diameter_str = format_diameter_to_str(diameter);
    // MultiNozzleUtils.cpp:33  std::map<int, std::vector<int>> nozzle_to_filaments;
    let mut nozzle_to_filaments: BTreeMap<i32, Vec<i32>> = BTreeMap::new();
    // MultiNozzleUtils.cpp:34  for(size_t idx = 0; idx < filament_nozzle_map.size(); ++idx){
    for idx in 0..filament_nozzle_map.len() {
        // MultiNozzleUtils.cpp:35  int nozzle_id = filament_nozzle_map[idx];
        let nozzle_id = filament_nozzle_map[idx];
        // MultiNozzleUtils.cpp:36  nozzle_to_filaments[nozzle_id].emplace_back(static_cast<int>(idx));
        nozzle_to_filaments
            .entry(nozzle_id)
            .or_insert_with(Vec::new)
            .push(idx as i32);
    }
    // MultiNozzleUtils.cpp:38  std::vector<NozzleInfo> ret;
    let mut ret: Vec<NozzleInfo> = Vec::new();
    // MultiNozzleUtils.cpp:39  for(auto& elem : nozzle_to_filaments){
    for (nozzle_id, filaments) in &nozzle_to_filaments {
        // MultiNozzleUtils.cpp:40  int nozzle_id = elem.first;
        // MultiNozzleUtils.cpp:41  auto& filaments = elem.second;
        // MultiNozzleUtils.cpp:42  NozzleInfo info;
        let mut info = NozzleInfo::default();
        // MultiNozzleUtils.cpp:43  info.diameter = diameter_str;
        info.diameter = diameter_str.clone();
        // MultiNozzleUtils.cpp:44  info.group_id = nozzle_id;
        info.group_id = *nozzle_id;
        // MultiNozzleUtils.cpp:45  info.extruder_id = filament_map[filaments.front()];
        info.extruder_id = filament_map[filaments[0] as usize];
        // MultiNozzleUtils.cpp:46  info.volume_type = NozzleVolumeType(filament_volume_map[filaments.front()]);
        info.volume_type = NozzleVolumeType::from_i32(filament_volume_map[filaments[0] as usize]);
        // MultiNozzleUtils.cpp:47  ret.emplace_back(std::move(info));
        ret.push(info);
    }
    // MultiNozzleUtils.cpp:49  return ret;
    ret
}

// MultiNozzleUtils.cpp:52  std::vector<NozzleInfo> load_nozzle_infos_with_compatibility(...)
pub fn load_nozzle_infos_with_compatibility(
    nozzle_infos: &[NozzleInfo],
    filament_infos: &[FilamentInfo],
    filament_map: &[i32],
    extruder_volume_types: &[NozzleVolumeType],
    nozzle_diameter: &[f64],
) -> Vec<NozzleInfo> {
    // MultiNozzleUtils.cpp:60  bool has_nozzle_info = !nozzle_infos.empty();
    let has_nozzle_info = !nozzle_infos.is_empty();
    // MultiNozzleUtils.cpp:61  bool has_valid_filament_info = !filament_infos.empty() &&
    //     std::all_of(..., [](const FilamentInfo& info){ return info.group_id.size() == 1; });
    let has_valid_filament_info = !filament_infos.is_empty()
        && filament_infos.iter().all(|info| info.group_id.len() == 1);

    // MultiNozzleUtils.cpp:65  if(!has_nozzle_info && !has_valid_filament_info){
    if !has_nozzle_info && !has_valid_filament_info {
        // MultiNozzleUtils.cpp:66  BOOST_LOG_TRIVIAL(warning) << ...: building nozzle list from filament map and volume types
        // (logging omitted)

        // MultiNozzleUtils.cpp:72  const size_t extruder_count = nozzle_diameter.size();
        let extruder_count = nozzle_diameter.len();

        // MultiNozzleUtils.cpp:74  std::vector<NozzleVolumeType> volume_types_fixed = extruder_volume_types;
        let mut volume_types_fixed: Vec<NozzleVolumeType> = extruder_volume_types.to_vec();
        // MultiNozzleUtils.cpp:75  volume_types_fixed.resize(extruder_count, NozzleVolumeType::nvtStandard);
        if volume_types_fixed.len() < extruder_count {
            volume_types_fixed.resize(extruder_count, NozzleVolumeType::NvtStandard);
        } else {
            volume_types_fixed.truncate(extruder_count);
        }

        // MultiNozzleUtils.cpp:77  std::vector<NozzleInfo> result;
        // MultiNozzleUtils.cpp:78  result.reserve(extruder_count);
        let mut result: Vec<NozzleInfo> = Vec::with_capacity(extruder_count);
        // MultiNozzleUtils.cpp:79  for (size_t extruder_id = 0; extruder_id < extruder_count; ++extruder_id) {
        for extruder_id in 0..extruder_count {
            // MultiNozzleUtils.cpp:80  NozzleInfo info;
            let mut info = NozzleInfo::default();
            // MultiNozzleUtils.cpp:81  info.diameter = format_diameter_to_str(nozzle_diameter[extruder_id]);
            info.diameter = format_diameter_to_str(nozzle_diameter[extruder_id]);
            // MultiNozzleUtils.cpp:82  info.group_id = static_cast<int>(extruder_id);
            info.group_id = extruder_id as i32;
            // MultiNozzleUtils.cpp:83  info.extruder_id = static_cast<int>(extruder_id);
            info.extruder_id = extruder_id as i32;
            // MultiNozzleUtils.cpp:84  info.volume_type = volume_types_fixed[extruder_id];
            info.volume_type = volume_types_fixed[extruder_id];
            // MultiNozzleUtils.cpp:85  result.emplace_back(std::move(info));
            result.push(info);
        }
        // MultiNozzleUtils.cpp:87  return result;
        return result;
    }

    // MultiNozzleUtils.cpp:90  if(!has_nozzle_info){
    if !has_nozzle_info {
        // MultiNozzleUtils.cpp:91  BOOST_LOG_TRIVIAL(info) << ...: building nozzle list from filament info  (logging omitted)
        // MultiNozzleUtils.cpp:92  std::map<int, NozzleInfo> nozzle_map; // group_id->NozzleInfo
        let mut nozzle_map: BTreeMap<i32, NozzleInfo> = BTreeMap::new();
        // MultiNozzleUtils.cpp:93  for(auto& filament : filament_infos){
        for filament in filament_infos {
            // MultiNozzleUtils.cpp:94  int group_id = filament.group_id.front();
            let group_id = filament.group_id[0];
            // MultiNozzleUtils.cpp:95  if(group_id < 0 || nozzle_map.find(group_id) != nozzle_map.end()){ continue; }
            if group_id < 0 || nozzle_map.contains_key(&group_id) {
                continue;
            }

            // MultiNozzleUtils.cpp:99  auto volume_type_str_to_enum = ConfigOptionEnum<NozzleVolumeType>::get_enum_values();
            let volume_type_str_to_enum = nozzle_volume_type_enum_values();

            // MultiNozzleUtils.cpp:101  NozzleInfo info;
            let mut info = NozzleInfo::default();
            // MultiNozzleUtils.cpp:102  info.diameter = format_diameter_to_str(filament.nozzle_diameter);
            info.diameter = format_diameter_to_str(filament.nozzle_diameter);
            // MultiNozzleUtils.cpp:103  info.group_id = group_id;
            info.group_id = group_id;
            // MultiNozzleUtils.cpp:104  info.extruder_id = filament_map[filament.id] -1; // 转成0-based;
            info.extruder_id = filament_map[filament.id as usize] - 1;

            // MultiNozzleUtils.cpp:106  if (volume_type_str_to_enum.count(filament.nozzle_volume_type))
            if let Some(&v) = volume_type_str_to_enum.get(&filament.nozzle_volume_type) {
                // MultiNozzleUtils.cpp:107  info.volume_type = NozzleVolumeType(volume_type_str_to_enum.at(filament.nozzle_volume_type));
                info.volume_type = NozzleVolumeType::from_i32(v);
            } else {
                // MultiNozzleUtils.cpp:109  info.volume_type = NozzleVolumeType::nvtStandard;
                info.volume_type = NozzleVolumeType::NvtStandard;
            }

            // MultiNozzleUtils.cpp:112  nozzle_map[group_id] = std::move(info);
            nozzle_map.insert(group_id, info);
        }

        // MultiNozzleUtils.cpp:115  std::vector<NozzleInfo> ret;
        let mut ret: Vec<NozzleInfo> = Vec::new();
        // MultiNozzleUtils.cpp:116  for(auto& elem : nozzle_map){ ret.emplace_back(elem.second); }
        for (_k, info) in &nozzle_map {
            ret.push(info.clone());
        }
        // MultiNozzleUtils.cpp:119  return ret;
        return ret;
    }

    // MultiNozzleUtils.cpp:123  auto result = nozzle_infos;
    let mut result: Vec<NozzleInfo> = nozzle_infos.to_vec();
    // MultiNozzleUtils.cpp:124  std::sort(result.begin(), result.end());
    result.sort_by(|a, b| {
        if a.lt(b) {
            std::cmp::Ordering::Less
        } else if b.lt(a) {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Equal
        }
    });
    // MultiNozzleUtils.cpp:125  BOOST_LOG_TRIVIAL(info) << ...: using new 3mf format with ... nozzle infos.  (logging omitted)
    // MultiNozzleUtils.cpp:126  return result;
    result
}

// MultiNozzleUtils.cpp:130  // ==================== LayeredNozzleGroupResult 实现 ====================
// MultiNozzleUtils.cpp:131  static bool has_filament_mapped_to_multiple_nozzles(...)
fn has_filament_mapped_to_multiple_nozzles(
    layer_filament_nozzle_maps: &[Vec<i32>],
    used_filaments: &[u32],
) -> bool {
    // MultiNozzleUtils.cpp:134  if (layer_filament_nozzle_maps.empty() || used_filaments.empty()) return false;
    if layer_filament_nozzle_maps.is_empty() || used_filaments.is_empty() {
        return false;
    }

    // MultiNozzleUtils.cpp:137  for (auto filament_id_u : used_filaments) {
    for &filament_id_u in used_filaments {
        // MultiNozzleUtils.cpp:138  int filament_id = static_cast<int>(filament_id_u);
        let filament_id = filament_id_u as i32;
        // MultiNozzleUtils.cpp:139  std::set<int> nozzle_ids;
        let mut nozzle_ids: BTreeSet<i32> = BTreeSet::new();

        // MultiNozzleUtils.cpp:141  for (size_t layer_id = 0; layer_id < layer_filament_nozzle_maps.size(); ++layer_id) {
        for layer_id in 0..layer_filament_nozzle_maps.len() {
            // MultiNozzleUtils.cpp:142  const auto &map = layer_filament_nozzle_maps[layer_id];
            let map = &layer_filament_nozzle_maps[layer_id];
            // MultiNozzleUtils.cpp:143  if (filament_id < 0 || filament_id >= static_cast<int>(map.size())) continue;
            if filament_id < 0 || filament_id >= map.len() as i32 {
                continue;
            }

            // MultiNozzleUtils.cpp:146  int nozzle_id = map[filament_id];
            let nozzle_id = map[filament_id as usize];
            // MultiNozzleUtils.cpp:147  if (nozzle_id < 0) continue;
            if nozzle_id < 0 {
                continue;
            }

            // MultiNozzleUtils.cpp:150  nozzle_ids.insert(nozzle_id);
            nozzle_ids.insert(nozzle_id);
            // MultiNozzleUtils.cpp:151  if (nozzle_ids.size() > 1) return true;
            if nozzle_ids.len() > 1 {
                return true;
            }
        }
    }

    // MultiNozzleUtils.cpp:156  return false;
    false
}

// MultiNozzleUtils.hpp:107  class LayeredNozzleGroupResult : public NozzleGroupResultBase
#[derive(Debug, Clone)]
pub struct LayeredNozzleGroupResult {
    // MultiNozzleUtils.hpp:83  bool support_dynamic_nozzle_map{false}; (base class member)
    support_dynamic_nozzle_map: bool,
    // MultiNozzleUtils.hpp:110  _layer_filament_nozzle_maps
    layer_filament_nozzle_maps: Vec<Vec<i32>>,
    // MultiNozzleUtils.hpp:111  _layer_filament_sequences
    layer_filament_sequences: Vec<Vec<u32>>,
    // MultiNozzleUtils.hpp:112  _default_filament_nozzle_map
    default_filament_nozzle_map: Vec<i32>,
    // MultiNozzleUtils.hpp:113  _used_filaments
    used_filaments: Vec<u32>,
    // MultiNozzleUtils.hpp:114  _nozzle_list
    nozzle_list: Vec<NozzleInfo>,
    // MultiNozzleUtils.hpp:115  bool handle_nozzle_as_extruder{ false };
    handle_nozzle_as_extruder: bool,
}

impl Default for LayeredNozzleGroupResult {
    // MultiNozzleUtils.hpp:117  LayeredNozzleGroupResult(bool support_dynamic_map = false), default argument
    // (`LayeredNozzleGroupResult()` of ToolOrdering.cpp).
    fn default() -> Self {
        LayeredNozzleGroupResult::with_support(false)
    }
}

impl LayeredNozzleGroupResult {
    // MultiNozzleUtils.hpp:117  LayeredNozzleGroupResult(bool support_dynamic_map = false)
    fn with_support(support_dynamic_map: bool) -> Self {
        LayeredNozzleGroupResult {
            support_dynamic_nozzle_map: support_dynamic_map,
            layer_filament_nozzle_maps: Vec::new(),
            layer_filament_sequences: Vec::new(),
            default_filament_nozzle_map: Vec::new(),
            used_filaments: Vec::new(),
            nozzle_list: Vec::new(),
            handle_nozzle_as_extruder: false,
        }
    }

    // MultiNozzleUtils.hpp:94  bool is_support_dynamic_nozzle_map() const
    pub fn is_support_dynamic_nozzle_map(&self) -> bool {
        self.support_dynamic_nozzle_map
    }

    // MultiNozzleUtils.hpp:124-128  create(filament_nozzle_map, nozzle_list, used_filaments,
    //   handle_nozzle_as_extruder = false): the overload with its last argument at the default. The
    //   overloads with every argument are the `create` of the traits below.
    pub fn create(
        filament_nozzle_map: &[i32],
        nozzle_list: &[NozzleInfo],
        used_filaments: &[u32],
    ) -> Option<LayeredNozzleGroupResult> {
        <LayeredNozzleGroupResult as CreateFromNozzleMap>::create(filament_nozzle_map, nozzle_list, used_filaments, false)
    }

    // MultiNozzleUtils.hpp:176  bool is_handle_nozzle_as_extruder() const { return handle_nozzle_as_extruder; }
    pub fn is_handle_nozzle_as_extruder(&self) -> bool {
        self.handle_nozzle_as_extruder
    }
}

/// `LayeredNozzleGroupResult::create(filament_nozzle_map, nozzle_list, used_filaments,
/// handle_nozzle_as_extruder)` (MultiNozzleUtils.hpp:124-128), the first of the three overloads:
/// call it as `<LayeredNozzleGroupResult as CreateFromNozzleMap>::create(...)`.
pub trait CreateFromNozzleMap {
    fn create(
        filament_nozzle_map: &[i32],
        nozzle_list: &[NozzleInfo],
        used_filaments: &[u32],
        handle_nozzle_as_extruder: bool,
    ) -> Option<LayeredNozzleGroupResult>;
}

/// `LayeredNozzleGroupResult::create(layer_filament_nozzle_maps, nozzle_list, used_filaments,
/// layer_filament_sequences, handle_nozzle_as_extruder)` (MultiNozzleUtils.hpp:130-135).
pub trait CreateFromLayerMaps {
    fn create(
        layer_filament_nozzle_maps: &[Vec<i32>],
        nozzle_list: &[NozzleInfo],
        used_filaments: &[u32],
        layer_filament_sequences: &[Vec<u32>],
        handle_nozzle_as_extruder: bool,
    ) -> Option<LayeredNozzleGroupResult>;
}

/// `LayeredNozzleGroupResult::create(used_filaments, filament_map, filament_volume_map,
/// filament_nozzle_map, nozzle_count, diameter, handle_nozzle_as_extruder)`
/// (MultiNozzleUtils.hpp:137-144).
pub trait CreateFromNozzleCount {
    #[allow(clippy::too_many_arguments)]
    fn create(
        used_filaments: &[u32],
        filament_map: &[i32],
        filament_volume_map: &[i32],
        filament_nozzle_map: &[i32],
        nozzle_count: &[BTreeMap<NozzleVolumeType, i32>],
        diameter: f32,
        handle_nozzle_as_extruder: bool,
    ) -> Option<LayeredNozzleGroupResult>;
}

impl CreateFromNozzleMap for LayeredNozzleGroupResult {
    // MultiNozzleUtils.cpp:310-327
    fn create(
        filament_nozzle_map: &[i32],
        nozzle_list: &[NozzleInfo],
        used_filaments: &[u32],
        handle_nozzle_as_extruder: bool,
    ) -> Option<LayeredNozzleGroupResult> {
        // MultiNozzleUtils.cpp:316-318
        if filament_nozzle_map.is_empty() || nozzle_list.is_empty() {
            return None;
        }

        // MultiNozzleUtils.cpp:320-324
        let mut result = LayeredNozzleGroupResult::with_support(false);
        result.default_filament_nozzle_map = filament_nozzle_map.to_vec();
        result.nozzle_list = nozzle_list.to_vec();
        result.used_filaments = used_filaments.to_vec();
        result.handle_nozzle_as_extruder = handle_nozzle_as_extruder;

        // MultiNozzleUtils.cpp:326
        Some(result)
    }
}

impl CreateFromLayerMaps for LayeredNozzleGroupResult {
    // MultiNozzleUtils.cpp:329-353
    fn create(
        layer_filament_nozzle_maps: &[Vec<i32>],
        nozzle_list: &[NozzleInfo],
        used_filaments: &[u32],
        layer_filament_sequences: &[Vec<u32>],
        handle_nozzle_as_extruder: bool,
    ) -> Option<LayeredNozzleGroupResult> {
        // MultiNozzleUtils.cpp:336-338
        if layer_filament_nozzle_maps.is_empty() || nozzle_list.is_empty() {
            return None;
        }

        // MultiNozzleUtils.cpp:340-346
        let support_dynamic_nozzle_map =
            has_filament_mapped_to_multiple_nozzles(layer_filament_nozzle_maps, used_filaments);
        let mut result = LayeredNozzleGroupResult::with_support(support_dynamic_nozzle_map);
        result.layer_filament_nozzle_maps = layer_filament_nozzle_maps.to_vec();
        result.layer_filament_sequences = layer_filament_sequences.to_vec();
        result.nozzle_list = nozzle_list.to_vec();
        result.used_filaments = used_filaments.to_vec();
        result.handle_nozzle_as_extruder = handle_nozzle_as_extruder;

        // MultiNozzleUtils.cpp:348-350
        if !layer_filament_nozzle_maps.is_empty() {
            result.default_filament_nozzle_map = layer_filament_nozzle_maps[0].clone();
        }

        // MultiNozzleUtils.cpp:352
        Some(result)
    }
}

impl CreateFromNozzleCount for LayeredNozzleGroupResult {
    // MultiNozzleUtils.cpp:355-409
    fn create(
        used_filaments: &[u32],
        filament_map: &[i32],
        filament_volume_map: &[i32],
        filament_nozzle_map: &[i32],
        nozzle_count: &[BTreeMap<NozzleVolumeType, i32>],
        diameter: f32,
        handle_nozzle_as_extruder: bool,
    ) -> Option<LayeredNozzleGroupResult> {
        // MultiNozzleUtils.cpp:208  std::vector<NozzleGroupInfo> nozzle_groups;
        let mut nozzle_groups: Vec<NozzleGroupInfo> = Vec::new();
        // MultiNozzleUtils.cpp:209  for (size_t extruder_id = 0; extruder_id < nozzle_count.size(); ++extruder_id) {
        for extruder_id in 0..nozzle_count.len() {
            // MultiNozzleUtils.cpp:210  for (auto elem : nozzle_count[extruder_id]) {
            for (volume_type, count) in &nozzle_count[extruder_id] {
                // MultiNozzleUtils.cpp:211  NozzleGroupInfo group_info;
                let mut group_info = NozzleGroupInfo::default();
                // MultiNozzleUtils.cpp:212  group_info.diameter = format_diameter_to_str(diameter);
                group_info.diameter = format_diameter_to_str(diameter as f64);
                // MultiNozzleUtils.cpp:213  group_info.volume_type = elem.first;
                group_info.volume_type = *volume_type;
                // MultiNozzleUtils.cpp:214  group_info.nozzle_count = elem.second;
                group_info.nozzle_count = *count;
                // MultiNozzleUtils.cpp:215  group_info.extruder_id = static_cast<int>(extruder_id);
                group_info.extruder_id = extruder_id as i32;
                // MultiNozzleUtils.cpp:216  nozzle_groups.emplace_back(group_info);
                nozzle_groups.push(group_info);
            }
        }

        // MultiNozzleUtils.cpp:220  auto nozzle_list = build_nozzle_list(nozzle_groups);
        let nozzle_list = build_nozzle_list(nozzle_groups);
        // MultiNozzleUtils.cpp:221  std::vector<bool> used_nozzle(nozzle_list.size(), false);
        let mut used_nozzle: Vec<bool> = vec![false; nozzle_list.len()];
        // MultiNozzleUtils.cpp:222  std::map<int, int> input_nozzle_id_to_output;
        let mut input_nozzle_id_to_output: BTreeMap<i32, i32> = BTreeMap::new();
        // MultiNozzleUtils.cpp:223  std::vector<int> output_nozzle_map(filament_nozzle_map.size(), 0);
        let mut output_nozzle_map: Vec<i32> = vec![0; filament_nozzle_map.len()];

        // MultiNozzleUtils.cpp:225  for (auto filament_idx : used_filaments) {
        for &filament_idx in used_filaments {
            let filament_idx = filament_idx as usize;
            // MultiNozzleUtils.cpp:226  NozzleVolumeType req_type = NozzleVolumeType(filament_volume_map[filament_idx]);
            let req_type = NozzleVolumeType::from_i32(filament_volume_map[filament_idx]);
            // MultiNozzleUtils.cpp:227  int req_extruder = filament_map[filament_idx];
            let req_extruder = filament_map[filament_idx];
            // MultiNozzleUtils.cpp:228  int input_nozzle_idx = filament_nozzle_map[filament_idx];
            let input_nozzle_idx = filament_nozzle_map[filament_idx];

            // MultiNozzleUtils.cpp:230  if (input_nozzle_id_to_output.find(input_nozzle_idx) != input_nozzle_id_to_output.end()) {
            if let Some(&out) = input_nozzle_id_to_output.get(&input_nozzle_idx) {
                // MultiNozzleUtils.cpp:231  output_nozzle_map[filament_idx] = input_nozzle_id_to_output[input_nozzle_idx];
                output_nozzle_map[filament_idx] = out;
                // MultiNozzleUtils.cpp:232  continue;
                continue;
            }

            // MultiNozzleUtils.cpp:235  int output_nozzle_idx = -1;
            let mut output_nozzle_idx: i32 = -1;
            // MultiNozzleUtils.cpp:236  for (size_t nozzle_idx = 0; nozzle_idx < nozzle_list.size(); ++nozzle_idx) {
            for nozzle_idx in 0..nozzle_list.len() {
                // MultiNozzleUtils.cpp:237  if (used_nozzle[nozzle_idx]) continue;
                if used_nozzle[nozzle_idx] {
                    continue;
                }

                // MultiNozzleUtils.cpp:239  auto &nozzle_info = nozzle_list[nozzle_idx];
                let nozzle_info = &nozzle_list[nozzle_idx];
                // MultiNozzleUtils.cpp:240  if (!(nozzle_info.extruder_id == req_extruder && nozzle_info.volume_type == req_type)) continue;
                if !(nozzle_info.extruder_id == req_extruder && nozzle_info.volume_type == req_type)
                {
                    continue;
                }

                // MultiNozzleUtils.cpp:242  output_nozzle_idx = static_cast<int>(nozzle_idx);
                output_nozzle_idx = nozzle_idx as i32;
                // MultiNozzleUtils.cpp:243  input_nozzle_id_to_output[input_nozzle_idx] = output_nozzle_idx;
                input_nozzle_id_to_output.insert(input_nozzle_idx, output_nozzle_idx);
                // MultiNozzleUtils.cpp:244  used_nozzle[nozzle_idx] = true;
                used_nozzle[nozzle_idx] = true;
                // MultiNozzleUtils.cpp:245  break;
                break;
            }

            // MultiNozzleUtils.cpp:248  if (output_nozzle_idx == -1) { return std::nullopt; }
            if output_nozzle_idx == -1 {
                return None;
            }
            // MultiNozzleUtils.cpp:249  output_nozzle_map[filament_idx] = output_nozzle_idx;
            output_nozzle_map[filament_idx] = output_nozzle_idx;
        }

        // MultiNozzleUtils.cpp:408  return create(output_nozzle_map, nozzle_list, used_filaments, handle_nozzle_as_extruder);
        <LayeredNozzleGroupResult as CreateFromNozzleMap>::create(&output_nozzle_map, &nozzle_list, used_filaments, handle_nozzle_as_extruder)
    }
}

impl LayeredNozzleGroupResult {
    // MultiNozzleUtils.cpp:255  bool LayeredNozzleGroupResult::are_filaments_same_extruder(filament_id1, filament_id2, layer_id) const
    pub fn are_filaments_same_extruder(
        &self,
        filament_id1: i32,
        filament_id2: i32,
        layer_id: i32,
    ) -> bool {
        // MultiNozzleUtils.cpp:257-258
        let nozzle_info1 = self.get_nozzle_for_filament(filament_id1, layer_id);
        let nozzle_info2 = self.get_nozzle_for_filament(filament_id2, layer_id);

        // MultiNozzleUtils.cpp:416  if (!nozzle_info1 || !nozzle_info2) return false;
        match (nozzle_info1, nozzle_info2) {
            (Some(n1), Some(n2)) => {
                // MultiNozzleUtils.cpp:418-419  if (handle_nozzle_as_extruder) return are_filaments_same_nozzle(...);
                if self.handle_nozzle_as_extruder {
                    return self.are_filaments_same_nozzle(filament_id1, filament_id2, layer_id);
                }
                // MultiNozzleUtils.cpp:421  return nozzle_info1->extruder_id == nozzle_info2->extruder_id;
                n1.extruder_id == n2.extruder_id
            }
            _ => false,
        }
    }

    // MultiNozzleUtils.cpp:265  bool LayeredNozzleGroupResult::are_filaments_same_nozzle(filament_id1, filament_id2, layer_id) const
    pub fn are_filaments_same_nozzle(
        &self,
        filament_id1: i32,
        filament_id2: i32,
        layer_id: i32,
    ) -> bool {
        // MultiNozzleUtils.cpp:267-268
        let nozzle_info1 = self.get_nozzle_for_filament(filament_id1, layer_id);
        let nozzle_info2 = self.get_nozzle_for_filament(filament_id2, layer_id);
        // MultiNozzleUtils.cpp:269  if (!nozzle_info1 || !nozzle_info2) return false;
        match (nozzle_info1, nozzle_info2) {
            (Some(n1), Some(n2)) => {
                // MultiNozzleUtils.cpp:271  return nozzle_info1->group_id == nozzle_info2->group_id;
                n1.group_id == n2.group_id
            }
            _ => false,
        }
    }

    // MultiNozzleUtils.cpp:274  int LayeredNozzleGroupResult::get_extruder_count() const
    pub fn get_extruder_count(&self) -> i32 {
        // MultiNozzleUtils.cpp:276  std::set<int> extruder_ids;
        let mut extruder_ids: BTreeSet<i32> = BTreeSet::new();
        // MultiNozzleUtils.cpp:277  for (const auto &nozzle : _nozzle_list) { extruder_ids.insert(nozzle.extruder_id); }
        for nozzle in &self.nozzle_list {
            extruder_ids.insert(nozzle.extruder_id);
        }
        // MultiNozzleUtils.cpp:278  return static_cast<int>(extruder_ids.size());
        extruder_ids.len() as i32
    }

    // MultiNozzleUtils.cpp:281  std::vector<NozzleInfo> LayeredNozzleGroupResult::get_used_nozzles_in_extruder(int target_extruder_id) const
    pub fn get_used_nozzles_in_extruder(&self, target_extruder_id: i32) -> Vec<NozzleInfo> {
        // MultiNozzleUtils.cpp:283  return get_used_nozzles_in_extruder(target_extruder_id, -1);
        self.get_used_nozzles_in_extruder_layer(target_extruder_id, -1)
    }

    // MultiNozzleUtils.cpp:286  std::vector<NozzleInfo> LayeredNozzleGroupResult::get_used_nozzles_in_extruder(int target_extruder_id, int layer_id) const
    pub fn get_used_nozzles_in_extruder_layer(
        &self,
        target_extruder_id: i32,
        layer_id: i32,
    ) -> Vec<NozzleInfo> {
        // MultiNozzleUtils.cpp:288  std::set<int> nozzle_ids;
        let mut nozzle_ids: BTreeSet<i32> = BTreeSet::new();
        // MultiNozzleUtils.cpp:289  std::vector<NozzleInfo> result;
        let mut result: Vec<NozzleInfo> = Vec::new();

        // MultiNozzleUtils.cpp:291  std::vector<unsigned int> target_filaments = get_used_filaments(layer_id);
        let target_filaments = self.get_used_filaments_layer(layer_id);

        // MultiNozzleUtils.cpp:293  for (unsigned int filament_id : target_filaments) {
        for filament_id in &target_filaments {
            let filament_id = *filament_id;
            // MultiNozzleUtils.cpp:294  if (layer_id != -1) {
            if layer_id != -1 {
                // MultiNozzleUtils.cpp:295  auto nozzle_opt = get_nozzle_for_filament(static_cast<int>(filament_id), layer_id);
                let nozzle_opt = self.get_nozzle_for_filament(filament_id as i32, layer_id);
                // MultiNozzleUtils.cpp:296  if (nozzle_opt) {
                if let Some(nozzle_opt) = nozzle_opt {
                    // MultiNozzleUtils.cpp:297  if (target_extruder_id == -1 || nozzle_opt->extruder_id == target_extruder_id) { nozzle_ids.insert(nozzle_opt->group_id); }
                    if target_extruder_id == -1 || nozzle_opt.extruder_id == target_extruder_id {
                        nozzle_ids.insert(nozzle_opt.group_id);
                    }
                }
            } else {
                // MultiNozzleUtils.cpp:300  auto nozzles = get_nozzles_for_filament(static_cast<int>(filament_id));
                let nozzles = self.get_nozzles_for_filament(filament_id as i32);
                // MultiNozzleUtils.cpp:301  for (const auto &nozzle : nozzles) {
                for nozzle in &nozzles {
                    // MultiNozzleUtils.cpp:302  if (target_extruder_id == -1 || nozzle.extruder_id == target_extruder_id) { nozzle_ids.insert(nozzle.group_id); }
                    if target_extruder_id == -1 || nozzle.extruder_id == target_extruder_id {
                        nozzle_ids.insert(nozzle.group_id);
                    }
                }
            }
        }
        // MultiNozzleUtils.cpp:306  for (int nozzle_id : nozzle_ids) {
        for &nozzle_id in &nozzle_ids {
            // MultiNozzleUtils.cpp:307  if (nozzle_id >= 0 && nozzle_id < static_cast<int>(_nozzle_list.size())) { result.push_back(_nozzle_list[nozzle_id]); }
            if nozzle_id >= 0 && nozzle_id < self.nozzle_list.len() as i32 {
                result.push(self.nozzle_list[nozzle_id as usize].clone());
            }
        }
        // MultiNozzleUtils.cpp:309  return result;
        result
    }

    // MultiNozzleUtils.cpp:312  std::vector<int> LayeredNozzleGroupResult::get_used_extruders() const
    pub fn get_used_extruders(&self) -> Vec<i32> {
        // MultiNozzleUtils.cpp:314  return get_used_extruders(-1);
        self.get_used_extruders_layer(-1)
    }

    // MultiNozzleUtils.cpp:317  std::vector<int> LayeredNozzleGroupResult::get_used_extruders(int layer_id) const
    pub fn get_used_extruders_layer(&self, layer_id: i32) -> Vec<i32> {
        // MultiNozzleUtils.cpp:319  std::set<int> used_extruders;
        let mut used_extruders: BTreeSet<i32> = BTreeSet::new();
        // MultiNozzleUtils.cpp:321  std::vector<unsigned int> target_filaments = get_used_filaments(layer_id);
        let target_filaments = self.get_used_filaments_layer(layer_id);
        // MultiNozzleUtils.cpp:322  for (auto filament_id : target_filaments) {
        for &filament_id in &target_filaments {
            // MultiNozzleUtils.cpp:323  if (layer_id != -1) {
            if layer_id != -1 {
                // 单层模式：获取该层特定耗材对应的喷嘴
                // MultiNozzleUtils.cpp:325  auto nozzle_opt = get_nozzle_for_filament(static_cast<int>(filament_id), layer_id);
                let nozzle_opt = self.get_nozzle_for_filament(filament_id as i32, layer_id);
                // MultiNozzleUtils.cpp:326  if (nozzle_opt) { used_extruders.insert(nozzle_opt->extruder_id); }
                if let Some(nozzle_opt) = nozzle_opt {
                    used_extruders.insert(nozzle_opt.extruder_id);
                }
            } else {
                // 全局模式：获取该耗材在所有层使用的所有喷嘴
                // MultiNozzleUtils.cpp:329  auto nozzles = get_nozzles_for_filament(static_cast<int>(filament_id));
                let nozzles = self.get_nozzles_for_filament(filament_id as i32);
                // MultiNozzleUtils.cpp:330  for (const auto &nozzle : nozzles) { used_extruders.insert(nozzle.extruder_id); }
                for nozzle in &nozzles {
                    used_extruders.insert(nozzle.extruder_id);
                }
            }
        }
        // MultiNozzleUtils.cpp:333  return std::vector<int>(used_extruders.begin(), used_extruders.end());
        used_extruders.into_iter().collect()
    }

    // MultiNozzleUtils.cpp:336  std::vector<int> LayeredNozzleGroupResult::get_extruder_map(bool zero_based, int layer_id) const
    pub fn get_extruder_map(&self, zero_based: bool, layer_id: i32) -> Vec<i32> {
        // MultiNozzleUtils.cpp:338  const std::vector<int> &filament_nozzle_map = get_layer_filament_nozzle_map(layer_id);
        let filament_nozzle_map = self.get_layer_filament_nozzle_map(layer_id);
        // MultiNozzleUtils.cpp:339  std::vector<int> extruder_map(filament_nozzle_map.size());
        let mut extruder_map: Vec<i32> = vec![0; filament_nozzle_map.len()];
        // MultiNozzleUtils.cpp:340  for (size_t idx = 0; idx < filament_nozzle_map.size(); ++idx) {
        for idx in 0..filament_nozzle_map.len() {
            // MultiNozzleUtils.cpp:341  int nozzle_id = filament_nozzle_map[idx];
            let nozzle_id = filament_nozzle_map[idx];
            // MultiNozzleUtils.cpp:342  if (nozzle_id >= 0 && nozzle_id < static_cast<int>(_nozzle_list.size())) {
            if nozzle_id >= 0 && nozzle_id < self.nozzle_list.len() as i32 {
                // MultiNozzleUtils.cpp:343  extruder_map[idx] = _nozzle_list[nozzle_id].extruder_id;
                extruder_map[idx] = self.nozzle_list[nozzle_id as usize].extruder_id;
            } else {
                // MultiNozzleUtils.cpp:345  extruder_map[idx] = -1;
                extruder_map[idx] = -1;
            }
        }

        // MultiNozzleUtils.cpp:349  if (zero_based) return extruder_map;
        if zero_based {
            return extruder_map;
        }

        // MultiNozzleUtils.cpp:351  auto new_filament_map = extruder_map;
        let mut new_filament_map = extruder_map;
        // MultiNozzleUtils.cpp:352  std::transform(..., [](int val) { return val + 1; });
        for val in new_filament_map.iter_mut() {
            *val += 1;
        }
        // MultiNozzleUtils.cpp:353  return new_filament_map;
        new_filament_map
    }

    // MultiNozzleUtils.cpp:356  std::vector<int> LayeredNozzleGroupResult::get_nozzle_map(int layer_id) const
    pub fn get_nozzle_map(&self, layer_id: i32) -> Vec<i32> {
        // MultiNozzleUtils.cpp:358  const std::vector<int> &filament_nozzle_map = get_layer_filament_nozzle_map(layer_id);
        let filament_nozzle_map = self.get_layer_filament_nozzle_map(layer_id);
        // MultiNozzleUtils.cpp:359  std::vector<int> nozzle_map(filament_nozzle_map.size());
        let mut nozzle_map: Vec<i32> = vec![0; filament_nozzle_map.len()];
        // MultiNozzleUtils.cpp:360  for (size_t idx = 0; idx < filament_nozzle_map.size(); ++idx) {
        for idx in 0..filament_nozzle_map.len() {
            // MultiNozzleUtils.cpp:361  int nozzle_id = filament_nozzle_map[idx];
            let nozzle_id = filament_nozzle_map[idx];
            // MultiNozzleUtils.cpp:362  if (nozzle_id >= 0 && nozzle_id < static_cast<int>(_nozzle_list.size())) {
            if nozzle_id >= 0 && nozzle_id < self.nozzle_list.len() as i32 {
                // MultiNozzleUtils.cpp:363  nozzle_map[idx] = _nozzle_list[nozzle_id].group_id;
                nozzle_map[idx] = self.nozzle_list[nozzle_id as usize].group_id;
            } else {
                // MultiNozzleUtils.cpp:365  nozzle_map[idx] = -1;
                nozzle_map[idx] = -1;
            }
        }
        // MultiNozzleUtils.cpp:368  return nozzle_map;
        nozzle_map
    }

    // MultiNozzleUtils.cpp:371  std::vector<int> LayeredNozzleGroupResult::get_volume_map(int layer_id) const
    pub fn get_volume_map(&self, layer_id: i32) -> Vec<i32> {
        // MultiNozzleUtils.cpp:373  const std::vector<int> &filament_nozzle_map = get_layer_filament_nozzle_map(layer_id);
        let filament_nozzle_map = self.get_layer_filament_nozzle_map(layer_id);
        // MultiNozzleUtils.cpp:374  std::vector<int> volume_map(filament_nozzle_map.size());
        let mut volume_map: Vec<i32> = vec![0; filament_nozzle_map.len()];
        // MultiNozzleUtils.cpp:375  for (size_t idx = 0; idx < filament_nozzle_map.size(); ++idx) {
        for idx in 0..filament_nozzle_map.len() {
            // MultiNozzleUtils.cpp:376  int nozzle_id = filament_nozzle_map[idx];
            let nozzle_id = filament_nozzle_map[idx];
            // MultiNozzleUtils.cpp:377  if (nozzle_id >= 0 && nozzle_id < static_cast<int>(_nozzle_list.size())) {
            if nozzle_id >= 0 && nozzle_id < self.nozzle_list.len() as i32 {
                // MultiNozzleUtils.cpp:378  volume_map[idx] = _nozzle_list[nozzle_id].volume_type; (enum->int)
                volume_map[idx] = self.nozzle_list[nozzle_id as usize].volume_type as i32;
            } else {
                // MultiNozzleUtils.cpp:380  volume_map[idx] = -1;
                volume_map[idx] = -1;
            }
        }
        // MultiNozzleUtils.cpp:383  return volume_map;
        volume_map
    }

    // MultiNozzleUtils.hpp:154  std::vector<unsigned int> get_used_filaments() const override { return _used_filaments; }
    pub fn get_used_filaments(&self) -> Vec<u32> {
        self.used_filaments.clone()
    }

    // MultiNozzleUtils.cpp:386  std::vector<unsigned int> LayeredNozzleGroupResult::get_used_filaments(int layer_id) const
    pub fn get_used_filaments_layer(&self, layer_id: i32) -> Vec<u32> {
        // MultiNozzleUtils.cpp:388  if (layer_id < 0) { return _used_filaments; }
        if layer_id < 0 {
            return self.used_filaments.clone();
        }
        // MultiNozzleUtils.cpp:389  if (layer_id >= static_cast<int>(_layer_filament_nozzle_maps.size())) { return _used_filaments; }
        if layer_id >= self.layer_filament_nozzle_maps.len() as i32 {
            return self.used_filaments.clone();
        }

        // MultiNozzleUtils.cpp:391  if (!_layer_filament_sequences.empty() && layer_id < static_cast<int>(_layer_filament_sequences.size())) {
        if !self.layer_filament_sequences.is_empty()
            && layer_id < self.layer_filament_sequences.len() as i32
        {
            // MultiNozzleUtils.cpp:392  return _layer_filament_sequences[layer_id];
            return self.layer_filament_sequences[layer_id as usize].clone();
        }
        // MultiNozzleUtils.cpp:394  return {};
        Vec::new()
    }

    // MultiNozzleUtils.cpp:397  std::optional<NozzleInfo> LayeredNozzleGroupResult::get_nozzle_for_filament(int filament_id, int layer_id) const
    pub fn get_nozzle_for_filament(&self, filament_id: i32, layer_id: i32) -> Option<NozzleInfo> {
        // MultiNozzleUtils.cpp:399  const std::vector<int> &filament_nozzle_map = get_layer_filament_nozzle_map(layer_id);
        let filament_nozzle_map = self.get_layer_filament_nozzle_map(layer_id);

        // MultiNozzleUtils.cpp:401  if (filament_id < 0 || filament_id >= static_cast<int>(filament_nozzle_map.size())) { return std::nullopt; }
        if filament_id < 0 || filament_id >= filament_nozzle_map.len() as i32 {
            return None;
        }

        // MultiNozzleUtils.cpp:403  int nozzle_id = filament_nozzle_map[filament_id];
        let nozzle_id = filament_nozzle_map[filament_id as usize];
        // MultiNozzleUtils.cpp:404  return get_nozzle_from_id(nozzle_id);
        self.get_nozzle_from_id(nozzle_id)
    }

    // MultiNozzleUtils.cpp:407  std::vector<NozzleInfo> LayeredNozzleGroupResult::get_nozzles_for_filament(int filament_id) const
    pub fn get_nozzles_for_filament(&self, filament_id: i32) -> Vec<NozzleInfo> {
        // MultiNozzleUtils.cpp:409  std::set<int> nozzle_ids;
        let mut nozzle_ids: BTreeSet<i32> = BTreeSet::new();

        // MultiNozzleUtils.cpp:411  if (!support_dynamic_nozzle_map) {
        if !self.support_dynamic_nozzle_map {
            // MultiNozzleUtils.cpp:412  if (filament_id >= 0 && filament_id < static_cast<int>(_default_filament_nozzle_map.size())) {
            if filament_id >= 0 && filament_id < self.default_filament_nozzle_map.len() as i32 {
                // MultiNozzleUtils.cpp:413  nozzle_ids.insert(_default_filament_nozzle_map[filament_id]);
                nozzle_ids.insert(self.default_filament_nozzle_map[filament_id as usize]);
            }
        } else {
            // MultiNozzleUtils.cpp:416  int start_layer = 0;
            let start_layer: i32 = 0;
            // MultiNozzleUtils.cpp:417  int end_layer = static_cast<int>(_layer_filament_nozzle_maps.size());
            let end_layer: i32 = self.layer_filament_nozzle_maps.len() as i32;

            // MultiNozzleUtils.cpp:419  for (int i = start_layer; i < end_layer; ++i) {
            for i in start_layer..end_layer {
                // MultiNozzleUtils.cpp:420  const auto &map = _layer_filament_nozzle_maps[i];
                let map = &self.layer_filament_nozzle_maps[i as usize];
                // MultiNozzleUtils.cpp:421  if (filament_id >= 0 && filament_id < static_cast<int>(map.size())) {
                if filament_id >= 0 && filament_id < map.len() as i32 {
                    // MultiNozzleUtils.cpp:422  nozzle_ids.insert(map[filament_id]);
                    nozzle_ids.insert(map[filament_id as usize]);
                }
            }
        }

        // MultiNozzleUtils.cpp:427  std::vector<NozzleInfo> result;
        let mut result: Vec<NozzleInfo> = Vec::new();
        // MultiNozzleUtils.cpp:428  for (int id : nozzle_ids) {
        for &id in &nozzle_ids {
            // MultiNozzleUtils.cpp:429  if (id >= 0 && id < static_cast<int>(_nozzle_list.size())) { result.push_back(_nozzle_list[id]); }
            if id >= 0 && id < self.nozzle_list.len() as i32 {
                result.push(self.nozzle_list[id as usize].clone());
            }
        }
        // MultiNozzleUtils.cpp:431  return result;
        result
    }

    // MultiNozzleUtils.cpp:434  std::optional<NozzleInfo> LayeredNozzleGroupResult::get_first_nozzle_for_filament(int filament_id) const
    pub fn get_first_nozzle_for_filament(&self, filament_id: i32) -> Option<NozzleInfo> {
        // MultiNozzleUtils.cpp:436  if (filament_id < 0) return std::nullopt;
        if filament_id < 0 {
            return None;
        }

        // MultiNozzleUtils.cpp:438  if (!support_dynamic_nozzle_map) {
        if !self.support_dynamic_nozzle_map {
            // MultiNozzleUtils.cpp:439  if (filament_id >= static_cast<int>(_default_filament_nozzle_map.size())) return std::nullopt;
            if filament_id >= self.default_filament_nozzle_map.len() as i32 {
                return None;
            }
            // MultiNozzleUtils.cpp:440  return get_nozzle_from_id(_default_filament_nozzle_map[filament_id]);
            return self.get_nozzle_from_id(self.default_filament_nozzle_map[filament_id as usize]);
        }

        // MultiNozzleUtils.cpp:443  for (size_t layer = 0; layer < _layer_filament_nozzle_maps.size(); ++layer) {
        for layer in 0..self.layer_filament_nozzle_maps.len() {
            // MultiNozzleUtils.cpp:444  auto layer_used_filaments = get_used_filaments(layer);
            let layer_used_filaments = self.get_used_filaments_layer(layer as i32);
            // MultiNozzleUtils.cpp:445  if (std::find(..., static_cast<unsigned int>(filament_id)) == layer_used_filaments.end()){ continue; }
            if !layer_used_filaments.contains(&(filament_id as u32)) {
                continue;
            }
            // MultiNozzleUtils.cpp:448  const auto &map = _layer_filament_nozzle_maps[layer];
            let map = &self.layer_filament_nozzle_maps[layer];
            // MultiNozzleUtils.cpp:449  if (filament_id >= 0 && filament_id < static_cast<int>(map.size())) {
            if filament_id >= 0 && filament_id < map.len() as i32 {
                // MultiNozzleUtils.cpp:450  int nozzle_id = map[filament_id];
                let nozzle_id = map[filament_id as usize];
                // MultiNozzleUtils.cpp:451  auto nozzle = get_nozzle_from_id(nozzle_id);
                let nozzle = self.get_nozzle_from_id(nozzle_id);
                // MultiNozzleUtils.cpp:452  if (nozzle) return nozzle;
                if nozzle.is_some() {
                    return nozzle;
                }
            }
        }

        // MultiNozzleUtils.cpp:456  return std::nullopt;
        None
    }

    // MultiNozzleUtils.cpp:459  std::optional<NozzleInfo> LayeredNozzleGroupResult::get_nozzle_from_id(int nozzle_id) const
    pub fn get_nozzle_from_id(&self, nozzle_id: i32) -> Option<NozzleInfo> {
        // MultiNozzleUtils.cpp:461  if (nozzle_id < 0 || nozzle_id >= static_cast<int>(_nozzle_list.size())) { return std::nullopt; }
        if nozzle_id < 0 || nozzle_id >= self.nozzle_list.len() as i32 {
            return None;
        }
        // MultiNozzleUtils.cpp:462  return _nozzle_list[nozzle_id];
        Some(self.nozzle_list[nozzle_id as usize].clone())
    }

    // MultiNozzleUtils.cpp:465  int LayeredNozzleGroupResult::get_extruder_id(int filament_id, int layer_id) const
    pub fn get_extruder_id(&self, filament_id: i32, layer_id: i32) -> i32 {
        // MultiNozzleUtils.cpp:467  auto nozzle_info = get_nozzle_for_filament(filament_id, layer_id);
        let nozzle_info = self.get_nozzle_for_filament(filament_id, layer_id);
        // MultiNozzleUtils.cpp:468  return nozzle_info ? nozzle_info->extruder_id : -1;
        match nozzle_info {
            Some(n) => n.extruder_id,
            None => -1,
        }
    }

    // MultiNozzleUtils.cpp:471  int LayeredNozzleGroupResult::get_nozzle_id(int filament_id, int layer_id) const
    pub fn get_nozzle_id(&self, filament_id: i32, layer_id: i32) -> i32 {
        // MultiNozzleUtils.cpp:473  auto nozzle_info = get_nozzle_for_filament(filament_id, layer_id);
        let nozzle_info = self.get_nozzle_for_filament(filament_id, layer_id);
        // MultiNozzleUtils.cpp:474  return nozzle_info ? nozzle_info->group_id : -1;
        match nozzle_info {
            Some(n) => n.group_id,
            None => -1,
        }
    }

    // MultiNozzleUtils.hpp:165  size_t get_layer_count() const { return _layer_filament_nozzle_maps.size(); }
    pub fn get_layer_count(&self) -> usize {
        self.layer_filament_nozzle_maps.len()
    }

    // MultiNozzleUtils.cpp:477  const std::vector<int> &LayeredNozzleGroupResult::get_layer_filament_nozzle_map(int layer_id) const
    pub fn get_layer_filament_nozzle_map(&self, layer_id: i32) -> &Vec<i32> {
        // MultiNozzleUtils.cpp:479  if (layer_id >= 0 && layer_id < static_cast<int>(_layer_filament_nozzle_maps.size())) { return _layer_filament_nozzle_maps[layer_id]; }
        if layer_id >= 0 && layer_id < self.layer_filament_nozzle_maps.len() as i32 {
            return &self.layer_filament_nozzle_maps[layer_id as usize];
        }
        // MultiNozzleUtils.cpp:480  return _default_filament_nozzle_map;
        &self.default_filament_nozzle_map
    }

    // MultiNozzleUtils.hpp:167  const std::vector<std::vector<int>> &get_layer_filament_nozzle_maps() const
    pub fn get_layer_filament_nozzle_maps(&self) -> &Vec<Vec<i32>> {
        &self.layer_filament_nozzle_maps
    }

    // MultiNozzleUtils.hpp:168  const std::vector<std::vector<unsigned int>>& get_layer_filament_sequences() const
    pub fn get_layer_filament_sequences(&self) -> &Vec<Vec<u32>> {
        &self.layer_filament_sequences
    }

    // MultiNozzleUtils.cpp:483  int LayeredNozzleGroupResult::estimate_seq_flush_weight(flush_matrix, filament_change_seq) const
    pub fn estimate_seq_flush_weight(
        &self,
        flush_matrix: &[Vec<Vec<f32>>],
        filament_change_seq: &[i32],
    ) -> i32 {
        // MultiNozzleUtils.cpp:485  auto get_weight_from_volume = [](float volume){ return static_cast<int>(volume * 1.26 * 0.01); };
        let get_weight_from_volume = |volume: f32| -> i32 {
            (volume as f64 * 1.26 * 0.01) as i32
        };

        // MultiNozzleUtils.cpp:489  float total_flush_volume = 0;
        let mut total_flush_volume: f32 = 0.0;
        // MultiNozzleUtils.cpp:490  MultiNozzleUtils::NozzleStatusRecorder recorder;
        let mut recorder = NozzleStatusRecorder::new();
        // MultiNozzleUtils.cpp:491  for(auto filament: filament_change_seq){
        for &filament in filament_change_seq {
            // MultiNozzleUtils.cpp:492  auto nozzle = get_nozzle_for_filament(filament, -1);
            let nozzle = self.get_nozzle_for_filament(filament, -1);
            // MultiNozzleUtils.cpp:493  if(!nozzle) continue;
            let nozzle = match nozzle {
                Some(n) => n,
                None => continue,
            };

            // MultiNozzleUtils.cpp:496  int extruder_id = nozzle->extruder_id;
            let extruder_id = nozzle.extruder_id;
            // MultiNozzleUtils.cpp:497  int nozzle_id = nozzle->group_id;
            let nozzle_id = nozzle.group_id;
            // MultiNozzleUtils.cpp:498  int last_filament = recorder.get_filament_in_nozzle(nozzle_id);
            let last_filament = recorder.get_filament_in_nozzle(nozzle_id);

            // MultiNozzleUtils.cpp:500  if(last_filament!= -1 && last_filament != filament){
            if last_filament != -1 && last_filament != filament {
                // MultiNozzleUtils.cpp:502  边界检查，避免越界访问
                if extruder_id >= 0
                    && extruder_id < flush_matrix.len() as i32
                    && last_filament >= 0
                    && last_filament < flush_matrix[extruder_id as usize].len() as i32
                    && filament >= 0
                    && filament
                        < flush_matrix[extruder_id as usize][last_filament as usize].len() as i32
                {
                    // MultiNozzleUtils.cpp:505  float flush_volume = flush_matrix[extruder_id][last_filament][filament];
                    let flush_volume =
                        flush_matrix[extruder_id as usize][last_filament as usize][filament as usize];
                    // MultiNozzleUtils.cpp:506  total_flush_volume += flush_volume;
                    total_flush_volume += flush_volume;
                }
            }
            // MultiNozzleUtils.cpp:509  recorder.set_nozzle_status(nozzle_id, filament);
            recorder.set_nozzle_status(nozzle_id, filament, -1);
        }

        // MultiNozzleUtils.cpp:512  return get_weight_from_volume(total_flush_volume);
        get_weight_from_volume(total_flush_volume)
    }
}

// MultiNozzleUtils.cpp:515  // ==================== StaticNozzleGroupResult 实现 ====================
// MultiNozzleUtils.hpp:184  class StaticNozzleGroupResult : public NozzleGroupResultBase
#[derive(Debug, Clone)]
pub struct StaticNozzleGroupResult {
    // MultiNozzleUtils.hpp:83  bool support_dynamic_nozzle_map{false}; (base class member)
    support_dynamic_nozzle_map: bool,
    // MultiNozzleUtils.hpp:187  _filament_to_nozzles
    filament_to_nozzles: BTreeMap<i32, BTreeSet<i32>>,
    // MultiNozzleUtils.hpp:188  _nozzle_list_map
    nozzle_list_map: BTreeMap<i32, NozzleInfo>,
    // MultiNozzleUtils.hpp:189  _filament_change_seq
    filament_change_seq: Vec<i32>,
    // MultiNozzleUtils.hpp:190  _nozzle_change_seq
    nozzle_change_seq: Vec<i32>,
}

impl StaticNozzleGroupResult {
    // MultiNozzleUtils.hpp:193  StaticNozzleGroupResult(bool support_dynamic_map)
    fn with_support(support_dynamic_map: bool) -> Self {
        StaticNozzleGroupResult {
            support_dynamic_nozzle_map: support_dynamic_map,
            filament_to_nozzles: BTreeMap::new(),
            nozzle_list_map: BTreeMap::new(),
            filament_change_seq: Vec::new(),
            nozzle_change_seq: Vec::new(),
        }
    }

    // MultiNozzleUtils.hpp:94  bool is_support_dynamic_nozzle_map() const
    pub fn is_support_dynamic_nozzle_map(&self) -> bool {
        self.support_dynamic_nozzle_map
    }

    // MultiNozzleUtils.cpp:517  static std::optional<StaticNozzleGroupResult> create(...)
    pub fn create(
        filaments_info: &[FilamentInfo],
        nozzles_info: &[NozzleInfo],
        filament_change_seq: &[i32],
        nozzle_change_seq: &[i32],
        support_dynamic_nozzle_map: bool,
    ) -> Option<StaticNozzleGroupResult> {
        // MultiNozzleUtils.cpp:524  if (filaments_info.empty() || nozzles_info.empty()) return std::nullopt;
        if filaments_info.is_empty() || nozzles_info.is_empty() {
            return None;
        }

        // MultiNozzleUtils.cpp:526  std::map<int, NozzleInfo> nozzle_list_map;
        let mut nozzle_list_map: BTreeMap<i32, NozzleInfo> = BTreeMap::new();
        // MultiNozzleUtils.cpp:527  std::map<int, std::set<int>> filament_to_nozzles;
        let mut filament_to_nozzles: BTreeMap<i32, BTreeSet<i32>> = BTreeMap::new();

        // MultiNozzleUtils.cpp:529  for (auto nozzle_info : nozzles_info)
        for nozzle_info in nozzles_info {
            // MultiNozzleUtils.cpp:530  nozzle_list_map[nozzle_info.group_id] = nozzle_info;
            nozzle_list_map.insert(nozzle_info.group_id, nozzle_info.clone());
        }

        // MultiNozzleUtils.cpp:532  for (auto filament_info : filaments_info) {
        for filament_info in filaments_info {
            // MultiNozzleUtils.cpp:533  auto fil_id = filament_info.id;
            let fil_id = filament_info.id;
            // MultiNozzleUtils.cpp:534  auto nozzles_id = filament_info.group_id;
            let nozzles_id = &filament_info.group_id;
            // MultiNozzleUtils.cpp:535  std::set<int> nozzles_set(nozzles_id.begin(), nozzles_id.end());
            let nozzles_set: BTreeSet<i32> = nozzles_id.iter().cloned().collect();
            // MultiNozzleUtils.cpp:536  filament_to_nozzles[fil_id] = nozzles_set;
            filament_to_nozzles.insert(fil_id, nozzles_set);
        }

        // MultiNozzleUtils.cpp:539  StaticNozzleGroupResult result(support_dynamic_nozzle_map);
        let mut result = StaticNozzleGroupResult::with_support(support_dynamic_nozzle_map);
        // MultiNozzleUtils.cpp:540  result._filament_to_nozzles = filament_to_nozzles;
        result.filament_to_nozzles = filament_to_nozzles;
        // MultiNozzleUtils.cpp:541  result._nozzle_list_map = nozzle_list_map;
        result.nozzle_list_map = nozzle_list_map;
        // MultiNozzleUtils.cpp:542  result._filament_change_seq = filament_change_seq;
        result.filament_change_seq = filament_change_seq.to_vec();
        // MultiNozzleUtils.cpp:543  result._nozzle_change_seq = nozzle_change_seq;
        result.nozzle_change_seq = nozzle_change_seq.to_vec();

        // MultiNozzleUtils.cpp:545  return result;
        Some(result)
    }

    // MultiNozzleUtils.cpp:548  std::optional<NozzleInfo> StaticNozzleGroupResult::get_nozzle_from_id(int nozzle_id) const
    pub fn get_nozzle_from_id(&self, nozzle_id: i32) -> Option<NozzleInfo> {
        // MultiNozzleUtils.cpp:550  auto iter = _nozzle_list_map.find(nozzle_id);
        // MultiNozzleUtils.cpp:551  if (iter == _nozzle_list_map.end()) { return std::nullopt; }
        // MultiNozzleUtils.cpp:552  return iter->second;
        self.nozzle_list_map.get(&nozzle_id).cloned()
    }

    // MultiNozzleUtils.cpp:555  int StaticNozzleGroupResult::get_extruder_count() const
    pub fn get_extruder_count(&self) -> i32 {
        // MultiNozzleUtils.cpp:557  std::set<int> extruder_ids;
        let mut extruder_ids: BTreeSet<i32> = BTreeSet::new();
        // MultiNozzleUtils.cpp:558  for (const auto &elem : _nozzle_list_map) { extruder_ids.insert(elem.second.extruder_id); }
        for (_k, v) in &self.nozzle_list_map {
            extruder_ids.insert(v.extruder_id);
        }
        // MultiNozzleUtils.cpp:559  return static_cast<int>(extruder_ids.size());
        extruder_ids.len() as i32
    }

    // MultiNozzleUtils.cpp:562  std::vector<NozzleInfo> StaticNozzleGroupResult::get_used_nozzles_in_extruder(int target_extruder_id) const
    pub fn get_used_nozzles_in_extruder(&self, target_extruder_id: i32) -> Vec<NozzleInfo> {
        // MultiNozzleUtils.cpp:564  std::vector<NozzleInfo> result;
        let mut result: Vec<NozzleInfo> = Vec::new();
        // MultiNozzleUtils.cpp:565  for (const auto &elem : _nozzle_list_map) {
        for (_k, nozzle) in &self.nozzle_list_map {
            // MultiNozzleUtils.cpp:566  const auto &nozzle = elem.second;
            // MultiNozzleUtils.cpp:567  if (target_extruder_id == -1 || nozzle.extruder_id == target_extruder_id) {
            if target_extruder_id == -1 || nozzle.extruder_id == target_extruder_id {
                // MultiNozzleUtils.cpp:568  result.push_back(nozzle);
                result.push(nozzle.clone());
            }
        }
        // MultiNozzleUtils.cpp:571  return result;
        result
    }

    // MultiNozzleUtils.cpp:574  std::vector<int> StaticNozzleGroupResult::get_used_extruders() const
    pub fn get_used_extruders(&self) -> Vec<i32> {
        // MultiNozzleUtils.cpp:576  std::set<int> used_extruders;
        let mut used_extruders: BTreeSet<i32> = BTreeSet::new();
        // MultiNozzleUtils.cpp:577  for (const auto &elem : _nozzle_list_map) { used_extruders.insert(elem.second.extruder_id); }
        for (_k, v) in &self.nozzle_list_map {
            used_extruders.insert(v.extruder_id);
        }
        // MultiNozzleUtils.cpp:578  return std::vector<int>(used_extruders.begin(), used_extruders.end());
        used_extruders.into_iter().collect()
    }

    // MultiNozzleUtils.cpp:581  std::vector<unsigned int> StaticNozzleGroupResult::get_used_filaments() const
    pub fn get_used_filaments(&self) -> Vec<u32> {
        // MultiNozzleUtils.cpp:583  std::vector<unsigned int> used_filaments;
        let mut used_filaments: Vec<u32> = Vec::new();
        // MultiNozzleUtils.cpp:584  used_filaments.reserve(_filament_to_nozzles.size());
        used_filaments.reserve(self.filament_to_nozzles.len());
        // MultiNozzleUtils.cpp:585  for (const auto &elem : _filament_to_nozzles) {
        for (&first, _set) in &self.filament_to_nozzles {
            // MultiNozzleUtils.cpp:586  if (elem.first >= 0) {
            if first >= 0 {
                // MultiNozzleUtils.cpp:587  used_filaments.push_back(static_cast<unsigned int>(elem.first));
                used_filaments.push(first as u32);
            }
        }
        // MultiNozzleUtils.cpp:590  return used_filaments;
        used_filaments
    }

    // MultiNozzleUtils.cpp:593  std::vector<NozzleInfo> StaticNozzleGroupResult::get_nozzles_for_filament(int filament_id) const
    pub fn get_nozzles_for_filament(&self, filament_id: i32) -> Vec<NozzleInfo> {
        // MultiNozzleUtils.cpp:595  auto iter = _filament_to_nozzles.find(filament_id);
        // MultiNozzleUtils.cpp:596  if (iter == _filament_to_nozzles.end()) { return std::vector<NozzleInfo>(); }
        let iter = match self.filament_to_nozzles.get(&filament_id) {
            Some(s) => s,
            None => return Vec::new(),
        };

        // MultiNozzleUtils.cpp:598  std::vector<NozzleInfo> result;
        let mut result: Vec<NozzleInfo> = Vec::new();
        // MultiNozzleUtils.cpp:599  for (int nozzle_id : iter->second) {
        for &nozzle_id in iter {
            // MultiNozzleUtils.cpp:600  auto nozzle_iter = _nozzle_list_map.find(nozzle_id);
            // MultiNozzleUtils.cpp:601  if (nozzle_iter != _nozzle_list_map.end()) { result.push_back(nozzle_iter->second); }
            if let Some(nozzle) = self.nozzle_list_map.get(&nozzle_id) {
                result.push(nozzle.clone());
            }
        }
        // MultiNozzleUtils.cpp:605  return result;
        result
    }

    // MultiNozzleUtils.cpp:608  std::optional<NozzleInfo> StaticNozzleGroupResult::get_first_nozzle_for_filament(int filament_id) const
    pub fn get_first_nozzle_for_filament(&self, filament_id: i32) -> Option<NozzleInfo> {
        // MultiNozzleUtils.cpp:610  if (filament_id < 0) return std::nullopt;
        if filament_id < 0 {
            return None;
        }

        // MultiNozzleUtils.cpp:612  if (!_filament_change_seq.empty() && _filament_change_seq.size() == _nozzle_change_seq.size()) {
        if !self.filament_change_seq.is_empty()
            && self.filament_change_seq.len() == self.nozzle_change_seq.len()
        {
            // MultiNozzleUtils.cpp:613  for (size_t idx = 0; idx < _filament_change_seq.size(); ++idx) {
            for idx in 0..self.filament_change_seq.len() {
                // MultiNozzleUtils.cpp:614  if (_filament_change_seq[idx] == filament_id) {
                if self.filament_change_seq[idx] == filament_id {
                    // MultiNozzleUtils.cpp:615  int nozzle_id = _nozzle_change_seq[idx];
                    let nozzle_id = self.nozzle_change_seq[idx];
                    // MultiNozzleUtils.cpp:616  auto nozzle = get_nozzle_from_id(nozzle_id);
                    let nozzle = self.get_nozzle_from_id(nozzle_id);
                    // MultiNozzleUtils.cpp:617  if (nozzle) return nozzle;
                    if nozzle.is_some() {
                        return nozzle;
                    }
                }
            }
        }

        // MultiNozzleUtils.cpp:622  auto iter = _filament_to_nozzles.find(filament_id);
        // MultiNozzleUtils.cpp:623  if (iter == _filament_to_nozzles.end()) return std::nullopt;
        let iter = match self.filament_to_nozzles.get(&filament_id) {
            Some(s) => s,
            None => return None,
        };

        // MultiNozzleUtils.cpp:625  for (int nozzle_id : iter->second) {
        for &nozzle_id in iter {
            // MultiNozzleUtils.cpp:626  auto nozzle = get_nozzle_from_id(nozzle_id);
            let nozzle = self.get_nozzle_from_id(nozzle_id);
            // MultiNozzleUtils.cpp:627  if (nozzle) return nozzle;
            if nozzle.is_some() {
                return nozzle;
            }
        }

        // MultiNozzleUtils.cpp:630  return std::nullopt;
        None
    }
}

// MultiNozzleUtils.hpp:239-242  struct FilamentChangeSimResult
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct FilamentChangeSimResult {
    pub actual_time: f64, // MultiNozzleUtils.hpp:240  = 0.0
    pub sliced_time: f64, // MultiNozzleUtils.hpp:241  = 0.0
}

// MultiNozzleUtils.cpp:843  enum class Location { IN_AMS, IN_SELECTOR, IN_EXTRUDER };
// (local to simulate_filament_change_time; `operator[]` of its map value-initializes to IN_AMS)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Location {
    #[default]
    InAms,
    InSelector,
    InExtruder,
}

// MultiNozzleUtils.cpp:797-991  FilamentChangeSimResult simulate_filament_change_time(...)
//   header defaults (MultiNozzleUtils.hpp:244-252): ams_preload_enabled = {}, calc_sliced_time = false
#[allow(clippy::too_many_arguments)]
pub fn simulate_filament_change_time(
    logical_filaments: &[i32],
    nozzle_list: &[NozzleInfo],
    filament_change_seq: &[i32],
    nozzle_change_seq: &[i32],
    group_of_filament: &[i32],
    time_params: &FilamentChangeTimeParams,
    ams_preload_enabled: &[bool],
    calc_sliced_time: bool,
) -> FilamentChangeSimResult {
    // MultiNozzleUtils.cpp:807
    let mut result = FilamentChangeSimResult::default();
    // MultiNozzleUtils.cpp:808-809
    if logical_filaments.is_empty() || nozzle_list.is_empty() || filament_change_seq.is_empty() || nozzle_change_seq.is_empty() {
        return result;
    }

    // MultiNozzleUtils.cpp:811-817  参数语义重新映射
    let load_ams_to_selector: f32 = time_params.standard_load_time - time_params.selector_load_time;
    let unload_ams_to_selector: f32 = time_params.standard_unload_time - time_params.selector_unload_time;
    let load_selector_to_ext: f32 = time_params.selector_load_time;
    let unload_ext_to_selector: f32 = time_params.selector_unload_time;

    // MultiNozzleUtils.cpp:819-823  nozzle_id -> extruder_id
    let mut nozzle_to_extruder: UnorderedMapInt<i32> = UnorderedMapInt::new();
    nozzle_to_extruder.reserve(nozzle_list.len());
    for nozzle in nozzle_list {
        *nozzle_to_extruder.entry(nozzle.group_id) = nozzle.extruder_id;
    }

    // MultiNozzleUtils.cpp:825-829  filament_id -> AMS group
    let mut filament_to_group: UnorderedMapInt<i32> = UnorderedMapInt::new();
    filament_to_group.reserve(logical_filaments.len());
    for i in 0..logical_filaments.len() {
        *filament_to_group.entry(logical_filaments[i]) = group_of_filament[i];
    }

    // MultiNozzleUtils.cpp:831-834
    let get_group = |filament_to_group: &UnorderedMapInt<i32>, filament_id: i32| -> i32 {
        match filament_to_group.get(filament_id) {
            Some(&g) => g,
            None => -1,
        }
    };

    // MultiNozzleUtils.cpp:836-840
    let is_preload_enabled = |group_id: i32| -> bool {
        if group_id < 0 || group_id >= ams_preload_enabled.len() as i32 {
            return false;
        }
        ams_preload_enabled[group_id as usize]
    };

    // MultiNozzleUtils.cpp:842-849  料的位置状态
    let mut filament_location: UnorderedMapInt<Location> = UnorderedMapInt::new(); // filament_id -> 当前位置
    let mut filament_extruder: UnorderedMapInt<i32> = UnorderedMapInt::new(); // filament_id -> 所在挤出机
    let mut extruder_filament: UnorderedMapInt<i32> = UnorderedMapInt::new(); // extruder_id -> 当前装载的料
    // group_id -> 当前占用了该AMS通道的料集合（IN_SELECTOR或IN_EXTRUDER）
    let mut ams_group_occupied: UnorderedMapInt<UnorderedMapInt<()>> = UnorderedMapInt::new();

    // MultiNozzleUtils.cpp:851-852
    filament_location.reserve(logical_filaments.len());
    filament_extruder.reserve(logical_filaments.len());

    // MultiNozzleUtils.cpp:854-856  初始状态：所有料在AMS，所有挤出机为空
    for &f in logical_filaments {
        *filament_location.entry(f) = Location::InAms;
    }

    // MultiNozzleUtils.cpp:858-859  切片预估模拟器
    let mut sliced_recorder = NozzleStatusRecorder::new();

    // MultiNozzleUtils.cpp:861-863
    let seq_len = filament_change_seq.len().min(nozzle_change_seq.len());
    let mut actual_time: f64 = 0.0;
    let mut sliced_time: f64 = 0.0;

    // MultiNozzleUtils.cpp:865
    for i in 0..seq_len {
        // MultiNozzleUtils.cpp:866-867
        let b = filament_change_seq[i];
        let nozzle_id = nozzle_change_seq[i];

        // MultiNozzleUtils.cpp:869-872
        let e = match nozzle_to_extruder.get(nozzle_id) {
            Some(&v) => v, // 目标挤出机
            None => continue,
        };

        // MultiNozzleUtils.cpp:874-891  Step 0: 计算切片预估时间
        if calc_sliced_time {
            let old_nozzle_in_e = sliced_recorder.get_nozzle_in_extruder(e);
            let old_filament_in_nozzle = sliced_recorder.get_filament_in_nozzle(nozzle_id);
            let old_filament_in_ext = sliced_recorder.get_filament_in_nozzle(old_nozzle_in_e);

            let nozzle_change = old_nozzle_in_e != nozzle_id;
            let filament_change = old_filament_in_nozzle != b;

            if nozzle_change || filament_change {
                if old_filament_in_ext != -1 {
                    sliced_time += time_params.standard_unload_time as f64;
                }
                sliced_time += time_params.standard_load_time as f64;
            }
            sliced_recorder.set_nozzle_status(nozzle_id, b, e);
        }

        // MultiNozzleUtils.cpp:893-899  Step 1: 查目标挤出机E当前装载的料A
        let mut a: i32 = -1;
        if let Some(&v) = extruder_filament.get(e) {
            a = v;
        }

        // MultiNozzleUtils.cpp:901-902
        let group_b = get_group(&filament_to_group, b);
        let group_a = if a != -1 { get_group(&filament_to_group, a) } else { -1 };

        // MultiNozzleUtils.cpp:904-921  Step 2: 清理B的AMS通道占用
        if let Some(occupied) = ams_group_occupied.get(group_b) {
            let occupied: Vec<i32> = occupied.keys();
            for x in occupied {
                if x == b {
                    continue;
                }
                // X 与 B 同 AMS，退回 AMS 让路
                let loc_x = *filament_location.entry(x);
                if loc_x == Location::InExtruder {
                    actual_time += (unload_ext_to_selector + unload_ams_to_selector) as f64;
                    let e2 = *filament_extruder.entry(x);
                    extruder_filament.erase(e2);
                    filament_extruder.erase(x);
                } else if loc_x == Location::InSelector {
                    actual_time += unload_ams_to_selector as f64;
                }
                *filament_location.entry(x) = Location::InAms;
            }
            ams_group_occupied.get_mut(group_b).expect("found above").clear();
        }

        // MultiNozzleUtils.cpp:923-940  Step 3: A从E退出 / Step 3.5: 预进料B
        let mut step3_executed = false;
        let mut step3_time: f32 = 0.0;
        if a != -1 && a != b && *filament_location.entry(a) == Location::InExtruder {
            if is_preload_enabled(group_a) && group_a != group_b {
                step3_time = unload_ext_to_selector;
                *filament_location.entry(a) = Location::InSelector;
            } else {
                step3_time = unload_ext_to_selector + unload_ams_to_selector;
                *filament_location.entry(a) = Location::InAms;
                ams_group_occupied.entry(group_a).erase(a);
            }
            extruder_filament.erase(e);
            filament_extruder.erase(a);
            step3_executed = true;
        }

        // MultiNozzleUtils.cpp:942-950
        let mut step3_5_time: f32 = 0.0;
        if step3_executed && *filament_location.entry(b) == Location::InAms && group_a != group_b && is_preload_enabled(group_b) {
            step3_5_time = load_ams_to_selector;
            *filament_location.entry(b) = Location::InSelector;
            ams_group_occupied.entry(group_b).insert(b, ());
        }

        // MultiNozzleUtils.cpp:952
        actual_time += step3_time.max(step3_5_time) as f64;

        // MultiNozzleUtils.cpp:954-962  Step 4: B推入E
        let mut step4_time: f32 = 0.0;
        let loc_b = *filament_location.entry(b);
        if loc_b == Location::InAms {
            step4_time = load_ams_to_selector + load_selector_to_ext;
        } else if loc_b == Location::InSelector {
            step4_time = load_selector_to_ext;
        }

        // MultiNozzleUtils.cpp:964-968  Step 5: 更新状态
        *extruder_filament.entry(e) = b;
        *filament_location.entry(b) = Location::InExtruder;
        *filament_extruder.entry(b) = e;
        ams_group_occupied.entry(group_b).insert(b, ());

        // MultiNozzleUtils.cpp:970-982  Step 6: 预进料下一个料C
        let mut step6_time: f32 = 0.0;
        if i + 1 < seq_len {
            let c = filament_change_seq[i + 1];
            let group_c = get_group(&filament_to_group, c);
            if *filament_location.entry(c) == Location::InAms
                && group_c != group_b
                && is_preload_enabled(group_c)
                && ams_group_occupied.entry(group_c).is_empty()
            {
                step6_time = load_ams_to_selector;
                *filament_location.entry(c) = Location::InSelector;
                ams_group_occupied.entry(group_c).insert(c, ());
            }
        }

        // MultiNozzleUtils.cpp:984
        actual_time += step4_time.max(step6_time) as f64;
    }

    // MultiNozzleUtils.cpp:987-989
    result.actual_time = actual_time;
    result.sliced_time = sliced_time;
    result
}

// MultiNozzleUtils.cpp:993-1007  float calc_filament_change_gap_for_assignment(...)
//   header default (MultiNozzleUtils.hpp:254-261): ams_preload_enabled = {}
#[allow(clippy::too_many_arguments)]
pub fn calc_filament_change_gap_for_assignment(
    logical_filaments: &[i32],
    nozzle_list: &[NozzleInfo],
    filament_change_seq: &[i32],
    nozzle_change_seq: &[i32],
    group_of_filament: &[i32],
    time_params: &FilamentChangeTimeParams,
    ams_preload_enabled: &[bool],
) -> f32 {
    // MultiNozzleUtils.cpp:1002-1005
    let r = simulate_filament_change_time(
        logical_filaments,
        nozzle_list,
        filament_change_seq,
        nozzle_change_seq,
        group_of_filament,
        time_params,
        ams_preload_enabled,
        /*calc_sliced_time=*/ true,
    );
    // MultiNozzleUtils.cpp:1006
    (r.actual_time - r.sliced_time) as f32
}

// MultiNozzleUtils.cpp:787  std::vector<int> find_optimal_physical_assignment(...)
//   header default: int max_ms = 1000  (MultiNozzleUtils.hpp:254)
pub fn find_optimal_physical_assignment(
    logical_filaments: &[i32],
    nozzle_list: &[NozzleInfo],
    filament_change_seq: &[i32],
    nozzle_change_seq: &[i32],
    group_count: i32,
    time_params: &FilamentChangeTimeParams,
    max_ms: i32,
) -> Vec<i32> {
    // MultiNozzleUtils.cpp:796  size_t count = logical_filaments.size();
    let count = logical_filaments.len();
    // MultiNozzleUtils.cpp:797  if (count == 0 || group_count <= 0) return {};
    if count == 0 || group_count <= 0 {
        return Vec::new();
    }

    // MultiNozzleUtils.cpp:799  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(max_ms);
    let deadline = Instant::now() + Duration::from_millis(max_ms as u64);

    // MultiNozzleUtils.cpp:801  std::vector<int> assignment(count, 0);
    let mut assignment: Vec<i32> = vec![0; count];
    // MultiNozzleUtils.cpp:802  std::vector<int> best_assignment = assignment;
    let mut best_assignment = assignment.clone();
    // MultiNozzleUtils.cpp:803  float best_gap = calc_filament_change_gap_for_assignment(...);
    let mut best_gap = calc_filament_change_gap_for_assignment(
        logical_filaments,
        nozzle_list,
        filament_change_seq,
        nozzle_change_seq,
        &assignment,
        time_params,
        &[],
    );

    // MultiNozzleUtils.cpp:805  bool done = false;
    let mut done = false;
    // MultiNozzleUtils.cpp:806  bool timed_out = false;
    let mut timed_out = false;
    // MultiNozzleUtils.cpp:807  while (!done) {
    while !done {
        // MultiNozzleUtils.cpp:808  if (assignment[0] == 0) {
        if assignment[0] == 0 {
            // MultiNozzleUtils.cpp:809  对称性剪枝：固定首个耗材在组0
            // MultiNozzleUtils.cpp:810  float gap = calc_filament_change_gap_for_assignment(...);
            let gap = calc_filament_change_gap_for_assignment(
                logical_filaments,
                nozzle_list,
                filament_change_seq,
                nozzle_change_seq,
                &assignment,
                time_params,
                &[],
            );
            // MultiNozzleUtils.cpp:811  if (gap < best_gap) {
            if gap < best_gap {
                // MultiNozzleUtils.cpp:812  best_gap = gap;
                best_gap = gap;
                // MultiNozzleUtils.cpp:813  best_assignment = assignment;
                best_assignment = assignment.clone();
            }
        }
        // MultiNozzleUtils.cpp:816  for (size_t pos = 0; pos < count; ++pos) {
        for pos in 0..count {
            // MultiNozzleUtils.cpp:817  assignment[pos] += 1;
            assignment[pos] += 1;
            // MultiNozzleUtils.cpp:818  if (assignment[pos] < group_count) break;
            if assignment[pos] < group_count {
                break;
            }
            // MultiNozzleUtils.cpp:819  assignment[pos] = 0;
            assignment[pos] = 0;
            // MultiNozzleUtils.cpp:820  if (pos == count - 1) done = true;
            if pos == count - 1 {
                done = true;
            }
        }

        // MultiNozzleUtils.cpp:823  if (!done && std::chrono::steady_clock::now() > deadline) {
        if !done && Instant::now() > deadline {
            // MultiNozzleUtils.cpp:824  timed_out = true;
            timed_out = true;
            // MultiNozzleUtils.cpp:825  break;
            break;
        }
    }

    // MultiNozzleUtils.cpp:829  if (timed_out) {
    if timed_out {
        // MultiNozzleUtils.cpp:830-834  BOOST_LOG_TRIVIAL(warning) << ... timed out ...  (logging omitted)
        let _ = timed_out;
    }

    // MultiNozzleUtils.cpp:837  return best_assignment;
    best_assignment
}

// ==================== std::unordered_map<int, V> of libc++ ====================
//
// FIDELITY-NOTE: `std::unordered_map` / `std::unordered_set` with `int` keys whose order of
// iteration reaches a result (MultiNozzleUtils.hpp:216-217 `NozzleStatusRecorder`,
// FilamentGroup.hpp:100 `filament_print_time`, FilamentGroup.hpp:110 `nozzle_status`,
// MultiNozzleUtils.cpp:846-849 the maps of simulate_filament_change_time). The order is
// implementation-defined; this is libc++ (`<__hash_table>`, `<unordered_map>` of the macOS SDK):
// one singly linked list of nodes, a bucket holds the node before its first node,
// `std::hash<int>` is the value widened to `size_t`, max_load_factor 1. Copy construction
// (`Clone`), copy assignment (`assign_from`), `clear`, `reserve` and insertion follow libc++, so a
// copy iterates in the order the C++ copy does.

/// The value `std::hash<int>` gives: the `int` converted to `size_t`.
fn libcxx_hash_int(k: i32) -> usize {
    k as i64 as u64 as usize
}

/// libc++ `std::__constrain_hash`.
fn libcxx_constrain_hash(h: usize, bc: usize) -> usize {
    if bc & bc.wrapping_sub(1) == 0 {
        h & bc.wrapping_sub(1)
    } else if h < bc {
        h
    } else {
        h % bc
    }
}

/// libc++ `std::__next_prime`: the smallest prime not below `n`.
fn libcxx_next_prime(n: usize) -> usize {
    let mut p = n.max(2);
    loop {
        let mut is_prime = true;
        let mut d = 2usize;
        while d * d <= p {
            if p % d == 0 {
                is_prime = false;
                break;
            }
            d += 1;
        }
        if is_prime {
            return p;
        }
        p += 1;
    }
}

/// libc++ `std::__is_hash_power2`.
fn libcxx_is_hash_power2(bc: usize) -> bool {
    bc > 2 && (bc & (bc - 1)) == 0
}

/// libc++ `std::__next_hash_pow2`.
fn libcxx_next_hash_pow2(n: usize) -> usize {
    if n < 2 {
        n
    } else {
        1usize << (usize::BITS - (n - 1).leading_zeros())
    }
}

const LIBCXX_NIL: usize = usize::MAX;

/// libc++ `std::unordered_map<int, V>` (see the note above). `std::unordered_set<int>` is
/// `UnorderedMapInt<()>`.
#[derive(Debug)]
pub struct UnorderedMapInt<V> {
    /// node 0 is `__p1_`, the node before the first; its key is not read
    keys: Vec<i32>,
    values: Vec<Option<V>>,
    next: Vec<usize>,
    /// `__bucket_list_`: for each bucket the node before its first node, or `LIBCXX_NIL`
    buckets: Vec<usize>,
    size: usize,
}

impl<V> Default for UnorderedMapInt<V> {
    fn default() -> Self {
        Self::new()
    }
}

impl<V> UnorderedMapInt<V> {
    /// `unordered_map()`
    pub fn new() -> Self {
        UnorderedMapInt { keys: vec![0], values: vec![None], next: vec![LIBCXX_NIL], buckets: Vec::new(), size: 0 }
    }

    /// `unordered_map(size_type __n)`: `__table_.__rehash_unique(__n)`.
    pub fn with_bucket_count(n: usize) -> Self {
        let mut m = Self::new();
        m.rehash(n);
        m
    }

    fn hash_of(&self, nd: usize) -> usize {
        libcxx_hash_int(self.keys[nd])
    }

    /// `bucket_count()`
    pub fn bucket_count(&self) -> usize {
        self.buckets.len()
    }

    /// `size()`
    pub fn len(&self) -> usize {
        self.size
    }

    /// `empty()`
    pub fn is_empty(&self) -> bool {
        self.size == 0
    }

    /// `__hash_table::__rehash<_UniqueKeys>(n)` (both branches; equal keys never meet here).
    fn rehash(&mut self, mut n: usize) {
        if n == 1 {
            n = 2;
        } else if n & n.wrapping_sub(1) != 0 {
            n = libcxx_next_prime(n);
        }
        let bc = self.buckets.len();
        if n > bc {
            self.do_rehash(n);
        } else if n < bc {
            let need = (self.size as f32 / 1.0f32).ceil() as usize;
            n = n.max(if libcxx_is_hash_power2(bc) { libcxx_next_hash_pow2(need) } else { libcxx_next_prime(need) });
            if n < bc {
                self.do_rehash(n);
            }
        }
    }

    /// `__hash_table::__do_rehash<_UniqueKeys>(nbc)`
    fn do_rehash(&mut self, nbc: usize) {
        self.buckets = vec![LIBCXX_NIL; nbc];
        if nbc == 0 {
            return;
        }
        let mut pp = 0usize;
        let mut cp = self.next[pp];
        if cp != LIBCXX_NIL {
            let mut chash = libcxx_constrain_hash(self.hash_of(cp), nbc);
            self.buckets[chash] = pp;
            let mut phash = chash;
            pp = cp;
            cp = self.next[cp];
            while cp != LIBCXX_NIL {
                chash = libcxx_constrain_hash(self.hash_of(cp), nbc);
                if chash == phash {
                    pp = cp;
                } else if self.buckets[chash] == LIBCXX_NIL {
                    self.buckets[chash] = pp;
                    pp = cp;
                    phash = chash;
                } else {
                    let np = cp;
                    self.next[pp] = self.next[np];
                    let b = self.buckets[chash];
                    self.next[np] = self.next[b];
                    self.next[b] = cp;
                }
                cp = self.next[pp];
            }
        }
    }

    /// `reserve(n)`: `__rehash_unique(size_type(ceil(float(n) / max_load_factor())))`.
    pub fn reserve(&mut self, n: usize) {
        self.rehash((n as f32 / 1.0f32).ceil() as usize);
    }

    /// `find(k)`: the node of the key, or `LIBCXX_NIL`.
    fn find_node(&self, k: i32) -> usize {
        let bc = self.buckets.len();
        if bc != 0 {
            let h = libcxx_hash_int(k);
            let chash = libcxx_constrain_hash(h, bc);
            let mut nd = self.buckets[chash];
            if nd != LIBCXX_NIL {
                nd = self.next[nd];
                while nd != LIBCXX_NIL
                    && (self.hash_of(nd) == h || libcxx_constrain_hash(self.hash_of(nd), bc) == chash)
                {
                    if self.keys[nd] == k {
                        return nd;
                    }
                    nd = self.next[nd];
                }
            }
        }
        LIBCXX_NIL
    }

    /// `find(k) != end()`, `count(k)`
    pub fn contains_key(&self, k: i32) -> bool {
        self.find_node(k) != LIBCXX_NIL
    }

    /// `count(k)`
    pub fn count(&self, k: i32) -> usize {
        self.contains_key(k) as usize
    }

    /// `find(k)->second`
    pub fn get(&self, k: i32) -> Option<&V> {
        let nd = self.find_node(k);
        if nd == LIBCXX_NIL {
            None
        } else {
            self.values[nd].as_ref()
        }
    }

    /// `find(k)->second`, writable
    pub fn get_mut(&mut self, k: i32) -> Option<&mut V> {
        let nd = self.find_node(k);
        if nd == LIBCXX_NIL {
            None
        } else {
            self.values[nd].as_mut()
        }
    }

    /// `__emplace_unique_key_args(k, ...)`: the node of the key, a new node holding `make()` at
    /// the front of its bucket when the key is not there. Returns (node, inserted).
    fn emplace_unique(&mut self, k: i32, make: impl FnOnce() -> V) -> (usize, bool) {
        let found = self.find_node(k);
        if found != LIBCXX_NIL {
            return (found, false);
        }
        let mut bc = self.buckets.len();
        if (self.size + 1) as f32 > bc as f32 * 1.0f32 || bc == 0 {
            let a = 2 * bc + (!libcxx_is_hash_power2(bc)) as usize;
            let b = ((self.size + 1) as f32 / 1.0f32).ceil() as usize;
            self.rehash(a.max(b));
            bc = self.buckets.len();
        }
        let chash = libcxx_constrain_hash(libcxx_hash_int(k), bc);
        let nd = self.keys.len();
        self.keys.push(k);
        self.values.push(Some(make()));
        self.next.push(LIBCXX_NIL);
        let pn = self.buckets[chash];
        if pn == LIBCXX_NIL {
            self.next[nd] = self.next[0];
            self.next[0] = nd;
            self.buckets[chash] = 0;
            if self.next[nd] != LIBCXX_NIL {
                let nh = libcxx_constrain_hash(self.hash_of(self.next[nd]), bc);
                self.buckets[nh] = nd;
            }
        } else {
            self.next[nd] = self.next[pn];
            self.next[pn] = nd;
        }
        self.size += 1;
        (nd, true)
    }

    /// `insert({k, v})` / `emplace(k, v)`: false (and no change) when the key is there.
    pub fn insert(&mut self, k: i32, v: V) -> bool {
        let mut v = Some(v);
        self.emplace_unique(k, || v.take().expect("one value")).1
    }

    /// `__node_insert_multi(k, v)`: after the last node of its bucket (copy assignment uses it).
    fn insert_multi(&mut self, k: i32, v: V) {
        let mut bc = self.buckets.len();
        if (self.size + 1) as f32 > bc as f32 * 1.0f32 || bc == 0 {
            let a = 2 * bc + (!libcxx_is_hash_power2(bc)) as usize;
            let b = ((self.size + 1) as f32 / 1.0f32).ceil() as usize;
            self.rehash(a.max(b));
            bc = self.buckets.len();
        }
        let h = libcxx_hash_int(k);
        let chash = libcxx_constrain_hash(h, bc);
        let mut pn = self.buckets[chash];
        if pn != LIBCXX_NIL {
            let mut found = false;
            while self.next[pn] != LIBCXX_NIL && libcxx_constrain_hash(self.hash_of(self.next[pn]), bc) == chash {
                let eq = self.hash_of(self.next[pn]) == h && self.keys[self.next[pn]] == k;
                if found != eq {
                    if !found {
                        found = true;
                    } else {
                        break;
                    }
                }
                pn = self.next[pn];
            }
        }
        let cp = self.keys.len();
        self.keys.push(k);
        self.values.push(Some(v));
        self.next.push(LIBCXX_NIL);
        if pn == LIBCXX_NIL {
            self.next[cp] = self.next[0];
            self.next[0] = cp;
            self.buckets[chash] = 0;
            if self.next[cp] != LIBCXX_NIL {
                let nh = libcxx_constrain_hash(self.hash_of(self.next[cp]), bc);
                self.buckets[nh] = cp;
            }
        } else {
            self.next[cp] = self.next[pn];
            self.next[pn] = cp;
            if self.next[cp] != LIBCXX_NIL {
                let nhash = libcxx_constrain_hash(self.hash_of(self.next[cp]), bc);
                if nhash != chash {
                    self.buckets[nhash] = cp;
                }
            }
        }
        self.size += 1;
    }

    /// `erase(k)` (`__hash_table::remove`): the number of elements removed.
    pub fn erase(&mut self, k: i32) -> usize {
        let cn = self.find_node(k);
        if cn == LIBCXX_NIL {
            return 0;
        }
        let bc = self.buckets.len();
        let chash = libcxx_constrain_hash(self.hash_of(cn), bc);
        let mut pn = self.buckets[chash];
        while self.next[pn] != cn {
            pn = self.next[pn];
        }
        if pn == 0 || libcxx_constrain_hash(self.hash_of(pn), bc) != chash {
            if self.next[cn] == LIBCXX_NIL || libcxx_constrain_hash(self.hash_of(self.next[cn]), bc) != chash {
                self.buckets[chash] = LIBCXX_NIL;
            }
        }
        if self.next[cn] != LIBCXX_NIL {
            let nhash = libcxx_constrain_hash(self.hash_of(self.next[cn]), bc);
            if nhash != chash {
                self.buckets[nhash] = pn;
            }
        }
        self.next[pn] = self.next[cn];
        self.next[cn] = LIBCXX_NIL;
        self.values[cn] = None;
        self.size -= 1;
        1
    }

    /// `clear()`: no element left, the bucket count kept.
    pub fn clear(&mut self) {
        if self.size > 0 {
            self.keys.truncate(1);
            self.values.truncate(1);
            self.next.truncate(1);
            self.next[0] = LIBCXX_NIL;
            for b in self.buckets.iter_mut() {
                *b = LIBCXX_NIL;
            }
            self.size = 0;
        }
    }

    /// The nodes from `begin()` to `end()`.
    fn order(&self) -> Vec<usize> {
        let mut out = Vec::with_capacity(self.size);
        let mut nd = self.next[0];
        while nd != LIBCXX_NIL {
            out.push(nd);
            nd = self.next[nd];
        }
        out
    }

    /// The pairs from `begin()` to `end()`.
    pub fn iter(&self) -> impl Iterator<Item = (i32, &V)> + '_ {
        self.order().into_iter().map(move |nd| (self.keys[nd], self.values[nd].as_ref().expect("a node with a value")))
    }

    /// The keys from `begin()` to `end()`.
    pub fn keys(&self) -> Vec<i32> {
        self.order().into_iter().map(|nd| self.keys[nd]).collect()
    }
}

impl<V: Default> UnorderedMapInt<V> {
    /// `operator[](k)`: the value of the key, added with the default value when it is not there.
    pub fn entry(&mut self, k: i32) -> &mut V {
        let (nd, _) = self.emplace_unique(k, V::default);
        self.values[nd].as_mut().expect("a node with a value")
    }
}

impl<V: Clone> UnorderedMapInt<V> {
    /// Copy assignment (`__hash_table::operator=(const __hash_table&)`, `__assign_multi`): the
    /// elements dropped, the bucket count kept, the elements of `other` inserted in its order
    /// after the last node of their bucket.
    pub fn assign_from(&mut self, other: &UnorderedMapInt<V>) {
        // __detach(): every bucket emptied, size 0
        self.keys.truncate(1);
        self.values.truncate(1);
        self.next.truncate(1);
        self.next[0] = LIBCXX_NIL;
        for b in self.buckets.iter_mut() {
            *b = LIBCXX_NIL;
        }
        self.size = 0;
        for nd in other.order() {
            self.insert_multi(other.keys[nd], other.values[nd].clone().expect("a node with a value"));
        }
    }
}

impl<V: Clone> Clone for UnorderedMapInt<V> {
    /// Copy construction (`unordered_map(const unordered_map&)`): `__rehash_unique(bucket_count())`
    /// then `insert(begin(), end())`, each element at the front of its bucket.
    fn clone(&self) -> Self {
        let mut m = UnorderedMapInt::new();
        m.rehash(self.buckets.len());
        for nd in self.order() {
            m.insert(self.keys[nd], self.values[nd].clone().expect("a node with a value"));
        }
        m
    }
}

// MultiNozzleUtils.cpp:840  // ==================== NozzleStatusRecorder 实现 ====================
// MultiNozzleUtils.hpp:213  class NozzleStatusRecorder
#[derive(Debug, Clone)]
pub struct NozzleStatusRecorder {
    // MultiNozzleUtils.hpp:216  std::unordered_map<int, int> nozzle_filament_status;
    nozzle_filament_status: UnorderedMapInt<i32>,
    // MultiNozzleUtils.hpp:217  std::unordered_map<int, int> extruder_nozzle_status;
    extruder_nozzle_status: UnorderedMapInt<i32>,
    // MultiNozzleUtils.hpp:218  int current_extruder_id_ = -1;
    current_extruder_id_: i32,
}

impl NozzleStatusRecorder {
    // MultiNozzleUtils.hpp:221  NozzleStatusRecorder() = default;
    pub fn new() -> Self {
        NozzleStatusRecorder {
            nozzle_filament_status: UnorderedMapInt::new(),
            extruder_nozzle_status: UnorderedMapInt::new(),
            current_extruder_id_: -1,
        }
    }

    // MultiNozzleUtils.cpp:842  bool NozzleStatusRecorder::is_nozzle_empty(int nozzle_id) const
    pub fn is_nozzle_empty(&self, nozzle_id: i32) -> bool {
        // MultiNozzleUtils.cpp:844  auto iter = nozzle_filament_status.find(nozzle_id);
        // MultiNozzleUtils.cpp:845  if (iter == nozzle_filament_status.end()) return true;
        // MultiNozzleUtils.cpp:846  return false;
        !self.nozzle_filament_status.contains_key(nozzle_id)
    }

    // MultiNozzleUtils.cpp:849  int NozzleStatusRecorder::get_filament_in_nozzle(int nozzle_id) const
    pub fn get_filament_in_nozzle(&self, nozzle_id: i32) -> i32 {
        // MultiNozzleUtils.cpp:851  auto iter = nozzle_filament_status.find(nozzle_id);
        // MultiNozzleUtils.cpp:852  if (iter == nozzle_filament_status.end()) return -1;
        // MultiNozzleUtils.cpp:853  return iter->second;
        match self.nozzle_filament_status.get(nozzle_id) {
            Some(&v) => v,
            None => -1,
        }
    }

    // MultiNozzleUtils.cpp:856  int NozzleStatusRecorder::get_nozzle_in_extruder(int extruder_id) const
    pub fn get_nozzle_in_extruder(&self, extruder_id: i32) -> i32 {
        // MultiNozzleUtils.cpp:858  auto iter = extruder_nozzle_status.find(extruder_id);
        // MultiNozzleUtils.cpp:859  if (iter == extruder_nozzle_status.end()) return -1;
        // MultiNozzleUtils.cpp:860  return iter->second;
        match self.extruder_nozzle_status.get(extruder_id) {
            Some(&v) => v,
            None => -1,
        }
    }

    // MultiNozzleUtils.hpp:225  int get_current_extruder_id() const { return current_extruder_id_; }
    pub fn get_current_extruder_id(&self) -> i32 {
        self.current_extruder_id_
    }

    // MultiNozzleUtils.cpp:865  void NozzleStatusRecorder::set_nozzle_status(int nozzle_id, int filament_id, int extruder_id)
    //   header default: int extruder_id = -1  (MultiNozzleUtils.hpp:231)
    pub fn set_nozzle_status(&mut self, nozzle_id: i32, filament_id: i32, extruder_id: i32) {
        // MultiNozzleUtils.cpp:867  nozzle_filament_status[nozzle_id] = filament_id;
        *self.nozzle_filament_status.entry(nozzle_id) = filament_id;
        // MultiNozzleUtils.cpp:868  if (extruder_id != -1) {
        if extruder_id != -1 {
            // MultiNozzleUtils.cpp:869  extruder_nozzle_status[extruder_id] = nozzle_id;
            *self.extruder_nozzle_status.entry(extruder_id) = nozzle_id;
        }
    }

    // MultiNozzleUtils.cpp:873  void NozzleStatusRecorder::clear_nozzle_status(int nozzle_id)
    pub fn clear_nozzle_status(&mut self, nozzle_id: i32) {
        // MultiNozzleUtils.cpp:875  auto iter = nozzle_filament_status.find(nozzle_id);
        // MultiNozzleUtils.cpp:876  if (iter == nozzle_filament_status.end()) return;
        // MultiNozzleUtils.cpp:877  nozzle_filament_status.erase(iter);
        self.nozzle_filament_status.erase(nozzle_id);
    }

    // MultiNozzleUtils.hpp:228  void set_current_extruder_id(int extruder_id) { current_extruder_id_ = extruder_id; }
    pub fn set_current_extruder_id(&mut self, extruder_id: i32) {
        self.current_extruder_id_ = extruder_id;
    }

    // MultiNozzleUtils.hpp:234  const std::unordered_map<int, int>& get_nozzle_filament_map() const
    pub fn get_nozzle_filament_map(&self) -> &UnorderedMapInt<i32> {
        &self.nozzle_filament_status
    }

    // MultiNozzleUtils.hpp:236  const std::unordered_map<int, int>& get_extruder_nozzle_map() const
    pub fn get_extruder_nozzle_map(&self) -> &UnorderedMapInt<i32> {
        &self.extruder_nozzle_status
    }
}

impl Default for NozzleStatusRecorder {
    fn default() -> Self {
        NozzleStatusRecorder::new()
    }
}

/// MultiNozzleUtils.cpp:1152-1176  resolve_ams_load_unload: the load and unload
/// times of AMS type `ams_type` from the tables, else the legacy machine times.
/// Returns `(found, out_load, out_unload)`.
pub fn resolve_ams_load_unload(
    loads: &[f64],
    unloads: &[f64],
    ams_type: i32,
    legacy_load: f32,
    legacy_unload: f32,
) -> (bool, f32, f32) {
    let (out_load, out_unload) = (legacy_load, legacy_unload);
    if ams_type < 0 {
        return (false, out_load, out_unload);
    }
    let idx = ams_type as usize;
    if idx >= loads.len() || idx >= unloads.len() {
        // :1167-1170 — warning, legacy times kept.
        return (false, out_load, out_unload);
    }
    (true, loads[idx] as f32, unloads[idx] as f32)
}

// MultiNozzleUtils.cpp:927  }} // namespace Slic3r::MultiNozzleUtils
