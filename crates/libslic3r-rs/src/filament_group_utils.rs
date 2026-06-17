//! Port of `FilamentGroupUtils.cpp` / `FilamentGroupUtils.hpp`.
//!
//! C++ Reference:
//! - src/libslic3r/FilamentGroupUtils.cpp
//! - src/libslic3r/FilamentGroupUtils.hpp
//!
//! Faithful 1:1 line-by-line translation. `coord_t` -> `i64`, `coordf_t` -> `f64`.
//!
//! Blocked symbols (NOT ported, see notes at bottom of file):
//! - `build_full_machine_filaments` / `build_machine_filaments`: require the
//!   dynamic `DynamicPrintConfig::option::<ConfigOptionStrings>(key)` reflection
//!   API which is not yet ported to the Rust crate (the local `DynamicPrintConfig`
//!   types in calib.rs / format/*.rs are placeholders with no typed-option access).
//!
//! Previously blocked, now ported (2026-06-13): `get_estimate_extruder_change_count`,
//! `get_estimate_nozzle_change_count`, `get_estimate_extruder_filament_change_count`,
//! `build_extruder_nozzle_list` — `MultiNozzleUtils::LayeredNozzleGroupResult` and
//! `NozzleInfo` are now fully ported in `multi_nozzle_utils.rs`. The C++ overloaded
//! methods `get_used_extruders(layer_id)` and
//! `get_used_nozzles_in_extruder(ext_id, layer_id)` map to the Rust
//! `get_used_extruders_layer` / `get_used_nozzles_in_extruder_layer`.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::fmt;

// FilamentGroupUtils.hpp:10  #include "MultiNozzleUtils.hpp"
use crate::multi_nozzle_utils::{LayeredNozzleGroupResult, NozzleInfo};

// FilamentGroupUtils.hpp:36 (FilamentInfo::usage_type) — the enum lives in
// PrintConfig.hpp:40 and is not yet ported to the Rust print_config module, so
// it is mirrored here faithfully (same variants, same order/discriminants).
/// C++ `enum FilamentUsageType` (PrintConfig.hpp:40).
// PrintConfig.hpp:40
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum FilamentUsageType {
    // PrintConfig.hpp:41
    SupportOnly,
    // PrintConfig.hpp:42
    ModelOnly,
    // PrintConfig.hpp:43
    Hybrid,
}

// FilamentGroupUtils.hpp:18
/// C++ `struct Color`.
///
/// Default field values mirror the C++ in-class initializers
/// (`r=g=b=0`, `a=255`).
// FilamentGroupUtils.hpp:18
#[derive(Debug, Clone, Copy)]
pub struct Color {
    // FilamentGroupUtils.hpp:20
    pub r: u8,
    // FilamentGroupUtils.hpp:21
    pub g: u8,
    // FilamentGroupUtils.hpp:22
    pub b: u8,
    // FilamentGroupUtils.hpp:23
    pub a: u8,
}

impl Default for Color {
    // FilamentGroupUtils.hpp:24
    // Color(unsigned char r_ = 0, unsigned char g_ = 0, unsigned char b_ = 0, unsigned a_ = 255)
    fn default() -> Self {
        Color {
            r: 0,
            g: 0,
            b: 0,
            a: 255,
        }
    }
}

impl Color {
    // FilamentGroupUtils.hpp:24
    /// `Color(unsigned char r_ = 0, unsigned char g_ = 0, unsigned char b_ = 0, unsigned a_ = 255)`
    pub fn new(r: u8, g: u8, b: u8, a: u8) -> Self {
        Color { r, g, b, a }
    }

    // FilamentGroupUtils.cpp:10
    /// `Color::Color(const std::string& hexstr)`
    pub fn from_hex_str(hexstr: &str) -> Self {
        // FilamentGroupUtils.cpp:11
        if hexstr.is_empty()
            || (hexstr.len() != 9 && hexstr.len() != 7)
            || hexstr.as_bytes()[0] != b'#'
        {
            // FilamentGroupUtils.cpp:13
            debug_assert!(false);
            // FilamentGroupUtils.cpp:14
            return Color {
                r: 0,
                g: 0,
                b: 0,
                a: 255,
            };
            // FilamentGroupUtils.cpp:15
        }

        // FilamentGroupUtils.cpp:18
        // auto hexToByte = [](const std::string& hex)->unsigned char
        let hex_to_byte = |hex: &str| -> u8 {
            // FilamentGroupUtils.cpp:20-22
            // unsigned int byte;
            // std::istringstream(hex) >> std::hex >> byte;
            // return static_cast<unsigned char>(byte);
            //
            // `std::istringstream >> std::hex >> byte` parses leading hex digits
            // and leaves `byte` unmodified (== 0 since default-initialized to 0 in
            // practice here, though technically uninitialized in C++) on failure.
            // For the well-formed 2-char substrings produced below this always
            // parses successfully.
            u32::from_str_radix(hex.trim(), 16).unwrap_or(0) as u8
        };
        // FilamentGroupUtils.cpp:24
        let r = hex_to_byte(&hexstr[1..3]);
        // FilamentGroupUtils.cpp:25
        let g = hex_to_byte(&hexstr[3..5]);
        // FilamentGroupUtils.cpp:26
        let b = hex_to_byte(&hexstr[5..7]);
        // FilamentGroupUtils.cpp:27
        let a = if hexstr.len() == 9 {
            // FilamentGroupUtils.cpp:28
            hex_to_byte(&hexstr[7..9])
        } else {
            // C++ leaves `a` at its in-class default of 255 here.
            255
        };
        Color { r, g, b, a }
    }

    // FilamentGroupUtils.cpp:50
    /// `std::string Color::to_hex_str(bool include_alpha = false) const`
    pub fn to_hex_str(&self, include_alpha: bool) -> String {
        // FilamentGroupUtils.cpp:51-55
        // oss << "#" << std::hex << std::setfill('0')
        //     << std::setw(2) << r << std::setw(2) << g << std::setw(2) << b;
        let mut oss = format!("#{:02x}{:02x}{:02x}", self.r, self.g, self.b);
        // FilamentGroupUtils.cpp:57
        if include_alpha {
            // FilamentGroupUtils.cpp:58
            oss.push_str(&format!("{:02x}", self.a));
        }
        // FilamentGroupUtils.cpp:60
        oss
    }
}

// FilamentGroupUtils.cpp:39
/// `bool Color::operator==(const Color& other) const`
impl PartialEq for Color {
    fn eq(&self, other: &Self) -> bool {
        // FilamentGroupUtils.cpp:41
        self.r == other.r && self.g == other.g && self.b == other.b && self.a == other.a
    }
}
impl Eq for Color {}

// FilamentGroupUtils.cpp:31
/// `bool Color::operator<(const Color& other) const`
impl Ord for Color {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // FilamentGroupUtils.cpp:33
        if self.r != other.r {
            return self.r.cmp(&other.r);
        }
        // FilamentGroupUtils.cpp:34
        if self.g != other.g {
            return self.g.cmp(&other.g);
        }
        // FilamentGroupUtils.cpp:35
        if self.b != other.b {
            return self.b.cmp(&other.b);
        }
        // FilamentGroupUtils.cpp:36
        self.a.cmp(&other.a)
    }
}

impl PartialOrd for Color {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

// FilamentGroupUtils.hpp:32
/// C++ `struct FilamentInfo`.
// FilamentGroupUtils.hpp:32
#[derive(Debug, Clone)]
pub struct FilamentInfo {
    // FilamentGroupUtils.hpp:33
    pub color: Color,
    // FilamentGroupUtils.hpp:34
    pub type_: String,
    // FilamentGroupUtils.hpp:35
    pub is_support: bool,
    // FilamentGroupUtils.hpp:36
    pub usage_type: FilamentUsageType,
}

// FilamentGroupUtils.hpp:39
/// C++ `struct MachineFilamentInfo : public FilamentInfo`.
///
/// C++ uses public inheritance from `FilamentInfo`; here the base members are
/// flattened into the derived struct (Rust has no struct inheritance).
// FilamentGroupUtils.hpp:39
#[derive(Debug, Clone)]
pub struct MachineFilamentInfo {
    // --- inherited from FilamentInfo (FilamentGroupUtils.hpp:32) ---
    // FilamentGroupUtils.hpp:33
    pub color: Color,
    // FilamentGroupUtils.hpp:34
    pub type_: String,
    // FilamentGroupUtils.hpp:35
    pub is_support: bool,
    // FilamentGroupUtils.hpp:36
    pub usage_type: FilamentUsageType,
    // --- MachineFilamentInfo own members ---
    // FilamentGroupUtils.hpp:40
    pub extruder_id: i32,
    // FilamentGroupUtils.hpp:41
    pub is_extended: bool,
}

impl MachineFilamentInfo {
    // FilamentGroupUtils.cpp:64
    /// `bool MachineFilamentInfo::operator<(const MachineFilamentInfo& other) const`
    pub fn lt(&self, other: &Self) -> bool {
        // FilamentGroupUtils.cpp:66
        if self.color != other.color {
            return self.color < other.color;
        }
        // FilamentGroupUtils.cpp:67
        if self.type_ != other.type_ {
            return self.type_ < other.type_;
        }
        // FilamentGroupUtils.cpp:68  return is_support < other.is_support;
        // For C++ `bool < bool`, `a < b` is equivalent to `!a && b` (false < true).
        !self.is_support && other.is_support
    }
}

// FilamentGroupUtils.hpp:45
/// C++ `class FilamentGroupException : public std::exception`.
#[derive(Debug, Clone)]
pub struct FilamentGroupException {
    // FilamentGroupUtils.hpp:54
    code: ErrorCode,
    // FilamentGroupUtils.hpp:55
    message: String,
}

// FilamentGroupUtils.hpp:47
/// C++ `enum ErrorCode` (nested in `FilamentGroupException`).
// FilamentGroupUtils.hpp:47
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorCode {
    // FilamentGroupUtils.hpp:48
    EmptyAmsFilaments,
    // FilamentGroupUtils.hpp:49
    ConflictLimits,
    // FilamentGroupUtils.hpp:50
    Unknown,
}

impl FilamentGroupException {
    // FilamentGroupUtils.hpp:58
    /// `FilamentGroupException(ErrorCode code, const std::string& message)`
    pub fn new(code: ErrorCode, message: String) -> Self {
        // FilamentGroupUtils.hpp:59
        FilamentGroupException { code, message }
    }

    // FilamentGroupUtils.hpp:61
    /// `ErrorCode code() const noexcept`
    pub fn code(&self) -> ErrorCode {
        // FilamentGroupUtils.hpp:62
        self.code
    }

    // FilamentGroupUtils.hpp:65
    /// `const char* what() const noexcept override`
    pub fn what(&self) -> &str {
        // FilamentGroupUtils.hpp:66
        &self.message
    }
}

impl fmt::Display for FilamentGroupException {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

impl std::error::Error for FilamentGroupException {}

// FilamentGroupUtils.cpp:73
// TODO: add explanation
/// `std::vector<int> calc_max_group_size(const std::vector<std::map<int, int>>& ams_counts, bool ignore_ext_filament)`
pub fn calc_max_group_size(ams_counts: &[BTreeMap<i32, i32>], ignore_ext_filament: bool) -> Vec<i32> {
    // FilamentGroupUtils.cpp:74
    // add default value to 2
    // FilamentGroupUtils.cpp:75
    let mut group_size: Vec<i32> = vec![0; 2];
    // FilamentGroupUtils.cpp:76
    for idx in 0..ams_counts.len() {
        // FilamentGroupUtils.cpp:77
        let ams_count = &ams_counts[idx];
        // FilamentGroupUtils.cpp:78
        for (key, value) in ams_count.iter() {
            // FilamentGroupUtils.cpp:79
            group_size[idx] += key * value;
        }
    }

    // FilamentGroupUtils.cpp:83
    for idx in 0..group_size.len() {
        // FilamentGroupUtils.cpp:84
        if !ignore_ext_filament && group_size[idx] == 0 {
            // FilamentGroupUtils.cpp:85
            group_size[idx] = 1;
        }
    }
    // FilamentGroupUtils.cpp:87
    group_size
}

// FilamentGroupUtils.cpp:209
/// `bool remove_intersection(std::set<int>& a, std::set<int>& b)`
pub fn remove_intersection(a: &mut BTreeSet<i32>, b: &mut BTreeSet<i32>) -> bool {
    // FilamentGroupUtils.cpp:210-211
    // std::vector<int>intersection;
    // std::set_intersection(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(intersection));
    let intersection: Vec<i32> = a.intersection(b).cloned().collect();
    // FilamentGroupUtils.cpp:212
    let have_intersection = !intersection.is_empty();
    // FilamentGroupUtils.cpp:213
    for item in &intersection {
        // FilamentGroupUtils.cpp:214
        a.remove(item);
        // FilamentGroupUtils.cpp:215
        b.remove(item);
    }
    // FilamentGroupUtils.cpp:217
    have_intersection
}

// FilamentGroupUtils.cpp:178
/// `bool collect_unprintable_limits(const std::vector<std::set<int>>& physical_unprintables, const std::vector<std::set<int>>& geometric_unprintables, std::vector<std::set<int>>& unprintable_limits)`
pub fn collect_unprintable_limits(
    physical_unprintables: &[BTreeSet<i32>],
    geometric_unprintables: &[BTreeSet<i32>],
    unprintable_limits: &mut Vec<BTreeSet<i32>>,
) -> bool {
    // FilamentGroupUtils.cpp:180
    unprintable_limits.clear();
    // FilamentGroupUtils.cpp:181
    unprintable_limits.resize(2, BTreeSet::new());
    // FilamentGroupUtils.cpp:182
    // resize unprintables to 2
    // FilamentGroupUtils.cpp:183-184
    let mut resized_physical_unprintables: Vec<BTreeSet<i32>> = physical_unprintables.to_vec();
    resized_physical_unprintables.resize(2, BTreeSet::new());
    // FilamentGroupUtils.cpp:185-186
    let mut resized_geometric_unprintables: Vec<BTreeSet<i32>> = geometric_unprintables.to_vec();
    resized_geometric_unprintables.resize(2, BTreeSet::new());

    // FilamentGroupUtils.cpp:188
    let mut conflict = false;
    // FilamentGroupUtils.cpp:189
    // conflict |= remove_intersection(resized_physical_unprintables[0], resized_physical_unprintables[1]);
    {
        let (left, right) = resized_physical_unprintables.split_at_mut(1);
        conflict |= remove_intersection(&mut left[0], &mut right[0]);
    }
    // FilamentGroupUtils.cpp:190
    // conflict |= remove_intersection(resized_geometric_unprintables[0], resized_geometric_unprintables[1]);
    {
        let (left, right) = resized_geometric_unprintables.split_at_mut(1);
        conflict |= remove_intersection(&mut left[0], &mut right[0]);
    }

    // FilamentGroupUtils.cpp:192
    let mut filament_unprintable_exts: BTreeMap<i32, i32> = BTreeMap::new();
    // FilamentGroupUtils.cpp:193
    // for (auto& ext_unprintables : { resized_physical_unprintables, resized_geometric_unprintables })
    for ext_unprintables in [&resized_physical_unprintables, &resized_geometric_unprintables] {
        // FilamentGroupUtils.cpp:194
        for eid in 0..ext_unprintables.len() {
            // FilamentGroupUtils.cpp:195
            for &fid in &ext_unprintables[eid] {
                // FilamentGroupUtils.cpp:196
                // if (auto iter = filament_unprintable_exts.find(fid); iter != ...end() && iter->second != eid)
                if let Some(&existing) = filament_unprintable_exts.get(&fid) {
                    if existing != eid as i32 {
                        // FilamentGroupUtils.cpp:197
                        conflict = true;
                    } else {
                        // FilamentGroupUtils.cpp:199
                        filament_unprintable_exts.insert(fid, eid as i32);
                    }
                } else {
                    // FilamentGroupUtils.cpp:199
                    filament_unprintable_exts.insert(fid, eid as i32);
                }
            }
        }
    }
    // FilamentGroupUtils.cpp:203
    for (key, value) in filament_unprintable_exts.iter() {
        // FilamentGroupUtils.cpp:204
        unprintable_limits[*value as usize].insert(*key);
    }

    // FilamentGroupUtils.cpp:206
    !conflict
}

// FilamentGroupUtils.cpp:220
/// `void extract_indices(const std::vector<unsigned int>& used_filaments, const std::vector<std::set<int>>& unprintable_elems, std::vector<std::set<int>>& unprintable_idxs)`
pub fn extract_indices(
    used_filaments: &[u32],
    unprintable_elems: &[BTreeSet<i32>],
    unprintable_idxs: &mut Vec<BTreeSet<i32>>,
) {
    // FilamentGroupUtils.cpp:222
    // std::vector<std::set<int>>(unprintable_elems.size()).swap(unprintable_idxs);
    *unprintable_idxs = vec![BTreeSet::new(); unprintable_elems.len()];
    // FilamentGroupUtils.cpp:223
    for gid in 0..unprintable_elems.len() {
        // FilamentGroupUtils.cpp:224
        for &f in &unprintable_elems[gid] {
            // FilamentGroupUtils.cpp:225
            // auto iter = std::find(used_filaments.begin(), used_filaments.end(), (unsigned)f);
            if let Some(pos) = used_filaments.iter().position(|&x| x == f as u32) {
                // FilamentGroupUtils.cpp:226-227
                // if (iter != used_filaments.end())
                //     unprintable_idxs[gid].insert(iter - used_filaments.begin());
                unprintable_idxs[gid].insert(pos as i32);
            }
        }
    }
}

// FilamentGroupUtils.cpp:232
/// `void extract_unprintable_limit_indices(const std::vector<std::set<int>>& unprintable_elems, const std::vector<unsigned int>& used_filaments, std::map<int, int>& unplaceable_limits)`
///
/// (First overload — `std::map<int, int>` output.)
pub fn extract_unprintable_limit_indices_map(
    unprintable_elems: &[BTreeSet<i32>],
    used_filaments: &[u32],
    unplaceable_limits: &mut BTreeMap<i32, i32>,
) {
    // FilamentGroupUtils.cpp:234
    unplaceable_limits.clear();
    // FilamentGroupUtils.cpp:235
    // map the unprintable filaments to idx of used filaments , if not used ,just ignore
    // FilamentGroupUtils.cpp:236
    let mut unprintable_idxs: Vec<BTreeSet<i32>> = Vec::new();
    // FilamentGroupUtils.cpp:237
    extract_indices(used_filaments, unprintable_elems, &mut unprintable_idxs);
    // FilamentGroupUtils.cpp:238
    if unprintable_idxs.len() > 1 {
        // FilamentGroupUtils.cpp:239
        let (left, right) = unprintable_idxs.split_at_mut(1);
        remove_intersection(&mut left[0], &mut right[0]);
    }

    // FilamentGroupUtils.cpp:241
    for idx in 0..unprintable_idxs.len() {
        // FilamentGroupUtils.cpp:242
        for &f in &unprintable_idxs[idx] {
            // FilamentGroupUtils.cpp:243
            // if (unplaceable_limits.count(f) == 0)
            if !unplaceable_limits.contains_key(&f) {
                // FilamentGroupUtils.cpp:244
                unplaceable_limits.insert(f, idx as i32);
            }
        }
    }
}

// FilamentGroupUtils.cpp:249
/// `void extract_unprintable_limit_indices(const std::vector<std::set<int>>& unprintable_elems, const std::vector<unsigned int>& used_filaments, std::unordered_map<int, std::vector<int>>& unplaceable_limits)`
///
/// (Second overload — `std::unordered_map<int, std::vector<int>>` output.)
pub fn extract_unprintable_limit_indices_multimap(
    unprintable_elems: &[BTreeSet<i32>],
    used_filaments: &[u32],
    unplaceable_limits: &mut HashMap<i32, Vec<i32>>,
) {
    // FilamentGroupUtils.cpp:251
    unplaceable_limits.clear();
    // FilamentGroupUtils.cpp:252
    let mut unprintable_idxs: Vec<BTreeSet<i32>> = Vec::new();
    // FilamentGroupUtils.cpp:253
    // map the unprintable filaments to idx of used filaments , if not used ,just ignore
    // FilamentGroupUtils.cpp:254
    extract_indices(used_filaments, unprintable_elems, &mut unprintable_idxs);
    // FilamentGroupUtils.cpp:255
    // remove elems that cannot be printed in both extruder
    // FilamentGroupUtils.cpp:256
    if unprintable_idxs.len() > 1 {
        // FilamentGroupUtils.cpp:257
        let (left, right) = unprintable_idxs.split_at_mut(1);
        remove_intersection(&mut left[0], &mut right[0]);
    }

    // FilamentGroupUtils.cpp:259
    for group_id in 0..unprintable_idxs.len() {
        // FilamentGroupUtils.cpp:260
        for &f in &unprintable_idxs[group_id] {
            // FilamentGroupUtils.cpp:261
            unplaceable_limits.entry(f).or_default().push(group_id as i32);
        }
    }

    // FilamentGroupUtils.cpp:263
    for (_key, value) in unplaceable_limits.iter_mut() {
        // FilamentGroupUtils.cpp:264
        sort_remove_duplicates(value);
    }
}

// FilamentGroupUtils.cpp:267
/// `bool check_printable(const std::vector<std::set<int>>& groups, const std::map<int,int>& unprintable)`
pub fn check_printable(groups: &[BTreeSet<i32>], unprintable: &BTreeMap<i32, i32>) -> bool {
    // FilamentGroupUtils.cpp:269
    for i in 0..groups.len() {
        // FilamentGroupUtils.cpp:270
        let group = &groups[i];
        // FilamentGroupUtils.cpp:271
        for filament in group {
            // FilamentGroupUtils.cpp:272
            // if (auto iter = unprintable.find(filament); iter != ...end() && i == iter->second)
            if let Some(&ext) = unprintable.get(filament) {
                if i as i32 == ext {
                    // FilamentGroupUtils.cpp:273
                    return false;
                }
            }
        }
    }
    // FilamentGroupUtils.cpp:276
    true
}

// FilamentGroupUtils.cpp:278
/// `int get_estimate_extruder_change_count(const std::vector<std::vector<unsigned int>> &layer_filaments, const MultiNozzleUtils::LayeredNozzleGroupResult &extruder_nozzle_info)`
pub fn get_estimate_extruder_change_count(
    layer_filaments: &[Vec<u32>],
    extruder_nozzle_info: &LayeredNozzleGroupResult,
) -> i32 {
    // FilamentGroupUtils.cpp:280
    let mut ret = 0;
    // FilamentGroupUtils.cpp:281
    for layer_id in 0..layer_filaments.len() {
        // FilamentGroupUtils.cpp:282
        // int extruder_count = extruder_nozzle_info.get_used_extruders(layer_id).size();
        let extruder_count = extruder_nozzle_info.get_used_extruders_layer(layer_id as i32).len() as i32;
        // FilamentGroupUtils.cpp:283
        ret += extruder_count - 1;
    }
    // FilamentGroupUtils.cpp:285
    ret
}

// FilamentGroupUtils.cpp:288
/// `int get_estimate_nozzle_change_count(const std::vector<std::vector<unsigned int>> &layer_filaments, const MultiNozzleUtils::LayeredNozzleGroupResult &extruder_nozzle_info)`
pub fn get_estimate_nozzle_change_count(
    layer_filaments: &[Vec<u32>],
    extruder_nozzle_info: &LayeredNozzleGroupResult,
) -> i32 {
    // FilamentGroupUtils.cpp:290
    let mut ret = 0;
    // FilamentGroupUtils.cpp:291
    for layer_id in 0..layer_filaments.len() {
        // FilamentGroupUtils.cpp:292
        // auto& filament_list = layer_filaments[layer_id];  (unused below, kept for parity)
        let _filament_list = &layer_filaments[layer_id];
        // FilamentGroupUtils.cpp:293
        // auto  extruder_list = extruder_nozzle_info.get_used_extruders(layer_id);
        let extruder_list = extruder_nozzle_info.get_used_extruders_layer(layer_id as i32);
        // FilamentGroupUtils.cpp:294
        for &extruder_id in &extruder_list {
            // FilamentGroupUtils.cpp:295
            // int nozzle_count = extruder_nozzle_info.get_used_nozzles_in_extruder(extruder_id, layer_id).size();
            let nozzle_count = extruder_nozzle_info
                .get_used_nozzles_in_extruder_layer(extruder_id, layer_id as i32)
                .len() as i32;
            // FilamentGroupUtils.cpp:296
            if nozzle_count > 1 {
                ret += nozzle_count - 1;
            }
        }
    }
    // FilamentGroupUtils.cpp:299
    ret
}

// FilamentGroupUtils.cpp:302
/// `std::pair<int, int> get_estimate_extruder_filament_change_count(const MultiNozzleUtils::LayeredNozzleGroupResult &extruder_nozzle_info)`
pub fn get_estimate_extruder_filament_change_count(
    extruder_nozzle_info: &LayeredNozzleGroupResult,
) -> (i32, i32) {
    // FilamentGroupUtils.cpp:304
    let mut ret: (i32, i32) = (0, 0);
    // FilamentGroupUtils.cpp:305
    let layer_nums = extruder_nozzle_info.get_layer_filament_sequences().len() as i32;
    // FilamentGroupUtils.cpp:306
    for layer_id in 0..layer_nums {
        // FilamentGroupUtils.cpp:307
        // std::vector<int> extruders = extruder_nozzle_info.get_used_extruders(layer_id);
        let extruders = extruder_nozzle_info.get_used_extruders_layer(layer_id);
        // FilamentGroupUtils.cpp:308
        ret.0 = extruders.len() as i32 - 1;

        // FilamentGroupUtils.cpp:310
        for &ext_id in &extruders {
            // FilamentGroupUtils.cpp:311
            // int nozzles = extruder_nozzle_info.get_used_nozzles_in_extruder(ext_id, layer_id).size();
            let nozzles = extruder_nozzle_info
                .get_used_nozzles_in_extruder_layer(ext_id, layer_id)
                .len() as i32;
            // FilamentGroupUtils.cpp:312
            ret.1 += nozzles;
        }
        // FilamentGroupUtils.cpp:314
        ret.1 = std::cmp::max(0, ret.1 - ret.0);
    }
    // FilamentGroupUtils.cpp:316
    ret
}

// FilamentGroupUtils.cpp:319
/// `std::map<int,std::vector<int>> build_extruder_nozzle_list(const std::vector<MultiNozzleUtils::NozzleInfo>& nozzle_list)`
pub fn build_extruder_nozzle_list(nozzle_list: &[NozzleInfo]) -> BTreeMap<i32, Vec<i32>> {
    // FilamentGroupUtils.cpp:321
    let mut ret: BTreeMap<i32, Vec<i32>> = BTreeMap::new();
    // FilamentGroupUtils.cpp:322
    for nozzle in nozzle_list {
        // FilamentGroupUtils.cpp:323
        ret.entry(nozzle.extruder_id).or_default().push(nozzle.group_id);
    }

    // FilamentGroupUtils.cpp:326
    for (_key, value) in ret.iter_mut() {
        // FilamentGroupUtils.cpp:327
        value.sort();
    }
    // FilamentGroupUtils.cpp:328
    ret
}

// FilamentGroupUtils.cpp:331
/// `std::vector<int> update_used_filament_values(const std::vector<int>& old_values, const std::vector<int>& new_values, const std::vector<unsigned int>& used_filaments)`
pub fn update_used_filament_values(
    old_values: &[i32],
    new_values: &[i32],
    used_filaments: &[u32],
) -> Vec<i32> {
    // FilamentGroupUtils.cpp:333
    let mut res: Vec<i32> = old_values.to_vec();
    // FilamentGroupUtils.cpp:334
    for i in 0..used_filaments.len() {
        // FilamentGroupUtils.cpp:335
        res[used_filaments[i] as usize] = new_values[used_filaments[i] as usize];
    }
    // FilamentGroupUtils.cpp:337
    res
}

// libslic3r.h:204
/// `template <typename T> inline void sort_remove_duplicates(std::vector<T> &vec)`
///
/// Shared utility from `libslic3r.h`; used by `extract_unprintable_limit_indices`.
fn sort_remove_duplicates<T: Ord>(vec: &mut Vec<T>) {
    // libslic3r.h:207
    vec.sort();
    // libslic3r.h:208
    vec.dedup();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_color_from_hex_str() {
        // FilamentGroupUtils.cpp:10 — RGB only (length 7), alpha defaults to 255.
        let c = Color::from_hex_str("#ff8000");
        assert_eq!((c.r, c.g, c.b, c.a), (0xff, 0x80, 0x00, 255));
        // RGBA (length 9).
        let c2 = Color::from_hex_str("#0a0b0c0d");
        assert_eq!((c2.r, c2.g, c2.b, c2.a), (0x0a, 0x0b, 0x0c, 0x0d));
    }

    #[test]
    fn test_color_to_hex_str() {
        // FilamentGroupUtils.cpp:50
        let c = Color::new(0xff, 0x80, 0x00, 0x12);
        assert_eq!(c.to_hex_str(false), "#ff8000");
        assert_eq!(c.to_hex_str(true), "#ff800012");
    }

    #[test]
    fn test_color_ordering() {
        // FilamentGroupUtils.cpp:31
        let a = Color::new(1, 2, 3, 4);
        let b = Color::new(1, 2, 3, 5);
        assert!(a < b);
        assert!(a != b);
        assert_eq!(a, Color::new(1, 2, 3, 4));
    }

    #[test]
    fn test_calc_max_group_size() {
        // FilamentGroupUtils.cpp:73 — group_size[idx] += first*second.
        let mut m0 = BTreeMap::new();
        m0.insert(4, 1); // 4 slots
        let mut m1 = BTreeMap::new();
        m1.insert(1, 2); // 2 slots
        let counts = vec![m0, m1];
        assert_eq!(calc_max_group_size(&counts, false), vec![4, 2]);
        // empty -> bumped to 1 when not ignoring ext filament.
        assert_eq!(calc_max_group_size(&[], false), vec![1, 1]);
        assert_eq!(calc_max_group_size(&[], true), vec![0, 0]);
    }

    #[test]
    fn test_remove_intersection() {
        let mut a: BTreeSet<i32> = [1, 2, 3].into_iter().collect();
        let mut b: BTreeSet<i32> = [2, 3, 4].into_iter().collect();
        assert!(remove_intersection(&mut a, &mut b));
        assert_eq!(a, [1].into_iter().collect());
        assert_eq!(b, [4].into_iter().collect());
        let mut c: BTreeSet<i32> = [1].into_iter().collect();
        let mut d: BTreeSet<i32> = [2].into_iter().collect();
        assert!(!remove_intersection(&mut c, &mut d));
    }

    #[test]
    fn test_extract_indices() {
        // used_filaments maps filament-id -> position.
        let used = vec![10u32, 20, 30];
        let elems = vec![[20i32, 30].into_iter().collect::<BTreeSet<_>>()];
        let mut idxs = Vec::new();
        extract_indices(&used, &elems, &mut idxs);
        assert_eq!(idxs, vec![[1i32, 2].into_iter().collect::<BTreeSet<_>>()]);
    }

    #[test]
    fn test_check_printable() {
        let groups = vec![
            [0i32, 1].into_iter().collect::<BTreeSet<_>>(),
            [2i32].into_iter().collect::<BTreeSet<_>>(),
        ];
        let mut unprintable = BTreeMap::new();
        unprintable.insert(2, 1); // filament 2 unprintable on extruder 1
        assert!(!check_printable(&groups, &unprintable));
        unprintable.clear();
        unprintable.insert(2, 0); // filament 2 unprintable on extruder 0, but it's in group 1
        assert!(check_printable(&groups, &unprintable));
    }

    #[test]
    fn test_update_used_filament_values() {
        let old = vec![0, 0, 0, 0];
        let new = vec![5, 6, 7, 8];
        let used = vec![1u32, 3];
        assert_eq!(update_used_filament_values(&old, &new, &used), vec![0, 6, 0, 8]);
    }

    #[test]
    fn test_build_extruder_nozzle_list() {
        // FilamentGroupUtils.cpp:319 — group nozzles by extruder_id, sorted.
        let mk = |extruder_id: i32, group_id: i32| NozzleInfo {
            extruder_id,
            group_id,
            ..NozzleInfo::default()
        };
        // Out-of-order group ids on extruder 0 to exercise the per-extruder sort.
        let nozzles = vec![mk(0, 2), mk(1, 1), mk(0, 0)];
        let ret = build_extruder_nozzle_list(&nozzles);
        assert_eq!(ret.get(&0), Some(&vec![0, 2]));
        assert_eq!(ret.get(&1), Some(&vec![1]));
    }

    // Builds a layered result: 2 extruders, one nozzle each (group 0 on
    // extruder 0, group 1 on extruder 1); filament i -> nozzle i.
    fn make_layered() -> LayeredNozzleGroupResult {
        let n0 = NozzleInfo { extruder_id: 0, group_id: 0, ..NozzleInfo::default() };
        let n1 = NozzleInfo { extruder_id: 1, group_id: 1, ..NozzleInfo::default() };
        let nozzle_list = vec![n0, n1];
        // layer 0: only filament 0 (extruder 0); layer 1: filaments 0 and 1 (both extruders).
        let layer_maps = vec![vec![0, 1], vec![0, 1]];
        let sequences = vec![vec![0u32], vec![0u32, 1u32]];
        let used = vec![0u32, 1u32];
        LayeredNozzleGroupResult::create_layered(&layer_maps, &nozzle_list, &used, &sequences)
            .expect("create_layered")
    }

    #[test]
    fn test_get_estimate_extruder_change_count() {
        // FilamentGroupUtils.cpp:278 — sum over layers of (used_extruders - 1).
        // layer 0: 1 extruder -> 0; layer 1: 2 extruders -> 1; total = 1.
        let info = make_layered();
        let layer_filaments = vec![vec![0u32], vec![0u32, 1u32]];
        assert_eq!(get_estimate_extruder_change_count(&layer_filaments, &info), 1);
    }

    #[test]
    fn test_get_estimate_nozzle_change_count() {
        // FilamentGroupUtils.cpp:288 — each extruder has a single nozzle here,
        // so nozzle_count is never > 1; result is 0.
        let info = make_layered();
        let layer_filaments = vec![vec![0u32], vec![0u32, 1u32]];
        assert_eq!(get_estimate_nozzle_change_count(&layer_filaments, &info), 0);
    }

    #[test]
    fn test_get_estimate_extruder_filament_change_count() {
        // FilamentGroupUtils.cpp:302 — ret.first is ASSIGNED each layer (last wins);
        // layer 1 has 2 extruders so ret.first = 1. ret.second accumulates nozzles
        // then clamps to max(0, second - first) each layer.
        let info = make_layered();
        let (extruder_changes, _filament_changes) =
            get_estimate_extruder_filament_change_count(&info);
        // layer 1 is last: 2 used extruders -> ret.first = 1.
        assert_eq!(extruder_changes, 1);
    }
}
