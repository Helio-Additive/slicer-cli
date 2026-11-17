//! Port of `FilamentGroupUtils.cpp` / `FilamentGroupUtils.hpp` at the pin.
//!
//! C++ Reference:
//! - src/libslic3r/FilamentGroupUtils.cpp
//! - src/libslic3r/FilamentGroupUtils.hpp
//!
//! Faithful 1:1 line-by-line translation, in the order of the .cpp. `coord_t` -> `i64`,
//! `coordf_t` -> `f64`. `DynamicPrintConfig` is `crate::print_config::config_def::DynamicPrintConfig`,
//! read through `ConfigOptionResolver::option` as `item.option<ConfigOptionXxx>(key)` reads it.
//!
//! The two overloads of `extract_unprintable_limit_indices` (std::map<int,int> and
//! std::unordered_map<int,std::vector<int>> output) are one generic function over the output type
//! ([`UnprintableLimitIndices`]). `MultiNozzleUtils::LayeredNozzleGroupResult::get_used_extruders(layer_id)`
//! and `get_used_nozzles_in_extruder(ext_id, layer_id)` are the Rust `get_used_extruders_layer` /
//! `get_used_nozzles_in_extruder_layer`.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::fmt;

// FilamentGroupUtils.hpp:10  #include "MultiNozzleUtils.hpp"
use crate::multi_nozzle_utils::{LayeredNozzleGroupResult, NozzleInfo};

// FilamentGroupUtils.hpp:36 (FilamentInfo::usage_type) — the enum of PrintConfig.hpp:45-49, mirrored
// here (same variants, same order/discriminants).
/// C++ `enum FilamentUsageType` (PrintConfig.hpp:45-49).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum FilamentUsageType {
    // PrintConfig.hpp:46
    #[default]
    SupportOnly,
    // PrintConfig.hpp:47
    ModelOnly,
    // PrintConfig.hpp:48
    Hybrid,
}

// FilamentGroupUtils.hpp:18-29
/// C++ `struct Color`.
///
/// Default field values mirror the C++ in-class initializers (`r=g=b=0`, `a=255`).
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
        Color { r: 0, g: 0, b: 0, a: 255 }
    }
}

/// `std::istringstream(hex) >> std::hex >> byte` of an `unsigned int` (FilamentGroupUtils.cpp:20-21):
/// leading white space skipped, an optional sign, then the longest run of hexadecimal digits. On a
/// failed extraction the C++ `byte` is left uninitialised.
///
/// FIDELITY-NOTE: the uninitialised value of a failed extraction is undefined behaviour in C++; it
/// is 0 here. A run past `unsigned int` sets failbit and stores UINT_MAX (num_get), kept here.
fn istream_hex_to_uint(hex: &[u8]) -> u32 {
    let mut i = 0usize;
    while i < hex.len() && (hex[i] == b' ' || (b'\t'..=b'\r').contains(&hex[i])) {
        i += 1;
    }
    let mut negative = false;
    if i < hex.len() && (hex[i] == b'+' || hex[i] == b'-') {
        negative = hex[i] == b'-';
        i += 1;
    }
    let start = i;
    let mut value: u64 = 0;
    let mut overflow = false;
    while i < hex.len() && hex[i].is_ascii_hexdigit() {
        let d = (hex[i] as char).to_digit(16).unwrap() as u64;
        value = value * 16 + d;
        if value > u32::MAX as u64 {
            overflow = true;
        }
        i += 1;
    }
    if i == start {
        return 0;
    }
    if overflow {
        return u32::MAX;
    }
    if negative {
        (value as u32).wrapping_neg()
    } else {
        value as u32
    }
}

impl Color {
    // FilamentGroupUtils.hpp:24
    /// `Color(unsigned char r_ = 0, unsigned char g_ = 0, unsigned char b_ = 0, unsigned a_ = 255)`
    pub fn new(r: u8, g: u8, b: u8, a: u8) -> Self {
        Color { r, g, b, a }
    }

    /// FilamentGroupUtils.cpp:10-29 `Color::Color(const std::string& hexstr)`
    pub fn from_hex_str(hexstr: &str) -> Self {
        let bytes = hexstr.as_bytes();
        // FilamentGroupUtils.cpp:11-16
        if bytes.is_empty() || (bytes.len() != 9 && bytes.len() != 7) || bytes[0] != b'#' {
            // assert(false);
            return Color { r: 0, g: 0, b: 0, a: 255 };
        }

        // FilamentGroupUtils.cpp:18-23  auto hexToByte = [](const std::string& hex)->unsigned char
        let hex_to_byte = |hex: &[u8]| -> u8 { istream_hex_to_uint(hex) as u8 };
        let mut c = Color::default();
        // FilamentGroupUtils.cpp:24-26
        c.r = hex_to_byte(&bytes[1..3]);
        c.g = hex_to_byte(&bytes[3..5]);
        c.b = hex_to_byte(&bytes[5..7]);
        // FilamentGroupUtils.cpp:27-28
        if bytes.len() == 9 {
            c.a = hex_to_byte(&bytes[7..9]);
        }
        c
    }

    /// FilamentGroupUtils.cpp:50-62 `std::string Color::to_hex_str(bool include_alpha = false) const`
    pub fn to_hex_str(&self, include_alpha: bool) -> String {
        // FilamentGroupUtils.cpp:51-55
        // oss << "#" << std::hex << std::setfill('0')
        //     << std::setw(2) << static_cast<int>(r) << std::setw(2) << static_cast<int>(g)
        //     << std::setw(2) << static_cast<int>(b);
        let mut oss = format!("#{:02x}{:02x}{:02x}", self.r as i32, self.g as i32, self.b as i32);
        // FilamentGroupUtils.cpp:57-59
        if include_alpha {
            oss.push_str(&format!("{:02x}", self.a as i32));
        }
        // FilamentGroupUtils.cpp:60
        oss
    }
}

/// FilamentGroupUtils.cpp:39-43 `bool Color::operator==(const Color& other) const` (and the `!=` of
/// :45-48, its negation).
impl PartialEq for Color {
    fn eq(&self, other: &Self) -> bool {
        self.r == other.r && self.g == other.g && self.b == other.b && self.a == other.a
    }
}
impl Eq for Color {}

/// FilamentGroupUtils.cpp:31-37 `bool Color::operator<(const Color& other) const`
impl Ord for Color {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        if self.r != other.r {
            return self.r.cmp(&other.r);
        }
        if self.g != other.g {
            return self.g.cmp(&other.g);
        }
        if self.b != other.b {
            return self.b.cmp(&other.b);
        }
        self.a.cmp(&other.a)
    }
}

impl PartialOrd for Color {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

// FilamentGroupUtils.hpp:32-38
/// C++ `struct FilamentInfo`.
///
/// FIDELITY-NOTE: `is_support` and `usage_type` have no initializer in C++ (indeterminate in a
/// default-initialised object); `Default` gives them the value-initialised `false` / `SupportOnly`.
#[derive(Debug, Clone, Default)]
pub struct FilamentInfo {
    // FilamentGroupUtils.hpp:33
    pub color: Color,
    // FilamentGroupUtils.hpp:34
    pub type_: String,
    // FilamentGroupUtils.hpp:35
    pub is_support: bool,
    // FilamentGroupUtils.hpp:36
    pub usage_type: FilamentUsageType,
    // FilamentGroupUtils.hpp:37  float diameter{0.f};
    pub diameter: f32,
}

// FilamentGroupUtils.hpp:40-45
/// C++ `struct MachineFilamentInfo : public FilamentInfo`.
///
/// The base members are flattened into the derived struct (Rust has no struct inheritance).
#[derive(Debug, Clone, Default)]
pub struct MachineFilamentInfo {
    // --- inherited from FilamentInfo (FilamentGroupUtils.hpp:32-38) ---
    pub color: Color,
    pub type_: String,
    pub is_support: bool,
    pub usage_type: FilamentUsageType,
    pub diameter: f32,
    // --- MachineFilamentInfo own members ---
    // FilamentGroupUtils.hpp:41
    pub extruder_id: i32,
    // FilamentGroupUtils.hpp:42
    pub is_extended: bool,
    // FilamentGroupUtils.hpp:43  材料类 ID（tray_info_idx），可区分同 filament_type 下的耗材小类
    pub filament_id: String,
}

impl MachineFilamentInfo {
    /// FilamentGroupUtils.cpp:64-69 `bool MachineFilamentInfo::operator<(const MachineFilamentInfo& other) const`
    /// (FilamentGroupUtils.hpp:44: filament_id is not compared, so that same-type same-colour
    /// filaments of different sub-types fall into one candidate set).
    pub fn lt(&self, other: &Self) -> bool {
        if self.color != other.color {
            return self.color < other.color;
        }
        if self.type_ != other.type_ {
            return self.type_ < other.type_;
        }
        // return is_support < other.is_support;
        !self.is_support && other.is_support
    }
}

/// `std::map<MachineFilamentInfo, ...>` orders its keys by `MachineFilamentInfo::operator<`
/// (FilamentGroup.cpp:1211): equal keys are those neither of which is less.
impl PartialEq for MachineFilamentInfo {
    fn eq(&self, other: &Self) -> bool {
        !self.lt(other) && !other.lt(self)
    }
}
impl Eq for MachineFilamentInfo {}
impl Ord for MachineFilamentInfo {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        if self.lt(other) {
            std::cmp::Ordering::Less
        } else if other.lt(self) {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Equal
        }
    }
}
impl PartialOrd for MachineFilamentInfo {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

// FilamentGroupUtils.hpp:47-66
/// C++ `struct NozzleLoadedFilamentInfo : public FilamentInfo`; the base is `base`.
#[derive(Debug, Clone)]
pub struct NozzleLoadedFilamentInfo {
    pub base: FilamentInfo,
    // FilamentGroupUtils.hpp:49  int nozzle_id = -1;
    pub nozzle_id: i32,
    // FilamentGroupUtils.hpp:50
    pub filament_id: String,
    // FilamentGroupUtils.hpp:51  bool valid = false;
    pub valid: bool,
}

impl Default for NozzleLoadedFilamentInfo {
    fn default() -> Self {
        NozzleLoadedFilamentInfo { base: FilamentInfo::default(), nozzle_id: -1, filament_id: String::new(), valid: false }
    }
}

/// FilamentGroupUtils.hpp:53-64 `bool operator==(const NozzleLoadedFilamentInfo& other) const`
impl PartialEq for NozzleLoadedFilamentInfo {
    fn eq(&self, other: &Self) -> bool {
        self.nozzle_id == other.nozzle_id
            && self.filament_id == other.filament_id
            && self.valid == other.valid
            && self.base.color == other.base.color
            && self.base.type_ == other.base.type_
            && self.base.is_support == other.base.is_support
            && self.base.usage_type == other.base.usage_type
            && self.base.diameter == other.base.diameter
    }
}

// FilamentGroupUtils.hpp:66-92
/// C++ `class FilamentGroupException : public std::exception`.
#[derive(Debug, Clone)]
pub struct FilamentGroupException {
    // FilamentGroupUtils.hpp:75
    code: ErrorCode,
    // FilamentGroupUtils.hpp:76
    message: String,
}

// FilamentGroupUtils.hpp:68-72
/// C++ `enum ErrorCode` nested in `FilamentGroupException`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorCode {
    EmptyAmsFilaments,
    ConflictLimits,
    Unknown,
}

impl FilamentGroupException {
    /// FilamentGroupUtils.hpp:79-80 `FilamentGroupException(ErrorCode code, const std::string& message)`
    pub fn new(code: ErrorCode, message: String) -> Self {
        FilamentGroupException { code, message }
    }

    /// FilamentGroupUtils.hpp:82-84 `ErrorCode code() const noexcept`
    pub fn code(&self) -> ErrorCode {
        self.code
    }

    /// FilamentGroupUtils.hpp:86-88 `const char* what() const noexcept override`
    pub fn what(&self) -> &str {
        &self.message
    }
}

impl fmt::Display for FilamentGroupException {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

impl std::error::Error for FilamentGroupException {}

/// FilamentGroupUtils.cpp:73-89 `std::vector<int> calc_max_group_size(const std::vector<std::map<int, int>>& ams_counts, bool ignore_ext_filament)`
pub fn calc_max_group_size(ams_counts: &[BTreeMap<i32, i32>], ignore_ext_filament: bool) -> Vec<i32> {
    // FilamentGroupUtils.cpp:74
    let mut group_size: Vec<i32> = vec![0; 2];
    // FilamentGroupUtils.cpp:75-81
    for idx in 0..ams_counts.len() {
        let ams_count = &ams_counts[idx];
        for (key, value) in ams_count.iter() {
            group_size[idx] += key * value;
        }
    }

    // FilamentGroupUtils.cpp:83-86
    for idx in 0..group_size.len() {
        if !ignore_ext_filament && group_size[idx] == 0 {
            group_size[idx] = 1;
        }
    }
    // FilamentGroupUtils.cpp:87
    group_size
}

/// `item.option<ConfigOptionStrings>(key)` then `get_at(0)`.
///
/// FIDELITY-NOTE: `get_at(0)` of an empty vector is undefined behaviour in C++ (`front()` of an
/// empty vector); it is the empty text here, which the caller skips.
fn option_string_at0(item: &crate::print_config::config_def::DynamicPrintConfig, key: &str) -> Option<String> {
    use crate::config::ConfigOptionResolver;
    let opt = item.option(key)?;
    let o = opt.as_any().downcast_ref::<crate::config::ConfigOptionStrings>()?;
    Some(o.values.first().cloned().unwrap_or_default())
}

/// The ECMAScript `\w` class of std::regex: `[A-Za-z0-9_]`.
fn is_regex_word(c: u8) -> bool {
    c.is_ascii_alphanumeric() || c == b'_'
}

/// FilamentGroupUtils.cpp:91-144 `static std::vector<std::vector<MachineFilamentInfo>> build_full_machine_filaments(const std::vector<std::vector<DynamicPrintConfig>>& filament_configs)`
fn build_full_machine_filaments(
    filament_configs: &[Vec<crate::print_config::config_def::DynamicPrintConfig>],
) -> Vec<Vec<MachineFilamentInfo>> {
    use crate::config::ConfigOptionResolver;
    // FilamentGroupUtils.cpp:93-104  auto extract_filament_type = [](const std::string& s)->std::string
    //   std::regex r1(R"(^Sup.(\w+)$)"); std::regex r2(R"(^(\w+)-S$)"); regex_match -> m[1]
    let extract_filament_type = |s: &str| -> String {
        let b = s.as_bytes();
        // r1: "Sup", one character that is not a line terminator, then one or more \w to the end.
        if b.len() >= 5 && &b[0..3] == b"Sup" && b[3] != b'\n' && b[3] != b'\r' && b[4..].iter().all(|&c| is_regex_word(c)) {
            if let Ok(m) = std::str::from_utf8(&b[4..]) {
                return m.to_string();
            }
        }
        // r2: one or more \w, then "-S" at the end.
        if b.len() >= 3 && &b[b.len() - 2..] == b"-S" && b[..b.len() - 2].iter().all(|&c| is_regex_word(c)) {
            return s[..s.len() - 2].to_string();
        }
        s.to_string()
    };

    // FilamentGroupUtils.cpp:106
    let mut machine_filaments: Vec<Vec<MachineFilamentInfo>> = vec![Vec::new(); 2];
    // FilamentGroupUtils.cpp:107-142
    for idx in 0..filament_configs.len() {
        let arr = &filament_configs[idx];
        for item in arr {
            let mut temp = MachineFilamentInfo::default();
            let mut type_ = String::new();
            let mut color = String::new();
            let mut tray_name = String::new();
            let mut is_support_filament = false;

            if let Some(c) = option_string_at0(item, "filament_colour") {
                color = c;
            }
            if let Some(t) = option_string_at0(item, "filament_type") {
                type_ = t;
                type_ = extract_filament_type(&type_);
            }
            if let Some(t) = option_string_at0(item, "tray_name") {
                tray_name = t;
            }
            if let Some(opt) = item.option("filament_is_support") {
                if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionBools>() {
                    // FIDELITY-NOTE: get_at(0) of an empty vector is undefined in C++; false here.
                    is_support_filament = o.values.first().copied().unwrap_or(0) != 0;
                }
            }
            if let Some(opt) = item.option("filament_id") {
                if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionStrings>() {
                    if !o.values.is_empty() {
                        temp.filament_id = o.values[0].clone();
                    }
                }
            }

            // FilamentGroupUtils.cpp:132-133
            if color.is_empty() || type_.is_empty() || tray_name.is_empty() {
                continue;
            }

            // FilamentGroupUtils.cpp:135-140
            temp.color = Color::from_hex_str(&color);
            temp.type_ = type_;
            temp.extruder_id = idx as i32;
            temp.is_extended = tray_name == "Ext"; // hard-coded ext flag
            temp.is_support = is_support_filament;
            machine_filaments[idx].push(temp);
        }
    }
    // FilamentGroupUtils.cpp:143
    machine_filaments
}

/// FilamentGroupUtils.cpp:146-180 `std::vector<std::vector<MachineFilamentInfo>> build_machine_filaments(const std::vector<std::vector<DynamicPrintConfig>>& filament_configs, const std::vector<std::map<int, int>>& ams_counts, bool ignore_ext_filament)`
pub fn build_machine_filaments(
    filament_configs: &[Vec<crate::print_config::config_def::DynamicPrintConfig>],
    ams_counts: &[BTreeMap<i32, i32>],
    ignore_ext_filament: bool,
) -> Vec<Vec<MachineFilamentInfo>> {
    // FilamentGroupUtils.cpp:148-150
    let mut ret: Vec<Vec<MachineFilamentInfo>> = vec![Vec::new(); 2];
    let mut ams_size: Vec<i32> = vec![0; 2];
    let full_machine_filaments = build_full_machine_filaments(filament_configs);
    // FilamentGroupUtils.cpp:151  assert(full_machine_filaments.size() == 2);
    // FilamentGroupUtils.cpp:152-157
    for idx in 0..ams_counts.len().min(ams_size.len()) {
        let ams_count = &ams_counts[idx];
        for (key, value) in ams_count.iter() {
            ams_size[idx] += key * value;
        }
    }

    // FilamentGroupUtils.cpp:159-178
    for idx in 0..ams_size.len().min(full_machine_filaments.len()) {
        let mut tmp: Vec<MachineFilamentInfo> = Vec::new();
        for j in 0..full_machine_filaments[idx].len() {
            let machine_filament = &full_machine_filaments[idx][j];
            if !machine_filament.is_extended {
                tmp.push(machine_filament.clone());
            }
        }

        if tmp.is_empty() && !ignore_ext_filament {
            for j in 0..full_machine_filaments[idx].len() {
                let machine_filament = &full_machine_filaments[idx][j];
                if machine_filament.is_extended {
                    tmp.push(machine_filament.clone());
                }
            }
        }

        ret[idx] = tmp;
    }
    // FilamentGroupUtils.cpp:179
    ret
}

/// FilamentGroupUtils.cpp:182-211 `bool collect_unprintable_limits(const std::vector<std::set<int>>& physical_unprintables, const std::vector<std::set<int>>& geometric_unprintables, std::vector<std::set<int>>& unprintable_limits)`
pub fn collect_unprintable_limits(
    physical_unprintables: &[BTreeSet<i32>],
    geometric_unprintables: &[BTreeSet<i32>],
    unprintable_limits: &mut Vec<BTreeSet<i32>>,
) -> bool {
    // FilamentGroupUtils.cpp:184-189
    unprintable_limits.clear();
    unprintable_limits.resize(2, BTreeSet::new());
    let mut resized_physical_unprintables: Vec<BTreeSet<i32>> = physical_unprintables.to_vec();
    resized_physical_unprintables.resize(2, BTreeSet::new());
    let mut resized_geometric_unprintables: Vec<BTreeSet<i32>> = geometric_unprintables.to_vec();
    resized_geometric_unprintables.resize(2, BTreeSet::new());

    // FilamentGroupUtils.cpp:191-193
    let mut conflict = false;
    {
        let (left, right) = resized_physical_unprintables.split_at_mut(1);
        conflict |= remove_intersection(&mut left[0], &mut right[0]);
    }
    {
        let (left, right) = resized_geometric_unprintables.split_at_mut(1);
        conflict |= remove_intersection(&mut left[0], &mut right[0]);
    }

    // FilamentGroupUtils.cpp:195-205
    let mut filament_unprintable_exts: BTreeMap<i32, i32> = BTreeMap::new();
    for ext_unprintables in [&resized_physical_unprintables, &resized_geometric_unprintables] {
        for eid in 0..ext_unprintables.len() {
            for &fid in &ext_unprintables[eid] {
                match filament_unprintable_exts.get(&fid) {
                    Some(&e) if e != eid as i32 => conflict = true,
                    _ => {
                        filament_unprintable_exts.insert(fid, eid as i32);
                    }
                }
            }
        }
    }
    // FilamentGroupUtils.cpp:206-207
    for (key, value) in filament_unprintable_exts.iter() {
        unprintable_limits[*value as usize].insert(*key);
    }

    // FilamentGroupUtils.cpp:209
    !conflict
}

/// FilamentGroupUtils.cpp:213-222 `bool remove_intersection(std::set<int>& a, std::set<int>& b)`
pub fn remove_intersection(a: &mut BTreeSet<i32>, b: &mut BTreeSet<i32>) -> bool {
    // FilamentGroupUtils.cpp:214-215  std::set_intersection(..., std::back_inserter(intersection));
    let mut intersection: Vec<i32> = Vec::new();
    for item in a.intersection(b) {
        intersection.push(*item);
    }
    // FilamentGroupUtils.cpp:216
    let have_intersection = !intersection.is_empty();
    // FilamentGroupUtils.cpp:217-220
    for item in &intersection {
        a.remove(item);
        b.remove(item);
    }
    // FilamentGroupUtils.cpp:221
    have_intersection
}

/// FilamentGroupUtils.cpp:224-234 `void extract_indices(const std::vector<unsigned int>& used_filaments, const std::vector<std::set<int>>& unprintable_elems, std::vector<std::set<int>>& unprintable_idxs)`
pub fn extract_indices(used_filaments: &[u32], unprintable_elems: &[BTreeSet<i32>], unprintable_idxs: &mut Vec<BTreeSet<i32>>) {
    // FilamentGroupUtils.cpp:226  std::vector<std::set<int>>(unprintable_elems.size()).swap(unprintable_idxs);
    *unprintable_idxs = vec![BTreeSet::new(); unprintable_elems.len()];
    // FilamentGroupUtils.cpp:227-233
    for gid in 0..unprintable_elems.len() {
        for &f in &unprintable_elems[gid] {
            if let Some(pos) = used_filaments.iter().position(|&x| x == f as u32) {
                unprintable_idxs[gid].insert(pos as i32);
            }
        }
    }
}

/// The output of the two `extract_unprintable_limit_indices` overloads (FilamentGroupUtils.hpp:104,
/// 106): `std::map<int,int>` (the first extruder that cannot print the filament) and
/// `std::unordered_map<int,std::vector<int>>` (every such extruder).
pub trait UnprintableLimitIndices {
    /// The body of the overload after the shared preamble.
    fn extract_unprintable_limit_indices(unprintable_elems: &[BTreeSet<i32>], used_filaments: &[u32], unplaceable_limits: &mut Self);
}

impl UnprintableLimitIndices for BTreeMap<i32, i32> {
    /// FilamentGroupUtils.cpp:236-251
    fn extract_unprintable_limit_indices(unprintable_elems: &[BTreeSet<i32>], used_filaments: &[u32], unplaceable_limits: &mut Self) {
        // FilamentGroupUtils.cpp:238-243
        unplaceable_limits.clear();
        let mut unprintable_idxs: Vec<BTreeSet<i32>> = Vec::new();
        extract_indices(used_filaments, unprintable_elems, &mut unprintable_idxs);
        if unprintable_idxs.len() > 1 {
            let (left, right) = unprintable_idxs.split_at_mut(1);
            remove_intersection(&mut left[0], &mut right[0]);
        }

        // FilamentGroupUtils.cpp:245-249
        for idx in 0..unprintable_idxs.len() {
            for &f in &unprintable_idxs[idx] {
                if !unplaceable_limits.contains_key(&f) {
                    unplaceable_limits.insert(f, idx as i32);
                }
            }
        }
    }
}

impl UnprintableLimitIndices for HashMap<i32, Vec<i32>> {
    /// FilamentGroupUtils.cpp:253-269
    fn extract_unprintable_limit_indices(unprintable_elems: &[BTreeSet<i32>], used_filaments: &[u32], unplaceable_limits: &mut Self) {
        // FilamentGroupUtils.cpp:255-260
        unplaceable_limits.clear();
        let mut unprintable_idxs: Vec<BTreeSet<i32>> = Vec::new();
        extract_indices(used_filaments, unprintable_elems, &mut unprintable_idxs);
        if unprintable_idxs.len() > 1 {
            let (left, right) = unprintable_idxs.split_at_mut(1);
            remove_intersection(&mut left[0], &mut right[0]);
        }

        // FilamentGroupUtils.cpp:262-264
        for group_id in 0..unprintable_idxs.len() {
            for &f in &unprintable_idxs[group_id] {
                unplaceable_limits.entry(f).or_default().push(group_id as i32);
            }
        }

        // FilamentGroupUtils.cpp:266-267
        for (_key, value) in unplaceable_limits.iter_mut() {
            sort_remove_duplicates(value);
        }
    }
}

/// FilamentGroupUtils.cpp:236-251 and 253-269, the two overloads of
/// `void extract_unprintable_limit_indices(const std::vector<std::set<int>>& unprintable_elems, const std::vector<unsigned int>& used_filaments, <map>& unplaceable_limits)`,
/// chosen by the type of `unplaceable_limits` as the C++ overload resolution chooses them.
pub fn extract_unprintable_limit_indices<M: UnprintableLimitIndices>(
    unprintable_elems: &[BTreeSet<i32>],
    used_filaments: &[u32],
    unplaceable_limits: &mut M,
) {
    M::extract_unprintable_limit_indices(unprintable_elems, used_filaments, unplaceable_limits)
}

/// FilamentGroupUtils.cpp:271-280 `bool check_printable(const std::vector<std::set<int>>& groups, const std::map<int,int>& unprintable)`
pub fn check_printable(groups: &[BTreeSet<i32>], unprintable: &BTreeMap<i32, i32>) -> bool {
    for i in 0..groups.len() {
        let group = &groups[i];
        for filament in group {
            if let Some(&ext) = unprintable.get(filament) {
                if i as i32 == ext {
                    return false;
                }
            }
        }
    }
    true
}

/// FilamentGroupUtils.cpp:282-290 `int get_estimate_extruder_change_count(const std::vector<std::vector<unsigned int>> &layer_filaments, const MultiNozzleUtils::LayeredNozzleGroupResult &extruder_nozzle_info)`
pub fn get_estimate_extruder_change_count(layer_filaments: &[Vec<u32>], extruder_nozzle_info: &LayeredNozzleGroupResult) -> i32 {
    let mut ret = 0;
    for layer_id in 0..layer_filaments.len() {
        // int extruder_count = extruder_nozzle_info.get_used_extruders(layer_id).size();
        let extruder_count = extruder_nozzle_info.get_used_extruders_layer(layer_id as i32).len() as i32;
        ret += extruder_count - 1;
    }
    ret
}

/// FilamentGroupUtils.cpp:292-304 `int get_estimate_nozzle_change_count(const std::vector<std::vector<unsigned int>> &layer_filaments, const MultiNozzleUtils::LayeredNozzleGroupResult &extruder_nozzle_info)`
pub fn get_estimate_nozzle_change_count(layer_filaments: &[Vec<u32>], extruder_nozzle_info: &LayeredNozzleGroupResult) -> i32 {
    let mut ret = 0;
    for layer_id in 0..layer_filaments.len() {
        // auto& filament_list = layer_filaments[layer_id];  (unused)
        let _filament_list = &layer_filaments[layer_id];
        let extruder_list = extruder_nozzle_info.get_used_extruders_layer(layer_id as i32);
        for &extruder_id in &extruder_list {
            let nozzle_count = extruder_nozzle_info.get_used_nozzles_in_extruder_layer(extruder_id, layer_id as i32).len() as i32;
            if nozzle_count > 1 {
                ret += nozzle_count - 1;
            }
        }
    }
    ret
}

/// FilamentGroupUtils.cpp:306-321 `std::pair<int, int> get_estimate_extruder_filament_change_count(const MultiNozzleUtils::LayeredNozzleGroupResult &extruder_nozzle_info)`
pub fn get_estimate_extruder_filament_change_count(extruder_nozzle_info: &LayeredNozzleGroupResult) -> (i32, i32) {
    let mut ret: (i32, i32) = (0, 0);
    let layer_nums = extruder_nozzle_info.get_layer_filament_sequences().len() as i32;
    for layer_id in 0..layer_nums {
        let extruders = extruder_nozzle_info.get_used_extruders_layer(layer_id);
        // ret.first = extruders.size() - 1;  (size_t arithmetic narrowed to int)
        ret.0 = (extruders.len() as i32).wrapping_sub(1);

        for &ext_id in &extruders {
            let nozzles = extruder_nozzle_info.get_used_nozzles_in_extruder_layer(ext_id, layer_id).len() as i32;
            ret.1 += nozzles;
        }
        ret.1 = std::cmp::max(0, ret.1 - ret.0);
    }
    ret
}

/// FilamentGroupUtils.cpp:323-334 `std::map<int,std::vector<int>> build_extruder_nozzle_list(const std::vector<MultiNozzleUtils::NozzleInfo>& nozzle_list)`
pub fn build_extruder_nozzle_list(nozzle_list: &[NozzleInfo]) -> BTreeMap<i32, Vec<i32>> {
    let mut ret: BTreeMap<i32, Vec<i32>> = BTreeMap::new();
    for nozzle in nozzle_list {
        // FilamentGroupUtils.cpp:327  empty physical slot: never a grouping target
        if !nozzle.installed {
            continue;
        }
        ret.entry(nozzle.extruder_id).or_default().push(nozzle.group_id);
    }

    // FilamentGroupUtils.cpp:331-332  std::sort (ints: stable and unstable sorts agree)
    for (_key, value) in ret.iter_mut() {
        value.sort();
    }
    ret
}

/// FilamentGroupUtils.cpp:336-342 `std::vector<int> update_used_filament_values(const std::vector<int>& old_values, const std::vector<int>& new_values, const std::vector<unsigned int>& used_filaments)`
pub fn update_used_filament_values(old_values: &[i32], new_values: &[i32], used_filaments: &[u32]) -> Vec<i32> {
    let mut res: Vec<i32> = old_values.to_vec();
    for i in 0..used_filaments.len() {
        res[used_filaments[i] as usize] = new_values[used_filaments[i] as usize];
    }
    res
}

// libslic3r.h:204
/// `template <typename T> inline void sort_remove_duplicates(std::vector<T> &vec)`
fn sort_remove_duplicates<T: Ord>(vec: &mut Vec<T>) {
    vec.sort();
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
        // FilamentGroupUtils.cpp:323 — group nozzles by extruder_id, sorted.
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
        <LayeredNozzleGroupResult as crate::multi_nozzle_utils::CreateFromLayerMaps>::create(&layer_maps, &nozzle_list, &used, &sequences, false)
            .expect("create")
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
