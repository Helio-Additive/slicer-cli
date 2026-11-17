//! Port of `FilamentGroup.cpp` / `FilamentGroup.hpp` at the pin.
//!
//! C++ Reference:
//! - src/libslic3r/FilamentGroup.cpp
//! - src/libslic3r/FilamentGroup.hpp
//!
//! Faithful 1:1 line-by-line translation, in the order of the .cpp.
//!
//! Mapping notes:
//! - The C++ exceptions are `Result`s: `FilamentGroupUtils::FilamentGroupException`,
//!   `Slic3r::RuntimeError` and the `std::out_of_range` of `std::vector::at` / `std::map::at` are the
//!   variants of [`FilamentGroupError`]. `calc_filament_group` catches the first, as the C++ does.
//! - `int* cost = nullptr` is `Option<&mut i32>`.
//! - `std::priority_queue<MemoryedGroup, std::vector<MemoryedGroup>, std::greater<MemoryedGroup>>`
//!   is [`MemoryedGroupHeap`], which repeats libc++'s `push_heap` / `pop_heap` (Floyd's sift-down),
//!   so equal-cost groups leave the heap in the order the C++ pops them.
//! - `std::unordered_map<int, ...>` whose iteration order reaches a result (the filament print
//!   times summed by `TimeEvaluator`, the nozzle status, the unplaceable limits KMediods iterates)
//!   are `crate::multi_nozzle_utils::UnorderedMapInt`, which follows libc++. Maps only searched are
//!   `HashMap`.
//! - `std::mt19937` + `std::shuffle` are libc++'s (`crate::placeholder_parser::Mt19937` and
//!   [`libcxx_shuffle`]).
//! - `std::function<bool(int, std::vector<int>&)>` is [`CustomSeq`] (`Rc`, as a std::function is
//!   copied).
//! - FIDELITY-NOTE: `MultiNozzleUtils::NozzleInfo::volume_type` is `crate::extruder::NozzleVolumeType`,
//!   the enum of an earlier pin without nvtTPU285 (4) and nvtE3DHighFlow (5) (extruder.rs); a value
//!   cast from an integer collapses 4 and 5 onto nvtTPUHighFlow. Where the C++ compares an enum
//!   cast from an integer of the configuration with a nozzle's type, the integers are compared.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet, VecDeque};
use std::rc::Rc;

use crate::extruder::NozzleVolumeType;
use crate::filament_group_utils::{
    extract_unprintable_limit_indices, Color, ErrorCode, FilamentGroupException, FilamentInfo, FilamentUsageType,
    MachineFilamentInfo, NozzleLoadedFilamentInfo,
};
use crate::flush_vol_predictor::flush_predict::{calc_color_distance, RGBColor};
use crate::gcode::tool_order_utils::{
    reorder_filaments_for_multi_nozzle_extruder, FlushMatrix, GroupMinCostFlowSolver, MatchModeGroupSolver, MaxFlowSolver,
    MinFlushFlowSolver, INVALID_ID,
};
use crate::multi_nozzle_utils::{self as mnu, FilamentChangeTimeParams, LayeredNozzleGroupResult, NozzleStatusRecorder, UnorderedMapInt};

// FilamentGroup.hpp:14  const static int DEFAULT_CLUSTER_SIZE = 16;
pub const DEFAULT_CLUSTER_SIZE: i32 = 16;

// FilamentGroup.hpp:16  const static int ABSOLUTE_FLUSH_GAP_TOLERANCE = 10;
pub const ABSOLUTE_FLUSH_GAP_TOLERANCE: i32 = 10;

// FilamentGroup.cpp:15  static constexpr long long ENUM_THRESHOLD = 10000;
const ENUM_THRESHOLD: i64 = 10000;
// FilamentGroup.cpp:16  static constexpr long long ENUM_EARLY_EXIT = 10000000;
const ENUM_EARLY_EXIT: i64 = 10000000;
// FilamentGroup.cpp:17  constexpr uint32_t GOLDEN_RATIO_32 = 0x9e3779b9;
pub const GOLDEN_RATIO_32: u32 = 0x9e3779b9;

/// ToolOrderUtils.hpp:23 `constexpr int MCMF_MAX_EDGE_COST = 10000000;` (namespace MaxFlowGraph).
///
/// FIDELITY-NOTE: the `max_flow_graph` module of tool_order_utils.rs does not carry it.
const MCMF_MAX_EDGE_COST: i32 = 10000000;

/// FilamentGroup.hpp:23-26 `enum FGStrategy`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FGStrategy {
    #[default]
    BestCost,
    BestFit,
}

/// FilamentGroup.hpp:28-31 `enum FGMode`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FGMode {
    #[default]
    FlushMode,
    MatchMode,
}

/// What the C++ of this unit throws.
#[derive(Debug, Clone)]
pub enum FilamentGroupError {
    /// `FilamentGroupUtils::FilamentGroupException`
    Group(FilamentGroupException),
    /// `Slic3r::RuntimeError`
    Runtime(crate::exception::RuntimeError),
    /// `std::out_of_range` of `std::vector::at` / `std::map::at`, with libc++'s `what()`.
    OutOfRange(String),
}

impl std::fmt::Display for FilamentGroupError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FilamentGroupError::Group(e) => write!(f, "{}", e),
            FilamentGroupError::Runtime(e) => write!(f, "{}", e),
            FilamentGroupError::OutOfRange(s) => write!(f, "{}", s),
        }
    }
}

fn runtime_error(msg: &str) -> FilamentGroupError {
    FilamentGroupError::Runtime(crate::exception::RuntimeError::new(msg.to_string()))
}

/// `std::function<bool(int, std::vector<int>&)>`
pub type CustomSeq = Rc<dyn Fn(i32, &mut Vec<i32>) -> bool>;

/// FilamentGroup.hpp:35-52 `struct FilamentGroupUtils::FlushTimeMachine`
#[derive(Debug, Clone, Copy)]
pub struct FlushTimeMachine {
    start: std::time::Instant,
}

impl Default for FlushTimeMachine {
    fn default() -> Self {
        FlushTimeMachine { start: std::time::Instant::now() }
    }
}

impl FlushTimeMachine {
    /// FilamentGroup.hpp:40-43 `void time_machine_start()`
    pub fn time_machine_start(&mut self) {
        self.start = std::time::Instant::now();
    }

    /// FilamentGroup.hpp:45-50 `int time_machine_end()`: the milliseconds since the start.
    pub fn time_machine_end(&self) -> i32 {
        self.start.elapsed().as_millis() as i32
    }
}

/// FilamentGroup.hpp:54-64 `struct FilamentGroupUtils::MemoryedGroup`
#[derive(Debug, Clone, Default)]
pub struct MemoryedGroup {
    // FilamentGroup.hpp:61  double cost{ 0 };
    pub cost: f64,
    // FilamentGroup.hpp:62  int prefer_level{ 0 };
    pub prefer_level: i32,
    // FilamentGroup.hpp:63  std::vector<int>group;
    pub group: Vec<i32>,
}

impl MemoryedGroup {
    /// FilamentGroup.hpp:56 `MemoryedGroup(const std::vector<int>& group_, const double cost_, const int prefer_level_)`
    pub fn new(group: Vec<i32>, cost: f64, prefer_level: i32) -> Self {
        MemoryedGroup { cost, prefer_level, group }
    }

    /// FilamentGroup.hpp:57-59 `bool operator>(const MemoryedGroup& other) const`
    pub fn gt(&self, other: &MemoryedGroup) -> bool {
        self.prefer_level < other.prefer_level || (self.prefer_level == other.prefer_level && self.cost > other.cost)
    }
}

/// FilamentGroup.hpp:66 `using MemoryedGroupHeap = std::priority_queue<MemoryedGroup,
/// std::vector<MemoryedGroup>, std::greater<MemoryedGroup>>;`, with libc++'s heap algorithms
/// (`<__algorithm/push_heap.h>`, `pop_heap.h`, `sift_down.h`): `comp(a, b)` is `a > b`.
#[derive(Debug, Clone, Default)]
pub struct MemoryedGroupHeap {
    c: Vec<MemoryedGroup>,
}

impl MemoryedGroupHeap {
    pub fn new() -> Self {
        MemoryedGroupHeap { c: Vec::new() }
    }

    /// `std::greater<MemoryedGroup>`
    fn comp(a: &MemoryedGroup, b: &MemoryedGroup) -> bool {
        a.gt(b)
    }

    /// libc++ `std::__sift_up(first, first + len, comp, len)`; the moved hole of libc++ is the
    /// swaps here, which leave the same arrangement.
    fn sift_up(c: &mut [MemoryedGroup], len: usize) {
        if len > 1 {
            let mut child = len - 1;
            while child > 0 {
                let parent = (child - 1) / 2;
                if Self::comp(&c[parent], &c[child]) {
                    c.swap(parent, child);
                    child = parent;
                } else {
                    break;
                }
            }
        }
    }

    /// The protected container `c` of `std::priority_queue`, in its order.
    pub fn c(&self) -> &[MemoryedGroup] {
        &self.c
    }

    /// `bool empty() const`
    pub fn empty(&self) -> bool {
        self.c.is_empty()
    }

    /// `size_type size() const`
    pub fn size(&self) -> usize {
        self.c.len()
    }

    /// `const value_type& top() const`
    pub fn top(&self) -> &MemoryedGroup {
        &self.c[0]
    }

    /// `void push(const value_type& v)`: `c.push_back(v); std::push_heap(c.begin(), c.end(), comp);`
    pub fn push(&mut self, v: MemoryedGroup) {
        self.c.push(v);
        let len = self.c.len();
        Self::sift_up(&mut self.c, len);
    }

    /// `void pop()`: `std::pop_heap(c.begin(), c.end(), comp); c.pop_back();` — libc++'s
    /// `__pop_heap`: Floyd's sift-down of the hole to a leaf, then the last element into the hole
    /// and a sift-up.
    pub fn pop(&mut self) {
        let len = self.c.len();
        if len > 1 {
            // __floyd_sift_down: the top moves down to a leaf, the greater child up each step.
            let mut hole = 0usize;
            let mut child = 0usize;
            loop {
                let mut child_i = 2 * child + 1;
                child = 2 * child + 1;
                if child + 1 < len && Self::comp(&self.c[child_i], &self.c[child_i + 1]) {
                    child_i += 1;
                    child += 1;
                }
                self.c.swap(hole, child_i);
                hole = child_i;
                if child > (len - 2) / 2 {
                    break;
                }
            }
            let last = len - 1;
            if hole != last {
                self.c.swap(hole, last);
                Self::sift_up(&mut self.c[..hole + 1], hole + 1);
            }
        }
        self.c.pop();
    }
}

/// libc++ `uniform_int_distribution<ptrdiff_t>{0, b}(mt19937&)` with its `__independent_bits_engine`
/// (`b < 2^32`: one engine draw masked to the bit width of `b + 1`, redrawn while out of range).
///
/// FIDELITY-NOTE: the distributions are implementation-defined; this is libc++ (macOS).
fn libcxx_uniform_int_ptrdiff(b: i64, g: &mut crate::placeholder_parser::Mt19937) -> i64 {
    let rp: u64 = (b as u64).wrapping_add(1);
    if rp == 1 {
        return 0;
    }
    let dt: u32 = 64;
    let mut w = dt - rp.leading_zeros() - 1;
    if (rp & (u64::MAX >> (dt - w))) != 0 {
        w += 1;
    }
    let mask0: u64 = if w >= 32 { u32::MAX as u64 } else { (1u64 << w) - 1 };
    loop {
        let u = (g.next_u32() as u64) & mask0;
        if u < rp {
            return u as i64;
        }
    }
}

/// libc++ `std::shuffle(first, last, mt19937&)` (`<__algorithm/shuffle.h>`).
fn libcxx_shuffle(v: &mut [i32], g: &mut crate::placeholder_parser::Mt19937) {
    let mut d = v.len() as i64;
    if d > 1 {
        let last = v.len() - 1;
        d -= 1;
        let mut first = 0usize;
        while first < last {
            let i = libcxx_uniform_int_ptrdiff(d, g);
            if i != 0 {
                v.swap(first, first + i as usize);
            }
            first += 1;
            d -= 1;
        }
    }
}

/// FilamentGroup.hpp:73-81 `struct FilamentGroupContext::ModelInfo`
#[derive(Debug, Clone, Default)]
pub struct ModelInfo {
    pub flush_matrix: Vec<FlushMatrix>,
    pub layer_filaments: Vec<Vec<u32>>,
    pub filament_info: Vec<FilamentInfo>,
    pub filament_ids: Vec<String>,
    pub selected_nozzle_diameters: Vec<String>,
    pub unprintable_filaments: Vec<BTreeSet<i32>>,
    pub unprintable_volumes: BTreeMap<i32, BTreeSet<NozzleVolumeType>>,
}

/// FilamentGroup.hpp:83-91 `struct FilamentGroupContext::GroupInfo`
///
/// FIDELITY-NOTE: the members without initializer are indeterminate in a default-initialised C++
/// object; `Default` value-initialises them.
#[derive(Debug, Clone, Default)]
pub struct GroupInfo {
    pub total_filament_num: i32,
    pub max_gap_threshold: f64,
    pub mode: FGMode,
    pub strategy: FGStrategy,
    pub ignore_ext_filament: bool,
    // FilamentGroup.hpp:89  bool has_filament_switcher = false;
    pub has_filament_switcher: bool,
    pub filament_volume_map: Vec<i32>,
}

/// FilamentGroup.hpp:93-99 `struct FilamentGroupContext::MachineInfo`
#[derive(Debug, Clone, Default)]
pub struct MachineInfo {
    pub max_group_size: Vec<i32>,
    pub machine_filament_info: Vec<Vec<MachineFilamentInfo>>,
    pub nozzle_loaded_filaments: Vec<NozzleLoadedFilamentInfo>,
    pub prefer_non_model_filament: Vec<bool>,
    pub master_extruder_id: i32,
}

/// FilamentGroup.hpp:101-108 `struct FilamentGroupContext::SpeedInfo`
#[derive(Debug, Clone, Default)]
pub struct SpeedInfo {
    pub filament_print_time: UnorderedMapInt<UnorderedMapInt<f64>>,
    pub extruder_change_time: f64,
    pub filament_change_time: f64,
    pub group_with_time: bool,
    pub change_time_params: FilamentChangeTimeParams,
    pub ams_preload_enabled: Vec<bool>,
}

/// FilamentGroup.hpp:110-117 `struct FilamentGroupContext::NozzleInfo`
#[derive(Debug, Clone, Default)]
pub struct NozzleInfoStruct {
    pub extruder_nozzle_list: BTreeMap<i32, Vec<i32>>,
    pub nozzle_list: Vec<mnu::NozzleInfo>,
    pub nozzle_status: UnorderedMapInt<i32>,
    // Explicit per-nozzle filament capacity, indexed by nozzle group_id (== nozzle_list index).
    // Used by the single-extruder  model (AMS nozzle = AMS slots + native Ext, fixed nozzles = 1,
    // empty slots = 0). Empty -> fall back to the flush-based heuristic.
    pub per_nozzle_capacity: Vec<i32>,
}

/// FilamentGroup.hpp:71-122 `struct FilamentGroupContext`
#[derive(Debug, Clone, Default)]
pub struct FilamentGroupContext {
    pub model_info: ModelInfo,
    pub group_info: GroupInfo,
    pub machine_info: MachineInfo,
    pub speed_info: SpeedInfo,
    pub nozzle_info: NozzleInfoStruct,
}

/// FilamentGroup.cpp:19-38 `static int calc_match_mode_cost(float color_distance, const std::string& model_filament_id,
/// const std::string& machine_filament_id, int max_assignment_count)`
pub fn calc_match_mode_cost(color_distance: f32, model_filament_id: &str, machine_filament_id: &str, max_assignment_count: i32) -> i32 {
    // 1. 将颜色距离保留两位精度并转为整数，避免直接转换造成截断。
    const K_COLOR_DISTANCE_SCALE: i32 = 100;

    // 2. 一个方案最多有 N 次小类不匹配；使用 N+1 权重，保证颜色差异始终优先。
    let color_priority_weight = std::cmp::max(1, max_assignment_count) + 1;
    // static_cast<int>(std::lround(color_distance * kColorDistanceScale)): float * int is a float;
    // std::lround(float) rounds half away from zero.
    let quantized_color_distance = (color_distance * K_COLOR_DISTANCE_SCALE as f32).round() as i64 as i32;
    let weighted_color_cost = quantized_color_distance.wrapping_mul(color_priority_weight);

    // 3. filament_id 表示耗材小类：一致不加成本，不一致加 1，仅在同色时择优。
    let filament_id_match =
        !model_filament_id.is_empty() && !machine_filament_id.is_empty() && model_filament_id == machine_filament_id;
    let filament_id_mismatch_cost = if filament_id_match { 0 } else { 1 };

    // 4. 单对成本 = 颜色成本 × (N+1) + 小类不匹配成本，求解器再对全部匹配求和。
    weighted_color_cost.wrapping_add(filament_id_mismatch_cost)
}

/// FilamentGroup.cpp:40-48 `static bool filament_diameter_matches_nozzle_volume(const FilamentGroupUtils::FilamentInfo &filament,
/// NozzleVolumeType volume_type)`
pub fn filament_diameter_matches_nozzle_volume(filament: &FilamentInfo, volume_type: NozzleVolumeType) -> bool {
    if filament.diameter <= 0.0 {
        return true;
    }
    let uses_285_filament = filament.diameter > 2.5;
    // volume_type == NozzleVolumeType::nvtTPU285 (PrintConfig.hpp:387, value 4)
    let uses_285_path = volume_type as i32 == 4;
    uses_285_filament == uses_285_path
}

/// FilamentGroup.cpp:50-63 `static void change_memoryed_heaps_to_arrays(MemoryedGroupHeap& heap,const int total_filament_num,
/// const std::vector<unsigned int>& used_filaments, std::vector<std::vector<int>>& arrs)`: clear the array and heap,
/// save the groups in heap to the array
pub fn change_memoryed_heaps_to_arrays(heap: &mut MemoryedGroupHeap, total_filament_num: i32, used_filaments: &[u32], arrs: &mut Vec<Vec<i32>>) {
    // switch the label idx
    arrs.clear();
    while !heap.empty() {
        let top = heap.top().clone();
        heap.pop();
        let mut labels_tmp: Vec<i32> = vec![0; total_filament_num as usize];
        for idx in 0..top.group.len() {
            labels_tmp[used_filaments[idx] as usize] = top.group[idx];
        }
        arrs.push(labels_tmp);
    }
}

/// FilamentGroup.cpp:65-75 `static std::unordered_map<int, int> get_merged_filament_map(const std::unordered_map<int, std::vector<int>>& merged_filaments)`
fn get_merged_filament_map(merged_filaments: &HashMap<i32, Vec<i32>>) -> HashMap<i32, i32> {
    let mut filament_merge_map: HashMap<i32, i32> = HashMap::new();
    for (key, filaments) in merged_filaments.iter() {
        for &f in filaments.iter() {
            //traverse filaments in merged group
            filament_merge_map.insert(f, *key);
        }
    }
    filament_merge_map
}

/// FilamentGroup.cpp:77-107 `static uint64_t fnv_hash_nozzle(const std::string& diameter, int volume_type, int is_right_extruder,
/// int loaded_filament = -1, int capacity = -1)`: 喷嘴枚举去重使用的语义 hash
pub fn fnv_hash_nozzle(diameter: &str, volume_type: i32, is_right_extruder: i32, loaded_filament: i32, capacity: i32) -> u64 {
    const FNV_OFFSET_BASIS: u64 = 14695981039346656037;
    const FNV_PRIME: u64 = 1099511628211;
    const SALT_A: u64 = 0xA5A5A5A5A5A5A5A5;
    const SALT_B: u64 = 0x5A5A5A5A5A5A5A5A;
    const SALT_C: u64 = 0x3C3C3C3C3C3C3C3C;
    const SALT_D: u64 = 0x9696969696969696;

    let mut h: u64 = FNV_OFFSET_BASIS;
    for &c in diameter.as_bytes() {
        h ^= c as u64;
        h = h.wrapping_mul(FNV_PRIME);
    }
    h ^= (volume_type as i64 as u64).wrapping_add(SALT_A);
    h = h.wrapping_mul(FNV_PRIME);
    h ^= (is_right_extruder as i64 as u64).wrapping_add(SALT_B);
    h = h.wrapping_mul(FNV_PRIME);
    if loaded_filament >= 0 {
        h ^= (loaded_filament as i64 as u64).wrapping_add(SALT_C);
        h = h.wrapping_mul(FNV_PRIME);
    }
    if capacity >= 0 {
        h ^= (capacity as i64 as u64).wrapping_add(SALT_D);
        h = h.wrapping_mul(FNV_PRIME);
    }

    h
}

/// FilamentGroup.cpp:109-111 冲刷体积(mm^3)折算成时间等价分(秒)：体积→质量(密度/1000)→冲刷耗时(速度)→×2修正
const FLUSH_VOLUME_TO_SCORE: f64 = 1.26 /*密度g/cm^3*/ * 180.0 /*冲刷s/g*/ * 2.0 /*修正*/ / 1000.0; // == 0.4536

/// FilamentGroup.cpp:112-117 `static double evaluate_score(const double flush, const double time, const bool with_time = false)`
pub fn evaluate_score(flush: f64, time: f64, with_time: bool) -> f64 {
    if !with_time {
        return flush;
    }

    let flush_score = flush * FLUSH_VOLUME_TO_SCORE;
    flush_score + time
}

/// FilamentGroup.cpp:119-133 `static double calc_change_time_for_group(...)`
#[allow(clippy::too_many_arguments)]
pub fn calc_change_time_for_group(
    filament_change_seq: &[i32],
    nozzle_change_seq: &[i32],
    logical_filaments: &[i32],
    nozzle_list: &[mnu::NozzleInfo],
    time_params: &FilamentChangeTimeParams,
    ams_preload_enabled: &[bool],
    group_of_filament: &[i32],
) -> f64 {
    let r = mnu::simulate_filament_change_time(
        logical_filaments,
        nozzle_list,
        filament_change_seq,
        nozzle_change_seq,
        group_of_filament,
        time_params,
        ams_preload_enabled,
        false,
    );
    r.actual_time
}

/// FilamentGroup.cpp:135-218 `static double full_evaluate(const std::vector<unsigned int>& used_filaments,
/// const std::vector<int>& filament_nozzle_map, const FilamentGroupContext& ctx,
/// std::optional<std::function<bool(int, std::vector<int>&)>> get_custom_seq = std::nullopt, int* out_flush = nullptr)`
pub fn full_evaluate(
    used_filaments: &[u32],
    filament_nozzle_map: &[i32],
    ctx: &FilamentGroupContext,
    get_custom_seq: Option<&CustomSeq>,
    out_flush: Option<&mut i32>,
) -> f64 {
    let group_res = LayeredNozzleGroupResult::create(filament_nozzle_map, &ctx.nozzle_info.nozzle_list, used_filaments);
    let Some(group_res) = group_res else {
        if let Some(o) = out_flush {
            *o = 0;
        }
        return 0.0;
    };

    let mut initial_status = NozzleStatusRecorder::new();
    for (nozzle_id, &filament_id) in ctx.nozzle_info.nozzle_status.iter() {
        if filament_id >= 0 {
            let mut extruder_id = 0;
            for nozzle in &ctx.nozzle_info.nozzle_list {
                if nozzle.group_id == nozzle_id {
                    extruder_id = nozzle.extruder_id;
                    break;
                }
            }
            initial_status.set_nozzle_status(nozzle_id, filament_id, extruder_id);
        }
    }

    let mut filament_sequences: Vec<Vec<u32>> = Vec::new();
    // get_custom_seq ? *get_custom_seq : std::function<bool(int, std::vector<int>&)>{}
    let custom: Option<&dyn Fn(i32, &mut Vec<i32>) -> bool> = get_custom_seq.map(|f| &**f as &dyn Fn(i32, &mut Vec<i32>) -> bool);
    let flush = reorder_filaments_for_multi_nozzle_extruder(
        used_filaments,
        &group_res,
        &ctx.model_info.layer_filaments,
        &ctx.model_info.flush_matrix,
        custom,
        Some(&mut filament_sequences),
        &initial_status,
    );

    if let Some(o) = out_flush {
        *o = flush;
    }

    let mut change_time = 0.0f64;
    if !filament_sequences.is_empty() {
        let mut filament_change_seq: Vec<i32> = Vec::new();
        let mut nozzle_change_seq: Vec<i32> = Vec::new();
        let mut prev_fil = -1i32;
        let mut prev_nozzle = -1i32;
        for layer_seq in &filament_sequences {
            for &fil in layer_seq {
                let Some(nozzle_info) = group_res.get_first_nozzle_for_filament(fil as i32) else {
                    continue;
                };
                let nid = nozzle_info.group_id;
                if fil as i32 == prev_fil && nid == prev_nozzle {
                    continue;
                }
                filament_change_seq.push(fil as i32);
                nozzle_change_seq.push(nid);
                prev_fil = fil as i32;
                prev_nozzle = nid;
            }
        }

        let logical_filaments: Vec<i32> = used_filaments.iter().map(|&f| f as i32).collect();
        let mut group_of_filament: Vec<i32> = vec![0; used_filaments.len()];
        for fi in 0..used_filaments.len() {
            let nozzle_id = filament_nozzle_map[used_filaments[fi] as usize];
            if nozzle_id >= 0 && nozzle_id < ctx.nozzle_info.nozzle_list.len() as i32 {
                group_of_filament[fi] = ctx.nozzle_info.nozzle_list[nozzle_id as usize].extruder_id;
            }
        }
        change_time = calc_change_time_for_group(
            &filament_change_seq,
            &nozzle_change_seq,
            &logical_filaments,
            &ctx.nozzle_info.nozzle_list,
            &ctx.speed_info.change_time_params,
            &ctx.speed_info.ams_preload_enabled,
            &group_of_filament,
        );
    }

    let mut print_time = 0.0f64;
    if ctx.speed_info.group_with_time {
        let time_evaluator = TimeEvaluator::new(&ctx.speed_info);
        print_time = time_evaluator.get_estimated_time(filament_nozzle_map);
    }

    evaluate_score(flush as f64, change_time + print_time, true)
}

/// FilamentGroup.cpp:220-250 `static long long estimate_dedup_enum_count(int k, int n, const FilamentGroupContext& ctx)`
pub fn estimate_dedup_enum_count(k: i32, n: i32, ctx: &FilamentGroupContext) -> i64 {
    if n <= 0 || k <= 0 {
        return 0;
    }

    let mut total: i64 = 1;
    for _i in 0..n {
        total = total.wrapping_mul(k as i64);
        if total > ENUM_EARLY_EXIT {
            return total;
        }
    }

    if k <= 1 {
        return total;
    }

    let mut dedup_factor: i32 = 1;
    let mut nozzle_type_count: BTreeMap<u64, i32> = BTreeMap::new();
    for nozzle in &ctx.nozzle_info.nozzle_list {
        let loaded_filament = match ctx.nozzle_info.nozzle_status.get(nozzle.group_id) {
            Some(&v) => v,
            None => -1,
        };
        let nozzle_capacity = if nozzle.group_id < ctx.nozzle_info.per_nozzle_capacity.len() as i32 {
            ctx.nozzle_info.per_nozzle_capacity[nozzle.group_id as usize]
        } else {
            -1
        };
        let hash = fnv_hash_nozzle(&nozzle.diameter, nozzle.volume_type as i32, (nozzle.extruder_id > 0) as i32, loaded_filament, nozzle_capacity);
        *nozzle_type_count.entry(hash).or_insert(0) += 1;
    }

    for (_hash, &count) in nozzle_type_count.iter() {
        let mut factorial: i32 = 1;
        for i in 2..=count {
            factorial = factorial.wrapping_mul(i);
        }
        dedup_factor = dedup_factor.wrapping_mul(factorial);
    }

    total / std::cmp::max(dedup_factor, 1) as i64
}

/// The unplaceable limits of the grouping algorithms: `std::unordered_map<int, std::vector<int>>`
/// (key: the index of a filament in `used_filaments`, value: the nozzles it cannot be placed on).
pub type UnplaceableLimits = UnorderedMapInt<Vec<i32>>;

/// The `HashMap` the solvers of tool_order_utils.rs take for a limits map they only search.
fn limits_as_hash_map(limits: &UnplaceableLimits) -> HashMap<i32, Vec<i32>> {
    let mut m = HashMap::new();
    for (k, v) in limits.iter() {
        m.insert(k, v.clone());
    }
    m
}

impl FilamentGroup {
    /// FilamentGroup.cpp:252-277 `double FilamentGroup::calc_one_change_budget(const std::vector<unsigned int>& used_filaments) const`
    pub fn calc_one_change_budget(&self, used_filaments: &[u32]) -> f64 {
        // “一次换料”预算：对所有已用料两两组合，各取“较贵方向”的 flush，再取中位数作为
        // “典型一次换料”的代价。最后折算到与 evaluate_score 相同的分数尺度。
        let n = used_filaments.len();
        if n < 2 || self.ctx.model_info.flush_matrix.is_empty() {
            return 0.0;
        }

        let mut pair_flush: Vec<f32> = Vec::new();
        for fm in &self.ctx.model_info.flush_matrix {
            for i in 0..n {
                for j in (i + 1)..n {
                    let a = fm[used_filaments[i] as usize][used_filaments[j] as usize];
                    let b = fm[used_filaments[j] as usize][used_filaments[i] as usize];
                    // std::max(a, b): b when a < b, else a
                    pair_flush.push(if a < b { b } else { a });
                }
            }
        }
        if pair_flush.is_empty() {
            return 0.0;
        }

        let mid = pair_flush.len() / 2;
        // std::nth_element(pair_flush.begin(), pair_flush.begin() + mid, pair_flush.end()): only the
        // element at mid is read, the mid-th smallest, which a sort puts there too.
        pair_flush.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let typical_flush = pair_flush[mid] as f64;

        const K_ONE_CHANGE_MARGIN: f64 = 1.2; // 留余量，保证完整一次换料仍在容忍度内
        typical_flush * FLUSH_VOLUME_TO_SCORE * K_ONE_CHANGE_MARGIN
    }

    /// FilamentGroup.cpp:279-426 `std::vector<int> FilamentGroup::calc_group_by_enum(int k, const std::vector<unsigned int>& used_filaments,
    /// const std::unordered_map<int, std::vector<int>>& unplaceable_limits, int* cost)`
    pub fn calc_group_by_enum(&mut self, k: i32, used_filaments: &[u32], unplaceable_limits: &UnplaceableLimits, cost: Option<&mut i32>) -> Vec<i32> {
        const UNPLACEABLE_LIMIT_REWARD: i32 = 10000;
        const MAX_SIZE_LIMIT_REWARD: i32 = 5000;
        const SUPPORT_PREFER_REWARD: i32 = 100;
        const BEST_FIT_LIMIT_REWARD: i32 = 10;

        let n = used_filaments.len() as i32;
        let one_change_budget = self.calc_one_change_budget(used_filaments);

        // candidates[i]: the groups filament i may be placed in (an unordered_set's elements; only
        // searched below).
        let mut candidates: Vec<Vec<i32>> = vec![Vec::new(); n as usize];
        for i in 0..n {
            let mut group_set: BTreeSet<i32> = BTreeSet::new();
            if let Some(limits) = unplaceable_limits.get(i) {
                for g in 0..k {
                    group_set.insert(g);
                }
                for g in limits {
                    group_set.remove(g);
                }
            } else {
                for g in 0..k {
                    group_set.insert(g);
                }
            }
            candidates[i as usize] = group_set.into_iter().collect();
        }

        // std::unordered_set<std::vector<uint64_t>, ...> group_set: membership of the sorted hashes.
        let mut group_set: HashSet<Vec<u64>> = HashSet::new();
        let mut group_hashs: Vec<u64> = Vec::new();

        let mut nozzles_hash: Vec<u64> = vec![0; k as usize];
        for nozzle_idx in 0..k {
            let nozzle = &self.ctx.nozzle_info.nozzle_list[nozzle_idx as usize];
            let loaded_filament = match self.ctx.nozzle_info.nozzle_status.get(nozzle.group_id) {
                Some(&v) => v,
                None => -1,
            };
            let nozzle_capacity = if nozzle_idx < self.ctx.nozzle_info.per_nozzle_capacity.len() as i32 {
                self.ctx.nozzle_info.per_nozzle_capacity[nozzle_idx as usize]
            } else {
                -1
            };
            nozzles_hash[nozzle_idx as usize] =
                fnv_hash_nozzle(&nozzle.diameter, nozzle.volume_type as i32, (nozzle.extruder_id > 0) as i32, loaded_filament, nozzle_capacity);
        }

        let mut best_full_map: Vec<i32> = vec![self.ctx.machine_info.master_extruder_id; self.ctx.group_info.total_filament_num as usize];
        let mut best_score = f64::MAX;
        let mut best_prefer_level = 0i32;
        let mut best_flush = 0i32;

        // (long long)std::pow(k, n)
        let total = (k as f64).powf(n as f64) as i64;
        for mask in 0..total {
            let mut num = mask;
            // std::unordered_map<int, std::vector<int>> nozzles_filaments: the hashes below are
            // sorted and the per-group sums do not depend on the order, so a BTreeMap serves.
            let mut nozzles_filaments: BTreeMap<i32, Vec<i32>> = BTreeMap::new();
            let mut groups_count: Vec<i32> = vec![0; k as usize];
            let mut used_labels: Vec<i32> = vec![0; n as usize];

            for i in 0..n {
                let g_id = (num % k as i64) as i32;
                num /= k as i64;
                used_labels[i as usize] = g_id;
                nozzles_filaments.entry(g_id).or_default().push(i);
                groups_count[g_id as usize] += 1;
            }

            // Hash dedup
            group_hashs.clear();
            for (nf_key, nf_val) in nozzles_filaments.iter() {
                let mut filament_mask: u64 = 0;
                for &filament in nf_val {
                    filament_mask |= 1u64.wrapping_shl(filament as u32);
                }
                let mut gh: u64 = nozzles_hash[*nf_key as usize];
                // gh ^= (std::hash<uint64_t>{}(filament_mask) + GOLDEN_RATIO_32 + (gh << 6) + (gh >> 2));
                gh ^= filament_mask.wrapping_add(GOLDEN_RATIO_32 as u64).wrapping_add(gh << 6).wrapping_add(gh >> 2);
                group_hashs.push(gh);
            }
            group_hashs.sort_unstable();
            if group_set.contains(&group_hashs) {
                continue;
            }
            group_set.insert(group_hashs.clone());

            // Prefer level
            let mut prefer_level = 0i32;
            let mut placeable_count = 0i32;
            for i in 0..n {
                if candidates[i as usize].contains(&used_labels[i as usize]) {
                    placeable_count += 1;
                }
            }
            prefer_level += placeable_count.wrapping_mul(UNPLACEABLE_LIMIT_REWARD);

            // Per-nozzle capacity (indexed by nozzle group_id), provided by ctx for every machine.
            let nozzle_caps = &self.ctx.nozzle_info.per_nozzle_capacity;
            let mut size_ok = true;
            for g in 0..k {
                if g < nozzle_caps.len() as i32 && groups_count[g as usize] > nozzle_caps[g as usize] {
                    size_ok = false;
                }
            }
            if size_ok {
                prefer_level += MAX_SIZE_LIMIT_REWARD;
            }

            if self.ctx.group_info.strategy == FGStrategy::BestFit {
                let mut all_full = true;
                for g in 0..k {
                    if g < nozzle_caps.len() as i32 && groups_count[g as usize] < nozzle_caps[g as usize] {
                        all_full = false;
                    }
                }
                if all_full {
                    prefer_level += BEST_FIT_LIMIT_REWARD;
                }
            }

            for g in 0..k {
                if g < self.ctx.machine_info.prefer_non_model_filament.len() as i32 && self.ctx.machine_info.prefer_non_model_filament[g as usize] {
                    // nozzles_filaments[g] (operator[]; an absent group holds no filament)
                    if let Some(fidxs) = nozzles_filaments.get(&g) {
                        for &fidx in fidxs {
                            if self.ctx.model_info.filament_info[used_filaments[fidx as usize] as usize].usage_type == FilamentUsageType::SupportOnly {
                                prefer_level += SUPPORT_PREFER_REWARD;
                            }
                        }
                    }
                }
            }

            // Build full map and evaluate
            let mut full_map: Vec<i32> = vec![self.ctx.machine_info.master_extruder_id; self.ctx.group_info.total_filament_num as usize];
            for i in 0..n as usize {
                full_map[used_filaments[i] as usize] = used_labels[i];
            }

            let mut flush_vol = 0i32;
            let mut score = full_evaluate(used_filaments, &full_map, &self.ctx, self.get_custom_seq.as_ref(), Some(&mut flush_vol));

            let master_ex_id = self.ctx.machine_info.master_extruder_id;
            if master_ex_id < k && groups_count[master_ex_id as usize] < ((used_filaments.len() + 1) / 2) as i32 {
                score += ABSOLUTE_FLUSH_GAP_TOLERANCE as f64;
            }

            if prefer_level > best_prefer_level || (prefer_level == best_prefer_level && score < best_score) {
                best_score = score;
                best_prefer_level = prefer_level;
                best_full_map = full_map;
                best_flush = flush_vol;
            }

            let mg = MemoryedGroup::new(used_labels, score, prefer_level);
            update_memoryed_groups(&mg, one_change_budget, self.ctx.group_info.max_gap_threshold, &mut self.m_memoryed_heap);
        }

        if let Some(c) = cost {
            *c = best_flush;
        }
        best_full_map
    }

    /// FilamentGroup.cpp:428-475 `std::vector<int> FilamentGroup::calc_group_by_kmedoids(int k, const std::vector<unsigned int>& used_filaments,
    /// const std::unordered_map<int, std::vector<int>>& unplaceable_limits, int* cost, int timeout_ms)`
    pub fn calc_group_by_kmedoids(
        &mut self,
        k: i32,
        used_filaments: &[u32],
        unplaceable_limits: &UnplaceableLimits,
        cost: Option<&mut i32>,
        timeout_ms: i32,
    ) -> Result<Vec<i32>, FilamentGroupError> {
        let distance_evaluator = Rc::new(FlushDistanceEvaluator::new(
            &self.ctx.model_info.flush_matrix,
            used_filaments,
            &self.ctx.model_info.layer_filaments,
            0.65,
        ));
        let mut pam = KMediods::new(k, used_filaments.len() as i32, distance_evaluator, self.ctx.machine_info.master_extruder_id);
        pam.set_unplacable_limits(unplaceable_limits);
        pam.set_memory_threshold(self.ctx.group_info.max_gap_threshold);
        pam.set_one_change_budget(self.calc_one_change_budget(used_filaments));

        let mut cluster_size_limit: Vec<(BTreeSet<i32>, i32)> = Vec::new();
        for (extruder_id, nozzles) in self.ctx.nozzle_info.extruder_nozzle_list.iter() {
            let first: BTreeSet<i32> = nozzles.iter().copied().collect();
            // ctx.machine_info.max_group_size.at(extruder_id)
            let second = match usize::try_from(*extruder_id).ok().and_then(|e| self.ctx.machine_info.max_group_size.get(e)) {
                Some(&v) => v,
                None => return Err(FilamentGroupError::OutOfRange("vector".to_string())),
            };
            cluster_size_limit.push((first, second));
        }
        pam.set_cluster_group_size(&cluster_size_limit);

        if !self.ctx.nozzle_info.per_nozzle_capacity.is_empty() {
            let cap = &self.ctx.nozzle_info.per_nozzle_capacity;
            let mut per_nozzle_max: Vec<i32> = vec![DEFAULT_CLUSTER_SIZE; k as usize];
            for nz in 0..k {
                per_nozzle_max[nz as usize] = if nz < cap.len() as i32 { cap[nz as usize] } else { DEFAULT_CLUSTER_SIZE };
            }
            pam.set_max_cluster_size(&per_nozzle_max);
        }

        pam.do_clustering(&self.ctx, timeout_ms, 30);

        self.m_memoryed_heap = pam.get_memoryed_groups();

        let labels = pam.get_cluster_labels();
        let mut full_map: Vec<i32> = vec![self.ctx.machine_info.master_extruder_id; self.ctx.group_info.total_filament_num as usize];
        for i in 0..labels.len() {
            full_map[used_filaments[i] as usize] = labels[i];
        }

        if let Some(c) = cost {
            let mut flush_vol = 0i32;
            full_evaluate(used_filaments, &full_map, &self.ctx, self.get_custom_seq.as_ref(), Some(&mut flush_vol));
            *c = flush_vol;
        }

        Ok(full_map)
    }
}

/// FilamentGroup.cpp:477-589 `std::vector<int> select_best_group_for_ams(const std::vector<std::vector<int>>& filament_to_nozzles,
/// const std::vector<MultiNozzleUtils::NozzleInfo>& nozzle_list, const std::vector<unsigned int>& used_filaments,
/// const std::vector<FilamentGroupUtils::FilamentInfo>& used_filament_info,
/// const std::vector<std::vector<MachineFilamentInfo>>& machine_filament_info_, const bool has_filament_switcher,
/// const double color_threshold)`: select the group that best fit the filaments in AMS.
///
/// Calculate the total color distance between the grouping results and the AMS filaments through
/// minimum cost maximum flow. Only those with a distance difference within the threshold are
/// considered valid. Header defaults (FilamentGroup.hpp:124-130): has_filament_switcher = false,
/// color_delta_threshold = 20.
#[allow(clippy::too_many_arguments)]
pub fn select_best_group_for_ams(
    filament_to_nozzles: &[Vec<i32>],
    nozzle_list: &[mnu::NozzleInfo],
    used_filaments: &[u32],
    used_filament_info: &[FilamentInfo],
    machine_filament_info_: &[Vec<MachineFilamentInfo>],
    has_filament_switcher: bool,
    color_threshold: f64,
) -> Vec<i32> {
    if has_filament_switcher {
        return if !filament_to_nozzles.is_empty() { filament_to_nozzles[0].clone() } else { Vec::new() };
    }

    const FAIL_COST: i32 = 9999;

    // these code is to make we machine filament info size is 2
    let mut machine_filament_info: Vec<Vec<MachineFilamentInfo>> = machine_filament_info_.to_vec();
    machine_filament_info.resize(2, Vec::new());

    // 设备侧完全没有耗材信息（未连接打印机 / 未同步 AMS）时，直接保留最小冲刷方案。
    if machine_filament_info.iter().all(|filaments| filaments.is_empty()) {
        return if !filament_to_nozzles.is_empty() { filament_to_nozzles[0].clone() } else { Vec::new() };
    }

    let mut best_cost = i32::MAX;
    let mut best_map: Vec<i32> = Vec::new();

    for filament_to_nozzle in filament_to_nozzles {
        let mut group_filaments: Vec<Vec<i32>> = vec![Vec::new(); 2];
        let mut group_colors: Vec<Vec<Color>> = vec![Vec::new(); 2];

        for i in 0..used_filaments.len() {
            let nozzle = &nozzle_list[filament_to_nozzle[used_filaments[i] as usize] as usize];
            let target_group = if nozzle.extruder_id == 0 { 0 } else { 1 };
            group_colors[target_group].push(used_filament_info[i].color);
            group_filaments[target_group].push(i as i32);
        }

        let mut group_cost: i32 = 0;
        for i in 0..2usize {
            if group_colors[i].is_empty() {
                continue;
            }
            if machine_filament_info[i].is_empty() {
                // 按耗材根数计罚，与下方匹配分支保持同一尺度。
                group_cost = group_cost.wrapping_add((group_colors[i].len() as i32).wrapping_mul(FAIL_COST));
                continue;
            }
            let mut distance_matrix: Vec<Vec<f32>> = vec![vec![0.0f32; machine_filament_info[i].len()]; group_colors[i].len()];

            // calculate color distance matrix
            for src in 0..group_colors[i].len() {
                for dst in 0..machine_filament_info[i].len() {
                    distance_matrix[src][dst] = calc_color_distance(
                        &RGBColor::new(group_colors[i][src].r, group_colors[i][src].g, group_colors[i][src].b),
                        &RGBColor::new(
                            machine_filament_info[i][dst].color.r,
                            machine_filament_info[i][dst].color.g,
                            machine_filament_info[i][dst].color.b,
                        ),
                    );
                }
            }

            // get min cost by min cost max flow
            let l_nodes: Vec<i32> = (0..group_colors[i].len() as i32).collect();
            let r_nodes: Vec<i32> = (0..machine_filament_info[i].len() as i32).collect();

            let mut unlink_limits: HashMap<i32, Vec<i32>> = HashMap::new();
            for from in 0..group_filaments[i].len() {
                for to in 0..machine_filament_info[i].len() {
                    let uf = &used_filament_info[group_filaments[i][from] as usize];
                    if uf.type_ != machine_filament_info[i][to].type_ || uf.is_support != machine_filament_info[i][to].is_support {
                        unlink_limits.entry(from as i32).or_default().push(to as i32);
                    }
                }
            }

            let mut mcmf = MatchModeGroupSolver::new(&distance_matrix, &l_nodes, &r_nodes, &vec![l_nodes.len() as i32; r_nodes.len()], &unlink_limits);
            let ams_map = mcmf.solve();

            for idx in 0..ams_map.len() {
                if ams_map[idx] == INVALID_ID || distance_matrix[idx][ams_map[idx] as usize] as f64 > color_threshold {
                    group_cost = group_cost.wrapping_add(FAIL_COST);
                } else {
                    // group_cost += distance_matrix[idx][ams_map[idx]]: int += float is
                    // group_cost = int(float(group_cost) + distance)
                    group_cost = (group_cost as f32 + distance_matrix[idx][ams_map[idx] as usize]) as i32;
                }
            }
        }

        if best_map.is_empty() || group_cost < best_cost {
            best_cost = group_cost;
            best_map = filament_to_nozzle.clone();
        }
    }

    best_map
}

/// FilamentGroup.cpp:591-633 `void FilamentGroupUtils::update_memoryed_groups(const MemoryedGroup& item, const double one_change_budget,
/// const double gap_threshold, MemoryedGroupHeap& groups)`
pub fn update_memoryed_groups(item: &MemoryedGroup, one_change_budget: f64, gap_threshold: f64, groups: &mut MemoryedGroupHeap) {
    // 保留比最优多约五次换料代价的候选，让 select_best_group_for_ams 能优先选匹配 AMS 的分组。
    let emplace_if_accepatle = |heap: &mut MemoryedGroupHeap, elem: &MemoryedGroup, best: &MemoryedGroup| {
        let gap = (elem.cost - best.cost).abs();
        let a = ABSOLUTE_FLUSH_GAP_TOLERANCE as f64 * 6.0;
        let b = one_change_budget * 5.0;
        // std::max(std::max(a, b), best.cost * gap_threshold)
        let m = if a < b { b } else { a };
        let c = best.cost * gap_threshold;
        let tolerance = if m < c { c } else { m };
        if gap <= tolerance {
            heap.push(elem.clone());
        }
    };

    if groups.empty() {
        groups.push(item.clone());
    } else {
        let top = groups.top().clone();
        // we only memory items with the highest prefer level
        if top.prefer_level > item.prefer_level {
            return;
        } else if top.prefer_level == item.prefer_level {
            if top.cost <= item.cost {
                emplace_if_accepatle(groups, item, &top);
            }
            // find a group with lower cost, rebuild the heap
            else {
                let mut new_heap = MemoryedGroupHeap::new();
                new_heap.push(item.clone());
                while !groups.empty() {
                    let top = groups.top().clone();
                    groups.pop();
                    emplace_if_accepatle(&mut new_heap, &top, item);
                }
                *groups = new_heap;
            }
        }
        // find a group with the higher prefer level, rebuild the heap
        else {
            *groups = MemoryedGroupHeap::new();
            groups.push(item.clone());
        }
    }
}

/// FilamentGroup.cpp:635-644 `std::vector<unsigned int> collect_sorted_used_filaments(const std::vector<std::vector<unsigned int>>& layer_filaments)`
pub fn collect_sorted_used_filaments(layer_filaments: &[Vec<u32>]) -> Vec<u32> {
    let mut used_filaments_set: BTreeSet<u32> = BTreeSet::new();
    for lf in layer_filaments.iter() {
        for &f in lf.iter() {
            used_filaments_set.insert(f);
        }
    }
    let mut used_filaments: Vec<u32> = used_filaments_set.into_iter().collect();
    // sort_remove_duplicates(used_filaments);
    used_filaments.sort_unstable();
    used_filaments.dedup();
    used_filaments
}

/// FilamentGroup.hpp:133-142 `class FlushDistanceEvaluator`
#[derive(Debug, Clone, Default)]
pub struct FlushDistanceEvaluator {
    m_distance_matrix: Vec<Vec<Vec<f32>>>,
}

impl FlushDistanceEvaluator {
    /// FilamentGroup.cpp:646-683 `FlushDistanceEvaluator::FlushDistanceEvaluator(const std::vector<FlushMatrix>& flush_matrix,
    /// const std::vector<unsigned int>& used_filaments, const std::vector<std::vector<unsigned int>>& layer_filaments, double p)`
    /// (header default p = 0.65).
    pub fn new(flush_matrix: &[FlushMatrix], used_filaments: &[u32], layer_filaments: &[Vec<u32>], p: f64) -> Self {
        //calc pair counts
        let nu = used_filaments.len();
        let mut count_matrix: Vec<Vec<i32>> = vec![vec![0; nu]; nu];
        for lf in layer_filaments {
            for (it, &f1) in lf.iter().enumerate() {
                let Some(idx1) = used_filaments.iter().position(|&u| u == f1) else {
                    continue;
                };
                for &f2 in &lf[it + 1..] {
                    let Some(idx2) = used_filaments.iter().position(|&u| u == f2) else {
                        continue;
                    };
                    count_matrix[idx1][idx2] += 1;
                    count_matrix[idx2][idx1] += 1;
                }
            }
        }

        let mut m_distance_matrix: Vec<Vec<Vec<f32>>> = vec![vec![vec![0.0f32; nu]; nu]; flush_matrix.len()];

        for i in 0..nu {
            for j in 0..nu {
                for k in 0..flush_matrix.len() {
                    if i == j {
                        m_distance_matrix[k][i][j] = 0.0;
                    } else {
                        //TODO: check m_flush_matrix
                        let a = flush_matrix[k][used_filaments[i] as usize][used_filaments[j] as usize];
                        let b = flush_matrix[k][used_filaments[j] as usize][used_filaments[i] as usize];
                        // std::max(a, b) / std::min(a, b)
                        let max_val = if a < b { b } else { a };
                        let min_val = if b < a { b } else { a };
                        // (max_val * p + min_val * (1 - p)) * (std::max(count_matrix[i][j], 1)):
                        // float * double promotes to double; clang contracts the first product into
                        // the addition (-ffp-contract=on).
                        let sum = (max_val as f64).mul_add(p, min_val as f64 * (1.0 - p));
                        m_distance_matrix[k][i][j] = (sum * std::cmp::max(count_matrix[i][j], 1) as f64) as f32;
                    }
                }
            }
        }

        FlushDistanceEvaluator { m_distance_matrix }
    }

    /// FilamentGroup.cpp:685-691 `double FlushDistanceEvaluator::get_distance(int idx_a, int idx_b, int extruder_id) const`
    pub fn get_distance(&self, idx_a: i32, idx_b: i32, extruder_id: i32) -> f64 {
        self.m_distance_matrix[extruder_id as usize][idx_a as usize][idx_b as usize] as f64
    }
}

/// FilamentGroup.hpp:145-152 `class TimeEvaluator`
#[derive(Debug, Clone)]
pub struct TimeEvaluator {
    m_speed_info: SpeedInfo,
}

impl TimeEvaluator {
    /// FilamentGroup.hpp:148 `TimeEvaluator(const FilamentGroupContext::SpeedInfo& speed_info) : m_speed_info(speed_info) {}`
    pub fn new(speed_info: &SpeedInfo) -> Self {
        TimeEvaluator { m_speed_info: speed_info.clone() }
    }

    /// FilamentGroup.cpp:693-703 `double TimeEvaluator::get_estimated_time(const std::vector<int>& filament_map) const`
    pub fn get_estimated_time(&self, filament_map: &[i32]) -> f64 {
        let mut time = 0.0f64;
        for (filament_idx, elem_second) in self.m_speed_info.filament_print_time.iter() {
            // auto extruder_time = elem.second; extruder_time[filament_extruder_id] (an absent key
            // value-initialises to 0 in the copy)
            let filament_extruder_id = filament_map[filament_idx as usize];
            time += match elem_second.get(filament_extruder_id) {
                Some(&t) => t,
                None => 0.0,
            };
        }
        time
    }
}

/// FilamentGroup.hpp:211-269 `class KMediods` (non_zero_clusters)
#[derive(Debug, Clone)]
pub struct KMediods {
    memoryed_groups: MemoryedGroupHeap,
    m_evaluator: Rc<FlushDistanceEvaluator>,
    pub m_unplaceable_limits: UnplaceableLimits, // 材料不允许分配到特定喷嘴
    pub m_placeable_limits: UnplaceableLimits,   // 材料必须分配到特定喷嘴
    m_max_cluster_size: Vec<i32>,            // 每个喷嘴能够分配的最大耗材数量
    m_cluster_labels: Vec<i32>,              // 分配结果，细化到喷嘴id
    m_cluster_group_size: Vec<(BTreeSet<i32>, i32)>,
    m_nozzle_to_extruder: Vec<i32>,
    m_k: i32,
    m_elem_count: i32,
    m_default_group_id: i32,
    memory_threshold: f64,
    m_one_change_budget: f64,
}

impl KMediods {
    /// FilamentGroup.hpp:217-223 `KMediods(const int k, const int elem_count, const std::shared_ptr<FlushDistanceEvaluator>& evaluator,
    /// int default_group_id = 0)`
    pub fn new(k: i32, elem_count: i32, evaluator: Rc<FlushDistanceEvaluator>, default_group_id: i32) -> Self {
        KMediods {
            memoryed_groups: MemoryedGroupHeap::new(),
            m_evaluator: evaluator,
            m_unplaceable_limits: UnplaceableLimits::new(),
            m_placeable_limits: UnplaceableLimits::new(),
            m_max_cluster_size: vec![DEFAULT_CLUSTER_SIZE; k.max(0) as usize],
            m_cluster_labels: Vec::new(),
            m_cluster_group_size: Vec::new(),
            m_nozzle_to_extruder: Vec::new(),
            m_k: k,
            m_elem_count: elem_count,
            m_default_group_id: default_group_id,
            memory_threshold: 0.0,
            m_one_change_budget: 0.0,
        }
    }

    /// FilamentGroup.hpp:225 `void set_max_cluster_size(const std::vector<int>& group_size)`
    pub fn set_max_cluster_size(&mut self, group_size: &[i32]) {
        self.m_max_cluster_size = group_size.to_vec();
    }

    /// FilamentGroup.hpp:230 `void set_placable_limits(const std::unordered_map<int, std::vector<int>>& placable_limits)`:
    /// key stores elem, value stores the cluster id that the elem must be placed
    pub fn set_placable_limits(&mut self, placable_limits: &UnplaceableLimits) {
        self.m_placeable_limits.assign_from(placable_limits);
    }

    /// FilamentGroup.hpp:233 `void set_unplacable_limits(const std::unordered_map<int, std::vector<int>>& unplacable_limits)`:
    /// key stores elem, value stores the cluster id that the elem cannot be placed
    pub fn set_unplacable_limits(&mut self, unplacable_limits: &UnplaceableLimits) {
        self.m_unplaceable_limits.assign_from(unplacable_limits);
    }

    /// FilamentGroup.hpp:235 `void set_memory_threshold(double threshold)`
    pub fn set_memory_threshold(&mut self, threshold: f64) {
        self.memory_threshold = threshold;
    }

    /// FilamentGroup.hpp:236 `void set_one_change_budget(double budget)`
    pub fn set_one_change_budget(&mut self, budget: f64) {
        self.m_one_change_budget = budget;
    }

    /// FilamentGroup.hpp:237 `MemoryedGroupHeap get_memoryed_groups()const`
    pub fn get_memoryed_groups(&self) -> MemoryedGroupHeap {
        self.memoryed_groups.clone()
    }

    /// FilamentGroup.hpp:240 `std::vector<int> get_cluster_labels()const`
    pub fn get_cluster_labels(&self) -> Vec<i32> {
        self.m_cluster_labels.clone()
    }

    /// FilamentGroup.cpp:707-714 `void KMediods::set_cluster_group_size(const std::vector<std::pair<std::set<int>, int>> &cluster_group_size)`
    pub fn set_cluster_group_size(&mut self, cluster_group_size: &[(BTreeSet<i32>, i32)]) {
        self.m_cluster_group_size = cluster_group_size.to_vec();
        self.m_nozzle_to_extruder.resize(self.m_k as usize, 0);
        for i in 0..self.m_cluster_group_size.len() {
            for &nozzle_id in &self.m_cluster_group_size[i].0 {
                self.m_nozzle_to_extruder[nozzle_id as usize] = i as i32;
            }
        }
    }

    /// FilamentGroup.cpp:716-745 `int KMediods::calc_cost(const std::vector<int>& cluster_labels, const std::vector<int>& cluster_centers,int cluster_id)`
    /// (header default cluster_id = -1).
    pub fn calc_cost(&self, cluster_labels: &[i32], cluster_centers: &[i32], cluster_id: i32) -> i32 {
        let mut total_cost: i32 = 0;

        let mut nozzle_cost: Vec<(i32, f64)> = vec![(0, 0.0); self.m_k as usize];
        let mut nozzle_filaments: Vec<i32> = vec![0; self.m_k as usize];
        for i in 0..self.m_elem_count {
            if cluster_id != -1 && cluster_labels[i as usize] != cluster_id {
                continue;
            }
            if cluster_centers[cluster_labels[i as usize] as usize] == -1 {
                continue;
            }

            nozzle_filaments[cluster_labels[i as usize] as usize] += 1;
            for j in (i + 1)..self.m_elem_count {
                let nozzle_i = cluster_labels[i as usize];
                let nozzle_j = cluster_labels[j as usize];
                if nozzle_i == nozzle_j {
                    nozzle_cost[nozzle_i as usize].0 += 1;
                    nozzle_cost[nozzle_j as usize].1 += self.m_evaluator.get_distance(i, j, self.m_nozzle_to_extruder[nozzle_i as usize]);
                }
            }
        }
        for i in 0..nozzle_cost.len() {
            if nozzle_filaments[i] > 0 && nozzle_cost[i].1 > 0.0 {
                // total_cost += second / first * (count - 1): int += double
                total_cost = (total_cost as f64 + nozzle_cost[i].1 / nozzle_cost[i].0 as f64 * (nozzle_filaments[i] - 1) as f64) as i32;
            }
        }

        total_cost
    }

    /// FilamentGroup.cpp:747-762 `bool KMediods::have_enough_size(const std::vector<int>& cluster_size,
    /// const std::vector<std::pair<std::set<int>, int>>& cluster_group_size,int elem_count)`
    pub fn have_enough_size(&self, cluster_size: &[i32], cluster_group_size: &[(BTreeSet<i32>, i32)], elem_count: i32) -> bool {
        let mut have_enough_size = true;
        let mut cluster_sum: Option<i32> = None;
        let mut cluster_group_sum: Option<i32> = None;

        if !cluster_size.is_empty() {
            cluster_sum = Some(cluster_size.iter().fold(0i32, |a, &b| a.wrapping_add(b)));
        }
        if !cluster_group_size.is_empty() {
            cluster_group_sum = Some(cluster_group_size.iter().fold(0i32, |a, p| a.wrapping_add(p.1)));
        }
        if let Some(s) = cluster_sum {
            have_enough_size &= s >= elem_count;
        }
        if let Some(s) = cluster_group_sum {
            have_enough_size &= s >= elem_count;
        }
        have_enough_size
    }

    /// FilamentGroup.cpp:765-806 `std::vector<int> KMediods::init_cluster_center(const std::unordered_map<int, std::vector<int>>& placeable_limits,
    /// const std::unordered_map<int, std::vector<int>>& unplaceable_limits,const std::vector<int>& cluster_size,
    /// const std::vector<std::pair<std::set<int>,int>>& cluster_group_size, int seed)`: make sure each cluster at least have one elements
    pub fn init_cluster_center(
        &self,
        placeable_limits: &UnplaceableLimits,
        unplaceable_limits: &UnplaceableLimits,
        _cluster_size: &[i32],
        _cluster_group_size: &[(BTreeSet<i32>, i32)],
        seed: i32,
    ) -> Vec<i32> {
        // max flow network
        let mut l_nodes: Vec<i32> = (0..self.m_elem_count).collect(); // represent the filament idx, to be shuffled
        let r_nodes: Vec<i32> = (0..self.m_k).collect(); // represent the group idx

        let mut shuffled_placeable_limits = UnplaceableLimits::new();
        let mut shuffled_unplaceable_limits = UnplaceableLimits::new();
        // shuffle the filaments and transfer placeable,unplaceable limits
        {
            let mut rng = crate::placeholder_parser::Mt19937::new(seed as u32);
            libcxx_shuffle(&mut l_nodes, &mut rng);

            // std::unordered_map<int, int> idx_transfer: only read through operator[], which gives 0
            // for an absent key.
            let mut idx_transfer: HashMap<i32, i32> = HashMap::new();
            for idx in 0..l_nodes.len() {
                let new_idx = l_nodes.iter().position(|&x| x == idx as i32).unwrap_or(l_nodes.len()) as i32;
                idx_transfer.insert(idx as i32, new_idx);
            }
            for (k, v) in placeable_limits.iter() {
                let t = *idx_transfer.entry(k).or_insert(0);
                *shuffled_placeable_limits.entry(t) = v.clone();
            }
            for (k, v) in unplaceable_limits.iter() {
                let t = *idx_transfer.entry(k).or_insert(0);
                *shuffled_unplaceable_limits.entry(t) = v.clone();
            }
        }

        let mut m = MaxFlowSolver::new(
            &l_nodes,
            &r_nodes,
            &limits_as_hash_map(&shuffled_placeable_limits),
            &limits_as_hash_map(&shuffled_unplaceable_limits),
            &[],
            &[],
            &[],
        );
        let ret = m.solve();

        // if still has -1，it means there has some filaments that cannot be placed under the limit. We neglect the -1 here since we
        // are deciding the cluster center, -1 can be handled in later steps
        let mut cluster_center: Vec<i32> = vec![-1; self.m_k as usize];
        for idx in 0..ret.len() {
            if ret[idx] != -1 {
                cluster_center[ret[idx] as usize] = l_nodes[idx];
            }
        }

        cluster_center
    }

    /// FilamentGroup.cpp:808-882 `std::vector<int> KMediods::assign_cluster_label(const std::vector<int>& center,
    /// const std::unordered_map<int, std::vector<int>>& placeable_limits, const std::unordered_map<int, std::vector<int>>& unplaceable_limits,
    /// const std::vector<int>& cluster_size, const std::vector<std::pair<std::set<int>, int>>& cluster_group_size)`
    pub fn assign_cluster_label(
        &self,
        center: &[i32],
        placeable_limits: &UnplaceableLimits,
        unplaceable_limits: &UnplaceableLimits,
        cluster_size: &[i32],
        cluster_group_size: &[(BTreeSet<i32>, i32)],
    ) -> Vec<i32> {
        let mut labels: Vec<i32> = vec![-1; self.m_elem_count as usize];
        let l_nodes: Vec<i32> = (0..self.m_elem_count).collect();
        let r_nodes: Vec<i32> = (0..self.m_k).collect();

        let mut distance_matrix: Vec<Vec<f32>> = vec![vec![0.0f32; self.m_k as usize]; self.m_elem_count as usize];
        for i in 0..self.m_elem_count {
            for j in 0..self.m_k {
                if center[j as usize] == -1 {
                    distance_matrix[i as usize][j as usize] = MCMF_MAX_EDGE_COST as f32;
                } else {
                    distance_matrix[i as usize][j as usize] =
                        self.m_evaluator.get_distance(i, center[j as usize], self.m_nozzle_to_extruder[j as usize]) as f32;
                }
            }
        }

        // only consider the size limit if the group can contain all of the filaments
        let r_nodes_capacity: Vec<i32>;
        let mut r_nodes_group_capacity: Vec<(BTreeSet<i32>, i32)> = Vec::new();
        if self.have_enough_size(cluster_size, cluster_group_size, self.m_elem_count) {
            r_nodes_capacity = cluster_size.to_vec();
            r_nodes_group_capacity = cluster_group_size.to_vec();
        } else {
            // TODO: xcr：throw exception here?
            // adjust group size to elem count if the group cannot contain all of the filamnts
            r_nodes_capacity = vec![self.m_elem_count; self.m_k as usize];
        }
        let l_nodes_capacity: Vec<i32> = vec![1; l_nodes.len()];

        // Each group can receive up to m_elem_count materials at most, so the flow from r_nodes to sink is adjusted to m_elem_count.
        let mut m = MinFlushFlowSolver::new(
            &distance_matrix,
            &l_nodes,
            &r_nodes,
            &limits_as_hash_map(placeable_limits),
            &limits_as_hash_map(unplaceable_limits),
            &l_nodes_capacity,
            &r_nodes_capacity,
            &r_nodes_group_capacity,
        );
        let ret = m.solve();

        for idx in 0..ret.len() {
            if ret[idx] != INVALID_ID {
                labels[l_nodes[idx] as usize] = r_nodes[ret[idx] as usize];
            }
        }

        // for (size_t idx = 0; idx < center.size(); ++idx) if (center[idx] != -1) assert(labels[center[idx]] == idx);

        // If there are materials that have not been grouped in the last step, assign them to a valid group.
        for idx in 0..labels.len() {
            if labels[idx] == -1 {
                let mut fallback = self.m_default_group_id;
                if let Some(limits) = unplaceable_limits.get(idx as i32) {
                    for nid in 0..self.m_k {
                        if !limits.contains(&nid) {
                            fallback = nid;
                            break;
                        }
                    }
                }
                labels[idx] = fallback;
            }
        }

        labels
    }

    /// FilamentGroup.cpp:884-967 `void KMediods::do_clustering(const FilamentGroupContext &context, int timeout_ms, int retry)`
    /// (header defaults timeout_ms = 100, retry = 10).
    ///
    /// 1.Select initial medoids randomly
    /// 2.Iterate while the cost decreases:
    ///   2.1 In each cluster, make the point that minimizes the sum of distances within the cluster the medoid
    ///   2.2 Reassign each point to the cluster defined by the closest medoid determined in the previous step
    pub fn do_clustering(&mut self, context: &FilamentGroupContext, timeout_ms: i32, retry: i32) {
        let mut t = FlushTimeMachine::default();
        t.time_machine_start();

        let used_filaments: Vec<u32> = collect_sorted_used_filaments(&context.model_info.layer_filaments);

        let default_group_id = self.m_default_group_id;
        let build_full_map = |labels: &[i32]| -> Vec<i32> {
            let mut full_map: Vec<i32> = vec![default_group_id; context.group_info.total_filament_num as usize];
            for i in 0..labels.len() {
                full_map[used_filaments[i] as usize] = labels[i];
            }
            full_map
        };

        let evaluate_labels = |labels: &[i32]| -> f64 {
            let full_map = build_full_map(labels);
            full_evaluate(&used_filaments, &full_map, context, None, None)
        };

        let mut best_cluster_centers: Vec<i32> = vec![0; self.m_k as usize];
        let mut best_cluster_labels: Vec<i32> = vec![self.m_default_group_id; self.m_elem_count as usize];
        let mut best_cluster_cost = f64::MAX;
        let mut retry_count = 0i32;

        while retry_count < retry && t.time_machine_end() < timeout_ms {
            let mut curr_cluster_centers = self.init_cluster_center(
                &self.m_placeable_limits,
                &self.m_unplaceable_limits,
                &self.m_max_cluster_size,
                &self.m_cluster_group_size,
                retry_count,
            );
            let mut curr_cluster_labels = self.assign_cluster_label(
                &curr_cluster_centers,
                &self.m_placeable_limits,
                &self.m_unplaceable_limits,
                &self.m_max_cluster_size,
                &self.m_cluster_group_size,
            );
            let mut curr_cluster_cost = evaluate_labels(&curr_cluster_labels);

            let g = MemoryedGroup::new(curr_cluster_labels.clone(), curr_cluster_cost, 1);
            update_memoryed_groups(&g, self.m_one_change_budget, self.memory_threshold, &mut self.memoryed_groups);

            let mut mediods_changed = true;
            while mediods_changed && t.time_machine_end() < timeout_ms {
                mediods_changed = false;
                let mut best_swap_cost = curr_cluster_cost;
                let mut best_swap_cluster: i32 = -1;
                let mut best_swap_elem: i32 = -1;

                for cluster_id in 0..self.m_k {
                    if curr_cluster_centers[cluster_id as usize] == -1 {
                        continue;
                    }
                    for elem in 0..self.m_elem_count {
                        // m_unplaceable_limits[cluster_id]: operator[] inserts an empty list for an
                        // absent key (the key here is a cluster id, the map's keys are elements);
                        // it is reached only when elem is not a center (`||`).
                        if curr_cluster_centers.contains(&elem) || self.m_unplaceable_limits.entry(cluster_id).contains(&elem) {
                            continue;
                        }
                        let mut tmp_centers = curr_cluster_centers.clone();
                        tmp_centers[cluster_id as usize] = elem;
                        let tmp_labels = self.assign_cluster_label(
                            &tmp_centers,
                            &self.m_placeable_limits,
                            &self.m_unplaceable_limits,
                            &self.m_max_cluster_size,
                            &self.m_cluster_group_size,
                        );
                        let tmp_cost = evaluate_labels(&tmp_labels);

                        if tmp_cost < best_swap_cost {
                            best_swap_cost = tmp_cost;
                            best_swap_cluster = cluster_id;
                            best_swap_elem = elem;
                            mediods_changed = true;
                        }
                    }
                }

                if mediods_changed {
                    curr_cluster_centers[best_swap_cluster as usize] = best_swap_elem;
                    curr_cluster_labels = self.assign_cluster_label(
                        &curr_cluster_centers,
                        &self.m_placeable_limits,
                        &self.m_unplaceable_limits,
                        &self.m_max_cluster_size,
                        &self.m_cluster_group_size,
                    );
                    curr_cluster_cost = evaluate_labels(&curr_cluster_labels);

                    let g = MemoryedGroup::new(curr_cluster_labels.clone(), curr_cluster_cost, 1);
                    update_memoryed_groups(&g, self.m_one_change_budget, self.memory_threshold, &mut self.memoryed_groups);
                }
            }

            if curr_cluster_cost < best_cluster_cost {
                best_cluster_centers = curr_cluster_centers;
                best_cluster_cost = curr_cluster_cost;
                best_cluster_labels = curr_cluster_labels;
            }
            retry_count += 1;
        }
        let _ = best_cluster_centers;
        self.m_cluster_labels = best_cluster_labels;
    }
}

impl FilamentGroupContext {
    /// FilamentGroup.cpp:969-1009 `FilamentGroupContext FilamentGroupContext::available_only(std::vector<int>& full_nozzle_ids) const`:
    /// 只保留可用喷嘴（已安装），重编号成紧凑 [0,k)；full_nozzle_ids 记录 紧凑id -> 原始id。
    pub fn available_only(&self, full_nozzle_ids: &mut Vec<i32>) -> FilamentGroupContext {
        full_nozzle_ids.clear();

        let mut compact = self.clone();

        // 只保留已安装喷嘴，把 group_id 重编号成紧凑 [0,k)
        let mut compact_nozzle_info = NozzleInfoStruct::default();
        for nozzle in &self.nozzle_info.nozzle_list {
            let full_id = nozzle.group_id;
            if !nozzle.installed {
                continue;
            }

            let compact_id = full_nozzle_ids.len() as i32;
            full_nozzle_ids.push(full_id);

            let mut compact_nozzle = nozzle.clone();
            compact_nozzle.group_id = compact_id;
            compact_nozzle_info.nozzle_list.push(compact_nozzle);
            compact_nozzle_info.per_nozzle_capacity.push(if full_id >= 0 && full_id < self.nozzle_info.per_nozzle_capacity.len() as i32 {
                self.nozzle_info.per_nozzle_capacity[full_id as usize]
            } else {
                1
            });
            compact_nozzle_info.extruder_nozzle_list.entry(nozzle.extruder_id).or_default().push(compact_id);

            if let Some(&v) = self.nozzle_info.nozzle_status.get(full_id) {
                *compact_nozzle_info.nozzle_status.entry(compact_id) = v;
            }
        }
        compact.nozzle_info = compact_nozzle_info;

        // master_extruder_id / prefer_non_model_filament 在 flush 算法里按喷嘴id 使用，需与 nozzle_info
        // 一并转到紧凑空间（master_extruder_id 兼作默认喷嘴槽位）。
        let machine = &mut compact.machine_info;
        machine.master_extruder_id = 0;
        for i in 0..full_nozzle_ids.len() {
            if full_nozzle_ids[i] == self.machine_info.master_extruder_id {
                machine.master_extruder_id = i as i32;
                break;
            }
        }

        let full_prefer = &self.machine_info.prefer_non_model_filament;
        machine.prefer_non_model_filament.clear();
        for &full_id in full_nozzle_ids.iter() {
            machine.prefer_non_model_filament.push(full_id >= 0 && full_id < full_prefer.len() as i32 && full_prefer[full_id as usize]);
        }

        compact
    }
}

/// FilamentGroup.cpp:1011-1017 `static void map_to_full_nozzle_ids(std::vector<int>& filament_to_nozzle, const std::vector<int>& full_nozzle_ids)`:
/// 把耗材映射里的紧凑喷嘴id 还原回原始喷嘴id
pub fn map_to_full_nozzle_ids(filament_to_nozzle: &mut [i32], full_nozzle_ids: &[i32]) {
    for nozzle_id in filament_to_nozzle.iter_mut() {
        if *nozzle_id >= 0 && *nozzle_id < full_nozzle_ids.len() as i32 {
            *nozzle_id = full_nozzle_ids[*nozzle_id as usize];
        }
    }
}

/// FilamentGroup.hpp:154-192 `class FilamentGroup`
///
/// The members are public as the C++ private ones are to a friend (the native test reads them).
pub struct FilamentGroup {
    pub ctx: FilamentGroupContext,
    pub m_memoryed_heap: MemoryedGroupHeap,
    pub m_memoryed_groups: Vec<Vec<i32>>,
    /// FilamentGroup.hpp:191 `std::optional<std::function<bool(int, std::vector<int>&)>> get_custom_seq;`
    pub get_custom_seq: Option<CustomSeq>,
}

impl FilamentGroup {
    /// FilamentGroup.hpp:159 `explicit FilamentGroup(const FilamentGroupContext& ctx_) :ctx(ctx_) {}` (a copy).
    pub fn new(ctx: &FilamentGroupContext) -> Self {
        FilamentGroup { ctx: ctx.clone(), m_memoryed_heap: MemoryedGroupHeap::new(), m_memoryed_groups: Vec::new(), get_custom_seq: None }
    }

    /// FilamentGroup.hpp:162 `std::vector<std::vector<int>> get_memoryed_groups()const`
    pub fn get_memoryed_groups(&self) -> Vec<Vec<i32>> {
        self.m_memoryed_groups.clone()
    }

    /// FilamentGroup.cpp:1019-1066 `std::vector<int> FilamentGroup::calc_min_flush_group(int* cost)`
    pub fn calc_min_flush_group(&mut self, cost: Option<&mut i32>) -> Result<Vec<i32>, FilamentGroupError> {
        let used_filaments = collect_sorted_used_filaments(&self.ctx.model_info.layer_filaments);
        let n = used_filaments.len() as i32;

        // 分组只在可用喷嘴上进行：enum/kmedoids 把容量当软奖励，空槽位必须从候选里剔除（也缩小 k^n）。
        // available_only 会把 nozzle_info 与按喷嘴id 使用的 machine 字段一并转到紧凑空间。
        let mut full_nozzle_ids: Vec<i32> = Vec::new(); // 紧凑id -> 原始id
        let available_ctx = self.ctx.available_only(&mut full_nozzle_ids);
        if full_nozzle_ids.is_empty() {
            return Err(runtime_error("No valid nozzle found. Please check nozzle count."));
        }

        let mut available_group = FilamentGroup::new(&available_ctx);
        available_group.get_custom_seq = self.get_custom_seq.clone();
        let k = available_ctx.nozzle_info.nozzle_list.len() as i32;

        let mut extruder_unplaceable: HashMap<i32, Vec<i32>> = HashMap::new();
        extract_unprintable_limit_indices(&available_ctx.model_info.unprintable_filaments, &used_filaments, &mut extruder_unplaceable);
        let unplaceable_limits =
            available_group.rebuild_nozzle_unprintables(&used_filaments, &extruder_unplaceable, &available_ctx.group_info.filament_volume_map);

        // 多口径模式下，显式指定了喷嘴口径的耗材若在所有可用喷嘴上均不可放置，直接报错，避免被软约束兜底分配到错误口径的喷嘴。
        for fidx in 0..n {
            let filament_id = used_filaments[fidx as usize];
            if filament_id as usize >= available_ctx.model_info.selected_nozzle_diameters.len()
                || available_ctx.model_info.selected_nozzle_diameters[filament_id as usize].is_empty()
            {
                continue;
            }
            if let Some(limits) = unplaceable_limits.get(fidx) {
                if limits.len() as i32 >= k {
                    return Err(runtime_error(
                        "Nozzle assignment error: no nozzle matches the diameter and flow selected for one of the filaments. Please check the nozzle setup.",
                    ));
                }
            }
        }

        self.m_memoryed_heap = MemoryedGroupHeap::new();
        let mut result: Vec<i32>;

        let estimated = estimate_dedup_enum_count(k, n, &available_ctx);
        if estimated < ENUM_THRESHOLD {
            result = available_group.calc_group_by_enum(k, &used_filaments, &unplaceable_limits, cost);
        } else {
            result = available_group.calc_group_by_kmedoids(k, &used_filaments, &unplaceable_limits, cost, 3000)?;
        }

        map_to_full_nozzle_ids(&mut result, &full_nozzle_ids);

        change_memoryed_heaps_to_arrays(
            &mut available_group.m_memoryed_heap,
            self.ctx.group_info.total_filament_num,
            &used_filaments,
            &mut self.m_memoryed_groups,
        );
        for memoryed_group in self.m_memoryed_groups.iter_mut() {
            map_to_full_nozzle_ids(memoryed_group, &full_nozzle_ids);
        }

        Ok(result)
    }

    /// FilamentGroup.cpp:1068-1094 `std::map<int, int> FilamentGroup::rebuild_unprintables(const std::vector<unsigned int>& used_filaments,
    /// const std::map<int,int>& extruder_unprintables)`
    pub fn rebuild_unprintables(&mut self, used_filaments: &[u32], extruder_unprintables: &BTreeMap<i32, i32>) -> BTreeMap<i32, i32> {
        let mut ret: BTreeMap<i32, i32> = BTreeMap::new();
        for f_idx in 0..used_filaments.len() as i32 {
            let mut unprintable_ext = -1;
            if let Some(&e) = extruder_unprintables.get(&f_idx) {
                unprintable_ext = e;
            }

            let mut multi_unprintable = false;
            // ctx.model_info.unprintable_volumes[used_filaments[f_idx]] (std::map operator[]: inserts)
            let unprintable_volumes = self.ctx.model_info.unprintable_volumes.entry(used_filaments[f_idx as usize] as i32).or_default().clone();
            for nozzle_idx in 0..self.ctx.nozzle_info.nozzle_list.len() {
                let nozzle_info = &self.ctx.nozzle_info.nozzle_list[nozzle_idx];

                if unprintable_volumes.contains(&nozzle_info.volume_type) {
                    if unprintable_ext == -1 {
                        unprintable_ext = nozzle_info.extruder_id;
                    } else if unprintable_ext != nozzle_info.extruder_id {
                        multi_unprintable = true;
                    }
                }
            }

            if !multi_unprintable && unprintable_ext != -1 {
                ret.insert(f_idx, unprintable_ext);
            }
        }
        ret
    }

    /// FilamentGroup.cpp:1096-1129 `std::unordered_map<int, std::vector<int>> FilamentGroup::try_merge_filaments()`
    pub fn try_merge_filaments(&self) -> HashMap<i32, Vec<i32>> {
        let mut merged_filaments: HashMap<i32, Vec<i32>> = HashMap::new();

        // std::unordered_map<std::string, std::vector<int>> merge_filament_map: the merged groups do
        // not depend on its order.
        let mut merge_filament_map: HashMap<String, Vec<i32>> = HashMap::new();

        let unprintable_filaments = &self.ctx.model_info.unprintable_filaments;
        let unprintable_stat_to_str = |idx: i32| -> String {
            let mut s = String::new();
            for eid in 0..unprintable_filaments.len() {
                if unprintable_filaments[eid].contains(&idx) {
                    if eid > 0 {
                        s.push(',');
                    }
                    s += &idx.to_string();
                }
            }
            s
        };

        for idx in 0..self.ctx.model_info.filament_ids.len() {
            let id = self.ctx.model_info.filament_ids[idx].clone();
            let color = self.ctx.model_info.filament_info[idx].color;
            let unprintable_str = unprintable_stat_to_str(idx as i32);

            let key = id + "," + &color.to_hex_str(true) + "," + &unprintable_str;
            merge_filament_map.entry(key).or_default().push(idx as i32);
        }

        for (_k, v) in merge_filament_map.iter() {
            if v.len() > 1 {
                merged_filaments.insert(v[0], v.clone());
            }
        }
        merged_filaments
    }

    /// FilamentGroup.cpp:1131-1141 `std::vector<int> FilamentGroup::seperate_merged_filaments(const std::vector<int>& filament_map,
    /// const std::unordered_map<int, std::vector<int>>& merged_filaments)`
    pub fn seperate_merged_filaments(&self, filament_map: &[i32], merged_filaments: &HashMap<i32, Vec<i32>>) -> Vec<i32> {
        let mut ret_map: Vec<i32> = filament_map.to_vec();
        for (&src, filaments) in merged_filaments.iter() {
            for &f in filaments {
                ret_map[f as usize] = ret_map[src as usize];
            }
        }
        ret_map
    }

    /// FilamentGroup.cpp:1143-1177 `void FilamentGroup::rebuild_context(const std::unordered_map<int, std::vector<int>>& merged_filaments)`
    pub fn rebuild_context(&mut self, merged_filaments: &HashMap<i32, Vec<i32>>) {
        if merged_filaments.is_empty() {
            return;
        }

        let mut new_ctx = self.ctx.clone();

        let filament_merge_map = get_merged_filament_map(merged_filaments);

        // modify layer filaments
        for layer_filament in new_ctx.model_info.layer_filaments.iter_mut() {
            for f in layer_filament.iter_mut() {
                if let Some(&m) = filament_merge_map.get(&(*f as i32)) {
                    *f = m as u32;
                }
            }
        }

        for unprintables in &new_ctx.model_info.unprintable_filaments {
            // The C++ builds the remapped set and drops it (it is never written back).
            let mut new_unprintables: BTreeSet<i32> = BTreeSet::new();
            for &f in unprintables {
                if let Some(&m) = filament_merge_map.get(&f) {
                    new_unprintables.insert(m);
                } else {
                    new_unprintables.insert(f);
                }
            }
            let _ = new_unprintables;
        }

        self.ctx = new_ctx;
    }

    /// FilamentGroup.cpp:1179-1195 `std::vector<int> FilamentGroup::calc_filament_group(int* cost)`
    pub fn calc_filament_group(&mut self, cost: Option<&mut i32>) -> Result<Vec<i32>, FilamentGroupError> {
        let mut cost = cost;
        /*auto extruder_variant_list = ctx.nozzle_info.extruder_nozzle_list;
        for (auto nozzle : ctx.nozzle_info.nozzle_list)
            if (nozzle.volume_type == NozzleVolumeType::nvtTPUHighFlow)
                return calc_filament_group_for_tpu(cost);*/

        // try { ... } catch (const FilamentGroupException&) { BOOST_LOG_TRIVIAL(warning) << ...; }
        if FGMode::MatchMode == self.ctx.group_info.mode {
            match self.calc_filament_group_for_match(cost.as_deref_mut()) {
                Ok(v) => return Ok(v),
                Err(FilamentGroupError::Group(_e)) => {}
                Err(e) => return Err(e),
            }
        }

        self.calc_filament_group_for_flush(cost)
    }

    /// FilamentGroup.cpp:1197-1406 `std::vector<int> FilamentGroup::calc_filament_group_for_match(int* cost)`
    pub fn calc_filament_group_for_match(&mut self, _cost: Option<&mut i32>) -> Result<Vec<i32>, FilamentGroupError> {
        const SUPPORT_PREFER_SCORE: i32 = 3;

        let used_filaments = collect_sorted_used_filaments(&self.ctx.model_info.layer_filaments);
        let mut used_filament_list: Vec<FilamentInfo> = Vec::new();
        for &f in &used_filaments {
            used_filament_list.push(self.ctx.model_info.filament_info[f as usize].clone());
        }

        let mut machine_filament_list: Vec<MachineFilamentInfo> = Vec::new();
        let mut machine_filament_set: BTreeMap<MachineFilamentInfo, BTreeSet<i32>> = BTreeMap::new();
        for eid in 0..self.ctx.machine_info.machine_filament_info.len() {
            for filament in &self.ctx.machine_info.machine_filament_info[eid] {
                machine_filament_set.entry(filament.clone()).or_default().insert(machine_filament_list.len() as i32);
                machine_filament_list.push(filament.clone());
            }
        }

        if machine_filament_list.is_empty() {
            return Err(FilamentGroupError::Group(FilamentGroupException::new(
                ErrorCode::EmptyAmsFilaments,
                "Empty ams filament in For-Match mode.".to_string(),
            )));
        }

        let mut unprintable_limit_indices: BTreeMap<i32, i32> = BTreeMap::new(); // key stores filament idx in used_filament, value stores unprintable extruder
        extract_unprintable_limit_indices(&self.ctx.model_info.unprintable_filaments, &used_filaments, &mut unprintable_limit_indices);
        unprintable_limit_indices = self.rebuild_unprintables(&used_filaments, &unprintable_limit_indices);

        // 行为工程耗材，列为设备耗材，成本越小越优先匹配。
        let mut match_cost_matrix: Vec<Vec<f32>> = vec![vec![0.0f32; machine_filament_list.len()]; used_filament_list.len()];
        for i in 0..used_filament_list.len() {
            // 映射回工程耗材下标，读取表示耗材小类的 filament_id。
            let model_filament_idx = used_filaments[i] as usize;
            let model_filament_id: String = if model_filament_idx < self.ctx.model_info.filament_ids.len() {
                self.ctx.model_info.filament_ids[model_filament_idx].clone()
            } else {
                String::new()
            };
            for j in 0..machine_filament_list.len() {
                let color_dist = calc_color_distance(
                    &RGBColor::new(used_filament_list[i].color.r, used_filament_list[i].color.g, used_filament_list[i].color.b),
                    &RGBColor::new(machine_filament_list[j].color.r, machine_filament_list[j].color.g, machine_filament_list[j].color.b),
                );
                // 颜色优先；同色时优先匹配相同耗材小类。
                match_cost_matrix[i][j] = calc_match_mode_cost(
                    color_dist,
                    &model_filament_id,
                    &machine_filament_list[j].filament_id,
                    used_filament_list.len() as i32,
                ) as f32;
            }
        }

        let mut l_nodes: Vec<i32> = (0..used_filaments.len() as i32).collect();
        let r_nodes: Vec<i32> = (0..machine_filament_list.len() as i32).collect();
        let machine_filament_capacity: Vec<i32> = vec![l_nodes.len() as i32; machine_filament_list.len()];
        let mut extruder_filament_count: Vec<i32> = vec![0; 2];

        let is_extruder_filament_compatible = |unprintable_limit_indices: &BTreeMap<i32, i32>, filament_idx: i32, extruder_id: i32| -> bool {
            if let Some(&e) = unprintable_limit_indices.get(&filament_idx) {
                if e == extruder_id {
                    return false;
                }
            }
            true
        };

        let build_unlink_limits = |l_nodes: &[i32], r_nodes: &[i32], can_link: &dyn Fn(i32, i32) -> bool| -> HashMap<i32, Vec<i32>> {
            let mut unlink_limits: HashMap<i32, Vec<i32>> = HashMap::new();
            for i in 0..l_nodes.len() {
                let mut unlink_filaments: Vec<i32> = Vec::new();
                for j in 0..r_nodes.len() {
                    if !can_link(l_nodes[i], r_nodes[j]) {
                        unlink_filaments.push(j as i32);
                    }
                }
                if !unlink_filaments.is_empty() {
                    unlink_limits.entry(i as i32).or_insert(unlink_filaments);
                }
            }
            unlink_limits
        };

        // auto optimize_map_to_machine_filament = [&](const std::vector<int>& map_to_machine_filament, const std::vector<int>& l_nodes,
        //     const std::vector<int>& r_nodes, std::vector<int>& filament_map) -> std::vector<int>
        let prefer_non_model_filament = self.ctx.machine_info.prefer_non_model_filament.clone();
        let optimize_map_to_machine_filament = |map_to_machine_filament: &[i32],
                                                l_nodes: &[i32],
                                                r_nodes: &[i32],
                                                filament_map: &mut Vec<i32>,
                                                extruder_filament_count: &mut Vec<i32>|
         -> Vec<i32> {
            let mut ungrouped_filaments: Vec<i32> = Vec::new();
            let mut filaments_to_optimize: Vec<usize> = Vec::new();

            let map_filament_to_machine_filament =
                |filament_idx: i32, machine_filament_idx: i32, filament_map: &mut Vec<i32>, extruder_filament_count: &mut Vec<i32>| {
                    let machine_filament = &machine_filament_list[machine_filament_idx as usize];
                    filament_map[used_filaments[filament_idx as usize] as usize] = machine_filament.extruder_id; // set extruder id to filament map
                    extruder_filament_count[machine_filament.extruder_id as usize] += 1; // increase filament count in extruder
                };
            let unmap_filament_to_machine_filament = |_filament_idx: i32, machine_filament_idx: i32, extruder_filament_count: &mut Vec<i32>| {
                let machine_filament = &machine_filament_list[machine_filament_idx as usize];
                extruder_filament_count[machine_filament.extruder_id as usize] -= 1; // increase filament count in extruder
            };

            for idx in 0..map_to_machine_filament.len() {
                if map_to_machine_filament[idx] == INVALID_ID {
                    ungrouped_filaments.push(l_nodes[idx]);
                    continue;
                }
                let used_filament_idx = l_nodes[idx];
                let machine_filament_idx = r_nodes[map_to_machine_filament[idx] as usize];
                let machine_filament = &machine_filament_list[machine_filament_idx as usize];
                if machine_filament_set[machine_filament].len() > 1 && !unprintable_limit_indices.contains_key(&used_filament_idx) {
                    filaments_to_optimize.push(idx);
                }

                map_filament_to_machine_filament(used_filament_idx, machine_filament_idx, filament_map, extruder_filament_count);
            }
            // try to optimize the result
            for &idx in &filaments_to_optimize {
                let filament_idx = l_nodes[idx];
                let is_support_filament = used_filament_list[filament_idx as usize].usage_type == FilamentUsageType::SupportOnly;
                let old_machine_filament_idx = r_nodes[map_to_machine_filament[idx] as usize];
                let old_machine_filament = &machine_filament_list[old_machine_filament_idx as usize];

                unmap_filament_to_machine_filament(filament_idx, old_machine_filament_idx, extruder_filament_count);

                let optional_filaments = machine_filament_set[old_machine_filament].clone();

                // 第一阶段：找出所有满足容量约束的候选方案，并计算它们的偏好得分
                let mut valid_candidates: Vec<(i32, i32)> = Vec::new(); // 存储可用的机器耗材idx与评分
                for &machine_filament in &optional_filaments {
                    let new_extruder_id = machine_filament_list[machine_filament as usize].extruder_id;

                    // 计算新分配的偏好得分
                    let mut preference_score = 0;
                    let new_extruder_prefer_support = prefer_non_model_filament[new_extruder_id as usize];

                    // 如果是支撑材料且分配给了偏好支撑的喷嘴，给予奖励
                    if is_support_filament && new_extruder_prefer_support {
                        preference_score += SUPPORT_PREFER_SCORE; // 给予奖励
                    }

                    valid_candidates.push((machine_filament, preference_score));
                }
                // 第二阶段：确定最佳偏好得分
                let mut best_preference_score = 0;
                for candidate in &valid_candidates {
                    if candidate.1 >= best_preference_score {
                        best_preference_score = candidate.1;
                    }
                }

                // 第三阶段：匹配成本优先，成本相同时选择负载更均衡的方案。
                let mut best_candidate = -1;
                let mut best_gap = i32::MAX;
                let mut best_match_cost = f32::MAX;

                for candidate in &valid_candidates {
                    // 只考虑具有最佳偏好得分的候选方案
                    let machine_filament = candidate.0;
                    let score = candidate.1;
                    if score == best_preference_score {
                        let new_extruder_id = machine_filament_list[machine_filament as usize].extruder_id;
                        let new_gap = (extruder_filament_count[new_extruder_id as usize] + 1 - extruder_filament_count[(1 - new_extruder_id) as usize]).abs();
                        let match_cost = match_cost_matrix[filament_idx as usize][machine_filament as usize];

                        // 优先选择匹配成本更低的候选，成本相同时选择负载更均衡的候选。
                        if match_cost < best_match_cost || (match_cost == best_match_cost && new_gap < best_gap) {
                            best_match_cost = match_cost;
                            best_gap = new_gap;
                            best_candidate = machine_filament;
                        }
                    }
                }
                // 应用最佳选择
                if best_candidate != -1 {
                    map_filament_to_machine_filament(filament_idx, best_candidate, filament_map, extruder_filament_count);
                } else {
                    map_filament_to_machine_filament(filament_idx, old_machine_filament_idx, filament_map, extruder_filament_count);
                }
            }
            ungrouped_filaments
        };

        let mut group: Vec<i32> = vec![0; self.ctx.group_info.total_filament_num as usize];
        let mut ungrouped_filaments: Vec<i32>;

        let unlink_limits_full = build_unlink_limits(&l_nodes, &r_nodes, &|used_filament_idx: i32, machine_filament_idx: i32| {
            used_filament_list[used_filament_idx as usize].type_ == machine_filament_list[machine_filament_idx as usize].type_
                && used_filament_list[used_filament_idx as usize].is_support == machine_filament_list[machine_filament_idx as usize].is_support
                && is_extruder_filament_compatible(
                    &unprintable_limit_indices,
                    used_filament_idx,
                    machine_filament_list[machine_filament_idx as usize].extruder_id,
                )
        });

        {
            let mut s = MatchModeGroupSolver::new(&match_cost_matrix, &l_nodes, &r_nodes, &machine_filament_capacity, &unlink_limits_full);
            let solved = s.solve();
            ungrouped_filaments = optimize_map_to_machine_filament(&solved, &l_nodes, &r_nodes, &mut group, &mut extruder_filament_count);
            if ungrouped_filaments.is_empty() {
                return Ok(group);
            }
        }

        // additionally remove type limits
        {
            l_nodes = ungrouped_filaments.clone();
            let unlink_limits = build_unlink_limits(&l_nodes, &r_nodes, &|used_filament_idx: i32, machine_filament_idx: i32| {
                is_extruder_filament_compatible(
                    &unprintable_limit_indices,
                    used_filament_idx,
                    machine_filament_list[machine_filament_idx as usize].extruder_id,
                )
            });

            let mut s = MatchModeGroupSolver::new(&match_cost_matrix, &l_nodes, &r_nodes, &machine_filament_capacity, &unlink_limits);
            let solved = s.solve();
            ungrouped_filaments = optimize_map_to_machine_filament(&solved, &l_nodes, &r_nodes, &mut group, &mut extruder_filament_count);
            if ungrouped_filaments.is_empty() {
                return Ok(group);
            }
        }

        // remove all limits
        {
            l_nodes = ungrouped_filaments.clone();
            let mut s = MatchModeGroupSolver::new(&match_cost_matrix, &l_nodes, &r_nodes, &machine_filament_capacity, &HashMap::new());
            let solved = s.solve();
            let ret = optimize_map_to_machine_filament(&solved, &l_nodes, &r_nodes, &mut group, &mut extruder_filament_count);
            // FIDELITY-NOTE: the C++ reads `ret` (the ungrouped filaments) as a match result here; it is
            // empty whenever the unconstrained match places every filament.
            for idx in 0..ret.len() {
                if ret[idx] == INVALID_ID {
                    // assert(false);
                } else {
                    group[used_filaments[l_nodes[idx] as usize] as usize] = machine_filament_list[r_nodes[ret[idx] as usize] as usize].extruder_id;
                }
            }
        }

        Ok(group)
    }

    /// FilamentGroup.cpp:1408-1423 `std::vector<int> FilamentGroup::calc_filament_group_for_flush(int* cost)`
    pub fn calc_filament_group_for_flush(&mut self, cost: Option<&mut i32>) -> Result<Vec<i32>, FilamentGroupError> {
        let used_filaments = collect_sorted_used_filaments(&self.ctx.model_info.layer_filaments);

        let mut ret = self.calc_min_flush_group(cost)?;
        let mut memoryed_maps: Vec<Vec<i32>> = self.m_memoryed_groups.clone();
        memoryed_maps.insert(0, ret.clone());

        let mut used_filament_info: Vec<FilamentInfo> = Vec::new();
        for &f in &used_filaments {
            used_filament_info.push(self.ctx.model_info.filament_info[f as usize].clone());
        }

        ret = select_best_group_for_ams(
            &memoryed_maps,
            &self.ctx.nozzle_info.nozzle_list,
            &used_filaments,
            &used_filament_info,
            &self.ctx.machine_info.machine_filament_info,
            self.ctx.group_info.has_filament_switcher,
            20.0,
        );
        Ok(ret)
    }

    /// FilamentGroup.cpp:1425-1469 `std::vector<int> FilamentGroup::calc_filament_group_for_tpu(int *cost)`
    pub fn calc_filament_group_for_tpu(&mut self, _cost: Option<&mut i32>) -> Vec<i32> {
        let used_filaments = collect_sorted_used_filaments(&self.ctx.model_info.layer_filaments);
        let mut used_filament_list: Vec<FilamentInfo> = Vec::new();
        for &f in &used_filaments {
            used_filament_list.push(self.ctx.model_info.filament_info[f as usize].clone());
        }

        let extruder_count = self.ctx.nozzle_info.extruder_nozzle_list.len();
        let mut print_time_matrix: Vec<Vec<f32>> = vec![vec![0.0f32; extruder_count]; used_filaments.len()];
        for i in 0..used_filaments.len() {
            for j in 0..extruder_count {
                // ctx.speed_info.filament_print_time[used_filaments[i]][j]: both operator[] insert.
                print_time_matrix[i][j] = *self.ctx.speed_info.filament_print_time.entry(used_filaments[i] as i32).entry(j as i32) as f32;
                if self.ctx.nozzle_info.nozzle_list[j].volume_type == NozzleVolumeType::NvtTPUHighFlow {
                    //同时存在TPU High Flow喷嘴和其他类型喷嘴时，优先将耗材分配至TPU High Flow喷嘴
                    print_time_matrix[i][j] = (print_time_matrix[i][j] as f64 * 0.9) as f32;
                }
            }
        }

        let l_nodes: Vec<i32> = (0..used_filaments.len() as i32).collect();
        let r_nodes: Vec<i32> = (0..extruder_count as i32).collect();
        let machine_filament_capacity: Vec<i32> = vec![used_filaments.len() as i32, used_filaments.len() as i32];

        let mut unprintable_limit_indices: BTreeMap<i32, i32> = BTreeMap::new(); // key stores filament idx in used_filament, value stores unprintable extruder
        extract_unprintable_limit_indices(&self.ctx.model_info.unprintable_filaments, &used_filaments, &mut unprintable_limit_indices);
        unprintable_limit_indices = self.rebuild_unprintables(&used_filaments, &unprintable_limit_indices);

        let mut unlink_limits: HashMap<i32, Vec<i32>> = HashMap::with_capacity(used_filaments.len());
        for i in 0..used_filaments.len() as i32 {
            match unprintable_limit_indices.get(&i) {
                Some(&e) if !(e < 0 || e >= 2) => unlink_limits.entry(i).or_default().push(e),
                _ => continue,
            }
        }

        let mut s = MatchModeGroupSolver::new(&print_time_matrix, &l_nodes, &r_nodes, &machine_filament_capacity, &unlink_limits);
        let mut ret = s.solve();
        for idx in 0..ret.len() {
            if ret[idx] == INVALID_ID {
                // assert(false);
                ret[idx] = 1;
            }
        }
        let mut group: Vec<i32> = vec![self.ctx.machine_info.master_extruder_id; self.ctx.group_info.total_filament_num as usize];
        for i in 0..ret.len() {
            group[used_filaments[i] as usize] = ret[i];
        }
        group
    }

    /// FilamentGroup.cpp:1471-1508 `std::unordered_map<int, std::vector<int>> FilamentGroup::rebuild_nozzle_unprintables(
    /// const std::vector<unsigned int>& used_filaments, const std::unordered_map<int, std::vector<int>>& extruder_unprintables,
    /// const std::vector<int>& filament_volume_map)` (sorted used_filaments)
    pub fn rebuild_nozzle_unprintables(
        &mut self,
        used_filaments: &[u32],
        extruder_unprintables: &HashMap<i32, Vec<i32>>,
        filament_volume_map: &[i32],
    ) -> UnplaceableLimits {
        let mut nozzle_unprintables = UnplaceableLimits::new();

        for fidx in 0..used_filaments.len() {
            // NozzleVolumeType expected_volume = NozzleVolumeType(filament_volume_map[used_filaments[fidx]]);
            let expected_volume: i32 = filament_volume_map[used_filaments[fidx] as usize];
            let mut unexpected_extruders: Vec<i32> = Vec::new();
            if let Some(v) = extruder_unprintables.get(&(fidx as i32)) {
                unexpected_extruders = v.clone();
            }

            // ctx.model_info.unprintable_volumes[used_filaments[fidx]] (std::map operator[]: inserts)
            let unprintable_volumes = self.ctx.model_info.unprintable_volumes.entry(used_filaments[fidx] as i32).or_default().clone();

            let mut unprintable_nozzles: Vec<i32> = Vec::new();
            for nozzle_idx in 0..self.ctx.nozzle_info.nozzle_list.len() {
                let nozzle_info = &self.ctx.nozzle_info.nozzle_list[nozzle_idx];
                let filament_id = used_filaments[fidx] as usize;
                let diameter_mismatch = filament_id < self.ctx.model_info.selected_nozzle_diameters.len()
                    && !self.ctx.model_info.selected_nozzle_diameters[filament_id].is_empty()
                    && self.ctx.model_info.selected_nozzle_diameters[filament_id] != nozzle_info.diameter;

                // expected_volume != nvtHybrid (2) && expected_volume != nozzle_info.volume_type
                if unexpected_extruders.contains(&nozzle_info.extruder_id)
                    || (expected_volume != 2 && expected_volume != nozzle_info.volume_type as i32)
                    || !filament_diameter_matches_nozzle_volume(&self.ctx.model_info.filament_info[filament_id], nozzle_info.volume_type)
                    || unprintable_volumes.contains(&nozzle_info.volume_type)
                    || diameter_mismatch
                {
                    unprintable_nozzles.push(nozzle_idx as i32);
                }
            }
            if unprintable_nozzles.is_empty() {
                continue;
            }

            // sort_remove_duplicates(unprintable_nozzles);
            unprintable_nozzles.sort_unstable();
            unprintable_nozzles.dedup();
            *nozzle_unprintables.entry(fidx as i32) = unprintable_nozzles;
        }

        nozzle_unprintables
    }
}

/// FilamentGroup.cpp:1510-1525 `std::vector<int> calc_filament_group_for_match_multi_nozzle(const FilamentGroupContext& ctx)`
pub fn calc_filament_group_for_match_multi_nozzle(ctx: &FilamentGroupContext) -> Result<Vec<i32>, FilamentGroupError> {
    let mut fg1 = FilamentGroup::new(ctx);
    let filament_extruder_map = fg1.calc_filament_group_for_match(None)?;

    let mut new_ctx = ctx.clone();
    let used_filaments = collect_sorted_used_filaments(&ctx.model_info.layer_filaments);
    for idx in 0..used_filaments.len() {
        new_ctx.model_info.unprintable_filaments[(1 - filament_extruder_map[used_filaments[idx] as usize]) as usize].insert(used_filaments[idx] as i32);
    }
    // Finite "no extra limit" value, see calc_filament_group_for_manual_multi_nozzle: INT_MAX
    // overflows have_enough_size() and disables the per-nozzle capacity that spreads fixed nozzles.
    let group_cap = std::cmp::max(1, new_ctx.group_info.total_filament_num);
    let len = new_ctx.machine_info.max_group_size.len();
    new_ctx.machine_info.max_group_size = vec![group_cap; len];
    let mut fg = FilamentGroup::new(&new_ctx);
    fg.calc_filament_group_for_flush(None)
}

/// One loaded entry of calc_filament_group_for_nozzle_match (FilamentGroup.cpp:1533-1536 `struct LoadedNozzle`);
/// the pointer to the machine's loaded filament is its index in `nozzle_loaded_filaments`.
#[derive(Debug, Clone)]
struct LoadedNozzle {
    filament: usize,
    nozzle: mnu::NozzleInfo,
}

/// FilamentGroup.cpp:1576-1583 `auto diameter_matches = [&ctx](int filament_id, const MultiNozzleUtils::NozzleInfo &nozzle)`
fn nozzle_match_diameter_matches(ctx: &FilamentGroupContext, filament_id: i32, nozzle: &mnu::NozzleInfo) -> bool {
    if filament_id < 0 || filament_id >= ctx.model_info.selected_nozzle_diameters.len() as i32 {
        return true;
    }
    let selected = &ctx.model_info.selected_nozzle_diameters[filament_id as usize];
    selected.is_empty() || *selected == nozzle.diameter
}

/// FilamentGroup.cpp:1662-1679 `auto can_place_unloaded = [&ctx, &diameter_matches](int filament_id, const MultiNozzleUtils::NozzleInfo& nozzle)`
fn nozzle_match_can_place_unloaded(ctx: &FilamentGroupContext, filament_id: i32, nozzle: &mnu::NozzleInfo) -> bool {
    if !nozzle.installed {
        return false;
    }
    if !nozzle_match_diameter_matches(ctx, filament_id, nozzle) {
        return false;
    }
    if !filament_diameter_matches_nozzle_volume(&ctx.model_info.filament_info[filament_id as usize], nozzle.volume_type) {
        return false;
    }
    if nozzle.extruder_id >= 0
        && nozzle.extruder_id < ctx.model_info.unprintable_filaments.len() as i32
        && ctx.model_info.unprintable_filaments[nozzle.extruder_id as usize].contains(&filament_id)
    {
        return false;
    }
    if filament_id >= 0 && filament_id < ctx.group_info.filament_volume_map.len() as i32 {
        let expected_volume = ctx.group_info.filament_volume_map[filament_id as usize];
        if expected_volume != 2 && expected_volume != nozzle.volume_type as i32 {
            return false;
        }
    }
    if let Some(vols) = ctx.model_info.unprintable_volumes.get(&filament_id) {
        if vols.contains(&nozzle.volume_type) {
            return false;
        }
    }
    true
}

/// FilamentGroup.cpp:1681-1685 `auto is_fixed_nozzle = [&ctx](const MultiNozzleUtils::NozzleInfo& nozzle)`
fn nozzle_match_is_fixed_nozzle(ctx: &FilamentGroupContext, nozzle: &mnu::NozzleInfo) -> bool {
    if nozzle.group_id >= 0 && nozzle.group_id < ctx.nozzle_info.per_nozzle_capacity.len() as i32 {
        return ctx.nozzle_info.per_nozzle_capacity[nozzle.group_id as usize] <= 1;
    }
    true
}

/// FilamentGroup.cpp:1650-1658 `auto append_virtual_nozzle = [&ctx](const MultiNozzleUtils::NozzleInfo& source_nozzle)`
fn nozzle_match_append_virtual_nozzle(ctx: &mut FilamentGroupContext, source_nozzle: &mnu::NozzleInfo) -> i32 {
    let group_id = ctx.nozzle_info.nozzle_list.len() as i32;
    let mut virtual_nozzle = source_nozzle.clone();
    virtual_nozzle.group_id = group_id;
    virtual_nozzle.installed = true;
    ctx.nozzle_info.nozzle_list.push(virtual_nozzle);
    ctx.nozzle_info.per_nozzle_capacity.push(1);
    group_id
}

/// FilamentGroup.cpp:1687-1714 `auto assign_unmatched = [&](int filament_id)`
fn nozzle_match_assign_unmatched(
    ctx: &mut FilamentGroupContext,
    remaining_capacity: &mut HashMap<i32, i32>,
    filament_id: i32,
) -> Result<i32, FilamentGroupError> {
    let mut virtual_nozzle_source: Option<mnu::NozzleInfo> = None;
    for prefer_fixed in [true, false] {
        for ni in 0..ctx.nozzle_info.nozzle_list.len() {
            let nozzle = &ctx.nozzle_info.nozzle_list[ni];
            if nozzle_match_is_fixed_nozzle(ctx, nozzle) != prefer_fixed {
                continue;
            }
            if !nozzle_match_can_place_unloaded(ctx, filament_id, nozzle) {
                continue;
            }

            if virtual_nozzle_source.is_none() {
                virtual_nozzle_source = Some(nozzle.clone());
            }

            let cap = remaining_capacity.entry(nozzle.group_id).or_insert(0);
            if *cap <= 0 {
                continue;
            }
            *cap -= 1;
            return Ok(nozzle.group_id);
        }
    }

    // 有兼容喷嘴但真实容量已满时，追加同类型虚拟喷嘴，保持“一喷嘴一料”的分组语义。
    if let Some(src) = virtual_nozzle_source {
        return Ok(nozzle_match_append_virtual_nozzle(ctx, &src));
    }

    Err(runtime_error(
        "Nozzle assignment error: no compatible nozzle is available for the unmatched filament. Please check the nozzle setup or regroup.",
    ))
}

/// FilamentGroup.cpp:1527-1776 `std::vector<int> calc_filament_group_for_nozzle_match(FilamentGroupContext &ctx)`
pub fn calc_filament_group_for_nozzle_match(ctx: &mut FilamentGroupContext) -> Result<Vec<i32>, FilamentGroupError> {
    let used_filaments = collect_sorted_used_filaments(&ctx.model_info.layer_filaments);

    // 1、收集当前设备侧已经确认“喷嘴内有料”的入口，按 nozzle_id 聚合。
    let mut loaded_filament_map: HashMap<i32, Vec<usize>> = HashMap::new();
    for (li, loaded) in ctx.machine_info.nozzle_loaded_filaments.iter().enumerate() {
        if loaded.valid {
            loaded_filament_map.entry(loaded.nozzle_id).or_default().push(li);
        }
    }

    let mut loaded_nozzles: Vec<LoadedNozzle> = Vec::new();
    for nozzle in &ctx.nozzle_info.nozzle_list {
        let loaded = loaded_filament_map.get(&nozzle.group_id);
        if !nozzle.installed || loaded.is_none() {
            continue;
        }
        for &filament in loaded.unwrap() {
            loaded_nozzles.push(LoadedNozzle { filament, nozzle: nozzle.clone() });
        }
    }

    // 没有工程耗材或没有设备已装料信息时，match 模式没有可参考目标，退回省冲刷分组。
    if used_filaments.is_empty() || loaded_nozzles.is_empty() {
        let mut fg = FilamentGroup::new(ctx);
        return fg.calc_filament_group_for_flush(None);
    }

    // 2、构建匹配图节点：左侧是工程实际使用的耗材，右侧是设备侧已装料入口。
    let l_nodes: Vec<i32> = (0..used_filaments.len() as i32).collect();
    let r_nodes: Vec<i32> = (0..loaded_nozzles.len() as i32).collect();

    // 3、定义硬约束：材料类型、支撑属性、不可打印限制、喷嘴口径和喷嘴流量类型都兼容时，才允许连边。
    let can_match = |ctx: &FilamentGroupContext, used_filament_idx: i32, loaded_idx: i32| -> bool {
        let filament_id = used_filaments[used_filament_idx as usize] as i32;
        let model = &ctx.model_info.filament_info[filament_id as usize];
        let loaded = &ctx.machine_info.nozzle_loaded_filaments[loaded_nozzles[loaded_idx as usize].filament];
        let nozzle = &loaded_nozzles[loaded_idx as usize].nozzle;

        if model.type_ != loaded.base.type_ || model.is_support != loaded.base.is_support {
            return false;
        }

        if !nozzle_match_diameter_matches(ctx, filament_id, nozzle) {
            return false;
        }

        if !filament_diameter_matches_nozzle_volume(model, nozzle.volume_type) {
            return false;
        }

        if nozzle.extruder_id >= 0
            && nozzle.extruder_id < ctx.model_info.unprintable_filaments.len() as i32
            && ctx.model_info.unprintable_filaments[nozzle.extruder_id as usize].contains(&filament_id)
        {
            return false;
        }

        if filament_id >= 0 && filament_id < ctx.group_info.filament_volume_map.len() as i32 {
            let expected_volume = ctx.group_info.filament_volume_map[filament_id as usize];
            if expected_volume != 2 && expected_volume != nozzle.volume_type as i32 {
                return false;
            }
        }

        if let Some(vols) = ctx.model_info.unprintable_volumes.get(&filament_id) {
            if vols.contains(&nozzle.volume_type) {
                return false;
            }
        }

        true
    };

    // 4、构造费用矩阵和禁连表：同一固定喷嘴的后续候选增加微小代价偏置，优先保留喷嘴内当前耗材。
    let mut nozzle_secondary_tray_penalty: Vec<f32> = vec![0.0f32; loaded_nozzles.len()];
    {
        let mut seen_fixed_nozzle_groups: HashSet<i32> = HashSet::new();
        for j in 0..loaded_nozzles.len() {
            let group_id = loaded_nozzles[j].nozzle.group_id;
            let cap = if group_id >= 0 && group_id < ctx.nozzle_info.per_nozzle_capacity.len() as i32 {
                ctx.nozzle_info.per_nozzle_capacity[group_id as usize]
            } else {
                1
            };
            if cap <= 1 && !seen_fixed_nozzle_groups.insert(group_id) {
                nozzle_secondary_tray_penalty[j] = 1.0;
            }
        }
    }
    let mut cost_matrix: Vec<Vec<f32>> = vec![vec![0.0f32; loaded_nozzles.len()]; used_filaments.len()];
    let mut unlink_limits: HashMap<i32, Vec<i32>> = HashMap::new();
    for i in 0..used_filaments.len() {
        let filament_id = used_filaments[i] as i32;
        let model = &ctx.model_info.filament_info[filament_id as usize];
        // 读取工程耗材小类，缺失时仅按颜色匹配。
        let model_filament_id: String = if (filament_id as usize) < ctx.model_info.filament_ids.len() {
            ctx.model_info.filament_ids[filament_id as usize].clone()
        } else {
            String::new()
        };
        for j in 0..loaded_nozzles.len() {
            if !can_match(ctx, i as i32, j as i32) {
                unlink_limits.entry(i as i32).or_default().push(j as i32);
                continue;
            }
            let loaded = &ctx.machine_info.nozzle_loaded_filaments[loaded_nozzles[j].filament];
            let cost = calc_color_distance(
                &RGBColor::new(model.color.r, model.color.g, model.color.b),
                &RGBColor::new(loaded.base.color.r, loaded.base.color.g, loaded.base.color.b),
            );
            // int + float is a float addition
            cost_matrix[i][j] = calc_match_mode_cost(cost, &model_filament_id, &loaded.filament_id, used_filaments.len() as i32) as f32
                + nozzle_secondary_tray_penalty[j];
        }
    }

    // 6、执行匹配：每个已装料槽位节点自身容量为 1（一对一匹配）。
    let v_capacity: Vec<i32> = vec![1; loaded_nozzles.len()];
    let mut solver = MatchModeGroupSolver::new(&cost_matrix, &l_nodes, &r_nodes, &v_capacity, &unlink_limits);
    let match_ = solver.solve();

    // 5、把匹配结果转换成“工程耗材 -> nozzle group_id” (auto build_match_result = [...](const std::vector<int> &match)).
    let mut group: Vec<i32> = vec![ctx.machine_info.master_extruder_id; ctx.group_info.total_filament_num as usize];
    let mut remaining_capacity: HashMap<i32, i32> = HashMap::new();

    // 5.1 初始化每个真实喷嘴的剩余容量。
    for nozzle in &ctx.nozzle_info.nozzle_list {
        let mut cap = 1;
        if nozzle.group_id >= 0 && nozzle.group_id < ctx.nozzle_info.per_nozzle_capacity.len() as i32 {
            cap = ctx.nozzle_info.per_nozzle_capacity[nozzle.group_id as usize];
        }
        remaining_capacity.insert(nozzle.group_id, cap);
    }

    // 5.5 应用匹配结果：同一固定喷嘴优先保留喷嘴上当前已进料的耗材。
    let mut matched_indices: Vec<usize> = Vec::new();
    let mut unmatched_indices: Vec<usize> = Vec::new();
    for idx in 0..match_.len() {
        if match_[idx] == INVALID_ID {
            unmatched_indices.push(idx);
        } else {
            matched_indices.push(idx);
        }
    }

    // std::stable_sort with the comparator below (Rust's sort_by is stable).
    let less = |a: usize, b: usize| -> bool {
        let pen_a = nozzle_secondary_tray_penalty[match_[a] as usize];
        let pen_b = nozzle_secondary_tray_penalty[match_[b] as usize];
        if pen_a != pen_b {
            return pen_a < pen_b;
        }
        let cost_a = cost_matrix[a][match_[a] as usize];
        let cost_b = cost_matrix[b][match_[b] as usize];
        if cost_a != cost_b {
            return cost_a < cost_b;
        }
        match_[a] < match_[b]
    };
    matched_indices.sort_by(|&a, &b| {
        if less(a, b) {
            std::cmp::Ordering::Less
        } else if less(b, a) {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Equal
        }
    });

    for &idx in &matched_indices {
        let filament_id = used_filaments[idx] as usize;
        let matched_group_id = loaded_nozzles[match_[idx] as usize].nozzle.group_id;
        let cap = remaining_capacity.entry(matched_group_id).or_insert(0);
        if *cap > 0 {
            *cap -= 1;
            group[filament_id] = matched_group_id;
        } else {
            unmatched_indices.push(idx);
        }
    }

    for &idx in &unmatched_indices {
        let filament_id = used_filaments[idx] as i32;
        group[filament_id as usize] = nozzle_match_assign_unmatched(ctx, &mut remaining_capacity, filament_id)?;
    }
    Ok(group)
}

/// FilamentGroup.hpp:201-205 `struct FilamentPlanRes`
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct FilamentPlanRes {
    pub fil_order: Vec<i32>,
    pub fil_nozzle_match: Vec<i32>,
}

/// FilamentGroup.cpp:1778-1907 `std::vector<FilamentPlanRes> plan_filament_nozzle_mapping_and_order(const FilamentGroupContext &ctx)`
pub fn plan_filament_nozzle_mapping_and_order(ctx: &FilamentGroupContext) -> Result<Vec<FilamentPlanRes>, FilamentGroupError> {
    let mut res: Vec<FilamentPlanRes> = Vec::new();

    // 右节点：喷嘴
    let mut r_nodes: Vec<i32> = vec![-1; ctx.nozzle_info.nozzle_list.len()];
    let mut initial_nozzle = ctx.nozzle_info.nozzle_status.clone();
    for r_id in 0..r_nodes.len() as i32 {
        if initial_nozzle.count(r_id) != 0 {
            r_nodes[r_id as usize] = *initial_nozzle.entry(r_id);
        }
    }

    let mut r_nodes_group: Vec<i32> = vec![-1; ctx.nozzle_info.nozzle_list.len()];
    for nozzle_info in &ctx.nozzle_info.nozzle_list {
        r_nodes_group[nozzle_info.group_id as usize] = nozzle_info.extruder_id;
    }

    let layer_nums = ctx.model_info.layer_filaments.len() as i32;
    let flush_matrix = &ctx.model_info.flush_matrix;

    let mut prev_layer_last_nozzle_id: i32 = -1;
    let mut used_prev_layer_last_nozzle = false;

    // 每一层，材料到喷嘴的匹配 filament->nozzle
    let mut layer_fil_nozzle_match: Vec<i32> = vec![0; ctx.model_info.filament_info.len()];
    for i in 0..layer_nums {
        // 左节点：材料
        let layer_filaments = &ctx.model_info.layer_filaments[i as usize];
        let mut l_nodes: Vec<i32> = layer_filaments.iter().map(|&f| f as i32).collect();
        l_nodes.sort_unstable();
        l_nodes.dedup();

        if l_nodes.is_empty() {
            res.push(FilamentPlanRes { fil_order: Vec::new(), fil_nozzle_match: Vec::new() });
            continue;
        }

        // 每一层，喷嘴内使用的材料队列
        let mut nozzle_fil_deq: Vec<VecDeque<i32>> = vec![VecDeque::new(); r_nodes.len()];

        // 构建喷嘴状态到索引的反向映射 (std::unordered_map<int, size_t>, only searched)
        let mut filament_to_nozzle: HashMap<i32, usize> = HashMap::with_capacity(r_nodes.len());
        for noz_id in 0..r_nodes.len() {
            if r_nodes[noz_id] >= 0 {
                filament_to_nozzle.insert(r_nodes[noz_id], noz_id);
            }
        }

        let epochs = (l_nodes.len() as f64 / r_nodes.len() as f64).ceil() as i32;
        for _j in 0..epochs {
            // 1、过滤掉材料与喷嘴内状态一致的节点
            let mut remaining_l_nodes: Vec<i32> = Vec::with_capacity(l_nodes.len());
            let mut remaining_r_nodes: Vec<i32> = Vec::with_capacity(r_nodes.len());
            let mut remaining_r_nodes_to_origin: Vec<i32> = Vec::with_capacity(r_nodes.len());
            let mut used_r_nodes: Vec<bool> = vec![false; r_nodes.len()];

            for &f_id in &l_nodes {
                if let Some(&noz_id) = filament_to_nozzle.get(&f_id) {
                    layer_fil_nozzle_match[f_id as usize] = noz_id as i32;
                    nozzle_fil_deq[noz_id].push_back(f_id);
                    used_prev_layer_last_nozzle = noz_id as i64 == prev_layer_last_nozzle_id as i64;
                    used_r_nodes[noz_id] = true;
                } else {
                    remaining_l_nodes.push(f_id);
                }
            }
            l_nodes = remaining_l_nodes;

            for r_id in 0..r_nodes.len() {
                if !used_r_nodes[r_id] {
                    remaining_r_nodes.push(r_nodes[r_id]);
                    remaining_r_nodes_to_origin.push(r_id as i32);
                }
            }

            // 2、剩下的节点跑费用流
            let mut s = GroupMinCostFlowSolver::new(flush_matrix, &l_nodes, &remaining_r_nodes, &r_nodes_group, &HashMap::new(), &HashMap::new());
            let match_ = s.solve();
            let mut write = 0usize;
            for l_id in 0..l_nodes.len() {
                if match_[l_id] >= 0 && (match_[l_id] as usize) < remaining_r_nodes.len() {
                    let noz_id = remaining_r_nodes_to_origin[match_[l_id] as usize];
                    let filament_id = l_nodes[l_id];
                    layer_fil_nozzle_match[filament_id as usize] = noz_id;
                    r_nodes[noz_id as usize] = filament_id;
                    nozzle_fil_deq[noz_id as usize].push_back(filament_id);
                    // 更新反向映射
                    filament_to_nozzle.insert(filament_id, noz_id as usize);
                } else {
                    l_nodes[write] = l_nodes[l_id];
                    write += 1;
                }
            }
            l_nodes.truncate(write);
        }

        // 排序
        let mut start_extruder = 0i32;
        let mut start_nozzle = 0i32;
        if used_prev_layer_last_nozzle {
            start_extruder = ctx.nozzle_info.nozzle_list[prev_layer_last_nozzle_id as usize].extruder_id;
            start_nozzle = prev_layer_last_nozzle_id;
        }

        let mut used_nozzle_deq: VecDeque<i32> = VecDeque::new();
        let extruder_count = ctx.nozzle_info.extruder_nozzle_list.len() as i32;
        for m in 0..extruder_count {
            let cur_extruder = (m + start_extruder) % extruder_count;
            // ctx.nozzle_info.extruder_nozzle_list.at(cur_extruder)
            let Some(nozzles) = ctx.nozzle_info.extruder_nozzle_list.get(&cur_extruder) else {
                return Err(FilamentGroupError::OutOfRange("map::at:  key not found".to_string()));
            };
            for &noz_id in nozzles {
                if nozzle_fil_deq[noz_id as usize].is_empty() {
                    continue;
                }
                if noz_id == start_nozzle {
                    used_nozzle_deq.push_front(noz_id);
                } else {
                    used_nozzle_deq.push_back(noz_id);
                }
            }
        }

        let mut layer_fil_order: Vec<i32> = Vec::new();
        for &noz_id in &used_nozzle_deq {
            let deq = &nozzle_fil_deq[noz_id as usize];
            layer_fil_order.reserve(deq.len());
            layer_fil_order.extend(deq.iter().copied());

            prev_layer_last_nozzle_id = noz_id;
        }

        let layer_pan = FilamentPlanRes { fil_order: layer_fil_order, fil_nozzle_match: layer_fil_nozzle_match.clone() };
        res.push(layer_pan);
    }

    Ok(res)
}

/// FilamentGroup.cpp:1909-1921 `std::vector<int> calc_filament_group_for_manual_multi_nozzle(const std::vector<int>& filament_map_manual,
/// const FilamentGroupContext& ctx)`
pub fn calc_filament_group_for_manual_multi_nozzle(filament_map_manual: &[i32], ctx: &FilamentGroupContext) -> Result<Vec<i32>, FilamentGroupError> {
    let mut new_ctx = ctx.clone();
    let used_filaments = collect_sorted_used_filaments(&ctx.model_info.layer_filaments);
    for idx in 0..used_filaments.len() {
        new_ctx.model_info.unprintable_filaments[(1 - filament_map_manual[used_filaments[idx] as usize]) as usize].insert(used_filaments[idx] as i32);
    }

    let group_cap = std::cmp::max(1, new_ctx.group_info.total_filament_num);
    let len = new_ctx.machine_info.max_group_size.len();
    new_ctx.machine_info.max_group_size = vec![group_cap; len];
    let mut fg = FilamentGroup::new(&new_ctx);
    fg.calc_filament_group_for_flush(None)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_evaluate_score() {
        assert_eq!(evaluate_score(100.0, 5.0, false), 100.0);
        assert_eq!(evaluate_score(100.0, 5.0, true), 100.0 * FLUSH_VOLUME_TO_SCORE + 5.0);
    }

    #[test]
    fn test_collect_sorted_used_filaments() {
        let lf = vec![vec![3u32, 1], vec![2, 3], vec![]];
        assert_eq!(collect_sorted_used_filaments(&lf), vec![1, 2, 3]);
    }

    #[test]
    fn test_memoryed_group_heap_order() {
        // std::greater: the top is the highest prefer level, then the lowest cost.
        let mut h = MemoryedGroupHeap::new();
        for (c, p) in [(5.0, 1), (3.0, 1), (9.0, 2), (1.0, 0), (3.0, 2)] {
            h.push(MemoryedGroup::new(vec![], c, p));
        }
        let mut out = Vec::new();
        while !h.empty() {
            out.push((h.top().cost, h.top().prefer_level));
            h.pop();
        }
        assert_eq!(out, vec![(3.0, 2), (9.0, 2), (3.0, 1), (5.0, 1), (1.0, 0)]);
    }

    #[test]
    fn test_update_memoryed_groups_keeps_highest_prefer() {
        let mut heap = MemoryedGroupHeap::new();
        update_memoryed_groups(&MemoryedGroup::new(vec![0], 10.0, 1), 0.0, 0.0, &mut heap);
        update_memoryed_groups(&MemoryedGroup::new(vec![1], 5.0, 2), 0.0, 0.0, &mut heap);
        update_memoryed_groups(&MemoryedGroup::new(vec![2], 1.0, 1), 0.0, 0.0, &mut heap);
        assert_eq!(heap.size(), 1);
        assert_eq!(heap.top().group, vec![1]);
    }

    #[test]
    fn test_fnv_hash_nozzle_salts() {
        let a = fnv_hash_nozzle("0.4", 0, 0, -1, -1);
        let b = fnv_hash_nozzle("0.4", 0, 0, 3, -1);
        let c = fnv_hash_nozzle("0.4", 0, 1, -1, -1);
        assert_ne!(a, b);
        assert_ne!(a, c);
    }
}
