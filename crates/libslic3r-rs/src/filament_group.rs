//! Port of `FilamentGroup.cpp` / `FilamentGroup.hpp`.
//!
//! C++ Reference:
//! - src/libslic3r/FilamentGroup.cpp
//! - src/libslic3r/FilamentGroup.hpp
//!
//! Faithful 1:1 line-by-line translation. `coord_t` -> `i64`, `coordf_t` -> `f64`.
//!
//! Blocked symbols (NOT ported — call genuinely-unported dependencies):
//! - `KMediods::cluster_small_data`: requires
//!   `get_estimate_extruder_filament_change_count` (declared blocked in
//!   `filament_group_utils.rs` — needs `MultiNozzleUtils` change-count helpers
//!   that are not yet ported).
//! - `FilamentGroup::calc_min_flush_group`,
//!   `FilamentGroup::calc_min_flush_group_by_enum`,
//!   `FilamentGroup::calc_min_flush_group_by_pam2`,
//!   `FilamentGroup::calc_filament_group_for_flush`,
//!   `FilamentGroup::calc_filament_group`: all require
//!   `reorder_filaments_for_minimum_flush_volume` (ToolOrderUtils.cpp — not yet
//!   ported to the Rust `gcode::tool_order_utils` module, which only contains the
//!   flow solvers).
//! - `plan_filament_nozzle_mapping_and_order`: requires `GroupMinCostFlowSolver`
//!   (ToolOrderUtils.cpp — not yet ported).
//!
//! Everything else (the data structures, evaluators, `select_best_group_for_ams`,
//! `update_memoryed_groups`, `collect_sorted_used_filaments`, the full `KMediods2`,
//! the tractable `KMediods` methods, the `FilamentGroup` match/tpu/merge helpers,
//! and the multi-nozzle MCMF/PAM solvers) is ported faithfully.

// Several file-scope static helpers (`change_memoryed_heaps_to_arrays`,
// `get_merged_filament_map`, `fnv_hash_two_ints`, `evaluate_score`) and a few
// `FilamentGroup` helper methods are ported faithfully but are currently reachable
// only through *blocked* callers (see module docs), so they read as dead code until
// those callers are ported. Allowed here to keep the build warning-clean.
#![allow(dead_code)]

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet, BinaryHeap, HashMap};

use crate::extruder::NozzleVolumeType;
use crate::filament_group_utils::{
    extract_unprintable_limit_indices_map, ErrorCode, FilamentGroupException, FilamentInfo,
    FilamentUsageType, MachineFilamentInfo,
};
use crate::flush_vol_predictor::flush_predict::{calc_color_distance, RGBColor};
use crate::gcode::tool_order_utils::{FlushMatrix, MatchModeGroupSolver, INVALID_ID};
use crate::multi_nozzle_utils::NozzleInfo;

// FilamentGroup.hpp:14
pub const DEFAULT_CLUSTER_SIZE: i32 = 16;

// FilamentGroup.hpp:16
pub const ABSOLUTE_FLUSH_GAP_TOLERANCE: i32 = 10;

// FilamentGroup.cpp:13
pub const GOLDEN_RATIO_32: u32 = 0x9e3779b9;

// FilamentGroup.hpp:22
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FGStrategy {
    // FilamentGroup.hpp:23
    BestCost,
    // FilamentGroup.hpp:24
    BestFit,
}

// FilamentGroup.hpp:27
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FGMode {
    // FilamentGroup.hpp:28
    FlushMode,
    // FilamentGroup.hpp:29
    MatchMode,
}

// =============================================================================
// FilamentGroupUtils namespace items defined inline in FilamentGroup.hpp:34-70
// =============================================================================

// FilamentGroup.hpp:55
/// C++ `struct MemoryedGroup`.
#[derive(Debug, Clone, Default)]
pub struct MemoryedGroup {
    // FilamentGroup.hpp:62
    pub cost: i32,
    // FilamentGroup.hpp:63
    pub prefer_level: i32,
    // FilamentGroup.hpp:64
    pub group: Vec<i32>,
}

impl MemoryedGroup {
    // FilamentGroup.hpp:57
    /// `MemoryedGroup(const std::vector<int>& group_, const int cost_, const int prefer_level_)`
    pub fn new(group: Vec<i32>, cost: i32, prefer_level: i32) -> Self {
        MemoryedGroup {
            group,
            cost,
            prefer_level,
        }
    }

    // FilamentGroup.hpp:58
    /// `bool operator>(const MemoryedGroup& other) const`
    pub fn gt(&self, other: &MemoryedGroup) -> bool {
        // FilamentGroup.hpp:59
        self.prefer_level < other.prefer_level
            || (self.prefer_level == other.prefer_level && self.cost > other.cost)
    }
}

// FilamentGroup.hpp:67
//   using MemoryedGroupHeap = std::priority_queue<MemoryedGroup, std::vector<MemoryedGroup>,
//                                                  std::greater<MemoryedGroup>>;
//
// C++ `std::greater<MemoryedGroup>` calls `operator>`, so a `std::priority_queue`
// with `std::greater` pops the element that is "smallest" under `operator>`, i.e.
// the one for which no other element compares "greater-than" it. We reproduce that
// behaviour with a `BinaryHeap` over a wrapper whose `Ord` is reversed relative to
// `MemoryedGroup::gt`, so that `peek()`/`pop()` yields the same element `top()` does
// in C++.
#[derive(Debug, Clone)]
pub struct HeapOrderedGroup(pub MemoryedGroup);

impl PartialEq for HeapOrderedGroup {
    fn eq(&self, other: &Self) -> bool {
        // equal iff neither is "greater" than the other
        !self.0.gt(&other.0) && !other.0.gt(&self.0)
    }
}
impl Eq for HeapOrderedGroup {}

impl Ord for HeapOrderedGroup {
    fn cmp(&self, other: &Self) -> Ordering {
        // std::priority_queue<.., std::greater<>> keeps the element `x` such that
        // for all `y`, `x > y` is false (the minimum under `operator>`). A Rust
        // `BinaryHeap` is a max-heap: `peek()` returns the greatest under `Ord`.
        // So define `Ord` such that the C++ `top()` element is the greatest here:
        //   self is "greater" (closer to top) when other.gt(self) is true.
        if self.0.gt(&other.0) {
            // self > other (under operator>) => self is further from top => smaller
            Ordering::Less
        } else if other.0.gt(&self.0) {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    }
}
impl PartialOrd for HeapOrderedGroup {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// C++ `MemoryedGroupHeap` (priority_queue with `std::greater`).
pub type MemoryedGroupHeap = BinaryHeap<HeapOrderedGroup>;

// FilamentGroup.cpp:170
/// `void FilamentGroupUtils::update_memoryed_groups(const MemoryedGroup& item, const double gap_threshold, MemoryedGroupHeap& groups)`
pub fn update_memoryed_groups(
    item: &MemoryedGroup,
    gap_threshold: f64,
    groups: &mut MemoryedGroupHeap,
) {
    // FilamentGroup.cpp:172
    // auto emplace_if_accepatle = [gap_threshold](MemoryedGroupHeap& heap, const MemoryedGroup& elem, const MemoryedGroup& best)
    let emplace_if_accepatle =
        |heap: &mut MemoryedGroupHeap, elem: &MemoryedGroup, best: &MemoryedGroup| {
            // FilamentGroup.cpp:173
            if best.cost == 0 {
                // FilamentGroup.cpp:174
                if (elem.cost - best.cost).abs() <= ABSOLUTE_FLUSH_GAP_TOLERANCE {
                    // FilamentGroup.cpp:175
                    heap.push(HeapOrderedGroup(elem.clone()));
                }
                // FilamentGroup.cpp:176
                return;
            }
            // FilamentGroup.cpp:178
            let gap_rate = ((elem.cost - best.cost).abs()) as f64 / (best.cost) as f64;
            // FilamentGroup.cpp:179
            if gap_rate <= gap_threshold {
                // FilamentGroup.cpp:180
                heap.push(HeapOrderedGroup(elem.clone()));
            }
        };

    // FilamentGroup.cpp:183
    if groups.is_empty() {
        // FilamentGroup.cpp:184
        groups.push(HeapOrderedGroup(item.clone()));
    } else {
        // FilamentGroup.cpp:187
        let top = groups.peek().unwrap().0.clone();
        // FilamentGroup.cpp:188
        // we only memory items with the highest prefer level
        // FilamentGroup.cpp:189
        if top.prefer_level > item.prefer_level {
            // FilamentGroup.cpp:190
            return;
        }
        // FilamentGroup.cpp:191
        else if top.prefer_level == item.prefer_level {
            // FilamentGroup.cpp:192
            if top.cost <= item.cost {
                // FilamentGroup.cpp:193
                emplace_if_accepatle(groups, item, &top);
            }
            // FilamentGroup.cpp:195
            // find a group with lower cost, rebuild the heap
            else {
                // FilamentGroup.cpp:197
                let mut new_heap: MemoryedGroupHeap = MemoryedGroupHeap::new();
                // FilamentGroup.cpp:198
                new_heap.push(HeapOrderedGroup(item.clone()));
                // FilamentGroup.cpp:199
                while !groups.is_empty() {
                    // FilamentGroup.cpp:200
                    let top = groups.peek().unwrap().0.clone();
                    // FilamentGroup.cpp:201
                    groups.pop();
                    // FilamentGroup.cpp:202
                    emplace_if_accepatle(&mut new_heap, &top, item);
                }
                // FilamentGroup.cpp:204
                *groups = new_heap;
            }
        }
        // FilamentGroup.cpp:207
        // find a group with the higher prefer level, rebuild the heap
        else {
            // FilamentGroup.cpp:209
            *groups = MemoryedGroupHeap::new();
            // FilamentGroup.cpp:210
            groups.push(HeapOrderedGroup(item.clone()));
        }
    }
}

// =============================================================================
// FilamentGroupContext (FilamentGroup.hpp:72-111)
// =============================================================================

// FilamentGroup.hpp:74
#[derive(Debug, Clone, Default)]
pub struct ModelInfo {
    // FilamentGroup.hpp:75
    pub flush_matrix: Vec<FlushMatrix>,
    // FilamentGroup.hpp:76
    pub layer_filaments: Vec<Vec<u32>>,
    // FilamentGroup.hpp:77
    pub filament_info: Vec<FilamentInfo>,
    // FilamentGroup.hpp:78
    pub filament_ids: Vec<String>,
    // FilamentGroup.hpp:79
    pub unprintable_filaments: Vec<BTreeSet<i32>>,
    // FilamentGroup.hpp:80
    pub unprintable_volumes: BTreeMap<i32, BTreeSet<NozzleVolumeType>>,
}

// FilamentGroup.hpp:83
#[derive(Debug, Clone)]
pub struct GroupInfo {
    // FilamentGroup.hpp:84
    pub total_filament_num: i32,
    // FilamentGroup.hpp:85
    pub max_gap_threshold: f64,
    // FilamentGroup.hpp:86
    pub mode: FGMode,
    // FilamentGroup.hpp:87
    pub strategy: FGStrategy,
    // FilamentGroup.hpp:88
    pub ignore_ext_filament: bool,
    // FilamentGroup.hpp:89
    pub filament_volume_map: Vec<i32>,
}

impl Default for GroupInfo {
    fn default() -> Self {
        GroupInfo {
            total_filament_num: 0,
            max_gap_threshold: 0.0,
            mode: FGMode::FlushMode,
            strategy: FGStrategy::BestCost,
            ignore_ext_filament: false,
            filament_volume_map: Vec::new(),
        }
    }
}

// FilamentGroup.hpp:92
#[derive(Debug, Clone, Default)]
pub struct MachineInfo {
    // FilamentGroup.hpp:93
    pub max_group_size: Vec<i32>,
    // FilamentGroup.hpp:94
    pub machine_filament_info: Vec<Vec<MachineFilamentInfo>>,
    // FilamentGroup.hpp:95
    pub prefer_non_model_filament: Vec<bool>,
    // FilamentGroup.hpp:96
    pub master_extruder_id: i32,
}

// FilamentGroup.hpp:99
#[derive(Debug, Clone, Default)]
pub struct SpeedInfo {
    // FilamentGroup.hpp:100
    pub filament_print_time: HashMap<i32, HashMap<i32, f64>>,
    // FilamentGroup.hpp:101
    pub extruder_change_time: f64,
    // FilamentGroup.hpp:102
    pub filament_change_time: f64,
    // FilamentGroup.hpp:103
    pub group_with_time: bool,
}

// FilamentGroup.hpp:106
#[derive(Debug, Clone, Default)]
pub struct NozzleInfoStruct {
    // FilamentGroup.hpp:107
    pub extruder_nozzle_list: BTreeMap<i32, Vec<i32>>,
    // FilamentGroup.hpp:108
    pub nozzle_list: Vec<NozzleInfo>,
    // FilamentGroup.hpp:109
    pub nozzle_status: HashMap<i32, i32>,
}

// FilamentGroup.hpp:72
#[derive(Debug, Clone, Default)]
pub struct FilamentGroupContext {
    // FilamentGroup.hpp:81
    pub model_info: ModelInfo,
    // FilamentGroup.hpp:90
    pub group_info: GroupInfo,
    // FilamentGroup.hpp:97
    pub machine_info: MachineInfo,
    // FilamentGroup.hpp:104
    pub speed_info: SpeedInfo,
    // FilamentGroup.hpp:110
    pub nozzle_info: NozzleInfoStruct,
}

// =============================================================================
// File-scope statics and helpers
// =============================================================================

// FilamentGroup.cpp:16
/// `static void change_memoryed_heaps_to_arrays(MemoryedGroupHeap& heap, const int total_filament_num, const std::vector<unsigned int>& used_filaments, std::vector<std::vector<int>>& arrs)`
fn change_memoryed_heaps_to_arrays(
    heap: &mut MemoryedGroupHeap,
    total_filament_num: i32,
    used_filaments: &[u32],
    arrs: &mut Vec<Vec<i32>>,
) {
    // FilamentGroup.cpp:18
    // switch the label idx
    // FilamentGroup.cpp:19
    arrs.clear();
    // FilamentGroup.cpp:20
    while !heap.is_empty() {
        // FilamentGroup.cpp:21
        let top = heap.peek().unwrap().0.clone();
        // FilamentGroup.cpp:22
        heap.pop();
        // FilamentGroup.cpp:23
        let mut labels_tmp: Vec<i32> = vec![0; total_filament_num as usize];
        // FilamentGroup.cpp:24
        for idx in 0..top.group.len() {
            // FilamentGroup.cpp:25
            labels_tmp[used_filaments[idx] as usize] = top.group[idx];
        }
        // FilamentGroup.cpp:26
        arrs.push(labels_tmp);
    }
}

// FilamentGroup.cpp:30
/// `static std::unordered_map<int, int> get_merged_filament_map(const std::unordered_map<int, std::vector<int>>& merged_filaments)`
fn get_merged_filament_map(merged_filaments: &HashMap<i32, Vec<i32>>) -> HashMap<i32, i32> {
    // FilamentGroup.cpp:32
    let mut filament_merge_map: HashMap<i32, i32> = HashMap::new();
    // FilamentGroup.cpp:33
    for elem in merged_filaments.iter() {
        // FilamentGroup.cpp:34
        for &f in elem.1.iter() {
            // FilamentGroup.cpp:35
            // traverse filaments in merged group
            // FilamentGroup.cpp:36
            filament_merge_map.insert(f, *elem.0);
        }
    }
    // FilamentGroup.cpp:39
    filament_merge_map
}

// FilamentGroup.cpp:42
/// `static uint64_t fnv_hash_two_ints(const int a, const int b)`
fn fnv_hash_two_ints(a: i32, b: i32) -> u64 {
    // FilamentGroup.cpp:44
    const FNV_OFFSET_BASIS: u64 = 14695981039346656037;
    // FilamentGroup.cpp:45
    const FNV_PRIME: u64 = 1099511628211;
    // FilamentGroup.cpp:46
    const SALT_A: u64 = 0xA5A5A5A5A5A5A5A5;
    // FilamentGroup.cpp:47
    const SALT_B: u64 = 0x5A5A5A5A5A5A5A5A;

    // FilamentGroup.cpp:49
    let mut h: u64 = FNV_OFFSET_BASIS;
    // FilamentGroup.cpp:50
    h ^= (a as u64).wrapping_add(SALT_A);
    // FilamentGroup.cpp:51
    h = h.wrapping_mul(FNV_PRIME);
    // FilamentGroup.cpp:52
    h ^= (b as u64).wrapping_add(SALT_B);
    // FilamentGroup.cpp:53
    h = h.wrapping_mul(FNV_PRIME);

    // FilamentGroup.cpp:55
    h
}

// FilamentGroup.cpp:58
/// `static double evaluate_score(const double flush, const double time, const bool with_time = false)`
fn evaluate_score(flush: f64, time: f64, with_time: bool) -> f64 {
    // FilamentGroup.cpp:59
    if !with_time {
        return flush;
    }

    // FilamentGroup.cpp:61
    let approx_density = 1.26; //   g/cm^3
                              // FilamentGroup.cpp:62
    let approx_flush_speed = 180.0; //   s/g
                                    // FilamentGroup.cpp:63
    let correction_factor = 2.0;
    // FilamentGroup.cpp:64
    let flush_score = flush * approx_density * approx_flush_speed * correction_factor / 1000.0;
    // FilamentGroup.cpp:65
    flush_score + time
}

// FilamentGroup.cpp:68-82 (doc comment)
/// Select the group that best fit the filaments in AMS.
///
/// Calculate the total color distance between the grouping results and the AMS
/// filaments through minimum cost maximum flow. Only those with a distance
/// difference within the threshold are considered valid.
// FilamentGroup.cpp:83
/// `std::vector<int> select_best_group_for_ams(...)`
#[allow(clippy::too_many_arguments)]
pub fn select_best_group_for_ams(
    filament_to_nozzles: &[Vec<i32>],
    nozzle_list: &[NozzleInfo],
    used_filaments: &[u32],
    used_filament_info: &[FilamentInfo],
    machine_filament_info_: &[Vec<MachineFilamentInfo>],
    color_threshold: f64,
) -> Vec<i32> {
    // FilamentGroup.cpp:90  using namespace FlushPredict;

    // FilamentGroup.cpp:92
    let fail_cost = 9999;

    // FilamentGroup.cpp:94
    // these code is to make we machine filament info size is 2
    // FilamentGroup.cpp:95
    let mut machine_filament_info: Vec<Vec<MachineFilamentInfo>> = machine_filament_info_.to_vec();
    // FilamentGroup.cpp:96
    machine_filament_info.resize(2, Vec::new());

    // FilamentGroup.cpp:98
    let mut best_cost: i32 = i32::MAX;
    // FilamentGroup.cpp:99
    let mut best_map: Vec<i32> = Vec::new();

    // FilamentGroup.cpp:101
    for filament_to_nozzle in filament_to_nozzles.iter() {
        // FilamentGroup.cpp:102
        let mut group_filaments: Vec<Vec<i32>> = vec![Vec::new(); 2];
        // FilamentGroup.cpp:103
        let mut group_colors: Vec<Vec<crate::filament_group_utils::Color>> = vec![Vec::new(); 2];

        // FilamentGroup.cpp:105
        for i in 0..used_filaments.len() {
            // FilamentGroup.cpp:106
            let nozzle = &nozzle_list[filament_to_nozzle[used_filaments[i] as usize] as usize];
            // FilamentGroup.cpp:107
            let target_group: usize = if nozzle.extruder_id == 0 { 0 } else { 1 };
            // FilamentGroup.cpp:108
            group_colors[target_group].push(used_filament_info[i].color);
            // FilamentGroup.cpp:109
            group_filaments[target_group].push(i as i32);
        }

        // FilamentGroup.cpp:112
        let mut group_cost: i32 = 0;
        // FilamentGroup.cpp:113
        for i in 0..2usize {
            // FilamentGroup.cpp:114
            if group_colors[i].is_empty() {
                // FilamentGroup.cpp:115
                continue;
            }
            // FilamentGroup.cpp:116
            if machine_filament_info[i].is_empty() {
                // FilamentGroup.cpp:117  group_cost += group_colors.size() * fail_cost;
                group_cost += group_colors.len() as i32 * fail_cost;
                // FilamentGroup.cpp:118
                continue;
            }
            // FilamentGroup.cpp:120
            let mut distance_matrix: Vec<Vec<f32>> =
                vec![vec![0.0f32; machine_filament_info[i].len()]; group_colors[i].len()];

            // FilamentGroup.cpp:122
            // calculate color distance matrix
            // FilamentGroup.cpp:123
            for src in 0..group_colors[i].len() {
                // FilamentGroup.cpp:124
                for dst in 0..machine_filament_info[i].len() {
                    // FilamentGroup.cpp:125
                    distance_matrix[src][dst] = calc_color_distance(
                        &RGBColor::new(
                            group_colors[i][src].r,
                            group_colors[i][src].g,
                            group_colors[i][src].b,
                        ),
                        &RGBColor::new(
                            machine_filament_info[i][dst].color.r,
                            machine_filament_info[i][dst].color.g,
                            machine_filament_info[i][dst].color.b,
                        ),
                    );
                }
            }

            // FilamentGroup.cpp:132
            // get min cost by min cost max flow
            // FilamentGroup.cpp:133
            let mut l_nodes: Vec<i32> = vec![0; group_colors[i].len()];
            let mut r_nodes: Vec<i32> = vec![0; machine_filament_info[i].len()];
            // FilamentGroup.cpp:134
            for (k, v) in l_nodes.iter_mut().enumerate() {
                *v = k as i32;
            }
            // FilamentGroup.cpp:135
            for (k, v) in r_nodes.iter_mut().enumerate() {
                *v = k as i32;
            }

            // FilamentGroup.cpp:137
            let mut unlink_limits: HashMap<i32, Vec<i32>> = HashMap::new();
            // FilamentGroup.cpp:138
            for from in 0..group_filaments[i].len() {
                // FilamentGroup.cpp:139
                for to in 0..machine_filament_info[i].len() {
                    // FilamentGroup.cpp:140-141
                    if used_filament_info[group_filaments[i][from] as usize].type_
                        != machine_filament_info[i][to].type_
                        || used_filament_info[group_filaments[i][from] as usize].is_support
                            != machine_filament_info[i][to].is_support
                    {
                        // FilamentGroup.cpp:142
                        unlink_limits.entry(from as i32).or_default().push(to as i32);
                    }
                }
            }

            // FilamentGroup.cpp:147
            let mut mcmf = MatchModeGroupSolver::new(
                &distance_matrix,
                &l_nodes,
                &r_nodes,
                &vec![l_nodes.len() as i32; r_nodes.len()],
                &unlink_limits,
            );
            // FilamentGroup.cpp:148
            let ams_map = mcmf.solve();

            // FilamentGroup.cpp:150
            for idx in 0..ams_map.len() {
                // FilamentGroup.cpp:151
                if ams_map[idx] == INVALID_ID
                    || distance_matrix[idx][ams_map[idx] as usize] as f64 > color_threshold
                {
                    // FilamentGroup.cpp:152
                    group_cost += fail_cost;
                }
                // FilamentGroup.cpp:154
                else {
                    // FilamentGroup.cpp:155
                    group_cost += distance_matrix[idx][ams_map[idx] as usize] as i32;
                }
            }
        }

        // FilamentGroup.cpp:160
        if best_map.is_empty() || group_cost < best_cost {
            // FilamentGroup.cpp:161
            best_cost = group_cost;
            // FilamentGroup.cpp:162
            best_map = filament_to_nozzle.clone();
        }
    }

    // FilamentGroup.cpp:166
    best_map
}

// FilamentGroup.cpp:215
/// `std::vector<unsigned int> collect_sorted_used_filaments(const std::vector<std::vector<unsigned int>>& layer_filaments)`
pub fn collect_sorted_used_filaments(layer_filaments: &[Vec<u32>]) -> Vec<u32> {
    // FilamentGroup.cpp:217
    let mut used_filaments_set: BTreeSet<u32> = BTreeSet::new();
    // FilamentGroup.cpp:218
    for lf in layer_filaments.iter() {
        // FilamentGroup.cpp:219
        for &f in lf.iter() {
            // FilamentGroup.cpp:220
            used_filaments_set.insert(f);
        }
    }
    // FilamentGroup.cpp:221
    let mut used_filaments: Vec<u32> = used_filaments_set.into_iter().collect();
    // FilamentGroup.cpp:222  sort_remove_duplicates(used_filaments);
    used_filaments.sort_unstable();
    used_filaments.dedup();
    // FilamentGroup.cpp:223
    used_filaments
}

// =============================================================================
// FlushDistanceEvaluator (FilamentGroup.hpp:121, FilamentGroup.cpp:226)
// =============================================================================

// FilamentGroup.hpp:121
pub struct FlushDistanceEvaluator {
    // FilamentGroup.hpp:128
    m_distance_matrix: Vec<Vec<Vec<f32>>>,
}

impl FlushDistanceEvaluator {
    // FilamentGroup.cpp:226
    /// `FlushDistanceEvaluator(const std::vector<FlushMatrix>& flush_matrix, const std::vector<unsigned int>& used_filaments, const std::vector<std::vector<unsigned int>>& layer_filaments, double p)`
    pub fn new(
        flush_matrix: &[FlushMatrix],
        used_filaments: &[u32],
        layer_filaments: &[Vec<u32>],
        p: f64,
    ) -> Self {
        // FilamentGroup.cpp:228
        // calc pair counts
        // FilamentGroup.cpp:229
        let mut count_matrix: Vec<Vec<i32>> =
            vec![vec![0i32; used_filaments.len()]; used_filaments.len()];
        // FilamentGroup.cpp:230
        for lf in layer_filaments.iter() {
            // FilamentGroup.cpp:231
            for iter in 0..lf.len() {
                // FilamentGroup.cpp:232  auto id_iter1 = std::find(used_filaments..., *iter)
                let id_iter1 = used_filaments.iter().position(|&x| x == lf[iter]);
                // FilamentGroup.cpp:233-234
                let idx1 = match id_iter1 {
                    None => continue,
                    Some(v) => v,
                };
                // FilamentGroup.cpp:236  for (auto niter = std::next(iter); ...)
                for niter in (iter + 1)..lf.len() {
                    // FilamentGroup.cpp:237
                    let id_iter2 = used_filaments.iter().position(|&x| x == lf[niter]);
                    // FilamentGroup.cpp:238-239
                    let idx2 = match id_iter2 {
                        None => continue,
                        Some(v) => v,
                    };
                    // FilamentGroup.cpp:241
                    count_matrix[idx1][idx2] += 1;
                    // FilamentGroup.cpp:242
                    count_matrix[idx2][idx1] += 1;
                }
            }
        }

        // FilamentGroup.cpp:247
        let mut m_distance_matrix: Vec<Vec<Vec<f32>>> =
            vec![vec![vec![0.0f32; used_filaments.len()]; used_filaments.len()]; flush_matrix.len()];

        // FilamentGroup.cpp:249
        for i in 0..used_filaments.len() {
            // FilamentGroup.cpp:250
            for j in 0..used_filaments.len() {
                // FilamentGroup.cpp:251
                for k in 0..flush_matrix.len() {
                    // FilamentGroup.cpp:252
                    if i == j {
                        // FilamentGroup.cpp:253
                        m_distance_matrix[k][i][j] = 0.0;
                    }
                    // FilamentGroup.cpp:254
                    else {
                        // FilamentGroup.cpp:255  //TODO: check m_flush_matrix
                        // FilamentGroup.cpp:256
                        let max_val = flush_matrix[k][used_filaments[i] as usize]
                            [used_filaments[j] as usize]
                            .max(
                                flush_matrix[k][used_filaments[j] as usize]
                                    [used_filaments[i] as usize],
                            );
                        // FilamentGroup.cpp:257
                        let min_val = flush_matrix[k][used_filaments[i] as usize]
                            [used_filaments[j] as usize]
                            .min(
                                flush_matrix[k][used_filaments[j] as usize]
                                    [used_filaments[i] as usize],
                            );
                        // FilamentGroup.cpp:258
                        m_distance_matrix[k][i][j] = (max_val * p as f32
                            + min_val * (1.0 - p as f32))
                            * (count_matrix[i][j].max(1)) as f32;
                    }
                }
            }
        }

        FlushDistanceEvaluator { m_distance_matrix }
    }

    // FilamentGroup.cpp:265
    /// `double FlushDistanceEvaluator::get_distance(int idx_a, int idx_b, int extruder_id) const`
    pub fn get_distance(&self, idx_a: i32, idx_b: i32, extruder_id: i32) -> f64 {
        // FilamentGroup.cpp:267
        debug_assert!(
            0 <= idx_a && (idx_a as usize) < self.m_distance_matrix[extruder_id as usize].len()
        );
        // FilamentGroup.cpp:268
        debug_assert!(
            0 <= idx_b && (idx_b as usize) < self.m_distance_matrix[extruder_id as usize].len()
        );

        // FilamentGroup.cpp:270
        self.m_distance_matrix[extruder_id as usize][idx_a as usize][idx_b as usize] as f64
    }
}

// =============================================================================
// TimeEvaluator (FilamentGroup.hpp:133, FilamentGroup.cpp:273)
// =============================================================================

// FilamentGroup.hpp:133
pub struct TimeEvaluator {
    // FilamentGroup.hpp:139
    m_speed_info: SpeedInfo,
}

impl TimeEvaluator {
    // FilamentGroup.hpp:136
    /// `TimeEvaluator(const FilamentGroupContext::SpeedInfo& speed_info) : m_speed_info(speed_info) {}`
    pub fn new(speed_info: SpeedInfo) -> Self {
        TimeEvaluator {
            m_speed_info: speed_info,
        }
    }

    // FilamentGroup.cpp:273
    /// `double TimeEvaluator::get_estimated_time(const std::vector<int>& filament_map) const`
    pub fn get_estimated_time(&self, filament_map: &[i32]) -> f64 {
        // FilamentGroup.cpp:275
        let mut time: f64 = 0.0;
        // FilamentGroup.cpp:276
        for elem in self.m_speed_info.filament_print_time.iter() {
            // FilamentGroup.cpp:277
            let filament_idx = *elem.0;
            // FilamentGroup.cpp:278
            let extruder_time = elem.1;
            // FilamentGroup.cpp:279
            let filament_extruder_id = filament_map[filament_idx as usize];
            // FilamentGroup.cpp:280
            time += extruder_time[&filament_extruder_id];
        }
        // FilamentGroup.cpp:282
        time
    }
}

// =============================================================================
// KMediods2 (FilamentGroup.hpp:204, FilamentGroup.cpp:286)
// =============================================================================

// FilamentGroup.hpp:204
pub struct KMediods2 {
    // FilamentGroup.hpp:241
    memoryed_groups: MemoryedGroupHeap,
    // FilamentGroup.hpp:242
    m_evaluator: std::rc::Rc<FlushDistanceEvaluator>,
    // FilamentGroup.hpp:243
    m_unplaceable_limits: BTreeMap<i32, i32>,
    // FilamentGroup.hpp:244
    m_cluster_labels: Vec<i32>,
    // FilamentGroup.hpp:245
    m_max_cluster_size: Vec<i32>,

    // FilamentGroup.hpp:247
    m_k: i32,
    // FilamentGroup.hpp:248
    m_elem_count: i32,
    // FilamentGroup.hpp:249
    m_default_group_id: i32,
    // FilamentGroup.hpp:250
    memory_threshold: f64,
}

impl KMediods2 {
    // FilamentGroup.hpp:215
    /// `KMediods2(const int elem_count, const std::shared_ptr<FlushDistanceEvaluator>& evaluator, int default_group_id = 0)`
    pub fn new(
        elem_count: i32,
        evaluator: std::rc::Rc<FlushDistanceEvaluator>,
        default_group_id: i32,
    ) -> Self {
        // FilamentGroup.hpp:247  const int m_k = 2;
        let m_k = 2;
        // FilamentGroup.hpp:220  m_max_cluster_size = std::vector<int>(m_k, DEFAULT_CLUSTER_SIZE);
        let m_max_cluster_size = vec![DEFAULT_CLUSTER_SIZE; m_k as usize];
        KMediods2 {
            memoryed_groups: MemoryedGroupHeap::new(),
            m_evaluator: evaluator,
            m_unplaceable_limits: BTreeMap::new(),
            m_cluster_labels: Vec::new(),
            m_max_cluster_size,
            m_k,
            m_elem_count: elem_count,
            m_default_group_id: default_group_id,
            memory_threshold: 0.0,
        }
    }

    // FilamentGroup.hpp:224
    /// set max group size
    pub fn set_max_cluster_size(&mut self, group_size: Vec<i32>) {
        self.m_max_cluster_size = group_size;
    }

    // FilamentGroup.hpp:227
    /// key stores elem idx, value stores the cluster id that elem cannot be placed
    pub fn set_unplaceable_limits(&mut self, placeable_limits: BTreeMap<i32, i32>) {
        self.m_unplaceable_limits = placeable_limits;
    }

    // FilamentGroup.hpp:231
    pub fn set_memory_threshold(&mut self, threshold: f64) {
        self.memory_threshold = threshold;
    }

    // FilamentGroup.hpp:232
    pub fn get_memoryed_groups(&self) -> MemoryedGroupHeap {
        self.memoryed_groups.clone()
    }

    // FilamentGroup.hpp:234
    pub fn get_cluster_labels(&self) -> Vec<i32> {
        self.m_cluster_labels.clone()
    }

    // FilamentGroup.cpp:286
    /// `std::vector<int> KMediods2::cluster_small_data(const std::map<int, int>& unplaceable_limits, const std::vector<int>& group_size)`
    fn cluster_small_data(
        &self,
        unplaceable_limits: &BTreeMap<i32, i32>,
        group_size: &[i32],
    ) -> Vec<i32> {
        // FilamentGroup.cpp:288
        let mut labels: Vec<i32> = vec![-1; self.m_elem_count as usize];
        // FilamentGroup.cpp:289
        let mut new_group_size: Vec<i32> = group_size.to_vec();

        // FilamentGroup.cpp:291
        for (&elem, &center) in unplaceable_limits.iter() {
            // FilamentGroup.cpp:292
            if labels[elem as usize] == -1 {
                // FilamentGroup.cpp:293
                let gid = 1 - center;
                // FilamentGroup.cpp:294
                labels[elem as usize] = gid;
                // FilamentGroup.cpp:295
                new_group_size[gid as usize] -= 1;
            }
        }

        // FilamentGroup.cpp:299
        for label in labels.iter_mut() {
            // FilamentGroup.cpp:300
            if *label == -1 {
                // FilamentGroup.cpp:301
                let mut gid: i32 = -1;
                // FilamentGroup.cpp:302
                for idx in 0..new_group_size.len() {
                    // FilamentGroup.cpp:303
                    if new_group_size[idx] > 0 {
                        // FilamentGroup.cpp:304
                        gid = idx as i32;
                        // FilamentGroup.cpp:305
                        break;
                    }
                }
                // FilamentGroup.cpp:308
                if gid != -1 {
                    // FilamentGroup.cpp:309
                    *label = gid;
                    // FilamentGroup.cpp:310
                    new_group_size[gid as usize] -= 1;
                }
                // FilamentGroup.cpp:312
                else {
                    // FilamentGroup.cpp:313
                    *label = self.m_default_group_id;
                }
            }
        }

        // FilamentGroup.cpp:318
        labels
    }

    // FilamentGroup.cpp:321
    /// `std::vector<int> KMediods2::assign_cluster_label(const std::vector<int>& center, const std::map<int, int>& unplaceable_limtis, const std::vector<int>& group_size, const FGStrategy& strategy)`
    fn assign_cluster_label(
        &self,
        center: &[i32],
        unplaceable_limtis: &BTreeMap<i32, i32>,
        group_size: &[i32],
        strategy: FGStrategy,
    ) -> Vec<i32> {
        // FilamentGroup.cpp:323-327  struct Comp { ... a.second > b.second ... }
        // C++ `std::priority_queue` with `Comp` (operator returns a.second > b.second)
        // is a min-heap on `.second` (the distance gap). `top()` is the smallest gap.

        // FilamentGroup.cpp:329
        let mut groups: Vec<BTreeSet<i32>> = vec![BTreeSet::new(); 2];
        // FilamentGroup.cpp:330
        let mut new_max_group_size: Vec<i32> = group_size.to_vec();
        // FilamentGroup.cpp:332
        // store filament idx and distance gap between center 0 and center 1
        // min_heap holds (idx, gap) and pops the smallest gap first.
        let mut min_heap: Vec<(i32, i32)> = Vec::new();

        // FilamentGroup.cpp:334
        for i in 0..self.m_elem_count {
            // FilamentGroup.cpp:335  if (auto it = unplaceable_limtis.find(i); it != ...end())
            if let Some(&gid) = unplaceable_limtis.get(&i) {
                // FilamentGroup.cpp:337
                debug_assert!(gid == 0 || gid == 1);
                // FilamentGroup.cpp:338  groups[1 - gid].insert(i);
                groups[(1 - gid) as usize].insert(i);
                // FilamentGroup.cpp:339  new_max_group_size[1 - gid] = std::max(new_max_group_size[1 - gid] - 1, 0);
                new_max_group_size[(1 - gid) as usize] =
                    (new_max_group_size[(1 - gid) as usize] - 1).max(0);
                // FilamentGroup.cpp:340
                continue;
            }
            // FilamentGroup.cpp:342  int distance_to_0 = m_evaluator->get_distance(i, center[0], 0);
            let distance_to_0 = self.m_evaluator.get_distance(i, center[0], 0) as i32;
            // FilamentGroup.cpp:343  int distance_to_1 = m_evaluator->get_distance(i, center[1], 1);
            let distance_to_1 = self.m_evaluator.get_distance(i, center[1], 1) as i32;
            // FilamentGroup.cpp:344  min_heap.push({ i, distance_to_0 - distance_to_1 });
            min_heap.push((i, distance_to_0 - distance_to_1));
        }

        // Build a binary heap that yields the smallest `.second` first (a min-heap),
        // matching C++ `std::priority_queue<.., Comp>` where Comp(a,b) = a.second > b.second.
        // We sort a Vec and pop from the front to keep the deterministic ordering of the
        // C++ priority_queue for equal gaps is unspecified; C++ uses a heap so ties are
        // resolved by heap order. To stay faithful to the *algorithm* we pop minimum gap
        // first; for ties the original code's behaviour is heap-implementation-defined.
        let pop_min = |heap: &mut Vec<(i32, i32)>| -> Option<(i32, i32)> {
            if heap.is_empty() {
                return None;
            }
            let mut best = 0usize;
            for k in 1..heap.len() {
                if heap[k].1 < heap[best].1 {
                    best = k;
                }
            }
            Some(heap.swap_remove(best))
        };

        // FilamentGroup.cpp:347  bool have_enough_size = (min_heap.size() <= (new_max_group_size[0] + new_max_group_size[1]));
        let have_enough_size =
            (min_heap.len() as i32) <= (new_max_group_size[0] + new_max_group_size[1]);

        // FilamentGroup.cpp:349
        if have_enough_size || strategy == FGStrategy::BestFit {
            // FilamentGroup.cpp:350
            while !min_heap.is_empty() {
                // FilamentGroup.cpp:351
                let top = pop_min(&mut min_heap).unwrap();
                // FilamentGroup.cpp:352  min_heap.pop(); (handled by pop_min)
                // FilamentGroup.cpp:353
                if (groups[0].len() as i32) < new_max_group_size[0]
                    && (top.1 <= 0 || (groups[1].len() as i32) >= new_max_group_size[1])
                {
                    // FilamentGroup.cpp:354
                    groups[0].insert(top.0);
                }
                // FilamentGroup.cpp:355
                else if (groups[1].len() as i32) < new_max_group_size[1]
                    && (top.1 > 0 || (groups[0].len() as i32) >= new_max_group_size[0])
                {
                    // FilamentGroup.cpp:356
                    groups[1].insert(top.0);
                }
                // FilamentGroup.cpp:357
                else {
                    // FilamentGroup.cpp:358
                    if top.1 <= 0 {
                        // FilamentGroup.cpp:359
                        groups[0].insert(top.0);
                    }
                    // FilamentGroup.cpp:360
                    else {
                        // FilamentGroup.cpp:361
                        groups[1].insert(top.0);
                    }
                }
            }
        }
        // FilamentGroup.cpp:365
        else {
            // FilamentGroup.cpp:366
            while !min_heap.is_empty() {
                // FilamentGroup.cpp:367
                let top = pop_min(&mut min_heap).unwrap();
                // FilamentGroup.cpp:368  min_heap.pop();
                // FilamentGroup.cpp:369
                if top.1 <= 0 {
                    // FilamentGroup.cpp:370
                    groups[0].insert(top.0);
                }
                // FilamentGroup.cpp:371
                else {
                    // FilamentGroup.cpp:372
                    groups[1].insert(top.0);
                }
            }
        }

        // FilamentGroup.cpp:376
        let mut labels: Vec<i32> = vec![0; self.m_elem_count as usize];
        // FilamentGroup.cpp:377
        for &f in groups[0].iter() {
            // FilamentGroup.cpp:378
            labels[f as usize] = 0;
        }
        // FilamentGroup.cpp:379
        for &f in groups[1].iter() {
            // FilamentGroup.cpp:380
            labels[f as usize] = 1;
        }

        // FilamentGroup.cpp:382
        labels
    }

    // FilamentGroup.cpp:385
    /// `int KMediods2::calc_cost(const std::vector<int>& labels, const std::vector<int>& medoids)`
    fn calc_cost(&self, labels: &[i32], medoids: &[i32]) -> i32 {
        // FilamentGroup.cpp:387
        let mut total_cost: i32 = 0;
        // FilamentGroup.cpp:388
        for i in 0..self.m_elem_count {
            // FilamentGroup.cpp:389
            total_cost += self
                .m_evaluator
                .get_distance(i, medoids[labels[i as usize] as usize], labels[i as usize])
                as i32;
        }
        // FilamentGroup.cpp:390
        total_cost
    }

    // FilamentGroup.cpp:393
    /// `void KMediods2::do_clustering(const FGStrategy& g_strategy, int timeout_ms)`
    pub fn do_clustering(&mut self, g_strategy: FGStrategy, timeout_ms: i32) {
        // FilamentGroup.cpp:395-396  FlushTimeMachine T; T.time_machine_start();
        let t = std::time::Instant::now();
        let time_machine_end = || -> i32 { t.elapsed().as_millis() as i32 };

        // FilamentGroup.cpp:398
        if self.m_elem_count < self.m_k {
            // FilamentGroup.cpp:399
            self.m_cluster_labels =
                self.cluster_small_data(&self.m_unplaceable_limits.clone(), &self.m_max_cluster_size.clone());
            // FilamentGroup.cpp:400
            {
                // FilamentGroup.cpp:401
                let mut cluster_center: Vec<i32> = vec![-1; self.m_k as usize];
                // FilamentGroup.cpp:402
                for idx in 0..self.m_cluster_labels.len() {
                    // FilamentGroup.cpp:403
                    if cluster_center[self.m_cluster_labels[idx] as usize] == -1 {
                        // FilamentGroup.cpp:404
                        cluster_center[self.m_cluster_labels[idx] as usize] = idx as i32;
                    }
                }
                // FilamentGroup.cpp:406
                let g = MemoryedGroup::new(
                    self.m_cluster_labels.clone(),
                    self.calc_cost(&self.m_cluster_labels.clone(), &cluster_center),
                    1,
                );
                // FilamentGroup.cpp:407
                update_memoryed_groups(&g, self.memory_threshold, &mut self.memoryed_groups);
            }
            // FilamentGroup.cpp:409
            return;
        }

        // FilamentGroup.cpp:412
        let mut best_labels: Vec<i32> = Vec::new();
        // FilamentGroup.cpp:413
        let mut best_cost: i32 = i32::MAX;

        // FilamentGroup.cpp:415
        'outer: for center_0 in 0..self.m_elem_count {
            // FilamentGroup.cpp:416  if (auto iter = m_unplaceable_limits.find(center_0); iter != ...end() && iter->second == 0)
            if let Some(&v) = self.m_unplaceable_limits.get(&center_0) {
                if v == 0 {
                    // FilamentGroup.cpp:417
                    continue;
                }
            }
            // FilamentGroup.cpp:418
            for center_1 in 0..self.m_elem_count {
                // FilamentGroup.cpp:419
                if center_0 == center_1 {
                    // FilamentGroup.cpp:420
                    continue;
                }
                // FilamentGroup.cpp:421  if (auto iter = m_unplaceable_limits.find(center_1); iter != ...end() && iter->second == 1)
                if let Some(&v) = self.m_unplaceable_limits.get(&center_1) {
                    if v == 1 {
                        // FilamentGroup.cpp:422
                        continue;
                    }
                }

                // FilamentGroup.cpp:424
                let new_centers: Vec<i32> = vec![center_0, center_1];
                // FilamentGroup.cpp:425
                let new_labels = self.assign_cluster_label(
                    &new_centers,
                    &self.m_unplaceable_limits.clone(),
                    &self.m_max_cluster_size.clone(),
                    g_strategy,
                );

                // FilamentGroup.cpp:427
                let new_cost = self.calc_cost(&new_labels, &new_centers);
                // FilamentGroup.cpp:428
                if new_cost < best_cost {
                    // FilamentGroup.cpp:429
                    best_cost = new_cost;
                    // FilamentGroup.cpp:430
                    best_labels = new_labels.clone();
                }

                // FilamentGroup.cpp:433
                {
                    // FilamentGroup.cpp:434
                    let g = MemoryedGroup::new(new_labels, new_cost, 1);
                    // FilamentGroup.cpp:435
                    update_memoryed_groups(&g, self.memory_threshold, &mut self.memoryed_groups);
                }

                // FilamentGroup.cpp:438
                if time_machine_end() > timeout_ms {
                    // FilamentGroup.cpp:439
                    break;
                }
            }
            // FilamentGroup.cpp:441
            if time_machine_end() > timeout_ms {
                // FilamentGroup.cpp:442
                break 'outer;
            }
        }
        // FilamentGroup.cpp:444
        self.m_cluster_labels = best_labels;
    }
}

// =============================================================================
// FilamentGroup class (FilamentGroup.hpp:142, FilamentGroup.cpp:855+)
// =============================================================================

// FilamentGroup.hpp:142
pub struct FilamentGroup {
    // FilamentGroup.hpp:169
    ctx: FilamentGroupContext,
    // FilamentGroup.hpp:170
    m_memoryed_groups: Vec<Vec<i32>>,
    // FilamentGroup.hpp:174  std::optional<std::function<bool(int, std::vector<int>&)>> get_custom_seq;
    // The custom-sequence callback is only consumed by
    // `reorder_filaments_for_minimum_flush_volume` (a blocked dependency); kept
    // here for parity of the public surface.
    #[allow(clippy::type_complexity)]
    pub get_custom_seq: Option<Box<dyn Fn(i32, &mut Vec<i32>) -> bool>>,
}

impl FilamentGroup {
    // FilamentGroup.hpp:147
    /// `explicit FilamentGroup(const FilamentGroupContext& ctx_) : ctx(ctx_) {}`
    pub fn new(ctx: FilamentGroupContext) -> Self {
        FilamentGroup {
            ctx,
            m_memoryed_groups: Vec::new(),
            get_custom_seq: None,
        }
    }

    // FilamentGroup.hpp:150
    pub fn get_memoryed_groups(&self) -> Vec<Vec<i32>> {
        self.m_memoryed_groups.clone()
    }

    // FilamentGroup.cpp:855
    /// `std::map<int, int> FilamentGroup::rebuild_unprintables(const std::vector<unsigned int>& used_filaments, const std::map<int, int>& extruder_unprintables)`
    fn rebuild_unprintables(
        &self,
        used_filaments: &[u32],
        extruder_unprintables: &BTreeMap<i32, i32>,
    ) -> BTreeMap<i32, i32> {
        // FilamentGroup.cpp:857
        let mut ret: BTreeMap<i32, i32> = BTreeMap::new();
        // FilamentGroup.cpp:858
        for f_idx in 0..used_filaments.len() as i32 {
            // FilamentGroup.cpp:859
            let mut unprintable_ext: i32 = -1;
            // FilamentGroup.cpp:860
            if extruder_unprintables.contains_key(&f_idx) {
                // FilamentGroup.cpp:861
                unprintable_ext = extruder_unprintables[&f_idx];
            }

            // FilamentGroup.cpp:864
            let mut multi_unprintable = false;
            // FilamentGroup.cpp:865  auto unprintable_volumes = ctx.model_info.unprintable_volumes[used_filaments[f_idx]];
            let empty_set: BTreeSet<NozzleVolumeType> = BTreeSet::new();
            let unprintable_volumes = self
                .ctx
                .model_info
                .unprintable_volumes
                .get(&(used_filaments[f_idx as usize] as i32))
                .unwrap_or(&empty_set);
            // FilamentGroup.cpp:866
            for nozzle_idx in 0..self.ctx.nozzle_info.nozzle_list.len() {
                // FilamentGroup.cpp:867
                let nozzle_info = &self.ctx.nozzle_info.nozzle_list[nozzle_idx];

                // FilamentGroup.cpp:869
                if unprintable_volumes.contains(&nozzle_info.volume_type) {
                    // FilamentGroup.cpp:870
                    if unprintable_ext == -1 {
                        // FilamentGroup.cpp:871
                        unprintable_ext = nozzle_info.extruder_id;
                    }
                    // FilamentGroup.cpp:872
                    else if unprintable_ext != nozzle_info.extruder_id {
                        // FilamentGroup.cpp:873
                        multi_unprintable = true;
                    }
                }
            }

            // FilamentGroup.cpp:877
            if !multi_unprintable && unprintable_ext != -1 {
                ret.insert(f_idx, unprintable_ext);
            }
        }
        // FilamentGroup.cpp:880
        ret
    }

    // FilamentGroup.cpp:883
    /// `std::unordered_map<int, std::vector<int>> FilamentGroup::try_merge_filaments()`
    fn try_merge_filaments(&self) -> HashMap<i32, Vec<i32>> {
        // FilamentGroup.cpp:885
        let mut merged_filaments: HashMap<i32, Vec<i32>> = HashMap::new();

        // FilamentGroup.cpp:887
        let mut merge_filament_map: BTreeMap<String, Vec<i32>> = BTreeMap::new();

        // FilamentGroup.cpp:889  auto unprintable_stat_to_str = [unprintable_filaments = this->ctx.model_info.unprintable_filaments](int idx)
        let unprintable_filaments = &self.ctx.model_info.unprintable_filaments;
        let unprintable_stat_to_str = |idx: i32| -> String {
            // FilamentGroup.cpp:890
            let mut s = String::new();
            // FilamentGroup.cpp:891
            for eid in 0..unprintable_filaments.len() {
                // FilamentGroup.cpp:892
                if unprintable_filaments[eid].contains(&idx) {
                    // FilamentGroup.cpp:893
                    if eid > 0 {
                        // FilamentGroup.cpp:894
                        s.push(',');
                    }
                    // FilamentGroup.cpp:895  str += std::to_string(idx);
                    s.push_str(&idx.to_string());
                }
            }
            // FilamentGroup.cpp:897
            s
        };

        // FilamentGroup.cpp:901
        for idx in 0..self.ctx.model_info.filament_ids.len() {
            // FilamentGroup.cpp:902
            let id = &self.ctx.model_info.filament_ids[idx];
            // FilamentGroup.cpp:903
            let color = self.ctx.model_info.filament_info[idx].color;
            // FilamentGroup.cpp:904
            let unprintable_str = unprintable_stat_to_str(idx as i32);

            // FilamentGroup.cpp:906  std::string key = id + "," + color.to_hex_str(true) + "," + unprintable_str;
            let key = format!("{},{},{}", id, color.to_hex_str(true), unprintable_str);
            // FilamentGroup.cpp:907
            merge_filament_map.entry(key).or_default().push(idx as i32);
        }

        // FilamentGroup.cpp:910
        for elem in merge_filament_map.iter() {
            // FilamentGroup.cpp:911
            if elem.1.len() > 1 {
                // FilamentGroup.cpp:912  merged_filaments[elem.second.front()] = elem.second;
                merged_filaments.insert(elem.1[0], elem.1.clone());
            }
        }
        // FilamentGroup.cpp:915
        merged_filaments
    }

    // FilamentGroup.cpp:918
    /// `std::vector<int> FilamentGroup::seperate_merged_filaments(const std::vector<int>& filament_map, const std::unordered_map<int, std::vector<int>>& merged_filaments)`
    fn seperate_merged_filaments(
        &self,
        filament_map: &[i32],
        merged_filaments: &HashMap<i32, Vec<i32>>,
    ) -> Vec<i32> {
        // FilamentGroup.cpp:920
        let mut ret_map: Vec<i32> = filament_map.to_vec();
        // FilamentGroup.cpp:921
        for elem in merged_filaments.iter() {
            // FilamentGroup.cpp:922
            let src = *elem.0;
            // FilamentGroup.cpp:923
            for &f in elem.1.iter() {
                // FilamentGroup.cpp:924
                ret_map[f as usize] = ret_map[src as usize];
            }
        }
        // FilamentGroup.cpp:927
        ret_map
    }

    // FilamentGroup.cpp:930
    /// `void FilamentGroup::rebuild_context(const std::unordered_map<int, std::vector<int>>& merged_filaments)`
    fn rebuild_context(&mut self, merged_filaments: &HashMap<i32, Vec<i32>>) {
        // FilamentGroup.cpp:932
        if merged_filaments.is_empty() {
            // FilamentGroup.cpp:933
            return;
        }

        // FilamentGroup.cpp:935
        let mut new_ctx = self.ctx.clone();

        // FilamentGroup.cpp:937
        let filament_merge_map = get_merged_filament_map(merged_filaments);

        // FilamentGroup.cpp:939
        // modify layer filaments
        // FilamentGroup.cpp:940
        for layer_filament in new_ctx.model_info.layer_filaments.iter_mut() {
            // FilamentGroup.cpp:941
            for f in layer_filament.iter_mut() {
                // FilamentGroup.cpp:942  if (auto iter = filament_merge_map.find((int)(f)); iter != ...end())
                if let Some(&v) = filament_merge_map.get(&(*f as i32)) {
                    // FilamentGroup.cpp:943
                    *f = v as u32;
                }
            }
        }

        // FilamentGroup.cpp:948
        for unprintables in new_ctx.model_info.unprintable_filaments.iter() {
            // FilamentGroup.cpp:949  std::set<int> new_unprintables;
            // NOTE: faithful to C++ — `new_unprintables` is computed but never stored
            // back (the C++ code builds it then discards it; this is a known no-op).
            let mut new_unprintables: BTreeSet<i32> = BTreeSet::new();
            // FilamentGroup.cpp:950
            for &f in unprintables.iter() {
                // FilamentGroup.cpp:951  if (auto iter = filament_merge_map.find((int)(f)); iter != ...end())
                if let Some(&v) = filament_merge_map.get(&f) {
                    // FilamentGroup.cpp:952
                    new_unprintables.insert(v);
                }
                // FilamentGroup.cpp:954
                else {
                    // FilamentGroup.cpp:955
                    new_unprintables.insert(f);
                }
            }
            // (`new_unprintables` deliberately not assigned anywhere — matches C++.)
            let _ = new_unprintables;
        }

        // FilamentGroup.cpp:960
        self.ctx = new_ctx;
        // FilamentGroup.cpp:961
    }

    // FilamentGroup.cpp:987
    /// `std::vector<int> FilamentGroup::calc_filament_group_for_match(int* cost)`
    pub fn calc_filament_group_for_match(
        &mut self,
        _cost: Option<&mut i32>,
    ) -> Result<Vec<i32>, FilamentGroupException> {
        // FilamentGroup.cpp:989  using namespace FlushPredict;
        // FilamentGroup.cpp:990
        const SUPPORT_PREFER_SCORE: i32 = 3;

        // FilamentGroup.cpp:992
        let used_filaments = collect_sorted_used_filaments(&self.ctx.model_info.layer_filaments);
        // FilamentGroup.cpp:993
        let mut used_filament_list: Vec<FilamentInfo> = Vec::new();
        // FilamentGroup.cpp:994
        for &f in used_filaments.iter() {
            // FilamentGroup.cpp:995
            used_filament_list.push(self.ctx.model_info.filament_info[f as usize].clone());
        }

        // FilamentGroup.cpp:997
        let mut machine_filament_list: Vec<MachineFilamentInfo> = Vec::new();
        // FilamentGroup.cpp:998  std::map<MachineFilamentInfo, std::set<int>> machine_filament_set;
        // `MachineFilamentInfo` has an `operator<` (`lt`); we key the map by the
        // 1:1 ordering it induces. We use a `Vec<(MachineFilamentInfo, BTreeSet<i32>)>`
        // backed by linear lookup with `lt`-equivalence to preserve exact semantics.
        let mut machine_filament_set: Vec<(MachineFilamentInfo, BTreeSet<i32>)> = Vec::new();
        // helper: ordering equivalence under `lt` (a==b iff !(a<b) && !(b<a))
        let mfi_find_or_insert =
            |set: &mut Vec<(MachineFilamentInfo, BTreeSet<i32>)>, key: &MachineFilamentInfo| -> usize {
                for (i, (k, _)) in set.iter().enumerate() {
                    if !k.lt(key) && !key.lt(k) {
                        return i;
                    }
                }
                set.push((key.clone(), BTreeSet::new()));
                set.len() - 1
            };
        // FilamentGroup.cpp:999
        for eid in 0..self.ctx.machine_info.machine_filament_info.len() {
            // FilamentGroup.cpp:1000
            for filament in self.ctx.machine_info.machine_filament_info[eid].iter() {
                // FilamentGroup.cpp:1001  machine_filament_set[filament].insert(machine_filament_list.size());
                let slot = mfi_find_or_insert(&mut machine_filament_set, filament);
                machine_filament_set[slot].1.insert(machine_filament_list.len() as i32);
                // FilamentGroup.cpp:1002
                machine_filament_list.push(filament.clone());
            }
        }

        // FilamentGroup.cpp:1006
        if machine_filament_list.is_empty() {
            // FilamentGroup.cpp:1007
            return Err(FilamentGroupException::new(
                ErrorCode::EmptyAmsFilaments,
                "Empty ams filament in For-Match mode.".to_string(),
            ));
        }

        // FilamentGroup.cpp:1009
        // key stores filament idx in used_filament, value stores unprintable extruder
        // FilamentGroup.cpp:1010
        let mut unprintable_limit_indices: BTreeMap<i32, i32> = BTreeMap::new();
        extract_unprintable_limit_indices_map(
            &self.ctx.model_info.unprintable_filaments,
            &used_filaments,
            &mut unprintable_limit_indices,
        );
        // FilamentGroup.cpp:1011
        unprintable_limit_indices =
            self.rebuild_unprintables(&used_filaments, &unprintable_limit_indices);

        // FilamentGroup.cpp:1013
        let mut color_dist_matrix: Vec<Vec<f32>> =
            vec![vec![0.0f32; machine_filament_list.len()]; used_filament_list.len()];
        // FilamentGroup.cpp:1014
        for i in 0..used_filament_list.len() {
            // FilamentGroup.cpp:1015
            for j in 0..machine_filament_list.len() {
                // FilamentGroup.cpp:1016
                color_dist_matrix[i][j] = calc_color_distance(
                    &RGBColor::new(
                        used_filament_list[i].color.r,
                        used_filament_list[i].color.g,
                        used_filament_list[i].color.b,
                    ),
                    &RGBColor::new(
                        machine_filament_list[j].color.r,
                        machine_filament_list[j].color.g,
                        machine_filament_list[j].color.b,
                    ),
                );
            }
        }

        // FilamentGroup.cpp:1023
        let mut l_nodes: Vec<i32> = (0..used_filaments.len() as i32).collect();
        // FilamentGroup.cpp:1025
        let r_nodes: Vec<i32> = (0..machine_filament_list.len() as i32).collect();
        // FilamentGroup.cpp:1027
        let machine_filament_capacity: Vec<i32> =
            vec![l_nodes.len() as i32; machine_filament_list.len()];
        // FilamentGroup.cpp:1028
        let mut extruder_filament_count: Vec<i32> = vec![0; 2];

        // FilamentGroup.cpp:1030  auto is_extruder_filament_compatible = [&unprintable_limit_indices](int filament_idx, int extruder_id)
        let is_extruder_filament_compatible = |filament_idx: i32, extruder_id: i32| -> bool {
            // FilamentGroup.cpp:1031-1033
            if let Some(&v) = unprintable_limit_indices.get(&filament_idx) {
                if v == extruder_id {
                    return false;
                }
            }
            // FilamentGroup.cpp:1034
            true
        };

        // FilamentGroup.cpp:1037  auto build_unlink_limits = [](const l_nodes, const r_nodes, const can_link)
        let build_unlink_limits =
            |l_nodes: &[i32], r_nodes: &[i32], can_link: &dyn Fn(i32, i32) -> bool| -> HashMap<i32, Vec<i32>> {
                // FilamentGroup.cpp:1038
                let mut unlink_limits: HashMap<i32, Vec<i32>> = HashMap::new();
                // FilamentGroup.cpp:1039
                for i in 0..l_nodes.len() {
                    // FilamentGroup.cpp:1040
                    let mut unlink_filaments: Vec<i32> = Vec::new();
                    // FilamentGroup.cpp:1041
                    for j in 0..r_nodes.len() {
                        // FilamentGroup.cpp:1042
                        if !can_link(l_nodes[i], r_nodes[j]) {
                            // FilamentGroup.cpp:1043
                            unlink_filaments.push(j as i32);
                        }
                    }
                    // FilamentGroup.cpp:1045
                    if !unlink_filaments.is_empty() {
                        // FilamentGroup.cpp:1046
                        unlink_limits.insert(i as i32, unlink_filaments);
                    }
                }
                // FilamentGroup.cpp:1048
                unlink_limits
            };

        // FilamentGroup.cpp:1051  auto optimize_map_to_machine_filament = [&](...)
        // (Defined as a closure in C++; implemented here as a local helper that takes
        //  the mutable state it captures by reference.)
        let optimize_map_to_machine_filament =
            |map_to_machine_filament: &[i32],
             l_nodes: &[i32],
             r_nodes: &[i32],
             filament_map: &mut [i32],
             extruder_filament_count: &mut [i32]|
             -> Vec<i32> {
                // FilamentGroup.cpp:1052
                let mut ungrouped_filaments: Vec<i32> = Vec::new();
                // FilamentGroup.cpp:1053
                let mut filaments_to_optimize: Vec<i32> = Vec::new();

                // FilamentGroup.cpp:1055  auto map_filament_to_machine_filament = [&](int filament_idx, int machine_filament_idx)
                // FilamentGroup.cpp:1060  auto unmap_filament_to_machine_filament = [&](...)
                // (inlined below)

                // FilamentGroup.cpp:1065
                for idx in 0..map_to_machine_filament.len() {
                    // FilamentGroup.cpp:1066
                    if map_to_machine_filament[idx] == INVALID_ID {
                        // FilamentGroup.cpp:1067
                        ungrouped_filaments.push(l_nodes[idx]);
                        // FilamentGroup.cpp:1068
                        continue;
                    }
                    // FilamentGroup.cpp:1070
                    let used_filament_idx = l_nodes[idx];
                    // FilamentGroup.cpp:1071
                    let machine_filament_idx = r_nodes[map_to_machine_filament[idx] as usize];
                    // FilamentGroup.cpp:1072
                    let machine_filament = machine_filament_list[machine_filament_idx as usize].clone();
                    // FilamentGroup.cpp:1073  if (machine_filament_set[machine_filament].size() > 1 && unprintable_limit_indices.count(used_filament_idx) == 0)
                    let mf_slot = {
                        let mut found = None;
                        for (i, (k, _)) in machine_filament_set.iter().enumerate() {
                            if !k.lt(&machine_filament) && !machine_filament.lt(k) {
                                found = Some(i);
                                break;
                            }
                        }
                        found.unwrap()
                    };
                    if machine_filament_set[mf_slot].1.len() > 1
                        && !unprintable_limit_indices.contains_key(&used_filament_idx)
                    {
                        // FilamentGroup.cpp:1074
                        filaments_to_optimize.push(idx as i32);
                    }

                    // FilamentGroup.cpp:1076  map_filament_to_machine_filament(used_filament_idx, machine_filament_idx);
                    // FilamentGroup.cpp:1057
                    filament_map[used_filaments[used_filament_idx as usize] as usize] =
                        machine_filament.extruder_id;
                    // FilamentGroup.cpp:1058
                    extruder_filament_count[machine_filament.extruder_id as usize] += 1;
                }
                // FilamentGroup.cpp:1078
                // try to optimize the result
                // FilamentGroup.cpp:1079
                for &idx in filaments_to_optimize.iter() {
                    // FilamentGroup.cpp:1080
                    let filament_idx = l_nodes[idx as usize];
                    // FilamentGroup.cpp:1081
                    let is_support_filament = used_filament_list[filament_idx as usize].usage_type
                        == FilamentUsageType::SupportOnly;
                    // FilamentGroup.cpp:1082
                    let old_machine_filament_idx =
                        r_nodes[map_to_machine_filament[idx as usize] as usize];
                    // FilamentGroup.cpp:1083
                    let old_machine_filament =
                        machine_filament_list[old_machine_filament_idx as usize].clone();

                    // FilamentGroup.cpp:1085  unmap_filament_to_machine_filament(filament_idx, old_machine_filament_idx);
                    // FilamentGroup.cpp:1062
                    extruder_filament_count[old_machine_filament.extruder_id as usize] -= 1;

                    // FilamentGroup.cpp:1087  auto optional_filaments = machine_filament_set[old_machine_filament];
                    let old_slot = {
                        let mut found = None;
                        for (i, (k, _)) in machine_filament_set.iter().enumerate() {
                            if !k.lt(&old_machine_filament) && !old_machine_filament.lt(k) {
                                found = Some(i);
                                break;
                            }
                        }
                        found.unwrap()
                    };
                    let optional_filaments: Vec<i32> =
                        machine_filament_set[old_slot].1.iter().copied().collect();

                    // FilamentGroup.cpp:1089  第一阶段：找出所有满足容量约束的候选方案，并计算它们的偏好得分
                    // FilamentGroup.cpp:1090  std::vector<std::pair<int, int>> valid_candidates;
                    let mut valid_candidates: Vec<(i32, i32)> = Vec::new();
                    // FilamentGroup.cpp:1091
                    for machine_filament in optional_filaments.iter() {
                        // FilamentGroup.cpp:1092
                        let new_extruder_id =
                            machine_filament_list[*machine_filament as usize].extruder_id;

                        // FilamentGroup.cpp:1094  计算新分配的偏好得分
                        // FilamentGroup.cpp:1095
                        let mut preference_score: i32 = 0;
                        // FilamentGroup.cpp:1096
                        let new_extruder_prefer_support =
                            self.ctx.machine_info.prefer_non_model_filament[new_extruder_id as usize];

                        // FilamentGroup.cpp:1098  如果是支撑材料且分配给了偏好支撑的喷嘴，给予奖励
                        // FilamentGroup.cpp:1099
                        if is_support_filament && new_extruder_prefer_support {
                            // FilamentGroup.cpp:1100  preference_score += SupportPreferScore;
                            preference_score += SUPPORT_PREFER_SCORE;
                        }

                        // FilamentGroup.cpp:1103
                        valid_candidates.push((*machine_filament, preference_score));
                    }
                    // FilamentGroup.cpp:1105  第二阶段：确定最佳偏好得分
                    // FilamentGroup.cpp:1106
                    let mut best_preference_score: i32 = 0;
                    // FilamentGroup.cpp:1107
                    for candidate in valid_candidates.iter() {
                        // FilamentGroup.cpp:1108
                        if candidate.1 >= best_preference_score {
                            // FilamentGroup.cpp:1109
                            best_preference_score = candidate.1;
                        }
                    }

                    // FilamentGroup.cpp:1113  第三阶段：在最佳偏好得分的候选方案中选择最均衡负载的方案
                    // FilamentGroup.cpp:1114
                    let mut best_candidate: i32 = -1;
                    // FilamentGroup.cpp:1115
                    let mut best_gap: i32 = i32::MAX;

                    // FilamentGroup.cpp:1117
                    for candidate in valid_candidates.iter() {
                        // FilamentGroup.cpp:1118  只考虑具有最佳偏好得分的候选方案
                        // FilamentGroup.cpp:1119
                        let machine_filament = candidate.0;
                        // FilamentGroup.cpp:1120
                        let score = candidate.1;
                        // FilamentGroup.cpp:1121
                        if score == best_preference_score {
                            // FilamentGroup.cpp:1122
                            let new_extruder_id =
                                machine_filament_list[machine_filament as usize].extruder_id;
                            // FilamentGroup.cpp:1123
                            let new_gap = (extruder_filament_count[new_extruder_id as usize] + 1
                                - extruder_filament_count[(1 - new_extruder_id) as usize])
                            .abs();

                            // FilamentGroup.cpp:1125  在偏好得分相同的方案中寻找负载最均衡的选项
                            // FilamentGroup.cpp:1126
                            if new_gap < best_gap {
                                // FilamentGroup.cpp:1127
                                best_gap = new_gap;
                                // FilamentGroup.cpp:1128
                                best_candidate = machine_filament;
                            }
                        }
                    }
                    // FilamentGroup.cpp:1132  应用最佳选择
                    // FilamentGroup.cpp:1133
                    if best_candidate != -1 {
                        // FilamentGroup.cpp:1134  map_filament_to_machine_filament(filament_idx, best_candidate);
                        let machine_filament = machine_filament_list[best_candidate as usize].clone();
                        filament_map[used_filaments[filament_idx as usize] as usize] =
                            machine_filament.extruder_id;
                        extruder_filament_count[machine_filament.extruder_id as usize] += 1;
                    }
                    // FilamentGroup.cpp:1135
                    else {
                        // FilamentGroup.cpp:1136  map_filament_to_machine_filament(filament_idx, old_machine_filament_idx);
                        let machine_filament =
                            machine_filament_list[old_machine_filament_idx as usize].clone();
                        filament_map[used_filaments[filament_idx as usize] as usize] =
                            machine_filament.extruder_id;
                        extruder_filament_count[machine_filament.extruder_id as usize] += 1;
                    }
                }
                // FilamentGroup.cpp:1139
                ungrouped_filaments
            };

        // FilamentGroup.cpp:1142
        let mut group: Vec<i32> = vec![
            self.ctx.machine_info.master_extruder_id;
            self.ctx.group_info.total_filament_num as usize
        ];
        // FilamentGroup.cpp:1143
        #[allow(unused_assignments)]
        let mut ungrouped_filaments: Vec<i32>;

        // FilamentGroup.cpp:1145  auto unlink_limits_full = build_unlink_limits(l_nodes, r_nodes, [...](used, machine))
        let unlink_limits_full = build_unlink_limits(
            &l_nodes,
            &r_nodes,
            &|used_filament_idx: i32, machine_filament_idx: i32| -> bool {
                // FilamentGroup.cpp:1146-1148
                used_filament_list[used_filament_idx as usize].type_
                    == machine_filament_list[machine_filament_idx as usize].type_
                    && used_filament_list[used_filament_idx as usize].is_support
                        == machine_filament_list[machine_filament_idx as usize].is_support
                    && is_extruder_filament_compatible(
                        used_filament_idx,
                        machine_filament_list[machine_filament_idx as usize].extruder_id,
                    )
            },
        );

        // FilamentGroup.cpp:1151
        {
            // FilamentGroup.cpp:1152
            let mut s = MatchModeGroupSolver::new(
                &color_dist_matrix,
                &l_nodes,
                &r_nodes,
                &machine_filament_capacity,
                &unlink_limits_full,
            );
            // FilamentGroup.cpp:1153
            ungrouped_filaments = optimize_map_to_machine_filament(
                &s.solve(),
                &l_nodes,
                &r_nodes,
                &mut group,
                &mut extruder_filament_count,
            );
            // FilamentGroup.cpp:1154
            if ungrouped_filaments.is_empty() {
                // FilamentGroup.cpp:1155
                return Ok(group);
            }
        }

        // FilamentGroup.cpp:1158
        // additionally remove type limits
        // FilamentGroup.cpp:1159
        {
            // FilamentGroup.cpp:1160
            l_nodes = ungrouped_filaments.clone();
            // FilamentGroup.cpp:1161  auto unlink_limits = build_unlink_limits(l_nodes, r_nodes, [...](used, machine))
            let unlink_limits = build_unlink_limits(
                &l_nodes,
                &r_nodes,
                &|used_filament_idx: i32, machine_filament_idx: i32| -> bool {
                    // FilamentGroup.cpp:1162
                    is_extruder_filament_compatible(
                        used_filament_idx,
                        machine_filament_list[machine_filament_idx as usize].extruder_id,
                    )
                },
            );

            // FilamentGroup.cpp:1165
            let mut s = MatchModeGroupSolver::new(
                &color_dist_matrix,
                &l_nodes,
                &r_nodes,
                &machine_filament_capacity,
                &unlink_limits,
            );
            // FilamentGroup.cpp:1166
            ungrouped_filaments = optimize_map_to_machine_filament(
                &s.solve(),
                &l_nodes,
                &r_nodes,
                &mut group,
                &mut extruder_filament_count,
            );
            // FilamentGroup.cpp:1167
            if ungrouped_filaments.is_empty() {
                // FilamentGroup.cpp:1168
                return Ok(group);
            }
        }

        // FilamentGroup.cpp:1171
        // remove all limits
        // FilamentGroup.cpp:1172
        {
            // FilamentGroup.cpp:1173
            l_nodes = ungrouped_filaments.clone();
            // FilamentGroup.cpp:1174
            let mut s = MatchModeGroupSolver::new(
                &color_dist_matrix,
                &l_nodes,
                &r_nodes,
                &machine_filament_capacity,
                &HashMap::new(),
            );
            // FilamentGroup.cpp:1175
            let ret = optimize_map_to_machine_filament(
                &s.solve(),
                &l_nodes,
                &r_nodes,
                &mut group,
                &mut extruder_filament_count,
            );
            // FilamentGroup.cpp:1176
            for idx in 0..ret.len() {
                // FilamentGroup.cpp:1177
                if ret[idx] == INVALID_ID {
                    // FilamentGroup.cpp:1178
                    debug_assert!(false);
                }
                // FilamentGroup.cpp:1179
                else {
                    // FilamentGroup.cpp:1180
                    group[used_filaments[l_nodes[idx] as usize] as usize] =
                        machine_filament_list[r_nodes[ret[idx] as usize] as usize].extruder_id;
                }
            }
        }

        // FilamentGroup.cpp:1184
        Ok(group)
    }

    // FilamentGroup.cpp:1204
    /// `std::vector<int> FilamentGroup::calc_filament_group_for_tpu(int *cost)`
    pub fn calc_filament_group_for_tpu(&mut self, _cost: Option<&mut i32>) -> Vec<i32> {
        // FilamentGroup.cpp:1206
        let used_filaments = collect_sorted_used_filaments(&self.ctx.model_info.layer_filaments);
        // FilamentGroup.cpp:1207
        let mut used_filament_list: Vec<FilamentInfo> = Vec::new();
        // FilamentGroup.cpp:1208
        for &f in used_filaments.iter() {
            // FilamentGroup.cpp:1209
            used_filament_list.push(self.ctx.model_info.filament_info[f as usize].clone());
        }
        let _ = &used_filament_list;

        // FilamentGroup.cpp:1211
        let mut print_time_matrix: Vec<Vec<f32>> = vec![
            vec![0.0f32; self.ctx.nozzle_info.extruder_nozzle_list.len()];
            used_filaments.len()
        ];
        // FilamentGroup.cpp:1212
        for i in 0..used_filaments.len() {
            // FilamentGroup.cpp:1213
            for j in 0..self.ctx.nozzle_info.extruder_nozzle_list.len() {
                // FilamentGroup.cpp:1214  print_time_matrix[i][j] = ctx.speed_info.filament_print_time[used_filaments[i]][j];
                print_time_matrix[i][j] = self
                    .ctx
                    .speed_info
                    .filament_print_time
                    .get(&(used_filaments[i] as i32))
                    .and_then(|m| m.get(&(j as i32)))
                    .copied()
                    .unwrap_or(0.0) as f32;
                // FilamentGroup.cpp:1215  同时存在TPU High Flow喷嘴和其他类型喷嘴时，优先将耗材分配至TPU High Flow喷嘴
                if self.ctx.nozzle_info.nozzle_list[j].volume_type
                    == NozzleVolumeType::NvtTPUHighFlow
                {
                    // FilamentGroup.cpp:1216
                    print_time_matrix[i][j] *= 0.9;
                }
            }
        }

        // FilamentGroup.cpp:1220
        let l_nodes: Vec<i32> = (0..used_filaments.len() as i32).collect();
        // FilamentGroup.cpp:1222
        let r_nodes: Vec<i32> = (0..self.ctx.nozzle_info.extruder_nozzle_list.len() as i32).collect();
        // FilamentGroup.cpp:1224
        let machine_filament_capacity: Vec<i32> =
            vec![used_filaments.len() as i32, used_filaments.len() as i32];

        // FilamentGroup.cpp:1226
        // key stores filament idx in used_filament, value stores unprintable extruder
        // FilamentGroup.cpp:1227
        let mut unprintable_limit_indices: BTreeMap<i32, i32> = BTreeMap::new();
        extract_unprintable_limit_indices_map(
            &self.ctx.model_info.unprintable_filaments,
            &used_filaments,
            &mut unprintable_limit_indices,
        );
        // FilamentGroup.cpp:1228
        unprintable_limit_indices =
            self.rebuild_unprintables(&used_filaments, &unprintable_limit_indices);

        // FilamentGroup.cpp:1230
        let mut unlink_limits: HashMap<i32, Vec<i32>> = HashMap::new();
        // FilamentGroup.cpp:1231
        for i in 0..used_filaments.len() as i32 {
            // FilamentGroup.cpp:1232
            // FilamentGroup.cpp:1233  if (iter == ...end() || iter->second < 0 || iter->second >= 2) continue;
            match unprintable_limit_indices.get(&i) {
                None => continue,
                Some(&v) => {
                    if v < 0 || v >= 2 {
                        continue;
                    }
                    // FilamentGroup.cpp:1234
                    unlink_limits.entry(i).or_default().push(v);
                }
            }
        }

        // FilamentGroup.cpp:1237
        let mut s = MatchModeGroupSolver::new(
            &print_time_matrix,
            &l_nodes,
            &r_nodes,
            &machine_filament_capacity,
            &unlink_limits,
        );
        // FilamentGroup.cpp:1238
        let mut ret = s.solve();
        // FilamentGroup.cpp:1239
        for idx in 0..ret.len() {
            // FilamentGroup.cpp:1240
            if ret[idx] == INVALID_ID {
                // FilamentGroup.cpp:1241
                debug_assert!(false);
                // FilamentGroup.cpp:1242
                ret[idx] = 1;
            }
        }
        // FilamentGroup.cpp:1245
        let mut group: Vec<i32> = vec![
            self.ctx.machine_info.master_extruder_id;
            self.ctx.group_info.total_filament_num as usize
        ];
        // FilamentGroup.cpp:1246
        for i in 0..ret.len() {
            group[used_filaments[i] as usize] = ret[i];
        }
        // FilamentGroup.cpp:1247
        group
    }
}

// =============================================================================
// FilamentGroupMultiNozzle (FilamentGroup.hpp:177, FilamentGroup.cpp:1422+)
// =============================================================================

// FilamentGroup.hpp:177
pub struct FilamentGroupMultiNozzle {
    // FilamentGroup.hpp:188
    m_context: FilamentGroupContext,
}

impl FilamentGroupMultiNozzle {
    // FilamentGroup.hpp:180
    /// `FilamentGroupMultiNozzle(const FilamentGroupContext& context) : m_context(context) {}`
    pub fn new(context: FilamentGroupContext) -> Self {
        FilamentGroupMultiNozzle { m_context: context }
    }

    // FilamentGroup.cpp:1422
    /// `std::unordered_map<int, std::vector<int>> FilamentGroupMultiNozzle::rebuild_nozzle_unprintables(...)`
    fn rebuild_nozzle_unprintables(
        &self,
        used_filaments: &[u32],
        extruder_unprintables: &HashMap<i32, Vec<i32>>,
        filament_volume_map: &[i32],
    ) -> HashMap<i32, Vec<i32>> {
        // FilamentGroup.cpp:1424
        let mut nozzle_unprintables: HashMap<i32, Vec<i32>> = HashMap::new();

        // FilamentGroup.cpp:1426
        for fidx in 0..used_filaments.len() {
            // FilamentGroup.cpp:1427  NozzleVolumeType expected_volume = NozzleVolumeType(filament_volume_map[used_filaments[fidx]]);
            let expected_volume =
                NozzleVolumeType::from_i32(filament_volume_map[used_filaments[fidx] as usize]);
            // FilamentGroup.cpp:1428
            let mut unexpected_extruders: Vec<i32> = Vec::new();
            // FilamentGroup.cpp:1429
            if let Some(v) = extruder_unprintables.get(&(fidx as i32)) {
                // FilamentGroup.cpp:1430
                unexpected_extruders = v.clone();
            }

            // FilamentGroup.cpp:1433  auto unprintable_volumes = m_context.model_info.unprintable_volumes[used_filaments[fidx]];
            let empty_set: BTreeSet<NozzleVolumeType> = BTreeSet::new();
            let unprintable_volumes = self
                .m_context
                .model_info
                .unprintable_volumes
                .get(&(used_filaments[fidx] as i32))
                .unwrap_or(&empty_set);

            // FilamentGroup.cpp:1435
            let mut unprintable_nozzles: Vec<i32> = Vec::new();
            // FilamentGroup.cpp:1436
            for nozzle_idx in 0..self.m_context.nozzle_info.nozzle_list.len() {
                // FilamentGroup.cpp:1437
                let nozzle_info = &self.m_context.nozzle_info.nozzle_list[nozzle_idx];

                // FilamentGroup.cpp:1439-1440
                if unexpected_extruders.contains(&nozzle_info.extruder_id)
                    || (expected_volume != NozzleVolumeType::NvtHybrid
                        && expected_volume != nozzle_info.volume_type)
                    || unprintable_volumes.contains(&nozzle_info.volume_type)
                {
                    // FilamentGroup.cpp:1441
                    unprintable_nozzles.push(nozzle_idx as i32);
                }
            }
            // FilamentGroup.cpp:1443
            if unprintable_nozzles.is_empty() {
                // FilamentGroup.cpp:1444
                continue;
            }

            // FilamentGroup.cpp:1446  sort_remove_duplicates(unprintable_nozzles);
            unprintable_nozzles.sort_unstable();
            unprintable_nozzles.dedup();
            // FilamentGroup.cpp:1447
            nozzle_unprintables.insert(fidx as i32, unprintable_nozzles);
        }

        // FilamentGroup.cpp:1450
        nozzle_unprintables
    }

    // FilamentGroup.cpp:1453
    /// `std::vector<int> FilamentGroupMultiNozzle::calc_filament_group_by_mcmf()`
    pub fn calc_filament_group_by_mcmf(&mut self) -> Vec<i32> {
        // FilamentGroup.cpp:1455
        let used_filaments =
            collect_sorted_used_filaments(&self.m_context.model_info.layer_filaments);

        // FilamentGroup.cpp:1457
        let mut unplaceable_limits: BTreeMap<i32, i32> = BTreeMap::new();
        extract_unprintable_limit_indices_map(
            &self.m_context.model_info.unprintable_filaments,
            &used_filaments,
            &mut unplaceable_limits,
        );

        // FilamentGroup.cpp:1460
        let distance_evaluator = std::rc::Rc::new(FlushDistanceEvaluator::new(
            &self.m_context.model_info.flush_matrix,
            &used_filaments,
            &self.m_context.model_info.layer_filaments,
            0.65,
        ));
        // FilamentGroup.cpp:1461
        let mut groups: Vec<BTreeSet<i32>> = vec![BTreeSet::new(); 2];

        // FilamentGroup.cpp:1463
        // first cluster
        // FilamentGroup.cpp:1464
        {
            // FilamentGroup.cpp:1465
            let mut pam = KMediods2::new(
                used_filaments.len() as i32,
                distance_evaluator.clone(),
                0,
            );
            // FilamentGroup.cpp:1466
            pam.set_max_cluster_size(vec![
                self.m_context.nozzle_info.extruder_nozzle_list[&0].len() as i32,
                self.m_context.nozzle_info.extruder_nozzle_list[&1].len() as i32,
            ]);
            // FilamentGroup.cpp:1467
            pam.set_unplaceable_limits(unplaceable_limits.clone());
            // FilamentGroup.cpp:1468
            pam.do_clustering(FGStrategy::BestFit, 100);
            // FilamentGroup.cpp:1469
            let first_clustered_labels = pam.get_cluster_labels();
            // FilamentGroup.cpp:1470
            let total_nozzle_num = self.m_context.nozzle_info.nozzle_list.len();

            // FilamentGroup.cpp:1472
            if total_nozzle_num > used_filaments.len() {
                // FilamentGroup.cpp:1473
                let mut ret: Vec<i32> =
                    vec![0; self.m_context.group_info.total_filament_num as usize];
                // FilamentGroup.cpp:1474
                for idx in 0..first_clustered_labels.len() {
                    // FilamentGroup.cpp:1475
                    ret[used_filaments[idx] as usize] = first_clustered_labels[idx];
                }
                // FilamentGroup.cpp:1477
                return ret;
            }

            // FilamentGroup.cpp:1480
            // first place the elem if it follows the limit
            // FilamentGroup.cpp:1481
            for idx in 0..first_clustered_labels.len() {
                // FilamentGroup.cpp:1482
                if unplaceable_limits.contains_key(&(idx as i32)) {
                    // FilamentGroup.cpp:1483
                    groups[first_clustered_labels[idx] as usize].insert(idx as i32);
                }
            }
            // FilamentGroup.cpp:1485
            // then fullfill the nozzle with other filaments
            // FilamentGroup.cpp:1486
            for idx in 0..first_clustered_labels.len() {
                // FilamentGroup.cpp:1487
                // place the elem in first cluster if the elem follow the limit
                // FilamentGroup.cpp:1488
                let gidx = first_clustered_labels[idx];
                // FilamentGroup.cpp:1489
                if (groups[gidx as usize].len())
                    < self.m_context.nozzle_info.extruder_nozzle_list[&gidx].len()
                {
                    // FilamentGroup.cpp:1490
                    groups[gidx as usize].insert(idx as i32);
                }
            }
        }

        // FilamentGroup.cpp:1494
        let mut ret_map: Vec<i32> = vec![0; self.m_context.group_info.total_filament_num as usize];
        // FilamentGroup.cpp:1495
        // second cluster
        // FilamentGroup.cpp:1496
        {
            // FilamentGroup.cpp:1497
            let mut unplaceable_limits: BTreeMap<i32, i32> = BTreeMap::new();
            // FilamentGroup.cpp:1498
            for idx in 0..groups.len() {
                // FilamentGroup.cpp:1499
                for &f in groups[idx].iter() {
                    // FilamentGroup.cpp:1500  unplaceable_limits.emplace(f, (int)(1 - idx));
                    unplaceable_limits.entry(f).or_insert(1 - idx as i32);
                }
            }
            // FilamentGroup.cpp:1502
            let mut pam = KMediods2::new(
                used_filaments.len() as i32,
                distance_evaluator.clone(),
                0,
            );
            // FilamentGroup.cpp:1503
            pam.set_max_cluster_size(self.m_context.machine_info.max_group_size.clone());
            // FilamentGroup.cpp:1504
            pam.set_unplaceable_limits(unplaceable_limits);
            // FilamentGroup.cpp:1505
            pam.do_clustering(FGStrategy::BestFit, 100);
            // FilamentGroup.cpp:1506
            let labels = pam.get_cluster_labels();

            // FilamentGroup.cpp:1508
            for idx in 0..labels.len() {
                // FilamentGroup.cpp:1509
                ret_map[used_filaments[idx] as usize] = labels[idx];
            }
        }
        // FilamentGroup.cpp:1511
        ret_map
    }
}

// =============================================================================
// Free functions (FilamentGroup.cpp:1556+, FilamentGroup.cpp:1701+)
// =============================================================================

// FilamentGroup.cpp:1701
/// `std::vector<int> calc_filament_group_for_manual_multi_nozzle(const std::vector<int>& filament_map_manual, const FilamentGroupContext& ctx)`
///
/// NOTE: Calls `FilamentGroupMultiNozzle::calc_filament_group_by_pam`, which is a
/// blocked symbol (depends on the unported `get_estimate_extruder_filament_change_count`
/// used in `KMediods::cluster_small_data`). Documented as blocked; not ported here.
// (blocked — see module-level docs)

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_evaluate_score_no_time() {
        // FilamentGroup.cpp:58 — with_time=false returns flush unchanged.
        assert_eq!(evaluate_score(123.0, 99.0, false), 123.0);
    }

    #[test]
    fn test_evaluate_score_with_time() {
        // FilamentGroup.cpp:64-65
        let flush = 1000.0;
        let time = 50.0;
        let expected = flush * 1.26 * 180.0 * 2.0 / 1000.0 + time;
        assert_eq!(evaluate_score(flush, time, true), expected);
    }

    #[test]
    fn test_fnv_hash_two_ints_deterministic() {
        // FilamentGroup.cpp:42 — same inputs produce same output.
        let a = fnv_hash_two_ints(3, 7);
        let b = fnv_hash_two_ints(3, 7);
        assert_eq!(a, b);
        // distinct inputs (very likely) produce distinct output
        assert_ne!(fnv_hash_two_ints(3, 7), fnv_hash_two_ints(7, 3));
    }

    #[test]
    fn test_collect_sorted_used_filaments() {
        // FilamentGroup.cpp:215 — unique + sorted.
        let layers = vec![vec![3u32, 1, 1], vec![2u32, 3], vec![]];
        assert_eq!(collect_sorted_used_filaments(&layers), vec![1, 2, 3]);
    }

    #[test]
    fn test_memoryed_group_gt() {
        // FilamentGroup.hpp:58 — operator>
        // higher prefer_level => NOT greater (prefer_level < other.prefer_level)
        let a = MemoryedGroup::new(vec![], 10, 1);
        let b = MemoryedGroup::new(vec![], 10, 2);
        assert!(a.gt(&b)); // a.prefer(1) < b.prefer(2) => true
        assert!(!b.gt(&a));
        // equal prefer, higher cost => greater
        let c = MemoryedGroup::new(vec![], 20, 1);
        assert!(c.gt(&a)); // same prefer, c.cost(20) > a.cost(10)
    }

    #[test]
    fn test_update_memoryed_groups_keeps_highest_prefer() {
        // FilamentGroup.cpp:170 — only keep the highest prefer_level items;
        // top() must be the highest-prefer / lowest-cost group.
        let mut heap: MemoryedGroupHeap = MemoryedGroupHeap::new();
        update_memoryed_groups(&MemoryedGroup::new(vec![0, 1], 100, 1), 1.0, &mut heap);
        // higher prefer level rebuilds the heap
        update_memoryed_groups(&MemoryedGroup::new(vec![1, 0], 200, 2), 1.0, &mut heap);
        let top = heap.peek().unwrap().0.clone();
        assert_eq!(top.prefer_level, 2);
        assert_eq!(top.group, vec![1, 0]);
    }
}
