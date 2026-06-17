//! Smoothing.rs - 1:1 faithful port of BambuStudio `GCode/Smoothing.cpp` (+ `.hpp`).
//!
//! C++ Reference:
//! - GCode/Smoothing.hpp
//! - GCode/Smoothing.cpp
//!
//! The `SmoothCalculator` applies gaussian smoothing passes to the outer-wall
//! feedrate and to the per-layer cooling time, so that the cooling-driven speed
//! changes vary smoothly between layers (avoiding visible surface defects from
//! abrupt speed steps).
//!
//! coord_t -> i64, coordf_t -> f64.  The C++ uses `std::map<int, CoolingNode>`
//! (an ordered map) so we mirror it with `BTreeMap<i32, CoolingNode>`.

// Smoothing.hpp:1-5
use std::collections::BTreeMap;

use crate::gcode::g_code_editor::{CoolingLineType, PerExtruderAdjustments};

// Smoothing.hpp:9-13
const GUASSIAN_WINDOW_SIZE: i32 = 11;
const GUASSIAN_R: i32 = 2;
const GUASSIAN_STOP_THRESHOLD: i32 = 5;
const GUASSIAN_LAYER_TIME_STOP_THRESHOLD: f32 = 3.0;
const MAX_STEPS_COUNT: i32 = 1000;

// Smoothing.hpp:15-22
#[derive(Debug, Clone)]
pub struct CoolingNode {
    // extruder pos, line pos;
    pub outwall_line: Vec<(i32, i32)>,
    pub max_feedrate: f32,
    pub filter_feedrate: f32,
    pub rate: f64,
}

impl Default for CoolingNode {
    fn default() -> Self {
        Self {
            outwall_line: Vec::new(),
            max_feedrate: 0.0,
            filter_feedrate: 0.0,
            rate: 1.0,
        }
    }
}

impl CoolingNode {
    pub fn new() -> Self {
        Self::default()
    }
}

// Smoothing.hpp:24-28
#[derive(Debug, Clone, Default)]
pub struct OutwallCollection {
    pub object_id: i32,
    pub cooling_nodes: BTreeMap<i32, CoolingNode>,
}

impl OutwallCollection {
    pub fn new() -> Self {
        Self::default()
    }
}

// Smoothing.hpp:30-99
pub struct SmoothCalculator {
    // public:
    pub objects_node_range: Vec<BTreeMap<i32, (i32, i32)>>,
    pub layers_wall_collection: Vec<Vec<OutwallCollection>>,
    pub layers_cooling_time: Vec<f32>,

    // private:
    // guassian filter
    guassian_filter: Vec<f64>,
    filter_sum: f64,
    layer_time_smoothing_threshold: f32,
}

impl SmoothCalculator {
    // Smoothing.hpp:38-42
    pub fn with_gap_limit(objects_size: i32, gap_limit: f64) -> Self {
        let mut calc = Self {
            objects_node_range: Vec::new(),
            layers_wall_collection: Vec::new(),
            layers_cooling_time: Vec::new(),
            guassian_filter: Vec::new(),
            filter_sum: 0.0f32 as f64,
            layer_time_smoothing_threshold: gap_limit as f32,
        };
        calc.guassian_filter_generator();
        calc.objects_node_range
            .resize_with(objects_size as usize, BTreeMap::new);
        calc
    }

    // Smoothing.hpp:44-48
    pub fn new(objects_size: i32) -> Self {
        let mut calc = Self {
            objects_node_range: Vec::new(),
            layers_wall_collection: Vec::new(),
            layers_cooling_time: Vec::new(),
            guassian_filter: Vec::new(),
            filter_sum: 0.0f32 as f64,
            layer_time_smoothing_threshold: 30.0f32,
        };
        calc.guassian_filter_generator();
        calc.objects_node_range
            .resize_with(objects_size as usize, BTreeMap::new);
        calc
    }

    // Smoothing.hpp:50-54
    pub fn append_data(&mut self, wall_collection: &[OutwallCollection], cooling_time: f32) {
        self.layers_wall_collection.push(wall_collection.to_vec());
        self.layers_cooling_time.push(cooling_time);
    }

    // Smoothing.hpp:56-59
    pub fn append_data_no_time(&mut self, wall_collection: &[OutwallCollection]) {
        self.layers_wall_collection.push(wall_collection.to_vec());
    }

    // Smoothing.cpp:5-43
    pub fn build_node(
        wall_collection: &mut Vec<OutwallCollection>,
        object_label: &[i32],
        per_extruder_adjustments: &[PerExtruderAdjustments],
    ) {
        if per_extruder_adjustments.is_empty() {
            return;
        }
        // BBS: update outwall feedrate
        // update feedrate of outwall after initial cooling process
        // initial and arrange node collection seq
        for object_idx in 0..object_label.len() {
            let mut object_level = OutwallCollection::new();
            object_level.object_id = object_label[object_idx];
            wall_collection.push(object_level);
        }

        for extruder_idx in 0..per_extruder_adjustments.len() {
            let extruder_adjustments = &per_extruder_adjustments[extruder_idx];
            for line_idx in 0..extruder_adjustments.lines.len() {
                let line = &extruder_adjustments.lines[line_idx];
                if line.outwall_smooth_mark {
                    // search node id
                    if !wall_collection[line.object_id as usize]
                        .cooling_nodes
                        .contains_key(&line.cooling_node_id)
                    {
                        let node = CoolingNode::new();
                        wall_collection[line.object_id as usize]
                            .cooling_nodes
                            .insert(line.cooling_node_id, node);
                    }

                    let node = wall_collection[line.object_id as usize]
                        .cooling_nodes
                        .get_mut(&line.cooling_node_id)
                        .unwrap();
                    if (line.line_type & CoolingLineType::EXTERNAL_PERIMETER) != 0 {
                        node.outwall_line
                            .push((line_idx as i32, extruder_idx as i32));
                        if node.max_feedrate < line.feedrate {
                            node.max_feedrate = line.feedrate;
                            node.filter_feedrate = node.max_feedrate;
                        }
                    }
                }
            }
        }
    }

    // Smoothing.cpp:71-88
    pub fn recaculate_layer_time(
        &mut self,
        layer_id: i32,
        extruder_adjustments: &mut [PerExtruderAdjustments],
    ) -> f32 {
        // rewrite feedrate
        for obj_id in 0..self.layers_wall_collection[layer_id as usize].len() {
            // NOTE: C++ iterates the std::map<int, CoolingNode> with `node_id` used
            // as the integer key 0..size() via operator[]. operator[] inserts a
            // default-constructed node if the key is absent, so the map can GROW and
            // the loop bound `cooling_nodes.size()` is RE-EVALUATED each iteration.
            // Mirror that exactly (re-read len() per iteration; entry() inserts).
            let mut node_id: usize = 0;
            while node_id
                < self.layers_wall_collection[layer_id as usize][obj_id]
                    .cooling_nodes
                    .len()
            {
                let node = self.layers_wall_collection[layer_id as usize][obj_id]
                    .cooling_nodes
                    .entry(node_id as i32)
                    .or_insert_with(CoolingNode::new)
                    .clone();
                // set outwall speed
                let rate = exclude_participate_in_speed_slowdown(
                    &node.outwall_line,
                    extruder_adjustments,
                    &node,
                );
                // Write back the rate mutated by exclude_participate_in_speed_slowdown.
                let stored = self.layers_wall_collection[layer_id as usize][obj_id]
                    .cooling_nodes
                    .get_mut(&(node_id as i32))
                    .unwrap();
                stored.rate = rate;

                node_id += 1;
            }
        }

        let mut layer_time = 0.0f32;
        for extruder in extruder_adjustments.iter() {
            layer_time += extruder.collection_line_times_of_extruder();
        }

        layer_time
    }

    // Smoothing.cpp:90-106
    pub fn init_object_node_range(&mut self) {
        for object_id in 0..self.objects_node_range.len() {
            for layer_id in 1..self.layers_wall_collection.len() {
                let each_object = &self.layers_wall_collection[layer_id][object_id];
                // auto it = each_object.cooling_nodes.begin();
                let keys: Vec<i32> = each_object.cooling_nodes.keys().copied().collect();
                for first in keys {
                    if !self.objects_node_range[object_id].contains_key(&first) {
                        self.objects_node_range[object_id]
                            .insert(first, (layer_id as i32, layer_id as i32));
                    } else {
                        self.objects_node_range[object_id].get_mut(&first).unwrap().1 =
                            layer_id as i32;
                    }
                }
            }
        }
    }

    // Smoothing.cpp:108-123
    pub fn smooth_layer_speed(&mut self) {
        self.init_object_node_range();

        for obj_id in 0..self.objects_node_range.len() {
            // auto it = objects_node_range[obj_id].begin();
            let node_ids: Vec<i32> = self.objects_node_range[obj_id].keys().copied().collect();
            for first in node_ids {
                let mut step_count = 0;
                while step_count < MAX_STEPS_COUNT
                    && self.speed_filter_continue(obj_id as i32, first)
                {
                    step_count += 1;
                    self.layer_speed_filter(obj_id as i32, first);
                }
            }
        }
    }

    // Smoothing.cpp:125-161
    fn layer_speed_filter(&mut self, object_id: i32, node_id: i32) {
        let start_pos = self.guassian_filter.len() as i32 / 2;
        // first layer don't need to be smoothed
        let layer_start = self.objects_node_range[object_id as usize][&node_id].0;
        let layer_end = self.objects_node_range[object_id as usize][&node_id].1;

        // BBS: some layers may empty as the support has indenpendent layer
        let mut layer_id = layer_start;
        while layer_id <= layer_end {
            if self.layers_wall_collection[layer_id as usize].is_empty() {
                layer_id += 1;
                continue;
            }

            if !self.layers_wall_collection[layer_id as usize][object_id as usize]
                .cooling_nodes
                .contains_key(&node_id)
            {
                break;
            }

            // node.outwall_line.empty() check (CoolingNode &node = ...)
            if self.layers_wall_collection[layer_id as usize][object_id as usize].cooling_nodes
                [&node_id]
                .outwall_line
                .is_empty()
            {
                layer_id += 1;
                continue;
            }

            let node_filter_feedrate = self.layers_wall_collection[layer_id as usize]
                [object_id as usize]
                .cooling_nodes[&node_id]
                .filter_feedrate;

            let mut conv_sum = 0.0f64;
            for filter_pos_idx in 0..self.guassian_filter.len() as i32 {
                let mut remap_data_pos = layer_id - start_pos + filter_pos_idx;

                if remap_data_pos < layer_start {
                    remap_data_pos = layer_start;
                } else if remap_data_pos > layer_end {
                    remap_data_pos = layer_end;
                }

                // some node may not start at layer 1
                // C++: layers_wall_collection[remap_data_pos][object_id].cooling_nodes[node_id]
                // uses operator[] which default-constructs the node if absent.
                let mut remap_data = node_filter_feedrate as f64;
                let remap_node = self.layers_wall_collection[remap_data_pos as usize]
                    [object_id as usize]
                    .cooling_nodes
                    .entry(node_id)
                    .or_insert_with(CoolingNode::new);
                if !remap_node.outwall_line.is_empty() {
                    remap_data = remap_node.filter_feedrate as f64;
                }

                conv_sum += self.guassian_filter[filter_pos_idx as usize] * remap_data;
            }
            // Smoothing.cpp:158-159  double filter_res = conv_sum / filter_sum;
            //   if (filter_res < node.filter_feedrate) node.filter_feedrate = filter_res;
            // C++ promotes node.filter_feedrate (float) to double for the compare,
            // then truncates the double back to float on assignment.
            let filter_res = conv_sum / self.filter_sum;
            let node = self.layers_wall_collection[layer_id as usize][object_id as usize]
                .cooling_nodes
                .get_mut(&node_id)
                .unwrap();
            if filter_res < node.filter_feedrate as f64 {
                node.filter_feedrate = filter_res as f32;
            }

            layer_id += 1;
        }
    }

    // Smoothing.cpp:163-175
    fn speed_filter_continue(&mut self, object_id: i32, node_id: i32) -> bool {
        let mut layer_id = self.objects_node_range[object_id as usize][&node_id].0;
        let layer_end = self.objects_node_range[object_id as usize][&node_id].1;

        // BBS: some layers may empty as the support has indenpendent layer
        while layer_id < layer_end {
            // C++ uses operator[] on the maps which default-constructs missing nodes.
            let next = self.layers_wall_collection[(layer_id + 1) as usize][object_id as usize]
                .cooling_nodes
                .entry(node_id)
                .or_insert_with(CoolingNode::new)
                .filter_feedrate;
            let cur = self.layers_wall_collection[layer_id as usize][object_id as usize]
                .cooling_nodes
                .entry(node_id)
                .or_insert_with(CoolingNode::new)
                .filter_feedrate;
            if (next - cur).abs() > GUASSIAN_STOP_THRESHOLD as f32 {
                return true;
            }
            layer_id += 1;
        }
        false
    }

    // Smoothing.cpp:177-202
    fn filter_layer_time(&mut self) {
        let start_pos = self.guassian_filter.len() as i32 / 2;
        // first layer don't need to be smoothed
        for layer_id in 1..self.layers_cooling_time.len() as i32 {
            if self.layers_cooling_time[layer_id as usize] > self.layer_time_smoothing_threshold {
                continue;
            }

            let mut conv_sum = 0.0f64;
            for filter_pos_idx in 0..self.guassian_filter.len() as i32 {
                let mut remap_data_pos = layer_id - start_pos + filter_pos_idx;

                if remap_data_pos < 1 {
                    remap_data_pos = 1;
                } else if remap_data_pos > self.layers_cooling_time.len() as i32 - 1 {
                    remap_data_pos = self.layers_cooling_time.len() as i32 - 1;
                }

                // if the layer time big enough, surface defact will disappear
                let data_temp =
                    if self.layers_cooling_time[remap_data_pos as usize]
                        > self.layer_time_smoothing_threshold
                    {
                        self.layer_time_smoothing_threshold
                    } else {
                        self.layers_cooling_time[remap_data_pos as usize]
                    };

                conv_sum += self.guassian_filter[filter_pos_idx as usize] * data_temp as f64;
            }
            let mut filter_res = conv_sum / self.filter_sum;
            filter_res = if filter_res > self.layer_time_smoothing_threshold as f64 {
                self.layer_time_smoothing_threshold as f64
            } else {
                filter_res
            };
            if filter_res > self.layers_cooling_time[layer_id as usize] as f64 {
                self.layers_cooling_time[layer_id as usize] = filter_res as f32;
            }
        }
    }

    // Smoothing.cpp:204-213
    fn layer_time_filter_continue(&self) -> bool {
        for layer_id in 1..self.layers_cooling_time.len() as i32 - 1 {
            // C++: `double layer_time = ... ? threshold : value;` — the (float) ternary
            // result is widened to double, the subtraction/abs run in double, and the
            // (float) stop threshold is promoted to double for the compare.
            let layer_time: f64 = if self.layers_cooling_time[layer_id as usize]
                > self.layer_time_smoothing_threshold
            {
                self.layer_time_smoothing_threshold as f64
            } else {
                self.layers_cooling_time[layer_id as usize] as f64
            };
            let layer_time_cmp: f64 = if self.layers_cooling_time[(layer_id + 1) as usize]
                > self.layer_time_smoothing_threshold
            {
                self.layer_time_smoothing_threshold as f64
            } else {
                self.layers_cooling_time[(layer_id + 1) as usize] as f64
            };

            if (layer_time - layer_time_cmp).abs() > GUASSIAN_LAYER_TIME_STOP_THRESHOLD as f64 {
                return true;
            }
        }
        false
    }

    // Smoothing.cpp:215-222
    pub fn smooth_layer_time(&mut self) {
        let mut step_count = 0;
        while step_count < MAX_STEPS_COUNT && self.layer_time_filter_continue() {
            step_count += 1;
            self.filter_layer_time();
        }
    }

    // Smoothing.hpp:69-71
    fn guassian_function(&self, x: f64, r: f64) -> f64 {
        (-x * x / (2.0 * r * r)).exp() / (r * (2.0 * std::f64::consts::PI).sqrt())
    }

    // Smoothing.hpp:73-81
    fn guassian_filter_generator(&mut self) {
        let r = GUASSIAN_R as f64;
        let half_win_size = GUASSIAN_WINDOW_SIZE / 2;
        let mut start = -half_win_size;
        while start <= half_win_size {
            let y = self.guassian_function(start as f64, r);
            self.filter_sum += y;
            self.guassian_filter.push(y);
            start += 1;
        }
    }
}

// Smoothing.cpp:46-69
//
// Returns the (possibly updated) `node.rate`. In C++ `node` is a mutable
// reference and this function writes `node.rate`; Rust borrow rules require us
// to return the value and let the caller write it back.
fn exclude_participate_in_speed_slowdown(
    lines: &[(i32, i32)],
    per_extruder_adjustments: &mut [PerExtruderAdjustments],
    node: &CoolingNode,
) -> f64 {
    // BBS: add protect, feedrate will be 0 if the outwall is overhang. just apply not adjust flage
    let apply_speed = node.max_feedrate > 0.0 && node.filter_feedrate > 0.0;
    let mut rate = node.rate;
    if apply_speed {
        rate = (node.filter_feedrate / node.max_feedrate) as f64;
    }

    for line_pos in lines {
        let line = &mut per_extruder_adjustments[line_pos.1 as usize].lines[line_pos.0 as usize];
        if apply_speed && line.feedrate > node.filter_feedrate {
            line.feedrate = node.filter_feedrate;
            line.slowdown = true;
        }

        // not adjust outwal line speed
        line.line_type &= !CoolingLineType::ADJUSTABLE;
        // update time cost
        if line.feedrate == 0.0 || line.length == 0.0 {
            line.time = 0.0;
        } else {
            line.time = line.length / line.feedrate;
        }
    }

    rate
}
