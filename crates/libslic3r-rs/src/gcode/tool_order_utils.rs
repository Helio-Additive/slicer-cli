//! Faithful 1:1 port of `src/libslic3r/GCode/ToolOrderUtils.{hpp,cpp}` (BambuStudio).
//!
//! Graph-based solvers for optimizing tool change sequences to minimize
//! flush/purge volume in multi-material / multi-nozzle prints.
//!
//! Fidelity notes:
//! - C++ `Edge::cost` is `int`; SPFA `dist`/`flow` are `int`. We mirror with `i32`.
//! - The `matrix` is `vector<vector<float>>`; `get_distance`/`get_flush_cost`
//!   return `int` (implicit float->int truncation). We mirror with `as i32`.
//! - Graph node addressing is index-based: left nodes are `0..L`, right nodes
//!   `L..L+R`, then group/source/sink nodes; this matches the C++ layout exactly.
//! - `boost::multiprecision::uint128_t` -> Rust native `u128`.
//! - The `#if DEBUG_MULTI_NOZZLE_MCMF` block (0 in the hpp, so not compiled in
//!   C++) is ported in the module `debug_multi_nozzle_mcmf`, which the engine
//!   never calls.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet, VecDeque};

use crate::multi_nozzle_utils::{LayeredNozzleGroupResult, NozzleStatusRecorder};

// ToolOrderUtils.hpp:16  using FlushMatrix = std::vector<std::vector<float>>;
pub type FlushMatrix = Vec<Vec<f32>>;

// ToolOrderUtils.hpp:19  namespace MaxFlowGraph
pub mod max_flow_graph {
    // ToolOrderUtils.hpp:20  const int INF = std::numeric_limits<int>::max();
    pub const INF: i32 = i32::MAX;
    // ToolOrderUtils.hpp:21  const int INVALID_ID = -1;
    pub const INVALID_ID: i32 = -1;
    // ToolOrderUtils.hpp:22  // Upper bound for MCMF edge cost to prevent int overflow in SPFA causing infinite loops
    // ToolOrderUtils.hpp:23  constexpr int MCMF_MAX_EDGE_COST = 10000000;
    pub const MCMF_MAX_EDGE_COST: i32 = 10000000;
}

/// R917: `TOOL_ORDER_UTILS_PIN` (faithful, on unless `=0`) brings this file to
/// ToolOrderUtils.cpp at the pin (56a57d0): the MCMF edge-cost bound in
/// `get_distance` (:193-201), `use_forcast = false` in
/// `reorder_filaments_for_minimum_flush_volume_base` (:1095), `int += float`
/// cost accumulation (the sum is taken in float, then truncated), and the
/// guards of `reorder_filaments_for_multi_nozzle_extruder` (:1546-1749). The
/// off arm is the port of the earlier source, kept byte for byte.
fn tool_order_utils_pin() -> bool {
    crate::faithful_gate("TOOL_ORDER_UTILS_PIN")
}

/// C++ `int cost; cost += float_value;` (usual arithmetic conversions): the int
/// is converted to float, the sum is taken in float and truncated back to int.
#[inline]
fn int_add_assign_float(cost: &mut i32, value: f32) {
    *cost = (*cost as f32 + value) as i32;
}

// Re-export for callers that referenced the crate-level alias historically.
pub const INF: i32 = max_flow_graph::INF;
pub const INVALID_ID: i32 = max_flow_graph::INVALID_ID;

// ToolOrderUtils.hpp:24  struct Edge
#[derive(Debug, Clone)]
pub struct Edge {
    pub from: i32,
    pub to: i32,
    pub capacity: i32,
    pub cost: i32,
    pub flow: i32,
}

impl Edge {
    // ToolOrderUtils.hpp:27  Edge(int u, int v, int cap, int cst = 0) : from(u), to(v), capacity(cap), cost(cst), flow(0) {}
    pub fn new(u: i32, v: i32, cap: i32, cst: i32) -> Self {
        Edge { from: u, to: v, capacity: cap, cost: cst, flow: 0 }
    }
}

// ==================== MaxFlowWithLowerBounds ====================
// ToolOrderUtils.cpp:11  struct MaxFlowWithLowerBounds
#[derive(Debug, Default)]
struct MaxFlowWithLowerBounds {
    l_nodes: Vec<i32>,
    r_nodes: Vec<i32>,
    edges: Vec<Edge>,
    adj: Vec<Vec<i32>>,
    level: Vec<i32>,
    it: Vec<i32>,

    total_nodes: i32,
    source_id: i32,
    sink_id: i32,
}

impl MaxFlowWithLowerBounds {
    fn new() -> Self {
        // ToolOrderUtils.cpp:28-30  in-class initializers { -1 }
        MaxFlowWithLowerBounds {
            l_nodes: Vec::new(),
            r_nodes: Vec::new(),
            edges: Vec::new(),
            adj: Vec::new(),
            level: Vec::new(),
            it: Vec::new(),
            total_nodes: -1,
            source_id: -1,
            sink_id: -1,
        }
    }

    // ToolOrderUtils.cpp:33  void MaxFlowWithLowerBounds::add_edge(int from, int to, int capacity)
    fn add_edge(&mut self, from: i32, to: i32, capacity: i32) {
        // ToolOrderUtils.cpp:35  adj[from].emplace_back(edges.size());
        self.adj[from as usize].push(self.edges.len() as i32);
        // ToolOrderUtils.cpp:36  edges.emplace_back(from, to, capacity, 0);
        self.edges.push(Edge::new(from, to, capacity, 0));
        // ToolOrderUtils.cpp:37  // also add reverse edge ,set capacity to zero,cost to negative
        // ToolOrderUtils.cpp:38  adj[to].emplace_back(edges.size());
        self.adj[to as usize].push(self.edges.len() as i32);
        // ToolOrderUtils.cpp:39  edges.emplace_back(to, from, 0, 0);
        self.edges.push(Edge::new(to, from, 0, 0));
    }

    // ToolOrderUtils.cpp:42  bool MaxFlowWithLowerBounds::bfs()
    fn bfs(&mut self) -> bool {
        // ToolOrderUtils.cpp:43  level.assign(total_nodes, -1);
        self.level = vec![-1; self.total_nodes as usize];
        // ToolOrderUtils.cpp:44  std::queue<int> q;
        let mut q: VecDeque<i32> = VecDeque::new();
        // ToolOrderUtils.cpp:45  q.push(source_id);
        q.push_back(self.source_id);
        // ToolOrderUtils.cpp:46  level[source_id] = 0;
        self.level[self.source_id as usize] = 0;

        // ToolOrderUtils.cpp:48  while (!q.empty())
        while let Some(u) = q.pop_front() {
            // ToolOrderUtils.cpp:50  for (int eid : adj[u])
            for idx in 0..self.adj[u as usize].len() {
                let eid = self.adj[u as usize][idx];
                let e = &self.edges[eid as usize];
                // ToolOrderUtils.cpp:52  if (e.flow < e.capacity && level[e.to] == -1)
                if e.flow < e.capacity && self.level[e.to as usize] == -1 {
                    // ToolOrderUtils.cpp:53  level[e.to] = level[u] + 1;
                    self.level[e.to as usize] = self.level[u as usize] + 1;
                    // ToolOrderUtils.cpp:54  q.push(e.to);
                    q.push_back(e.to);
                }
            }
        }
        // ToolOrderUtils.cpp:58  return level[sink_id] != -1;
        self.level[self.sink_id as usize] != -1
    }

    // ToolOrderUtils.cpp:61  int MaxFlowWithLowerBounds::dfs(int u, int f)
    fn dfs(&mut self, u: i32, f: i32) -> i32 {
        // ToolOrderUtils.cpp:62  if (u == sink_id) return f;
        if u == self.sink_id {
            return f;
        }
        // ToolOrderUtils.cpp:63  for (int &i = it[u]; i < (int)adj[u].size(); ++i)
        while (self.it[u as usize] as usize) < self.adj[u as usize].len() {
            let i = self.it[u as usize];
            // ToolOrderUtils.cpp:64  int eid = adj[u][i];
            let eid = self.adj[u as usize][i as usize];
            let (e_flow, e_capacity, e_to) = {
                let e = &self.edges[eid as usize];
                (e.flow, e.capacity, e.to)
            };
            // ToolOrderUtils.cpp:66  if (e.flow < e.capacity && level[e.to] == level[u] + 1)
            if e_flow < e_capacity && self.level[e_to as usize] == self.level[u as usize] + 1 {
                // ToolOrderUtils.cpp:67  int pushed = dfs(e.to, std::min(f, e.capacity - e.flow));
                let pushed = self.dfs(e_to, f.min(e_capacity - e_flow));
                // ToolOrderUtils.cpp:68  if (pushed > 0)
                if pushed > 0 {
                    // ToolOrderUtils.cpp:69  e.flow += pushed;
                    self.edges[eid as usize].flow += pushed;
                    // ToolOrderUtils.cpp:70  edges[eid ^ 1].flow -= pushed;
                    self.edges[(eid ^ 1) as usize].flow -= pushed;
                    // ToolOrderUtils.cpp:71  return pushed;
                    return pushed;
                }
            }
            self.it[u as usize] += 1;
        }
        // ToolOrderUtils.cpp:75  return 0;
        0
    }

    // ToolOrderUtils.cpp:78  int MaxFlowWithLowerBounds::solve(std::vector<int>& matching)
    fn solve(&mut self, matching: &mut Vec<i32>) -> i32 {
        // ToolOrderUtils.cpp:79  int flow = 0;
        let mut flow = 0;
        // ToolOrderUtils.cpp:80  while (bfs())
        while self.bfs() {
            // ToolOrderUtils.cpp:81  it.assign(total_nodes, 0);
            self.it = vec![0; self.total_nodes as usize];
            // ToolOrderUtils.cpp:82  while (int pushed = dfs(source_id, MaxFlowGraph::INF))
            loop {
                let pushed = self.dfs(self.source_id, max_flow_graph::INF);
                if pushed == 0 {
                    break;
                }
                // ToolOrderUtils.cpp:83  flow += pushed;
                flow += pushed;
            }
        }

        // ToolOrderUtils.cpp:86  int L = l_nodes.size();
        let l = self.l_nodes.len() as i32;
        // ToolOrderUtils.cpp:87  int R = r_nodes.size();
        let r = self.r_nodes.len() as i32;
        // ToolOrderUtils.cpp:89  matching.resize(l_nodes.size(), MaxFlowGraph::INVALID_ID);
        matching.resize(self.l_nodes.len(), max_flow_graph::INVALID_ID);
        // ToolOrderUtils.cpp:90  for (int u = 0; u < L; ++u)
        for u in 0..l {
            // ToolOrderUtils.cpp:91  for (int eid : adj[u])
            for &eid in &self.adj[u as usize] {
                let e = &self.edges[eid as usize];
                // ToolOrderUtils.cpp:93  if (e.flow > 0 && e.to >= L && e.to < L + R)
                if e.flow > 0 && e.to >= l && e.to < l + r {
                    // ToolOrderUtils.cpp:94  matching[e.from] = e.to - L;
                    matching[e.from as usize] = e.to - l;
                }
            }
        }
        // ToolOrderUtils.cpp:98  return flow;
        flow
    }
}

// ==================== MinCostMaxFlow ====================
// ToolOrderUtils.cpp:102  struct MinCostMaxFlow
#[derive(Debug, Clone, Default)]
pub struct MinCostMaxFlow {
    pub matrix: Vec<Vec<f32>>,
    pub l_nodes: Vec<i32>,
    pub r_nodes: Vec<i32>,
    pub edges: Vec<Edge>,
    pub adj: Vec<Vec<i32>>,

    pub total_nodes: i32,
    pub source_id: i32,
    pub sink_id: i32,
}

impl MinCostMaxFlow {
    fn new() -> Self {
        // ToolOrderUtils.cpp:115-117  in-class initializers { -1 }
        MinCostMaxFlow {
            matrix: Vec::new(),
            l_nodes: Vec::new(),
            r_nodes: Vec::new(),
            edges: Vec::new(),
            adj: Vec::new(),
            total_nodes: -1,
            source_id: -1,
            sink_id: -1,
        }
    }

    // ToolOrderUtils.cpp:120  std::vector<int> MinCostMaxFlow::solve()
    pub fn solve(&mut self) -> Vec<i32> {
        // ToolOrderUtils.cpp:122  while (spfa(source_id, sink_id));
        while self.spfa(self.source_id, self.sink_id) {}

        // ToolOrderUtils.cpp:124  std::vector<int>matching(l_nodes.size(), MaxFlowGraph::INVALID_ID);
        let mut matching = vec![max_flow_graph::INVALID_ID; self.l_nodes.len()];
        // ToolOrderUtils.cpp:125-126  // to get the match info, just traverse the left nodes and check edges
        // ToolOrderUtils.cpp:127  for (int u = 0; u < l_nodes.size(); ++u)
        for u in 0..self.l_nodes.len() {
            // ToolOrderUtils.cpp:128  for (int eid : adj[u])
            for &eid in &self.adj[u] {
                let e = &self.edges[eid as usize];
                // ToolOrderUtils.cpp:130  if (e.flow > 0 && e.to >= l_nodes.size() && e.to < l_nodes.size() + r_nodes.size())
                if e.flow > 0
                    && e.to >= self.l_nodes.len() as i32
                    && e.to < (self.l_nodes.len() + self.r_nodes.len()) as i32
                {
                    // ToolOrderUtils.cpp:131  matching[e.from] = e.to - l_nodes.size();
                    matching[e.from as usize] = e.to - self.l_nodes.len() as i32;
                }
            }
        }

        // ToolOrderUtils.cpp:135  return matching;
        matching
    }

    // ToolOrderUtils.cpp:138  void MinCostMaxFlow::add_edge(int from, int to, int capacity, int cost)
    pub fn add_edge(&mut self, from: i32, to: i32, capacity: i32, cost: i32) {
        // ToolOrderUtils.cpp:140  adj[from].emplace_back(edges.size());
        self.adj[from as usize].push(self.edges.len() as i32);
        // ToolOrderUtils.cpp:141  edges.emplace_back(from, to, capacity, cost);
        self.edges.push(Edge::new(from, to, capacity, cost));
        // ToolOrderUtils.cpp:142  //also add reverse edge ,set capacity to zero,cost to negative
        // ToolOrderUtils.cpp:143  adj[to].emplace_back(edges.size());
        self.adj[to as usize].push(self.edges.len() as i32);
        // ToolOrderUtils.cpp:144  edges.emplace_back(to, from, 0, -cost);
        self.edges.push(Edge::new(to, from, 0, -cost));
    }

    // ToolOrderUtils.cpp:147  bool MinCostMaxFlow::spfa(int source, int sink)
    pub fn spfa(&mut self, source: i32, sink: i32) -> bool {
        // ToolOrderUtils.cpp:149  std::vector<int>dist(total_nodes, MaxFlowGraph::INF);
        let mut dist = vec![max_flow_graph::INF; self.total_nodes as usize];
        // ToolOrderUtils.cpp:150  std::vector<bool>in_queue(total_nodes, false);
        let mut in_queue = vec![false; self.total_nodes as usize];
        // ToolOrderUtils.cpp:151  std::vector<int>flow(total_nodes, MaxFlowGraph::INF);
        let mut flow = vec![max_flow_graph::INF; self.total_nodes as usize];
        // ToolOrderUtils.cpp:152  std::vector<int>prev(total_nodes, 0);
        let mut prev = vec![0i32; self.total_nodes as usize];

        // ToolOrderUtils.cpp:154  std::queue<int>q;
        let mut q: VecDeque<i32> = VecDeque::new();
        // ToolOrderUtils.cpp:155  q.push(source);
        q.push_back(source);
        // ToolOrderUtils.cpp:156  in_queue[source] = true;
        in_queue[source as usize] = true;
        // ToolOrderUtils.cpp:157  dist[source] = 0;
        dist[source as usize] = 0;

        // ToolOrderUtils.cpp:159  while (!q.empty())
        while let Some(now_at) = q.pop_front() {
            // ToolOrderUtils.cpp:162  in_queue[now_at] = false;
            in_queue[now_at as usize] = false;

            // ToolOrderUtils.cpp:164  for (auto eid : adj[now_at]) //traverse all linked edges
            for idx in 0..self.adj[now_at as usize].len() {
                let eid = self.adj[now_at as usize][idx];
                let (e_flow, e_capacity, e_cost, e_to) = {
                    let e = &self.edges[eid as usize];
                    (e.flow, e.capacity, e.cost, e.to)
                };
                // ToolOrderUtils.cpp:167  if (e.flow<e.capacity && dist[e.to]>dist[now_at] + e.cost)
                if e_flow < e_capacity && dist[e_to as usize] > dist[now_at as usize] + e_cost {
                    // ToolOrderUtils.cpp:168  dist[e.to] = dist[now_at] + e.cost;
                    dist[e_to as usize] = dist[now_at as usize] + e_cost;
                    // ToolOrderUtils.cpp:169  prev[e.to] = eid;
                    prev[e_to as usize] = eid;
                    // ToolOrderUtils.cpp:170  flow[e.to] = std::min(flow[now_at], e.capacity - e.flow);
                    flow[e_to as usize] = flow[now_at as usize].min(e_capacity - e_flow);
                    // ToolOrderUtils.cpp:171  if (!in_queue[e.to])
                    if !in_queue[e_to as usize] {
                        // ToolOrderUtils.cpp:172  q.push(e.to);
                        q.push_back(e_to);
                        // ToolOrderUtils.cpp:173  in_queue[e.to] = true;
                        in_queue[e_to as usize] = true;
                    }
                }
            }
        }

        // ToolOrderUtils.cpp:179  if (dist[sink] == MaxFlowGraph::INF)
        if dist[sink as usize] == max_flow_graph::INF {
            // ToolOrderUtils.cpp:180  return false;
            return false;
        }

        // ToolOrderUtils.cpp:182  int now_at = sink;
        let mut now_at = sink;
        // ToolOrderUtils.cpp:183  while (now_at != source)
        while now_at != source {
            // ToolOrderUtils.cpp:184  int prev_edge = prev[now_at];
            let prev_edge = prev[now_at as usize];
            // ToolOrderUtils.cpp:185  edges[prev_edge].flow += flow[sink];
            self.edges[prev_edge as usize].flow += flow[sink as usize];
            // ToolOrderUtils.cpp:186  edges[prev_edge ^ 1].flow -= flow[sink];
            self.edges[(prev_edge ^ 1) as usize].flow -= flow[sink as usize];
            // ToolOrderUtils.cpp:187  now_at = edges[prev_edge].from;
            now_at = self.edges[prev_edge as usize].from;
        }

        // ToolOrderUtils.cpp:190  return true;
        true
    }

    // ToolOrderUtils.cpp:193  int MinCostMaxFlow::get_distance(int idx_in_left, int idx_in_right)
    pub fn get_distance(&self, idx_in_left: i32, idx_in_right: i32) -> i32 {
        // ToolOrderUtils.cpp:195  if (l_nodes[idx_in_left] == -1)
        if self.l_nodes[idx_in_left as usize] == -1 {
            // ToolOrderUtils.cpp:196  return 0;
            return 0;
        }

        if tool_order_utils_pin() {
            // ToolOrderUtils.cpp:199  float val = matrix[l_nodes[idx_in_left]][r_nodes[idx_in_right]];
            let val: f32 = self.matrix[self.l_nodes[idx_in_left as usize] as usize]
                [self.r_nodes[idx_in_right as usize] as usize];
            // ToolOrderUtils.cpp:200  return std::min(static_cast<int>(val), MaxFlowGraph::MCMF_MAX_EDGE_COST);
            return std::cmp::min(val as i32, max_flow_graph::MCMF_MAX_EDGE_COST);
        }
        // Earlier source: return matrix[l_nodes[idx_in_left]][r_nodes[idx_in_right]];
        self.matrix[self.l_nodes[idx_in_left as usize] as usize]
            [self.r_nodes[idx_in_right as usize] as usize] as i32
    }
}

// ==================== MaxFlowSolver ====================
// ToolOrderUtils.hpp:30  class MaxFlowSolver
#[derive(Debug, Clone)]
pub struct MaxFlowSolver {
    total_nodes: i32,
    source_id: i32,
    sink_id: i32,
    edges: Vec<Edge>,
    l_nodes: Vec<i32>,
    r_nodes: Vec<i32>,
    adj: Vec<Vec<i32>>,
}

impl MaxFlowSolver {
    // ToolOrderUtils.cpp:209  MaxFlowSolver::MaxFlowSolver(...)
    pub fn new(
        u_nodes: &[i32],
        v_nodes: &[i32],
        uv_link_limits: &HashMap<i32, Vec<i32>>,
        uv_unlink_limits: &HashMap<i32, Vec<i32>>,
        u_capacity: &[i32],
        v_capacity: &[i32],
        v_group_capacity: &[(BTreeSet<i32>, i32)],
    ) -> Self {
        // ToolOrderUtils.cpp:216  assert(u_capacity.empty() || u_capacity.size() == u_nodes.size());
        debug_assert!(u_capacity.is_empty() || u_capacity.len() == u_nodes.len());
        // ToolOrderUtils.cpp:217  assert(v_capacity.empty() || v_capacity.size() == v_nodes.size());
        debug_assert!(v_capacity.is_empty() || v_capacity.len() == v_nodes.len());

        // ToolOrderUtils.cpp:218  l_nodes = u_nodes;
        // ToolOrderUtils.cpp:219  r_nodes = v_nodes;
        // ToolOrderUtils.cpp:220  total_nodes = u_nodes.size() + v_nodes.size() + v_group_capacity.size() + 2;
        let total_nodes =
            (u_nodes.len() + v_nodes.len() + v_group_capacity.len() + 2) as i32;
        // ToolOrderUtils.cpp:221  source_id = total_nodes - 2;
        let source_id = total_nodes - 2;
        // ToolOrderUtils.cpp:222  sink_id = total_nodes - 1;
        let sink_id = total_nodes - 1;

        let mut s = MaxFlowSolver {
            total_nodes,
            source_id,
            sink_id,
            edges: Vec::new(),
            l_nodes: u_nodes.to_vec(),
            r_nodes: v_nodes.to_vec(),
            // ToolOrderUtils.cpp:224  adj.resize(total_nodes);
            adj: vec![Vec::new(); total_nodes as usize],
        };

        // ToolOrderUtils.cpp:226  std::vector<int>v_node_to(v_nodes.size(), sink_id);
        let mut v_node_to = vec![sink_id; v_nodes.len()];
        // ToolOrderUtils.cpp:227  for (size_t gid = 0; gid < v_group_capacity.size(); ++gid)
        for (gid, gc) in v_group_capacity.iter().enumerate() {
            // ToolOrderUtils.cpp:228  for (auto vid : v_group_capacity[gid].first)
            for &vid in &gc.0 {
                // ToolOrderUtils.cpp:229  v_node_to[vid] = l_nodes.size() + r_nodes.size() + gid;
                v_node_to[vid as usize] =
                    (s.l_nodes.len() + s.r_nodes.len() + gid) as i32;
            }
        }

        // ToolOrderUtils.cpp:232  // add edge from source to left nodes
        // ToolOrderUtils.cpp:233  for (int idx = 0; idx < l_nodes.size(); ++idx)
        for idx in 0..s.l_nodes.len() as i32 {
            // ToolOrderUtils.cpp:234  int capacity = u_capacity.empty() ? 1 : u_capacity[idx];
            let capacity = if u_capacity.is_empty() { 1 } else { u_capacity[idx as usize] };
            // ToolOrderUtils.cpp:235  add_edge(source_id, idx, capacity);
            s.add_edge(source_id, idx, capacity);
        }
        // ToolOrderUtils.cpp:237  // add edge from right nodes to v_node_to(sink node or temp group node)
        // ToolOrderUtils.cpp:238  for (int idx = 0; idx < r_nodes.size(); ++idx)
        for idx in 0..s.r_nodes.len() as i32 {
            // ToolOrderUtils.cpp:239  int capacity = v_capacity.empty() ? 1 : v_capacity[idx];
            let capacity = if v_capacity.is_empty() { 1 } else { v_capacity[idx as usize] };
            // ToolOrderUtils.cpp:240  add_edge(l_nodes.size() + idx, v_node_to[idx], capacity);
            s.add_edge(s.l_nodes.len() as i32 + idx, v_node_to[idx as usize], capacity);
        }

        // ToolOrderUtils.cpp:243  // add edge from temp group node to sink node
        // ToolOrderUtils.cpp:244  for (int idx = 0; idx < v_group_capacity.size(); ++idx)
        for idx in 0..v_group_capacity.len() {
            // ToolOrderUtils.cpp:245  int capacity = v_group_capacity[idx].second;
            let capacity = v_group_capacity[idx].1;
            // ToolOrderUtils.cpp:246  add_edge(l_nodes.size() + r_nodes.size() + idx, sink_id, capacity);
            s.add_edge(
                (s.l_nodes.len() + s.r_nodes.len() + idx) as i32,
                sink_id,
                capacity,
            );
        }

        // ToolOrderUtils.cpp:249  // add edge from left nodes to right nodes
        // ToolOrderUtils.cpp:250  for (int i = 0; i < l_nodes.size(); ++i)
        for i in 0..s.l_nodes.len() as i32 {
            // ToolOrderUtils.cpp:251  int from_idx = i;
            let from_idx = i;
            // ToolOrderUtils.cpp:252  // process link limits , i can only link to uv_link_limits
            // ToolOrderUtils.cpp:253  if (auto iter = uv_link_limits.find(i); iter != uv_link_limits.end())
            if let Some(links) = uv_link_limits.get(&i) {
                // ToolOrderUtils.cpp:254  for (auto r_id : iter->second)
                for &r_id in links {
                    // ToolOrderUtils.cpp:255  add_edge(from_idx, l_nodes.size() + r_id, 1);
                    s.add_edge(from_idx, s.l_nodes.len() as i32 + r_id, 1);
                }
                // ToolOrderUtils.cpp:256  continue;
                continue;
            }
            // ToolOrderUtils.cpp:258  // process unlink limits
            // ToolOrderUtils.cpp:259  std::optional<std::vector<int>> unlink_limits;
            // ToolOrderUtils.cpp:260  if (auto iter = uv_unlink_limits.find(i); iter != uv_unlink_limits.end())
            let unlink_limits = uv_unlink_limits.get(&i);

            // ToolOrderUtils.cpp:263  for (int j = 0; j < r_nodes.size(); ++j)
            for j in 0..s.r_nodes.len() as i32 {
                // ToolOrderUtils.cpp:264  // check whether i can link to j
                // ToolOrderUtils.cpp:265  if (unlink_limits.has_value() && std::find(...) != ...end())
                if let Some(ul) = unlink_limits {
                    if ul.contains(&j) {
                        // ToolOrderUtils.cpp:266  continue;
                        continue;
                    }
                }
                // ToolOrderUtils.cpp:267  add_edge(from_idx, l_nodes.size() + j, 1);
                s.add_edge(from_idx, s.l_nodes.len() as i32 + j, 1);
            }
        }

        s
    }

    // ToolOrderUtils.cpp:272  void MaxFlowSolver::add_edge(int from, int to, int capacity)
    fn add_edge(&mut self, from: i32, to: i32, capacity: i32) {
        // ToolOrderUtils.cpp:274  adj[from].emplace_back(edges.size());
        self.adj[from as usize].push(self.edges.len() as i32);
        // ToolOrderUtils.cpp:275  edges.emplace_back(from, to, capacity);
        self.edges.push(Edge::new(from, to, capacity, 0));
        // ToolOrderUtils.cpp:276  //also add reverse edge ,set capacity to zero
        // ToolOrderUtils.cpp:277  adj[to].emplace_back(edges.size());
        self.adj[to as usize].push(self.edges.len() as i32);
        // ToolOrderUtils.cpp:278  edges.emplace_back(to, from, 0);
        self.edges.push(Edge::new(to, from, 0, 0));
    }

    // ToolOrderUtils.cpp:281  std::vector<int> MaxFlowSolver::solve()
    pub fn solve(&mut self) -> Vec<i32> {
        // ToolOrderUtils.cpp:282  std::vector<int> augment;
        let mut augment: Vec<i32> = Vec::new();
        // ToolOrderUtils.cpp:283  std::vector<int> previous(total_nodes, 0);
        let mut previous = vec![0i32; self.total_nodes as usize];
        // ToolOrderUtils.cpp:284  while (1)
        loop {
            // ToolOrderUtils.cpp:285  std::vector<int>(total_nodes, 0).swap(augment);
            augment = vec![0; self.total_nodes as usize];
            // ToolOrderUtils.cpp:286  std::queue<int> travel;
            let mut travel: VecDeque<i32> = VecDeque::new();
            // ToolOrderUtils.cpp:287  travel.push(source_id);
            travel.push_back(self.source_id);
            // ToolOrderUtils.cpp:288  augment[source_id] = MaxFlowGraph::INF;
            augment[self.source_id as usize] = max_flow_graph::INF;
            // ToolOrderUtils.cpp:289  while (!travel.empty())
            while let Some(from) = travel.pop_front() {
                // ToolOrderUtils.cpp:293  // traverse all linked edges
                // ToolOrderUtils.cpp:294  for (int i = 0; i < adj[from].size(); ++i)
                for i in 0..self.adj[from as usize].len() {
                    // ToolOrderUtils.cpp:295  int eid = adj[from][i];
                    let eid = self.adj[from as usize][i];
                    let (tmp_to, tmp_capacity, tmp_flow) = {
                        let tmp = &self.edges[eid as usize];
                        (tmp.to, tmp.capacity, tmp.flow)
                    };
                    // ToolOrderUtils.cpp:297  if (augment[tmp.to] == 0 && tmp.capacity > tmp.flow)
                    if augment[tmp_to as usize] == 0 && tmp_capacity > tmp_flow {
                        // ToolOrderUtils.cpp:298  previous[tmp.to] = eid;
                        previous[tmp_to as usize] = eid;
                        // ToolOrderUtils.cpp:299  augment[tmp.to] = std::min(augment[from], tmp.capacity - tmp.flow);
                        augment[tmp_to as usize] =
                            augment[from as usize].min(tmp_capacity - tmp_flow);
                        // ToolOrderUtils.cpp:300  travel.push(tmp.to);
                        travel.push_back(tmp_to);
                    }
                }

                // ToolOrderUtils.cpp:304  // already find an extend path, stop and do update
                // ToolOrderUtils.cpp:305  if (augment[sink_id] != 0)
                if augment[self.sink_id as usize] != 0 {
                    // ToolOrderUtils.cpp:306  break;
                    break;
                }
            }
            // ToolOrderUtils.cpp:308  // no longer have extend path
            // ToolOrderUtils.cpp:309  if (augment[sink_id] == 0)
            if augment[self.sink_id as usize] == 0 {
                // ToolOrderUtils.cpp:310  break;
                break;
            }

            // ToolOrderUtils.cpp:312  for (int i = sink_id; i != source_id; i = edges[previous[i]].from)
            let mut i = self.sink_id;
            while i != self.source_id {
                // ToolOrderUtils.cpp:313  edges[previous[i]].flow += augment[sink_id];
                self.edges[previous[i as usize] as usize].flow +=
                    augment[self.sink_id as usize];
                // ToolOrderUtils.cpp:314  edges[previous[i] ^ 1].flow -= augment[sink_id];
                self.edges[(previous[i as usize] ^ 1) as usize].flow -=
                    augment[self.sink_id as usize];
                i = self.edges[previous[i as usize] as usize].from;
            }
        }

        // ToolOrderUtils.cpp:318  std::vector<int> matching(l_nodes.size(), MaxFlowGraph::INVALID_ID);
        let mut matching = vec![max_flow_graph::INVALID_ID; self.l_nodes.len()];
        // ToolOrderUtils.cpp:319-320  // to get the match info, just traverse left nodes and check edges
        // ToolOrderUtils.cpp:321  for (int u = 0; u < l_nodes.size(); ++u)
        for u in 0..self.l_nodes.len() {
            // ToolOrderUtils.cpp:322  for (int eid : adj[u])
            for &eid in &self.adj[u] {
                let e = &self.edges[eid as usize];
                // ToolOrderUtils.cpp:324  if (e.flow > 0 && e.to >= l_nodes.size() && e.to < l_nodes.size() + r_nodes.size())
                if e.flow > 0
                    && e.to >= self.l_nodes.len() as i32
                    && e.to < (self.l_nodes.len() + self.r_nodes.len()) as i32
                {
                    // ToolOrderUtils.cpp:325  matching[e.from] = r_nodes[e.to - l_nodes.size()];
                    matching[e.from as usize] =
                        self.r_nodes[(e.to - self.l_nodes.len() as i32) as usize];
                }
            }
        }
        // ToolOrderUtils.cpp:328  return matching;
        matching
    }
}

// ==================== GeneralMinCostSolver ====================
// ToolOrderUtils.hpp:58  class GeneralMinCostSolver
pub struct GeneralMinCostSolver {
    m_solver: MinCostMaxFlow,
}

impl GeneralMinCostSolver {
    // ToolOrderUtils.cpp:336  GeneralMinCostSolver::GeneralMinCostSolver(...)
    pub fn new(matrix_: &[Vec<f32>], u_nodes: &[i32], v_nodes: &[i32]) -> Self {
        // ToolOrderUtils.cpp:338  m_solver = std::make_unique<MinCostMaxFlow>();
        let mut m_solver = MinCostMaxFlow::new();
        // ToolOrderUtils.cpp:339  m_solver->matrix = matrix_;
        m_solver.matrix = matrix_.to_vec();
        // ToolOrderUtils.cpp:340  m_solver->l_nodes = u_nodes;
        m_solver.l_nodes = u_nodes.to_vec();
        // ToolOrderUtils.cpp:341  m_solver->r_nodes = v_nodes;
        m_solver.r_nodes = v_nodes.to_vec();

        // ToolOrderUtils.cpp:343  m_solver->total_nodes = u_nodes.size() + v_nodes.size() + 2;
        m_solver.total_nodes = (u_nodes.len() + v_nodes.len() + 2) as i32;

        // ToolOrderUtils.cpp:345  m_solver->source_id =m_solver->total_nodes - 2;
        m_solver.source_id = m_solver.total_nodes - 2;
        // ToolOrderUtils.cpp:346  m_solver->sink_id = m_solver->total_nodes - 1;
        m_solver.sink_id = m_solver.total_nodes - 1;

        // ToolOrderUtils.cpp:348  m_solver->adj.resize(m_solver->total_nodes);
        m_solver.adj = vec![Vec::new(); m_solver.total_nodes as usize];

        // ToolOrderUtils.cpp:351  // add edge from source to left nodes,cost to 0
        // ToolOrderUtils.cpp:352  for (int i = 0; i < m_solver->l_nodes.size(); ++i)
        for i in 0..m_solver.l_nodes.len() as i32 {
            // ToolOrderUtils.cpp:353  m_solver->add_edge(m_solver->source_id, i, 1, 0);
            let source_id = m_solver.source_id;
            m_solver.add_edge(source_id, i, 1, 0);
        }

        // ToolOrderUtils.cpp:355  // add edge from right nodes to sink,cost to 0
        // ToolOrderUtils.cpp:356  for (int i = 0; i < m_solver->r_nodes.size(); ++i)
        for i in 0..m_solver.r_nodes.len() as i32 {
            // ToolOrderUtils.cpp:357  m_solver->add_edge(m_solver->l_nodes.size() + i, m_solver->sink_id, 1, 0);
            let l = m_solver.l_nodes.len() as i32;
            let sink_id = m_solver.sink_id;
            m_solver.add_edge(l + i, sink_id, 1, 0);
        }

        // ToolOrderUtils.cpp:359  // add edge from left node to right nodes
        // ToolOrderUtils.cpp:360  for (int i = 0; i < m_solver->l_nodes.size(); ++i)
        for i in 0..m_solver.l_nodes.len() as i32 {
            // ToolOrderUtils.cpp:361  int from_idx = i;
            let from_idx = i;
            // ToolOrderUtils.cpp:362  for (int j = 0; j < m_solver->r_nodes.size(); ++j)
            for j in 0..m_solver.r_nodes.len() as i32 {
                // ToolOrderUtils.cpp:363  int to_idx = m_solver->l_nodes.size() + j;
                let to_idx = m_solver.l_nodes.len() as i32 + j;
                // ToolOrderUtils.cpp:364  m_solver->add_edge(from_idx, to_idx, 1, m_solver->get_distance(i, j));
                let dist = m_solver.get_distance(i, j);
                m_solver.add_edge(from_idx, to_idx, 1, dist);
            }
        }

        GeneralMinCostSolver { m_solver }
    }

    // ToolOrderUtils.cpp:369  std::vector<int> GeneralMinCostSolver::solve()
    pub fn solve(&mut self) -> Vec<i32> {
        // ToolOrderUtils.cpp:370  return m_solver->solve();
        self.m_solver.solve()
    }
}

// ==================== GeneralMinCostLowerBoundsSolver ====================
// ToolOrderUtils.hpp:71  class GeneralMinCostLowerBoundsSolver
pub struct GeneralMinCostLowerBoundsSolver {
    m_solver_lower_bounds: MaxFlowWithLowerBounds,
    m_solver_min_cost: MinCostMaxFlow,

    flush_matrix: Vec<FlushMatrix>,
    l_nodes: Vec<i32>,
    r_nodes: Vec<i32>,
    r_nodes_group: Vec<i32>,
    m_uv_link_limits: HashMap<i32, Vec<i32>>,
    m_uv_unlink_limits: HashMap<i32, Vec<i32>>,
    num_groups: i32,

    // ToolOrderUtils.hpp:112  std::vector<int> demand;
    demand: Vec<i32>,
    // ToolOrderUtils.hpp:113  std::vector<LowerBoundEdge> lower_bound_edges;
    lower_bound_edges: Vec<LowerBoundEdge>,

    super_source: i32,
    super_sink: i32,
    source_id: i32,
    sink_id: i32,
    max_flow_edges: i32,
}

// ToolOrderUtils.hpp:107  struct LowerBoundEdge
#[derive(Debug, Clone)]
struct LowerBoundEdge {
    edge_id: i32,
    lower: i32,
}

impl GeneralMinCostLowerBoundsSolver {
    // ToolOrderUtils.cpp:376  GeneralMinCostLowerBoundsSolver::GeneralMinCostLowerBoundsSolver(...)
    pub fn new(
        matrix_: &[FlushMatrix],
        u_nodes: &[i32],
        v_nodes: &[i32],
        v_nodes_group: &[i32],
        uv_link_limits: &HashMap<i32, Vec<i32>>,
        uv_unlink_limits: &HashMap<i32, Vec<i32>>,
    ) -> Self {
        // ToolOrderUtils.cpp:383  flush_matrix       = matrix_;
        // ToolOrderUtils.cpp:384  l_nodes            = u_nodes;
        // ToolOrderUtils.cpp:385  r_nodes            = v_nodes;
        // ToolOrderUtils.cpp:386  r_nodes_group      = v_nodes_group;
        // ToolOrderUtils.cpp:387  m_uv_link_limits   = uv_link_limits;
        // ToolOrderUtils.cpp:388  m_uv_unlink_limits = uv_unlink_limits;
        // ToolOrderUtils.cpp:389  num_groups = *std::max_element(r_nodes_group.begin(), r_nodes_group.end()) + 1;
        let num_groups = *v_nodes_group.iter().max().unwrap() + 1;

        // ToolOrderUtils.cpp:391  m_solver_lower_bounds = std::make_unique<MaxFlowWithLowerBounds>();
        // ToolOrderUtils.cpp:392  m_solver_min_cost = std::make_unique<MinCostMaxFlow>();
        GeneralMinCostLowerBoundsSolver {
            m_solver_lower_bounds: MaxFlowWithLowerBounds::new(),
            m_solver_min_cost: MinCostMaxFlow::new(),
            flush_matrix: matrix_.to_vec(),
            l_nodes: u_nodes.to_vec(),
            r_nodes: v_nodes.to_vec(),
            r_nodes_group: v_nodes_group.to_vec(),
            m_uv_link_limits: uv_link_limits.clone(),
            m_uv_unlink_limits: uv_unlink_limits.clone(),
            num_groups,
            demand: Vec::new(),
            lower_bound_edges: Vec::new(),
            super_source: -1,
            super_sink: -1,
            source_id: -1,
            sink_id: -1,
            max_flow_edges: 0,
        }
    }

    // ToolOrderUtils.cpp:395  std::vector<int> GeneralMinCostLowerBoundsSolver::solve()
    pub fn solve(&mut self) -> Vec<i32> {
        // ToolOrderUtils.cpp:397  // 不需要下界约束的group节点
        // ToolOrderUtils.cpp:398  std::unordered_set<int> no_lower_group;
        let mut no_lower_group: HashSet<i32> = HashSet::new();
        // ToolOrderUtils.cpp:399  for (int i = 0; i < r_nodes.size(); i++)
        for i in 0..self.r_nodes.len() {
            // ToolOrderUtils.cpp:400  if (r_nodes[i] >= 0)
            if self.r_nodes[i] >= 0 {
                // ToolOrderUtils.cpp:401  no_lower_group.insert(r_nodes_group[i]);
                no_lower_group.insert(self.r_nodes_group[i]);
            }
        }

        // ToolOrderUtils.cpp:404  // 1. 构建下界网络图
        // ToolOrderUtils.cpp:405  build_feasible_graph(no_lower_group);
        self.build_feasible_graph(&no_lower_group);

        // ToolOrderUtils.cpp:407  // 2. 计算最大流
        // ToolOrderUtils.cpp:408  int need = 0;
        let mut need = 0;
        // ToolOrderUtils.cpp:409  for (int d : demand)
        for &d in &self.demand {
            // ToolOrderUtils.cpp:410  if (d > 0) need += d;
            if d > 0 {
                need += d;
            }
        }
        // ToolOrderUtils.cpp:411  std::vector<int> feasible_matching;
        let mut feasible_matching: Vec<i32> = Vec::new();
        // ToolOrderUtils.cpp:412  int pushed_flow = m_solver_lower_bounds->solve(feasible_matching);
        let pushed_flow = self.m_solver_lower_bounds.solve(&mut feasible_matching);
        // ToolOrderUtils.cpp:413  assert(need == pushed_flow);
        debug_assert_eq!(need, pushed_flow);

        // ToolOrderUtils.cpp:415  // 3. 下界最大流网络转化为最小费用最大流网络
        // ToolOrderUtils.cpp:416  build_graph_with_feasible_result();
        self.build_graph_with_feasible_result();
        // ToolOrderUtils.cpp:417  // 4. 计算最小费用最大流
        // ToolOrderUtils.cpp:418  auto min_cost_matching = m_solver_min_cost->solve();
        let min_cost_matching = self.m_solver_min_cost.solve();

        // ToolOrderUtils.cpp:420  return min_cost_matching;
        min_cost_matching
    }

    // ToolOrderUtils.cpp:423  void GeneralMinCostLowerBoundsSolver::build_feasible_graph(const std::unordered_set<int> &no_lower_groups)
    fn build_feasible_graph(&mut self, no_lower_groups: &HashSet<i32>) {
        // ToolOrderUtils.cpp:425  m_solver_lower_bounds->l_nodes = l_nodes;
        self.m_solver_lower_bounds.l_nodes = self.l_nodes.clone();
        // ToolOrderUtils.cpp:426  m_solver_lower_bounds->r_nodes = r_nodes;
        self.m_solver_lower_bounds.r_nodes = self.r_nodes.clone();
        // ToolOrderUtils.cpp:427  m_solver_lower_bounds->total_nodes = l_nodes.size() + r_nodes.size() + num_groups + 2;
        self.m_solver_lower_bounds.total_nodes =
            (self.l_nodes.len() + self.r_nodes.len()) as i32 + self.num_groups + 2;

        // ToolOrderUtils.cpp:429  m_solver_lower_bounds->source_id = m_solver_lower_bounds->total_nodes - 2;
        self.m_solver_lower_bounds.source_id = self.m_solver_lower_bounds.total_nodes - 2;
        // ToolOrderUtils.cpp:430  m_solver_lower_bounds->sink_id   = m_solver_lower_bounds->total_nodes - 1;
        self.m_solver_lower_bounds.sink_id = self.m_solver_lower_bounds.total_nodes - 1;
        // ToolOrderUtils.cpp:431  m_solver_lower_bounds->adj.resize(m_solver_lower_bounds->total_nodes);
        self.m_solver_lower_bounds.adj =
            vec![Vec::new(); self.m_solver_lower_bounds.total_nodes as usize];
        // ToolOrderUtils.cpp:432  demand.resize(m_solver_lower_bounds->total_nodes, 0);
        self.demand = vec![0; self.m_solver_lower_bounds.total_nodes as usize];

        // ToolOrderUtils.cpp:434  const int L = m_solver_lower_bounds->l_nodes.size();
        let l = self.m_solver_lower_bounds.l_nodes.len() as i32;
        // ToolOrderUtils.cpp:435  const int R = m_solver_lower_bounds->r_nodes.size();
        let r = self.m_solver_lower_bounds.r_nodes.len() as i32;

        // ToolOrderUtils.cpp:437  // source -> l
        // ToolOrderUtils.cpp:438  for (int i = 0; i < L; ++i)
        for i in 0..l {
            // ToolOrderUtils.cpp:439  m_solver_lower_bounds->add_edge(m_solver_lower_bounds->source_id, i, 1);
            let source_id = self.m_solver_lower_bounds.source_id;
            self.m_solver_lower_bounds.add_edge(source_id, i, 1);
        }

        // ToolOrderUtils.cpp:441  // u -> v (with link/unlink limits)
        // ToolOrderUtils.cpp:442  for (int i = 0; i < L; ++i)
        for i in 0..l {
            // ToolOrderUtils.cpp:443  if (auto it = m_uv_link_limits.find(i); it != m_uv_link_limits.end())
            if let Some(links) = self.m_uv_link_limits.get(&i).cloned() {
                // ToolOrderUtils.cpp:444  for (int j : it->second)
                for j in links {
                    // ToolOrderUtils.cpp:445  m_solver_lower_bounds->add_edge(i, L + j, 1);
                    self.m_solver_lower_bounds.add_edge(i, l + j, 1);
                }
                // ToolOrderUtils.cpp:446  continue;
                continue;
            }

            // ToolOrderUtils.cpp:449  std::optional<std::vector<int>> unlink_limits;
            // ToolOrderUtils.cpp:450  if (auto it = m_uv_unlink_limits.find(i); it != m_uv_unlink_limits.end())
            let unlink_limits = self.m_uv_unlink_limits.get(&i).cloned();

            // ToolOrderUtils.cpp:453  for (int j = 0; j < R; ++j)
            for j in 0..r {
                // ToolOrderUtils.cpp:454  if (unlink_limits.has_value() && std::find(...) != ...end())
                if let Some(ul) = &unlink_limits {
                    if ul.contains(&j) {
                        // ToolOrderUtils.cpp:455  continue;
                        continue;
                    }
                }
                // ToolOrderUtils.cpp:456  m_solver_lower_bounds->add_edge(i, L + j, 1);
                self.m_solver_lower_bounds.add_edge(i, l + j, 1);
            }
        }

        // ToolOrderUtils.cpp:460  // r -> group
        // ToolOrderUtils.cpp:461  for (int j = 0; j < R; ++j)
        for j in 0..r {
            // ToolOrderUtils.cpp:462  int g = r_nodes_group[j];
            let g = self.r_nodes_group[j as usize];
            // ToolOrderUtils.cpp:463  m_solver_lower_bounds->add_edge(L + j, L + R + g, 1);
            self.m_solver_lower_bounds.add_edge(l + j, l + r + g, 1);
        }

        // ToolOrderUtils.cpp:466  // group -> sink（low bound=1）
        // ToolOrderUtils.cpp:467  for (int g = 0; g < num_groups; ++g)
        for g in 0..self.num_groups {
            // ToolOrderUtils.cpp:468  if (no_lower_groups.count(g))
            if no_lower_groups.contains(&g) {
                // ToolOrderUtils.cpp:469  m_solver_lower_bounds->add_edge(L + R + g, m_solver_lower_bounds->sink_id, R);
                let sink_id = self.m_solver_lower_bounds.sink_id;
                self.m_solver_lower_bounds.add_edge(l + r + g, sink_id, r);
            } else {
                // ToolOrderUtils.cpp:471  add_edge_with_lower_bound(L + R + g, m_solver_lower_bounds->sink_id, 1, R, 0);
                let sink_id = self.m_solver_lower_bounds.sink_id;
                self.add_edge_with_lower_bound(l + r + g, sink_id, 1, r, 0);
            }
        }

        // ToolOrderUtils.cpp:474  max_flow_edges = m_solver_lower_bounds->edges.size();
        self.max_flow_edges = self.m_solver_lower_bounds.edges.len() as i32;

        // ToolOrderUtils.cpp:476  // support lower bounds, add super source  super sink
        // ToolOrderUtils.cpp:477  super_source = m_solver_lower_bounds->total_nodes++;
        self.super_source = self.m_solver_lower_bounds.total_nodes;
        self.m_solver_lower_bounds.total_nodes += 1;
        // ToolOrderUtils.cpp:478  super_sink = m_solver_lower_bounds->total_nodes++;
        self.super_sink = self.m_solver_lower_bounds.total_nodes;
        self.m_solver_lower_bounds.total_nodes += 1;

        // ToolOrderUtils.cpp:480  m_solver_lower_bounds->adj.resize(m_solver_lower_bounds->total_nodes);
        self.m_solver_lower_bounds
            .adj
            .resize(self.m_solver_lower_bounds.total_nodes as usize, Vec::new());
        // ToolOrderUtils.cpp:481  demand.resize(m_solver_lower_bounds->total_nodes, 0);
        self.demand
            .resize(self.m_solver_lower_bounds.total_nodes as usize, 0);

        // ToolOrderUtils.cpp:483  for (int i = 0; i < super_source; ++i)
        for i in 0..self.super_source {
            // ToolOrderUtils.cpp:484  if (demand[i] > 0)
            if self.demand[i as usize] > 0 {
                // ToolOrderUtils.cpp:485  m_solver_lower_bounds->add_edge(super_source, i, demand[i]);
                let super_source = self.super_source;
                let d = self.demand[i as usize];
                self.m_solver_lower_bounds.add_edge(super_source, i, d);
            } else if self.demand[i as usize] < 0 {
                // ToolOrderUtils.cpp:486  else if (demand[i] < 0)
                // ToolOrderUtils.cpp:487  m_solver_lower_bounds->add_edge(i, super_sink, -demand[i]);
                let super_sink = self.super_sink;
                let d = self.demand[i as usize];
                self.m_solver_lower_bounds.add_edge(i, super_sink, -d);
            }
        }
        // ToolOrderUtils.cpp:490  m_solver_lower_bounds->add_edge(m_solver_lower_bounds->sink_id, m_solver_lower_bounds->source_id, MaxFlowGraph::INF);
        let sink_id = self.m_solver_lower_bounds.sink_id;
        let source_id = self.m_solver_lower_bounds.source_id;
        self.m_solver_lower_bounds
            .add_edge(sink_id, source_id, max_flow_graph::INF);
        // ToolOrderUtils.cpp:491  source_id = m_solver_lower_bounds->source_id;
        self.source_id = self.m_solver_lower_bounds.source_id;
        // ToolOrderUtils.cpp:492  sink_id = m_solver_lower_bounds->sink_id;
        self.sink_id = self.m_solver_lower_bounds.sink_id;
        // ToolOrderUtils.cpp:493  m_solver_lower_bounds->source_id = super_source;
        self.m_solver_lower_bounds.source_id = self.super_source;
        // ToolOrderUtils.cpp:494  m_solver_lower_bounds->sink_id = super_sink;
        self.m_solver_lower_bounds.sink_id = self.super_sink;
    }

    // ToolOrderUtils.cpp:497  void GeneralMinCostLowerBoundsSolver::build_graph_with_feasible_result()
    fn build_graph_with_feasible_result(&mut self) {
        // ToolOrderUtils.cpp:499  for (auto&lb:lower_bound_edges)
        for lb in &self.lower_bound_edges {
            // ToolOrderUtils.cpp:500  m_solver_lower_bounds->edges[lb.edge_id].flow += lb.lower;
            self.m_solver_lower_bounds.edges[lb.edge_id as usize].flow += lb.lower;
            // ToolOrderUtils.cpp:501  m_solver_lower_bounds->edges[lb.edge_id ^ 1].flow -= lb.lower;
            self.m_solver_lower_bounds.edges[(lb.edge_id ^ 1) as usize].flow -= lb.lower;
        }

        // ToolOrderUtils.cpp:504  m_solver_min_cost->l_nodes = m_solver_lower_bounds->l_nodes;
        self.m_solver_min_cost.l_nodes = self.m_solver_lower_bounds.l_nodes.clone();
        // ToolOrderUtils.cpp:505  m_solver_min_cost->r_nodes = m_solver_lower_bounds->r_nodes;
        self.m_solver_min_cost.r_nodes = self.m_solver_lower_bounds.r_nodes.clone();

        // ToolOrderUtils.cpp:507  m_solver_min_cost->source_id = source_id;
        self.m_solver_min_cost.source_id = self.source_id;
        // ToolOrderUtils.cpp:508  m_solver_min_cost->sink_id = sink_id;
        self.m_solver_min_cost.sink_id = self.sink_id;
        // ToolOrderUtils.cpp:509  m_solver_min_cost->total_nodes = sink_id + 1;
        self.m_solver_min_cost.total_nodes = self.sink_id + 1;

        // ToolOrderUtils.cpp:511  m_solver_min_cost->edges = m_solver_lower_bounds->edges;
        self.m_solver_min_cost.edges = self.m_solver_lower_bounds.edges.clone();
        // ToolOrderUtils.cpp:512  m_solver_min_cost->edges.erase(m_solver_min_cost->edges.begin() + max_flow_edges, m_solver_min_cost->edges.end());
        self.m_solver_min_cost.edges.truncate(self.max_flow_edges as usize);

        // ToolOrderUtils.cpp:514  m_solver_min_cost->adj = m_solver_lower_bounds->adj;
        self.m_solver_min_cost.adj = self.m_solver_lower_bounds.adj.clone();
        // ToolOrderUtils.cpp:515  m_solver_min_cost->adj.resize(m_solver_min_cost->total_nodes);
        self.m_solver_min_cost
            .adj
            .resize(self.m_solver_min_cost.total_nodes as usize, Vec::new());
        // ToolOrderUtils.cpp:516  for (auto &node_edges : m_solver_min_cost->adj)
        // ToolOrderUtils.cpp:517  node_edges.erase(std::remove_if(... val >= this->max_flow_edges ...), ...);
        let max_flow_edges = self.max_flow_edges;
        for node_edges in &mut self.m_solver_min_cost.adj {
            node_edges.retain(|&val| val < max_flow_edges);
        }

        // ToolOrderUtils.cpp:521  for (auto& e : m_solver_min_cost->edges)
        let l = self.m_solver_min_cost.l_nodes.len() as i32;
        let r = self.m_solver_min_cost.r_nodes.len() as i32;
        for ei in 0..self.m_solver_min_cost.edges.len() {
            // ToolOrderUtils.cpp:522  int L = m_solver_min_cost->l_nodes.size();
            // ToolOrderUtils.cpp:523  int R = m_solver_min_cost->r_nodes.size();
            let (e_from, e_to) = {
                let e = &self.m_solver_min_cost.edges[ei];
                (e.from, e.to)
            };

            // ToolOrderUtils.cpp:525  if (e.from < L && e.to >= L && e.to < L + R)
            if e_from < l && e_to >= l && e_to < l + r {
                // ToolOrderUtils.cpp:526  int idx_in_left  = e.from;
                let idx_in_left = e_from;
                // ToolOrderUtils.cpp:527  int idx_in_right = e.to - L;
                let idx_in_right = e_to - l;
                // ToolOrderUtils.cpp:528  int group_id = r_nodes_group[idx_in_right];
                let group_id = self.r_nodes_group[idx_in_right as usize];

                // ToolOrderUtils.cpp:530  if (r_nodes[idx_in_right] == -1) continue;
                if self.r_nodes[idx_in_right as usize] == -1 {
                    continue;
                }
                // ToolOrderUtils.cpp:531  e.cost = flush_matrix[group_id][l_nodes[idx_in_left]][r_nodes[idx_in_right]];
                self.m_solver_min_cost.edges[ei].cost = self.flush_matrix[group_id as usize]
                    [self.l_nodes[idx_in_left as usize] as usize]
                    [self.r_nodes[idx_in_right as usize] as usize]
                    as i32;
            }
        }
    }

    // ToolOrderUtils.cpp:535  void GeneralMinCostLowerBoundsSolver::add_edge_with_lower_bound(int from, int to, int lower, int upper, int cost)
    fn add_edge_with_lower_bound(&mut self, from: i32, to: i32, lower: i32, upper: i32, _cost: i32) {
        // ToolOrderUtils.cpp:537  int eid = m_solver_lower_bounds->edges.size();
        let eid = self.m_solver_lower_bounds.edges.len() as i32;
        // ToolOrderUtils.cpp:538  m_solver_lower_bounds->add_edge(from, to, upper - lower);
        self.m_solver_lower_bounds.add_edge(from, to, upper - lower);

        // ToolOrderUtils.cpp:540  lower_bound_edges.push_back({eid, lower});
        self.lower_bound_edges.push(LowerBoundEdge { edge_id: eid, lower });
        // ToolOrderUtils.cpp:541  demand[from] -= lower;
        self.demand[from as usize] -= lower;
        // ToolOrderUtils.cpp:542  demand[to]   += lower;
        self.demand[to as usize] += lower;
    }
}

// ==================== GroupMinCostFlowSolver ====================
// ToolOrderUtils.hpp:122  class GroupMinCostFlowSolver
pub struct GroupMinCostFlowSolver {
    m_solver: MinCostMaxFlow,
    flush_matrix: Vec<FlushMatrix>,
    l_nodes: Vec<i32>,
    r_nodes: Vec<i32>,
    r_nodes_group: Vec<i32>,
    m_uv_link_limits: HashMap<i32, Vec<i32>>,
    m_uv_unlink_limits: HashMap<i32, Vec<i32>>,
    num_groups: i32,
}

impl GroupMinCostFlowSolver {
    // ToolOrderUtils.cpp:548  GroupMinCostFlowSolver::GroupMinCostFlowSolver(...)
    pub fn new(
        matrix_: &[FlushMatrix],
        u_nodes: &[i32],
        v_nodes: &[i32],
        v_nodes_group: &[i32],
        uv_link_limits: &HashMap<i32, Vec<i32>>,
        uv_unlink_limits: &HashMap<i32, Vec<i32>>,
    ) -> Self {
        // ToolOrderUtils.cpp:555  flush_matrix       = matrix_;
        // ToolOrderUtils.cpp:556  l_nodes            = u_nodes;
        // ToolOrderUtils.cpp:557  r_nodes            = v_nodes;
        // ToolOrderUtils.cpp:558  r_nodes_group      = v_nodes_group;
        // ToolOrderUtils.cpp:559  m_uv_link_limits   = uv_link_limits;
        // ToolOrderUtils.cpp:560  m_uv_unlink_limits = uv_unlink_limits;
        // ToolOrderUtils.cpp:561  num_groups = *std::max_element(r_nodes_group.begin(), r_nodes_group.end()) + 1;
        let num_groups = *v_nodes_group.iter().max().unwrap() + 1;

        // ToolOrderUtils.cpp:563  m_solver = std::make_unique<MinCostMaxFlow>();
        let mut s = GroupMinCostFlowSolver {
            m_solver: MinCostMaxFlow::new(),
            flush_matrix: matrix_.to_vec(),
            l_nodes: u_nodes.to_vec(),
            r_nodes: v_nodes.to_vec(),
            r_nodes_group: v_nodes_group.to_vec(),
            m_uv_link_limits: uv_link_limits.clone(),
            m_uv_unlink_limits: uv_unlink_limits.clone(),
            num_groups,
        };
        // ToolOrderUtils.cpp:564  build_graph();
        s.build_graph();
        s
    }

    // ToolOrderUtils.cpp:567  int GroupMinCostFlowSolver::get_flush_cost(int l_idx, int r_idx)
    fn get_flush_cost(&self, l_idx: i32, r_idx: i32) -> i32 {
        // ToolOrderUtils.cpp:569  if (r_nodes[r_idx] == -1)
        if self.r_nodes[r_idx as usize] == -1 {
            // ToolOrderUtils.cpp:570  return 0;
            return 0;
        }
        // ToolOrderUtils.cpp:571  int group_id = r_nodes_group[r_idx];
        let group_id = self.r_nodes_group[r_idx as usize];
        // ToolOrderUtils.cpp:572  return (int)flush_matrix[group_id][l_nodes[l_idx]][r_nodes[r_idx]];
        self.flush_matrix[group_id as usize][self.l_nodes[l_idx as usize] as usize]
            [self.r_nodes[r_idx as usize] as usize] as i32
    }

    // ToolOrderUtils.cpp:575  void GroupMinCostFlowSolver::build_graph()
    fn build_graph(&mut self) {
        // ToolOrderUtils.cpp:577  const int L = (int)l_nodes.size();
        let l = self.l_nodes.len() as i32;
        // ToolOrderUtils.cpp:578  const int R = (int)r_nodes.size();
        let r = self.r_nodes.len() as i32;
        // ToolOrderUtils.cpp:579  const int G = num_groups;
        let g = self.num_groups;

        // ToolOrderUtils.cpp:581  m_solver->l_nodes    = l_nodes;
        self.m_solver.l_nodes = self.l_nodes.clone();
        // ToolOrderUtils.cpp:582  m_solver->r_nodes    = r_nodes;
        self.m_solver.r_nodes = self.r_nodes.clone();
        // ToolOrderUtils.cpp:583  m_solver->total_nodes = L + R + G + 2;
        self.m_solver.total_nodes = l + r + g + 2;
        // ToolOrderUtils.cpp:584  m_solver->source_id  = L + R + G;
        self.m_solver.source_id = l + r + g;
        // ToolOrderUtils.cpp:585  m_solver->sink_id    = L + R + G + 1;
        self.m_solver.sink_id = l + r + g + 1;
        // ToolOrderUtils.cpp:586  m_solver->adj.resize(m_solver->total_nodes);
        self.m_solver.adj = vec![Vec::new(); self.m_solver.total_nodes as usize];

        // ToolOrderUtils.cpp:588  int max_flush = 0;
        let mut max_flush = 0;
        // ToolOrderUtils.cpp:589  for (const auto &mat : flush_matrix)
        for mat in &self.flush_matrix {
            // ToolOrderUtils.cpp:590  for (const auto &row : mat)
            for row in mat {
                // ToolOrderUtils.cpp:591  for (float v : row)
                for &v in row {
                    // ToolOrderUtils.cpp:592  max_flush = std::max(max_flush, (int)v);
                    max_flush = max_flush.max(v as i32);
                }
            }
        }
        // ToolOrderUtils.cpp:593  int bonus = max_flush * L + 1;
        let bonus = max_flush * l + 1;

        // ToolOrderUtils.cpp:595  // source -> l_i
        // ToolOrderUtils.cpp:596  for (int i = 0; i < L; ++i)
        for i in 0..l {
            // ToolOrderUtils.cpp:597  m_solver->add_edge(m_solver->source_id, i, 1, 0);
            let source_id = self.m_solver.source_id;
            self.m_solver.add_edge(source_id, i, 1, 0);
        }

        // ToolOrderUtils.cpp:599  // l_i -> r_j (with link/unlink limits)
        // ToolOrderUtils.cpp:600  for (int i = 0; i < L; ++i)
        for i in 0..l {
            // ToolOrderUtils.cpp:601  if (auto it = m_uv_link_limits.find(i); it != m_uv_link_limits.end())
            if let Some(links) = self.m_uv_link_limits.get(&i).cloned() {
                // ToolOrderUtils.cpp:602  for (int j : it->second)
                for j in links {
                    // ToolOrderUtils.cpp:603  m_solver->add_edge(i, L + j, 1, get_flush_cost(i, j));
                    let cost = self.get_flush_cost(i, j);
                    self.m_solver.add_edge(i, l + j, 1, cost);
                }
                // ToolOrderUtils.cpp:604  continue;
                continue;
            }

            // ToolOrderUtils.cpp:607  std::optional<std::vector<int>> unlink_limits;
            // ToolOrderUtils.cpp:608  if (auto it = m_uv_unlink_limits.find(i); it != m_uv_unlink_limits.end())
            let unlink_limits = self.m_uv_unlink_limits.get(&i).cloned();

            // ToolOrderUtils.cpp:611  for (int j = 0; j < R; ++j)
            for j in 0..r {
                // ToolOrderUtils.cpp:612  if (unlink_limits.has_value() && std::find(...) != ...end())
                if let Some(ul) = &unlink_limits {
                    if ul.contains(&j) {
                        // ToolOrderUtils.cpp:613  continue;
                        continue;
                    }
                }
                // ToolOrderUtils.cpp:614  m_solver->add_edge(i, L + j, 1, get_flush_cost(i, j));
                let cost = self.get_flush_cost(i, j);
                self.m_solver.add_edge(i, l + j, 1, cost);
            }
        }

        // ToolOrderUtils.cpp:618-625  // r_j -> group_g  (capacity / nozzle bonus comments)
        // ToolOrderUtils.cpp:626  int nozzle_bonus = max_flush + 1;
        let nozzle_bonus = max_flush + 1;
        // ToolOrderUtils.cpp:627  std::vector<int> r_in_degree(R, 0);
        let mut r_in_degree = vec![0i32; r as usize];
        // ToolOrderUtils.cpp:628  for (int i = 0; i < L; ++i)
        for i in 0..l {
            // ToolOrderUtils.cpp:629  if (auto it = m_uv_link_limits.find(i); it != m_uv_link_limits.end())
            if let Some(links) = self.m_uv_link_limits.get(&i) {
                // ToolOrderUtils.cpp:630  for (int j : it->second)
                for &j in links {
                    // ToolOrderUtils.cpp:631  r_in_degree[j]++;
                    r_in_degree[j as usize] += 1;
                }
                // ToolOrderUtils.cpp:632  continue;
                continue;
            }
            // ToolOrderUtils.cpp:634  std::optional<std::vector<int>> unlink_limits;
            // ToolOrderUtils.cpp:635  if (auto it = m_uv_unlink_limits.find(i); it != m_uv_unlink_limits.end())
            let unlink_limits = self.m_uv_unlink_limits.get(&i);
            // ToolOrderUtils.cpp:637  for (int j = 0; j < R; ++j)
            for j in 0..r {
                // ToolOrderUtils.cpp:638  if (unlink_limits.has_value() && std::find(...) != ...end())
                if let Some(ul) = unlink_limits {
                    if ul.contains(&j) {
                        // ToolOrderUtils.cpp:639  continue;
                        continue;
                    }
                }
                // ToolOrderUtils.cpp:640  r_in_degree[j]++;
                r_in_degree[j as usize] += 1;
            }
        }

        // ToolOrderUtils.cpp:644  for (int j = 0; j < R; ++j)
        for j in 0..r {
            // ToolOrderUtils.cpp:645  int g = r_nodes_group[j];
            let gg = self.r_nodes_group[j as usize];
            // ToolOrderUtils.cpp:646  int cap = std::max(r_in_degree[j], 1);
            let cap = r_in_degree[j as usize].max(1);
            // ToolOrderUtils.cpp:647  // First unit gets -nozzle_bonus to prefer using distinct nozzles
            // ToolOrderUtils.cpp:648  m_solver->add_edge(L + j, L + R + g, 1, -nozzle_bonus);
            self.m_solver.add_edge(l + j, l + r + gg, 1, -nozzle_bonus);
            // ToolOrderUtils.cpp:649  if (cap > 1)
            if cap > 1 {
                // ToolOrderUtils.cpp:650  m_solver->add_edge(L + j, L + R + g, cap - 1, 0);
                self.m_solver.add_edge(l + j, l + r + gg, cap - 1, 0);
            }
        }

        // ToolOrderUtils.cpp:653-654  // group_g -> sink (split: first unit gets -bonus, rest gets 0)
        // ToolOrderUtils.cpp:655  for (int g = 0; g < G; ++g)
        for gg in 0..g {
            // ToolOrderUtils.cpp:656  m_solver->add_edge(L + R + g, m_solver->sink_id, 1, -bonus);
            let sink_id = self.m_solver.sink_id;
            self.m_solver.add_edge(l + r + gg, sink_id, 1, -bonus);
            // ToolOrderUtils.cpp:657  if (L > 1)
            if l > 1 {
                // ToolOrderUtils.cpp:658  m_solver->add_edge(L + R + g, m_solver->sink_id, L - 1, 0);
                self.m_solver.add_edge(l + r + gg, sink_id, l - 1, 0);
            }
        }
    }

    // ToolOrderUtils.cpp:662  std::vector<int> GroupMinCostFlowSolver::solve()
    pub fn solve(&mut self) -> Vec<i32> {
        // ToolOrderUtils.cpp:664  return m_solver->solve();
        self.m_solver.solve()
    }
}

// ==================== MinFlushFlowSolver ====================
// ToolOrderUtils.hpp:150  class MinFlushFlowSolver
pub struct MinFlushFlowSolver {
    m_solver: MinCostMaxFlow,
}

impl MinFlushFlowSolver {
    // ToolOrderUtils.cpp:672  MinFlushFlowSolver::MinFlushFlowSolver(...)
    pub fn new(
        matrix_: &[Vec<f32>],
        u_nodes: &[i32],
        v_nodes: &[i32],
        uv_link_limits: &HashMap<i32, Vec<i32>>,
        uv_unlink_limits: &HashMap<i32, Vec<i32>>,
        u_capacity: &[i32],
        v_capacity: &[i32],
        v_group_capacity: &[(BTreeSet<i32>, i32)],
    ) -> Self {
        // ToolOrderUtils.cpp:679  assert(u_capacity.empty() || u_capacity.size() == u_nodes.size());
        debug_assert!(u_capacity.is_empty() || u_capacity.len() == u_nodes.len());
        // ToolOrderUtils.cpp:680  assert(v_capacity.empty() || v_capacity.size() == v_nodes.size());
        debug_assert!(v_capacity.is_empty() || v_capacity.len() == v_nodes.len());

        // ToolOrderUtils.cpp:681  m_solver = std::make_unique<MinCostMaxFlow>();
        let mut m_solver = MinCostMaxFlow::new();
        // ToolOrderUtils.cpp:682  m_solver->matrix = matrix_;;
        m_solver.matrix = matrix_.to_vec();
        // ToolOrderUtils.cpp:683  m_solver->l_nodes = u_nodes;
        m_solver.l_nodes = u_nodes.to_vec();
        // ToolOrderUtils.cpp:684  m_solver->r_nodes = v_nodes;
        m_solver.r_nodes = v_nodes.to_vec();

        // ToolOrderUtils.cpp:686  m_solver->total_nodes = u_nodes.size() + v_nodes.size() + v_group_capacity.size() + 2;
        m_solver.total_nodes =
            (u_nodes.len() + v_nodes.len() + v_group_capacity.len() + 2) as i32;

        // ToolOrderUtils.cpp:688  m_solver->source_id =m_solver->total_nodes - 2;
        m_solver.source_id = m_solver.total_nodes - 2;
        // ToolOrderUtils.cpp:689  m_solver->sink_id = m_solver->total_nodes - 1;
        m_solver.sink_id = m_solver.total_nodes - 1;

        // ToolOrderUtils.cpp:691  m_solver->adj.resize(m_solver->total_nodes);
        m_solver.adj = vec![Vec::new(); m_solver.total_nodes as usize];

        // ToolOrderUtils.cpp:693  std::vector<int> v_node_to(v_nodes.size(), m_solver->sink_id);
        let mut v_node_to = vec![m_solver.sink_id; v_nodes.len()];
        // ToolOrderUtils.cpp:694  for (size_t gid = 0; gid < v_group_capacity.size(); ++gid)
        for (gid, gc) in v_group_capacity.iter().enumerate() {
            // ToolOrderUtils.cpp:695  for (auto vid : v_group_capacity[gid].first)
            for &vid in &gc.0 {
                // ToolOrderUtils.cpp:696  v_node_to[vid] = m_solver->l_nodes.size() + m_solver->r_nodes.size() + gid;
                v_node_to[vid as usize] =
                    (m_solver.l_nodes.len() + m_solver.r_nodes.len() + gid) as i32;
            }
        }

        // ToolOrderUtils.cpp:699  // add edge from source to left nodes,cost to 0
        // ToolOrderUtils.cpp:700  for (int i = 0; i < m_solver->l_nodes.size(); ++i)
        for i in 0..m_solver.l_nodes.len() as i32 {
            // ToolOrderUtils.cpp:701  int capacity = u_capacity.empty() ? 1 : u_capacity[i];
            let capacity = if u_capacity.is_empty() { 1 } else { u_capacity[i as usize] };
            // ToolOrderUtils.cpp:702  m_solver->add_edge(m_solver->source_id, i, capacity, 0);
            let source_id = m_solver.source_id;
            m_solver.add_edge(source_id, i, capacity, 0);
        }
        // ToolOrderUtils.cpp:704  // add edge from right nodes to sink,cost to 0
        // ToolOrderUtils.cpp:705  for (int i = 0; i < m_solver->r_nodes.size(); ++i)
        for i in 0..m_solver.r_nodes.len() as i32 {
            // ToolOrderUtils.cpp:706  int capacity = v_capacity.empty() ? 1 : v_capacity[i];
            let capacity = if v_capacity.is_empty() { 1 } else { v_capacity[i as usize] };
            // ToolOrderUtils.cpp:707  m_solver->add_edge(m_solver->l_nodes.size() + i, v_node_to[i], capacity, 0);
            let l = m_solver.l_nodes.len() as i32;
            m_solver.add_edge(l + i, v_node_to[i as usize], capacity, 0);
        }
        // ToolOrderUtils.cpp:709  // add edge from temp group node to sink node
        // ToolOrderUtils.cpp:710  for(int i=0;i<v_group_capacity.size();++i)
        for i in 0..v_group_capacity.len() {
            // ToolOrderUtils.cpp:711  int capacity = v_group_capacity[i].second;
            let capacity = v_group_capacity[i].1;
            // ToolOrderUtils.cpp:712  m_solver->add_edge(m_solver->l_nodes.size() + m_solver->r_nodes.size() + i, m_solver->sink_id, capacity, 0);
            let base = (m_solver.l_nodes.len() + m_solver.r_nodes.len() + i) as i32;
            let sink_id = m_solver.sink_id;
            m_solver.add_edge(base, sink_id, capacity, 0);
        }
        // ToolOrderUtils.cpp:714  // add edge from left node to right nodes
        // ToolOrderUtils.cpp:715  for (int i = 0; i < m_solver->l_nodes.size(); ++i)
        for i in 0..m_solver.l_nodes.len() as i32 {
            // ToolOrderUtils.cpp:716  int from_idx = i;
            let from_idx = i;
            // ToolOrderUtils.cpp:717  // process link limits, i can only link to link_limits
            // ToolOrderUtils.cpp:718  if (auto iter = uv_link_limits.find(i); iter != uv_link_limits.end())
            if let Some(links) = uv_link_limits.get(&i) {
                // ToolOrderUtils.cpp:719  for (auto r_id : iter->second)
                for &r_id in links {
                    // ToolOrderUtils.cpp:720  m_solver->add_edge(from_idx, m_solver->l_nodes.size() + r_id, 1, m_solver->get_distance(i, r_id));
                    let l = m_solver.l_nodes.len() as i32;
                    let dist = m_solver.get_distance(i, r_id);
                    m_solver.add_edge(from_idx, l + r_id, 1, dist);
                }
                // ToolOrderUtils.cpp:721  continue;
                continue;
            }

            // ToolOrderUtils.cpp:724  // process unlink limits, check whether i can link to j
            // ToolOrderUtils.cpp:725  std::optional<std::vector<int>> unlink_limits;
            // ToolOrderUtils.cpp:726  if (auto iter = uv_unlink_limits.find(i); iter != uv_unlink_limits.end())
            let unlink_limits = uv_unlink_limits.get(&i).cloned();
            // ToolOrderUtils.cpp:728  for (int j = 0; j < m_solver->r_nodes.size(); ++j)
            for j in 0..m_solver.r_nodes.len() as i32 {
                // ToolOrderUtils.cpp:729  if (unlink_limits.has_value() && std::find(...) != ...end())
                if let Some(ul) = &unlink_limits {
                    if ul.contains(&j) {
                        // ToolOrderUtils.cpp:730  continue;
                        continue;
                    }
                }
                // ToolOrderUtils.cpp:731  m_solver->add_edge(from_idx, m_solver->l_nodes.size() + j, 1, m_solver->get_distance(i, j));
                let l = m_solver.l_nodes.len() as i32;
                let dist = m_solver.get_distance(i, j);
                m_solver.add_edge(from_idx, l + j, 1, dist);
            }
        }

        MinFlushFlowSolver { m_solver }
    }

    // ToolOrderUtils.cpp:736  std::vector<int> MinFlushFlowSolver::solve()
    pub fn solve(&mut self) -> Vec<i32> {
        // ToolOrderUtils.cpp:737  return m_solver->solve();
        self.m_solver.solve()
    }
}

// ==================== MatchModeGroupSolver ====================
// ToolOrderUtils.hpp:169  class MatchModeGroupSolver
pub struct MatchModeGroupSolver {
    m_solver: MinCostMaxFlow,
}

impl MatchModeGroupSolver {
    // ToolOrderUtils.cpp:745  MatchModeGroupSolver::MatchModeGroupSolver(...)
    pub fn new(
        matrix_: &[Vec<f32>],
        u_nodes: &[i32],
        v_nodes: &[i32],
        v_capacity: &[i32],
        uv_unlink_limits: &HashMap<i32, Vec<i32>>,
    ) -> Self {
        // ToolOrderUtils.cpp:747  assert(v_nodes.size() == v_capacity.size());
        debug_assert_eq!(v_nodes.len(), v_capacity.len());
        // ToolOrderUtils.cpp:748  m_solver = std::make_unique<MinCostMaxFlow>();
        let mut m_solver = MinCostMaxFlow::new();
        // ToolOrderUtils.cpp:749  m_solver->matrix = matrix_;;
        m_solver.matrix = matrix_.to_vec();
        // ToolOrderUtils.cpp:750  m_solver->l_nodes = u_nodes;
        m_solver.l_nodes = u_nodes.to_vec();
        // ToolOrderUtils.cpp:751  m_solver->r_nodes = v_nodes;
        m_solver.r_nodes = v_nodes.to_vec();

        // ToolOrderUtils.cpp:753  m_solver->total_nodes = u_nodes.size() + v_nodes.size() + 2;
        m_solver.total_nodes = (u_nodes.len() + v_nodes.len() + 2) as i32;

        // ToolOrderUtils.cpp:755  m_solver->source_id = m_solver->total_nodes - 2;
        m_solver.source_id = m_solver.total_nodes - 2;
        // ToolOrderUtils.cpp:756  m_solver->sink_id = m_solver->total_nodes - 1;
        m_solver.sink_id = m_solver.total_nodes - 1;

        // ToolOrderUtils.cpp:758  m_solver->adj.resize(m_solver->total_nodes);
        m_solver.adj = vec![Vec::new(); m_solver.total_nodes as usize];

        // ToolOrderUtils.cpp:761  // add edge from source to left nodes,cost to 0
        // ToolOrderUtils.cpp:762  for (int i = 0; i < m_solver->l_nodes.size(); ++i)
        for i in 0..m_solver.l_nodes.len() as i32 {
            // ToolOrderUtils.cpp:763  m_solver->add_edge(m_solver->source_id, i, 1, 0);
            let source_id = m_solver.source_id;
            m_solver.add_edge(source_id, i, 1, 0);
        }

        // ToolOrderUtils.cpp:765  // add edge from right nodes to sink,cost to 0
        // ToolOrderUtils.cpp:766  for (int i = 0; i < m_solver->r_nodes.size(); ++i)
        for i in 0..m_solver.r_nodes.len() as i32 {
            // ToolOrderUtils.cpp:767  m_solver->add_edge(m_solver->l_nodes.size() + i, m_solver->sink_id, v_capacity[i], 0);
            let l = m_solver.l_nodes.len() as i32;
            let sink_id = m_solver.sink_id;
            m_solver.add_edge(l + i, sink_id, v_capacity[i as usize], 0);
        }

        // ToolOrderUtils.cpp:769  // add edge from left node to right nodes
        // ToolOrderUtils.cpp:770  for (int i = 0; i < m_solver->l_nodes.size(); ++i)
        for i in 0..m_solver.l_nodes.len() as i32 {
            // ToolOrderUtils.cpp:771  int from_idx = i;
            let from_idx = i;

            // ToolOrderUtils.cpp:773  // process unlink limits, check whether i can link to j
            // ToolOrderUtils.cpp:774  std::optional<std::vector<int>> unlink_limits;
            // ToolOrderUtils.cpp:775  if (auto iter = uv_unlink_limits.find(i); iter != uv_unlink_limits.end())
            let unlink_limits = uv_unlink_limits.get(&i).cloned();
            // ToolOrderUtils.cpp:777  for (int j = 0; j < m_solver->r_nodes.size(); ++j)
            for j in 0..m_solver.r_nodes.len() as i32 {
                // ToolOrderUtils.cpp:778  if (unlink_limits.has_value() && std::find(...) != ...end())
                if let Some(ul) = &unlink_limits {
                    if ul.contains(&j) {
                        // ToolOrderUtils.cpp:779  continue;
                        continue;
                    }
                }
                // ToolOrderUtils.cpp:780  m_solver->add_edge(from_idx, m_solver->l_nodes.size() + j, 1, m_solver->get_distance(i, j));
                let l = m_solver.l_nodes.len() as i32;
                let dist = m_solver.get_distance(i, j);
                m_solver.add_edge(from_idx, l + j, 1, dist);
            }
        }

        MatchModeGroupSolver { m_solver }
    }

    // ToolOrderUtils.cpp:785  std::vector<int> MatchModeGroupSolver::solve()
    pub fn solve(&mut self) -> Vec<i32> {
        // ToolOrderUtils.cpp:786  return m_solver->solve();
        self.m_solver.solve()
    }
}

// ==================== static functions ====================
// ToolOrderUtils.cpp:790  //solve the problem by searching the least flush of current filament
// ToolOrderUtils.cpp:791  static std::vector<unsigned int> solve_extruder_order_with_greedy(...)
fn solve_extruder_order_with_greedy(
    wipe_volumes: &[Vec<f32>],
    curr_layer_extruders: &[u32],
    start_extruder_id: Option<u32>,
    min_cost: Option<&mut f32>,
) -> Vec<u32> {
    // ToolOrderUtils.cpp:796  float cost = 0;
    let mut cost: f32 = 0.0;
    // ToolOrderUtils.cpp:797  std::vector<unsigned int> best_seq;
    let mut best_seq: Vec<u32> = Vec::new();
    // ToolOrderUtils.cpp:798  std::vector<bool>is_visited(curr_layer_extruders.size(), false);
    let mut is_visited = vec![false; curr_layer_extruders.len()];
    // ToolOrderUtils.cpp:799  std::optional<unsigned int>prev_filament = start_extruder_id;
    let mut prev_filament = start_extruder_id;
    // ToolOrderUtils.cpp:800  int idx = curr_layer_extruders.size();
    let mut idx = curr_layer_extruders.len() as i32;
    // ToolOrderUtils.cpp:801  while (idx > 0)
    while idx > 0 {
        // ToolOrderUtils.cpp:802  if (!prev_filament)
        if prev_filament.is_none() {
            // ToolOrderUtils.cpp:803  auto iter = std::find_if(is_visited.begin(), is_visited.end(), [](auto item) {return item == 0; });
            let pos = is_visited.iter().position(|&item| !item);
            // ToolOrderUtils.cpp:804  assert(iter != is_visited.end());
            debug_assert!(pos.is_some());
            // ToolOrderUtils.cpp:805  prev_filament = curr_layer_extruders[iter - is_visited.begin()];
            prev_filament = Some(curr_layer_extruders[pos.unwrap()]);
        }
        // ToolOrderUtils.cpp:807  int target_idx = -1;
        let mut target_idx: i32 = -1;
        // ToolOrderUtils.cpp:808  int target_cost = std::numeric_limits<int>::max();
        let mut target_cost: i32 = i32::MAX;
        // ToolOrderUtils.cpp:809  for (size_t k = 0; k < is_visited.size(); ++k)
        for k in 0..is_visited.len() {
            // ToolOrderUtils.cpp:810  if (!is_visited[k])
            if !is_visited[k] {
                let pf = prev_filament.unwrap();
                // ToolOrderUtils.cpp:811-812  comparison of wipe_volumes against target_cost (int) and equality+self-loop check
                // C++ compares `float < int` / `float == int`: the `int` target_cost is
                // promoted to float, so the comparison happens in float space. Mirror that
                // by promoting target_cost to f32, not by truncating wv first.
                let wv = wipe_volumes[pf as usize][curr_layer_extruders[k] as usize];
                if wv < target_cost as f32
                    || (wv == target_cost as f32 && pf == curr_layer_extruders[k])
                {
                    // ToolOrderUtils.cpp:813  target_idx = k;
                    target_idx = k as i32;
                    // ToolOrderUtils.cpp:814  target_cost = wipe_volumes[*prev_filament][curr_layer_extruders[k]];
                    // (float->int assignment truncates, matching C++ `int target_cost`)
                    target_cost = wv as i32;
                }
            }
        }
        // ToolOrderUtils.cpp:818  assert(target_idx != -1);
        debug_assert!(target_idx != -1);
        // ToolOrderUtils.cpp:819  cost += target_cost;
        cost += target_cost as f32;
        // ToolOrderUtils.cpp:820  best_seq.emplace_back(curr_layer_extruders[target_idx]);
        best_seq.push(curr_layer_extruders[target_idx as usize]);
        // ToolOrderUtils.cpp:821  prev_filament = curr_layer_extruders[target_idx];
        prev_filament = Some(curr_layer_extruders[target_idx as usize]);
        // ToolOrderUtils.cpp:822  is_visited[target_idx] = true;
        is_visited[target_idx as usize] = true;
        // ToolOrderUtils.cpp:823  idx -= 1;
        idx -= 1;
    }
    // ToolOrderUtils.cpp:825  if (min_cost)
    if let Some(mc) = min_cost {
        // ToolOrderUtils.cpp:826  *min_cost = cost;
        *mc = cost;
    }
    // ToolOrderUtils.cpp:827  return best_seq;
    best_seq
}

// ToolOrderUtils.cpp:830  //solve the problem by forcasting one layer
// ToolOrderUtils.cpp:831  static std::vector<unsigned int> solve_extruder_order_with_forcast(...)
fn solve_extruder_order_with_forcast(
    wipe_volumes: &[Vec<f32>],
    mut curr_layer_extruders: Vec<u32>,
    mut next_layer_extruders: Vec<u32>,
    start_extruder_id: Option<u32>,
    min_cost: Option<&mut f32>,
) -> Vec<u32> {
    // ToolOrderUtils.cpp:837  std::sort(curr_layer_extruders.begin(), curr_layer_extruders.end());
    curr_layer_extruders.sort_unstable();
    // ToolOrderUtils.cpp:838  float best_cost = std::numeric_limits<float>::max();
    let mut best_cost = f32::MAX;
    // ToolOrderUtils.cpp:839  int best_change = std::numeric_limits<int>::max();
    let mut best_change = i32::MAX;
    // ToolOrderUtils.cpp:840  std::vector<unsigned int>best_seq;
    let mut best_seq: Vec<u32> = Vec::new();

    // ToolOrderUtils.cpp:842  auto get_filament_change_count = [](...)
    let get_filament_change_count =
        |curr_seq: &[u32], next_seq: &[u32], start_extruder_id: Option<u32>| -> i32 {
            // ToolOrderUtils.cpp:843  int count = 0;
            let mut count = 0;
            // ToolOrderUtils.cpp:844  auto prev_extruder_id = start_extruder_id;
            let mut prev_extruder_id = start_extruder_id;
            // ToolOrderUtils.cpp:845  for (auto seq : { curr_seq,next_seq })
            for seq in [curr_seq, next_seq] {
                // ToolOrderUtils.cpp:846  for (auto eid : seq)
                for &eid in seq {
                    // ToolOrderUtils.cpp:847  if (prev_extruder_id && prev_extruder_id != eid)
                    if let Some(p) = prev_extruder_id {
                        if p != eid {
                            // ToolOrderUtils.cpp:848  count += 1;
                            count += 1;
                        }
                    }
                    // ToolOrderUtils.cpp:850  prev_extruder_id = eid;
                    prev_extruder_id = Some(eid);
                }
            }
            // ToolOrderUtils.cpp:853  return count;
            count
        };

    // ToolOrderUtils.cpp:857  do { ... } while (std::next_permutation(curr_layer_extruders...));
    loop {
        // ToolOrderUtils.cpp:858  std::optional<unsigned int>prev_extruder_1 = start_extruder_id;
        let mut prev_extruder_1 = start_extruder_id;
        // ToolOrderUtils.cpp:859  float curr_layer_cost = 0;
        let mut curr_layer_cost: f32 = 0.0;
        // ToolOrderUtils.cpp:860  for (size_t idx = 0; idx < curr_layer_extruders.size(); ++idx)
        for idx in 0..curr_layer_extruders.len() {
            // ToolOrderUtils.cpp:861  if (prev_extruder_1)
            if let Some(p) = prev_extruder_1 {
                // ToolOrderUtils.cpp:862  curr_layer_cost += wipe_volumes[*prev_extruder_1][curr_layer_extruders[idx]];
                curr_layer_cost += wipe_volumes[p as usize][curr_layer_extruders[idx] as usize];
            }
            // ToolOrderUtils.cpp:863  prev_extruder_1 = curr_layer_extruders[idx];
            prev_extruder_1 = Some(curr_layer_extruders[idx]);
        }
        // ToolOrderUtils.cpp:865  if (curr_layer_cost > best_cost)
        if curr_layer_cost > best_cost {
            // ToolOrderUtils.cpp:866  continue;
            if !next_permutation(&mut curr_layer_extruders) {
                break;
            }
            continue;
        }
        // ToolOrderUtils.cpp:867  std::sort(next_layer_extruders.begin(), next_layer_extruders.end());
        next_layer_extruders.sort_unstable();
        // ToolOrderUtils.cpp:868  do { ... } while (std::next_permutation(next_layer_extruders...));
        loop {
            // ToolOrderUtils.cpp:869  std::optional<unsigned int>prev_extruder_2 = prev_extruder_1;
            let mut prev_extruder_2 = prev_extruder_1;
            // ToolOrderUtils.cpp:870  float total_cost = curr_layer_cost;
            let mut total_cost = curr_layer_cost;
            // ToolOrderUtils.cpp:871  int total_change = get_filament_change_count(curr_layer_extruders, next_layer_extruders, start_extruder_id);
            let total_change = get_filament_change_count(
                &curr_layer_extruders,
                &next_layer_extruders,
                start_extruder_id,
            );

            // ToolOrderUtils.cpp:873  for (size_t idx = 0; idx < next_layer_extruders.size(); ++idx)
            for idx in 0..next_layer_extruders.len() {
                // ToolOrderUtils.cpp:874  if (prev_extruder_2)
                if let Some(p) = prev_extruder_2 {
                    // ToolOrderUtils.cpp:875  total_cost += wipe_volumes[*prev_extruder_2][next_layer_extruders[idx]];
                    total_cost += wipe_volumes[p as usize][next_layer_extruders[idx] as usize];
                }
                // ToolOrderUtils.cpp:876  prev_extruder_2 = next_layer_extruders[idx];
                prev_extruder_2 = Some(next_layer_extruders[idx]);
            }

            // ToolOrderUtils.cpp:879  if (total_cost < best_cost || (total_cost == best_cost && total_change < best_change))
            if total_cost < best_cost || (total_cost == best_cost && total_change < best_change) {
                // ToolOrderUtils.cpp:880  best_cost = total_cost;
                best_cost = total_cost;
                // ToolOrderUtils.cpp:881  best_seq = curr_layer_extruders;
                best_seq = curr_layer_extruders.clone();
                // ToolOrderUtils.cpp:882  best_change = total_change;
                best_change = total_change;
            }
            // ToolOrderUtils.cpp:884  } while (std::next_permutation(next_layer_extruders...));
            if !next_permutation(&mut next_layer_extruders) {
                break;
            }
        }
        // ToolOrderUtils.cpp:885  } while (std::next_permutation(curr_layer_extruders...));
        if !next_permutation(&mut curr_layer_extruders) {
            break;
        }
    }

    // ToolOrderUtils.cpp:887  if (min_cost)
    if let Some(mc) = min_cost {
        // ToolOrderUtils.cpp:888  float real_cost = 0;
        let mut real_cost: f32 = 0.0;
        // ToolOrderUtils.cpp:889  std::optional<unsigned int>prev_extruder = start_extruder_id;
        let mut prev_extruder = start_extruder_id;
        // ToolOrderUtils.cpp:890  for (size_t idx = 0; idx < best_seq.size(); ++idx)
        for idx in 0..best_seq.len() {
            // ToolOrderUtils.cpp:891  if (prev_extruder)
            if let Some(p) = prev_extruder {
                // ToolOrderUtils.cpp:892  real_cost += wipe_volumes[*prev_extruder][best_seq[idx]];
                real_cost += wipe_volumes[p as usize][best_seq[idx] as usize];
            }
            // ToolOrderUtils.cpp:893  prev_extruder = best_seq[idx];
            prev_extruder = Some(best_seq[idx]);
        }
        // ToolOrderUtils.cpp:895  *min_cost = real_cost;
        *mc = real_cost;
    }
    // ToolOrderUtils.cpp:897  return best_seq;
    best_seq
}

// Helper: std::next_permutation equivalent over a slice (lexicographic).
// Returns false (and resets to the sorted/ascending order) when the sequence
// is the last permutation, mirroring std::next_permutation's return contract.
fn next_permutation<T: Ord>(arr: &mut [T]) -> bool {
    if arr.len() < 2 {
        return false;
    }
    let mut i = arr.len() - 1;
    loop {
        let i1 = i;
        i -= 1;
        if arr[i] < arr[i1] {
            let mut j = arr.len() - 1;
            while !(arr[i] < arr[j]) {
                j -= 1;
            }
            arr.swap(i, j);
            arr[i1..].reverse();
            return true;
        }
        if i == 0 {
            arr.reverse();
            return false;
        }
    }
}

// ToolOrderUtils.cpp:900  // Shortest hamilton path problem
// ToolOrderUtils.cpp:901  static std::vector<unsigned int> solve_extruder_order(...)
fn solve_extruder_order(
    wipe_volumes: &[Vec<f32>],
    mut all_extruders: Vec<u32>,
    mut start_extruder_id: Option<u32>,
    min_cost: Option<&mut f32>,
) -> Vec<u32> {
    // ToolOrderUtils.cpp:906  bool add_start_extruder_flag = false;
    let mut add_start_extruder_flag = false;

    // ToolOrderUtils.cpp:908  if (start_extruder_id)
    if let Some(start) = start_extruder_id {
        // ToolOrderUtils.cpp:909  auto start_iter = std::find(all_extruders.begin(), all_extruders.end(), start_extruder_id);
        // ToolOrderUtils.cpp:910  if (start_iter == all_extruders.end())
        if let Some(pos) = all_extruders.iter().position(|&x| x == start) {
            // ToolOrderUtils.cpp:913  std::swap(*all_extruders.begin(), *start_iter);
            all_extruders.swap(0, pos);
        } else {
            // ToolOrderUtils.cpp:911  all_extruders.insert(all_extruders.begin(), *start_extruder_id), add_start_extruder_flag = true;
            all_extruders.insert(0, start);
            add_start_extruder_flag = true;
        }
    } else {
        // ToolOrderUtils.cpp:916  start_extruder_id = all_extruders.front();
        start_extruder_id = Some(all_extruders[0]);
    }

    // ToolOrderUtils.cpp:919  unsigned int iterations = (1 << all_extruders.size());
    let iterations: u32 = 1u32 << all_extruders.len();
    // ToolOrderUtils.cpp:920  unsigned int final_state = iterations - 1;
    let final_state = iterations - 1;
    // ToolOrderUtils.cpp:921  std::vector<std::vector<float>>cache(iterations, std::vector<float>(all_extruders.size(), 0x7fffffff));
    let mut cache: Vec<Vec<f32>> =
        vec![vec![0x7fffffff as f32; all_extruders.len()]; iterations as usize];
    // ToolOrderUtils.cpp:922  std::vector<std::vector<int>>prev(iterations, std::vector<int>(all_extruders.size(), -1));
    let mut prev: Vec<Vec<i32>> = vec![vec![-1; all_extruders.len()]; iterations as usize];
    // ToolOrderUtils.cpp:923  cache[1][0] = 0.;
    cache[1][0] = 0.0;
    // ToolOrderUtils.cpp:924  for (unsigned int state = 0; state < iterations; ++state)
    for state in 0..iterations {
        // ToolOrderUtils.cpp:925  if (state & 1)
        if state & 1 != 0 {
            // ToolOrderUtils.cpp:926  for (unsigned int target = 0; target < all_extruders.size(); ++target)
            for target in 0..all_extruders.len() as u32 {
                // ToolOrderUtils.cpp:927  if (state >> target & 1)
                if (state >> target) & 1 != 0 {
                    // ToolOrderUtils.cpp:928  for (unsigned int mid_point = 0; mid_point < all_extruders.size(); ++mid_point)
                    for mid_point in 0..all_extruders.len() as u32 {
                        // ToolOrderUtils.cpp:929  if (state >> mid_point & 1)
                        if (state >> mid_point) & 1 != 0 {
                            // ToolOrderUtils.cpp:930  auto tmp = cache[state - (1 << target)][mid_point] + wipe_volumes[all_extruders[mid_point]][all_extruders[target]];
                            let tmp = cache[(state - (1u32 << target)) as usize][mid_point as usize]
                                + wipe_volumes[all_extruders[mid_point as usize] as usize]
                                    [all_extruders[target as usize] as usize];
                            // ToolOrderUtils.cpp:931  if (cache[state][target] > tmp)
                            if cache[state as usize][target as usize] > tmp {
                                // ToolOrderUtils.cpp:932  cache[state][target] = tmp;
                                cache[state as usize][target as usize] = tmp;
                                // ToolOrderUtils.cpp:933  prev[state][target] = mid_point;
                                prev[state as usize][target as usize] = mid_point as i32;
                            }
                        }
                    }
                }
            }
        }
    }

    // ToolOrderUtils.cpp:942  //get res
    // ToolOrderUtils.cpp:943  float cost = std::numeric_limits<float>::max();
    let mut cost = f32::MAX;
    // ToolOrderUtils.cpp:944  int final_dst = 0;
    let mut final_dst: i32 = 0;
    // ToolOrderUtils.cpp:945  for (unsigned int dst = 0; dst < all_extruders.size(); ++dst)
    for dst in 0..all_extruders.len() as u32 {
        // ToolOrderUtils.cpp:946  if (all_extruders[dst] != start_extruder_id && cost > cache[final_state][dst])
        if Some(all_extruders[dst as usize]) != start_extruder_id
            && cost > cache[final_state as usize][dst as usize]
        {
            // ToolOrderUtils.cpp:947  cost = cache[final_state][dst];
            cost = cache[final_state as usize][dst as usize];
            // ToolOrderUtils.cpp:948  if (min_cost)
            // (deferred until after the loop; see below — C++ writes inside loop)
            // ToolOrderUtils.cpp:950  final_dst = dst;
            final_dst = dst as i32;
        }
    }
    // ToolOrderUtils.cpp:948-949  if (min_cost) *min_cost = cost;
    // C++ assigns *min_cost on each improving iteration; the final value equals
    // the last (best) cost. We assign once with the resulting best cost.
    if let Some(mc) = min_cost {
        *mc = cost;
    }

    // ToolOrderUtils.cpp:954  std::vector<unsigned int>path;
    let mut path: Vec<u32> = Vec::new();
    // ToolOrderUtils.cpp:955  unsigned int curr_state = final_state;
    let mut curr_state = final_state;
    // ToolOrderUtils.cpp:956  int curr_point = final_dst;
    let mut curr_point = final_dst;
    // ToolOrderUtils.cpp:957  while (curr_point != -1)
    while curr_point != -1 {
        // ToolOrderUtils.cpp:958  path.emplace_back(all_extruders[curr_point]);
        path.push(all_extruders[curr_point as usize]);
        // ToolOrderUtils.cpp:959  auto mid_point = prev[curr_state][curr_point];
        let mid_point = prev[curr_state as usize][curr_point as usize];
        // ToolOrderUtils.cpp:960  curr_state -= (1 << curr_point);
        curr_state -= 1u32 << curr_point;
        // ToolOrderUtils.cpp:961  curr_point = mid_point;
        curr_point = mid_point;
    }

    // ToolOrderUtils.cpp:964  if (add_start_extruder_flag)
    if add_start_extruder_flag {
        // ToolOrderUtils.cpp:965  path.pop_back();
        path.pop();
    }

    // ToolOrderUtils.cpp:967  std::reverse(path.begin(), path.end());
    path.reverse();
    // ToolOrderUtils.cpp:968  return path;
    path
}

/// The `static_cast<T>(f)` of `collect_filaments_in_groups<T>`, for the two
/// instantiations the file uses (`unsigned int` and `int`).
trait FilamentIdCast {
    fn from_filament_id(f: u32) -> Self;
}
impl FilamentIdCast for u32 {
    fn from_filament_id(f: u32) -> Self {
        f
    }
}
impl FilamentIdCast for i32 {
    fn from_filament_id(f: u32) -> Self {
        f as i32
    }
}

// ToolOrderUtils.cpp:967  template<class T>
// ToolOrderUtils.cpp:968  static std::vector<T> collect_filaments_in_groups(const std::unordered_set<unsigned int>& group, const std::vector<unsigned int>& filament_list) {
fn collect_filaments_in_groups<T: FilamentIdCast>(group: &HashSet<u32>, filament_list: &[u32]) -> Vec<T> {
    // ToolOrderUtils.cpp:969  std::vector<T>ret;
    let mut ret: Vec<T> = Vec::new();
    // ToolOrderUtils.cpp:970  ret.reserve(group.size());
    ret.reserve(group.len());
    // ToolOrderUtils.cpp:971  for (auto& f : filament_list) {
    for &f in filament_list {
        // ToolOrderUtils.cpp:972  if (auto iter = group.find(f); iter != group.end())
        if group.contains(&f) {
            // ToolOrderUtils.cpp:973  ret.emplace_back(static_cast<T>(f));
            ret.push(T::from_filament_id(f));
        }
    }
    // ToolOrderUtils.cpp:975  return ret;
    ret
}

fn collect_filaments_in_groups_u32(group: &HashSet<u32>, filament_list: &[u32]) -> Vec<u32> {
    collect_filaments_in_groups::<u32>(group, filament_list)
}

// ToolOrderUtils.cpp:983  // get best filament order of single nozzle
// ToolOrderUtils.cpp:984  std::vector<unsigned int> get_extruders_order(...)
pub fn get_extruders_order(
    wipe_volumes: &[Vec<f32>],
    curr_layer_extruders: &[u32],
    next_layer_extruders: &[u32],
    start_extruder_id: Option<u32>,
    use_forcast: bool,
    cost: Option<&mut f32>,
) -> Vec<u32> {
    // ToolOrderUtils.cpp:991  if (curr_layer_extruders.empty())
    if curr_layer_extruders.is_empty() {
        // ToolOrderUtils.cpp:992  if (cost)
        if let Some(c) = cost {
            // ToolOrderUtils.cpp:993  *cost = 0;
            *c = 0.0;
        }
        // ToolOrderUtils.cpp:994  return curr_layer_extruders;
        return curr_layer_extruders.to_vec();
    }
    // ToolOrderUtils.cpp:996  if (curr_layer_extruders.size() == 1)
    if curr_layer_extruders.len() == 1 {
        // ToolOrderUtils.cpp:997  if (cost)
        if let Some(c) = cost {
            // ToolOrderUtils.cpp:998  *cost = 0;
            *c = 0.0;
            // ToolOrderUtils.cpp:999  if (start_extruder_id)
            if let Some(start) = start_extruder_id {
                // ToolOrderUtils.cpp:1000  *cost = wipe_volumes[*start_extruder_id][curr_layer_extruders[0]];
                *c = wipe_volumes[start as usize][curr_layer_extruders[0] as usize];
            }
        }
        // ToolOrderUtils.cpp:1002  return curr_layer_extruders;
        return curr_layer_extruders.to_vec();
    }

    // ToolOrderUtils.cpp:1005  if (use_forcast)
    if use_forcast {
        // ToolOrderUtils.cpp:1006  return solve_extruder_order_with_forcast(...)
        solve_extruder_order_with_forcast(
            wipe_volumes,
            curr_layer_extruders.to_vec(),
            next_layer_extruders.to_vec(),
            start_extruder_id,
            cost,
        )
    } else if curr_layer_extruders.len() <= 20 {
        // ToolOrderUtils.cpp:1007  else if (curr_layer_extruders.size() <= 20)
        // ToolOrderUtils.cpp:1008  return solve_extruder_order(...)
        solve_extruder_order(
            wipe_volumes,
            curr_layer_extruders.to_vec(),
            start_extruder_id,
            cost,
        )
    } else {
        // ToolOrderUtils.cpp:1010  return solve_extruder_order_with_greedy(...)
        solve_extruder_order_with_greedy(wipe_volumes, curr_layer_extruders, start_extruder_id, cost)
    }
}

// ToolOrderUtils.cpp:1016  // TODO:  add cusotm sequence
// ToolOrderUtils.cpp:1017  static int reorder_filaments_for_minimum_flush_volume_base(...)
fn reorder_filaments_for_minimum_flush_volume_base(
    filament_lists: &[u32],
    layer_filaments: &[Vec<u32>],
    flush_matrix: &FlushMatrix,
    get_custom_seq: Option<&dyn Fn(i32, &mut Vec<i32>) -> bool>,
    filament_sequences: Option<&mut Vec<Vec<u32>>>,
    initial_filament_id: Option<u32>,
) -> i32 {
    let pin = tool_order_utils_pin();
    // ToolOrderUtils.cpp:1019  constexpr int max_n_with_forcast = 5;
    const MAX_N_WITH_FORCAST: usize = 5;
    // ToolOrderUtils.cpp:1025  using uint128_t = boost::multiprecision::uint128_t; -> u128

    // ToolOrderUtils.cpp:1027  if (filament_sequences)
    let mut filament_sequences = filament_sequences;
    if let Some(fs) = filament_sequences.as_deref_mut() {
        // ToolOrderUtils.cpp:1028  filament_sequences->clear();
        fs.clear();
        // ToolOrderUtils.cpp:1029  filament_sequences->reserve(layer_filaments.size());
        fs.reserve(layer_filaments.len());
    }

    // ToolOrderUtils.cpp:1031  auto filament_list_to_hash_key = [](...) -> uint128_t
    let filament_list_to_hash_key =
        |curr_layer_filaments: &[u32],
         next_layer_filaments: &[u32],
         prev_filament: Option<u32>,
         use_forcast: bool|
         -> u128 {
            // ToolOrderUtils.cpp:1033  uint128_t hash_key = 0;
            let mut hash_key: u128 = 0;
            // ToolOrderUtils.cpp:1035  if (prev_filament) hash_key |= (uint128_t(1) << (64 + *prev_filament));
            if let Some(pf) = prev_filament {
                hash_key |= 1u128 << (64 + pf);
            }
            // ToolOrderUtils.cpp:1037  if (use_forcast)
            if use_forcast {
                // ToolOrderUtils.cpp:1038  for (auto item : next_layer_filaments) { hash_key |= (uint128_t(1) << (32 + item)); }
                for &item in next_layer_filaments {
                    hash_key |= 1u128 << (32 + item);
                }
            }
            // ToolOrderUtils.cpp:1041  for (auto item : curr_layer_filaments) { hash_key |= (uint128_t(1) << item); }
            for &item in curr_layer_filaments {
                hash_key |= 1u128 << item;
            }
            // ToolOrderUtils.cpp:1042  return hash_key;
            hash_key
        };

    // ToolOrderUtils.cpp:1045  int cost = 0;
    let mut cost = 0;
    // ToolOrderUtils.cpp:1046  std::map<size_t, std::vector<unsigned int>> custom_layer_sequence_map;
    let mut custom_layer_sequence_map: BTreeMap<usize, Vec<u32>> = BTreeMap::new();
    // ToolOrderUtils.cpp:1047  std::unordered_map<uint128_t, std::pair<float, std::vector<unsigned int>>> caches;
    let mut caches: HashMap<u128, (f32, Vec<u32>)> = HashMap::new();
    // ToolOrderUtils.cpp:1048  std::unordered_set<unsigned int> filament_sets(filament_lists.begin(), filament_lists.end());
    let filament_sets: HashSet<u32> = filament_lists.iter().copied().collect();
    // ToolOrderUtils.cpp:1049  std::optional<unsigned int> curr_filament_id;
    let mut curr_filament_id: Option<u32> = None;
    // ToolOrderUtils.cpp:1050  // 如果传入了有效的初始材料ID，则使用它作为初始状态
    // ToolOrderUtils.cpp:1051  if (initial_filament_id.has_value() && *initial_filament_id < flush_matrix.size())
    if let Some(init) = initial_filament_id {
        if (init as usize) < flush_matrix.len() {
            // ToolOrderUtils.cpp:1052  curr_filament_id = initial_filament_id;
            curr_filament_id = initial_filament_id;
        }
    }

    // ToolOrderUtils.cpp:1055  for (size_t layer = 0; layer < layer_filaments.size(); ++layer)
    for layer in 0..layer_filaments.len() {
        // ToolOrderUtils.cpp:1056  const auto& curr_lf = layer_filaments[layer];  (unused below in this loop)
        // ToolOrderUtils.cpp:1057  std::vector<int> custom_filament_seq;
        let mut custom_filament_seq: Vec<i32> = Vec::new();
        // ToolOrderUtils.cpp:1058  if (get_custom_seq && get_custom_seq(layer, custom_filament_seq) && !custom_filament_seq.empty())
        if let Some(gcs) = get_custom_seq {
            if gcs(layer as i32, &mut custom_filament_seq) && !custom_filament_seq.is_empty() {
                // ToolOrderUtils.cpp:1059  std::vector<unsigned int> unsign_custom_extruder_seq;
                let mut unsign_custom_extruder_seq: Vec<u32> = Vec::new();
                // ToolOrderUtils.cpp:1060  for (int extruder : custom_filament_seq)
                for &extruder in &custom_filament_seq {
                    // ToolOrderUtils.cpp:1061  unsigned int unsign_extruder = static_cast<unsigned int>(extruder) - 1;
                    let unsign_extruder = (extruder as u32).wrapping_sub(1);
                    // ToolOrderUtils.cpp:1062  auto it = std::find(layer_filaments[layer].begin(), layer_filaments[layer].end(), unsign_extruder);
                    // ToolOrderUtils.cpp:1063  if (it != layer_filaments[layer].end())
                    if layer_filaments[layer].contains(&unsign_extruder) {
                        // ToolOrderUtils.cpp:1064  unsign_custom_extruder_seq.emplace_back(unsign_extruder);
                        unsign_custom_extruder_seq.push(unsign_extruder);
                    }
                }
                // ToolOrderUtils.cpp:1066  assert(layer_filaments[layer].size() == unsign_custom_extruder_seq.size());
                debug_assert_eq!(layer_filaments[layer].len(), unsign_custom_extruder_seq.len());

                // ToolOrderUtils.cpp:1068  custom_layer_sequence_map[layer] = unsign_custom_extruder_seq;
                custom_layer_sequence_map.insert(layer, unsign_custom_extruder_seq);
            }
        }
    }

    // ToolOrderUtils.cpp:1072  for (size_t layer = 0; layer < layer_filaments.size(); ++layer)
    for layer in 0..layer_filaments.len() {
        // ToolOrderUtils.cpp:1073  const auto& curr_lf = layer_filaments[layer];
        let curr_lf = &layer_filaments[layer];

        // ToolOrderUtils.cpp:1075  if(auto iter = custom_layer_sequence_map.find(layer); iter != custom_layer_sequence_map.end())
        if let Some(custom_seq) = custom_layer_sequence_map.get(&layer) {
            // ToolOrderUtils.cpp:1076  auto sequence_in_group = collect_filaments_in_groups<unsigned int>(std::unordered_set<unsigned int>(filament_lists...), iter->second);
            let local_sets: HashSet<u32> = filament_lists.iter().copied().collect();
            let sequence_in_group = collect_filaments_in_groups_u32(&local_sets, custom_seq);

            // ToolOrderUtils.cpp:1078  std::optional<unsigned int> prev = curr_filament_id;
            let mut prev = curr_filament_id;
            // ToolOrderUtils.cpp:1079  for (auto& f: sequence_in_group)
            for &f in &sequence_in_group {
                // ToolOrderUtils.cpp:1080  if(prev)
                if let Some(p) = prev {
                    // ToolOrderUtils.cpp:1076  cost += flush_matrix[*prev][f];
                    if pin {
                        int_add_assign_float(&mut cost, flush_matrix[p as usize][f as usize]);
                    } else {
                        cost += flush_matrix[p as usize][f as usize] as i32;
                    }
                }
                // ToolOrderUtils.cpp:1082  prev = f;
                prev = Some(f);
            }

            // ToolOrderUtils.cpp:1085  if(!sequence_in_group.empty())
            if !sequence_in_group.is_empty() {
                // ToolOrderUtils.cpp:1086  curr_filament_id = sequence_in_group.back();
                curr_filament_id = Some(*sequence_in_group.last().unwrap());
            }

            // ToolOrderUtils.cpp:1089  if(filament_sequences)
            if let Some(fs) = filament_sequences.as_deref_mut() {
                // ToolOrderUtils.cpp:1090  filament_sequences->emplace_back(sequence_in_group);
                fs.push(sequence_in_group);
            }

            // ToolOrderUtils.cpp:1092  continue;
            continue;
        }

        // ToolOrderUtils.cpp:1095  std::vector<unsigned int> filament_used = collect_filaments_in_groups<unsigned int>(filament_sets, curr_lf);
        let filament_used = collect_filaments_in_groups_u32(&filament_sets, curr_lf);
        // ToolOrderUtils.cpp:1096  std::vector<unsigned int> next_lf;
        let mut next_lf: Vec<u32> = Vec::new();
        // ToolOrderUtils.cpp:1097  if (layer + 1 < layer_filaments.size()) next_lf = layer_filaments[layer + 1];
        if layer + 1 < layer_filaments.len() {
            next_lf = layer_filaments[layer + 1].clone();
        }
        // ToolOrderUtils.cpp:1098  std::vector<unsigned int> filament_used_next_layer = collect_filaments_in_groups<unsigned int>(filament_sets, next_lf);
        let filament_used_next_layer = collect_filaments_in_groups_u32(&filament_sets, &next_lf);

        // ToolOrderUtils.cpp:1095  bool use_forcast = false;
        // (the earlier source had `filament_used.size() <= max_n_with_forcast &&
        // filament_used_next_layer.size() <= max_n_with_forcast`, the off arm)
        let use_forcast = if pin {
            false
        } else {
            filament_used.len() <= MAX_N_WITH_FORCAST
                && filament_used_next_layer.len() <= MAX_N_WITH_FORCAST
        };
        // ToolOrderUtils.cpp:1101  float tmp_cost = 0;
        let mut tmp_cost: f32 = 0.0;
        // ToolOrderUtils.cpp:1102  std::vector<unsigned int> sequence;
        let sequence: Vec<u32>;
        // ToolOrderUtils.cpp:1103  uint128_t hash_key = filament_list_to_hash_key(filament_used, filament_used_next_layer, curr_filament_id, use_forcast);
        let hash_key = filament_list_to_hash_key(
            &filament_used,
            &filament_used_next_layer,
            curr_filament_id,
            use_forcast,
        );
        // ToolOrderUtils.cpp:1104  if (auto iter = caches.find(hash_key); iter != caches.end())
        if let Some((c, s)) = caches.get(&hash_key) {
            // ToolOrderUtils.cpp:1105  tmp_cost = iter->second.first;
            tmp_cost = *c;
            // ToolOrderUtils.cpp:1106  sequence = iter->second.second;
            sequence = s.clone();
        } else {
            // ToolOrderUtils.cpp:1109  sequence = get_extruders_order(flush_matrix, filament_used, filament_used_next_layer, curr_filament_id, use_forcast, &tmp_cost);
            sequence = get_extruders_order(
                flush_matrix,
                &filament_used,
                &filament_used_next_layer,
                curr_filament_id,
                use_forcast,
                Some(&mut tmp_cost),
            );
            // ToolOrderUtils.cpp:1110  caches[hash_key] = { tmp_cost,sequence };
            caches.insert(hash_key, (tmp_cost, sequence.clone()));
        }

        // ToolOrderUtils.cpp:1113  if (filament_sequences)
        if let Some(fs) = filament_sequences.as_deref_mut() {
            // ToolOrderUtils.cpp:1114  filament_sequences->emplace_back(sequence);
            fs.push(sequence.clone());
        }

        // ToolOrderUtils.cpp:1116  if (!sequence.empty())
        if !sequence.is_empty() {
            // ToolOrderUtils.cpp:1117  curr_filament_id = sequence.back();
            curr_filament_id = Some(*sequence.last().unwrap());
        }

        // ToolOrderUtils.cpp:1114  cost += tmp_cost;
        if pin {
            int_add_assign_float(&mut cost, tmp_cost);
        } else {
            cost += tmp_cost as i32;
        }
    }

    // ToolOrderUtils.cpp:1117  return cost;
    cost
}

// ToolOrderUtils.cpp:1125  int reorder_filaments_for_minimum_flush_volume(...)
pub fn reorder_filaments_for_minimum_flush_volume(
    filament_lists: &[u32],
    filament_maps: &[i32],
    layer_filaments: &[Vec<u32>],
    flush_matrix: &[FlushMatrix],
    get_custom_seq: Option<&dyn Fn(i32, &mut Vec<i32>) -> bool>,
    filament_sequences: Option<&mut Vec<Vec<u32>>>,
    nozzle_status: &HashMap<i32, i32>,
) -> i32 {
    let pin = tool_order_utils_pin();
    // ToolOrderUtils.cpp:1128  //only when layer filament num <= 5,we do forcast
    // ToolOrderUtils.cpp:1129  constexpr int max_n_with_forcast = 5;
    const MAX_N_WITH_FORCAST: usize = 5;
    // ToolOrderUtils.cpp:1135  int cost = 0;
    let mut cost = 0;
    // ToolOrderUtils.cpp:1136  std::vector<std::unordered_set<unsigned int>>groups(2); //save the grouped filaments
    let mut groups: Vec<HashSet<u32>> = vec![HashSet::new(), HashSet::new()];
    // ToolOrderUtils.cpp:1137  std::vector<std::vector<std::vector<unsigned int>>> layer_sequences(2);
    let mut layer_sequences: Vec<Vec<Vec<u32>>> = vec![Vec::new(), Vec::new()];
    // ToolOrderUtils.cpp:1138  std::map<size_t, std::vector<unsigned int>> custom_layer_sequence_map;
    let mut custom_layer_sequence_map: BTreeMap<usize, Vec<u32>> = BTreeMap::new();

    // ToolOrderUtils.cpp:1140  // group the filament
    // ToolOrderUtils.cpp:1141  for (int i = 0; i < filament_maps.size(); ++i)
    for i in 0..filament_maps.len() {
        // ToolOrderUtils.cpp:1142  if (filament_maps[i] == 0)
        if filament_maps[i] == 0 {
            // ToolOrderUtils.cpp:1143  groups[0].insert(filament_lists[i]);
            groups[0].insert(filament_lists[i]);
        }
        // ToolOrderUtils.cpp:1144  if (filament_maps[i] == 1)
        if filament_maps[i] == 1 {
            // ToolOrderUtils.cpp:1145  groups[1].insert(filament_lists[i]);
            groups[1].insert(filament_lists[i]);
        }
    }

    // ToolOrderUtils.cpp:1148  // store custom layer sequence
    // ToolOrderUtils.cpp:1149  for (size_t layer = 0; layer < layer_filaments.size(); ++layer)
    for layer in 0..layer_filaments.len() {
        // ToolOrderUtils.cpp:1150  const auto& curr_lf = layer_filaments[layer];
        let curr_lf = &layer_filaments[layer];

        // ToolOrderUtils.cpp:1152  std::vector<int>custom_filament_seq;
        let mut custom_filament_seq: Vec<i32> = Vec::new();
        // ToolOrderUtils.cpp:1153  if (get_custom_seq && (*get_custom_seq)(layer, custom_filament_seq) && !custom_filament_seq.empty())
        if let Some(gcs) = get_custom_seq {
            if gcs(layer as i32, &mut custom_filament_seq) && !custom_filament_seq.is_empty() {
                // ToolOrderUtils.cpp:1154  std::vector<unsigned int> unsign_custom_extruder_seq;
                let mut unsign_custom_extruder_seq: Vec<u32> = Vec::new();
                // ToolOrderUtils.cpp:1155  for (int extruder : custom_filament_seq)
                for &extruder in &custom_filament_seq {
                    // ToolOrderUtils.cpp:1156  unsigned int unsign_extruder = static_cast<unsigned int>(extruder) - 1;
                    let unsign_extruder = (extruder as u32).wrapping_sub(1);
                    // ToolOrderUtils.cpp:1157  auto it = std::find(curr_lf.begin(), curr_lf.end(), unsign_extruder);
                    // ToolOrderUtils.cpp:1158  if (it != curr_lf.end())
                    if curr_lf.contains(&unsign_extruder) {
                        // ToolOrderUtils.cpp:1159  unsign_custom_extruder_seq.emplace_back(unsign_extruder);
                        unsign_custom_extruder_seq.push(unsign_extruder);
                    }
                }
                // ToolOrderUtils.cpp:1161  assert(curr_lf.size() == unsign_custom_extruder_seq.size());
                debug_assert_eq!(curr_lf.len(), unsign_custom_extruder_seq.len());

                // ToolOrderUtils.cpp:1163  custom_layer_sequence_map[layer] = unsign_custom_extruder_seq;
                custom_layer_sequence_map.insert(layer, unsign_custom_extruder_seq);
            }
        }
    }
    // ToolOrderUtils.cpp:1166  using uint128_t = boost::multiprecision::uint128_t; -> u128
    // ToolOrderUtils.cpp:1167  auto extruders_to_hash_key = [](...)->uint128_t
    let extruders_to_hash_key = |curr_layer_extruders: &[u32],
                                 next_layer_extruders: &[u32],
                                 prev_extruder: Option<u32>,
                                 use_forcast: bool|
     -> u128 {
        // ToolOrderUtils.cpp:1172  uint128_t hash_key = 0;
        let mut hash_key: u128 = 0;
        // ToolOrderUtils.cpp:1174  if (prev_extruder)
        if let Some(pe) = prev_extruder {
            // ToolOrderUtils.cpp:1175  hash_key |= (uint128_t(1) << (64 + *prev_extruder));
            hash_key |= 1u128 << (64 + pe);
        }
        // ToolOrderUtils.cpp:1177  if (use_forcast)
        if use_forcast {
            // ToolOrderUtils.cpp:1178  for (auto item : next_layer_extruders)
            for &item in next_layer_extruders {
                // ToolOrderUtils.cpp:1179  hash_key |= (uint128_t(1) << (32 + item));
                hash_key |= 1u128 << (32 + item);
            }
        }
        // ToolOrderUtils.cpp:1182  for (auto item : curr_layer_extruders)
        for &item in curr_layer_extruders {
            // ToolOrderUtils.cpp:1183  hash_key |= (uint128_t(1) << item);
            hash_key |= 1u128 << item;
        }
        // ToolOrderUtils.cpp:1184  return hash_key;
        hash_key
    };

    // ToolOrderUtils.cpp:1188  // get best layer sequence by group
    // ToolOrderUtils.cpp:1189  for (size_t idx = 0; idx < groups.size(); ++idx)
    for idx in 0..groups.len() {
        // ToolOrderUtils.cpp:1190  // case with one group
        // ToolOrderUtils.cpp:1191  if (groups[idx].empty())
        if groups[idx].is_empty() {
            // ToolOrderUtils.cpp:1192  continue;
            continue;
        }
        // ToolOrderUtils.cpp:1193  std::optional<unsigned int>current_extruder_id;
        let mut current_extruder_id: Option<u32> = None;
        // ToolOrderUtils.cpp:1194  // 尝试从 nozzle_status 获取该 group(nozzle) 的初始 filament
        // ToolOrderUtils.cpp:1195  if (auto it = nozzle_status.find(static_cast<int>(idx)); it != nozzle_status.end() && it->second >= 0)
        if let Some(&v) = nozzle_status.get(&(idx as i32)) {
            if v >= 0 {
                // ToolOrderUtils.cpp:1196  unsigned int initial_fil = static_cast<unsigned int>(it->second);
                let initial_fil = v as u32;
                // ToolOrderUtils.cpp:1197  if (initial_fil < flush_matrix[idx].size())
                if (initial_fil as usize) < flush_matrix[idx].len() {
                    // ToolOrderUtils.cpp:1198  current_extruder_id = initial_fil;
                    current_extruder_id = Some(initial_fil);
                }
            }
        }

        // ToolOrderUtils.cpp:1201  std::unordered_map<uint128_t, std::pair<float, std::vector<unsigned int>>> caches;
        let mut caches: HashMap<u128, (f32, Vec<u32>)> = HashMap::new();

        // ToolOrderUtils.cpp:1203  for (size_t layer = 0; layer < layer_filaments.size(); ++layer)
        for layer in 0..layer_filaments.len() {
            // ToolOrderUtils.cpp:1204  const auto& curr_lf = layer_filaments[layer];
            let curr_lf = &layer_filaments[layer];

            // ToolOrderUtils.cpp:1206  if (auto iter = custom_layer_sequence_map.find(layer); iter != custom_layer_sequence_map.end())
            if let Some(custom_seq) = custom_layer_sequence_map.get(&layer) {
                // ToolOrderUtils.cpp:1207  auto sequence_in_group = collect_filaments_in_groups<unsigned int>(groups[idx], iter->second);
                let sequence_in_group = collect_filaments_in_groups_u32(&groups[idx], custom_seq);

                // ToolOrderUtils.cpp:1209  float tmp_cost = 0;
                let mut tmp_cost: f32 = 0.0;
                // ToolOrderUtils.cpp:1210  std::optional<unsigned int>prev = current_extruder_id;
                let mut prev = current_extruder_id;
                // ToolOrderUtils.cpp:1211  for (auto& f : sequence_in_group)
                for &f in &sequence_in_group {
                    // ToolOrderUtils.cpp:1212  if (prev) { tmp_cost += flush_matrix[idx][*prev][f]; }
                    if let Some(p) = prev {
                        tmp_cost += flush_matrix[idx][p as usize][f as usize];
                    }
                    // ToolOrderUtils.cpp:1213  prev = f;
                    prev = Some(f);
                }
                // ToolOrderUtils.cpp:1210  cost += tmp_cost;
                if pin {
                    int_add_assign_float(&mut cost, tmp_cost);
                } else {
                    cost += tmp_cost as i32;
                }

                // ToolOrderUtils.cpp:1217  if (!sequence_in_group.empty())
                if !sequence_in_group.is_empty() {
                    // ToolOrderUtils.cpp:1218  current_extruder_id = sequence_in_group.back();
                    current_extruder_id = Some(*sequence_in_group.last().unwrap());
                }
                // ToolOrderUtils.cpp:1219  //insert an empty array
                // ToolOrderUtils.cpp:1220  if (filament_sequences)
                if filament_sequences.is_some() {
                    // ToolOrderUtils.cpp:1221  layer_sequences[idx].emplace_back(std::vector<unsigned int>());
                    layer_sequences[idx].push(Vec::new());
                }

                // ToolOrderUtils.cpp:1223  continue;
                continue;
            }

            // ToolOrderUtils.cpp:1226  std::vector<unsigned int>filament_used_in_group = collect_filaments_in_groups<unsigned int>(groups[idx], curr_lf);
            let filament_used_in_group = collect_filaments_in_groups_u32(&groups[idx], curr_lf);

            // ToolOrderUtils.cpp:1228  std::vector<unsigned int>next_lf;
            let mut next_lf: Vec<u32> = Vec::new();
            // ToolOrderUtils.cpp:1229  if (layer + 1 < layer_filaments.size())
            if layer + 1 < layer_filaments.len() {
                // ToolOrderUtils.cpp:1230  next_lf = layer_filaments[layer + 1];
                next_lf = layer_filaments[layer + 1].clone();
            }
            // ToolOrderUtils.cpp:1231  std::vector<unsigned int>filament_used_in_group_next_layer = collect_filaments_in_groups<unsigned int>(groups[idx], next_lf);
            let filament_used_in_group_next_layer =
                collect_filaments_in_groups_u32(&groups[idx], &next_lf);

            // ToolOrderUtils.cpp:1233  bool use_forcast = (filament_used_in_group.size() <= max_n_with_forcast && filament_used_in_group_next_layer.size() <= max_n_with_forcast);
            let use_forcast = filament_used_in_group.len() <= MAX_N_WITH_FORCAST
                && filament_used_in_group_next_layer.len() <= MAX_N_WITH_FORCAST;
            // ToolOrderUtils.cpp:1234  float tmp_cost = 0;
            let mut tmp_cost: f32 = 0.0;
            // ToolOrderUtils.cpp:1235  std::vector<unsigned int>sequence;
            let sequence: Vec<u32>;
            // ToolOrderUtils.cpp:1236  uint128_t hash_key = extruders_to_hash_key(filament_used_in_group, filament_used_in_group_next_layer, current_extruder_id, use_forcast);
            let hash_key = extruders_to_hash_key(
                &filament_used_in_group,
                &filament_used_in_group_next_layer,
                current_extruder_id,
                use_forcast,
            );
            // ToolOrderUtils.cpp:1237  if (auto iter = caches.find(hash_key); iter != caches.end())
            if let Some((c, s)) = caches.get(&hash_key) {
                // ToolOrderUtils.cpp:1238  tmp_cost = iter->second.first;
                tmp_cost = *c;
                // ToolOrderUtils.cpp:1239  sequence = iter->second.second;
                sequence = s.clone();
            } else {
                // ToolOrderUtils.cpp:1242  sequence = get_extruders_order(flush_matrix[idx], filament_used_in_group, filament_used_in_group_next_layer, current_extruder_id, use_forcast, &tmp_cost);
                sequence = get_extruders_order(
                    &flush_matrix[idx],
                    &filament_used_in_group,
                    &filament_used_in_group_next_layer,
                    current_extruder_id,
                    use_forcast,
                    Some(&mut tmp_cost),
                );
                // ToolOrderUtils.cpp:1243  caches[hash_key] = { tmp_cost,sequence };
                caches.insert(hash_key, (tmp_cost, sequence.clone()));
            }

            // ToolOrderUtils.cpp:1246  assert(sequence.size() == filament_used_in_group.size());
            debug_assert_eq!(sequence.len(), filament_used_in_group.len());

            // ToolOrderUtils.cpp:1248  if (filament_sequences)
            if filament_sequences.is_some() {
                // ToolOrderUtils.cpp:1249  layer_sequences[idx].emplace_back(sequence);
                layer_sequences[idx].push(sequence.clone());
            }

            // ToolOrderUtils.cpp:1251  if (!sequence.empty())
            if !sequence.is_empty() {
                // ToolOrderUtils.cpp:1252  current_extruder_id = sequence.back();
                current_extruder_id = Some(*sequence.last().unwrap());
            }
            // ToolOrderUtils.cpp:1248  cost += tmp_cost;
            if pin {
                int_add_assign_float(&mut cost, tmp_cost);
            } else {
                cost += tmp_cost as i32;
            }
        }
    }

    // ToolOrderUtils.cpp:1257  // get the final layer sequences
    // ToolOrderUtils.cpp:1258  // if only have one group,we need to check whether layer sequence[idx] is valid
    // ToolOrderUtils.cpp:1259  if (filament_sequences)
    if let Some(fs) = filament_sequences {
        // ToolOrderUtils.cpp:1260  filament_sequences->clear();
        fs.clear();
        // ToolOrderUtils.cpp:1261  filament_sequences->resize(layer_filaments.size());
        fs.resize(layer_filaments.len(), Vec::new());
        // ToolOrderUtils.cpp:1262  int last_group_id = 0;
        let mut last_group_id = 0;
        // ToolOrderUtils.cpp:1263  //if last_group == 0,print group 0 first ,else print group 1 first
        // ToolOrderUtils.cpp:1264  if (!custom_layer_sequence_map.empty())
        if !custom_layer_sequence_map.is_empty() {
            // ToolOrderUtils.cpp:1265  const auto& first_layer = custom_layer_sequence_map.begin()->first;
            let (first_layer, first_layer_filaments) =
                custom_layer_sequence_map.iter().next().unwrap();
            // ToolOrderUtils.cpp:1267  assert(!first_layer_filaments.empty());
            debug_assert!(!first_layer_filaments.is_empty());

            // ToolOrderUtils.cpp:1269  bool first_group = groups[0].count(first_layer_filaments.front()) ? 0 : 1;
            let first_group: i32 = if groups[0].contains(&first_layer_filaments[0]) { 0 } else { 1 };
            // ToolOrderUtils.cpp:1270  last_group_id = (first_layer & 1) ? !first_group : first_group;
            last_group_id = if first_layer & 1 != 0 {
                // C++ !int: 0 -> 1, nonzero -> 0
                if first_group != 0 { 0 } else { 1 }
            } else {
                first_group
            };
        }

        // ToolOrderUtils.cpp:1273  for (size_t layer = 0; layer < layer_filaments.size(); ++layer)
        for layer in 0..layer_filaments.len() {
            // ToolOrderUtils.cpp:1274  auto& curr_layer_seq = (*filament_sequences)[layer];
            // ToolOrderUtils.cpp:1275  if (custom_layer_sequence_map.find(layer) != custom_layer_sequence_map.end())
            if let Some(custom_seq) = custom_layer_sequence_map.get(&layer) {
                // ToolOrderUtils.cpp:1276  curr_layer_seq = custom_layer_sequence_map[layer];
                fs[layer] = custom_seq.clone();
                // ToolOrderUtils.cpp:1277  if (!curr_layer_seq.empty())
                if !fs[layer].is_empty() {
                    // ToolOrderUtils.cpp:1278  last_group_id = groups[0].count(curr_layer_seq.back()) ? 0 : 1;
                    last_group_id =
                        if groups[0].contains(fs[layer].last().unwrap()) { 0 } else { 1 };
                }
                // ToolOrderUtils.cpp:1280  continue;
                continue;
            }
            // ToolOrderUtils.cpp:1282  if (last_group_id == 1)
            if last_group_id == 1 {
                // ToolOrderUtils.cpp:1283  // try reuse the last group
                // ToolOrderUtils.cpp:1284  if (!layer_sequences[1].empty() && !layer_sequences[1][layer].empty())
                if !layer_sequences[1].is_empty() && !layer_sequences[1][layer].is_empty() {
                    // ToolOrderUtils.cpp:1285  curr_layer_seq.insert(curr_layer_seq.end(), layer_sequences[1][layer]...);
                    fs[layer].extend_from_slice(&layer_sequences[1][layer]);
                }
                // ToolOrderUtils.cpp:1286  if (!layer_sequences[0].empty() && !layer_sequences[0][layer].empty())
                if !layer_sequences[0].is_empty() && !layer_sequences[0][layer].is_empty() {
                    // ToolOrderUtils.cpp:1287  curr_layer_seq.insert(curr_layer_seq.end(), layer_sequences[0][layer]...);
                    fs[layer].extend_from_slice(&layer_sequences[0][layer]);
                    // ToolOrderUtils.cpp:1288  last_group_id = 0; // update last group id
                    last_group_id = 0;
                }
            } else if last_group_id == 0 {
                // ToolOrderUtils.cpp:1291  else if(last_group_id == 0)
                // ToolOrderUtils.cpp:1292  if (!layer_sequences[0].empty() && !layer_sequences[0][layer].empty())
                if !layer_sequences[0].is_empty() && !layer_sequences[0][layer].is_empty() {
                    // ToolOrderUtils.cpp:1293  curr_layer_seq.insert(curr_layer_seq.end(), layer_sequences[0][layer]...);
                    fs[layer].extend_from_slice(&layer_sequences[0][layer]);
                }
                // ToolOrderUtils.cpp:1295  if (!layer_sequences[1].empty() && !layer_sequences[1][layer].empty())
                if !layer_sequences[1].is_empty() && !layer_sequences[1][layer].is_empty() {
                    // ToolOrderUtils.cpp:1296  curr_layer_seq.insert(curr_layer_seq.end(), layer_sequences[1][layer]...);
                    fs[layer].extend_from_slice(&layer_sequences[1][layer]);
                    // ToolOrderUtils.cpp:1297  last_group_id = 1; // update last group id
                    last_group_id = 1;
                }
            }
        }
    }

    // ToolOrderUtils.cpp:1303  return cost;
    cost
}

// ToolOrderUtils.cpp:1299-1544  #if DEBUG_MULTI_NOZZLE_MCMF ... #endif
//
// ToolOrderUtils.hpp:13 defines DEBUG_MULTI_NOZZLE_MCMF as 0, so the C++ build
// leaves these functions out and the engine never calls them; they are ported
// here, in a module named after the switch, so the whole file has a Rust
// counterpart. The names the hpp declares inside the `#if` (:195-213) are
// `pub`; the `static` ones stay private to the module.
pub mod debug_multi_nozzle_mcmf {
    use super::{
        collect_filaments_in_groups, reorder_filaments_for_minimum_flush_volume, FlushMatrix, GeneralMinCostSolver,
    };
    use std::collections::{BTreeMap, HashMap, HashSet};

    // ToolOrderUtils.cpp:1302  //solve the problem by min cost max flow
    // ToolOrderUtils.cpp:1303  static std::vector<std::vector<int>> solve_extruder_order_with_MCMF(...)
    fn solve_extruder_order_with_mcmf(
        wipe_volumes: &[Vec<f32>],
        curr_layer_filaments: &[i32],
        nozzle_state: &[i32],
        min_cost: Option<&mut f32>,
    ) -> Vec<Vec<i32>> {
        // ToolOrderUtils.cpp:1308  std::vector<std::vector<int>>ret;
        let mut ret: Vec<Vec<i32>> = Vec::new();
        // ToolOrderUtils.cpp:1309  std::vector<int>l_nodes = nozzle_state;
        let mut l_nodes: Vec<i32> = nozzle_state.to_vec();
        // ToolOrderUtils.cpp:1310  std::vector<int>r_nodes = curr_layer_filaments;
        let mut r_nodes: Vec<i32> = curr_layer_filaments.to_vec();
        // ToolOrderUtils.cpp:1311  std::sort(r_nodes.begin(), r_nodes.end());
        //   (ints: equal elements are indistinguishable, so any sort gives the same vector)
        r_nodes.sort_unstable();

        // ToolOrderUtils.cpp:1313  int nozzle_num = nozzle_state.size();
        let nozzle_num = nozzle_state.len() as i32;
        // ToolOrderUtils.cpp:1314  int filament_num = curr_layer_filaments.size();
        let filament_num = curr_layer_filaments.len() as i32;
        // ToolOrderUtils.cpp:1315  int epochs = std::ceil(filament_num / (double)(nozzle_num));
        let epochs = (filament_num as f64 / nozzle_num as f64).ceil() as i32;
        // ToolOrderUtils.cpp:1316  for (int i = 0; i < epochs; ++i) {
        for _i in 0..epochs {
            // ToolOrderUtils.cpp:1317  GeneralMinCostSolver s(wipe_volumes, l_nodes, r_nodes);
            let mut s = GeneralMinCostSolver::new(wipe_volumes, &l_nodes, &r_nodes);
            // ToolOrderUtils.cpp:1318  auto match = s.solve();
            let r#match = s.solve();
            // ToolOrderUtils.cpp:1319  ret.emplace_back(match);
            ret.push(r#match.clone());
            // ToolOrderUtils.cpp:1320  l_nodes = match;
            l_nodes = r#match.clone();

            // ToolOrderUtils.cpp:1322-1324  auto remove_iter = std::remove_if(r_nodes..., [match](auto item) { return std::find(match..., item) != match.end(); });
            // ToolOrderUtils.cpp:1325  r_nodes.erase(remove_iter, r_nodes.end());
            r_nodes.retain(|item| !r#match.contains(item));
        }

        // ToolOrderUtils.cpp:1328  if (min_cost) {
        if let Some(min_cost) = min_cost {
            // ToolOrderUtils.cpp:1329  float cost = 0;
            let mut cost: f32 = 0.0;
            // ToolOrderUtils.cpp:1330  std::vector<int>nozzle_filament = nozzle_state;
            let mut nozzle_filament: Vec<i32> = nozzle_state.to_vec();
            // ToolOrderUtils.cpp:1331  for (int seq = 0; seq < ret.size(); ++seq) {
            for seq in 0..ret.len() {
                // ToolOrderUtils.cpp:1332  for (int id = 0; id < ret[seq].size(); ++id) {
                for id in 0..ret[seq].len() {
                    // ToolOrderUtils.cpp:1333  if (ret[seq][id] != -1) {
                    if ret[seq][id] != -1 {
                        // ToolOrderUtils.cpp:1334-1335  if (nozzle_filament[id] != -1) cost += wipe_volumes[nozzle_filament[id]][ret[seq][id]];
                        if nozzle_filament[id] != -1 {
                            cost += wipe_volumes[nozzle_filament[id] as usize][ret[seq][id] as usize];
                        }
                        // ToolOrderUtils.cpp:1336  nozzle_filament[id] = ret[seq][id];
                        nozzle_filament[id] = ret[seq][id];
                    }
                }
            }
            // ToolOrderUtils.cpp:1340  *min_cost = cost;
            *min_cost = cost;
        }

        // ToolOrderUtils.cpp:1343  return ret;
        ret
    }

    // ToolOrderUtils.cpp:1346  // get best filament order of multiple nozzle
    // ToolOrderUtils.cpp:1347  std::vector<std::vector<int>> get_extruders_order(...)
    pub fn get_extruders_order(
        wipe_volumes: &[Vec<f32>],
        curr_layer_filaments: &[i32],
        nozzle_state: &[i32],
        min_cost: Option<&mut f32>,
    ) -> Vec<Vec<i32>> {
        // ToolOrderUtils.cpp:1352  return solve_extruder_order_with_MCMF(wipe_volumes, curr_layer_filaments, nozzle_state, min_cost);
        solve_extruder_order_with_mcmf(wipe_volumes, curr_layer_filaments, nozzle_state, min_cost)
    }

    // ToolOrderUtils.cpp:1356  // get nozzle match data when user set the filament sequence
    // ToolOrderUtils.cpp:1357  static std::vector<std::vector<int>> get_nozzle_match_in_given_order(...)
    fn get_nozzle_match_in_given_order(
        wipe_volumes: &[Vec<f32>],
        layer_filaments_sequence: &[u32],
        nozzle_state: &[i32],
        min_cost: Option<&mut f32>,
    ) -> Vec<Vec<i32>> {
        // ToolOrderUtils.cpp:1362  std::vector<std::vector<int>> ret;
        let mut ret: Vec<Vec<i32>> = Vec::new();
        // ToolOrderUtils.cpp:1363  std::vector<int>state = nozzle_state;
        let mut state: Vec<i32> = nozzle_state.to_vec();
        // ToolOrderUtils.cpp:1364  std::vector<int>filament_nozzles; //nozzles where each filament is placed
        let mut filament_nozzles: Vec<i32> = Vec::new();

        // ToolOrderUtils.cpp:1366  float cost = 0;
        let mut cost: f32 = 0.0;

        // ToolOrderUtils.cpp:1368  for (auto f : layer_filaments_sequence) {
        for &f in layer_filaments_sequence {
            // ToolOrderUtils.cpp:1369  float min_flush = std::numeric_limits<float>::max();
            let mut min_flush = f32::MAX;
            // ToolOrderUtils.cpp:1370  int min_flush_idx = -1;
            let mut min_flush_idx: i32 = -1;
            // ToolOrderUtils.cpp:1371  for (size_t idx = 0; idx < state.size(); ++idx) {
            for idx in 0..state.len() {
                // ToolOrderUtils.cpp:1372  float flush = (state[idx] == -1 ? 0 : wipe_volumes[state[idx]][f]);
                let flush: f32 = if state[idx] == -1 { 0.0 } else { wipe_volumes[state[idx] as usize][f as usize] };
                // ToolOrderUtils.cpp:1373  if (flush < min_flush) {
                if flush < min_flush {
                    // ToolOrderUtils.cpp:1374  min_flush = flush;
                    min_flush = flush;
                    // ToolOrderUtils.cpp:1375  min_flush_idx = idx;
                    min_flush_idx = idx as i32;
                }
            }
            // ToolOrderUtils.cpp:1378  cost += min_flush;
            cost += min_flush;
            // ToolOrderUtils.cpp:1379  state[min_flush_idx] = f;
            state[min_flush_idx as usize] = f as i32;
            // ToolOrderUtils.cpp:1380  filament_nozzles.emplace_back(min_flush_idx);
            filament_nozzles.push(min_flush_idx);
        }

        // ToolOrderUtils.cpp:1383  assert(filament_nozzles.size() == layer_filaments_sequence.size());
        debug_assert_eq!(filament_nozzles.len(), layer_filaments_sequence.len());
        // ToolOrderUtils.cpp:1384  int prev = -1;
        let mut prev: i32 = -1;
        // ToolOrderUtils.cpp:1385  for (int idx = 0; idx < filament_nozzles.size(); ++idx) {
        for idx in 0..filament_nozzles.len() {
            // ToolOrderUtils.cpp:1386-1388  if (prev == -1 || filament_nozzles[idx] <= prev) ret.emplace_back(std::vector<int>(nozzle_state.size(), -1));
            if prev == -1 || filament_nozzles[idx] <= prev {
                ret.push(vec![-1; nozzle_state.len()]);
            }
            // ToolOrderUtils.cpp:1389  ret.back()[filament_nozzles[idx]] = layer_filaments_sequence[idx];
            ret.last_mut().unwrap()[filament_nozzles[idx] as usize] = layer_filaments_sequence[idx] as i32;
            // ToolOrderUtils.cpp:1390  prev = filament_nozzles[idx];
            prev = filament_nozzles[idx];
        }

        // ToolOrderUtils.cpp:1393-1394  if (min_cost) *min_cost = cost;
        if let Some(min_cost) = min_cost {
            *min_cost = cost;
        }

        // ToolOrderUtils.cpp:1396  return ret;
        ret
    }

    // ToolOrderUtils.cpp:1398  int reorder_filaments_for_multi_nozzle_extruder(const std::vector<unsigned int>& filament_lists,
    //                              const std::vector<int>& filament_maps, ..., int multi_nozzle_extruder_id,
    //                              int multi_nozzle_num, layer_sequences = nullptr, nozzle_match_per_layer = nullptr)
    #[allow(clippy::too_many_arguments)]
    pub fn reorder_filaments_for_multi_nozzle_extruder(
        filament_lists: &[u32],
        filament_maps: &[i32],
        layer_filaments: &[Vec<u32>],
        flush_matrix: &[FlushMatrix],
        get_custom_seq: Option<&dyn Fn(i32, &mut Vec<i32>) -> bool>,
        multi_nozzle_extruder_id: i32,
        multi_nozzle_num: i32,
        layer_sequences: Option<&mut Vec<Vec<u32>>>,
        nozzle_match_per_layer: Option<&mut Vec<Vec<Vec<i32>>>>,
    ) -> i32 {
        // ToolOrderUtils.cpp:1408  assert(0 <= multi_nozzle_extruder_id && multi_nozzle_extruder_id <= 1);
        debug_assert!(0 <= multi_nozzle_extruder_id && multi_nozzle_extruder_id <= 1);

        // ToolOrderUtils.cpp:1410  int fixed_nozzle_extruder_id = 1 - multi_nozzle_extruder_id;
        let fixed_nozzle_extruder_id = 1 - multi_nozzle_extruder_id;

        // ToolOrderUtils.cpp:1412  std::vector<std::unordered_set<unsigned int>>groups(2); //save the grouped filaments
        let mut groups: Vec<HashSet<u32>> = vec![HashSet::new(), HashSet::new()];
        // ToolOrderUtils.cpp:1413  std::map<size_t, std::vector<unsigned int>>custom_layer_sequence_map;
        let mut custom_layer_sequence_map: BTreeMap<usize, Vec<u32>> = BTreeMap::new();

        // ToolOrderUtils.cpp:1415  for (int i = 0; i < filament_maps.size(); ++i) {
        for i in 0..filament_maps.len() {
            // ToolOrderUtils.cpp:1416-1417  if (filament_maps[i] == 0) groups[0].insert(filament_lists[i]);
            if filament_maps[i] == 0 {
                groups[0].insert(filament_lists[i]);
            }
            // ToolOrderUtils.cpp:1418-1419  if (filament_maps[i] == 1) groups[1].insert(filament_lists[i]);
            if filament_maps[i] == 1 {
                groups[1].insert(filament_lists[i]);
            }
        }

        // ToolOrderUtils.cpp:1422  // store custom layer sequence
        // ToolOrderUtils.cpp:1423  for (size_t layer = 0; layer < layer_filaments.size(); ++layer) {
        for layer in 0..layer_filaments.len() {
            // ToolOrderUtils.cpp:1424  const auto& curr_lf = layer_filaments[layer];
            let curr_lf = &layer_filaments[layer];
            // ToolOrderUtils.cpp:1425  std::vector<int>custom_filament_seq;
            let mut custom_filament_seq: Vec<i32> = Vec::new();
            // ToolOrderUtils.cpp:1426  if (get_custom_seq && (*get_custom_seq)(layer, custom_filament_seq) && !custom_filament_seq.empty()) {
            if let Some(gcs) = get_custom_seq {
                if gcs(layer as i32, &mut custom_filament_seq) && !custom_filament_seq.is_empty() {
                    // ToolOrderUtils.cpp:1427  std::vector<unsigned int> unsign_custom_extruder_seq;
                    let mut unsign_custom_extruder_seq: Vec<u32> = Vec::new();
                    // ToolOrderUtils.cpp:1428  for (int extruder : custom_filament_seq) {
                    for &extruder in &custom_filament_seq {
                        // ToolOrderUtils.cpp:1429  unsigned int unsign_extruder = static_cast<unsigned int>(extruder) - 1;
                        let unsign_extruder = (extruder as u32).wrapping_sub(1);
                        // ToolOrderUtils.cpp:1430-1432  if (std::find(curr_lf..., unsign_extruder) != curr_lf.end()) emplace_back
                        if curr_lf.contains(&unsign_extruder) {
                            unsign_custom_extruder_seq.push(unsign_extruder);
                        }
                    }
                    // ToolOrderUtils.cpp:1434  assert(curr_lf.size() == unsign_custom_extruder_seq.size());
                    debug_assert_eq!(curr_lf.len(), unsign_custom_extruder_seq.len());
                    // ToolOrderUtils.cpp:1435  custom_layer_sequence_map[layer] = unsign_custom_extruder_seq;
                    custom_layer_sequence_map.insert(layer, unsign_custom_extruder_seq);
                }
            }
        }

        // ToolOrderUtils.cpp:1439  float cost = 0;
        let mut cost: f32 = 0.0;
        // ToolOrderUtils.cpp:1440  std::vector<std::vector<std::vector<int>>>nozzle_match_info;
        let mut nozzle_match_info: Vec<Vec<Vec<i32>>> = Vec::new();
        // ToolOrderUtils.cpp:1441  std::vector<std::vector<std::vector<unsigned int>>>temp_layer_sequence(2);
        let mut temp_layer_sequence: Vec<Vec<Vec<u32>>> = vec![Vec::new(), Vec::new()];
        let want_layer_sequences = layer_sequences.is_some();

        // ToolOrderUtils.cpp:1443  // first deal with fixed nozzle
        // ToolOrderUtils.cpp:1444  if (!groups[fixed_nozzle_extruder_id].empty())
        if !groups[fixed_nozzle_extruder_id as usize].is_empty() {
            // ToolOrderUtils.cpp:1446  int idx = fixed_nozzle_extruder_id;
            let idx = fixed_nozzle_extruder_id as usize;
            // ToolOrderUtils.cpp:1447  std::vector<unsigned int> filament_used_in_group(groups[idx].begin(), groups[idx].end());
            let filament_used_in_group: Vec<u32> = groups[idx].iter().copied().collect();
            // ToolOrderUtils.cpp:1448  std::vector<int>filament_map_in_group(groups[idx].size(), idx);
            let filament_map_in_group: Vec<i32> = vec![idx as i32; groups[idx].len()];
            // ToolOrderUtils.cpp:1449  cost += reorder_filaments_for_minimum_flush_volume(filament_used_in_group, filament_map_in_group, layer_filaments, flush_matrix, get_custom_seq, layer_sequences ? &temp_layer_sequence[idx] : nullptr);
            //   (float += int: the int result is converted to float)
            cost += reorder_filaments_for_minimum_flush_volume(
                &filament_used_in_group,
                &filament_map_in_group,
                layer_filaments,
                flush_matrix,
                get_custom_seq,
                if want_layer_sequences { Some(&mut temp_layer_sequence[idx]) } else { None },
                &HashMap::new(),
            ) as f32;
        }

        // ToolOrderUtils.cpp:1452  //then deal with multiple nozzle
        // ToolOrderUtils.cpp:1453  if (!groups[multi_nozzle_extruder_id].empty())
        if !groups[multi_nozzle_extruder_id as usize].is_empty() {
            // ToolOrderUtils.cpp:1455  int idx = multi_nozzle_extruder_id;
            let idx = multi_nozzle_extruder_id as usize;
            // ToolOrderUtils.cpp:1456-1457  if (layer_sequences) temp_layer_sequence[idx].reserve(layer_filaments.size());
            if want_layer_sequences {
                temp_layer_sequence[idx].reserve(layer_filaments.len());
            }
            // ToolOrderUtils.cpp:1458-1459  if (nozzle_match_per_layer) nozzle_match_info.reserve(layer_filaments.size());
            if nozzle_match_per_layer.is_some() {
                nozzle_match_info.reserve(layer_filaments.len());
            }

            // ToolOrderUtils.cpp:1461  std::vector<int> nozzle_state(multi_nozzle_num, -1);
            let mut nozzle_state: Vec<i32> = vec![-1; multi_nozzle_num as usize];

            // ToolOrderUtils.cpp:1463  auto update_nozzle_state = [&nozzle_state](const std::vector<std::vector<int>>& match_info) {
            let update_nozzle_state = |nozzle_state: &mut Vec<i32>, match_info: &Vec<Vec<i32>>| {
                // ToolOrderUtils.cpp:1464  for (size_t i = 0; i < match_info.size(); ++i) {
                for i in 0..match_info.len() {
                    // ToolOrderUtils.cpp:1465  for (size_t j = 0; j < match_info[i].size(); ++j)
                    for j in 0..match_info[i].len() {
                        // ToolOrderUtils.cpp:1466-1467  if (match_info[i][j] != -1) nozzle_state[j] = match_info[i][j];
                        if match_info[i][j] != -1 {
                            nozzle_state[j] = match_info[i][j];
                        }
                    }
                }
            };

            // ToolOrderUtils.cpp:1471  for (size_t layer = 0; layer < layer_filaments.size(); ++layer) {
            for layer in 0..layer_filaments.len() {
                // ToolOrderUtils.cpp:1472  if (auto iter = custom_layer_sequence_map.find(layer); iter != custom_layer_sequence_map.end()) {
                if let Some(sequence) = custom_layer_sequence_map.get(&layer) {
                    // ToolOrderUtils.cpp:1473  const auto& sequence = iter->second;
                    // ToolOrderUtils.cpp:1474  std::vector<unsigned int>sequence_in_group = collect_filaments_in_groups<unsigned int>(groups[idx], sequence);
                    let sequence_in_group: Vec<u32> = collect_filaments_in_groups::<u32>(&groups[idx], sequence);

                    // ToolOrderUtils.cpp:1476  float tmp_cost = 0;
                    let mut tmp_cost: f32 = 0.0;
                    // ToolOrderUtils.cpp:1477  auto nozzle_match = get_nozzle_match_in_given_order(flush_matrix[idx], sequence_in_group, nozzle_state, &tmp_cost);
                    let nozzle_match = get_nozzle_match_in_given_order(
                        &flush_matrix[idx],
                        &sequence_in_group,
                        &nozzle_state,
                        Some(&mut tmp_cost),
                    );
                    // ToolOrderUtils.cpp:1478  cost += tmp_cost;
                    cost += tmp_cost;
                    // ToolOrderUtils.cpp:1479  update_nozzle_state(nozzle_match);
                    update_nozzle_state(&mut nozzle_state, &nozzle_match);

                    // ToolOrderUtils.cpp:1481-1482  if (layer_sequences) temp_layer_sequence[idx].emplace_back(std::vector<unsigned int>()); // insert an empty array as placeholder
                    if want_layer_sequences {
                        temp_layer_sequence[idx].push(Vec::new());
                    }
                    // ToolOrderUtils.cpp:1483-1484  if (nozzle_match_per_layer) nozzle_match_info.emplace_back(nozzle_match);
                    if nozzle_match_per_layer.is_some() {
                        nozzle_match_info.push(nozzle_match);
                    }
                    // ToolOrderUtils.cpp:1485  continue;
                    continue;
                }

                // ToolOrderUtils.cpp:1489  const auto& curr_lf = layer_filaments[layer];
                let curr_lf = &layer_filaments[layer];
                // ToolOrderUtils.cpp:1490  std::vector<int>filament_used_in_group = collect_filaments_in_groups<int>(groups[idx], curr_lf);
                let filament_used_in_group: Vec<i32> = collect_filaments_in_groups::<i32>(&groups[idx], curr_lf);

                // ToolOrderUtils.cpp:1492  float tmp_cost = 0;
                let mut tmp_cost: f32 = 0.0;
                // ToolOrderUtils.cpp:1493  auto ret = solve_extruder_order_with_MCMF(flush_matrix[idx], filament_used_in_group, nozzle_state, &tmp_cost);
                let ret = solve_extruder_order_with_mcmf(
                    &flush_matrix[idx],
                    &filament_used_in_group,
                    &nozzle_state,
                    Some(&mut tmp_cost),
                );
                // ToolOrderUtils.cpp:1494  cost += tmp_cost;
                cost += tmp_cost;
                // ToolOrderUtils.cpp:1495  update_nozzle_state(ret);
                update_nozzle_state(&mut nozzle_state, &ret);

                // ToolOrderUtils.cpp:1497  if (layer_sequences) {
                if want_layer_sequences {
                    // ToolOrderUtils.cpp:1498  std::vector<unsigned int> flatten;
                    let mut flatten: Vec<u32> = Vec::new();
                    // ToolOrderUtils.cpp:1499  for (size_t i = 0; i < ret.size(); ++i) {
                    for i in 0..ret.len() {
                        // ToolOrderUtils.cpp:1500  for (size_t j = 0; j < ret[i].size(); ++j)
                        for j in 0..ret[i].len() {
                            // ToolOrderUtils.cpp:1501-1502  if (ret[i][j] != -1) flatten.emplace_back(ret[i][j]);
                            if ret[i][j] != -1 {
                                flatten.push(ret[i][j] as u32);
                            }
                        }
                    }
                    // ToolOrderUtils.cpp:1504  temp_layer_sequence[idx].emplace_back(flatten);
                    temp_layer_sequence[idx].push(flatten);
                }
                // ToolOrderUtils.cpp:1506-1507  if (nozzle_match_per_layer) nozzle_match_info.emplace_back(ret);
                if nozzle_match_per_layer.is_some() {
                    nozzle_match_info.push(ret);
                }
            }
        }

        // ToolOrderUtils.cpp:1511  // save the final sequence and match info if necessary
        // ToolOrderUtils.cpp:1512  if (layer_sequences) {
        if let Some(layer_sequences) = layer_sequences {
            // ToolOrderUtils.cpp:1513  layer_sequences->clear();
            layer_sequences.clear();
            // ToolOrderUtils.cpp:1514  layer_sequences->resize(layer_filaments.size());
            layer_sequences.resize(layer_filaments.len(), Vec::new());
            // ToolOrderUtils.cpp:1515  bool last_group = 0;
            let mut last_group = false;
            // ToolOrderUtils.cpp:1516  //if last_group == 0,print group 0 first ,else print group 1 first
            // ToolOrderUtils.cpp:1517  for (size_t i = 0; i < layer_filaments.size(); ++i) {
            for i in 0..layer_filaments.len() {
                // ToolOrderUtils.cpp:1518  auto& curr_layer_seq = (*layer_sequences)[i];
                // ToolOrderUtils.cpp:1519  if (auto iter = custom_layer_sequence_map.find(i); iter != custom_layer_sequence_map.end()) {
                if let Some(seq) = custom_layer_sequence_map.get(&i) {
                    // ToolOrderUtils.cpp:1520  curr_layer_seq = iter->second;
                    layer_sequences[i] = seq.clone();
                    // ToolOrderUtils.cpp:1521-1523  if (!curr_layer_seq.empty()) last_group = groups[0].count(curr_layer_seq.back()) ? 0 : 1;
                    if let Some(back) = layer_sequences[i].last() {
                        last_group = !groups[0].contains(back);
                    }
                    // ToolOrderUtils.cpp:1524  continue;
                    continue;
                }
                // ToolOrderUtils.cpp:1526  if (last_group) {
                //   (temp_layer_sequence[g][i] of a group without filaments reads an
                //   empty vector out of bounds in C++; Rust stops there)
                if last_group {
                    // ToolOrderUtils.cpp:1527  curr_layer_seq.insert(curr_layer_seq.end(), temp_layer_sequence[1][i]...);
                    let s1 = temp_layer_sequence[1][i].clone();
                    layer_sequences[i].extend_from_slice(&s1);
                    // ToolOrderUtils.cpp:1528  curr_layer_seq.insert(curr_layer_seq.end(), temp_layer_sequence[0][i]...);
                    let s0 = temp_layer_sequence[0][i].clone();
                    layer_sequences[i].extend_from_slice(&s0);
                } else {
                    // ToolOrderUtils.cpp:1531  curr_layer_seq.insert(curr_layer_seq.end(), temp_layer_sequence[0][i]...);
                    let s0 = temp_layer_sequence[0][i].clone();
                    layer_sequences[i].extend_from_slice(&s0);
                    // ToolOrderUtils.cpp:1532  curr_layer_seq.insert(curr_layer_seq.end(), temp_layer_sequence[1][i]...);
                    let s1 = temp_layer_sequence[1][i].clone();
                    layer_sequences[i].extend_from_slice(&s1);
                }
                // ToolOrderUtils.cpp:1534  last_group = !last_group;
                last_group = !last_group;
            }
        }

        // ToolOrderUtils.cpp:1538-1539  if (nozzle_match_per_layer) *nozzle_match_per_layer = nozzle_match_info;
        if let Some(nozzle_match_per_layer) = nozzle_match_per_layer {
            *nozzle_match_per_layer = nozzle_match_info;
        }

        // ToolOrderUtils.cpp:1541  return cost;
        //   (float to the int return type: truncation)
        cost as i32
    }
}

// ToolOrderUtils.cpp:1546  int reorder_filaments_for_multi_nozzle_extruder(...)
//   (the overload taking MultiNozzleUtils::LayeredNozzleGroupResult; outside the #if)
pub fn reorder_filaments_for_multi_nozzle_extruder(
    filament_lists: &[u32],
    nozzle_group_result: &LayeredNozzleGroupResult,
    layer_filaments: &[Vec<u32>],
    flush_matrix: &[FlushMatrix],
    get_custom_seq: Option<&dyn Fn(i32, &mut Vec<i32>) -> bool>,
    filament_sequences: Option<&mut Vec<Vec<u32>>>,
    initial_status: &NozzleStatusRecorder,
) -> i32 {
    if !tool_order_utils_pin() {
        return reorder_filaments_for_multi_nozzle_extruder_pre_pin(
            filament_lists,
            nozzle_group_result,
            layer_filaments,
            flush_matrix,
            get_custom_seq,
            filament_sequences,
            initial_status,
        );
    }
    // ToolOrderUtils.cpp:1554  std::map<int,std::set<unsigned int>> nozzle_filament_groups;
    let mut nozzle_filament_groups: BTreeMap<i32, BTreeSet<u32>> = BTreeMap::new();
    // ToolOrderUtils.cpp:1555  std::map<int,std::set<int>> extruder_to_nozzle;
    let mut extruder_to_nozzle: BTreeMap<i32, BTreeSet<i32>> = BTreeMap::new();

    // ToolOrderUtils.cpp:1557  for(auto filament_idx : filament_lists){
    for &filament_idx in filament_lists {
        // ToolOrderUtils.cpp:1558  auto nozzle_info = nozzle_group_result.get_nozzle_for_filament(filament_idx, -1);
        let nozzle_info = nozzle_group_result.get_nozzle_for_filament(filament_idx as i32, -1);
        // ToolOrderUtils.cpp:1559-1560  if (!nozzle_info) continue;
        let Some(nozzle_info) = nozzle_info else {
            continue;
        };
        // ToolOrderUtils.cpp:1561  nozzle_filament_groups[nozzle_info->group_id].insert(filament_idx);
        nozzle_filament_groups.entry(nozzle_info.group_id).or_default().insert(filament_idx);
        // ToolOrderUtils.cpp:1562  extruder_to_nozzle[nozzle_info->extruder_id].insert(nozzle_info->group_id);
        extruder_to_nozzle.entry(nozzle_info.extruder_id).or_default().insert(nozzle_info.group_id);
    }

    // ToolOrderUtils.cpp:1565  std::map<size_t, std::vector<unsigned int>>custom_layer_sequence_map;
    let mut custom_layer_sequence_map: BTreeMap<usize, Vec<u32>> = BTreeMap::new();
    // ToolOrderUtils.cpp:1566  for (size_t layer = 0; layer < layer_filaments.size(); ++layer){
    for layer in 0..layer_filaments.len() {
        // ToolOrderUtils.cpp:1567  const auto& curr_lf = layer_filaments[layer];  (the body reads layer_filaments[layer])
        // ToolOrderUtils.cpp:1568  std::vector<int> custom_filament_seq;
        let mut custom_filament_seq: Vec<i32> = Vec::new();
        // ToolOrderUtils.cpp:1569  if (get_custom_seq && get_custom_seq(layer, custom_filament_seq) && !custom_filament_seq.empty()) {
        if let Some(gcs) = get_custom_seq {
            if gcs(layer as i32, &mut custom_filament_seq) && !custom_filament_seq.is_empty() {
                // ToolOrderUtils.cpp:1570  std::vector<unsigned int> unsign_custom_extruder_seq;
                let mut unsign_custom_extruder_seq: Vec<u32> = Vec::new();
                // ToolOrderUtils.cpp:1571  for (int extruder : custom_filament_seq) {
                for &extruder in &custom_filament_seq {
                    // ToolOrderUtils.cpp:1572  unsigned int unsign_extruder = static_cast<unsigned int>(extruder) - 1;
                    let unsign_extruder = (extruder as u32).wrapping_sub(1);
                    // ToolOrderUtils.cpp:1573-1575  if (std::find(layer_filaments[layer]..., unsign_extruder) != end) emplace_back
                    if layer_filaments[layer].contains(&unsign_extruder) {
                        unsign_custom_extruder_seq.push(unsign_extruder);
                    }
                }
                // ToolOrderUtils.cpp:1577  assert(layer_filaments[layer].size() == unsign_custom_extruder_seq.size());
                debug_assert_eq!(layer_filaments[layer].len(), unsign_custom_extruder_seq.len());
                // ToolOrderUtils.cpp:1579  custom_layer_sequence_map[layer] = unsign_custom_extruder_seq;
                custom_layer_sequence_map.insert(layer, unsign_custom_extruder_seq);
            }
        }
    }

    // ToolOrderUtils.cpp:1584  std::map<int, std::vector<std::vector<unsigned int>>> nozzle_filament_sequences;
    let mut nozzle_filament_sequences: BTreeMap<i32, Vec<Vec<u32>>> = BTreeMap::new();
    // ToolOrderUtils.cpp:1585  bool store_sequence = filament_sequences != nullptr;
    let store_sequence = filament_sequences.is_some();

    // ToolOrderUtils.cpp:1587  int cost = 0;
    let mut cost: i32 = 0;
    // ToolOrderUtils.cpp:1588  for(auto& group : nozzle_filament_groups){
    for (&nozzle_id, filament_in_nozzle) in &nozzle_filament_groups {
        // ToolOrderUtils.cpp:1589  int nozzle_id = group.first;
        // ToolOrderUtils.cpp:1590  auto& filament_in_nozzle = group.second;

        // ToolOrderUtils.cpp:1592  int extruder_id = 0;
        let mut extruder_id: i32 = 0;
        // ToolOrderUtils.cpp:1593  for(auto& [ext, nozzle_set] : extruder_to_nozzle){
        for (&ext, nozzle_set) in &extruder_to_nozzle {
            // ToolOrderUtils.cpp:1594  if(nozzle_set.count(nozzle_id)){
            if nozzle_set.contains(&nozzle_id) {
                // ToolOrderUtils.cpp:1595  extruder_id = ext;
                extruder_id = ext;
                // ToolOrderUtils.cpp:1596  break;
                break;
            }
        }

        // ToolOrderUtils.cpp:1600-1601  if(filament_in_nozzle.empty()) continue;
        if filament_in_nozzle.is_empty() {
            continue;
        }
        // ToolOrderUtils.cpp:1602-1603  if (extruder_id < 0 || extruder_id >= static_cast<int>(flush_matrix.size())) continue;
        if extruder_id < 0 || extruder_id >= flush_matrix.len() as i32 {
            continue;
        }

        // ToolOrderUtils.cpp:1605  std::vector<unsigned int> filament_vec_in_nozzle(filament_in_nozzle.begin(), filament_in_nozzle.end());
        let filament_vec_in_nozzle: Vec<u32> = filament_in_nozzle.iter().copied().collect();

        // ToolOrderUtils.cpp:1607  const auto& extruder_flush = flush_matrix[extruder_id];
        let extruder_flush = &flush_matrix[extruder_id as usize];
        // ToolOrderUtils.cpp:1608  int initial_fil = initial_status.get_filament_in_nozzle(nozzle_id);
        let initial_fil = initial_status.get_filament_in_nozzle(nozzle_id);
        // ToolOrderUtils.cpp:1609  std::optional<unsigned int> initial_fil_id = (initial_fil >= 0 && initial_fil < static_cast<int>(extruder_flush.size()))? ... : std::nullopt;
        let initial_fil_id: Option<u32> = if initial_fil >= 0 && initial_fil < extruder_flush.len() as i32 {
            Some(initial_fil as u32)
        } else {
            None
        };

        // ToolOrderUtils.cpp:1611  std::vector<std::vector<unsigned int>> filament_seq;
        let mut filament_seq: Vec<Vec<u32>> = Vec::new();
        // ToolOrderUtils.cpp:1612-1613  cost += reorder_filaments_for_minimum_flush_volume_base(filament_vec_in_nozzle, layer_filaments, extruder_flush, get_custom_seq, store_sequence ? &filament_seq : nullptr, initial_fil_id);
        cost += reorder_filaments_for_minimum_flush_volume_base(
            &filament_vec_in_nozzle,
            layer_filaments,
            extruder_flush,
            get_custom_seq,
            if store_sequence { Some(&mut filament_seq) } else { None },
            initial_fil_id,
        );
        // ToolOrderUtils.cpp:1614-1615  if(store_sequence) nozzle_filament_sequences.emplace(nozzle_id, std::move(filament_seq));
        if store_sequence {
            nozzle_filament_sequences.entry(nozzle_id).or_insert(filament_seq);
        }
    }

    // ToolOrderUtils.cpp:1619-1620  if(!store_sequence) return cost;
    let Some(filament_sequences) = filament_sequences else {
        return cost;
    };

    // ToolOrderUtils.cpp:1622  std::vector<int> extruders;
    let mut extruders: Vec<i32> = Vec::new();
    // ToolOrderUtils.cpp:1623  std::map<int, std::vector<int>> nozzles_per_extruder;
    let mut nozzles_per_extruder: BTreeMap<i32, Vec<i32>> = BTreeMap::new();
    // ToolOrderUtils.cpp:1624  for (auto& [extruder_id, nozzle_set] : extruder_to_nozzle) {
    for (&extruder_id, nozzle_set) in &extruder_to_nozzle {
        // ToolOrderUtils.cpp:1625  extruders.push_back(extruder_id);
        extruders.push(extruder_id);
        // ToolOrderUtils.cpp:1626-1628  nozzles_per_extruder[extruder_id] = std::vector<int>(nozzle_set.begin(), nozzle_set.end());
        nozzles_per_extruder.insert(extruder_id, nozzle_set.iter().copied().collect());
    }

    // ToolOrderUtils.cpp:1631  filament_sequences->clear();
    filament_sequences.clear();
    // ToolOrderUtils.cpp:1632  filament_sequences->resize(layer_filaments.size());
    filament_sequences.resize(layer_filaments.len(), Vec::new());
    // ToolOrderUtils.cpp:1633  // No filament mapped to a nozzle: keep the empty per-layer sequences.
    // ToolOrderUtils.cpp:1634  // max_element on an empty extruder list is undefined and used to corrupt the heap.
    // ToolOrderUtils.cpp:1635-1636  if (extruders.empty()) return cost;
    if extruders.is_empty() {
        return cost;
    }

    // ToolOrderUtils.cpp:1638  auto get_extruder_for_filament = [nozzle_group_result](unsigned int filament_idx) {
    let get_extruder_for_filament = |filament_idx: u32| -> i32 {
        // ToolOrderUtils.cpp:1639  auto nozzle = nozzle_group_result.get_nozzle_for_filament(filament_idx, -1);
        match nozzle_group_result.get_nozzle_for_filament(filament_idx as i32, -1) {
            // ToolOrderUtils.cpp:1640-1641  if (!nozzle) return -1;
            None => -1,
            // ToolOrderUtils.cpp:1642  return nozzle->extruder_id;
            Some(nozzle) => nozzle.extruder_id,
        }
    };

    // ToolOrderUtils.cpp:1645  auto get_nozzle_idx_for_filament = [&nozzles_per_extruder, nozzle_group_result](unsigned int filament_idx)->int {
    //   (takes the map as an argument: it is captured by reference and read
    //   only, while the layer loop below never writes it)
    let get_nozzle_idx_for_filament = |nozzles_per_extruder: &BTreeMap<i32, Vec<i32>>, filament_idx: u32| -> i32 {
        // ToolOrderUtils.cpp:1646  auto nozzle = nozzle_group_result.get_nozzle_for_filament(filament_idx, -1);
        // ToolOrderUtils.cpp:1647-1648  if (!nozzle) return -1;
        let Some(nozzle) = nozzle_group_result.get_nozzle_for_filament(filament_idx as i32, -1) else {
            return -1;
        };
        // ToolOrderUtils.cpp:1649  auto ext_it = nozzles_per_extruder.find(nozzle->extruder_id);
        // ToolOrderUtils.cpp:1650-1651  if (ext_it == nozzles_per_extruder.end()) return -1;
        let Some(nozzles) = nozzles_per_extruder.get(&nozzle.extruder_id) else {
            return -1;
        };
        // ToolOrderUtils.cpp:1652  const auto& nozzles = ext_it->second;
        // ToolOrderUtils.cpp:1653  auto it = std::find(nozzles.begin(), nozzles.end(), nozzle->group_id);
        // ToolOrderUtils.cpp:1654-1655  if (it == nozzles.end()) return -1;
        // ToolOrderUtils.cpp:1656  return static_cast<int>(std::distance(nozzles.begin(), it));
        match nozzles.iter().position(|&x| x == nozzle.group_id) {
            None => -1,
            Some(pos) => pos as i32,
        }
    };

    // ToolOrderUtils.cpp:1659  int initial_extruder = initial_status.get_current_extruder_id();
    let initial_extruder = initial_status.get_current_extruder_id();
    // ToolOrderUtils.cpp:1660  int last_extruder_idx = (initial_extruder >= 0 && initial_extruder < static_cast<int>(extruders.size()))? initial_extruder : 0;
    let mut last_extruder_idx: i32 =
        if initial_extruder >= 0 && initial_extruder < extruders.len() as i32 { initial_extruder } else { 0 };
    // ToolOrderUtils.cpp:1661  // set size to max extruder_id in case extruder_id is not continuous
    // ToolOrderUtils.cpp:1662  int max_extruder_id = *std::max_element(extruders.begin(), extruders.end());
    let max_extruder_id: i32 = *extruders.iter().max().unwrap();
    // ToolOrderUtils.cpp:1663  std::vector<int> last_nozzle_idx(max_extruder_id >= 0 ? static_cast<size_t>(max_extruder_id) + 1 : 0, 0);
    let mut last_nozzle_idx: Vec<i32> =
        vec![0; if max_extruder_id >= 0 { max_extruder_id as usize + 1 } else { 0 }];
    // ToolOrderUtils.cpp:1664  auto nozzle_idx_in_range = [&last_nozzle_idx](int extruder_id) {
    //   (takes the vector as an argument: its size never changes after this point)
    let nozzle_idx_in_range = |last_nozzle_idx: &[i32], extruder_id: i32| -> bool {
        // ToolOrderUtils.cpp:1665  return extruder_id >= 0 && extruder_id < static_cast<int>(last_nozzle_idx.size());
        extruder_id >= 0 && extruder_id < last_nozzle_idx.len() as i32
    };
    // ToolOrderUtils.cpp:1667  for (int ext_id = 0; ext_id < static_cast<int>(last_nozzle_idx.size()); ext_id++) {
    for ext_id in 0..last_nozzle_idx.len() as i32 {
        // ToolOrderUtils.cpp:1668  auto ext_it = nozzles_per_extruder.find(ext_id);
        // ToolOrderUtils.cpp:1669-1670  if (ext_it == nozzles_per_extruder.end()) continue;
        let Some(ext_nozzles) = nozzles_per_extruder.get(&ext_id) else {
            continue;
        };
        // ToolOrderUtils.cpp:1671  int initial_nozzle = initial_status.get_nozzle_in_extruder(ext_id);
        let initial_nozzle = initial_status.get_nozzle_in_extruder(ext_id);
        // ToolOrderUtils.cpp:1672  const auto& ext_nozzles = ext_it->second;
        // ToolOrderUtils.cpp:1673  auto it = std::find(ext_nozzles.begin(), ext_nozzles.end(), initial_nozzle);
        // ToolOrderUtils.cpp:1674-1675  if (it != ext_nozzles.end()) last_nozzle_idx[ext_id] = static_cast<int>(std::distance(ext_nozzles.begin(), it));
        if let Some(pos) = ext_nozzles.iter().position(|&x| x == initial_nozzle) {
            last_nozzle_idx[ext_id as usize] = pos as i32;
        }
    }

    // ToolOrderUtils.cpp:1678  const int extruder_count = static_cast<int>(extruders.size());
    let extruder_count = extruders.len() as i32;
    // ToolOrderUtils.cpp:1679  for (size_t layer = 0; layer < layer_filaments.size(); ++layer) {
    for layer in 0..layer_filaments.len() {
        // ToolOrderUtils.cpp:1680  auto& out_seq = (*filament_sequences)[layer];

        // ToolOrderUtils.cpp:1682  if (custom_layer_sequence_map.find(layer) != custom_layer_sequence_map.end()) {
        if let Some(custom_seq) = custom_layer_sequence_map.get(&layer) {
            // ToolOrderUtils.cpp:1683  out_seq = custom_layer_sequence_map[layer];
            filament_sequences[layer] = custom_seq.clone();
            let out_seq = &filament_sequences[layer];
            // ToolOrderUtils.cpp:1684  if (!out_seq.empty()) {
            if !out_seq.is_empty() {
                // ToolOrderUtils.cpp:1685  int back_ext = get_extruder_for_filament(out_seq.back());
                let back_ext = get_extruder_for_filament(*out_seq.last().unwrap());
                // ToolOrderUtils.cpp:1686-1687  if (back_ext >= 0) last_extruder_idx = back_ext;
                if back_ext >= 0 {
                    last_extruder_idx = back_ext;
                }
                // ToolOrderUtils.cpp:1688  for (auto filament : out_seq) {
                for &filament in out_seq {
                    // ToolOrderUtils.cpp:1689  int cur_ext_id = get_extruder_for_filament(filament);
                    let cur_ext_id = get_extruder_for_filament(filament);
                    // ToolOrderUtils.cpp:1690  int nozzle_idx = get_nozzle_idx_for_filament(filament);
                    let nozzle_idx = get_nozzle_idx_for_filament(&nozzles_per_extruder, filament);
                    // ToolOrderUtils.cpp:1691-1692  // Filaments absent from this grouping have extruder -1. Writing that
                    // index used to overwrite the vector heap header (free(): invalid pointer).
                    // ToolOrderUtils.cpp:1693-1694  if (nozzle_idx >= 0 && nozzle_idx_in_range(cur_ext_id)) last_nozzle_idx[cur_ext_id] = nozzle_idx;
                    if nozzle_idx >= 0 && nozzle_idx_in_range(&last_nozzle_idx, cur_ext_id) {
                        last_nozzle_idx[cur_ext_id as usize] = nozzle_idx;
                    }
                }
            }
            // ToolOrderUtils.cpp:1697  continue;
            continue;
        }

        // ToolOrderUtils.cpp:1700-1701  if (last_extruder_idx < 0) last_extruder_idx = 0;
        if last_extruder_idx < 0 {
            last_extruder_idx = 0;
        }

        // ToolOrderUtils.cpp:1703  int curr_last_extruder_idx = last_extruder_idx;
        let mut curr_last_extruder_idx = last_extruder_idx;
        // ToolOrderUtils.cpp:1704  auto curr_last_nozzle_idx = last_nozzle_idx;
        let mut curr_last_nozzle_idx = last_nozzle_idx.clone();
        // ToolOrderUtils.cpp:1705  int start = last_extruder_idx % extruder_count;
        let mut start = last_extruder_idx % extruder_count;
        // ToolOrderUtils.cpp:1706-1707  if (start < 0) start += extruder_count;
        if start < 0 {
            start += extruder_count;
        }
        // ToolOrderUtils.cpp:1708  for (int i = 0; i < extruder_count; ++i) {
        for i in 0..extruder_count {
            // ToolOrderUtils.cpp:1709  int extruder_id = extruders[(start + i) % extruder_count];
            let extruder_id = extruders[((start + i) % extruder_count) as usize];
            // ToolOrderUtils.cpp:1710  auto ext_it = nozzles_per_extruder.find(extruder_id);
            // ToolOrderUtils.cpp:1711-1712  if (ext_it == nozzles_per_extruder.end() || ext_it->second.empty()) continue;
            let base_nozzles = match nozzles_per_extruder.get(&extruder_id) {
                Some(v) if !v.is_empty() => v,
                _ => continue,
            };
            // ToolOrderUtils.cpp:1713  auto& base_nozzles = ext_it->second;

            // ToolOrderUtils.cpp:1715  bool has_seq = false;
            let mut has_seq = false;
            // ToolOrderUtils.cpp:1716  int start_nozzle_idx = 0;
            let mut start_nozzle_idx: i32 = 0;
            // ToolOrderUtils.cpp:1717  if (nozzle_idx_in_range(extruder_id)) {
            if nozzle_idx_in_range(&last_nozzle_idx, extruder_id) {
                // ToolOrderUtils.cpp:1718-1719  if (last_nozzle_idx[extruder_id] < 0) last_nozzle_idx[extruder_id] = 0;
                if last_nozzle_idx[extruder_id as usize] < 0 {
                    last_nozzle_idx[extruder_id as usize] = 0;
                }
                // ToolOrderUtils.cpp:1720  start_nozzle_idx = last_nozzle_idx[extruder_id];
                start_nozzle_idx = last_nozzle_idx[extruder_id as usize];
            }

            // ToolOrderUtils.cpp:1723  const int nozzle_count = static_cast<int>(base_nozzles.size());
            let nozzle_count = base_nozzles.len() as i32;
            // ToolOrderUtils.cpp:1724  for (int j = 0; j < nozzle_count; ++j) {
            for j in 0..nozzle_count {
                // ToolOrderUtils.cpp:1725  int nozzle_idx = (start_nozzle_idx + j) % nozzle_count;
                let mut nozzle_idx = (start_nozzle_idx + j) % nozzle_count;
                // ToolOrderUtils.cpp:1726-1727  if (nozzle_idx < 0) nozzle_idx += nozzle_count;
                if nozzle_idx < 0 {
                    nozzle_idx += nozzle_count;
                }
                // ToolOrderUtils.cpp:1728  int nozzle_id = base_nozzles[nozzle_idx];
                let nozzle_id = base_nozzles[nozzle_idx as usize];
                // ToolOrderUtils.cpp:1729  auto seq_it = nozzle_filament_sequences.find(nozzle_id);
                // ToolOrderUtils.cpp:1730-1731  if (seq_it == nozzle_filament_sequences.end() || layer >= seq_it->second.size()) continue;
                let frag = match nozzle_filament_sequences.get(&nozzle_id) {
                    Some(seq) if layer < seq.len() => &seq[layer],
                    _ => continue,
                };
                // ToolOrderUtils.cpp:1732  const auto& frag = seq_it->second[layer];
                // ToolOrderUtils.cpp:1733-1734  if (frag.empty()) continue;
                if frag.is_empty() {
                    continue;
                }
                // ToolOrderUtils.cpp:1735  has_seq = true;
                has_seq = true;
                // ToolOrderUtils.cpp:1736-1737  if (nozzle_idx_in_range(extruder_id)) curr_last_nozzle_idx[extruder_id] = nozzle_idx;
                if nozzle_idx_in_range(&last_nozzle_idx, extruder_id) {
                    curr_last_nozzle_idx[extruder_id as usize] = nozzle_idx;
                }
                // ToolOrderUtils.cpp:1738  out_seq.insert(out_seq.end(), frag.begin(), frag.end());
                filament_sequences[layer].extend_from_slice(frag);
            }

            // ToolOrderUtils.cpp:1741-1742  if (has_seq) curr_last_extruder_idx = extruder_id;
            if has_seq {
                curr_last_extruder_idx = extruder_id;
            }
        }
        // ToolOrderUtils.cpp:1744  last_extruder_idx = curr_last_extruder_idx;
        last_extruder_idx = curr_last_extruder_idx;
        // ToolOrderUtils.cpp:1745  last_nozzle_idx = curr_last_nozzle_idx;
        last_nozzle_idx = curr_last_nozzle_idx;
    }
    // ToolOrderUtils.cpp:1747  return cost;
    cost
}

/// The off arm of `TOOL_ORDER_UTILS_PIN`: the port of the earlier source of
/// `reorder_filaments_for_multi_nozzle_extruder`, kept byte for byte.
fn reorder_filaments_for_multi_nozzle_extruder_pre_pin(
    filament_lists: &[u32],
    nozzle_group_result: &LayeredNozzleGroupResult,
    layer_filaments: &[Vec<u32>],
    flush_matrix: &[FlushMatrix],
    get_custom_seq: Option<&dyn Fn(i32, &mut Vec<i32>) -> bool>,
    filament_sequences: Option<&mut Vec<Vec<u32>>>,
    initial_status: &NozzleStatusRecorder,
) -> i32 {
    // ToolOrderUtils.cpp:1559  std::map<int,std::set<unsigned int>> nozzle_filament_groups;
    let mut nozzle_filament_groups: BTreeMap<i32, BTreeSet<u32>> = BTreeMap::new();
    // ToolOrderUtils.cpp:1560  std::map<int,std::set<int>> extruder_to_nozzle;
    let mut extruder_to_nozzle: BTreeMap<i32, BTreeSet<i32>> = BTreeMap::new();

    // ToolOrderUtils.cpp:1562  for(auto filament_idx : filament_lists)
    for &filament_idx in filament_lists {
        // ToolOrderUtils.cpp:1563  auto nozzle_info = nozzle_group_result.get_nozzle_for_filament(filament_idx, -1);
        let nozzle_info = nozzle_group_result.get_nozzle_for_filament(filament_idx as i32, -1);
        // ToolOrderUtils.cpp:1564  if (!nozzle_info)
        let nozzle_info = match nozzle_info {
            // ToolOrderUtils.cpp:1565  continue;
            None => continue,
            Some(n) => n,
        };
        // ToolOrderUtils.cpp:1566  nozzle_filament_groups[nozzle_info->group_id].insert(filament_idx);
        nozzle_filament_groups
            .entry(nozzle_info.group_id)
            .or_default()
            .insert(filament_idx);
        // ToolOrderUtils.cpp:1567  extruder_to_nozzle[nozzle_info->extruder_id].insert(nozzle_info->group_id);
        extruder_to_nozzle
            .entry(nozzle_info.extruder_id)
            .or_default()
            .insert(nozzle_info.group_id);
    }

    // ToolOrderUtils.cpp:1570  std::map<size_t, std::vector<unsigned int>>custom_layer_sequence_map;
    let mut custom_layer_sequence_map: BTreeMap<usize, Vec<u32>> = BTreeMap::new();
    // ToolOrderUtils.cpp:1571  for (size_t layer = 0; layer < layer_filaments.size(); ++layer)
    for layer in 0..layer_filaments.len() {
        // ToolOrderUtils.cpp:1572  const auto& curr_lf = layer_filaments[layer];  (referenced via layer_filaments[layer])
        // ToolOrderUtils.cpp:1573  std::vector<int> custom_filament_seq;
        let mut custom_filament_seq: Vec<i32> = Vec::new();
        // ToolOrderUtils.cpp:1574  if (get_custom_seq && get_custom_seq(layer, custom_filament_seq) && !custom_filament_seq.empty())
        if let Some(gcs) = get_custom_seq {
            if gcs(layer as i32, &mut custom_filament_seq) && !custom_filament_seq.is_empty() {
                // ToolOrderUtils.cpp:1575  std::vector<unsigned int> unsign_custom_extruder_seq;
                let mut unsign_custom_extruder_seq: Vec<u32> = Vec::new();
                // ToolOrderUtils.cpp:1576  for (int extruder : custom_filament_seq)
                for &extruder in &custom_filament_seq {
                    // ToolOrderUtils.cpp:1577  unsigned int unsign_extruder = static_cast<unsigned int>(extruder) - 1;
                    let unsign_extruder = (extruder as u32).wrapping_sub(1);
                    // ToolOrderUtils.cpp:1578  auto it = std::find(layer_filaments[layer]..., unsign_extruder);
                    // ToolOrderUtils.cpp:1579  if (it != layer_filaments[layer].end())
                    if layer_filaments[layer].contains(&unsign_extruder) {
                        // ToolOrderUtils.cpp:1580  unsign_custom_extruder_seq.emplace_back(unsign_extruder);
                        unsign_custom_extruder_seq.push(unsign_extruder);
                    }
                }
                // ToolOrderUtils.cpp:1582  assert(layer_filaments[layer].size() == unsign_custom_extruder_seq.size());
                debug_assert_eq!(layer_filaments[layer].len(), unsign_custom_extruder_seq.len());

                // ToolOrderUtils.cpp:1584  custom_layer_sequence_map[layer] = unsign_custom_extruder_seq;
                custom_layer_sequence_map.insert(layer, unsign_custom_extruder_seq);
            }
        }
    }

    // ToolOrderUtils.cpp:1589  std::map<int, std::vector<std::vector<unsigned int>>> nozzle_filament_sequences;
    let mut nozzle_filament_sequences: BTreeMap<i32, Vec<Vec<u32>>> = BTreeMap::new();
    // ToolOrderUtils.cpp:1590  bool store_sequence = filament_sequences != nullptr;
    let store_sequence = filament_sequences.is_some();

    // ToolOrderUtils.cpp:1592  int cost = 0;
    let mut cost = 0;
    // ToolOrderUtils.cpp:1593  for(auto& group : nozzle_filament_groups)
    for (nozzle_id_ref, filament_in_nozzle) in &nozzle_filament_groups {
        // ToolOrderUtils.cpp:1594  int nozzle_id = group.first;
        let nozzle_id = *nozzle_id_ref;

        // ToolOrderUtils.cpp:1597  int extruder_id = 0;
        let mut extruder_id = 0;
        // ToolOrderUtils.cpp:1598  for(auto& [ext, nozzle_set] : extruder_to_nozzle)
        for (ext, nozzle_set) in &extruder_to_nozzle {
            // ToolOrderUtils.cpp:1599  if(nozzle_set.count(nozzle_id))
            if nozzle_set.contains(&nozzle_id) {
                // ToolOrderUtils.cpp:1600  extruder_id = ext;
                extruder_id = *ext;
                // ToolOrderUtils.cpp:1601  break;
                break;
            }
        }

        // ToolOrderUtils.cpp:1605  if(filament_in_nozzle.empty())
        if filament_in_nozzle.is_empty() {
            // ToolOrderUtils.cpp:1606  continue;
            continue;
        }

        // ToolOrderUtils.cpp:1608  std::vector<unsigned int> filament_vec_in_nozzle(filament_in_nozzle.begin(), filament_in_nozzle.end());
        let filament_vec_in_nozzle: Vec<u32> = filament_in_nozzle.iter().copied().collect();

        // ToolOrderUtils.cpp:1610  int initial_fil = initial_status.get_filament_in_nozzle(nozzle_id);
        let initial_fil = initial_status.get_filament_in_nozzle(nozzle_id);
        // ToolOrderUtils.cpp:1611  std::optional<unsigned int> initial_fil_id = (initial_fil >= 0 && initial_fil < flush_matrix[extruder_id].size())? ... : std::nullopt;
        let initial_fil_id: Option<u32> =
            if initial_fil >= 0 && (initial_fil as usize) < flush_matrix[extruder_id as usize].len()
            {
                Some(initial_fil as u32)
            } else {
                None
            };

        // ToolOrderUtils.cpp:1613  std::vector<std::vector<unsigned int>> filament_seq;
        let mut filament_seq: Vec<Vec<u32>> = Vec::new();
        // ToolOrderUtils.cpp:1614  cost += reorder_filaments_for_minimum_flush_volume_base(...);
        cost += reorder_filaments_for_minimum_flush_volume_base(
            &filament_vec_in_nozzle,
            layer_filaments,
            &flush_matrix[extruder_id as usize],
            get_custom_seq,
            if store_sequence { Some(&mut filament_seq) } else { None },
            initial_fil_id,
        );
        // ToolOrderUtils.cpp:1616  if(store_sequence)
        if store_sequence {
            // ToolOrderUtils.cpp:1617  nozzle_filament_sequences.emplace(nozzle_id, std::move(filament_seq));
            nozzle_filament_sequences.insert(nozzle_id, filament_seq);
        }
    }

    // ToolOrderUtils.cpp:1621  if(!store_sequence)
    if !store_sequence {
        // ToolOrderUtils.cpp:1622  return cost;
        return cost;
    }

    // ToolOrderUtils.cpp:1624  std::vector<int> extruders;
    let mut extruders: Vec<i32> = Vec::new();
    // ToolOrderUtils.cpp:1625  std::map<int, std::vector<int>> nozzles_per_extruder;
    let mut nozzles_per_extruder: BTreeMap<i32, Vec<i32>> = BTreeMap::new();
    // ToolOrderUtils.cpp:1626  for (auto& [extruder_id, nozzle_set] : extruder_to_nozzle)
    for (extruder_id, nozzle_set) in &extruder_to_nozzle {
        // ToolOrderUtils.cpp:1627  extruders.push_back(extruder_id);
        extruders.push(*extruder_id);
        // ToolOrderUtils.cpp:1628  nozzles_per_extruder[extruder_id] = std::vector<int>(nozzle_set.begin(), nozzle_set.end());
        nozzles_per_extruder.insert(*extruder_id, nozzle_set.iter().copied().collect());
    }

    // `filament_sequences` is Some here because store_sequence == true.
    let filament_sequences = filament_sequences.unwrap();
    // ToolOrderUtils.cpp:1633  filament_sequences->clear();
    filament_sequences.clear();
    // ToolOrderUtils.cpp:1634  filament_sequences->resize(layer_filaments.size());
    filament_sequences.resize(layer_filaments.len(), Vec::new());

    // ToolOrderUtils.cpp:1636  auto get_extruder_for_filament = [nozzle_group_result](unsigned int filament_idx)
    let get_extruder_for_filament = |filament_idx: u32| -> i32 {
        // ToolOrderUtils.cpp:1637  auto nozzle = nozzle_group_result.get_nozzle_for_filament(filament_idx, -1);
        // ToolOrderUtils.cpp:1638-1639  if (!nozzle) return -1;
        match nozzle_group_result.get_nozzle_for_filament(filament_idx as i32, -1) {
            None => -1,
            // ToolOrderUtils.cpp:1640  return nozzle->extruder_id;
            Some(nozzle) => nozzle.extruder_id,
        }
    };

    // ToolOrderUtils.cpp:1643  auto get_nozzle_idx_for_filament = [nozzles_per_extruder, nozzle_group_result](unsigned int filament_idx)->int
    //   The C++ lambda captures `nozzles_per_extruder` BY COPY; mirror that so it
    //   does not alias the mutable map used later in the layer loop.
    let nozzles_per_extruder_capture = nozzles_per_extruder.clone();
    let get_nozzle_idx_for_filament = |filament_idx: u32| -> i32 {
        // ToolOrderUtils.cpp:1644  auto nozzle = nozzle_group_result.get_nozzle_for_filament(filament_idx, -1);
        // ToolOrderUtils.cpp:1645-1646  if (!nozzle) return -1;
        match nozzle_group_result.get_nozzle_for_filament(filament_idx as i32, -1) {
            None => -1,
            Some(nozzle) => {
                // ToolOrderUtils.cpp:1647  return std::find(nozzles_per_extruder.at(nozzle->extruder_id)..., nozzle->group_id) - ...begin();
                let v = &nozzles_per_extruder_capture[&nozzle.extruder_id];
                v.iter().position(|&x| x == nozzle.group_id).unwrap_or(v.len()) as i32
            }
        }
    };

    // ToolOrderUtils.cpp:1650  int initial_extruder = initial_status.get_current_extruder_id();
    let initial_extruder = initial_status.get_current_extruder_id();
    // ToolOrderUtils.cpp:1651  int last_extruder_idx = (initial_extruder >= 0 && initial_extruder < extruders.size())? initial_extruder : 0;
    let mut last_extruder_idx =
        if initial_extruder >= 0 && (initial_extruder as usize) < extruders.len() {
            initial_extruder
        } else {
            0
        };
    // ToolOrderUtils.cpp:1652  // set size to max extruder_id in case extruder_id is not continuous
    // ToolOrderUtils.cpp:1653  std::vector<int> last_nozzle_idx(*std::max_element(extruders.begin(),extruders.end()) + 1,0);
    let mut last_nozzle_idx = vec![0i32; (*extruders.iter().max().unwrap() + 1) as usize];
    // ToolOrderUtils.cpp:1654  for (int ext_id = 0; ext_id < static_cast<int>(last_nozzle_idx.size()); ext_id++)
    for ext_id in 0..last_nozzle_idx.len() as i32 {
        // ToolOrderUtils.cpp:1655  int initial_nozzle = initial_status.get_nozzle_in_extruder(ext_id);
        let initial_nozzle = initial_status.get_nozzle_in_extruder(ext_id);
        // ToolOrderUtils.cpp:1656  auto ext_nozzles = nozzles_per_extruder[ext_id];
        let ext_nozzles = nozzles_per_extruder.entry(ext_id).or_default().clone();
        // ToolOrderUtils.cpp:1657  auto it = std::find(ext_nozzles.begin(), ext_nozzles.end(), initial_nozzle);
        // ToolOrderUtils.cpp:1658  if (it != ext_nozzles.end())
        if let Some(pos) = ext_nozzles.iter().position(|&x| x == initial_nozzle) {
            // ToolOrderUtils.cpp:1659  last_nozzle_idx[ext_id] = static_cast<int>(std::distance(ext_nozzles.begin(), it));
            last_nozzle_idx[ext_id as usize] = pos as i32;
        }
    }

    // ToolOrderUtils.cpp:1662  for (size_t layer = 0; layer < layer_filaments.size(); ++layer)
    for layer in 0..layer_filaments.len() {
        // ToolOrderUtils.cpp:1663  auto& out_seq = (*filament_sequences)[layer];

        // ToolOrderUtils.cpp:1665  if (custom_layer_sequence_map.find(layer) != custom_layer_sequence_map.end())
        if let Some(custom_seq) = custom_layer_sequence_map.get(&layer) {
            // ToolOrderUtils.cpp:1666  out_seq = custom_layer_sequence_map[layer];
            filament_sequences[layer] = custom_seq.clone();
            // ToolOrderUtils.cpp:1667  if (!out_seq.empty())
            if !filament_sequences[layer].is_empty() {
                // ToolOrderUtils.cpp:1668  last_extruder_idx = get_extruder_for_filament(out_seq.back());
                last_extruder_idx =
                    get_extruder_for_filament(*filament_sequences[layer].last().unwrap());
                // ToolOrderUtils.cpp:1669  for (auto filament : out_seq)
                for &filament in &filament_sequences[layer] {
                    // ToolOrderUtils.cpp:1670  int cur_ext_id = get_extruder_for_filament(filament);
                    let cur_ext_id = get_extruder_for_filament(filament);
                    // ToolOrderUtils.cpp:1671  last_nozzle_idx[cur_ext_id] = get_nozzle_idx_for_filament(filament);
                    last_nozzle_idx[cur_ext_id as usize] = get_nozzle_idx_for_filament(filament);
                }
            }
            // ToolOrderUtils.cpp:1674  continue;
            continue;
        }

        // ToolOrderUtils.cpp:1677  if (last_extruder_idx == -1)
        if last_extruder_idx == -1 {
            // ToolOrderUtils.cpp:1678  last_extruder_idx = 0;
            last_extruder_idx = 0;
        }

        // ToolOrderUtils.cpp:1680  int curr_last_extruder_idx = last_extruder_idx;
        let mut curr_last_extruder_idx = last_extruder_idx;
        // ToolOrderUtils.cpp:1681  auto curr_last_nozzle_idx = last_nozzle_idx;
        let mut curr_last_nozzle_idx = last_nozzle_idx.clone();
        // ToolOrderUtils.cpp:1682  for (int i = 0; i < extruders.size(); ++i)
        for i in 0..extruders.len() {
            // ToolOrderUtils.cpp:1683  int extruder_id = extruders[(last_extruder_idx + i) % extruders.size()];
            let extruder_id = extruders[(last_extruder_idx as usize + i) % extruders.len()];
            // ToolOrderUtils.cpp:1684  auto& base_nozzles = nozzles_per_extruder[extruder_id];
            let base_nozzles = nozzles_per_extruder.entry(extruder_id).or_default().clone();

            // ToolOrderUtils.cpp:1686  bool has_seq = false;
            let mut has_seq = false;
            // ToolOrderUtils.cpp:1687  if (last_nozzle_idx[extruder_id] == -1)
            if last_nozzle_idx[extruder_id as usize] == -1 {
                // ToolOrderUtils.cpp:1688  last_nozzle_idx[extruder_id] = 0;
                last_nozzle_idx[extruder_id as usize] = 0;
            }

            // ToolOrderUtils.cpp:1690  for (int j = 0; j < base_nozzles.size(); ++j)
            for j in 0..base_nozzles.len() {
                // ToolOrderUtils.cpp:1691  int nozzle_idx = (last_nozzle_idx[extruder_id] + j) % base_nozzles.size();
                let nozzle_idx =
                    (last_nozzle_idx[extruder_id as usize] as usize + j) % base_nozzles.len();
                // ToolOrderUtils.cpp:1692  int nozzle_id = base_nozzles[nozzle_idx];
                let nozzle_id = base_nozzles[nozzle_idx];
                // ToolOrderUtils.cpp:1693  const auto& frag = nozzle_filament_sequences[nozzle_id][layer];
                let frag = nozzle_filament_sequences.entry(nozzle_id).or_default();
                let frag = if layer < frag.len() { frag[layer].clone() } else { Vec::new() };
                // ToolOrderUtils.cpp:1694  if (frag.empty())
                if frag.is_empty() {
                    // ToolOrderUtils.cpp:1695  continue;
                    continue;
                }
                // ToolOrderUtils.cpp:1696  has_seq = true;
                has_seq = true;
                // ToolOrderUtils.cpp:1697  curr_last_nozzle_idx[extruder_id] = nozzle_idx;
                curr_last_nozzle_idx[extruder_id as usize] = nozzle_idx as i32;
                // ToolOrderUtils.cpp:1698  out_seq.insert(out_seq.end(), frag.begin(), frag.end());
                filament_sequences[layer].extend_from_slice(&frag);
            }

            // ToolOrderUtils.cpp:1701  if (has_seq)
            if has_seq {
                // ToolOrderUtils.cpp:1702  curr_last_extruder_idx = extruder_id;
                curr_last_extruder_idx = extruder_id;
            }
        }
        // ToolOrderUtils.cpp:1704  last_extruder_idx = curr_last_extruder_idx;
        last_extruder_idx = curr_last_extruder_idx;
        // ToolOrderUtils.cpp:1705  last_nozzle_idx = curr_last_nozzle_idx;
        last_nozzle_idx = curr_last_nozzle_idx;
    }
    // ToolOrderUtils.cpp:1707  return cost;
    cost
}
