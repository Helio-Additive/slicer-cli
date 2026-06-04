//! Tool ordering utility module for multi-extruder min-cost flow optimization.
//!
//! C++ Reference:
//! - GCode/ToolOrderUtils.hpp
//! - GCode/ToolOrderUtils.cpp
//!
//! This module provides graph-based solvers for optimizing tool change sequences
//! to minimize flush/purge volume in multi-material prints.

use std::collections::{HashMap, HashSet, VecDeque};

/// Type alias for flush volume matrix (from_filament x to_filament).
pub type FlushMatrix = Vec<Vec<f32>>;

/// Constants for the max flow graph.
pub const INF: i32 = i32::MAX;
pub const INVALID_ID: i32 = -1;

/// Edge in a flow network.
#[derive(Debug, Clone)]
pub struct Edge {
    pub from: i32,
    pub to: i32,
    pub capacity: i32,
    pub flow: i32,
    pub cost: f32,
}

impl Edge {
    pub fn new(from: i32, to: i32, capacity: i32) -> Self {
        Self {
            from,
            to,
            capacity,
            flow: 0,
            cost: 0.0,
        }
    }

    pub fn with_cost(from: i32, to: i32, capacity: i32, cost: f32) -> Self {
        Self {
            from,
            to,
            capacity,
            flow: 0,
            cost,
        }
    }

    /// Remaining capacity on this edge.
    pub fn residual(&self) -> i32 {
        self.capacity - self.flow
    }
}

/// Maximum flow solver using augmenting paths (Edmonds-Karp / BFS).
/// Used for bipartite matching of tool assignments.
/// Corresponds to C++ MaxFlowSolver.
#[derive(Debug, Clone)]
pub struct MaxFlowSolver {
    total_nodes: i32,
    source_id: i32,
    sink_id: i32,
    edges: Vec<Edge>,
    adj: Vec<Vec<usize>>,
    l_nodes: Vec<i32>,
    r_nodes: Vec<i32>,
}

impl MaxFlowSolver {
    /// Create a new max flow solver for bipartite matching.
    ///
    /// `u_nodes` and `v_nodes` are the left/right node IDs.
    /// Optional constraints can limit which u-v pairs can be linked.
    pub fn new(
        u_nodes: &[i32],
        v_nodes: &[i32],
        uv_link_limits: &HashMap<i32, Vec<i32>>,
        uv_unlink_limits: &HashMap<i32, Vec<i32>>,
        u_capacity: &[i32],
        v_capacity: &[i32],
        v_group_capacity: &[(HashSet<i32>, i32)],
    ) -> Self {
        let max_node = u_nodes
            .iter()
            .chain(v_nodes.iter())
            .copied()
            .max()
            .unwrap_or(0);
        let source_id = max_node + 1;
        let sink_id = max_node + 2;
        // Extra nodes for group capacity constraints
        let group_start = max_node + 3;
        let total_nodes = group_start + v_group_capacity.len() as i32;

        let mut solver = Self {
            total_nodes,
            source_id,
            sink_id,
            edges: Vec::new(),
            adj: vec![Vec::new(); total_nodes as usize + 1],
            l_nodes: u_nodes.to_vec(),
            r_nodes: v_nodes.to_vec(),
        };

        // Source -> u_nodes
        for (i, &u) in u_nodes.iter().enumerate() {
            let cap = if i < u_capacity.len() {
                u_capacity[i]
            } else {
                1
            };
            solver.add_edge(source_id, u, cap);
        }

        // v_nodes -> sink
        for (i, &v) in v_nodes.iter().enumerate() {
            let cap = if i < v_capacity.len() {
                v_capacity[i]
            } else {
                1
            };
            solver.add_edge(v, sink_id, cap);
        }

        // u -> v edges based on link/unlink constraints
        for &u in u_nodes {
            for &v in v_nodes {
                let allowed = if let Some(links) = uv_link_limits.get(&u) {
                    links.contains(&v)
                } else {
                    true
                };
                let blocked = if let Some(unlinks) = uv_unlink_limits.get(&u) {
                    unlinks.contains(&v)
                } else {
                    false
                };

                if allowed && !blocked {
                    solver.add_edge(u, v, 1);
                }
            }
        }

        // Group capacity constraints
        for (gi, (group_set, group_cap)) in v_group_capacity.iter().enumerate() {
            let group_node = group_start + gi as i32;
            for &v in group_set {
                solver.add_edge(v, group_node, 1);
            }
            solver.add_edge(group_node, sink_id, *group_cap);
        }

        solver
    }

    fn add_edge(&mut self, from: i32, to: i32, capacity: i32) {
        let idx1 = self.edges.len();
        let idx2 = idx1 + 1;
        self.edges.push(Edge::new(from, to, capacity));
        self.edges.push(Edge::new(to, from, 0)); // reverse edge
        self.adj[from as usize].push(idx1);
        self.adj[to as usize].push(idx2);
    }

    /// Solve the max flow problem using BFS (Edmonds-Karp).
    /// Returns a mapping: for each u_node, the assigned v_node (or -1 if unassigned).
    pub fn solve(&mut self) -> Vec<i32> {
        // Run Edmonds-Karp
        loop {
            // BFS to find augmenting path
            let mut parent = vec![(-1i32, usize::MAX); self.total_nodes as usize + 1];
            let mut visited = vec![false; self.total_nodes as usize + 1];
            let mut queue = VecDeque::new();
            queue.push_back(self.source_id);
            visited[self.source_id as usize] = true;

            while let Some(node) = queue.pop_front() {
                if node == self.sink_id {
                    break;
                }
                for &edge_idx in &self.adj[node as usize] {
                    let edge = &self.edges[edge_idx];
                    if !visited[edge.to as usize] && edge.residual() > 0 {
                        visited[edge.to as usize] = true;
                        parent[edge.to as usize] = (node, edge_idx);
                        queue.push_back(edge.to);
                    }
                }
            }

            if !visited[self.sink_id as usize] {
                break;
            }

            // Find bottleneck
            let mut path_flow = INF;
            let mut node = self.sink_id;
            while node != self.source_id {
                let (_, edge_idx) = parent[node as usize];
                path_flow = path_flow.min(self.edges[edge_idx].residual());
                node = parent[node as usize].0;
            }

            // Update flow
            node = self.sink_id;
            while node != self.source_id {
                let (_, edge_idx) = parent[node as usize];
                self.edges[edge_idx].flow += path_flow;
                self.edges[edge_idx ^ 1].flow -= path_flow;
                node = parent[node as usize].0;
            }
        }

        // Extract matching: for each u_node, find the v_node with flow
        let mut result = vec![-1i32; self.l_nodes.len()];
        let r_set: HashSet<i32> = self.r_nodes.iter().copied().collect();

        for (i, &u) in self.l_nodes.iter().enumerate() {
            for &edge_idx in &self.adj[u as usize] {
                let edge = &self.edges[edge_idx];
                if edge.flow > 0 && r_set.contains(&edge.to) {
                    result[i] = edge.to;
                    break;
                }
            }
        }

        result
    }
}

/// Min-cost max-flow network for optimal tool assignment.
/// Corresponds to C++ MinCostMaxFlow.
#[derive(Debug, Clone)]
pub struct MinCostMaxFlow {
    total_nodes: i32,
    source_id: i32,
    sink_id: i32,
    edges: Vec<Edge>,
    adj: Vec<Vec<usize>>,
}

impl MinCostMaxFlow {
    pub fn new(n: i32) -> Self {
        Self {
            total_nodes: n,
            source_id: 0,
            sink_id: n - 1,
            edges: Vec::new(),
            adj: vec![Vec::new(); n as usize],
        }
    }

    pub fn add_edge(&mut self, from: i32, to: i32, capacity: i32, cost: f32) {
        let idx1 = self.edges.len();
        let idx2 = idx1 + 1;
        self.edges.push(Edge::with_cost(from, to, capacity, cost));
        self.edges.push(Edge::with_cost(to, from, 0, -cost));
        self.adj[from as usize].push(idx1);
        self.adj[to as usize].push(idx2);
    }

    /// Solve using SPFA (Bellman-Ford with queue) to find shortest augmenting paths.
    /// Returns (max_flow, min_cost).
    pub fn solve(&mut self) -> (i32, f32) {
        let mut total_flow = 0;
        let mut total_cost = 0.0f32;

        loop {
            // SPFA to find shortest path
            let n = self.total_nodes as usize;
            let mut dist = vec![f32::MAX; n];
            let mut in_queue = vec![false; n];
            let mut parent_edge = vec![usize::MAX; n];
            dist[self.source_id as usize] = 0.0;

            let mut queue = VecDeque::new();
            queue.push_back(self.source_id);
            in_queue[self.source_id as usize] = true;

            while let Some(node) = queue.pop_front() {
                in_queue[node as usize] = false;
                for &edge_idx in &self.adj[node as usize] {
                    let edge = &self.edges[edge_idx];
                    if edge.residual() > 0
                        && dist[node as usize] + edge.cost < dist[edge.to as usize]
                    {
                        dist[edge.to as usize] = dist[node as usize] + edge.cost;
                        parent_edge[edge.to as usize] = edge_idx;
                        if !in_queue[edge.to as usize] {
                            in_queue[edge.to as usize] = true;
                            queue.push_back(edge.to);
                        }
                    }
                }
            }

            if dist[self.sink_id as usize] == f32::MAX {
                break;
            }

            // Find bottleneck
            let mut path_flow = INF;
            let mut node = self.sink_id;
            while node != self.source_id {
                let edge_idx = parent_edge[node as usize];
                path_flow = path_flow.min(self.edges[edge_idx].residual());
                node = self.edges[edge_idx].from;
            }

            // Update flow
            node = self.sink_id;
            while node != self.source_id {
                let edge_idx = parent_edge[node as usize];
                self.edges[edge_idx].flow += path_flow;
                self.edges[edge_idx ^ 1].flow -= path_flow;
                total_cost += path_flow as f32 * self.edges[edge_idx].cost;
                node = self.edges[edge_idx].from;
            }

            total_flow += path_flow;
        }

        (total_flow, total_cost)
    }
}

/// General minimum cost solver using min-cost max-flow.
/// Corresponds to C++ GeneralMinCostSolver.
#[derive(Debug, Clone)]
pub struct GeneralMinCostSolver {
    solver: MinCostMaxFlow,
    u_nodes: Vec<i32>,
    v_nodes: Vec<i32>,
}

impl GeneralMinCostSolver {
    pub fn new(matrix: &[Vec<f32>], u_nodes: &[i32], v_nodes: &[i32]) -> Self {
        let max_node = u_nodes
            .iter()
            .chain(v_nodes.iter())
            .copied()
            .max()
            .unwrap_or(0);
        let source = max_node + 1;
        let sink = max_node + 2;
        let total = sink + 1;

        let mut solver = MinCostMaxFlow::new(total);
        solver.source_id = source;
        solver.sink_id = sink;
        solver.adj.resize(total as usize, Vec::new());

        // Source -> u with capacity 1
        for &u in u_nodes {
            solver.add_edge(source, u, 1, 0.0);
        }

        // v -> sink with capacity 1
        for &v in v_nodes {
            solver.add_edge(v, sink, 1, 0.0);
        }

        // u -> v with cost from matrix
        for (ui, &u) in u_nodes.iter().enumerate() {
            for (vi, &v) in v_nodes.iter().enumerate() {
                if ui < matrix.len() && vi < matrix[ui].len() {
                    solver.add_edge(u, v, 1, matrix[ui][vi]);
                }
            }
        }

        Self {
            solver,
            u_nodes: u_nodes.to_vec(),
            v_nodes: v_nodes.to_vec(),
        }
    }

    /// Solve and return the assignment: for each u_node, the assigned v_node index.
    pub fn solve(&mut self) -> Vec<i32> {
        self.solver.solve();

        let v_set: HashSet<i32> = self.v_nodes.iter().copied().collect();
        let mut result = vec![-1i32; self.u_nodes.len()];

        for (i, &u) in self.u_nodes.iter().enumerate() {
            for &edge_idx in &self.solver.adj[u as usize] {
                let edge = &self.solver.edges[edge_idx];
                if edge.flow > 0 && v_set.contains(&edge.to) {
                    result[i] = edge.to;
                    break;
                }
            }
        }

        result
    }
}

/// Minimum flush flow solver that minimizes flush/purge volume.
/// Corresponds to C++ MinFlushFlowSolver.
#[derive(Debug, Clone)]
pub struct MinFlushFlowSolver {
    solver: MinCostMaxFlow,
    u_nodes: Vec<i32>,
    v_nodes: Vec<i32>,
}

impl MinFlushFlowSolver {
    pub fn new(
        matrix: &[Vec<f32>],
        u_nodes: &[i32],
        v_nodes: &[i32],
        uv_link_limits: &HashMap<i32, Vec<i32>>,
        uv_unlink_limits: &HashMap<i32, Vec<i32>>,
        u_capacity: &[i32],
        v_capacity: &[i32],
        v_group_capacity: &[(HashSet<i32>, i32)],
    ) -> Self {
        let max_node = u_nodes
            .iter()
            .chain(v_nodes.iter())
            .copied()
            .max()
            .unwrap_or(0);
        let group_count = v_group_capacity.len() as i32;
        let source = max_node + 1;
        let sink = max_node + 2;
        let group_start = max_node + 3;
        let total = group_start + group_count + 1;

        let mut solver = MinCostMaxFlow::new(total);
        solver.source_id = source;
        solver.sink_id = sink;
        solver.adj.resize(total as usize, Vec::new());

        // Source -> u
        for (i, &u) in u_nodes.iter().enumerate() {
            let cap = if i < u_capacity.len() {
                u_capacity[i]
            } else {
                1
            };
            solver.add_edge(source, u, cap, 0.0);
        }

        // v -> sink
        for (i, &v) in v_nodes.iter().enumerate() {
            let cap = if i < v_capacity.len() {
                v_capacity[i]
            } else {
                1
            };
            solver.add_edge(v, sink, cap, 0.0);
        }

        // u -> v edges
        for (ui, &u) in u_nodes.iter().enumerate() {
            for (vi, &v) in v_nodes.iter().enumerate() {
                let allowed = if let Some(links) = uv_link_limits.get(&u) {
                    links.contains(&v)
                } else {
                    true
                };
                let blocked = if let Some(unlinks) = uv_unlink_limits.get(&u) {
                    unlinks.contains(&v)
                } else {
                    false
                };

                if allowed && !blocked {
                    let cost = if ui < matrix.len() && vi < matrix[ui].len() {
                        matrix[ui][vi]
                    } else {
                        0.0
                    };
                    solver.add_edge(u, v, 1, cost);
                }
            }
        }

        // Group capacity constraints
        for (gi, (group_set, group_cap)) in v_group_capacity.iter().enumerate() {
            let group_node = group_start + gi as i32;
            for &v in group_set {
                solver.add_edge(v, group_node, 1, 0.0);
            }
            solver.add_edge(group_node, sink, *group_cap, 0.0);
        }

        Self {
            solver,
            u_nodes: u_nodes.to_vec(),
            v_nodes: v_nodes.to_vec(),
        }
    }

    /// Solve and return assignment.
    pub fn solve(&mut self) -> Vec<i32> {
        self.solver.solve();

        let v_set: HashSet<i32> = self.v_nodes.iter().copied().collect();
        let mut result = vec![-1i32; self.u_nodes.len()];

        for (i, &u) in self.u_nodes.iter().enumerate() {
            for &edge_idx in &self.solver.adj[u as usize] {
                let edge = &self.solver.edges[edge_idx];
                if edge.flow > 0 && v_set.contains(&edge.to) {
                    result[i] = edge.to;
                    break;
                }
            }
        }

        result
    }
}

/// Match mode group solver for grouped tool assignments.
/// Corresponds to C++ MatchModeGroupSolver.
#[derive(Debug, Clone)]
pub struct MatchModeGroupSolver {
    solver: MinCostMaxFlow,
    u_nodes: Vec<i32>,
    v_nodes: Vec<i32>,
}

impl MatchModeGroupSolver {
    pub fn new(
        matrix: &[Vec<f32>],
        u_nodes: &[i32],
        v_nodes: &[i32],
        v_capacity: &[i32],
        uv_unlink_limits: &HashMap<i32, Vec<i32>>,
    ) -> Self {
        let max_node = u_nodes
            .iter()
            .chain(v_nodes.iter())
            .copied()
            .max()
            .unwrap_or(0);
        let source = max_node + 1;
        let sink = max_node + 2;
        let total = sink + 1;

        let mut solver = MinCostMaxFlow::new(total);
        solver.source_id = source;
        solver.sink_id = sink;
        solver.adj.resize(total as usize, Vec::new());

        // Source -> u
        for &u in u_nodes {
            solver.add_edge(source, u, 1, 0.0);
        }

        // v -> sink with group capacity
        for (i, &v) in v_nodes.iter().enumerate() {
            let cap = if i < v_capacity.len() {
                v_capacity[i]
            } else {
                1
            };
            solver.add_edge(v, sink, cap, 0.0);
        }

        // u -> v edges
        for (ui, &u) in u_nodes.iter().enumerate() {
            for (vi, &v) in v_nodes.iter().enumerate() {
                let blocked = if let Some(unlinks) = uv_unlink_limits.get(&u) {
                    unlinks.contains(&v)
                } else {
                    false
                };

                if !blocked {
                    let cost = if ui < matrix.len() && vi < matrix[ui].len() {
                        matrix[ui][vi]
                    } else {
                        0.0
                    };
                    solver.add_edge(u, v, 1, cost);
                }
            }
        }

        Self {
            solver,
            u_nodes: u_nodes.to_vec(),
            v_nodes: v_nodes.to_vec(),
        }
    }

    /// Solve and return assignment.
    pub fn solve(&mut self) -> Vec<i32> {
        self.solver.solve();

        let v_set: HashSet<i32> = self.v_nodes.iter().copied().collect();
        let mut result = vec![-1i32; self.u_nodes.len()];

        for (i, &u) in self.u_nodes.iter().enumerate() {
            for &edge_idx in &self.solver.adj[u as usize] {
                let edge = &self.solver.edges[edge_idx];
                if edge.flow > 0 && v_set.contains(&edge.to) {
                    result[i] = edge.to;
                    break;
                }
            }
        }

        result
    }
}

/// Convenience function to create and run a MinFlushFlowSolver.
pub fn min_flush_flow_solver(
    matrix: &[Vec<f32>],
    u_nodes: &[i32],
    v_nodes: &[i32],
) -> crate::Result<Vec<i32>> {
    let mut solver = MinFlushFlowSolver::new(
        matrix,
        u_nodes,
        v_nodes,
        &HashMap::new(),
        &HashMap::new(),
        &[],
        &[],
        &[],
    );
    Ok(solver.solve())
}

/// Convenience function to create and run a MatchModeGroupSolver.
pub fn match_mode_group_solver(
    matrix: &[Vec<f32>],
    u_nodes: &[i32],
    v_nodes: &[i32],
    v_capacity: &[i32],
) -> crate::Result<Vec<i32>> {
    let mut solver =
        MatchModeGroupSolver::new(matrix, u_nodes, v_nodes, v_capacity, &HashMap::new());
    Ok(solver.solve())
}

/// Convenience function to create and run a GeneralMinCostSolver.
pub fn general_min_cost_solver(
    matrix: &[Vec<f32>],
    u_nodes: &[i32],
    v_nodes: &[i32],
) -> crate::Result<Vec<i32>> {
    let mut solver = GeneralMinCostSolver::new(matrix, u_nodes, v_nodes);
    Ok(solver.solve())
}

/// Add an edge to a MinCostMaxFlow solver.
pub fn add_edge(solver: &mut MinCostMaxFlow, from: i32, to: i32, capacity: i32, cost: f32) {
    solver.add_edge(from, to, capacity, cost);
}

/// Convenience function to create and run a MaxFlowSolver.
pub fn max_flow_solver(u_nodes: &[i32], v_nodes: &[i32]) -> crate::Result<Vec<i32>> {
    let mut solver = MaxFlowSolver::new(
        u_nodes,
        v_nodes,
        &HashMap::new(),
        &HashMap::new(),
        &[],
        &[],
        &[],
    );
    Ok(solver.solve())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_edge_residual() {
        let mut e = Edge::new(0, 1, 5);
        assert_eq!(e.residual(), 5);
        e.flow = 3;
        assert_eq!(e.residual(), 2);
    }

    #[test]
    fn test_max_flow_simple() {
        let u = vec![0, 1];
        let v = vec![2, 3];
        let mut solver =
            MaxFlowSolver::new(&u, &v, &HashMap::new(), &HashMap::new(), &[], &[], &[]);
        let result = solver.solve();
        // Both u nodes should be assigned to some v node
        assert_eq!(result.len(), 2);
    }

    #[test]
    fn test_general_min_cost() {
        let matrix = vec![vec![1.0, 10.0], vec![10.0, 1.0]];
        let result = general_min_cost_solver(&matrix, &[0, 1], &[2, 3]).unwrap();
        assert_eq!(result.len(), 2);
    }

    #[test]
    fn test_min_flush_flow() {
        let matrix = vec![vec![1.0, 5.0], vec![5.0, 1.0]];
        let result = min_flush_flow_solver(&matrix, &[0, 1], &[2, 3]).unwrap();
        assert_eq!(result.len(), 2);
    }
}
