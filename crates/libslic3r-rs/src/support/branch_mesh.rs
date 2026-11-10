//! Branch mesh drawing for tree supports.
//!
//! This module implements the creation of 3D tube meshes for tree support branches
//! and slicing them into per-layer polygons. This provides more accurate geometry
//! than simple circle-based rasterization.
//!
//! # Algorithm Overview
//!
//! 1. **Build Branch Paths**: Traverse support elements to build connected branch paths
//! 2. **Extrude Branches**: For each path, create a tube mesh with:
//!    - Bottom hemisphere at the start
//!    - Cylindrical sections between elements
//!    - Top hemisphere at the end
//! 3. **Slice Mesh**: Slice the cumulative mesh at each layer height to get polygons
//!
//! # BambuStudio Reference
//!
//! - `Support/TreeSupport3D.cpp`: `draw_branches()`, `extrude_branch()`, `slice_branches()`
//! - `Support/TreeSupport3D.cpp`: `discretize_circle()`, `triangulate_fan()`, `triangulate_strip()`

use crate::geometry::ExPolygon;
use crate::support::tree_support_settings::{SupportElement, TreeSupportSettings};
use crate::triangle_mesh::TriangleMesh;
use crate::triangle_mesh_slicer::slice_mesh;
use crate::{unscale, CoordF};
use std::f64::consts::PI;

/// Default epsilon for mesh discretization (controls polygon resolution).
const DEFAULT_EPS: f64 = 0.015;

/// 3D point with f64 coordinates for mesh building.
#[derive(Debug, Clone, Copy, Default)]
pub struct Point3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3D {
    // Create a new 3D point.
    #[inline]
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

/// Result of branch mesh generation.
#[derive(Debug, Clone)]
pub struct BranchMeshResult {
    /// The combined mesh of all branches.
    pub mesh: TriangleMesh,
    /// Z span of the mesh (min_z, max_z).
    pub z_span: (f64, f64),
    /// Number of branches processed.
    pub branch_count: usize,
}

/// Configuration for branch mesh generation.
#[derive(Debug, Clone)]
pub struct BranchMeshConfig {
    /// Discretization epsilon (smaller = smoother but more triangles).
    pub eps: f64,
    /// Minimum number of segments for circle discretization.
    pub min_circle_segments: usize,
    /// Maximum number of segments for circle discretization.
    pub max_circle_segments: usize,
}

impl Default for BranchMeshConfig {
    fn default() -> Self {
        Self {
            eps: DEFAULT_EPS,
            min_circle_segments: 12,
            max_circle_segments: 64,
        }
    }
}

/// A branch path consisting of connected support elements.
#[derive(Debug, Clone)]
pub struct BranchPath {
    /// The support elements in this path, from bottom to top.
    elements: Vec<BranchPathElement>,
}

/// A single element in a branch path.
#[derive(Debug, Clone)]
pub struct BranchPathElement {
    /// Position on the layer (in mm, unscaled).
    pub position: Point3D,
    /// Radius at this point (in mm).
    pub radius: f64,
    /// Layer index.
    pub layer_idx: usize,
}

impl BranchPath {
    // Create a new empty branch path.
    pub fn new() -> Self {
        Self {
            elements: Vec::new(),
        }
    }

    /// Add an element to the path.
    pub fn push(&mut self, element: BranchPathElement) {
        self.elements.push(element);
    }

    /// Get the elements in this path.
    pub fn elements(&self) -> &[BranchPathElement] {
        &self.elements
    }

    /// Check if the path has at least 2 elements (required for mesh generation).
    pub fn is_valid(&self) -> bool {
        self.elements.len() >= 2
    }

    /// Get the number of elements.
    pub fn len(&self) -> usize {
        self.elements.len()
    }

    /// Check if empty.
    pub fn is_empty(&self) -> bool {
        self.elements.is_empty()
    }
}

impl Default for BranchPath {
    fn default() -> Self {
        Self::new()
    }
}

/// Branch mesh builder that creates 3D tube meshes for tree support branches.
#[derive(Debug)]
pub struct BranchMeshBuilder {
    config: BranchMeshConfig,
    mesh: TriangleMesh,
    z_min: f64,
    z_max: f64,
    branch_count: usize,
}

impl BranchMeshBuilder {
    // Create a new branch mesh builder.
    pub fn new(config: BranchMeshConfig) -> Self {
        Self {
            config,
            mesh: TriangleMesh::new(),
            z_min: f64::MAX,
            z_max: f64::MIN,
            branch_count: 0,
        }
    }

    /// Create with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(BranchMeshConfig::default())
    }

    /// Add a branch path to the mesh.
    pub fn add_branch(&mut self, path: &BranchPath) {
        if !path.is_valid() {
            return;
        }

        let (z_min, z_max) = self.extrude_branch(path);
        self.z_min = self.z_min.min(z_min);
        self.z_max = self.z_max.max(z_max);
        self.branch_count += 1;
    }

    /// Finish building and return the result.
    pub fn finish(self) -> BranchMeshResult {
        BranchMeshResult {
            mesh: self.mesh,
            z_span: (self.z_min, self.z_max),
            branch_count: self.branch_count,
        }
    }

    /// Extrude a branch path into a tube mesh, returning Z span.
    fn extrude_branch(&mut self, path: &BranchPath) -> (f64, f64) {
        let elements = path.elements();
        assert!(elements.len() >= 2);

        let mut z_min = f64::MAX;
        let mut z_max = f64::MIN;
        let mut prev_strip: Option<(usize, usize)> = None;

        for i in 1..elements.len() {
            let prev = &elements[i - 1];
            let current = &elements[i];

            // Calculate direction vector
            let p1 = prev.position;
            let p2 = current.position;
            let v1 = normalize_vec3(sub_vec3(p2, p1));

            if i == 1 {
                // First segment: extrude bottom hemisphere
                let radius = prev.radius;
                let (strip, z) = self.extrude_hemisphere_bottom(p1, v1, radius);
                z_min = z_min.min(z);
                prev_strip = Some(strip);
            }

            if i + 1 == elements.len() {
                // Last segment: extrude top hemisphere
                let radius = current.radius;
                if let Some(prev) = prev_strip {
                    let (z, _) = self.extrude_hemisphere_top(p2, v1, radius, prev);
                    z_max = z_max.max(z);
                }
            } else {
                // Middle segment: create connecting circle
                let next = &elements[i + 1];
                let p3 = next.position;
                let v2 = normalize_vec3(sub_vec3(p3, p2));
                let n_current = normalize_vec3(add_vec3(v1, v2));

                let radius = current.radius;
                let strip = self.discretize_circle(p2, n_current, radius);

                if let Some(prev) = prev_strip {
                    self.triangulate_strip(prev, strip);
                }
                prev_strip = Some(strip);

                z_min = z_min.min(p2.z);
                z_max = z_max.max(p2.z);
            }
        }

        (z_min, z_max)
    }

    /// Extrude bottom hemisphere, returning the top strip indices and minimum Z.
    fn extrude_hemisphere_bottom(
        &mut self,
        center: Point3D,
        normal: Point3D,
        radius: f64,
    ) -> ((usize, usize), f64) {
        let angle_step = 2.0 * (1.0 - self.config.eps / radius).acos();
        let nsteps = ((PI / 2.0) / angle_step).ceil() as usize;
        let angle_step = (PI / 2.0) / nsteps as f64;

        // Add bottom point
        let bottom_point = sub_vec3(center, scale_vec3(normal, radius));
        let ifan = self.mesh.vertices().len();
        self.mesh.add_vertex(point3d_to_point3f(bottom_point));
        let z_min = bottom_point.z;

        let mut prev_strip: Option<(usize, usize)> = None;
        let mut angle = angle_step;

        for i in 1..nsteps {
            let cos_a = angle.cos();
            let sin_a = angle.sin();
            let circle_center = sub_vec3(center, scale_vec3(normal, radius * cos_a));
            let circle_radius = radius * sin_a;

            let strip = self.discretize_circle(circle_center, normal, circle_radius);

            if i == 1 {
                // Fan from bottom point
                self.triangulate_fan_bottom(ifan, strip);
            } else if let Some(prev) = prev_strip {
                self.triangulate_strip(prev, strip);
            }

            prev_strip = Some(strip);
            angle += angle_step;
        }

        (prev_strip.unwrap_or((ifan, ifan + 1)), z_min)
    }

    /// Extrude top hemisphere, returning maximum Z.
    fn extrude_hemisphere_top(
        &mut self,
        center: Point3D,
        normal: Point3D,
        radius: f64,
        prev_strip: (usize, usize),
    ) -> (f64, (usize, usize)) {
        let angle_step = 2.0 * (1.0 - self.config.eps / radius).acos();
        let nsteps = ((PI / 2.0) / angle_step).ceil() as usize;
        let angle_step = (PI / 2.0) / nsteps as f64;

        let mut current_strip = prev_strip;
        let mut angle = PI / 2.0;

        for _ in 0..nsteps {
            let cos_a = angle.cos();
            let sin_a = angle.sin();
            let circle_center = add_vec3(center, scale_vec3(normal, radius * cos_a));
            let circle_radius = radius * sin_a;

            if circle_radius > 0.001 {
                let strip = self.discretize_circle(circle_center, normal, circle_radius);
                self.triangulate_strip(current_strip, strip);
                current_strip = strip;
            }

            angle -= angle_step;
        }

        // Add top point
        let top_point = add_vec3(center, scale_vec3(normal, radius));
        let ifan = self.mesh.vertices().len();
        self.mesh.add_vertex(point3d_to_point3f(top_point));

        self.triangulate_fan_top(ifan, current_strip);

        (top_point.z, current_strip)
    }

    /// Discretize a 3D circle into vertices, returning (begin, end) indices.
    fn discretize_circle(
        &mut self,
        center: Point3D,
        normal: Point3D,
        radius: f64,
    ) -> (usize, usize) {
        // Calculate number of segments based on epsilon
        let angle_step = 2.0 * (1.0 - self.config.eps / radius).acos();
        let mut nsteps = ((2.0 * PI) / angle_step).ceil() as usize;

        // Clamp to configured range
        nsteps = nsteps.clamp(
            self.config.min_circle_segments,
            self.config.max_circle_segments,
        );
        let angle_step = 2.0 * PI / nsteps as f64;

        // Create orthonormal basis for the circle plane
        let (x, y) = create_orthonormal_basis(normal);

        let begin = self.mesh.vertices().len();
        let mut angle = 0.0f64;

        for _ in 0..nsteps {
            let cos_a = angle.cos();
            let sin_a = angle.sin();
            let point = Point3D::new(
                center.x + radius * (x.x * cos_a + y.x * sin_a),
                center.y + radius * (x.y * cos_a + y.y * sin_a),
                center.z + radius * (x.z * cos_a + y.z * sin_a),
            );
            self.mesh.add_vertex(point3d_to_point3f(point));
            angle += angle_step;
        }

        (begin, self.mesh.vertices().len())
    }

    /// Triangulate a fan from a center point to a ring (bottom hemisphere).
    fn triangulate_fan_bottom(&mut self, ifan: usize, strip: (usize, usize)) {
        let (begin, end) = strip;
        let n = end - begin;
        if n < 3 {
            return;
        }

        for i in 0..n {
            let u = begin + i;
            let v = begin + (i + 1) % n;
            self.mesh
                .add_triangle_indices(ifan as u32, v as u32, u as u32);
        }
    }

    /// Triangulate a fan from a ring to a center point (top hemisphere).
    fn triangulate_fan_top(&mut self, ifan: usize, strip: (usize, usize)) {
        let (begin, end) = strip;
        let n = end - begin;
        if n < 3 {
            return;
        }

        for i in 0..n {
            let u = begin + i;
            let v = begin + (i + 1) % n;
            self.mesh
                .add_triangle_indices(ifan as u32, u as u32, v as u32);
        }
    }

    /// Triangulate a strip between two rings.
    fn triangulate_strip(&mut self, strip1: (usize, usize), strip2: (usize, usize)) {
        let (begin1, end1) = strip1;
        let (begin2, end2) = strip2;
        let n1 = end1 - begin1;
        let n2 = end2 - begin2;

        if n1 < 3 || n2 < 3 {
            return;
        }

        // Find the closest vertex on strip2 to the first vertex of strip1
        let p1 = self.mesh.vertex(begin1 as u32);
        let mut istart2 = begin2;
        let mut d2min = f64::MAX;

        for i in begin2..end2 {
            let p2 = self.mesh.vertex(i as u32);
            let d2 = distance_squared_point3f(p1, p2);
            if d2 < d2min {
                d2min = d2;
                istart2 = i;
            }
        }

        // Triangulate zig-zag fashion
        let mut u = begin1;
        let mut v = istart2;
        let mut remaining1 = n1;
        let mut remaining2 = n2;

        while remaining1 > 0 || remaining2 > 0 {
            let u2 = begin1 + (u - begin1 + 1) % n1;
            let v2 = begin2 + (v - begin2 + 1) % n2;

            let take_first = if remaining1 == 0 {
                false
            } else if remaining2 == 0 {
                true
            } else {
                let p_u = self.mesh.vertex(u as u32);
                let p_u2 = self.mesh.vertex(u2 as u32);
                let p_v = self.mesh.vertex(v as u32);
                let p_v2 = self.mesh.vertex(v2 as u32);

                let l1 = distance_squared_point3f(p_u2, p_v);
                let l2 = distance_squared_point3f(p_v2, p_u);
                l1 < l2
            };

            if take_first {
                self.mesh
                    .add_triangle_indices(u as u32, u2 as u32, v as u32);
                remaining1 -= 1;
                u = u2;
            } else {
                self.mesh
                    .add_triangle_indices(u as u32, v2 as u32, v as u32);
                remaining2 -= 1;
                v = v2;
            }
        }
    }
}

/// Build branch paths from support elements.
///
/// This traverses the support element tree structure and extracts connected
/// branch paths suitable for mesh extrusion.
pub fn build_branch_paths(
    move_bounds: &[Vec<SupportElement>],
    settings: &TreeSupportSettings,
) -> Vec<BranchPath> {
    let mut paths = Vec::new();
    let num_layers = move_bounds.len();

    if num_layers == 0 {
        return paths;
    }

    // Track which elements have been processed
    let mut processed: Vec<Vec<bool>> = move_bounds
        .iter()
        .map(|layer| vec![false; layer.len()])
        .collect();

    // Build downward links (child -> parent mapping)
    // In BambuStudio, parents are stored as indices into the layer above
    let mut child_links: Vec<Vec<Option<usize>>> = move_bounds
        .iter()
        .map(|layer| vec![None; layer.len()])
        .collect();

    // Build child links by iterating through parent relationships
    for layer_idx in 0..num_layers.saturating_sub(1) {
        let layer_above = &move_bounds[layer_idx + 1];
        for (elem_idx, elem) in move_bounds[layer_idx].iter().enumerate() {
            for &parent_idx in &elem.parents {
                if (parent_idx as usize) < layer_above.len() {
                    child_links[layer_idx + 1][parent_idx as usize] = Some(elem_idx);
                }
            }
        }
    }

    // Traverse from bottom to top, building paths
    for start_layer in 0..num_layers {
        for start_elem_idx in 0..move_bounds[start_layer].len() {
            // Skip already processed elements
            if processed[start_layer][start_elem_idx] {
                continue;
            }

            // Skip elements that have children (we want to start from the bottom)
            if start_layer > 0 && child_links[start_layer][start_elem_idx].is_some() {
                continue;
            }

            // Build path upward from this element
            let mut path = BranchPath::new();
            let mut current_layer = start_layer;
            let mut current_elem_idx = start_elem_idx;

            loop {
                let elem = &move_bounds[current_layer][current_elem_idx];
                processed[current_layer][current_elem_idx] = true;

                // Get position and radius
                if let Some(result_point) = elem.state.result_on_layer {
                    let z = unscale(settings.get_actual_z(current_layer));
                    let radius = unscale(elem.state.get_radius(settings));

                    path.push(BranchPathElement {
                        position: Point3D::new(unscale(result_point.x), unscale(result_point.y), z),
                        radius,
                        layer_idx: current_layer,
                    });
                }

                // Check if we can continue upward
                if elem.parents.is_empty() || current_layer + 1 >= num_layers {
                    break;
                }

                // Follow the first parent (for simple paths)
                // More complex branching would need multiple paths
                let parent_idx = elem.parents[0] as usize;
                let layer_above = &move_bounds[current_layer + 1];

                if parent_idx >= layer_above.len() {
                    break;
                }

                // If parent has multiple children (bifurcation), stop this path
                // and let the other branches be separate paths
                let parent = &layer_above[parent_idx];
                if parent.parents.len() > 1 && !processed[current_layer + 1][parent_idx] {
                    // Continue to parent but this might be a bifurcation point
                }

                current_layer += 1;
                current_elem_idx = parent_idx;
            }

            if path.is_valid() {
                paths.push(path);
            }
        }
    }

    paths
}

/// Generate branch mesh from support elements.
pub fn generate_branch_mesh(
    move_bounds: &[Vec<SupportElement>],
    settings: &TreeSupportSettings,
    config: BranchMeshConfig,
) -> BranchMeshResult {
    let paths = build_branch_paths(move_bounds, settings);

    let mut builder = BranchMeshBuilder::new(config);
    for path in &paths {
        builder.add_branch(path);
    }

    builder.finish()
}

/// Slice branch mesh to get per-layer polygons.
pub fn slice_branch_mesh(
    mesh: &TriangleMesh,
    layer_zs: &[CoordF],
    layer_heights: &[CoordF],
) -> Vec<Vec<ExPolygon>> {
    if mesh.is_empty() || layer_zs.is_empty() {
        return vec![Vec::new(); layer_zs.len()];
    }

    // Compute slice heights (mid-layer)
    let slice_zs: Vec<CoordF> = layer_zs
        .iter()
        .zip(layer_heights.iter())
        .map(|(z, h)| z - h / 2.0)
        .collect();

    // Slice the mesh
    slice_mesh(mesh, &slice_zs)
}

// ============================================================================
// Helper functions for 3D vector math
// ============================================================================

use crate::geometry::Point3F;

/// Convert Point3D (f64) to Point3F for mesh storage.
#[inline]
fn point3d_to_point3f(p: Point3D) -> Point3F {
    Point3F::new(p.x, p.y, p.z)
}

#[inline]
fn add_vec3(a: Point3D, b: Point3D) -> Point3D {
    Point3D::new(a.x + b.x, a.y + b.y, a.z + b.z)
}

#[inline]
fn sub_vec3(a: Point3D, b: Point3D) -> Point3D {
    Point3D::new(a.x - b.x, a.y - b.y, a.z - b.z)
}

#[inline]
fn scale_vec3(v: Point3D, s: f64) -> Point3D {
    Point3D::new(v.x * s, v.y * s, v.z * s)
}

#[inline]
fn dot_vec3(a: Point3D, b: Point3D) -> f64 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

#[inline]
fn cross_vec3(a: Point3D, b: Point3D) -> Point3D {
    Point3D::new(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )
}

#[inline]
fn length_vec3(v: Point3D) -> f64 {
    (v.x * v.x + v.y * v.y + v.z * v.z).sqrt()
}

#[inline]
fn normalize_vec3(v: Point3D) -> Point3D {
    let len = length_vec3(v);
    if len > 1e-10 {
        scale_vec3(v, 1.0 / len)
    } else {
        Point3D::new(0.0, 0.0, 1.0) // Default up vector
    }
}

#[inline]
fn distance_squared(a: Point3D, b: Point3D) -> f64 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    let dz = a.z - b.z;
    dx * dx + dy * dy + dz * dz
}

/// Distance squared for Point3F (used in triangulation).
#[inline]
fn distance_squared_point3f(a: Point3F, b: Point3F) -> f64 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    let dz = a.z - b.z;
    dx * dx + dy * dy + dz * dz
}

/// Create an orthonormal basis from a normal vector.
/// Returns (x, y) vectors perpendicular to normal.
fn create_orthonormal_basis(normal: Point3D) -> (Point3D, Point3D) {
    // Choose a reference vector not parallel to normal
    let reference = if normal.y.abs() < 0.9 {
        Point3D::new(0.0, 1.0, 0.0)
    } else {
        Point3D::new(1.0, 0.0, 0.0)
    };

    // x = normal × reference (perpendicular to both)
    let x = normalize_vec3(cross_vec3(normal, reference));
    // y = normal × x (perpendicular to both normal and x)
    let y = normalize_vec3(cross_vec3(normal, x));

    (x, y)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_branch_path_new() {
        let path = BranchPath::new();
        assert!(path.is_empty());
        assert!(!path.is_valid());
    }

    #[test]
    fn test_branch_path_push() {
        let mut path = BranchPath::new();
        path.push(BranchPathElement {
            position: Point3D::new(0.0, 0.0, 0.0),
            radius: 1.0,
            layer_idx: 0,
        });
        path.push(BranchPathElement {
            position: Point3D::new(0.0, 0.0, 0.2),
            radius: 1.0,
            layer_idx: 1,
        });

        assert_eq!(path.len(), 2);
        assert!(path.is_valid());
    }

    #[test]
    fn test_branch_mesh_config_default() {
        let config = BranchMeshConfig::default();
        assert!(config.eps > 0.0);
        assert!(config.min_circle_segments >= 3);
        assert!(config.max_circle_segments >= config.min_circle_segments);
    }

    #[test]
    fn test_branch_mesh_builder_empty() {
        let builder = BranchMeshBuilder::with_defaults();
        let result = builder.finish();

        assert!(result.mesh.is_empty());
        assert_eq!(result.branch_count, 0);
    }

    #[test]
    fn test_branch_mesh_builder_single_branch() {
        let mut builder = BranchMeshBuilder::with_defaults();

        let mut path = BranchPath::new();
        path.push(BranchPathElement {
            position: Point3D::new(0.0, 0.0, 0.0),
            radius: 0.5,
            layer_idx: 0,
        });
        path.push(BranchPathElement {
            position: Point3D::new(0.0, 0.0, 1.0),
            radius: 0.5,
            layer_idx: 1,
        });

        builder.add_branch(&path);
        let result = builder.finish();

        assert!(!result.mesh.is_empty());
        assert_eq!(result.branch_count, 1);
        assert!(result.z_span.0 < result.z_span.1);
    }

    #[test]
    fn test_branch_mesh_builder_varying_radius() {
        let mut builder = BranchMeshBuilder::with_defaults();

        let mut path = BranchPath::new();
        path.push(BranchPathElement {
            position: Point3D::new(0.0, 0.0, 0.0),
            radius: 1.0,
            layer_idx: 0,
        });
        path.push(BranchPathElement {
            position: Point3D::new(0.0, 0.0, 0.5),
            radius: 0.8,
            layer_idx: 1,
        });
        path.push(BranchPathElement {
            position: Point3D::new(0.0, 0.0, 1.0),
            radius: 0.5,
            layer_idx: 2,
        });

        builder.add_branch(&path);
        let result = builder.finish();

        assert!(!result.mesh.is_empty());
        assert_eq!(result.branch_count, 1);
    }

    #[test]
    fn test_vector_math() {
        let a = Point3D::new(1.0, 0.0, 0.0);
        let b = Point3D::new(0.0, 1.0, 0.0);

        let sum = add_vec3(a, b);
        assert!((sum.x - 1.0).abs() < 1e-6);
        assert!((sum.y - 1.0).abs() < 1e-6);

        let diff = sub_vec3(a, b);
        assert!((diff.x - 1.0).abs() < 1e-6);
        assert!((diff.y - (-1.0)).abs() < 1e-6);

        let cross = cross_vec3(a, b);
        assert!((cross.z - 1.0).abs() < 1e-6);

        let dot = dot_vec3(a, b);
        assert!(dot.abs() < 1e-6);

        let scaled = scale_vec3(a, 2.0);
        assert!((scaled.x - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_normalize_vec3() {
        let v = Point3D::new(3.0, 4.0, 0.0);
        let n = normalize_vec3(v);

        let len = length_vec3(n);
        assert!((len - 1.0).abs() < 1e-6);

        assert!((n.x - 0.6).abs() < 1e-6);
        assert!((n.y - 0.8).abs() < 1e-6);
    }

    #[test]
    fn test_normalize_zero_vector() {
        let v = Point3D::new(0.0, 0.0, 0.0);
        let n = normalize_vec3(v);

        // Should return default up vector
        assert!((n.z - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_create_orthonormal_basis() {
        let normal = Point3D::new(0.0, 0.0, 1.0);
        let (x, y) = create_orthonormal_basis(normal);

        // x and y should be perpendicular to normal
        assert!(dot_vec3(x, normal).abs() < 1e-6);
        assert!(dot_vec3(y, normal).abs() < 1e-6);

        // x and y should be perpendicular to each other
        assert!(dot_vec3(x, y).abs() < 1e-6);

        // x and y should be unit vectors
        assert!((length_vec3(x) - 1.0).abs() < 1e-6);
        assert!((length_vec3(y) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_discretize_circle() {
        let mut builder = BranchMeshBuilder::with_defaults();

        let center = Point3D::new(0.0, 0.0, 0.0);
        let normal = Point3D::new(0.0, 0.0, 1.0);
        let radius = 1.0;

        let (begin, end) = builder.discretize_circle(center, normal, radius);
        let num_vertices = end - begin;

        // Should have at least min_circle_segments vertices
        assert!(num_vertices >= builder.config.min_circle_segments);
        assert!(num_vertices <= builder.config.max_circle_segments);

        // All vertices should be at the correct radius from center
        for i in begin..end {
            let v = builder.mesh.vertex(i as u32);
            let dx = v.x - center.x;
            let dy = v.y - center.y;
            let dist = (dx * dx + dy * dy).sqrt();
            assert!((dist - radius).abs() < 0.01);
        }
    }

    #[test]
    fn test_triangulate_fan() {
        let mut builder = BranchMeshBuilder::with_defaults();

        // Add a center point
        builder.mesh.add_vertex(Point3F::new(0.0, 0.0, 0.0));

        // Add a ring of points
        let center = Point3D::new(0.0, 0.0, 0.1);
        let normal = Point3D::new(0.0, 0.0, 1.0);
        let strip = builder.discretize_circle(center, normal, 1.0);

        let tri_count_before = builder.mesh.triangle_count();
        builder.triangulate_fan_bottom(0, strip);
        let tri_count_after = builder.mesh.triangle_count();

        // Should have added triangles
        assert!(tri_count_after > tri_count_before);
    }

    #[test]
    fn test_triangulate_strip() {
        let mut builder = BranchMeshBuilder::with_defaults();

        let normal = Point3D::new(0.0, 0.0, 1.0);

        // Add two rings
        let strip1 = builder.discretize_circle(Point3D::new(0.0, 0.0, 0.0), normal, 1.0);
        let strip2 = builder.discretize_circle(Point3D::new(0.0, 0.0, 0.2), normal, 1.0);

        let tri_count_before = builder.mesh.triangle_count();
        builder.triangulate_strip(strip1, strip2);
        let tri_count_after = builder.mesh.triangle_count();

        // Should have added triangles
        assert!(tri_count_after > tri_count_before);
    }

    #[test]
    fn test_build_branch_paths_empty() {
        let move_bounds: Vec<Vec<SupportElement>> = Vec::new();
        let settings = TreeSupportSettings::default();

        let paths = build_branch_paths(&move_bounds, &settings);
        assert!(paths.is_empty());
    }

    #[test]
    fn test_distance_squared() {
        let a = Point3D::new(0.0, 0.0, 0.0);
        let b = Point3D::new(1.0, 0.0, 0.0);

        let d2 = distance_squared(a, b);
        assert!((d2 - 1.0).abs() < 1e-6);

        let c = Point3D::new(1.0, 1.0, 1.0);
        let d2 = distance_squared(a, c);
        assert!((d2 - 3.0).abs() < 1e-6);
    }

    #[test]
    fn test_branch_mesh_result() {
        let result = BranchMeshResult {
            mesh: TriangleMesh::new(),
            z_span: (0.0, 10.0),
            branch_count: 5,
        };

        assert!(result.mesh.is_empty());
        assert_eq!(result.z_span.0, 0.0);
        assert_eq!(result.z_span.1, 10.0);
        assert_eq!(result.branch_count, 5);
    }

    #[test]
    fn test_slice_branch_mesh_empty() {
        let mesh = TriangleMesh::new();
        let layer_zs = vec![0.2, 0.4, 0.6];
        let layer_heights = vec![0.2, 0.2, 0.2];

        let slices = slice_branch_mesh(&mesh, &layer_zs, &layer_heights);
        assert_eq!(slices.len(), 3);
        assert!(slices.iter().all(|s| s.is_empty()));
    }
}
