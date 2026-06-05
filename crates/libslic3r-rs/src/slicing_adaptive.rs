//! 1:1 faithful port of `src/libslic3r/SlicingAdaptive.{hpp,cpp}` (BambuStudio).
//!
//! Adaptive slicing: vary layer height based on the surface slope of the mesh.
//!
//! Based on the work of Florens Waserfall (@platch on github)
//! and his paper
//! Florens Wasserfall, Norman Hendrich, Jianwei Zhang:
//! Adaptive Slicing for the FDM Process Revisited
//! 13th IEEE Conference on Automation Science and Engineering (CASE-2017), August 20-23, Xi'an, China. DOI: 10.1109/COASE.2017.8256074
//! https://tams.informatik.uni-hamburg.de/publications/2017/Adaptive%20Slicing%20for%20the%20FDM%20Process%20Revisited.pdf
//!
//! # BambuStudio Reference
//!
//! - `src/libslic3r/SlicingAdaptive.hpp`
//! - `src/libslic3r/SlicingAdaptive.cpp`

// SlicingAdaptive.cpp:1-4 includes (libslic3r.h, Model.hpp, TriangleMesh.hpp, SlicingAdaptive.hpp)
use crate::model::ModelObject;
use crate::slicing::SlicingParams;
use crate::libslic3r::EPSILON;

// Vojtech believes that there is a bug in @platch's derivation of the triangle area error metric.
// Following Octave code paints graphs of recommended layer height versus surface slope angle.
// SlicingAdaptive.cpp:16-29 (Octave snippet, #if 0)

// SlicingAdaptive.cpp:31-33
// #ifndef NDEBUG
//     #define ADAPTIVE_LAYER_HEIGHT_DEBUG
// #endif /* NDEBUG */

// By Florens Waserfall aka @platch:
// This constant essentially describes the volumetric error at the surface which is induced
// by stacking "elliptic" extrusion threads. It is empirically determined by
// 1. measuring the surface profile of printed parts to find
// the ratio between layer height and profile height and then
// 2. computing the geometric difference between the model-surface and the elliptic profile.
//
// The definition of the roughness formula is in
// https://tams.informatik.uni-hamburg.de/publications/2017/Adaptive%20Slicing%20for%20the%20FDM%20Process%20Revisited.pdf
// (page 51, formula (8))
// Currenty @platch's error metric formula is not used.
// SlicingAdaptive.cpp:49
//static constexpr const double SURFACE_CONST = 0.18403;

/// SlicingAdaptive.hpp:26-32
/// ```cpp
/// struct FaceZ {
///     std::pair<float, float> z_span;
///     // Cosine of the normal vector towards the Z axis.
///     float                   n_cos;
///     // Sine of the normal vector towards the Z axis.
///     float                   n_sin;
/// };
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FaceZ {
    /// SlicingAdaptive.hpp:27
    pub z_span: (f32, f32),
    /// Cosine of the normal vector towards the Z axis.
    /// SlicingAdaptive.hpp:29
    pub n_cos: f32,
    /// Sine of the normal vector towards the Z axis.
    /// SlicingAdaptive.hpp:31
    pub n_sin: f32,
}

// for a given facet, compute maximum height within the allowed surface roughness / stairstepping deviation
// SlicingAdaptive.cpp:52
fn layer_height_from_slope(face: &FaceZ, max_surface_deviation: f32) -> f32 {
    // @platch's formula, see his paper "Adaptive Slicing for the FDM Process Revisited".
    // SlicingAdaptive.cpp:54-55
    //    return float(max_surface_deviation / (SURFACE_CONST + 0.5 * std::abs(normal_z)));

    // Constant stepping in horizontal direction, as used by Cura.
    // SlicingAdaptive.cpp:57-58
    //    return (face.n_cos > 1e-5) ? float(max_surface_deviation * face.n_sin / face.n_cos) : FLT_MAX;

    // Constant error measured as an area of the surface error triangle, Vojtech's formula.
    // SlicingAdaptive.cpp:60-61
    //    return (face.n_cos > 1e-5) ? float(1.44 * max_surface_deviation * sqrt(face.n_sin / face.n_cos)) : FLT_MAX;

    // Constant error measured as an area of the surface error triangle, Vojtech's formula with clamping to roughness at 90 degrees.
    // SlicingAdaptive.cpp:63-64
    f32::min(
        max_surface_deviation / 0.184f32,
        if face.n_cos > 1e-5 {
            (1.44 * f64::from(max_surface_deviation) * f64::from(face.n_sin / face.n_cos).sqrt())
                as f32
        } else {
            f32::MAX
        },
    )

    // Constant stepping along the surface, equivalent to the "surface roughness" metric by Perez and later Pandey et all, see @platch's paper for references.
    // SlicingAdaptive.cpp:66-67
    //    return float(max_surface_deviation * face.n_sin);
}

/// SlicingAdaptive.hpp:14-38
/// ```cpp
/// class SlicingAdaptive { ... };
/// ```
#[derive(Debug, Default)]
pub struct SlicingAdaptive {
    /// SlicingAdaptive.hpp:35
    m_slicing_params: SlicingParams,
    /// SlicingAdaptive.hpp:37
    m_faces: Vec<FaceZ>,
}

impl SlicingAdaptive {
    // SlicingAdaptive.cpp:70
    pub fn clear(&mut self) {
        // SlicingAdaptive.cpp:72
        self.m_faces.clear();
    }

    /// SlicingAdaptive.hpp:18
    /// `void set_slicing_parameters(SlicingParameters params) { m_slicing_params = params; }`
    pub fn set_slicing_parameters(&mut self, params: SlicingParams) {
        self.m_slicing_params = params;
    }

    // SlicingAdaptive.cpp:75
    pub fn prepare(&mut self, object: &ModelObject) {
        // SlicingAdaptive.cpp:77
        self.clear();

        // SlicingAdaptive.cpp:79-81
        // TriangleMesh       mesh           = object.raw_mesh();
        // const ModelInstance &first_instance = *object.instances.front();
        // mesh.transform(first_instance.get_matrix(), first_instance.is_left_handed());
        //
        // DIVERGENCE: the Rust `ModelObject` stores a single merged `mesh` (no
        // `raw_mesh()` merging of multiple ModelVolumes), and `Instance` exposes
        // `transform()` rather than `get_matrix()`/`is_left_handed()`. The
        // is_left_handed winding flip is not modelled by the Rust transform.
        let mut mesh = object.mesh.clone();
        let first_instance = object.instances.first().expect("object.instances.front()");
        mesh.transform(&first_instance.transform());

        // 1) Collect faces from mesh.
        // SlicingAdaptive.cpp:84
        self.m_faces.reserve(mesh.indices().len());
        // SlicingAdaptive.cpp:85
        let vertices = mesh.vertices();
        for face in mesh.indices() {
            // SlicingAdaptive.cpp:86
            // stl_vertex vertex[3] = { mesh.its.vertices[face[0]], mesh.its.vertices[face[1]], mesh.its.vertices[face[2]] };
            let v0 = &vertices[face.indices[0] as usize];
            let v1 = &vertices[face.indices[1] as usize];
            let v2 = &vertices[face.indices[2] as usize];
            // SlicingAdaptive.cpp:87
            // stl_vertex n = face_normal_normalized(vertex);
            // face_normal_normalized(vertex) = ((v1 - v0).cross(v2 - v1)).normalized().normalized()
            let e1x = (v1.x - v0.x) as f32;
            let e1y = (v1.y - v0.y) as f32;
            let e1z = (v1.z - v0.z) as f32;
            let e2x = (v2.x - v1.x) as f32;
            let e2y = (v2.y - v1.y) as f32;
            let e2z = (v2.z - v1.z) as f32;
            // cross product e1 x e2
            let mut nx = e1y * e2z - e1z * e2y;
            let mut ny = e1z * e2x - e1x * e2z;
            let mut nz = e1x * e2y - e1y * e2x;
            // normalize (face_normal then face_normal_normalized -> normalize once is idempotent)
            let len = (nx * nx + ny * ny + nz * nz).sqrt();
            if len > 0.0 {
                nx /= len;
                ny /= len;
                nz /= len;
            }
            // SlicingAdaptive.cpp:88-91
            // std::pair<float, float> face_z_span {
            //     std::min(std::min(vertex[0].z(), vertex[1].z()), vertex[2].z()),
            //     std::max(std::max(vertex[0].z(), vertex[1].z()), vertex[2].z())
            // };
            let z0 = v0.z as f32;
            let z1 = v1.z as f32;
            let z2 = v2.z as f32;
            let face_z_span = (z0.min(z1).min(z2), z0.max(z1).max(z2));
            // SlicingAdaptive.cpp:92
            // m_faces.emplace_back(FaceZ({ face_z_span, std::abs(n.z()), std::sqrt(n.x() * n.x() + n.y() * n.y()) }));
            self.m_faces.push(FaceZ {
                z_span: face_z_span,
                n_cos: nz.abs(),
                n_sin: (nx * nx + ny * ny).sqrt(),
            });
        }

        // 2) Sort faces lexicographically by their Z span.
        // SlicingAdaptive.cpp:96
        // std::sort(m_faces.begin(), m_faces.end(), [](const FaceZ &f1, const FaceZ &f2) { return f1.z_span < f2.z_span; });
        self.m_faces.sort_by(|f1, f2| {
            // std::pair operator< : compares .first, then .second
            f1.z_span
                .0
                .partial_cmp(&f2.z_span.0)
                .unwrap()
                .then(f1.z_span.1.partial_cmp(&f2.z_span.1).unwrap())
        });
    }

    // current_facet is in/out parameter, rememebers the index of the last face of m_faces visited,
    // where this function will start from.
    // print_z - the top print surface of the previous layer.
    // returns height of the next layer.
    // SlicingAdaptive.cpp:103
    pub fn next_layer_height(&self, print_z: f32, quality_factor: f32, current_facet: &mut usize) -> f32 {
        // SlicingAdaptive.cpp:105
        let mut height = self.m_slicing_params.max_layer_height as f32;

        // SlicingAdaptive.cpp:107
        let max_surface_deviation: f32;

        {
            // SlicingAdaptive.cpp:110-114 (#if 0, @platch's formula for quality)
            //     double delta_min = SURFACE_CONST * m_slicing_params.min_layer_height;
            //     double delta_mid = (SURFACE_CONST + 0.5) * m_slicing_params.layer_height;
            //     double delta_max = (SURFACE_CONST + 0.5) * m_slicing_params.max_layer_height;
            // SlicingAdaptive.cpp:116-119
            // Vojtech's formula for triangle area error metric.
            let delta_min: f64 = self.m_slicing_params.min_layer_height;
            let delta_mid: f64 = self.m_slicing_params.layer_height;
            let delta_max: f64 = self.m_slicing_params.max_layer_height;
            // SlicingAdaptive.cpp:121-123
            max_surface_deviation = if quality_factor < 0.5f32 {
                lerp(delta_min, delta_mid, 2. * f64::from(quality_factor)) as f32
            } else {
                lerp(delta_max, delta_mid, 2. * (1. - f64::from(quality_factor))) as f32
            };
        }

        // find all facets intersecting the slice-layer
        // SlicingAdaptive.cpp:127
        let mut ordered_id = *current_facet;
        {
            // SlicingAdaptive.cpp:129
            let mut first_hit = false;
            // SlicingAdaptive.cpp:130
            while ordered_id < self.m_faces.len() {
                // SlicingAdaptive.cpp:131
                let zspan = self.m_faces[ordered_id].z_span;
                // facet's minimum is higher than slice_z -> end loop
                // SlicingAdaptive.cpp:133-134
                if zspan.0 >= print_z {
                    break;
                }
                // facet's maximum is higher than slice_z -> store the first event for next cusp_height call to begin at this point
                // SlicingAdaptive.cpp:136
                if zspan.1 > print_z {
                    // first event?
                    // SlicingAdaptive.cpp:138-141
                    if !first_hit {
                        first_hit = true;
                        *current_facet = ordered_id;
                    }
                    // skip touching facets which could otherwise cause small cusp values
                    // SlicingAdaptive.cpp:143-144
                    if zspan.1 < print_z + EPSILON as f32 {
                        ordered_id += 1;
                        continue;
                    }
                    // compute cusp-height for this facet and store minimum of all heights
                    // SlicingAdaptive.cpp:146
                    height = height.min(layer_height_from_slope(&self.m_faces[ordered_id], max_surface_deviation));
                }
                ordered_id += 1;
            }
        }

        // lower height limit due to printer capabilities
        // SlicingAdaptive.cpp:152
        height = height.max(self.m_slicing_params.min_layer_height as f32);

        // check for sloped facets inside the determined layer and correct height if necessary
        // SlicingAdaptive.cpp:155
        if height > self.m_slicing_params.min_layer_height as f32 {
            // SlicingAdaptive.cpp:156
            while ordered_id < self.m_faces.len() {
                // SlicingAdaptive.cpp:157
                let zspan = self.m_faces[ordered_id].z_span;
                // facet's minimum is higher than slice_z + height -> end loop
                // SlicingAdaptive.cpp:159-160
                if zspan.0 >= print_z + height {
                    break;
                }

                // skip touching facets which could otherwise cause small cusp values
                // SlicingAdaptive.cpp:163-164
                if zspan.1 < print_z + EPSILON as f32 {
                    ordered_id += 1;
                    continue;
                }

                // Compute cusp-height for this facet and check against height.
                // SlicingAdaptive.cpp:167
                let reduced_height = layer_height_from_slope(&self.m_faces[ordered_id], max_surface_deviation);

                // SlicingAdaptive.cpp:169
                let z_diff = zspan.0 - print_z;
                // SlicingAdaptive.cpp:170
                if reduced_height < z_diff {
                    // SlicingAdaptive.cpp:171
                    debug_assert!(z_diff < height + EPSILON as f32);
                    // The currently visited triangle's slope limits the next layer height so much, that
                    // the lowest point of the currently visible triangle is already above the newly proposed layer height.
                    // This means, that we need to limit the layer height so that the offending newly visited triangle
                    // is just above of the new layer.
                    // SlicingAdaptive.cpp:176-178 (ADAPTIVE_LAYER_HEIGHT_DEBUG trace)
                    // SlicingAdaptive.cpp:179
                    height = z_diff;
                } else if reduced_height < height {
                    // SlicingAdaptive.cpp:180
                    // SlicingAdaptive.cpp:181-183 (ADAPTIVE_LAYER_HEIGHT_DEBUG trace)
                    // SlicingAdaptive.cpp:184
                    height = reduced_height;
                }
                ordered_id += 1;
            }
            // lower height limit due to printer capabilities again
            // SlicingAdaptive.cpp:188
            height = height.max(self.m_slicing_params.min_layer_height as f32);
        }

        // SlicingAdaptive.cpp:191-193 (ADAPTIVE_LAYER_HEIGHT_DEBUG trace)
        // SlicingAdaptive.cpp:194
        height
    }

    // Returns the distance to the next horizontal facet in Z-dir
    // to consider horizontal object features in slice thickness
    // SlicingAdaptive.cpp:199
    pub fn horizontal_facet_distance(&self, z: f32) -> f32 {
        // SlicingAdaptive.cpp:201
        for i in 0..self.m_faces.len() {
            // SlicingAdaptive.cpp:202
            let zspan = self.m_faces[i].z_span;
            // facet's minimum is higher than max forward distance -> end loop
            // SlicingAdaptive.cpp:204-205
            if zspan.0 > z + self.m_slicing_params.max_layer_height as f32 {
                break;
            }
            // min_z == max_z -> horizontal facet
            // SlicingAdaptive.cpp:207-208
            if zspan.0 > z && zspan.0 == zspan.1 {
                return zspan.0 - z;
            }
        }

        // objects maximum?
        // SlicingAdaptive.cpp:212-213
        if z + self.m_slicing_params.max_layer_height as f32
            > self.m_slicing_params.object_print_z_height() as f32
        {
            f32::max(
                self.m_slicing_params.object_print_z_height() as f32 - z,
                0.0,
            )
        } else {
            self.m_slicing_params.max_layer_height as f32
        }
    }
}

// libslic3r.h:281 — constexpr inline T lerp(const T& a, const T& b, Number t)
// return (Number(1) - t) * a + t * b;
#[inline]
fn lerp(a: f64, b: f64, t: f64) -> f64 {
    debug_assert!(t >= -(EPSILON) && t <= 1.0 + EPSILON);
    (1.0 - t) * a + t * b
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::triangle_mesh::TriangleMesh;

    fn make_slicer(mesh: TriangleMesh) -> SlicingAdaptive {
        let mut object = ModelObject::new("test", mesh);
        let mut sa = SlicingAdaptive::default();
        let mut params = SlicingParams::default();
        params.object_print_z_max = 10.0;
        sa.set_slicing_parameters(params);
        object.instances = vec![crate::model::Instance::default()];
        sa.prepare(&object);
        sa
    }

    #[test]
    fn test_clear() {
        let mut sa = SlicingAdaptive::default();
        sa.m_faces.push(FaceZ {
            z_span: (0.0, 1.0),
            n_cos: 1.0,
            n_sin: 0.0,
        });
        sa.clear();
        assert!(sa.m_faces.is_empty());
    }

    #[test]
    fn test_layer_height_from_slope_horizontal() {
        // Horizontal facet: n_sin = 0 -> 1.44 * dev * sqrt(0) = 0; min(dev/0.184, 0) = 0
        let face = FaceZ {
            z_span: (1.0, 1.0),
            n_cos: 1.0,
            n_sin: 0.0,
        };
        let h = layer_height_from_slope(&face, 0.2);
        assert!((h - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_layer_height_from_slope_vertical_clamped() {
        // Vertical facet: n_cos = 0 -> FLT_MAX branch; min(dev/0.184, MAX) = dev/0.184
        let face = FaceZ {
            z_span: (0.0, 10.0),
            n_cos: 0.0,
            n_sin: 1.0,
        };
        let h = layer_height_from_slope(&face, 0.2);
        assert!((h - 0.2f32 / 0.184f32).abs() < 1e-6);
    }

    #[test]
    fn test_prepare_cube() {
        let mesh = TriangleMesh::cube(10.0);
        let sa = make_slicer(mesh);
        assert!(!sa.m_faces.is_empty());
        // Faces sorted lexicographically by z_span.
        for i in 1..sa.m_faces.len() {
            let a = sa.m_faces[i - 1].z_span;
            let b = sa.m_faces[i].z_span;
            assert!(a.0 < b.0 || (a.0 == b.0 && a.1 <= b.1));
        }
    }

    #[test]
    fn test_next_layer_height_bounds() {
        let mesh = TriangleMesh::cube(10.0);
        let sa = make_slicer(mesh);
        let mut current_facet = 0usize;
        let h = sa.next_layer_height(0.0, 0.5, &mut current_facet);
        assert!(h >= sa.m_slicing_params.min_layer_height as f32 - 1e-6);
        assert!(h <= sa.m_slicing_params.max_layer_height as f32 + 1e-6);
    }

    #[test]
    fn test_horizontal_facet_distance_max() {
        let mesh = TriangleMesh::cube(10.0);
        let sa = make_slicer(mesh);
        let d = sa.horizontal_facet_distance(0.0);
        assert!(d > 0.0);
    }

    #[test]
    fn test_lerp() {
        assert!((lerp(0.0, 10.0, 0.0) - 0.0).abs() < 1e-12);
        assert!((lerp(0.0, 10.0, 1.0) - 10.0).abs() < 1e-12);
        assert!((lerp(0.0, 10.0, 0.5) - 5.0).abs() < 1e-12);
    }
}
