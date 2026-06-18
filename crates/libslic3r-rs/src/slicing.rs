//! Slicing parameters and configuration.
//!
//! This module provides the SlicingParams type containing all configuration
//! needed for the slicing process, mirroring BambuStudio's SlicingParameters.

use crate::CoordF;
use serde::{Deserialize, Serialize};
use std::fmt;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
/// Slicing mode determines how the mesh is interpreted during slicing
/// TriangleMeshSlicer.hpp:25-30
pub enum SlicingMode {
    /// Regular slicing - maintains all contours and their orientation
    /// TriangleMeshSlicer.hpp:26
    #[default]
    Regular,
    /// Even-odd fill rule - for compatibility with certain model types
    /// TriangleMeshSlicer.hpp:27
    EvenOdd,
    /// Positive mode - orients all contours CCW, closes holes
    /// TriangleMeshSlicer.hpp:28
    Positive,
    /// Positive largest contour - keeps only the largest contour
    /// TriangleMeshSlicer.hpp:29
    PositiveLargestContour,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
/// Parameters controlling the slicing process
/// Slicing.hpp:29-105
pub struct SlicingParams {
    /// Whether the parameters have been initialized (from a config).
    /// Slicing.hpp:21
    pub valid: bool,

    /// Regular layer height (mm)
    /// Slicing.hpp:63
    pub layer_height: CoordF,

    /// Minimum layer height for variable layer height (mm)
    /// Slicing.hpp:67
    pub min_layer_height: CoordF,

    /// Maximum layer height for variable layer height (mm)
    /// Slicing.hpp:68
    pub max_layer_height: CoordF,

    // NOTE: mode/closing_radius/extra_offset/resolution are NOT part of C++
    // SlicingParameters (Slicing.hpp); they belong to MeshSlicingParams
    // (TriangleMeshSlicer.hpp) and will live on that struct when ported.

    /// Number of raft base layers
    /// Slicing.hpp:49
    pub base_raft_layers: usize,

    /// Number of raft interface layers
    /// Slicing.hpp:51
    pub interface_raft_layers: usize,

    /// Height of raft base layers (mm)
    /// Slicing.hpp:54
    pub base_raft_layer_height: CoordF,

    /// Height of raft interface layers (mm)
    /// Slicing.hpp:55
    pub interface_raft_layer_height: CoordF,

    /// Height of raft contact layer (mm)
    /// Slicing.hpp:56
    pub contact_raft_layer_height: CoordF,

    /// Whether the first object layer uses bridging flow over non-soluble raft
    /// Slicing.hpp:81
    pub first_object_layer_bridging: bool,

    /// Whether the support interface is soluble
    /// Slicing.hpp:85
    pub soluble_interface: bool,

    /// Gap between raft and object (mm)
    /// Slicing.hpp:87
    pub gap_raft_object: CoordF,

    /// Gap between object and support (mm)
    /// Slicing.hpp:89
    pub gap_object_support: CoordF,

    /// Gap between support and object (mm)
    /// Slicing.hpp:91
    pub gap_support_object: CoordF,

    // ----------------------------------------------------------------------
    // Fields mirroring C++ SlicingParameters (Slicing.hpp).
    // ----------------------------------------------------------------------
    /// Maximum support layer height.
    /// Slicing.hpp:66
    pub max_suport_layer_height: CoordF,

    /// First layer height of the print (may be used for the first raft layer
    /// or for the first print layer).
    /// Slicing.hpp:70
    pub first_print_layer_height: CoordF,

    /// Thickness of the first object layer. This is either the first print
    /// layer thickness (no raft), a bridging flow thickness (non-soluble raft),
    /// or a normal layer height (soluble raft).
    /// Slicing.hpp:75
    pub first_object_layer_height: CoordF,

    /// Top z of the raft base.
    /// Slicing.hpp:93
    pub raft_base_top_z: CoordF,
    /// Top z of the raft interface.
    /// Slicing.hpp:94
    pub raft_interface_top_z: CoordF,
    /// Top z of the raft contact layer.
    /// Slicing.hpp:95
    pub raft_contact_top_z: CoordF,

    /// Bottom of the printed object. 0 without a raft, else the raft height.
    /// Slicing.hpp:97
    pub object_print_z_min: CoordF,
    /// Top of the printed object.
    /// Slicing.hpp:98
    pub object_print_z_max: CoordF,
}

/// Implementation of SlicingParams methods
/// Slicing.hpp:29-44
impl SlicingParams {
    // Create new slicing parameters with default values
    // Slicing.hpp:31
    pub fn new() -> Self {
        Self::default()
    }

    /// Create slicing parameters with a specific layer height
    /// Slicing.hpp:31
    pub fn with_layer_height(layer_height: CoordF) -> Self {
        Self {
            layer_height,
            ..Default::default()
        }
    }

    /// Check if raft is enabled
    /// Slicing.hpp:40
    pub fn has_raft(&self) -> bool {
        self.raft_layers() > 0
    }

    /// Get the total number of raft layers
    /// Slicing.hpp:41
    pub fn raft_layers(&self) -> usize {
        self.base_raft_layers + self.interface_raft_layers
    }

    /// Check if the first object layer height is fixed
    /// Slicing.hpp:42
    pub fn first_object_layer_height_fixed(&self) -> bool {
        // Slicing.hpp:42
        !self.has_raft() || self.first_object_layer_bridging
    }

    /// Height of the object to be printed. This value does not contain the raft height.
    /// Slicing.hpp:45
    pub fn object_print_z_height(&self) -> CoordF {
        // Slicing.hpp:45
        self.object_print_z_max - self.object_print_z_min
    }
}

/// Default trait implementation for SlicingParams
/// Slicing.hpp:31
impl Default for SlicingParams {
    // Create default SlicingParams with standard values
    // Slicing.hpp:31
    fn default() -> Self {
        Self {
            valid: false,
            layer_height: 0.2,
            min_layer_height: 0.07,
            max_layer_height: 0.3,
            base_raft_layers: 0,
            interface_raft_layers: 0,
            base_raft_layer_height: 0.3,
            interface_raft_layer_height: 0.2,
            contact_raft_layer_height: 0.2,
            first_object_layer_bridging: false,
            soluble_interface: false,
            gap_raft_object: 0.1,
            gap_object_support: 0.2,
            gap_support_object: 0.2,
            max_suport_layer_height: 0.0,
            first_print_layer_height: 0.2,
            first_object_layer_height: 0.2,
            raft_base_top_z: 0.0,
            raft_interface_top_z: 0.0,
            raft_contact_top_z: 0.0,
            object_print_z_min: 0.0,
            object_print_z_max: 0.0,
        }
    }
}

/// Display trait implementation for SlicingParams
/// Slicing.hpp:29
impl fmt::Display for SlicingParams {
    // Format SlicingParams for display
    // Slicing.hpp:29
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "SlicingParams(layer_height={:.3}mm, first_layer={:.3}mm)",
            self.layer_height, self.first_print_layer_height
        )
    }
}

/// Check if two slicing parameter sets produce the same layering.
/// Slicing.hpp:103-131
pub fn equal_layering(sp1: &SlicingParams, sp2: &SlicingParams) -> bool {
    // Slicing.hpp:105-106
    debug_assert!(sp1.valid);
    debug_assert!(sp2.valid);
    // Slicing.hpp:107-130. Exact equality (matching C++). max_suport_layer_height
    // is commented out and the soluble/gap comparisons are behind `#if 0` upstream.
    sp1.base_raft_layers == sp2.base_raft_layers
        && sp1.interface_raft_layers == sp2.interface_raft_layers
        && sp1.base_raft_layer_height == sp2.base_raft_layer_height
        && sp1.interface_raft_layer_height == sp2.interface_raft_layer_height
        && sp1.contact_raft_layer_height == sp2.contact_raft_layer_height
        && sp1.layer_height == sp2.layer_height
        && sp1.min_layer_height == sp2.min_layer_height
        && sp1.max_layer_height == sp2.max_layer_height
        && sp1.first_print_layer_height == sp2.first_print_layer_height
        && sp1.first_object_layer_height == sp2.first_object_layer_height
        && sp1.first_object_layer_bridging == sp2.first_object_layer_bridging
        && sp1.raft_base_top_z == sp2.raft_base_top_z
        && sp1.raft_interface_top_z == sp2.raft_interface_top_z
        && sp1.raft_contact_top_z == sp2.raft_contact_top_z
        && sp1.object_print_z_min == sp2.object_print_z_min
}

use crate::libslic3r::EPSILON;

// FIDELITY-NOTE: three Slicing.cpp free functions are NOT ported here because they
// depend on config/mesh-range infrastructure that the crate models differently from
// C++ (per-extruder ConfigOption*Nullable with `.get_at(idx)`, t_layer_config_ranges):
//   - inline min/max_layer_height_from_nozzle (Slicing.cpp:29-60)
//   - SlicingParameters::create_from_config   (Slicing.cpp:62-160)
//   - layer_height_profile_from_ranges        (Slicing.cpp:164-235)
// layer_height_profile_adaptive (Slicing.cpp:237-330) IS ported below.

// Slicing.cpp:24
// Used by the not-yet-ported create_from_config; retained for parity.
#[allow(dead_code)]
const MIN_LAYER_HEIGHT: CoordF = 0.01;
// Slicing.cpp:25
// Used by the not-yet-ported create_from_config; retained for parity.
#[allow(dead_code)]
const MIN_LAYER_HEIGHT_DEFAULT: CoordF = 0.07;
// Slicing.cpp:26
const LAYER_HEIGHT_CHANGE_STEP: f64 = 0.04;

/// Linear interpolation, matching C++ `lerp(a, b, t) = (1 - t) * a + t * b`.
/// libslic3r.h:280-285
#[inline]
fn lerp(a: CoordF, b: CoordF, t: CoordF) -> CoordF {
    // libslic3r.h:284
    (1.0 - t) * a + t * b
}

/// `is_approx(value, test_value) = |value - test_value| < EPSILON`.
/// libslic3r.h:287-291
#[inline]
fn is_approx(value: CoordF, test_value: CoordF) -> bool {
    // libslic3r.h:290
    (value - test_value).abs() < EPSILON
}

/// Based on the work of @platsch.
/// Fill layer_height_profile by heights ensuring a prescribed maximum cusp height.
/// Slicing.cpp:237-330
pub fn layer_height_profile_adaptive(
    slicing_params: &SlicingParams,
    object: &crate::model::ModelObject,
    quality_factor: f32,
) -> Vec<f64> {
    // 1) Initialize the SlicingAdaptive class with the object meshes.
    // Slicing.cpp:242-244
    let mut as_ = crate::slicing_adaptive::SlicingAdaptive::default();
    as_.set_slicing_parameters(slicing_params.clone());
    as_.prepare(object);

    // 2) Generate layers using the algorithm of @platsch
    // Slicing.cpp:247
    let mut layer_height_profile: Vec<f64> = Vec::new();
    // Slicing.cpp:248-249
    layer_height_profile.push(0.0);
    layer_height_profile.push(slicing_params.first_object_layer_height);
    // Slicing.cpp:250
    if slicing_params.first_object_layer_height_fixed() {
        // Slicing.cpp:251-252
        layer_height_profile.push(slicing_params.first_object_layer_height);
        layer_height_profile.push(slicing_params.first_object_layer_height);
    }
    // Slicing.cpp:254
    let mut print_z = slicing_params.first_object_layer_height;
    // last facet visited by the as.next_layer_height() function, where the facets are sorted by their increasing Z span.
    // Slicing.cpp:256
    let mut current_facet: usize = 0;
    // loop until we have at least one layer and the max slice_z reaches the object height
    // Slicing.cpp:258
    while print_z + EPSILON < slicing_params.object_print_z_height() {
        // Slicing.cpp:259
        let mut height = slicing_params.max_layer_height as f32;
        // determine next layer height
        // Slicing.cpp:262
        let cusp_height = as_.next_layer_height(print_z as f32, quality_factor, &mut current_facet);

        // Slicing.cpp:264-288: horizontal feature check is behind `#if 0` upstream; omitted.

        // Slicing.cpp:289
        height = cusp_height.min(height);

        // Slicing.cpp:291-310: z-gradation and custom-range overrides are commented out upstream; omitted.

        //BBS: avoid the layer height change to be too steep
        // Slicing.cpp:312
        let last_h = *layer_height_profile.last().unwrap();
        if last_h < height as f64 && height as f64 - last_h > LAYER_HEIGHT_CHANGE_STEP {
            // Slicing.cpp:313
            height = (last_h + LAYER_HEIGHT_CHANGE_STEP) as f32;
        } else if last_h > height as f64 && last_h - height as f64 > LAYER_HEIGHT_CHANGE_STEP {
            // Slicing.cpp:315
            height = (last_h - LAYER_HEIGHT_CHANGE_STEP) as f32;
        }

        // Slicing.cpp:317-318
        layer_height_profile.push(print_z);
        layer_height_profile.push(height as f64);
        // Slicing.cpp:319
        print_z += height as f64;
    }

    // Slicing.cpp:322
    let z_gap =
        slicing_params.object_print_z_height() - layer_height_profile[layer_height_profile.len() - 2];
    // Slicing.cpp:323
    if z_gap > 0.0 {
        // Slicing.cpp:325
        layer_height_profile.push(slicing_params.object_print_z_height());
        // Slicing.cpp:326
        layer_height_profile
            .push(z_gap.clamp(slicing_params.min_layer_height, slicing_params.max_layer_height));
    }

    // Slicing.cpp:329
    layer_height_profile
}

/// Type of a layer-height editing action.
/// Slicing.hpp:157-162
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LayerHeightEditActionType {
    /// Slicing.hpp:158
    Increase = 0,
    /// Slicing.hpp:159
    Decrease = 1,
    /// Slicing.hpp:160
    Reduce = 2,
    /// Slicing.hpp:161
    Smooth = 3,
}

/// Parameters controlling the gaussian smoothing of a layer height profile.
/// Slicing.hpp:144-151
#[derive(Clone, Copy, Debug)]
pub struct HeightProfileSmoothingParams {
    /// Slicing.hpp:146
    pub radius: u32,
    /// Slicing.hpp:147
    pub keep_min: bool,
}

impl Default for HeightProfileSmoothingParams {
    // Slicing.hpp:149
    fn default() -> Self {
        Self {
            radius: 5,
            keep_min: false,
        }
    }
}

impl HeightProfileSmoothingParams {
    /// Slicing.hpp:149
    pub fn new() -> Self {
        // Slicing.hpp:149
        Self::default()
    }

    /// Slicing.hpp:150
    pub fn with_params(radius: u32, keep_min: bool) -> Self {
        // Slicing.hpp:150
        Self { radius, keep_min }
    }
}

/// Smooth a layer height profile with a biased gaussian blur, repeated up to 6 times.
/// Slicing.cpp:332-439
pub fn smooth_height_profile(
    profile: &[f64],
    slicing_params: &SlicingParams,
    smoothing_params: &HeightProfileSmoothingParams,
) -> Vec<f64> {
    // Slicing.cpp:335-352
    let gauss_kernel = |radius: u32| -> Vec<f64> {
        // Slicing.cpp:336
        let size = 2 * radius + 1;
        // Slicing.cpp:337
        let mut ret: Vec<f64> = Vec::with_capacity(size as usize);

        // Reworked from static inline int getGaussianKernelSize(float sigma) taken from opencv-4.1.2\modules\features2d\src\kaze\AKAZEFeatures.cpp
        // Slicing.cpp:341
        let sigma = 0.3 * (radius as f64 - 1.0) + 0.8;
        // Slicing.cpp:342
        let two_sq_sigma = 2.0 * sigma * sigma;
        // Slicing.cpp:343
        let inv_root_two_pi_sq_sigma = 1.0 / (std::f64::consts::PI * two_sq_sigma).sqrt();

        // Slicing.cpp:345
        for i in 0..size {
            // Slicing.cpp:347
            let x = i as f64 - radius as f64;
            // Slicing.cpp:348
            ret.push(inv_root_two_pi_sq_sigma * (-x * x / two_sq_sigma).exp());
        }

        ret
    };

    // Slicing.cpp:334-410
    let gauss_blur = |profile: &[f64], smoothing_params: &HeightProfileSmoothingParams| -> Vec<f64> {
        // skip first layer ?
        // Slicing.cpp:355
        let skip_count: usize = if slicing_params.first_object_layer_height_fixed() {
            4
        } else {
            0
        };

        // not enough data to smmoth
        // Slicing.cpp:358
        if profile.len() as i64 - (skip_count as i64) < 6 {
            return profile.to_vec();
        }

        // Slicing.cpp:361
        let radius = smoothing_params.radius.max(1);
        // Slicing.cpp:362
        let kernel = gauss_kernel(radius);
        // Slicing.cpp:363
        let two_radius = 2 * radius as i32;

        // Slicing.cpp:365-367
        let size = profile.len();
        let mut ret: Vec<f64> = Vec::with_capacity(size);

        // leave first layer untouched
        // Slicing.cpp:370
        for i in 0..skip_count {
            ret.push(profile[i]);
        }

        // smooth the rest of the profile by biasing a gaussian blur
        // the bias moves the smoothed profile closer to the min_layer_height
        // Slicing.cpp:377
        let delta_h = slicing_params.max_layer_height - slicing_params.min_layer_height;
        // Slicing.cpp:378
        let inv_delta_h = if delta_h != 0.0 { 1.0 / delta_h } else { 1.0 };

        // Slicing.cpp:380
        let max_dz_band = radius as f64 * slicing_params.layer_height;
        // Slicing.cpp:381
        let mut i = skip_count;
        while i < size {
            // Slicing.cpp:383
            let zi = profile[i];
            // Slicing.cpp:384
            let hi = profile[i + 1];
            // Slicing.cpp:385
            ret.push(zi);
            // Slicing.cpp:386
            ret.push(0.0);
            // Slicing.cpp:387: double& height = ret.back();
            let mut height = 0.0_f64;
            // Slicing.cpp:388
            let begin = ((i as i32) - two_radius).max(skip_count as i32);
            // Slicing.cpp:389
            let end = ((i as i32) + two_radius).min(size as i32 - 2);
            // Slicing.cpp:390
            let mut weight_total = 0.0_f64;
            // Slicing.cpp:391
            let mut j = begin;
            while j <= end {
                // Slicing.cpp:393
                let kernel_id = (radius as i32 + (j - i as i32) / 2) as usize;
                // Slicing.cpp:394
                let dz = (zi - profile[j as usize]).abs();
                // Slicing.cpp:395
                if dz * slicing_params.layer_height <= max_dz_band {
                    // Slicing.cpp:397
                    let dh = (slicing_params.max_layer_height - profile[j as usize + 1]).abs();
                    // Slicing.cpp:398
                    let weight = kernel[kernel_id] * (dh * inv_delta_h).sqrt();
                    // Slicing.cpp:399
                    height += weight * profile[j as usize + 1];
                    // Slicing.cpp:400
                    weight_total += weight;
                }
                // Slicing.cpp:391
                j += 2;
            }

            // Slicing.cpp:404
            height = (if weight_total == 0.0 {
                hi
            } else {
                height / weight_total
            })
            .clamp(slicing_params.min_layer_height, slicing_params.max_layer_height);
            // Slicing.cpp:405
            if smoothing_params.keep_min {
                // Slicing.cpp:406
                height = height.min(hi);
            }
            // Write back the computed height into the (z, height) pair.
            *ret.last_mut().unwrap() = height;

            // Slicing.cpp:381
            i += 2;
        }

        ret
    };

    // Slicing.cpp:429-437
    let mut count = 0;
    let mut ret = profile.to_vec();
    // bool has_steep_change = has_steep_height_change(ret, LAYER_HEIGHT_CHANGE_STEP);
    while /* has_steep_change && */ count < 6 {
        // Slicing.cpp:433
        ret = gauss_blur(&ret, smoothing_params);
        // has_steep_change = has_steep_height_change(ret, LAYER_HEIGHT_CHANGE_STEP);
        count += 1;
    }
    ret
}

/// Modify a layer height profile around `z` by `layer_thickness_delta`, weighted within a cosine band.
/// Slicing.cpp:441-628
pub fn adjust_layer_height_profile(
    slicing_params: &SlicingParams,
    layer_height_profile: &mut Vec<f64>,
    z: CoordF,
    mut layer_thickness_delta: CoordF,
    band_width: CoordF,
    action: LayerHeightEditActionType,
) {
    // Constrain the profile variability by the 1st layer height.
    // Slicing.cpp:450-453
    let z_span_variable: (CoordF, CoordF) = (
        if slicing_params.first_object_layer_height_fixed() {
            slicing_params.first_object_layer_height
        } else {
            0.0
        },
        slicing_params.object_print_z_height(),
    );
    // Slicing.cpp:454
    if z < z_span_variable.0 || z > z_span_variable.1 {
        return;
    }

    debug_assert!(layer_height_profile.len() >= 2); // Slicing.cpp:457
    debug_assert!(
        // Slicing.cpp:458
        (layer_height_profile[layer_height_profile.len() - 2] - slicing_params.object_print_z_height()).abs()
            < EPSILON
    );

    // 1) Get the current layer thickness at z.
    // Slicing.cpp:461
    let mut current_layer_height = slicing_params.layer_height;
    // Slicing.cpp:462
    let mut i = 0;
    while i < layer_height_profile.len() {
        // Slicing.cpp:463
        if i + 2 == layer_height_profile.len() {
            // Slicing.cpp:464
            current_layer_height = layer_height_profile[i + 1];
            break;
        } else if layer_height_profile[i + 2] > z {
            // Slicing.cpp:467-470
            let z1 = layer_height_profile[i];
            let h1 = layer_height_profile[i + 1];
            let z2 = layer_height_profile[i + 2];
            let h2 = layer_height_profile[i + 3];
            // Slicing.cpp:471
            current_layer_height = lerp(h1, h2, (z - z1) / (z2 - z1));
            break;
        }
        i += 2;
    }

    // 2) Is it possible to apply the delta?
    // Slicing.cpp:477
    match action {
        // Slicing.cpp:478-490
        LayerHeightEditActionType::Decrease | LayerHeightEditActionType::Increase => {
            if action == LayerHeightEditActionType::Decrease {
                // Slicing.cpp:479
                layer_thickness_delta = -layer_thickness_delta;
            }
            // Slicing.cpp:482
            if layer_thickness_delta > 0.0 {
                // Slicing.cpp:483
                if current_layer_height >= slicing_params.max_layer_height - EPSILON {
                    return;
                }
                // Slicing.cpp:485
                layer_thickness_delta =
                    layer_thickness_delta.min(slicing_params.max_layer_height - current_layer_height);
            } else {
                // Slicing.cpp:487
                if current_layer_height <= slicing_params.min_layer_height + EPSILON {
                    return;
                }
                // Slicing.cpp:489
                layer_thickness_delta =
                    layer_thickness_delta.max(slicing_params.min_layer_height - current_layer_height);
            }
        }
        // Slicing.cpp:492-498
        LayerHeightEditActionType::Reduce | LayerHeightEditActionType::Smooth => {
            // Slicing.cpp:494
            layer_thickness_delta = layer_thickness_delta.abs();
            // Slicing.cpp:495
            layer_thickness_delta =
                layer_thickness_delta.min((slicing_params.layer_height - current_layer_height).abs());
            // Slicing.cpp:496
            if layer_thickness_delta < EPSILON {
                return;
            }
        }
    }

    // 3) Densify the profile inside z +- band_width/2, remove duplicate Zs from the height profile inside the band.
    // Slicing.cpp:505
    let lo = z_span_variable.0.max(z - 0.5 * band_width);
    // Do not limit the upper side of the band, so that the modifications to the top point of the profile will be allowed.
    // Slicing.cpp:507
    let hi = z + 0.5 * band_width;
    // Slicing.cpp:508
    let z_step = 0.1;
    // Slicing.cpp:509
    let mut idx: usize = 0;
    // Slicing.cpp:510
    while idx < layer_height_profile.len() && layer_height_profile[idx] < lo {
        idx += 2;
    }
    // Slicing.cpp:512: size_t underflow when idx == 0 (lo <= profile[0]) wraps to
    // SIZE_MAX in C++; reproduce the wraparound so idx + 2 lands back at 0/1.
    idx = idx.wrapping_sub(2);

    // Slicing.cpp:514-515
    let mut profile_new: Vec<f64> = Vec::with_capacity(layer_height_profile.len());
    debug_assert!(idx.wrapping_add(1) < layer_height_profile.len()); // Slicing.cpp:516
    // Slicing.cpp:517
    profile_new.extend_from_slice(&layer_height_profile[..idx.wrapping_add(2)]);
    // Slicing.cpp:518
    let mut zz = lo;
    // Slicing.cpp:519
    let i_resampled_start = profile_new.len();
    // Slicing.cpp:520
    while zz < hi {
        // Slicing.cpp:521
        let next = idx + 2;
        // Slicing.cpp:522-523
        let z1 = layer_height_profile[idx];
        let h1 = layer_height_profile[idx + 1];
        // Slicing.cpp:524
        let mut height = h1;
        // Slicing.cpp:525
        if next < layer_height_profile.len() {
            // Slicing.cpp:526-527
            let z2 = layer_height_profile[next];
            let h2 = layer_height_profile[next + 1];
            // Slicing.cpp:528
            height = lerp(h1, h2, (zz - z1) / (z2 - z1));
        }
        // Adjust height by layer_thickness_delta.
        // Slicing.cpp:531
        let weight = if (zz - z).abs() < 0.5 * band_width {
            0.5 + 0.5 * (2.0 * std::f64::consts::PI * (zz - z) / band_width).cos()
        } else {
            0.0
        };
        // Slicing.cpp:532
        match action {
            // Slicing.cpp:533-536
            LayerHeightEditActionType::Increase | LayerHeightEditActionType::Decrease => {
                height += weight * layer_thickness_delta;
            }
            // Slicing.cpp:537-546
            LayerHeightEditActionType::Reduce => {
                // Slicing.cpp:539
                let delta = height - slicing_params.layer_height;
                // Slicing.cpp:540
                let mut step = weight * layer_thickness_delta;
                // Slicing.cpp:541
                step = if delta.abs() > step {
                    if delta > 0.0 {
                        -step
                    } else {
                        step
                    }
                } else {
                    -delta
                };
                // Slicing.cpp:544
                height += step;
            }
            // Slicing.cpp:547-551: Don't modify the profile during resampling process, do it at the next step.
            LayerHeightEditActionType::Smooth => {}
        }
        // Slicing.cpp:556
        height = height.clamp(slicing_params.min_layer_height, slicing_params.max_layer_height);
        // Slicing.cpp:557
        if zz == z_span_variable.1 {
            // This is the last point of the profile.
            // Slicing.cpp:559
            if profile_new[profile_new.len() - 2] + EPSILON > zz {
                // Slicing.cpp:560-561
                profile_new.pop();
                profile_new.pop();
            }
            // Slicing.cpp:563-564
            profile_new.push(zz);
            profile_new.push(height);
            // Slicing.cpp:565
            idx = layer_height_profile.len();
            break;
        }
        // Avoid entering a too short segment.
        // Slicing.cpp:569
        if profile_new[profile_new.len() - 2] + EPSILON < zz {
            // Slicing.cpp:570-571
            profile_new.push(zz);
            profile_new.push(height);
        }
        // Limit zz to the object height, so the next iteration the last profile point will be set.
        // Slicing.cpp:574
        zz = (zz + z_step).min(z_span_variable.1);
        // Slicing.cpp:575
        idx = next;
        // Slicing.cpp:576
        while idx < layer_height_profile.len() && layer_height_profile[idx] < zz {
            idx += 2;
        }
        // Slicing.cpp:578
        idx -= 2;
    }

    // Slicing.cpp:581
    idx += 2;
    debug_assert!(idx > 0); // Slicing.cpp:582
    // Slicing.cpp:583
    let i_resampled_end = profile_new.len();
    // Slicing.cpp:584
    if idx < layer_height_profile.len() {
        debug_assert!(zz >= layer_height_profile[idx - 2]); // Slicing.cpp:585
        debug_assert!(zz <= layer_height_profile[idx]); // Slicing.cpp:586
        // Slicing.cpp:587
        profile_new.extend_from_slice(&layer_height_profile[idx..]);
    }
    // Slicing.cpp:589
    else if profile_new[profile_new.len() - 2] + 0.5 * EPSILON < z_span_variable.1 {
        // Slicing.cpp:590
        let n = layer_height_profile.len();
        profile_new.extend_from_slice(&layer_height_profile[n - 2..]);
    }
    // Slicing.cpp:592
    *layer_height_profile = profile_new;

    // Slicing.cpp:594
    if action == LayerHeightEditActionType::Smooth {
        // Slicing.cpp:595-596
        let mut i_resampled_start = i_resampled_start;
        if i_resampled_start == 0 {
            i_resampled_start += 1;
        }
        // Slicing.cpp:597-598
        let mut i_resampled_end = i_resampled_end;
        if i_resampled_end == layer_height_profile.len() {
            i_resampled_end -= 2;
        }
        // Slicing.cpp:599
        let n_rounds = 6;
        // Slicing.cpp:600
        for _i_round in 0..n_rounds {
            // Slicing.cpp:601
            let profile_new = layer_height_profile.clone();
            // Slicing.cpp:602
            let mut i = i_resampled_start;
            while i < i_resampled_end {
                // Slicing.cpp:603
                let zz = profile_new[i];
                // Slicing.cpp:604
                let t = if (zz - z).abs() < 0.5 * band_width {
                    0.25 + 0.25 * (2.0 * std::f64::consts::PI * (zz - z) / band_width).cos()
                } else {
                    0.0
                };
                debug_assert!(t >= 0.0 && t <= 0.500_000_1); // Slicing.cpp:605
                // Slicing.cpp:606
                if i == 0 {
                    // Slicing.cpp:607
                    layer_height_profile[i + 1] =
                        (1.0 - t) * profile_new[i + 1] + t * profile_new[i + 3];
                } else if i + 1 == profile_new.len() {
                    // Slicing.cpp:609
                    layer_height_profile[i + 1] =
                        (1.0 - t) * profile_new[i + 1] + t * profile_new[i - 1];
                } else {
                    // Slicing.cpp:611
                    layer_height_profile[i + 1] = (1.0 - t) * profile_new[i + 1]
                        + 0.5 * t * (profile_new[i - 1] + profile_new[i + 3]);
                }
                i += 2;
            }
        }
    }

    debug_assert!(layer_height_profile.len() > 2); // Slicing.cpp:616
    debug_assert!(layer_height_profile.len() % 2 == 0); // Slicing.cpp:617
    debug_assert!(layer_height_profile[0] == 0.0); // Slicing.cpp:618
    debug_assert!(
        // Slicing.cpp:619
        (layer_height_profile[layer_height_profile.len() - 2] - slicing_params.object_print_z_height()).abs()
            < EPSILON
    );
}

/// Adjust the last 5 layers so the layer series ends exactly at the object height.
/// Slicing.cpp:630-721
pub fn adjust_layer_series_to_align_object_height(
    slicing_params: &SlicingParams,
    layer_series: &mut [f64],
) -> bool {
    // Slicing.cpp:632
    let object_height = slicing_params.object_print_z_height();
    // Slicing.cpp:633
    if is_approx(*layer_series.last().unwrap(), object_height) {
        return true;
    }

    // need at least 5 + 1(first_layer) layers to adjust the height
    // Slicing.cpp:637
    let layer_size = layer_series.len();
    // Slicing.cpp:638
    if layer_size < 12 {
        return false;
    }

    // Slicing.cpp:641-644
    let mut last_5_layers_height: Vec<CoordF> = Vec::new();
    for i in 0..5 {
        last_5_layers_height.push(
            layer_series[layer_size - 10 + 2 * i + 1] - layer_series[layer_size - 10 + 2 * i],
        );
    }

    // Slicing.cpp:646
    let mut gap = (*layer_series.last().unwrap() - object_height).abs();
    // to record whether every layer can adjust layer height
    // Slicing.cpp:647
    let mut can_adjust = [true; 5];
    // Slicing.cpp:648
    let taller_than_object = *layer_series.last().unwrap() < object_height;

    // Slicing.cpp:650-656
    let get_valid_size = |can_adjust: &[bool; 5]| -> i32 {
        let mut valid_size = 0;
        for &b_adjust in can_adjust.iter() {
            valid_size += if b_adjust { 1 } else { 0 };
        }
        valid_size
    };

    // Slicing.cpp:658-697
    let adjust_layer_height = |gap: CoordF,
                               last_5_layers_height: &mut [CoordF],
                               can_adjust: &mut [bool; 5]|
     -> CoordF {
        // Slicing.cpp:659
        let delta_gap = gap / get_valid_size(can_adjust) as CoordF;
        // Slicing.cpp:660
        let mut remain_gap = 0.0;
        // Slicing.cpp:661
        for i in 0..last_5_layers_height.len() {
            // Slicing.cpp:662: coordf_t& l_height = last_5_layers_height[i];
            // Slicing.cpp:663
            if taller_than_object {
                // Slicing.cpp:664
                if can_adjust[i] && is_approx(last_5_layers_height[i], slicing_params.max_layer_height) {
                    // Slicing.cpp:665-666
                    remain_gap += delta_gap;
                    can_adjust[i] = false;
                    continue;
                }

                // Slicing.cpp:670
                if can_adjust[i]
                    && last_5_layers_height[i] + delta_gap > slicing_params.max_layer_height
                {
                    // Slicing.cpp:671-673
                    remain_gap += last_5_layers_height[i] + delta_gap - slicing_params.max_layer_height;
                    last_5_layers_height[i] = slicing_params.max_layer_height;
                    can_adjust[i] = false;
                } else {
                    // Slicing.cpp:676
                    last_5_layers_height[i] += delta_gap;
                }
            } else {
                // Slicing.cpp:680
                if can_adjust[i] && is_approx(last_5_layers_height[i], slicing_params.min_layer_height) {
                    // Slicing.cpp:681-682
                    remain_gap += delta_gap;
                    can_adjust[i] = false;
                    continue;
                }

                // Slicing.cpp:686
                if can_adjust[i]
                    && last_5_layers_height[i] - delta_gap < slicing_params.min_layer_height
                {
                    // Slicing.cpp:687-689
                    remain_gap += slicing_params.min_layer_height + delta_gap - last_5_layers_height[i];
                    last_5_layers_height[i] = slicing_params.min_layer_height;
                    can_adjust[i] = false;
                } else {
                    // Slicing.cpp:692
                    last_5_layers_height[i] -= delta_gap;
                }
            }
        }
        // Slicing.cpp:696
        remain_gap
    };

    // Slicing.cpp:699
    while gap > 0.0 {
        // Slicing.cpp:700
        let valid_size = get_valid_size(&can_adjust);
        // Slicing.cpp:701
        if valid_size == 0 {
            // 5 layers can not adjust z within valid layer height
            return false;
        }

        // Slicing.cpp:706
        gap = adjust_layer_height(gap, &mut last_5_layers_height, &mut can_adjust);
        // Slicing.cpp:707
        if is_approx(gap, 0.0) {
            // adjust succeed
            break;
        }
    }

    // Slicing.cpp:713-718
    for i in 0..last_5_layers_height.len() {
        // Slicing.cpp:714
        if i > 0 {
            // Slicing.cpp:715
            layer_series[layer_size - 10 + 2 * i] = layer_series[layer_size - 10 + 2 * i - 1];
        }
        // Slicing.cpp:717
        layer_series[layer_size - 10 + 2 * i + 1] =
            layer_series[layer_size - 10 + 2 * i] + last_5_layers_height[i];
    }

    true
}

/// Produce object layers as pairs of low / high layer boundaries, stored into a linear vector.
/// Slicing.cpp:723-779
pub fn generate_object_layers(
    slicing_params: &SlicingParams,
    layer_height_profile: &[f64],
    is_precise_z_height: bool,
) -> Vec<f64> {
    debug_assert!(!layer_height_profile.is_empty()); // Slicing.cpp:729

    // Slicing.cpp:731-732
    let mut print_z: CoordF = 0.0;
    let mut height: CoordF;

    // Slicing.cpp:734
    let mut out: Vec<f64> = Vec::new();

    // Slicing.cpp:736
    if slicing_params.first_object_layer_height_fixed() {
        // Slicing.cpp:737
        out.push(0.0);
        // Slicing.cpp:738
        print_z = slicing_params.first_object_layer_height;
        // Slicing.cpp:739
        out.push(print_z);
    }

    // Slicing.cpp:742
    let mut idx_layer_height_profile: usize = 0;
    // loop until we have at least one layer and the max slice_z reaches the object height
    // Slicing.cpp:744
    let mut slice_z = print_z + 0.5 * slicing_params.min_layer_height;
    // Slicing.cpp:745
    while slice_z < slicing_params.object_print_z_height() {
        // Slicing.cpp:746
        height = slicing_params.min_layer_height;
        // Slicing.cpp:747
        if idx_layer_height_profile < layer_height_profile.len() {
            // Slicing.cpp:748
            let mut next = idx_layer_height_profile + 2;
            // Slicing.cpp:749
            loop {
                // Slicing.cpp:750
                if next >= layer_height_profile.len() || slice_z < layer_height_profile[next] {
                    break;
                }
                // Slicing.cpp:752-753
                idx_layer_height_profile = next;
                next += 2;
            }
            // Slicing.cpp:755-756
            let z1 = layer_height_profile[idx_layer_height_profile];
            let h1 = layer_height_profile[idx_layer_height_profile + 1];
            // Slicing.cpp:757
            height = h1;
            // Slicing.cpp:758
            if next < layer_height_profile.len() {
                // Slicing.cpp:759-760
                let z2 = layer_height_profile[next];
                let h2 = layer_height_profile[next + 1];
                // Slicing.cpp:761
                height = lerp(h1, h2, (slice_z - z1) / (z2 - z1));
                debug_assert!(
                    // Slicing.cpp:762
                    height >= slicing_params.min_layer_height - EPSILON
                        && height <= slicing_params.max_layer_height + EPSILON
                );
            }
        }
        // Slicing.cpp:765
        slice_z = print_z + 0.5 * height;
        // Slicing.cpp:766
        if slice_z >= slicing_params.object_print_z_height() {
            break;
        }
        debug_assert!(height > slicing_params.min_layer_height - EPSILON); // Slicing.cpp:768
        debug_assert!(height < slicing_params.max_layer_height + EPSILON); // Slicing.cpp:769
        // Slicing.cpp:770
        out.push(print_z);
        // Slicing.cpp:771
        print_z += height;
        // Slicing.cpp:772
        slice_z = print_z + 0.5 * slicing_params.min_layer_height;
        // Slicing.cpp:773
        out.push(print_z);
    }

    // Slicing.cpp:776
    if is_precise_z_height {
        adjust_layer_series_to_align_object_height(slicing_params, &mut out);
    }
    // Slicing.cpp:778
    out
}

/// Check whether the layer height profile describes a fixed layer height profile.
/// Slicing.cpp:782-813
pub fn check_object_layers_fixed(
    slicing_params: &SlicingParams,
    layer_height_profile: &[f64],
) -> bool {
    debug_assert!(layer_height_profile.len() >= 4); // Slicing.cpp:786
    debug_assert!(layer_height_profile.len() % 2 == 0); // Slicing.cpp:787
    debug_assert!(layer_height_profile[0] == 0.0); // Slicing.cpp:788

    // Slicing.cpp:790
    if layer_height_profile.len() != 4 && layer_height_profile.len() != 8 {
        return false;
    }

    // Slicing.cpp:793
    let fixed_step1 = is_approx(layer_height_profile[1], layer_height_profile[3]);
    // Slicing.cpp:794
    let fixed_step2 = layer_height_profile.len() == 4
        || (layer_height_profile[2] == layer_height_profile[4]
            && is_approx(layer_height_profile[5], layer_height_profile[7]));

    // Slicing.cpp:797
    if !fixed_step1 || !fixed_step2 {
        return false;
    }

    // Slicing.cpp:800
    if layer_height_profile[2] < 0.5 * slicing_params.first_object_layer_height + EPSILON
        || !is_approx(layer_height_profile[3], slicing_params.first_object_layer_height)
    {
        return false;
    }

    // Slicing.cpp:804
    let z_max = layer_height_profile[layer_height_profile.len() - 2];
    // Slicing.cpp:805
    let z_2nd = slicing_params.first_object_layer_height + 0.5 * slicing_params.layer_height;
    // Slicing.cpp:806
    if z_2nd > z_max {
        return true;
    }
    // Slicing.cpp:808
    if z_2nd < layer_height_profile[layer_height_profile.len() - 4] + EPSILON
        || !is_approx(*layer_height_profile.last().unwrap(), slicing_params.layer_height)
    {
        return false;
    }

    // Slicing.cpp:812
    true
}

/// Produce a 1D texture packed into a 2D texture describing in the RGBA format
/// the planned object layers.
/// Returns number of cells used by the texture of the 0th LOD level.
/// Slicing.cpp:815-925
pub fn generate_layer_height_texture(
    slicing_params: &SlicingParams,
    layers: &[CoordF],
    data: &mut [u8],
    rows: i32,
    cols: i32,
    level_of_detail_2nd_level: bool,
) -> i32 {
    // https://github.com/aschn/gnuplot-colorbrewer
    // Slicing.cpp:821-829. Vec3crd palette entries (hex int literals preserved).
    let palette_raw: [[i32; 3]; 8] = [
        [0x01A, 0x098, 0x050],
        [0x066, 0x0BD, 0x063],
        [0x0A6, 0x0D9, 0x06A],
        [0x0D9, 0x0F1, 0x0EB],
        [0x0FE, 0x0E6, 0x0EB],
        [0x0FD, 0x0AE, 0x061],
        [0x0F4, 0x06D, 0x043],
        [0x0D7, 0x030, 0x027],
    ];

    // Clear the main texture and the 2nd LOD level.
    // Slicing.cpp:832: memset is commented out upstream.
    // 2nd LOD level data start
    // Slicing.cpp:834: data1 = data + rows * cols * 4. Stored as an offset into `data`
    // so the negative-index pointer writes below can be expressed as buffer indices.
    let data1_offset = (rows * cols * 4) as usize;
    // Slicing.cpp:835
    let ncells = ((cols - 1) * rows).min(
        (16.0 * (slicing_params.object_print_z_height() / slicing_params.min_layer_height)).ceil()
            as i32,
    );
    // Slicing.cpp:836
    let ncells1 = ncells / 2;
    // Slicing.cpp:837
    let cols1 = cols / 2;
    // Slicing.cpp:838
    let z_to_cell = (ncells - 1) as CoordF / slicing_params.object_print_z_height();
    // Slicing.cpp:839
    let cell_to_z = slicing_params.object_print_z_height() / (ncells - 1) as CoordF;
    // Slicing.cpp:840
    let z_to_cell1 = (ncells1 - 1) as CoordF / slicing_params.object_print_z_height();
    // for color scaling
    // Slicing.cpp:842
    let mut hscale = 2.0
        * (slicing_params.max_layer_height - slicing_params.layer_height)
            .max(slicing_params.layer_height - slicing_params.min_layer_height);
    // Slicing.cpp:843
    if hscale == 0.0 {
        // All layers have the same height. Provide some height scale to avoid division by zero.
        // Slicing.cpp:845
        hscale = slicing_params.layer_height;
    }
    // Slicing.cpp:846
    let mut idx_layer = 0;
    while idx_layer < layers.len() {
        // Slicing.cpp:847-848
        let lo = layers[idx_layer];
        let mut hi = layers[idx_layer + 1];
        // Slicing.cpp:849
        let mid = 0.5 * (lo + hi);
        debug_assert!(mid <= slicing_params.object_print_z_height()); // Slicing.cpp:850
        // Slicing.cpp:851
        let h = hi - lo;
        // Slicing.cpp:852
        hi = hi.min(slicing_params.object_print_z_height());
        // Slicing.cpp:853
        let cell_first = ((lo * z_to_cell).ceil() as i32).clamp(0, ncells - 1);
        // Slicing.cpp:854
        let cell_last = ((hi * z_to_cell).floor() as i32).clamp(0, ncells - 1);
        // Slicing.cpp:855
        for cell in cell_first..=cell_last {
            // Slicing.cpp:856
            let idxf =
                (0.5 * hscale + (h - slicing_params.layer_height)) * (palette_raw.len() - 1) as CoordF / hscale;
            // Slicing.cpp:857
            let idx1 = (idxf.floor() as i32).clamp(0, (palette_raw.len() - 1) as i32);
            // Slicing.cpp:858
            let idx2 = ((palette_raw.len() - 1) as i32).min(idx1 + 1);
            // Slicing.cpp:859
            let t = idxf - idx1 as CoordF;
            // Slicing.cpp:860-861
            let color1 = palette_raw[idx1 as usize];
            let color2 = palette_raw[idx2 as usize];
            // Slicing.cpp:862
            let z = cell_to_z * cell as CoordF;
            debug_assert!(lo - EPSILON <= z && z <= hi + EPSILON); // Slicing.cpp:863
            // Intensity profile to visualize the layers.
            // Slicing.cpp:865
            let intensity = (std::f64::consts::PI * 0.7 * (mid - z) / h).cos();
            // Color mapping from layer height to RGB.
            // Slicing.cpp:867-870
            let color = [
                intensity * lerp(color1[0] as CoordF, color2[0] as CoordF, t),
                intensity * lerp(color1[1] as CoordF, color2[1] as CoordF, t),
                intensity * lerp(color1[2] as CoordF, color2[2] as CoordF, t),
            ];
            // Slicing.cpp:871
            let row = cell / (cols - 1);
            // Slicing.cpp:872
            let col = cell - row * (cols - 1);
            debug_assert!(row >= 0 && row < rows); // Slicing.cpp:873
            debug_assert!(col >= 0 && col < cols); // Slicing.cpp:874
            // Slicing.cpp:875: ptr = data + (row * cols + col) * 4
            let ptr = ((row * cols + col) * 4) as usize;
            // Slicing.cpp:876-879
            data[ptr] = (color[0] + 0.5).floor().clamp(0.0, 255.0) as u8;
            data[ptr + 1] = (color[1] + 0.5).floor().clamp(0.0, 255.0) as u8;
            data[ptr + 2] = (color[2] + 0.5).floor().clamp(0.0, 255.0) as u8;
            data[ptr + 3] = 255;
            // Slicing.cpp:880
            if col == 0 && row > 0 {
                // Duplicate the first value in a row as a last value of the preceding row.
                // Slicing.cpp:882-885: ptr[-4..-1]
                data[ptr - 4] = data[ptr];
                data[ptr - 3] = data[ptr + 1];
                data[ptr - 2] = data[ptr + 2];
                data[ptr - 1] = data[ptr + 3];
            }
        }
        // Slicing.cpp:888
        if level_of_detail_2nd_level {
            // Slicing.cpp:889
            let cell_first = ((lo * z_to_cell1).ceil() as i32).clamp(0, ncells1 - 1);
            // Slicing.cpp:890
            let cell_last = ((hi * z_to_cell1).floor() as i32).clamp(0, ncells1 - 1);
            // Slicing.cpp:891
            for cell in cell_first..=cell_last {
                // Slicing.cpp:892
                let idxf = (0.5 * hscale + (h - slicing_params.layer_height))
                    * (palette_raw.len() - 1) as CoordF
                    / hscale;
                // Slicing.cpp:893
                let idx1 = (idxf.floor() as i32).clamp(0, (palette_raw.len() - 1) as i32);
                // Slicing.cpp:894
                let idx2 = ((palette_raw.len() - 1) as i32).min(idx1 + 1);
                // Slicing.cpp:895
                let t = idxf - idx1 as CoordF;
                // Slicing.cpp:896-897
                let color1 = palette_raw[idx1 as usize];
                let color2 = palette_raw[idx2 as usize];
                // Color mapping from layer height to RGB.
                // Slicing.cpp:899-902
                let color = [
                    lerp(color1[0] as CoordF, color2[0] as CoordF, t),
                    lerp(color1[1] as CoordF, color2[1] as CoordF, t),
                    lerp(color1[2] as CoordF, color2[2] as CoordF, t),
                ];
                // Slicing.cpp:903
                let row = cell / (cols1 - 1);
                // Slicing.cpp:904
                let col = cell - row * (cols1 - 1);
                debug_assert!(row >= 0 && row < rows / 2); // Slicing.cpp:905
                debug_assert!(col >= 0 && col < cols / 2); // Slicing.cpp:906
                // Slicing.cpp:907: ptr = data1 + (row * cols1 + col) * 4
                let ptr = data1_offset + ((row * cols1 + col) * 4) as usize;
                // Slicing.cpp:908-911
                data[ptr] = (color[0] + 0.5).floor().clamp(0.0, 255.0) as u8;
                data[ptr + 1] = (color[1] + 0.5).floor().clamp(0.0, 255.0) as u8;
                data[ptr + 2] = (color[2] + 0.5).floor().clamp(0.0, 255.0) as u8;
                data[ptr + 3] = 255;
                // Slicing.cpp:912
                if col == 0 && row > 0 {
                    // Duplicate the first value in a row as a last value of the preceding row.
                    // Slicing.cpp:914-917: ptr[-4..-1]
                    data[ptr - 4] = data[ptr];
                    data[ptr - 3] = data[ptr + 1];
                    data[ptr - 2] = data[ptr + 2];
                    data[ptr - 1] = data[ptr + 3];
                }
            }
        }
        // Slicing.cpp:846
        idx_layer += 2;
    }

    // Returns number of cells of the 0th LOD level.
    // Slicing.cpp:924
    ncells
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_slicing_params_default() {
        let params = SlicingParams::default();
        assert!((params.layer_height - 0.2).abs() < 1e-6);
        assert!((params.first_print_layer_height - 0.2).abs() < 1e-6);
    }

    #[test]
    fn test_slicing_params_fields() {
        let mut params = SlicingParams::default();
        params.layer_height = 0.15;
        params.first_print_layer_height = 0.25;

        assert!((params.layer_height - 0.15).abs() < 1e-6);
        assert!((params.first_print_layer_height - 0.25).abs() < 1e-6);
    }

    #[test]
    fn test_slicing_params_raft() {
        let mut params = SlicingParams::default();
        assert!(!params.has_raft());
        assert_eq!(params.raft_layers(), 0);

        params.base_raft_layers = 2;
        params.interface_raft_layers = 1;
        assert!(params.has_raft());
        assert_eq!(params.raft_layers(), 3);
    }

    #[test]
    fn test_equal_layering() {
        let mut params1 = SlicingParams::default();
        params1.valid = true;
        let mut params2 = SlicingParams::default();
        params2.valid = true;
        assert!(equal_layering(&params1, &params2));

        let mut params3 = SlicingParams::default();
        params3.valid = true;
        params3.layer_height = 0.15;
        assert!(!equal_layering(&params1, &params3));
    }

    #[test]
    fn test_slicing_mode() {
        assert_eq!(SlicingMode::default(), SlicingMode::Regular);
    }
}
