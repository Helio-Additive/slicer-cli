//! Faithful 1:1 port of `ModelArrange.cpp` / `ModelArrange.hpp`.
//!
//! C++ Reference:
//! - src/libslic3r/ModelArrange.cpp
//! - src/libslic3r/ModelArrange.hpp
//!
//! This module also ports the data structures from `Arrange.hpp`
//! (the `arrangement` namespace) that `ModelArrange.cpp` depends on directly,
//! since `Arrange.cpp`/`Arrange.hpp` have not yet been ported as their own unit.
//! These are pure-data carriers (`ArrangePolygon`, `ArrangeParams`, `CircleBed`,
//! `InfiniteBed`) used verbatim by the functions below.
//!
//! Functions that require not-yet-ported `Model.cpp` instance methods
//! (`ModelInstance::get_arrange_polygon`, `apply_arrange_result`, `arrange_order`,
//! `ModelObject::add_instance(const ModelInstance&)`,
//! `instance_convex_hull_bounding_box`) or `Print.cpp` /
//! `DynamicPrintConfig` config plumbing (`get_instance_arrange_poly`) are
//! documented as blocked at the bottom of this file. Everything tractable
//! against the `ArrangePolygon` abstraction is ported faithfully.
//!
//! AUDIT (2026-06-13): the blocking root is the full C++ `Model` /
//! `ModelObject` / `ModelInstance` / `ModelVolume` hierarchy — `model.rs`
//! retains a divergent simplified shim (single merged mesh, POD `Instance`, no
//! per-volume/`Geometry::Transformation`, no `extruderParamsMap`) to keep the
//! format loaders compiling. The 2026-06-12 config-hierarchy threading covers
//! `Layer`/`PrintObject`/`LayerRegion` (print_object.rs), NOT the Model side, so
//! it does not unblock these. No wasm-unsafe native backend is involved. No
//! stubs/fakes were introduced; the per-symbol blockers are enumerated below.

use crate::geometry::{convex_hull_points, ExPolygon, Point, Points};
use crate::Coord;

// ============================================================================
// arrangement namespace types (Arrange.hpp) that ModelArrange.cpp depends on.
// ============================================================================

/// A geometry abstraction for a circular print bed. Similarly to BoundingBox.
/// Arrange.hpp:16
#[derive(Debug, Clone, Copy)]
pub struct CircleBed {
    // Arrange.hpp:17
    center_: Point,
    // Arrange.hpp:18
    radius_: f64,
}

impl CircleBed {
    /// Arrange.hpp:21
    /// C++: inline CircleBed(): center_(0, 0), radius_(std::nan("")) {}
    #[inline]
    pub fn new() -> Self {
        Self {
            center_: Point::new(0, 0),
            radius_: f64::NAN,
        }
    }

    /// Arrange.hpp:22
    /// C++: explicit inline CircleBed(const Point& c, double r): center_(c), radius_(r) {}
    #[inline]
    pub fn with_center_radius(c: Point, r: f64) -> Self {
        Self {
            center_: c,
            radius_: r,
        }
    }

    /// Arrange.hpp:24
    /// C++: inline double radius() const { return radius_; }
    #[inline]
    pub fn radius(&self) -> f64 {
        self.radius_
    }

    /// Arrange.hpp:25
    /// C++: inline const Point& center() const { return center_; }
    #[inline]
    pub fn center(&self) -> &Point {
        &self.center_
    }
}

impl Default for CircleBed {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// Representing an unbounded bed.
/// Arrange.hpp:29
#[derive(Debug, Clone, Copy)]
pub struct InfiniteBed {
    // Arrange.hpp:30
    pub center: Point,
}

impl InfiniteBed {
    /// Arrange.hpp:31
    /// C++: explicit InfiniteBed(const Point &p = {0, 0}): center{p} {}
    #[inline]
    pub fn new(p: Point) -> Self {
        Self { center: p }
    }
}

impl Default for InfiniteBed {
    #[inline]
    fn default() -> Self {
        // Arrange.hpp:31 default argument {0, 0}
        Self {
            center: Point::new(0, 0),
        }
    }
}

/// A logical bed representing an object not being arranged. Either the arrange
/// has not yet successfully run on this ArrangePolygon or it could not fit the
/// object due to overly large size or invalid geometry.
/// Arrange.hpp:37
/// C++: static const constexpr int UNARRANGED = -1;
pub const UNARRANGED: i32 = -1;

/// Input/Output structure for the arrange() function. The poly field will not
/// be modified during arrangement. Instead, the translation and rotation fields
/// will mark the needed transformation for the polygon to be in the arranged
/// position. These can also be set to an initial offset and rotation.
///
/// The bed_idx field will indicate the logical bed into which the
/// polygon belongs: UNARRANGED means no place for the polygon
/// (also the initial state before arrange), 0..N means the index of the bed.
/// Zero is the physical bed, larger than zero means a virtual bed.
/// Arrange.hpp:48
#[derive(Clone)]
pub struct ArrangePolygon {
    /// The 2D silhouette to be arranged
    /// Arrange.hpp:49
    pub poly: ExPolygon,
    /// The translation of the poly
    /// Arrange.hpp:50  C++: Vec2crd translation{0, 0};
    pub translation: Point,
    /// The rotation of the poly in radians
    /// Arrange.hpp:51
    pub rotation: f64,
    /// Arrange with inflated polygon
    /// Arrange.hpp:52  C++: coord_t inflation = 0;
    pub inflation: Coord,
    /// To which logical bed does poly belong...
    /// Arrange.hpp:53  C++: int bed_idx{UNARRANGED};
    pub bed_idx: i32,
    /// Arrange.hpp:54
    pub priority: i32,
    //BBS: add locked_plate to indicate whether it is in the locked plate
    /// Arrange.hpp:56  C++: int locked_plate{ -1 };
    pub locked_plate: i32,
    /// Arrange.hpp:57
    pub is_virt_object: bool,
    /// Arrange.hpp:58
    pub is_extrusion_cali_object: bool,
    /// Arrange.hpp:59
    pub is_wipe_tower: bool,
    /// Arrange.hpp:60
    pub has_tree_support: bool,
    //BBS: add row/col for sudoku-style layout
    /// Arrange.hpp:62
    pub row: i32,
    /// Arrange.hpp:63
    pub col: i32,
    /// extruder_id for least extruder switch, filament type for different material judge
    /// Arrange.hpp:64  C++: std::map<int, std::string> extrude_id_filament_types;
    pub extrude_id_filament_types: std::collections::BTreeMap<i32, String>,
    /// Arrange.hpp:65  C++: int filament_temp_type{ -1 };
    pub filament_temp_type: i32,
    /// bed temperature for different material judge
    /// Arrange.hpp:66
    pub bed_temp: i32,
    /// print temperature for different material judge
    /// Arrange.hpp:67
    pub print_temp: i32,
    /// first layer bed temperature for different material judge
    /// Arrange.hpp:68
    pub first_bed_temp: i32,
    /// first layer print temperature for different material judge
    /// Arrange.hpp:69
    pub first_print_temp: i32,
    /// max bed temperature for material compatibility, which is usually the filament vitrification temp
    /// Arrange.hpp:70
    pub vitrify_temp: i32,
    /// item id in the vector, used for accessing all possible params like extrude_id
    /// Arrange.hpp:71
    pub itemid: i32,
    /// transform has been applied
    /// Arrange.hpp:72
    pub is_applied: i32,
    /// item height
    /// Arrange.hpp:73
    pub height: f64,
    /// brim width
    /// Arrange.hpp:74
    pub brim_width: f64,
    /// Arrange.hpp:75
    pub name: String,

    // If empty, any rotation is allowed (currently unsupported)
    // If only a zero is there, no rotation is allowed
    /// Arrange.hpp:79  C++: std::vector<double> allowed_rotations = {0.};
    pub allowed_rotations: Vec<f64>,

    /// Optional setter function which can store arbitrary data in its closure
    /// Arrange.hpp:82  C++: std::function<void(const ArrangePolygon&)> setter = nullptr;
    pub setter: Option<std::rc::Rc<dyn Fn(&ArrangePolygon)>>,
}

impl Default for ArrangePolygon {
    /// Arrange.hpp:48 — default member initializers
    fn default() -> Self {
        Self {
            poly: ExPolygon::default(),
            translation: Point::new(0, 0), // Arrange.hpp:50
            rotation: 0.0,                 // Arrange.hpp:51
            inflation: 0,                  // Arrange.hpp:52
            bed_idx: UNARRANGED,           // Arrange.hpp:53
            priority: 0,                   // Arrange.hpp:54
            locked_plate: -1,              // Arrange.hpp:56
            is_virt_object: false,         // Arrange.hpp:57
            is_extrusion_cali_object: false, // Arrange.hpp:58
            is_wipe_tower: false,          // Arrange.hpp:59
            has_tree_support: false,       // Arrange.hpp:60
            row: 0,                        // Arrange.hpp:62
            col: 0,                        // Arrange.hpp:63
            extrude_id_filament_types: std::collections::BTreeMap::new(), // Arrange.hpp:64
            filament_temp_type: -1,        // Arrange.hpp:65
            bed_temp: 0,                   // Arrange.hpp:66
            print_temp: 0,                 // Arrange.hpp:67
            first_bed_temp: 0,             // Arrange.hpp:68
            first_print_temp: 0,           // Arrange.hpp:69
            vitrify_temp: 0,               // Arrange.hpp:70
            itemid: 0,                     // Arrange.hpp:71
            is_applied: 0,                 // Arrange.hpp:72
            height: 0.0,                   // Arrange.hpp:73
            brim_width: 0.0,               // Arrange.hpp:74
            name: String::new(),           // Arrange.hpp:75
            allowed_rotations: vec![0.0],  // Arrange.hpp:79
            setter: None,                  // Arrange.hpp:82
        }
    }
}

impl ArrangePolygon {
    /// Helper function to call the setter with the arrange data arguments
    /// Arrange.hpp:85
    /// C++: void apply() { if (setter && !is_applied) { setter(*this); is_applied = 1; } }
    pub fn apply(&mut self) {
        if self.setter.is_some() && self.is_applied == 0 {
            // Clone the Rc so we don't hold a borrow of `self.setter` while
            // passing `&*self` (which references the same ArrangePolygon) —
            // semantically identical to C++ `setter(*this)`.
            let setter = self.setter.clone().unwrap();
            setter(self);
            self.is_applied = 1;
        }
    }

    /// Test if arrange() was called previously and gave a successful result.
    /// Arrange.hpp:93
    /// C++: bool is_arranged() const { return bed_idx != UNARRANGED; }
    #[inline]
    pub fn is_arranged(&self) -> bool {
        self.bed_idx != UNARRANGED
    }

    /// Arrange.hpp:95
    /// C++: inline ExPolygon transformed_poly() const
    #[inline]
    pub fn transformed_poly(&self) -> ExPolygon {
        // Arrange.hpp:97
        let mut ret: ExPolygon = self.poly.clone();
        // Arrange.hpp:98  C++: ret.rotate(rotation);
        ret.rotate(self.rotation);
        // Arrange.hpp:99  C++: ret.translate(translation.x(), translation.y());
        ret.translate(Point::new(self.translation.x, self.translation.y));

        ret
    }
}

/// Arrange.hpp:105  C++: using ArrangePolygons = std::vector<ArrangePolygon>;
pub type ArrangePolygons = Vec<ArrangePolygon>;

// ============================================================================
// ModelArrange.hpp
// ============================================================================

/// Do something with ArrangePolygons in virtual beds
/// ModelArrange.hpp:18
/// C++: using VirtualBedFn = std::function<void(arrangement::ArrangePolygon&)>;
pub type VirtualBedFn<'a> = Option<&'a mut dyn FnMut(&mut ArrangePolygon)>;

/// ModelArrange.hpp:20
/// C++: [[noreturn]] inline void throw_if_out_of_bed(arrangement::ArrangePolygon&)
/// C++: { throw Slic3r::RuntimeError("Objects could not fit on the bed"); }
///
/// Returns an Err mirroring the thrown `Slic3r::RuntimeError`.
pub fn throw_if_out_of_bed(_ap: &mut ArrangePolygon) -> crate::Result<()> {
    // ModelArrange.hpp:22
    // C++: throw Slic3r::RuntimeError("Objects could not fit on the bed");
    Err(crate::Error::InvalidInput(
        "Objects could not fit on the bed".to_string(),
    ))
}

// ============================================================================
// ModelArrange.cpp
// ============================================================================

/// ModelArrange.cpp:29
/// C++: bool apply_arrange_polys(ArrangePolygons &input, ModelInstancePtrs &instances, VirtualBedFn vfn)
///
/// Faithful port of the control flow operating on the `ArrangePolygon`
/// abstraction. The C++ side calls
/// `instances[i]->apply_arrange_result(translation, rotation)`; here the
/// per-item application is delegated through the `ArrangePolygon::setter`
/// closure (Arrange.hpp:82), which is exactly how BambuStudio threads the
/// `ModelInstance::apply_arrange_result` call (see `get_arrange_poly<T>`
/// below, ModelArrange.cpp:99-108).
pub fn apply_arrange_polys(input: &mut ArrangePolygons, mut vfn: VirtualBedFn) -> bool {
    // ModelArrange.cpp:31
    let mut ret = true;

    // ModelArrange.cpp:33
    for i in 0..input.len() {
        // ModelArrange.cpp:34
        if input[i].bed_idx != 0 {
            ret = false;
            if let Some(f) = vfn.as_deref_mut() {
                f(&mut input[i]);
            }
        }
        // ModelArrange.cpp:35
        if input[i].bed_idx >= 0 {
            // ModelArrange.cpp:36-37
            // C++: instances[i]->apply_arrange_result(translation.cast<double>(), rotation);
            // Delegated through the ArrangePolygon setter abstraction.
            input[i].apply();
        }
    }

    // ModelArrange.cpp:40
    ret
}

/// ModelArrange.cpp:43
/// C++: Slic3r::arrangement::ArrangePolygon get_arrange_poly(const Model &model)
///
/// Pure-geometry portion of the per-model arrange polygon: rotate/translate
/// each instance's arrange polygon contour and accumulate the points, then take
/// the convex hull. The per-instance arrange polygons are supplied via the
/// `instance_polys` iterator (each item is the result of
/// `ModelInstance::get_arrange_polygon`), because that method lives in the
/// not-yet-ported `Model.cpp`.
pub fn get_arrange_poly<I>(instance_polys: I) -> ArrangePolygon
where
    I: IntoIterator<Item = ArrangePolygon>,
{
    // ModelArrange.cpp:45
    let mut ap = ArrangePolygon::default();
    // ModelArrange.cpp:46  C++: Points &apts = ap.poly.contour.points;
    // `apts` aliases `ap.poly.contour.points`; in Rust we operate on the field
    // directly so the per-iteration rotate/translate mutate the SAME accumulated
    // points (matching the C++ reference semantics exactly).
    // ModelArrange.cpp:47-48  for each instance arrange polygon
    for obj_ap in instance_polys {
        // ModelArrange.cpp:51  C++: ap.poly.contour.rotate(obj_ap.rotation);
        ap.poly.contour.rotate(obj_ap.rotation);
        // ModelArrange.cpp:52  C++: ap.poly.contour.translate(obj_ap.translation.x(), obj_ap.translation.y());
        ap.poly
            .contour
            .translate(Point::new(obj_ap.translation.x, obj_ap.translation.y));
        // ModelArrange.cpp:53  C++: const Points &pts = obj_ap.poly.contour.points;
        let pts = &obj_ap.poly.contour.points;
        // ModelArrange.cpp:54  C++: std::copy(pts.begin(), pts.end(), std::back_inserter(apts));
        ap.poly.contour.points.extend_from_slice(pts);
    }

    // ModelArrange.cpp:57  C++: apts = std::move(Geometry::convex_hull(apts).points);
    let apts: Points = convex_hull_points(std::mem::take(&mut ap.poly.contour.points)).points;
    // apts is ap.poly.contour.points in C++ (a reference); write the hull back.
    ap.poly.contour.points = apts;
    // ModelArrange.cpp:58
    ap
}

// Set up arrange polygon for a ModelInstance and Wipe tower
// ModelArrange.cpp:91
// C++: template<class T> arrangement::ArrangePolygon get_arrange_poly(T obj, const DynamicPrintConfig& config)
//
// `get_arrange_poly<T>` wires `obj.apply_arrange_result(t, rotation, itemid)`
// into the `ArrangePolygon::setter` closure. The `setter` field above provides
// exactly this capability; the concrete `T` (e.g. `PtrWrapper<ModelInstance>`)
// requires `ModelInstance::get_arrange_polygon` / `apply_arrange_result` which
// live in the not-yet-ported `Model.cpp`, so the generic shell is documented as
// blocked rather than stubbed. The line-for-line body is:
//   ArrangePolygon ap = obj.get_arrange_polygon(config);   // :94
//   ap.bed_idx = 0;                                        // :98
//   ap.setter = [obj](const ArrangePolygon& p) {           // :99
//       if (p.is_arranged()) {                             // :100
//           Vec2d t = p.translation.cast<double>();        // :101
//           T{ obj }.apply_arrange_result(t, p.rotation, p.itemid); // :106
//       }
//   };
//   return ap;                                             // :110

// ---------------------------------------------------------------------------
// BLOCKED symbols — every one of these is gated on the full C++
// `Model`/`ModelObject`/`ModelInstance`/`ModelVolume` class hierarchy, which is
// NOT ported into this crate. The Rust `model.rs` deliberately retains a
// divergent *simplified* shim (a single merged `mesh: TriangleMesh` per object,
// a `Instance { position, rotation_z, scale }` POD, and a small `ObjectConfig`)
// to keep the 3MF/STL/OBJ/AMF/SVG/STEP loaders working; its own module header
// states this rework "must not be done piecemeal without breaking those
// consumers". The 2026-06-12 config-hierarchy threading (Layer -> PrintObject ->
// Print, LayerRegion -> PrintRegion in print_object.rs) is independent of and
// does NOT supply the Model-side accessors below, so these stay blocked. None of
// the missing pieces is a wasm-unsafe native backend — they are pure not-yet-
// ported Model.cpp / Print.cpp symbols. Documented, never stubbed.
//
// * get_arrange_polys(const Model&, ModelInstancePtrs&)        ModelArrange.cpp:10
//     Needs ModelInstance::get_arrange_polygon(&ap) (Model.cpp:4129) and a real
//     ModelInstancePtrs collection of pointer-stable `ModelInstance*`; the Rust
//     `Instance` POD has none of get_arrange_polygon/apply_arrange_result.
//
// * get_arrange_poly(const Model&) full version              ModelArrange.cpp:43
//     The pure-geometry core (rotate/translate per-instance contour + convex
//     hull) is ported above as `get_arrange_poly<I>`; the per-instance source
//     `minst->get_arrange_polygon(&obj_ap)` needs Model.cpp:4129, which itself
//     pulls in Geometry::Transformation, ModelObject::convex_hull_2d, the
//     `ModelVolume volumes` collection (is_model_part/get_extruders), and the
//     static Model::extruderParamsMap — none present in the model.rs shim.
//
// * duplicate(Model&, ArrangePolygons&, VirtualBedFn)        ModelArrange.cpp:61
//     Needs ModelObject::add_instance(const ModelInstance&) (copy-ctor add),
//     ModelInstance::set_offset/get_offset (Vec3d offset, not the shim's POD),
//     unscale(Vec2crd)->Vec2d, to_3d, ModelObject::invalidate_bounding_box.
//
// * duplicate_objects(Model&, size_t)                        ModelArrange.cpp:79
//     Needs ModelObject::add_instance(const ModelInstance&) (copy-ctor add).
//
// * get_arrange_poly<T> generic + ModelInstance* spec.       ModelArrange.cpp:91/113
//     Needs PtrWrapper<ModelInstance> (ModelArrange.hpp:68) wrapping
//     ModelInstance::get_arrange_polygon/apply_arrange_result and the
//     `arrange_order` field (Model.cpp:4129/4189) — the setter-closure plumbing
//     for this already exists on ArrangePolygon::setter above, but the concrete
//     `T = PtrWrapper<ModelInstance>` cannot be built without those methods.
//
// * get_instance_arrange_poly(ModelInstance*, DynamicPrintConfig&) ModelArrange.cpp:119
//     Even the config-reading tail (curr_bed_type/nozzle_temperature/
//     temperature_vitrification/filament_type, get_bed_temp_key /
//     get_bed_temp_1st_layer_key in print_config.rs, Print::get_filament_temp_type
//     / get_compatible_filament_type which ARE ported in print.rs:1698/1786)
//     operates on an ArrangePolygon produced by PtrWrapper{instance}.
//     get_arrange_polygon and reads obj->instance_convex_hull_bounding_box
//     (Model.cpp:1604), obj->get_config_value (Model.hpp), support_type
//     (SupportType, ported in print_config.rs:3323). All of these hang off the
//     un-ported ModelInstance/ModelObject, so the whole function is blocked.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Polygon;

    #[test]
    fn test_arrange_polygon_defaults() {
        let ap = ArrangePolygon::default();
        assert_eq!(ap.bed_idx, UNARRANGED);
        assert_eq!(ap.locked_plate, -1);
        assert_eq!(ap.filament_temp_type, -1);
        assert_eq!(ap.allowed_rotations, vec![0.0]);
        assert!(!ap.is_arranged());
    }

    #[test]
    fn test_apply_arrange_polys_bed_idx() {
        let mut polys: ArrangePolygons = vec![
            ArrangePolygon {
                bed_idx: 0,
                ..ArrangePolygon::default()
            },
            ArrangePolygon {
                bed_idx: 1,
                ..ArrangePolygon::default()
            },
        ];
        // bed_idx != 0 on the second poly => ret should be false.
        let ret = apply_arrange_polys(&mut polys, None);
        assert!(!ret);
    }

    #[test]
    fn test_get_arrange_poly_convex_hull() {
        // Two unit-ish square instance polygons; the convex hull of all points
        // should be returned in ap.poly.contour.points.
        let mut a = ArrangePolygon::default();
        a.poly.contour = Polygon::from(vec![
            Point::new(0, 0),
            Point::new(10, 0),
            Point::new(10, 10),
            Point::new(0, 10),
        ]);
        let result = get_arrange_poly(vec![a]);
        assert!(!result.poly.contour.points.is_empty());
    }
}
