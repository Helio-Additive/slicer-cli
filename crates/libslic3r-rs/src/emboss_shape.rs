//! Plane shape information used to emboss and edit a shape.
//!
//! C++ Reference:
//! - BambuStudio/src/libslic3r/EmbossShape.hpp
//!
//! Faithful 1:1 line-by-line port of `EmbossShape.hpp`. This is a header-only file
//! (there is no matching `.cpp`); all definitions live in the header.
//!
//! The `Emboss::` namespace types declared in this C++ header (`Glyph`, `Glyphs`,
//! `FontFile`, `FontFileWithCache`) are already ported in [`crate::emboss`]; they are
//! re-exported here so they are not duplicated, mirroring the C++ `namespace Emboss`.

// EmbossShape.hpp:4-15  #include <string> / <optional> / <memory> / cereal headers /
//                       "Point.hpp" (Transform3d) / "ExPolygon.hpp" /
//                       "ExPolygonSerialize.hpp" / "nanosvg/nanosvg.h" (NSVGimage)
use crate::geometry::{ExPolygons, Transform3D};
use crate::SCALING_FACTOR;
use serde::{Deserialize, Serialize};

// EmbossShape.hpp:41-111  namespace Emboss { Glyph / Glyphs / FontFile / FontFileWithCache }
//
// These live in `EmbossShape.hpp` under `namespace Emboss` in BambuStudio. They are
// already ported faithfully in `crate::emboss`; re-export them so callers see the same
// `Emboss::`-scoped types without duplicating the definitions.
pub use crate::emboss::{FontFile, FontFileWithCache, Glyph};

/// `Slic3r::Transform3d` — a 3D affine transform (4x4, double precision).
///
/// EmbossShape.hpp:12  #include "Point.hpp" // Transform3d
pub type Transform3d = Transform3D;

/// 2D single-precision vector, mirroring Eigen `Vec2f` used for `text_align_offsets`.
///
/// EmbossShape.hpp:194  std::vector<Vec2f> text_align_offsets;
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct Vec2f {
    pub x: f32,
    pub y: f32,
}

// EmbossShape.hpp:19
/// Define how to emboss a shape.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct EmbossProjection {
    // EmbossShape.hpp:20-21
    // Emboss depth, Size in local Z direction
    /// `depth` [in local mm] // Modify By BBS 20241220
    pub depth: f64, // [in loacal mm]//Modify By BBS 20241220
    // EmbossShape.hpp:22
    // NOTE: User should see and modify mainly world size not local

    // EmbossShape.hpp:24-25
    // Flag that result volume use surface cutted from source objects
    pub use_surface: bool,

    // EmbossShape.hpp:26
    /// for old depth
    pub embeded_depth: f64, // for old depth
}

impl Default for EmbossProjection {
    fn default() -> Self {
        Self {
            // EmbossShape.hpp:21  double depth = 2.f;
            depth: 2.0,
            // EmbossShape.hpp:25  bool use_surface = false;
            use_surface: false,
            // EmbossShape.hpp:26  double embeded_depth = 0.f;
            embeded_depth: 0.0,
        }
    }
}

impl PartialEq for EmbossProjection {
    // EmbossShape.hpp:27-29
    // bool operator==(const EmbossProjection &other) const {
    //     return depth == other.depth && use_surface == other.use_surface && embeded_depth == other.embeded_depth;
    // }
    fn eq(&self, other: &Self) -> bool {
        self.depth == other.depth
            && self.use_surface == other.use_surface
            && self.embeded_depth == other.embeded_depth
    }
}

impl EmbossProjection {
    // EmbossShape.hpp:31-32
    // undo / redo stack recovery
    // template<class Archive> void serialize(Archive &ar) { ar(depth, use_surface, embeded_depth); }
    //
    // Cereal `serialize` is replaced by the derived `serde::Serialize`/`Deserialize`,
    // whose field order (depth, use_surface, embeded_depth) matches `ar(...)`.
    pub fn serialize_fields(&self) -> (f64, bool, f64) {
        (self.depth, self.use_surface, self.embeded_depth)
    }
}

// EmbossShape.hpp:35-40
// Extend expolygons with information whether it was successfull healed
/// `HealedExPolygons` is already ported in [`crate::emboss`]; re-export it so the type
/// referenced throughout `EmbossShape.hpp` resolves to the single definition.
///
/// EmbossShape.hpp:36-40
/// ```cpp
/// struct HealedExPolygons{
///     ExPolygons expolygons;
///     bool is_healed;
///     operator ExPolygons&() { return expolygons; }
/// };
/// ```
pub use crate::emboss::HealedExPolygons;

// EmbossShape.hpp:53
// cache for glyph by unicode
// using Glyphs = std::map<int, Glyph>;
//
// std::map<int, Glyph> is an ordered map keyed by codepoint. BTreeMap preserves the
// ordered-by-key semantics of std::map.
/// EmbossShape.hpp:53  using Glyphs = std::map<int, Glyph>;
pub type Glyphs = std::collections::BTreeMap<i32, Glyph>;

// EmbossShape.hpp:112-127
// Help structure to identify expolygons grups
// e.g. emboss -> per glyph -> identify character
/// EmbossShape.hpp:114
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExPolygonsWithId {
    // EmbossShape.hpp:116-119
    // Identificator for shape
    // In text it separate letters and the name is unicode value of letter
    // Is svg it is id of path
    pub id: u32,

    // EmbossShape.hpp:121-123
    // shape defined by integer point contain only lines
    // Curves are converted to sequence of lines
    pub expoly: ExPolygons,

    // EmbossShape.hpp:125-126
    // flag whether expolygons are fully healed(without duplication)
    pub is_healed: bool,
}

impl ExPolygonsWithId {
    /// Construct with the default `is_healed = true` (EmbossShape.hpp:126).
    pub fn new(id: u32, expoly: ExPolygons) -> Self {
        Self {
            id,
            expoly,
            // EmbossShape.hpp:126  bool is_healed = true;
            is_healed: true,
        }
    }
}

/// EmbossShape.hpp:128  using ExPolygonsWithIds = std::vector<ExPolygonsWithId>;
pub type ExPolygonsWithIds = Vec<ExPolygonsWithId>;

// EmbossShape.hpp:130-132
/// Contain plane shape information to be able emboss it and edit it
// EmbossShape.hpp:133
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmbossShape {
    // EmbossShape.hpp:135-136
    // shapes to to emboss separately over surface
    pub shapes_with_ids: ExPolygonsWithIds,

    // EmbossShape.hpp:137-141
    // Only cache for final shape
    // It is calculated from ExPolygonsWithIds
    // Flag is_healed --> whether union of shapes is healed
    // Healed mean without selfintersection and point duplication
    pub final_shape: HealedExPolygons,

    // EmbossShape.hpp:143-144
    // scale of shape, multiplier to get 3d point in mm from integer shape
    /// `scale = SCALING_FACTOR`
    pub scale: f64,

    // EmbossShape.hpp:146-147
    // Define how to emboss shape
    pub projection: EmbossProjection,

    // EmbossShape.hpp:149-154
    // !!! Volume stored in .3mf has transformed vertices.
    // (baked transformation into vertices position)
    // Only place for fill this is when load from .3mf
    // This is correction for volume transformation
    // Stored_Transform3d * fix_3mf_tr = Transform3d_before_store_to_3mf
    pub fix_3mf_tr: Option<Transform3d>,

    // EmbossShape.hpp:188-189
    // When embossing shape is made by svg file this is source data
    pub svg_file: Option<SvgFile>,

    // EmbossShape.hpp:190
    pub text_scales: Vec<f32>,

    // EmbossShape.hpp:191
    pub text_cursors: Vec<f32>,

    // EmbossShape.hpp:192
    pub text_absolute_cursors: Vec<f32>,

    // EmbossShape.hpp:193
    pub text_align_offsets: Vec<Vec2f>,

    // EmbossShape.hpp:194
    pub align_type: (i32, i32),
}

impl Default for EmbossShape {
    fn default() -> Self {
        Self {
            shapes_with_ids: ExPolygonsWithIds::new(),
            final_shape: HealedExPolygons {
                expolygons: ExPolygons::new(),
                is_healed: false,
            },
            // EmbossShape.hpp:144  double scale = SCALING_FACTOR;
            // C++ SCALING_FACTOR == 0.00001 (libslic3r.h:58): the mm-per-integer
            // multiplier used as `mm = integer_coord * scale`. This crate inverts the
            // convention and stores `SCALING_FACTOR = 100_000.0` (the integer-per-mm
            // multiplier, i.e. the reciprocal). To match the C++ literal value we must use
            // its reciprocal here, otherwise `scale` would be off by 1e10.
            // FIDELITY-NOTE(F2): crate-wide SCALING_FACTOR uses the inverted (integer-per-mm)
            // convention vs C++ libslic3r.h:58 SCALING_FACTOR=0.00001 (mm-per-integer);
            // reproduce the C++ value locally as 1.0 / SCALING_FACTOR.
            scale: 1.0 / SCALING_FACTOR,
            projection: EmbossProjection::default(),
            fix_3mf_tr: None,
            svg_file: None,
            text_scales: Vec::new(),
            text_cursors: Vec::new(),
            text_absolute_cursors: Vec::new(),
            text_align_offsets: Vec::new(),
            align_type: (0, 0),
        }
    }
}

// EmbossShape.hpp:156
/// SVG source for an embossed shape.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SvgFile {
    // EmbossShape.hpp:157-159
    // File(.svg) path on local computer
    // When empty can't reload from disk
    pub path: String,

    // EmbossShape.hpp:161-164
    // File path into .3mf(.zip)
    // When empty svg is not stored into .3mf file yet.
    // and will create dialog to delete private data on save.
    pub path_in_3mf: String,

    // EmbossShape.hpp:166-168
    // Loaded svg file data.
    // !!! It is not serialized on undo/redo stack
    // std::shared_ptr<NSVGimage> image = nullptr;
    //
    // `NSVGimage` is a runtime parse cache, explicitly NOT serialized. There is no
    // `NSVGimage` type ported yet, so this cache is represented as an absent option and
    // skipped by serde (mirroring "It is not serialized").
    #[serde(skip)]
    pub image: Option<()>, // std::shared_ptr<NSVGimage> image = nullptr;

    // EmbossShape.hpp:170-171
    // Loaded string data from file
    pub file_data: Option<String>, // std::shared_ptr<std::string> file_data = nullptr;
}

impl SvgFile {
    // EmbossShape.hpp:173-179
    // template<class Archive> void save(Archive &ar) const {
    //     // Note: image is only cache it is not neccessary to store
    //
    //     // Store file data as plain string
    //     // For Embossed text file_data are nullptr
    //     ar(path, path_in_3mf, (file_data != nullptr) ? *file_data : std::string(""));
    // }
    /// Cereal `save`: archives `path`, `path_in_3mf`, and the file data string (empty
    /// when `file_data` is null). `image` is cache and is not stored.
    pub fn save_fields(&self) -> (&str, &str, String) {
        (
            self.path.as_str(),
            self.path_in_3mf.as_str(),
            // EmbossShape.hpp:178  (file_data != nullptr) ? *file_data : std::string("")
            match &self.file_data {
                Some(s) => s.clone(),
                None => String::new(),
            },
        )
    }

    // EmbossShape.hpp:180-186
    // template<class Archive> void load(Archive &ar) {
    //     // for restore shared pointer on file data
    //     std::string file_data_str;
    //     ar(path, path_in_3mf, file_data_str);
    //     if (!file_data_str.empty())
    //         file_data = std::make_unique<std::string>(file_data_str);
    // }
    /// Cereal `load`: reads `path`, `path_in_3mf`, and a file-data string; only stores
    /// the string when it is non-empty.
    pub fn load_fields(&mut self, path: String, path_in_3mf: String, file_data_str: String) {
        // EmbossShape.hpp:183  ar(path, path_in_3mf, file_data_str);
        self.path = path;
        self.path_in_3mf = path_in_3mf;
        // EmbossShape.hpp:184-185  if (!file_data_str.empty()) file_data = ...
        if !file_data_str.is_empty() {
            self.file_data = Some(file_data_str);
        }
    }
}

impl EmbossShape {
    // EmbossShape.hpp:195-201
    // undo / redo stack recovery
    // template<class Archive> void save(Archive &ar) const
    // {
    //     // final_shape is not neccessary to store - it is only cache
    //     ar(shapes_with_ids, final_shape, scale, projection, svg_file);
    //     cereal::save(ar, fix_3mf_tr);
    // }
    //
    // The derived `serde::Serialize` archives the fields in declaration order, mirroring
    // the C++ archive order: shapes_with_ids, final_shape, scale, projection, svg_file,
    // then fix_3mf_tr. (Despite the comment, the original stores `final_shape` too.)

    // EmbossShape.hpp:202-206
    // template<class Archive> void load(Archive &ar)
    // {
    //     ar(shapes_with_ids, final_shape, scale, projection, svg_file);
    //     cereal::load(ar, fix_3mf_tr);
    // }
    //
    // Mirrored by the derived `serde::Deserialize`.
}

// EmbossShape.hpp:210-214
// Serialization through the Cereal library
// namespace cereal {
// template<class Archive> void serialize(Archive &ar, Slic3r::ExPolygonsWithId &o) { ar(o.id, o.expoly, o.is_healed); }
// template<class Archive> void serialize(Archive &ar, Slic3r::HealedExPolygons &o) { ar(o.expolygons, o.is_healed); }
// }; // namespace cereal
//
// In Rust these external Cereal `serialize` functions are provided by the derived
// `serde::Serialize`/`Deserialize` on `ExPolygonsWithId` (fields id, expoly, is_healed)
// and on `HealedExPolygons` (fields expolygons, is_healed), whose field orders match the
// `ar(...)` argument orders above.

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn emboss_projection_defaults() {
        // EmbossShape.hpp:21,25,26
        let p = EmbossProjection::default();
        assert_eq!(p.depth, 2.0);
        assert!(!p.use_surface);
        assert_eq!(p.embeded_depth, 0.0);
    }

    #[test]
    fn emboss_projection_eq() {
        // EmbossShape.hpp:27-29
        let a = EmbossProjection::default();
        let b = EmbossProjection::default();
        assert_eq!(a, b);
        let c = EmbossProjection {
            depth: 3.0,
            ..EmbossProjection::default()
        };
        assert_ne!(a, c);
    }

    #[test]
    fn emboss_shape_default_scale_is_scaling_factor() {
        // EmbossShape.hpp:144  double scale = SCALING_FACTOR; (C++ SCALING_FACTOR == 0.00001)
        // The crate's SCALING_FACTOR is the inverted (100_000.0) convention; the C++ literal
        // value is its reciprocal. See FIDELITY-NOTE(F2) on the Default impl.
        let s = EmbossShape::default();
        assert_eq!(s.scale, 1.0 / SCALING_FACTOR);
        assert!(s.shapes_with_ids.is_empty());
        assert!(s.fix_3mf_tr.is_none());
        assert!(s.svg_file.is_none());
        assert_eq!(s.align_type, (0, 0));
    }

    #[test]
    fn expolygons_with_id_default_healed() {
        // EmbossShape.hpp:126  bool is_healed = true;
        let e = ExPolygonsWithId::new(7, ExPolygons::new());
        assert_eq!(e.id, 7);
        assert!(e.is_healed);
    }

    #[test]
    fn svg_file_save_empty_when_no_data() {
        // EmbossShape.hpp:178  (file_data != nullptr) ? *file_data : std::string("")
        let f = SvgFile::default();
        let (path, path_in_3mf, data) = f.save_fields();
        assert_eq!(path, "");
        assert_eq!(path_in_3mf, "");
        assert_eq!(data, "");
    }

    #[test]
    fn svg_file_load_skips_empty() {
        // EmbossShape.hpp:184-185  if (!file_data_str.empty()) ...
        let mut f = SvgFile::default();
        f.load_fields("a.svg".to_string(), "3mf/a.svg".to_string(), String::new());
        assert_eq!(f.path, "a.svg");
        assert_eq!(f.path_in_3mf, "3mf/a.svg");
        assert!(f.file_data.is_none());

        let mut g = SvgFile::default();
        g.load_fields("b.svg".to_string(), String::new(), "<svg/>".to_string());
        assert_eq!(g.file_data.as_deref(), Some("<svg/>"));
    }
}
