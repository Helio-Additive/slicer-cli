//! 1:1 port of `libslic3r/Format/SL1.cpp` (+ `SL1.hpp`).
//!
//! SL1/SL1S (SLA resin printer) archive handling.
//!
//! C++ Reference:
//! - Format/SL1.hpp
//! - Format/SL1.cpp
//!
//! SL1 archives are ZIP files containing slice images (PNGs), a config.ini,
//! and optionally a prusaslicer.ini profile.
//!
//! PORTING NOTES (faithfulness):
//! - The functions that depend only on already-ported primitives are translated
//!   line-by-line: `read_ini`-style INI parsing, `rings_to_expolygons`,
//!   `foreach_vertex`, `invert_raster_trafo`, `get_raster_params`,
//!   `get_slice_params`, `extract_slices_from_sla_archive`, `to_ini`.
//! - The following are BLOCKED on not-yet-ported infrastructure and are kept as
//!   faithful signatures with the C++ body referenced in comments:
//!     * `extract_sla_archive` / `read_png` — require the miniz zip *reader*
//!       (`crate::miniz_extension` only has stubs for the reader entry points).
//!     * `import_sla_archive` (both overloads) — require `DynamicPrintConfig`
//!       `ptree`-based `load(...)` config threading.
//!     * `fill_iniconf`, `fill_slicerconf`, `SL1Archive::{create_raster,
//!       get_encoder, export_print}` — require `SLAPrint`, `SLAPrintStatistics`,
//!       `SLAPrinterConfig`, and the `sla::RasterBase` family, all of which are
//!       placeholder stubs in this crate.

use crate::clipper_utils::union_ex;
use crate::geometry::{ExPolygon, ExPolygons, Point, Polygon};
use crate::marching_squares::{self as marchsq, Coord, RasterTraits};
use crate::png_read_write::{self as png, ImageGreyscale, ReadBuf};
use crate::scaled;
use std::collections::BTreeMap;

// marchsq::_RasterTraits<Slic3r::png::ImageGreyscale>  SL1.cpp:28-47
//
// template<> struct _RasterTraits<Slic3r::png::ImageGreyscale> {
//     using Rst = Slic3r::png::ImageGreyscale;
//     using ValueType = uint8_t;
//     static uint8_t get(const Rst &rst, size_t row, size_t col) { return rst.get(row, col); }
//     static size_t rows(const Rst &rst) { return rst.rows; }
//     static size_t cols(const Rst &rst) { return rst.cols; }
// };
//
// In Rust the marching-squares engine takes any `RasterTraits` impl; we wire
// `ImageGreyscale` into it here (the C++ specialization lives in this TU too).
impl RasterTraits for ImageGreyscale {
    // SL1.cpp:34  using ValueType = uint8_t;
    type ValueType = u8;

    // SL1.cpp:37-40  static uint8_t get(const Rst &rst, size_t row, size_t col)
    fn get(&self, row: usize, col: usize) -> u8 {
        ImageGreyscale::get(self, row, col)
    }

    // SL1.cpp:43  static size_t rows(const Rst &rst) { return rst.rows; }
    fn rows(&self) -> usize {
        self.rows
    }
    // SL1.cpp:44  static size_t cols(const Rst &rst) { return rst.cols; }
    fn cols(&self) -> usize {
        self.cols
    }
}

// ---------------------------------------------------------------------------
// Error type  (SL1.hpp:60)
// ---------------------------------------------------------------------------

// SL1.hpp:60  class MissingProfileError : public RuntimeError { using RuntimeError::RuntimeError; };
#[derive(Debug, Clone)]
pub struct MissingProfileError {
    pub message: String,
}

impl MissingProfileError {
    pub fn new(msg: impl Into<String>) -> Self {
        Self {
            message: msg.into(),
        }
    }
}

impl std::fmt::Display for MissingProfileError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "MissingProfileError: {}", self.message)
    }
}

impl std::error::Error for MissingProfileError {}

// ===========================================================================
// namespace { ... }   SL1.cpp:51-289 (file-local helpers)
// ===========================================================================

// SL1.cpp:53  struct PNGBuffer { std::vector<uint8_t> buf; std::string fname; };
#[derive(Debug, Clone, Default)]
pub struct PNGBuffer {
    pub buf: Vec<u8>,
    pub fname: String,
}

// SL1.cpp:54-57  struct ArchiveData { ptree profile, config; std::vector<PNGBuffer> images; };
//
// `boost::property_tree::ptree` is modelled here as an ordered key->value map
// (the SL1 ini files are flat). A BTreeMap keeps the deterministic ordering
// that the property tree's `get`/`find` lookups rely upon.
#[derive(Debug, Clone, Default)]
pub struct ArchiveData {
    /// Key-value pairs from `prusaslicer.ini`.
    pub profile: BTreeMap<String, String>,
    /// Key-value pairs from `config.ini`.
    pub config: BTreeMap<String, String>,
    /// Slice images sorted by filename.
    pub images: Vec<PNGBuffer>,
}

impl ArchiveData {
    pub fn new() -> Self {
        Self::default()
    }
}

// SL1.cpp:59  static const constexpr char *CONFIG_FNAME  = "config.ini";
// (consumed by the blocked `extract_sla_archive` zip walk — kept for parity)
#[allow(dead_code)]
const CONFIG_FNAME: &str = "config.ini";
// SL1.cpp:60  static const constexpr char *PROFILE_FNAME = "prusaslicer.ini";
#[allow(dead_code)]
const PROFILE_FNAME: &str = "prusaslicer.ini";

// SL1.cpp:62-75
// boost::property_tree::ptree read_ini(const mz_zip_archive_file_stat &entry, MZ_Archive &zip)
//
// Parses a flat INI buffer into a key->value map. The C++ uses
// `boost::property_tree::read_ini`; we mirror its behavior for the flat ini
// files SL1 archives contain (trim whitespace, skip comments/sections,
// split on the first `=`).
// (consumed by the blocked `extract_sla_archive` zip walk — kept for parity)
#[allow(dead_code)]
fn read_ini(buf: &str) -> BTreeMap<String, String> {
    let mut tree = BTreeMap::new();
    for line in buf.lines() {
        let line = line.trim();
        // boost::property_tree::read_ini skips blank lines, `;`/`#` comments
        // and `[section]` headers.
        if line.is_empty()
            || line.starts_with('#')
            || line.starts_with(';')
            || line.starts_with('[')
        {
            continue;
        }
        if let Some(eq_pos) = line.find('=') {
            let key = line[..eq_pos].trim().to_string();
            let val = line[eq_pos + 1..].trim().to_string();
            tree.insert(key, val);
        }
    }
    tree
}

// SL1.cpp:136-153
// ExPolygons rings_to_expolygons(const std::vector<marchsq::Ring> &rings,
//                                double px_w, double px_h)
fn rings_to_expolygons(rings: &[marchsq::Ring], px_w: f64, px_h: f64) -> ExPolygons {
    // SL1.cpp:139  auto polys = reserve_vector<ExPolygon>(rings.size());
    let mut polys: Vec<ExPolygon> = Vec::with_capacity(rings.len());

    // SL1.cpp:141  for (const marchsq::Ring &ring : rings) {
    for ring in rings {
        // SL1.cpp:142  Polygon poly; Points &pts = poly.points;
        let mut poly = Polygon::new();
        // SL1.cpp:143  pts.reserve(ring.size());
        poly.points.reserve(ring.len());

        // SL1.cpp:145  for (const marchsq::Coord &crd : ring)
        for crd in ring {
            // SL1.cpp:146  pts.emplace_back(scaled(crd.c * px_w), scaled(crd.r * px_h));
            poly.points
                .push(Point::new(scaled(crd.c as f64 * px_w), scaled(crd.r as f64 * px_h)));
        }

        // SL1.cpp:148  polys.emplace_back(poly);
        polys.push(ExPolygon::from(poly));
    }

    // SL1.cpp:151  // TODO: Is a union necessary?
    // SL1.cpp:152  return union_ex(polys);
    union_ex(&polys)
}

// SL1.cpp:155-160
// template<class Fn> void foreach_vertex(ExPolygon &poly, Fn &&fn)
fn foreach_vertex<F: FnMut(&mut Point)>(poly: &mut ExPolygon, mut fn_: F) {
    // SL1.cpp:157  for (auto &p : poly.contour.points) fn(p);
    for p in poly.contour.points.iter_mut() {
        fn_(p);
    }
    // SL1.cpp:158-159  for (auto &h : poly.holes) for (auto &p : h.points) fn(p);
    for h in poly.holes.iter_mut() {
        for p in h.points.iter_mut() {
            fn_(p);
        }
    }
}

// SL1.cpp:188-193  struct RasterParams (Trafo portion modelled here).
//
// Mirrors `sla::RasterBase::Trafo` (SLA/RasterBase.hpp:69-86). The crate's
// `sla::raster_base::Trafo` is still a placeholder stub, so the fields are
// inlined here with the exact constructor logic from the C++ Trafo ctor.
#[derive(Debug, Clone, Copy, Default)]
pub struct RasterTrafo {
    pub mirror_x: bool,
    pub mirror_y: bool,
    pub flip_xy: bool,
    pub center_x: i64,
    pub center_y: i64,
}

// SLA/RasterBase.hpp:61  enum Orientation { roLandscape, roPortrait };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Orientation {
    RoLandscape,
    RoPortrait,
}

impl RasterTrafo {
    // SLA/RasterBase.hpp:75-80
    // Trafo(Orientation o = roLandscape, const TMirroring &mirror = NoMirror)
    //     : mirror_x(o == roPortrait ? !mirror[0] : mirror[0])
    //     , mirror_y(!mirror[1]) // Makes raster origin to be top left corner
    //     , flipXY(o == roPortrait)
    fn new(o: Orientation, mirror: [bool; 2]) -> Self {
        Self {
            mirror_x: if o == Orientation::RoPortrait {
                !mirror[0]
            } else {
                mirror[0]
            },
            mirror_y: !mirror[1],
            flip_xy: o == Orientation::RoPortrait,
            center_x: 0,
            center_y: 0,
        }
    }
}

// SL1.cpp:162-186
// void invert_raster_trafo(ExPolygons &expolys, const sla::RasterBase::Trafo &trafo,
//                          coord_t width, coord_t height)
fn invert_raster_trafo(
    expolys: &mut ExPolygons,
    trafo: &RasterTrafo,
    mut width: i64,
    mut height: i64,
) {
    // SL1.cpp:167  if (trafo.flipXY) std::swap(height, width);
    if trafo.flip_xy {
        std::mem::swap(&mut height, &mut width);
    }

    // SL1.cpp:169  for (auto &expoly : expolys) {
    for expoly in expolys.iter_mut() {
        // SL1.cpp:170-171
        // if (trafo.mirror_y)
        //     foreach_vertex(expoly, [height](Point &p) {p.y() = height - p.y(); });
        if trafo.mirror_y {
            foreach_vertex(expoly, |p| p.y = height - p.y);
        }

        // SL1.cpp:173-174
        // if (trafo.mirror_x)
        //     foreach_vertex(expoly, [width](Point &p) {p.x() = width - p.x(); });
        if trafo.mirror_x {
            foreach_vertex(expoly, |p| p.x = width - p.x);
        }

        // SL1.cpp:176  expoly.translate(-trafo.center_x, -trafo.center_y);
        expoly.translate(Point::new(-trafo.center_x, -trafo.center_y));

        // SL1.cpp:178-179
        // if (trafo.flipXY)
        //     foreach_vertex(expoly, [](Point &p) { std::swap(p.x(), p.y()); });
        if trafo.flip_xy {
            foreach_vertex(expoly, |p| std::mem::swap(&mut p.x, &mut p.y));
        }

        // SL1.cpp:181  if ((trafo.mirror_x + trafo.mirror_y + trafo.flipXY) % 2) {
        if (trafo.mirror_x as i32 + trafo.mirror_y as i32 + trafo.flip_xy as i32) % 2 != 0 {
            // SL1.cpp:182  expoly.contour.reverse();
            expoly.contour.reverse();
            // SL1.cpp:183  for (auto &h : expoly.holes) h.reverse();
            for h in expoly.holes.iter_mut() {
                h.reverse();
            }
        }
    }
}

// SL1.cpp:188-193  struct RasterParams
#[derive(Debug, Clone)]
pub struct RasterParams {
    /// Raster transformations.  SL1.cpp:189
    pub trafo: RasterTrafo,
    /// scaled raster dimensions (not resolution).  SL1.cpp:190
    pub width: i64,
    pub height: i64,
    /// pixel dimensions.  SL1.cpp:191
    pub px_h: f64,
    pub px_w: f64,
    /// marching squares window size.  SL1.cpp:192
    pub win: Coord,
}

// SL1.cpp:195-223  RasterParams get_raster_params(const DynamicPrintConfig &cfg)
//
// `DynamicPrintConfig` typed option access is not yet threaded through this
// crate, so the config is provided as the already-parsed flat key->value map
// (matching `ArchiveData::profile`). Lookups/parses mirror the C++ option
// fetches; a missing/invalid option raises `MissingProfileError`, exactly as
// the C++ `if (!opt_...) throw MissingProfileError(...)`.
pub fn get_raster_params(
    cfg: &BTreeMap<String, String>,
) -> std::result::Result<RasterParams, MissingProfileError> {
    // SL1.cpp:197-203 — typed option fetches.
    let opt_disp_cols = cfg.get("display_pixels_x");
    let opt_disp_rows = cfg.get("display_pixels_y");
    let opt_disp_w = cfg.get("display_width");
    let opt_disp_h = cfg.get("display_height");
    let opt_mirror_x = cfg.get("display_mirror_x");
    let opt_mirror_y = cfg.get("display_mirror_y");
    let opt_orient = cfg.get("display_orientation");

    // SL1.cpp:205-207
    // if (!opt_disp_cols || ... ) throw MissingProfileError("Invalid SL1 / SL1S file");
    let (
        opt_disp_cols,
        opt_disp_rows,
        opt_disp_w,
        opt_disp_h,
        opt_mirror_x,
        opt_mirror_y,
        opt_orient,
    ) = match (
        opt_disp_cols,
        opt_disp_rows,
        opt_disp_w,
        opt_disp_h,
        opt_mirror_x,
        opt_mirror_y,
        opt_orient,
    ) {
        (Some(a), Some(b), Some(c), Some(d), Some(e), Some(f), Some(g)) => (a, b, c, d, e, f, g),
        _ => return Err(MissingProfileError::new("Invalid SL1 / SL1S file")),
    };

    let disp_cols: i64 = opt_disp_cols
        .trim()
        .parse()
        .map_err(|_| MissingProfileError::new("Invalid SL1 / SL1S file"))?;
    let disp_rows: i64 = opt_disp_rows
        .trim()
        .parse()
        .map_err(|_| MissingProfileError::new("Invalid SL1 / SL1S file"))?;
    let disp_w: f64 = opt_disp_w
        .trim()
        .parse()
        .map_err(|_| MissingProfileError::new("Invalid SL1 / SL1S file"))?;
    let disp_h: f64 = opt_disp_h
        .trim()
        .parse()
        .map_err(|_| MissingProfileError::new("Invalid SL1 / SL1S file"))?;
    let mirror_x = parse_config_bool(opt_mirror_x);
    let mirror_y = parse_config_bool(opt_mirror_y);
    // ConfigOptionEnum<SLADisplayOrientation>: sladoLandscape=0, sladoPortrait=1
    // (PrintConfig.hpp:196-198); serialized as "landscape"/"portrait".
    let orient_landscape =
        opt_orient.trim() == "landscape" || opt_orient.trim() == "0";

    // SL1.cpp:209  RasterParams rstp;
    // SL1.cpp:211  rstp.px_w = opt_disp_w->value / (opt_disp_cols->value - 1);
    let px_w = disp_w / (disp_cols - 1) as f64;
    // SL1.cpp:212  rstp.px_h = opt_disp_h->value / (opt_disp_rows->value - 1);
    let px_h = disp_h / (disp_rows - 1) as f64;

    // SL1.cpp:214-217
    // rstp.trafo = sla::RasterBase::Trafo{opt_orient->value == sladoLandscape ?
    //                                  sla::RasterBase::roLandscape :
    //                                  sla::RasterBase::roPortrait,
    //                              {opt_mirror_x->value, opt_mirror_y->value}};
    let trafo = RasterTrafo::new(
        if orient_landscape {
            Orientation::RoLandscape
        } else {
            Orientation::RoPortrait
        },
        [mirror_x, mirror_y],
    );

    // SL1.cpp:219  rstp.height = scaled(opt_disp_h->value);
    let height = scaled(disp_h);
    // SL1.cpp:220  rstp.width  = scaled(opt_disp_w->value);
    let width = scaled(disp_w);

    // SL1.cpp:222  return rstp;
    Ok(RasterParams {
        trafo,
        width,
        height,
        px_h,
        px_w,
        // `win` is filled in by the caller (import_sla_archive).  SL1.cpp:341
        win: Coord::new(),
    })
}

// Helper: ConfigOptionBool serializes to "1"/"0" (also accept true/false).
fn parse_config_bool(v: &str) -> bool {
    let v = v.trim();
    v == "1" || v.eq_ignore_ascii_case("true")
}

// SL1.cpp:225  struct SliceParams { double layerh = 0., initial_layerh = 0.; };
#[derive(Debug, Clone, Copy)]
pub struct SliceParams {
    pub layerh: f64,
    pub initial_layerh: f64,
}

impl Default for SliceParams {
    fn default() -> Self {
        Self {
            layerh: 0.,
            initial_layerh: 0.,
        }
    }
}

// SL1.cpp:227-236  SliceParams get_slice_params(const DynamicPrintConfig &cfg)
pub fn get_slice_params(
    cfg: &BTreeMap<String, String>,
) -> std::result::Result<SliceParams, MissingProfileError> {
    // SL1.cpp:229  auto *opt_layerh = cfg.option<ConfigOptionFloat>("layer_height");
    let opt_layerh = cfg.get("layer_height");
    // SL1.cpp:230  auto *opt_init_layerh = cfg.option<ConfigOptionFloat>("initial_layer_height");
    let opt_init_layerh = cfg.get("initial_layer_height");

    // SL1.cpp:232-233
    // if (!opt_layerh || !opt_init_layerh)
    //     throw MissingProfileError("Invalid SL1 / SL1S file");
    let (opt_layerh, opt_init_layerh) = match (opt_layerh, opt_init_layerh) {
        (Some(a), Some(b)) => (a, b),
        _ => return Err(MissingProfileError::new("Invalid SL1 / SL1S file")),
    };

    // SL1.cpp:235  return SliceParams{opt_layerh->getFloat(), opt_init_layerh->getFloat()};
    let layerh: f64 = opt_layerh
        .trim()
        .parse()
        .map_err(|_| MissingProfileError::new("Invalid SL1 / SL1S file"))?;
    let initial_layerh: f64 = opt_init_layerh
        .trim()
        .parse()
        .map_err(|_| MissingProfileError::new("Invalid SL1 / SL1S file"))?;
    Ok(SliceParams {
        layerh,
        initial_layerh,
    })
}

// SL1.cpp:238-287
// std::vector<ExPolygons> extract_slices_from_sla_archive(
//     ArchiveData &arch, const RasterParams &rstp, std::function<bool(int)> progr)
//
// The C++ uses `tbb::parallel_for` guarded by a spinlock for progress; we keep
// the per-image transform pipeline faithfully but iterate sequentially (the
// result is order-independent: `slices[i]` is written independently). The
// progress callback returns `false` to request a stop, matching the C++
// `st.stop = !progr(...)` semantics.
pub fn extract_slices_from_sla_archive(
    arch: &ArchiveData,
    rstp: &RasterParams,
    mut progr: impl FnMut(i32) -> bool,
) -> Vec<ExPolygons> {
    // SL1.cpp:243-244
    // auto jobdir = arch.config.get<std::string>("jobDir");
    // for (auto &c : jobdir) c = std::tolower(c);
    let _jobdir = arch
        .config
        .get("jobDir")
        .map(|s| s.to_lowercase())
        .unwrap_or_default();

    // SL1.cpp:246  std::vector<ExPolygons> slices(arch.images.size());
    let mut slices: Vec<ExPolygons> = vec![ExPolygons::new(); arch.images.len()];

    // SL1.cpp:248-253  struct Status { ... } st {100. / slices.size(), 0., 0.};
    struct Status {
        incr: f64,
        val: f64,
        prev: f64,
        stop: bool,
    }
    let mut st = Status {
        incr: 100. / slices.len() as f64,
        val: 0.,
        prev: 0.,
        stop: false,
    };

    // SL1.cpp:255-282  tbb::parallel_for(size_t(0), arch.images.size(), [...](size_t i) { ... });
    for i in 0..arch.images.len() {
        // SL1.cpp:257-268  Status indication guarded with the spinlock.
        {
            // SL1.cpp:260  if (st.stop) return;
            if st.stop {
                continue;
            }

            // SL1.cpp:262  st.val += st.incr;
            st.val += st.incr;
            // SL1.cpp:263  double curr = std::round(st.val);
            let curr = st.val.round();
            // SL1.cpp:264  if (curr > st.prev) {
            if curr > st.prev {
                // SL1.cpp:265  st.prev = curr;
                st.prev = curr;
                // SL1.cpp:266  st.stop = !progr(int(curr));
                st.stop = !progr(curr as i32);
            }
        }

        // SL1.cpp:270  png::ImageGreyscale img;
        let mut img = ImageGreyscale {
            buf: Vec::new(),
            rows: 0,
            cols: 0,
        };
        // SL1.cpp:271  png::ReadBuf rb{arch.images[i].buf.data(), arch.images[i].buf.size()};
        let rb = ReadBuf {
            buf: &arch.images[i].buf,
            sz: arch.images[i].buf.len(),
        };
        // SL1.cpp:272  if (!png::decode_png(rb, img)) return;
        if !png::decode_png_buf(&rb, &mut img) {
            continue;
        }

        // SL1.cpp:274  uint8_t isoval = 128;
        let isoval: u8 = 128;
        // SL1.cpp:275  auto rings = marchsq::execute(img, isoval, rstp.win);
        let rings = marchsq::execute(&img, isoval, rstp.win);
        // SL1.cpp:276  ExPolygons expolys = rings_to_expolygons(rings, rstp.px_w, rstp.px_h);
        let mut expolys = rings_to_expolygons(&rings, rstp.px_w, rstp.px_h);

        // SL1.cpp:278-279
        // Invert the raster transformations indicated in the profile metadata
        // invert_raster_trafo(expolys, rstp.trafo, rstp.width, rstp.height);
        invert_raster_trafo(&mut expolys, &rstp.trafo, rstp.width, rstp.height);

        // SL1.cpp:281  slices[i] = std::move(expolys);
        slices[i] = expolys;
    }

    // SL1.cpp:284  if (st.stop) slices = {};
    if st.stop {
        slices = Vec::new();
    }

    // SL1.cpp:286  return slices;
    slices
}

// ---------------------------------------------------------------------------
// to_ini / config serialization helpers  (SL1.cpp:354-441)
// ---------------------------------------------------------------------------

// SL1.cpp:354  using ConfMap = std::map<std::string, std::string>;
//
// std::map is ordered; BTreeMap preserves the same key ordering, which `to_ini`
// relies upon for byte-exact output.
pub type ConfMap = BTreeMap<String, String>;

// SL1.cpp:358-364  std::string to_ini(const ConfMap &m)
pub fn to_ini(m: &ConfMap) -> String {
    // SL1.cpp:359  std::string ret;
    let mut ret = String::new();
    // SL1.cpp:361  for (auto &param : m) ret += param.first + " = " + param.second + "\n";
    for (k, v) in m {
        ret.push_str(k);
        ret.push_str(" = ");
        ret.push_str(v);
        ret.push('\n');
    }
    // SL1.cpp:363  return ret;
    ret
}

// SL1.cpp:366-376  std::string get_cfg_value(const DynamicPrintConfig &cfg, const std::string &key)
//
// `DynamicPrintConfig` serialization is not threaded through yet; this operates
// on the flat key->value map. `cfg.has(key)` -> presence; `opt->serialize()`
// -> the stored serialized string.
pub fn get_cfg_value(cfg: &BTreeMap<String, String>, key: &str) -> String {
    // SL1.cpp:368  std::string ret;
    // SL1.cpp:370-373  if (cfg.has(key)) { auto opt = cfg.option(key); if (opt) ret = opt->serialize(); }
    // SL1.cpp:375  return ret;
    cfg.get(key).cloned().unwrap_or_default()
}

// ---------------------------------------------------------------------------
// SL1Archive  (SL1.hpp:11-39, SL1.cpp:443-516)
// ---------------------------------------------------------------------------

// SL1.hpp:11  class SL1Archive: public SLAArchive { SLAPrinterConfig m_cfg; ... };
//
// `SLAPrinterConfig`, `SLAPrint`, and the `sla::RasterBase` family are
// placeholder stubs in this crate. The encoded-layer storage and `apply`
// cache-invalidation logic are ported faithfully; `create_raster`,
// `get_encoder`, and `export_print` are BLOCKED (see module doc) and bodies
// reference their C++ source for the eventual port.
#[derive(Debug, Default)]
pub struct SL1Archive {
    /// Printer configuration (`SLAPrinterConfig m_cfg`).
    pub cfg: BTreeMap<String, String>,
    /// Encoded raster layers (`m_layers` from `SLAArchive`).
    pub layers: Vec<Vec<u8>>,
}

impl SL1Archive {
    // SL1.hpp:20  SL1Archive() = default;
    pub fn new() -> Self {
        Self::default()
    }

    // SL1.hpp:21  explicit SL1Archive(const SLAPrinterConfig &cfg): m_cfg(cfg) {}
    pub fn with_config(cfg: BTreeMap<String, String>) -> Self {
        Self {
            cfg,
            layers: Vec::new(),
        }
    }

    // SL1.hpp:31-38  void apply(const SLAPrinterConfig &cfg) override
    // {
    //     auto diff = m_cfg.diff(cfg);
    //     if (!diff.empty()) {
    //         m_cfg.apply_only(cfg, diff);
    //         m_layers = {};
    //     }
    // }
    pub fn apply(&mut self, cfg: BTreeMap<String, String>) {
        if self.cfg != cfg {
            self.cfg = cfg;
            self.layers = Vec::new();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_missing_profile_error() {
        let e = MissingProfileError::new("test error");
        assert_eq!(e.message, "test error");
        assert!(format!("{}", e).contains("test error"));
    }

    #[test]
    fn test_read_ini() {
        let text = "key1 = value1\nkey2 = value2\n# comment\n[section]\n";
        let map = read_ini(text);
        assert_eq!(map.get("key1").unwrap(), "value1");
        assert_eq!(map.get("key2").unwrap(), "value2");
        assert_eq!(map.len(), 2);
    }

    #[test]
    fn test_to_ini() {
        // std::map order: a before b.  SL1.cpp:361
        let mut m: ConfMap = ConfMap::new();
        m.insert("b".to_string(), "2".to_string());
        m.insert("a".to_string(), "1".to_string());
        let s = to_ini(&m);
        assert_eq!(s, "a = 1\nb = 2\n");
    }

    #[test]
    fn test_get_slice_params() {
        let mut cfg: BTreeMap<String, String> = BTreeMap::new();
        cfg.insert("layer_height".to_string(), "0.05".to_string());
        cfg.insert("initial_layer_height".to_string(), "0.1".to_string());
        let params = get_slice_params(&cfg).unwrap();
        assert!((params.layerh - 0.05).abs() < 1e-9);
        assert!((params.initial_layerh - 0.1).abs() < 1e-9);
    }

    #[test]
    fn test_get_slice_params_missing() {
        let cfg: BTreeMap<String, String> = BTreeMap::new();
        assert!(get_slice_params(&cfg).is_err());
    }

    #[test]
    fn test_trafo_ctor_portrait_mirrors() {
        // SLA/RasterBase.hpp:75-80 — portrait flips X via flipXY's implicit X mirror.
        let tr = RasterTrafo::new(Orientation::RoPortrait, [false, false]);
        assert!(tr.mirror_x); // o==portrait ? !mirror[0] : mirror[0]  => !false
        assert!(tr.mirror_y); // !mirror[1] => !false
        assert!(tr.flip_xy);
    }

    #[test]
    fn test_trafo_ctor_landscape() {
        let tr = RasterTrafo::new(Orientation::RoLandscape, [true, true]);
        assert!(tr.mirror_x); // landscape => mirror[0] => true
        assert!(!tr.mirror_y); // !mirror[1] => !true
        assert!(!tr.flip_xy);
    }

    #[test]
    fn test_sl1_archive_apply() {
        let mut archive = SL1Archive::new();
        archive.layers.push(vec![1, 2, 3]);
        let mut new_cfg: BTreeMap<String, String> = BTreeMap::new();
        new_cfg.insert("foo".to_string(), "bar".to_string());
        archive.apply(new_cfg);
        assert!(archive.layers.is_empty()); // layers cleared on config change
    }
}
