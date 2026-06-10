//! Faithful 1:1 line-by-line port of BambuStudio `src/libslic3r/SLA/RasterBase.cpp`
//! (+ `src/libslic3r/SLA/RasterBase.hpp`).
//!
//! C++ Reference:
//! - SLA/RasterBase.hpp
//! - SLA/RasterBase.cpp
//!
//! The PNG encoder in the C++ source calls miniz's
//! `tdefl_write_image_to_png_file_in_memory` (miniz.c:2180-2184, which forwards
//! to `tdefl_write_image_to_png_file_in_memory_ex` at miniz.c:2102-2179 with
//! level 6 / no flip). That helper is ported here privately. The deflate
//! bitstream is produced by `flate2`'s pure-Rust backend (`miniz_oxide`, a
//! direct port of miniz's tdefl): `Compression::new(6)` with a zlib header maps
//! to the exact same compressor flags (`NUM_PROBES[6] == 128 |
//! TDEFL_WRITE_ZLIB_HEADER`) that miniz.c:2125 uses
//! (`s_tdefl_png_num_probes[6] | TDEFL_WRITE_ZLIB_HEADER`), so the output is
//! byte-identical to the C++ build. wasm-safe: no native deps.
//!
//! BLOCKED SYMBOL: `create_raster_grayscale_aa` (RasterBase.cpp:65-81) — see
//! the note at the bottom of this file; it constructs the AGG-based rasterizer
//! types from SLA/AGGRaster.hpp which are not yet ported.

use std::io::Write;

use crate::geometry::{ExPolygon, Point};
use crate::Coord;

// =============================================================================
// RasterBase.hpp
// =============================================================================

// RasterBase.hpp:18-19  Raw byte buffer paired with its size. Suitable for
// compressed image data.
// RasterBase.hpp:19  class EncodedRaster
#[derive(Debug, Clone, Default)]
pub struct EncodedRaster {
    // RasterBase.hpp:21  std::vector<uint8_t> m_buffer;
    m_buffer: Vec<u8>,
    // RasterBase.hpp:22  std::string m_ext;
    m_ext: String,
}

impl EncodedRaster {
    // RasterBase.hpp:24  EncodedRaster() = default;
    // (covered by `#[derive(Default)]` above)

    // RasterBase.hpp:25-27
    // explicit EncodedRaster(std::vector<uint8_t> &&buf, std::string ext)
    //     : m_buffer(std::move(buf)), m_ext(std::move(ext))
    pub fn new(buf: Vec<u8>, ext: String) -> Self {
        Self {
            m_buffer: buf,
            m_ext: ext,
        }
    }

    // RasterBase.hpp:29  size_t size() const { return m_buffer.size(); }
    pub fn size(&self) -> usize {
        self.m_buffer.len()
    }

    // RasterBase.hpp:30  const void * data() const { return m_buffer.data(); }
    pub fn data(&self) -> &[u8] {
        &self.m_buffer
    }

    // RasterBase.hpp:31  const char * extension() const { return m_ext.c_str(); }
    pub fn extension(&self) -> &str {
        &self.m_ext
    }
}

// RasterBase.hpp:34  /// Type that represents a resolution in pixels.
// RasterBase.hpp:35-43  struct Resolution
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Resolution {
    // RasterBase.hpp:37  size_t width_px  = 0;
    pub width_px: usize,
    // RasterBase.hpp:38  size_t height_px = 0;
    pub height_px: usize,
}

impl Resolution {
    // RasterBase.hpp:40  Resolution() = default;
    // (covered by `#[derive(Default)]` above; both fields default to 0)

    // RasterBase.hpp:41  Resolution(size_t w, size_t h) : width_px(w), height_px(h) {}
    pub fn new(w: usize, h: usize) -> Self {
        Self {
            width_px: w,
            height_px: h,
        }
    }

    // RasterBase.hpp:42  size_t pixels() const { return width_px * height_px; }
    pub fn pixels(&self) -> usize {
        self.width_px * self.height_px
    }
}

// RasterBase.hpp:45  /// Types that represents the dimension of a pixel in millimeters.
// RasterBase.hpp:46-53  struct PixelDim
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PixelDim {
    // RasterBase.hpp:48  double w_mm = 1.;
    pub w_mm: f64,
    // RasterBase.hpp:49  double h_mm = 1.;
    pub h_mm: f64,
}

impl Default for PixelDim {
    // RasterBase.hpp:51  PixelDim() = default; (member initializers = 1.)
    fn default() -> Self {
        Self { w_mm: 1., h_mm: 1. }
    }
}

impl PixelDim {
    // RasterBase.hpp:52
    // PixelDim(double px_width_mm, double px_height_mm) : w_mm(px_width_mm), h_mm(px_height_mm) {}
    pub fn new(px_width_mm: f64, px_height_mm: f64) -> Self {
        Self {
            w_mm: px_width_mm,
            h_mm: px_height_mm,
        }
    }
}

// RasterBase.hpp:55-56
// using RasterEncoder =
//     std::function<EncodedRaster(const void *ptr, size_t w, size_t h, size_t num_components)>;
//
// `const void *ptr` carries `w * h * num_components` bytes of pixel data; the
// memory-safe Rust equivalent is a byte slice.
pub type RasterEncoder = Box<dyn FnMut(&[u8], usize, usize, usize) -> EncodedRaster>;

// RasterBase.hpp:61  enum Orientation { roLandscape, roPortrait };
// (nested in `class RasterBase` in C++; hoisted to module scope in Rust)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Orientation {
    RoLandscape,
    RoPortrait,
}

// RasterBase.hpp:63  using TMirroring = std::array<bool, 2>;
pub type TMirroring = [bool; 2];

// RasterBase.hpp:64  static const constexpr TMirroring NoMirror = {false, false};
pub const NO_MIRROR: TMirroring = [false, false];
// RasterBase.hpp:65  static const constexpr TMirroring MirrorX  = {true, false};
pub const MIRROR_X: TMirroring = [true, false];
// RasterBase.hpp:66  static const constexpr TMirroring MirrorY  = {false, true};
pub const MIRROR_Y: TMirroring = [false, true];
// RasterBase.hpp:67  static const constexpr TMirroring MirrorXY = {true, true};
pub const MIRROR_XY: TMirroring = [true, true];

// RasterBase.hpp:69-85  struct Trafo (nested in `class RasterBase` in C++)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Trafo {
    // RasterBase.hpp:70  bool mirror_x = false, mirror_y = false, flipXY = false;
    pub mirror_x: bool,
    pub mirror_y: bool,
    pub flip_xy: bool,
    // RasterBase.hpp:71  coord_t center_x = 0, center_y = 0;
    pub center_x: Coord,
    pub center_y: Coord,
}

impl Trafo {
    // RasterBase.hpp:73-74
    // Portrait orientation will make sure the drawed polygons are rotated
    // by 90 degrees.
    //
    // RasterBase.hpp:75-80
    // Trafo(Orientation o = roLandscape, const TMirroring &mirror = NoMirror)
    //     // XY flipping implicitly does an X mirror
    //     : mirror_x(o == roPortrait ? !mirror[0] : mirror[0])
    //     , mirror_y(!mirror[1]) // Makes raster origin to be top left corner
    //     , flipXY(o == roPortrait)
    pub fn new(o: Orientation, mirror: TMirroring) -> Self {
        Self {
            // XY flipping implicitly does an X mirror
            mirror_x: if o == Orientation::RoPortrait {
                !mirror[0]
            } else {
                mirror[0]
            },
            mirror_y: !mirror[1], // Makes raster origin to be top left corner
            flip_xy: o == Orientation::RoPortrait,
            center_x: 0,
            center_y: 0,
        }
    }

    // RasterBase.hpp:82
    // TMirroring get_mirror() const { return { (roPortrait ? !mirror_x : mirror_x), mirror_y}; }
    //
    // NOTE (faithful C++ quirk): the condition is the bare enum constant
    // `roPortrait` (== 1), NOT `flipXY`, so it is always truthy and the first
    // component is always `!mirror_x`.
    pub fn get_mirror(&self) -> TMirroring {
        [!self.mirror_x, self.mirror_y]
    }

    // RasterBase.hpp:83
    // Orientation get_orientation() const { return flipXY ? roPortrait : roLandscape; }
    pub fn get_orientation(&self) -> Orientation {
        if self.flip_xy {
            Orientation::RoPortrait
        } else {
            Orientation::RoLandscape
        }
    }

    // RasterBase.hpp:84  Point get_center() const { return {center_x, center_y}; }
    pub fn get_center(&self) -> Point {
        Point::new(self.center_x, self.center_y)
    }
}

impl Default for Trafo {
    // RasterBase.hpp:75  default arguments: Trafo(roLandscape, NoMirror)
    fn default() -> Self {
        Self::new(Orientation::RoLandscape, NO_MIRROR)
    }
}

// RasterBase.hpp:58  class RasterBase
// (abstract base class -> Rust trait; `virtual ~RasterBase() = default;` at
// RasterBase.hpp:87 has no Rust equivalent)
pub trait RasterBase {
    // RasterBase.hpp:89-90
    // /// Draw a polygon with holes.
    // virtual void draw(const ExPolygon& poly) = 0;
    fn draw(&mut self, poly: &ExPolygon);

    // RasterBase.hpp:92-93
    // /// Get the resolution of the raster.
    // virtual Trafo trafo() const = 0;
    fn trafo(&self) -> Trafo;

    // RasterBase.hpp:95  virtual EncodedRaster encode(RasterEncoder encoder) const = 0;
    fn encode(&self, encoder: RasterEncoder) -> EncodedRaster;
}

// RasterBase.hpp:98-100  struct PNGRasterEncoder
#[derive(Debug, Clone, Copy, Default)]
pub struct PNGRasterEncoder;

// RasterBase.hpp:102-104  struct PPMRasterEncoder
#[derive(Debug, Clone, Copy, Default)]
pub struct PPMRasterEncoder;

// =============================================================================
// miniz.c helpers used by RasterBase.cpp:20 (private; see module docs)
// =============================================================================

// miniz.h  #define MZ_CRC32_INIT (0)
const MZ_CRC32_INIT: u32 = 0;

// miniz.c  mz_ulong mz_crc32(mz_ulong crc, const mz_uint8 *ptr, size_t buf_len)
// Standard IEEE CRC-32 (reflected, poly 0xEDB88320), bitwise formulation.
fn mz_crc32(prev: u32, data: &[u8]) -> u32 {
    let mut crc = !prev;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            let mask = (!(crc & 1)).wrapping_add(1); // 0xFFFFFFFF if lsb set else 0
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

// miniz.c:2180-2184
// void *tdefl_write_image_to_png_file_in_memory(const void *pImage, int w, int h,
//                                               int num_chans, size_t *pLen_out)
// { return tdefl_write_image_to_png_file_in_memory_ex(pImage, w, h, num_chans,
//                                                     pLen_out, 6, MZ_FALSE); }
//
// miniz.c:2102-2179  tdefl_write_image_to_png_file_in_memory_ex
// Specialized here for level == 6, flip == MZ_FALSE (the only call site).
// Returns None on compression failure (C++ returns NULL).
fn tdefl_write_image_to_png_file_in_memory(
    p_image: &[u8],
    w: usize,
    h: usize,
    num_chans: usize,
) -> Option<Vec<u8>> {
    // miniz.c:2108  int i, bpl = w * num_chans, y, z;
    let bpl = w * num_chans;

    // miniz.c:2121-2130  After the dummy-header loop the local `z` is 0, so
    // each scanline is prefixed with a single 0x00 PNG filter byte and the
    // whole stream is tdefl-compressed at
    // `s_tdefl_png_num_probes[6] | TDEFL_WRITE_ZLIB_HEADER` (miniz.c:2125).
    // flate2's miniz_oxide backend with Compression::new(6) + zlib header
    // produces the identical flags and bitstream.
    let mut raw: Vec<u8> = Vec::with_capacity((1 + bpl) * h);
    for y in 0..h {
        // miniz.c:2128  tdefl_compress_buffer(pComp, &z, 1, TDEFL_NO_FLUSH);
        raw.push(0u8);
        // miniz.c:2129  (flip is MZ_FALSE, so plain row order)
        raw.extend_from_slice(&p_image[y * bpl..y * bpl + bpl]);
    }
    // miniz.c:2131-2136  TDEFL_FINISH; on failure return NULL
    let mut enc = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::new(6));
    enc.write_all(&raw).ok()?;
    let compressed = enc.finish().ok()?;

    // miniz.c:2138  *pLen_out = out_buf.m_size - 41;
    let len_out = compressed.len();

    // miniz.c:2140  static const mz_uint8 chans[] = { 0x00, 0x00, 0x04, 0x02, 0x06 };
    let chans: [u8; 5] = [0x00, 0x00, 0x04, 0x02, 0x06];
    // miniz.c:2141-2149  mz_uint8 pnghdr[41] = {...};
    // (PNG signature, IHDR length/tag, zeroed IHDR payload + CRC slot,
    //  IDAT length slot, IDAT tag)
    let mut pnghdr: [u8; 41] = [
        0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d, 0x49, 0x48, 0x44,
        0x52, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x44, 0x41, 0x54,
    ];
    // miniz.c:2150-2153  NOTE: miniz only stores the LOW 16 BITS of w and h
    // (bytes 16-17 / 20-21 stay zero). Preserved faithfully.
    pnghdr[18] = (w >> 8) as u8;
    pnghdr[19] = w as u8;
    pnghdr[22] = (h >> 8) as u8;
    pnghdr[23] = h as u8;
    // miniz.c:2154  pnghdr[25] = chans[num_chans];
    pnghdr[25] = chans[num_chans];
    // miniz.c:2155-2158  big-endian IDAT chunk length
    pnghdr[33] = (len_out >> 24) as u8;
    pnghdr[34] = (len_out >> 16) as u8;
    pnghdr[35] = (len_out >> 8) as u8;
    pnghdr[36] = len_out as u8;
    // miniz.c:2159-2161  IHDR CRC over pnghdr[12..29] ("IHDR" tag + 13 payload
    // bytes), stored big-endian at pnghdr[29..33]
    let c = mz_crc32(MZ_CRC32_INIT, &pnghdr[12..29]);
    pnghdr[29..33].copy_from_slice(&c.to_be_bytes());
    // miniz.c:2162  memcpy(out_buf.m_pBuf, pnghdr, 41);
    let mut out_buf: Vec<u8> = Vec::with_capacity(57 + len_out);
    out_buf.extend_from_slice(&pnghdr);
    out_buf.extend_from_slice(&compressed);

    // miniz.c:2164-2171  write footer (IDAT CRC-32 placeholder, followed by
    // IEND chunk)
    out_buf.extend_from_slice(b"\0\0\0\0\0\0\0\0\x49\x45\x4e\x44\xae\x42\x60\x82");
    // miniz.c:2172-2174  IDAT CRC over out_buf[37..41+len_out] ("IDAT" tag +
    // compressed payload), stored big-endian in the first 4 footer bytes
    let c = mz_crc32(MZ_CRC32_INIT, &out_buf[41 - 4..41 + len_out]);
    let n = out_buf.len();
    out_buf[n - 16..n - 12].copy_from_slice(&c.to_be_bytes());

    // miniz.c:2176-2178  *pLen_out += 57; return out_buf.m_pBuf;
    Some(out_buf)
}

// =============================================================================
// RasterBase.cpp
// =============================================================================

impl PNGRasterEncoder {
    // RasterBase.cpp:14-34
    // EncodedRaster PNGRasterEncoder::operator()(const void *ptr, size_t w, size_t h,
    //                                            size_t      num_components)
    pub fn call(&mut self, ptr: &[u8], w: usize, h: usize, num_components: usize) -> EncodedRaster {
        // RasterBase.cpp:17-18  std::vector<uint8_t> buf; size_t s = 0;
        // RasterBase.cpp:20-21
        // void *rawdata = tdefl_write_image_to_png_file_in_memory(
        //     ptr, int(w), int(h), int(num_components), &s);
        let rawdata = tdefl_write_image_to_png_file_in_memory(ptr, w, h, num_components);

        // RasterBase.cpp:23-25
        // On error, data() will return an empty vector. No other info can be
        // retrieved from miniz anyway...
        // if (rawdata == nullptr) return EncodedRaster({}, "png");
        let Some(buf) = rawdata else {
            return EncodedRaster::new(Vec::new(), "png".to_string());
        };

        // RasterBase.cpp:27-33  (copy into buf + MZ_FREE collapse to a move)
        EncodedRaster::new(buf, "png".to_string())
    }
}

// RasterBase.cpp:36-42
// std::ostream &operator<<(std::ostream &stream, const EncodedRaster &bytes)
// (operator<< has no Rust equivalent; ported as a free function over any
// io::Write sink)
pub fn write_encoded_raster<W: Write>(stream: &mut W, bytes: &EncodedRaster) -> std::io::Result<()> {
    // RasterBase.cpp:38-39
    // stream.write(reinterpret_cast<const char *>(bytes.data()),
    //              std::streamsize(bytes.size()));
    stream.write_all(bytes.data())?;

    // RasterBase.cpp:41  return stream;
    Ok(())
}

impl PPMRasterEncoder {
    // RasterBase.cpp:44-63
    // EncodedRaster PPMRasterEncoder::operator()(const void *ptr, size_t w, size_t h,
    //                                            size_t      num_components)
    pub fn call(&mut self, ptr: &[u8], w: usize, h: usize, num_components: usize) -> EncodedRaster {
        // RasterBase.cpp:47  std::vector<uint8_t> buf;
        let mut buf: Vec<u8> = Vec::new();

        // RasterBase.cpp:49-51
        // auto header = std::string("P5 ") +
        //         std::to_string(w) + " " +
        //         std::to_string(h) + " " + "255 ";
        let header = format!("P5 {} {} 255 ", w, h);

        // RasterBase.cpp:53-54
        let sz = w * h * num_components;
        let s = sz + header.len();

        // RasterBase.cpp:56  buf.reserve(s);
        buf.reserve(s);

        // RasterBase.cpp:58-60
        // auto buff = reinterpret_cast<const std::uint8_t*>(ptr);
        // std::copy(header.begin(), header.end(), std::back_inserter(buf));
        // std::copy(buff, buff+sz, std::back_inserter(buf));
        buf.extend_from_slice(header.as_bytes());
        buf.extend_from_slice(&ptr[..sz]);

        // RasterBase.cpp:62  return EncodedRaster(std::move(buf), "ppm");
        EncodedRaster::new(buf, "ppm".to_string())
    }
}

// RasterBase.cpp:65-81
// std::unique_ptr<RasterBase> create_raster_grayscale_aa(
//     const Resolution &res,
//     const PixelDim &  pxdim,
//     double                        gamma,
//     const RasterBase::Trafo &     tr)
// {
//     std::unique_ptr<RasterBase> rst;
//
//     if (gamma > 0)
//         rst = std::make_unique<RasterGrayscaleAAGammaPower>(res, pxdim, tr, gamma);
//     else if (std::abs(gamma - 1.) < 1e-6)
//         rst = std::make_unique<RasterGrayscaleAA>(res, pxdim, tr, agg::gamma_none());
//     else
//         rst = std::make_unique<RasterGrayscaleAA>(res, pxdim, tr, agg::gamma_threshold(.5));
//
//     return rst;
// }
//
// BLOCKED: not ported. The factory constructs `RasterGrayscaleAAGammaPower` /
// `RasterGrayscaleAA` (SLA/AGGRaster.hpp) which wrap the vendored AGG C++
// rasterizer (agg_rasterizer_scanline_aa, gamma LUTs, scanline renderers).
// `crate::sla::agg_raster` is still an unported placeholder, so a faithful
// implementation is impossible here without porting AGGRaster.hpp + the AGG
// rasterization kernel first. Re-add this function when that lands.
