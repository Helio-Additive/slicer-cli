//! Faithful 1:1 port of `PNGReadWrite.cpp` / `PNGReadWrite.hpp`.
//!
//! C++ Reference:
//! - src/libslic3r/PNGReadWrite.hpp
//! - src/libslic3r/PNGReadWrite.cpp
//!
//! Namespace: `Slic3r::png` -> module `png_read_write`.
//!
//! NATIVE-DEP NOTE: the C++ uses libpng (`<png.h>`), which is a native C
//! library and is NOT wasm-safe. The header-level types and all of the
//! pure-C++ data-marshaling / scaling logic are ported line-by-line below.
//! The libpng `png_read_*` / `png_write_png` calls are replaced by a
//! self-contained, pure-Rust PNG codec (DEFLATE via `flate2`/`miniz_oxide`,
//! filter reconstruction on read, fixed zero-filter + IDAT framing with CRC
//! on write) so that behavior matches libpng for the formats this module
//! supports (8-bit GRAY / RGB / RGBA, non-interlaced). This keeps the crate
//! wasm-safe without pulling in a native backend.

// PNGReadWrite.cpp:1   #include "PNGReadWrite.hpp"
// PNGReadWrite.cpp:8   #include <boost/log/trivial.hpp>

use flate2::read::ZlibDecoder;
use flate2::write::ZlibEncoder;
use flate2::Compression;
use std::io::{Read, Write};

// ---------------------------------------------------------------------------
// Header (PNGReadWrite.hpp) — types, in source order.
// ---------------------------------------------------------------------------

// PNGReadWrite.hpp:14   // Interface for an input stream of encoded png image data.
// PNGReadWrite.hpp:14   struct IStream {
// PNGReadWrite.hpp:15       virtual ~IStream() = default;
// PNGReadWrite.hpp:16       virtual size_t read(std::uint8_t *outp, size_t amount) = 0;
// PNGReadWrite.hpp:17       virtual bool is_ok() const = 0;
// PNGReadWrite.hpp:18   };
/// Interface for an input stream of encoded png image data.
pub trait IStream {
    fn read(&mut self, outp: &mut [u8], amount: usize) -> usize;
    fn is_ok(&self) -> bool;
}

// PNGReadWrite.hpp:20   // The output format of decode_png: a 2D pixel matrix stored continuously row
// PNGReadWrite.hpp:21   // after row (row major layout).
// PNGReadWrite.hpp:22   template<class PxT> struct Image {
// PNGReadWrite.hpp:23       std::vector<PxT> buf;
// PNGReadWrite.hpp:24       size_t rows, cols;
// PNGReadWrite.hpp:25       PxT get(size_t row, size_t col) const { return buf[row * cols + col]; }
// PNGReadWrite.hpp:26   };
/// The output format of decode_png: a 2D pixel matrix stored continuously row
/// after row (row major layout).
#[derive(Debug, Clone, Default)]
pub struct Image<PxT> {
    pub buf: Vec<PxT>,
    pub rows: usize,
    pub cols: usize,
}

impl<PxT: Copy> Image<PxT> {
    // PNGReadWrite.hpp:25   PxT get(size_t row, size_t col) const { return buf[row * cols + col]; }
    pub fn get(&self, row: usize, col: usize) -> PxT {
        self.buf[row * self.cols + col]
    }
}

// PNGReadWrite.hpp:28   using ImageGreyscale = Image<uint8_t>;
pub type ImageGreyscale = Image<u8>;

// PNGReadWrite.hpp:29   struct ImageColorscale:Image<unsigned char>
// PNGReadWrite.hpp:30   {
// PNGReadWrite.hpp:31       int bytes_per_pixel;
// PNGReadWrite.hpp:32   };
#[derive(Debug, Clone, Default)]
pub struct ImageColorscale {
    pub buf: Vec<u8>,
    pub rows: usize,
    pub cols: usize,
    pub bytes_per_pixel: i32,
}

// PNGReadWrite.hpp:49   // Encoded png data buffer: a simple read-only buffer and its size.
// PNGReadWrite.hpp:49   struct ReadBuf { const void *buf = nullptr; const size_t sz = 0; };
/// Encoded png data buffer: a simple read-only buffer and its size.
pub struct ReadBuf<'a> {
    pub buf: &'a [u8],
    pub sz: usize,
}

// PNGReadWrite.hpp:53   struct ReadBufStream: public IStream {
// PNGReadWrite.hpp:54       const ReadBuf &rbuf_ref;
// PNGReadWrite.hpp:55       size_t pos = 0;
// PNGReadWrite.hpp:57       explicit ReadBufStream(const ReadBuf &buf): rbuf_ref{buf} {}
pub struct ReadBufStream<'a, 'b> {
    pub rbuf_ref: &'b ReadBuf<'a>,
    pub pos: usize,
}

impl<'a, 'b> ReadBufStream<'a, 'b> {
    // PNGReadWrite.hpp:57   explicit ReadBufStream(const ReadBuf &buf): rbuf_ref{buf} {}
    pub fn new(buf: &'b ReadBuf<'a>) -> Self {
        ReadBufStream { rbuf_ref: buf, pos: 0 }
    }
}

impl<'a, 'b> IStream for ReadBufStream<'a, 'b> {
    // PNGReadWrite.hpp:59   size_t read(std::uint8_t *outp, size_t amount) override
    // PNGReadWrite.hpp:60   {
    // PNGReadWrite.hpp:61       if (amount > rbuf_ref.sz - pos) return 0;
    // PNGReadWrite.hpp:63       auto buf = static_cast<const std::uint8_t *>(rbuf_ref.buf);
    // PNGReadWrite.hpp:64       std::copy(buf + pos, buf + (pos + amount), outp);
    // PNGReadWrite.hpp:65       pos += amount;
    // PNGReadWrite.hpp:67       return amount;
    // PNGReadWrite.hpp:68   }
    fn read(&mut self, outp: &mut [u8], amount: usize) -> usize {
        if amount > self.rbuf_ref.sz - self.pos {
            return 0;
        }

        let buf = self.rbuf_ref.buf;
        outp[..amount].copy_from_slice(&buf[self.pos..self.pos + amount]);
        self.pos += amount;

        amount
    }

    // PNGReadWrite.hpp:70   bool is_ok() const override { return pos < rbuf_ref.sz; }
    fn is_ok(&self) -> bool {
        self.pos < self.rbuf_ref.sz
    }
}

// ---------------------------------------------------------------------------
// Pure-Rust libpng replacement: minimal PNG codec primitives.
//
// These provide the equivalent of libpng's read/write path for the formats
// this module supports (8-bit GRAY / RGB / RGBA, non-interlaced). They are NOT
// part of the C++ source; they replace the native `<png.h>` calls so the
// surrounding C++ logic can be ported 1:1 while staying wasm-safe.
// ---------------------------------------------------------------------------

// PNG color type constants (mirror libpng's <png.h>).
const PNG_COLOR_TYPE_GRAY: i32 = 0;
const PNG_COLOR_TYPE_RGB: i32 = 2;
const PNG_COLOR_TYPE_RGB_ALPHA: i32 = 6;
// libpng exposes both spellings; PNG_COLOR_TYPE_RGBA == PNG_COLOR_TYPE_RGB_ALPHA.
const PNG_COLOR_TYPE_RGBA: i32 = PNG_COLOR_TYPE_RGB_ALPHA;

// CRC-32 (ISO 3309) used for PNG chunk integrity, matching libpng output.
fn png_crc32(prev: u32, data: &[u8]) -> u32 {
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

/// Decoded PNG header + raw (unfiltered, reconstructed) pixel rows.
struct DecodedPng {
    width: usize,
    height: usize,
    color_type: i32,
    bit_depth: i32,
    rowbytes: usize,
    pixels: Vec<u8>,
}

fn png_channels(color_type: i32) -> usize {
    match color_type {
        PNG_COLOR_TYPE_GRAY => 1,
        PNG_COLOR_TYPE_RGB => 3,
        PNG_COLOR_TYPE_RGB_ALPHA => 4,
        _ => 0,
    }
}

fn paeth_predictor(a: i32, b: i32, c: i32) -> i32 {
    let p = a + b - c;
    let pa = (p - a).abs();
    let pb = (p - b).abs();
    let pc = (p - c).abs();
    if pa <= pb && pa <= pc {
        a
    } else if pb <= pc {
        b
    } else {
        c
    }
}

/// Read the remaining encoded PNG stream (the bytes AFTER the 8-byte signature)
/// from `in_buf`, parse chunks, inflate IDAT, and reconstruct filtered rows.
/// Mirrors what libpng does in `png_read_info` + `png_read_row` for the
/// non-interlaced 8-bit formats supported here. Returns None on any failure,
/// matching libpng's longjmp-on-error behavior used by these callers.
fn read_png_after_signature(in_buf: &mut dyn IStream) -> Option<DecodedPng> {
    // Drain the rest of the encoded stream into memory. libpng pulls bytes via
    // the read callback on demand; here we pull all remaining bytes.
    let mut data: Vec<u8> = Vec::new();
    let mut tmp = [0u8; 8192];
    let cap = tmp.len();
    loop {
        let got = in_buf.read(&mut tmp, cap);
        if got == 0 {
            break;
        }
        data.extend_from_slice(&tmp[..got]);
    }

    let mut pos = 0usize;
    let mut width = 0usize;
    let mut height = 0usize;
    let mut bit_depth = 0i32;
    let mut color_type = 0i32;
    let mut interlace = 0i32;
    let mut idat: Vec<u8> = Vec::new();
    let mut seen_ihdr = false;

    while pos + 8 <= data.len() {
        let len = u32::from_be_bytes([
            data[pos],
            data[pos + 1],
            data[pos + 2],
            data[pos + 3],
        ]) as usize;
        let ctype = &data[pos + 4..pos + 8];
        let body_start = pos + 8;
        if body_start + len + 4 > data.len() {
            break;
        }
        let body = &data[body_start..body_start + len];

        match ctype {
            b"IHDR" => {
                if len < 13 {
                    return None;
                }
                width = u32::from_be_bytes([body[0], body[1], body[2], body[3]]) as usize;
                height = u32::from_be_bytes([body[4], body[5], body[6], body[7]]) as usize;
                bit_depth = body[8] as i32;
                color_type = body[9] as i32;
                interlace = body[12] as i32;
                seen_ihdr = true;
            }
            b"IDAT" => {
                idat.extend_from_slice(body);
            }
            b"IEND" => {
                break;
            }
            _ => {}
        }

        pos = body_start + len + 4; // skip body + CRC
    }

    if !seen_ihdr || interlace != 0 {
        return None;
    }

    let channels = png_channels(color_type);
    if channels == 0 || bit_depth != 8 {
        // Header parsed; let caller apply its own color/bit-depth checks too,
        // but unsupported pixel layouts cannot be reconstructed here.
        // Still return header so callers performing their own validation work.
        return Some(DecodedPng {
            width,
            height,
            color_type,
            bit_depth,
            rowbytes: width.saturating_mul(channels.max(1)),
            pixels: Vec::new(),
        });
    }

    // Inflate IDAT (zlib stream).
    let mut raw = Vec::new();
    if ZlibDecoder::new(idat.as_slice())
        .read_to_end(&mut raw)
        .is_err()
    {
        return None;
    }

    let bpp = channels; // 8-bit -> bytes per pixel == channels
    let rowbytes = width * bpp;
    let stride = rowbytes + 1; // each row prefixed by 1 filter byte
    if raw.len() < stride * height {
        return None;
    }

    let mut pixels = vec![0u8; rowbytes * height];
    for r in 0..height {
        let filter = raw[r * stride];
        let src = &raw[r * stride + 1..r * stride + 1 + rowbytes];
        let (prev_split, cur_split) = pixels.split_at_mut(r * rowbytes);
        let cur = &mut cur_split[..rowbytes];
        let prev: &[u8] = if r == 0 {
            &[]
        } else {
            &prev_split[(r - 1) * rowbytes..r * rowbytes]
        };
        for i in 0..rowbytes {
            let x = src[i] as i32;
            let a = if i >= bpp { cur[i - bpp] as i32 } else { 0 };
            let b = if !prev.is_empty() { prev[i] as i32 } else { 0 };
            let c = if !prev.is_empty() && i >= bpp {
                prev[i - bpp] as i32
            } else {
                0
            };
            let val = match filter {
                0 => x,
                1 => x + a,
                2 => x + b,
                3 => x + (a + b) / 2,
                4 => x + paeth_predictor(a, b, c),
                _ => return None,
            };
            cur[i] = (val & 0xff) as u8;
        }
    }

    Some(DecodedPng {
        width,
        height,
        color_type,
        bit_depth,
        rowbytes,
        pixels,
    })
}

/// Encode an 8-bit GRAY/RGB/RGBA image to a PNG byte stream using filter 0
/// (None) on every row, matching libpng's `png_write_png(...,
/// PNG_TRANSFORM_IDENTITY, ...)` data layout. `data` is `height * line_width`
/// bytes, top row first.
pub(crate) fn encode_png(width: usize, height: usize, color_type: i32, data: &[u8]) -> Option<Vec<u8>> {
    let channels = png_channels(color_type);
    if channels == 0 {
        return None;
    }
    let line_width = width * channels;
    if data.len() < line_width * height {
        return None;
    }

    let mut out: Vec<u8> = Vec::new();
    // PNG signature.
    out.extend_from_slice(&[137, 80, 78, 71, 13, 10, 26, 10]);

    let write_chunk = |out: &mut Vec<u8>, ctype: &[u8; 4], body: &[u8]| {
        out.extend_from_slice(&(body.len() as u32).to_be_bytes());
        out.extend_from_slice(ctype);
        out.extend_from_slice(body);
        let mut crc = png_crc32(0, ctype);
        crc = png_crc32(crc, body);
        out.extend_from_slice(&crc.to_be_bytes());
    };

    // IHDR
    let mut ihdr = Vec::with_capacity(13);
    ihdr.extend_from_slice(&(width as u32).to_be_bytes());
    ihdr.extend_from_slice(&(height as u32).to_be_bytes());
    ihdr.push(8); // bit depth
    ihdr.push(color_type as u8);
    ihdr.push(0); // compression
    ihdr.push(0); // filter
    ihdr.push(0); // interlace
    write_chunk(&mut out, b"IHDR", &ihdr);

    // IDAT: filter 0 per row, then zlib-deflate.
    let mut raw = Vec::with_capacity((line_width + 1) * height);
    for y in 0..height {
        raw.push(0u8); // filter type None
        raw.extend_from_slice(&data[y * line_width..y * line_width + line_width]);
    }
    let mut encoder = ZlibEncoder::new(Vec::new(), Compression::default());
    if encoder.write_all(&raw).is_err() {
        return None;
    }
    let compressed = match encoder.finish() {
        Ok(c) => c,
        Err(_) => return None,
    };
    write_chunk(&mut out, b"IDAT", &compressed);

    // IEND
    write_chunk(&mut out, b"IEND", &[]);

    Some(out)
}

// ---------------------------------------------------------------------------
// Implementation (PNGReadWrite.cpp).
// ---------------------------------------------------------------------------

// PNGReadWrite.cpp:13   struct PNGDescr {
// PNGReadWrite.cpp:14       png_struct *png = nullptr; png_info *info = nullptr;
//
// PNGDescr is a RAII wrapper around libpng's read structs. The native handles
// have no analog in the pure-Rust codec, so this is intentionally inert; the
// surrounding control flow that constructs/destroys it is preserved as
// comments at each use site.
struct PNGDescr;

impl PNGDescr {
    // PNGReadWrite.cpp:16   PNGDescr() = default;
    fn new() -> Self {
        PNGDescr
    }
}

// PNGReadWrite.cpp:29   bool is_png(const ReadBuf &rb)
pub fn is_png(rb: &ReadBuf) -> bool {
    // PNGReadWrite.cpp:31   static const constexpr int PNG_SIG_BYTES = 8;
    const PNG_SIG_BYTES: usize = 8;

    // PNGReadWrite.cpp:33-42  (libpng version-dependent signature buffer setup)
    //   auto buf = static_cast<png_const_bytep>(rb.buf);
    // png_sig_cmp(buf, 0, PNG_SIG_BYTES) returns 0 iff the first PNG_SIG_BYTES
    // bytes match the 8-byte PNG signature; we replicate that test directly.
    static PNG_SIGNATURE: [u8; PNG_SIG_BYTES] = [137, 80, 78, 71, 13, 10, 26, 10];

    // PNGReadWrite.cpp:44   return rb.sz >= PNG_SIG_BYTES && !png_sig_cmp(buf, 0, PNG_SIG_BYTES);
    rb.sz >= PNG_SIG_BYTES && rb.buf[..PNG_SIG_BYTES] == PNG_SIGNATURE
}

// PNGReadWrite.cpp:47   // Buffer read callback for libpng. It provides an allocated output buffer and
// PNGReadWrite.cpp:48   // the amount of data it desires to read from the input.
// PNGReadWrite.cpp:49   static void png_read_callback(...)
//
// In the pure-Rust codec, reading happens in `read_png_after_signature`, which
// pulls bytes directly from the `IStream`; this preserves the original
// behavior of routing all reads through the stream's `read`/`is_ok`.

/// Helper that, given a freshly read 8-byte signature, validates it like
/// `png_check_sig`.
fn png_check_sig(sig: &[u8], num_to_check: usize) -> bool {
    static PNG_SIGNATURE: [u8; 8] = [137, 80, 78, 71, 13, 10, 26, 10];
    num_to_check <= 8 && sig[..num_to_check] == PNG_SIGNATURE[..num_to_check]
}

// PNGReadWrite.cpp:61   bool decode_png(IStream &in_buf, ImageGreyscale &out_img)
pub fn decode_png(in_buf: &mut dyn IStream, out_img: &mut ImageGreyscale) -> bool {
    // PNGReadWrite.cpp:63   static const constexpr int PNG_SIG_BYTES = 8;
    const PNG_SIG_BYTES: usize = 8;

    // PNGReadWrite.cpp:65   std::vector<png_byte> sig(PNG_SIG_BYTES, 0);
    let mut sig = vec![0u8; PNG_SIG_BYTES];
    // PNGReadWrite.cpp:66   in_buf.read(sig.data(), PNG_SIG_BYTES);
    in_buf.read(&mut sig, PNG_SIG_BYTES);
    // PNGReadWrite.cpp:67   if (!png_check_sig(sig.data(), PNG_SIG_BYTES))
    // PNGReadWrite.cpp:68       return false;
    if !png_check_sig(&sig, PNG_SIG_BYTES) {
        return false;
    }

    // PNGReadWrite.cpp:70   PNGDescr dsc;
    let _dsc = PNGDescr::new();
    // PNGReadWrite.cpp:71-77   dsc.png = png_create_read_struct(...); if(!dsc.png) return false;
    //                          dsc.info = png_create_info_struct(...); if(!dsc.info) return false;
    // PNGReadWrite.cpp:79   png_set_read_fn(dsc.png, &in_buf, png_read_callback);
    // PNGReadWrite.cpp:82   png_set_sig_bytes(dsc.png, PNG_SIG_BYTES);
    // PNGReadWrite.cpp:84   png_read_info(dsc.png, dsc.info);
    let dec = match read_png_after_signature(in_buf) {
        Some(d) => d,
        None => return false,
    };

    // PNGReadWrite.cpp:86   out_img.cols = png_get_image_width(dsc.png, dsc.info);
    out_img.cols = dec.width;
    // PNGReadWrite.cpp:87   out_img.rows = png_get_image_height(dsc.png, dsc.info);
    out_img.rows = dec.height;
    // PNGReadWrite.cpp:88   size_t color_type = png_get_color_type(dsc.png, dsc.info);
    let color_type = dec.color_type;
    // PNGReadWrite.cpp:89   size_t bit_depth  = png_get_bit_depth(dsc.png, dsc.info);
    let bit_depth = dec.bit_depth;

    // PNGReadWrite.cpp:91   if (color_type != PNG_COLOR_TYPE_GRAY || bit_depth != 8)
    // PNGReadWrite.cpp:92       return false;
    if color_type != PNG_COLOR_TYPE_GRAY || bit_depth != 8 {
        return false;
    }

    // PNGReadWrite.cpp:94   out_img.buf.resize(out_img.rows * out_img.cols);
    out_img.buf.resize(out_img.rows * out_img.cols, 0);

    // PNGReadWrite.cpp:96   auto readbuf = static_cast<png_bytep>(out_img.buf.data());
    // PNGReadWrite.cpp:97   for (size_t r = 0; r < out_img.rows; ++r)
    // PNGReadWrite.cpp:98       png_read_row(dsc.png, readbuf + r * out_img.cols, nullptr);
    for r in 0..out_img.rows {
        let dst = r * out_img.cols;
        let src = r * dec.rowbytes;
        out_img.buf[dst..dst + out_img.cols]
            .copy_from_slice(&dec.pixels[src..src + out_img.cols]);
    }

    // PNGReadWrite.cpp:100  return true;
    true
}

// PNGReadWrite.cpp:103  bool decode_colored_png(IStream &in_buf, ImageColorscale &out_img)
pub fn decode_colored_png_stream(in_buf: &mut dyn IStream, out_img: &mut ImageColorscale) -> bool {
    // PNGReadWrite.cpp:105  static const constexpr int PNG_SIG_BYTES = 8;
    const PNG_SIG_BYTES: usize = 8;

    // PNGReadWrite.cpp:107  std::vector<png_byte> sig(PNG_SIG_BYTES, 0);
    let mut sig = vec![0u8; PNG_SIG_BYTES];
    // PNGReadWrite.cpp:108  in_buf.read(sig.data(), PNG_SIG_BYTES);
    in_buf.read(&mut sig, PNG_SIG_BYTES);
    // PNGReadWrite.cpp:109  if (!png_check_sig(sig.data(), PNG_SIG_BYTES)) {
    if !png_check_sig(&sig, PNG_SIG_BYTES) {
        // PNGReadWrite.cpp:110  BOOST_LOG_TRIVIAL(error) << "decode_colored_png: png_check_sig failed";
        log::error!("decode_colored_png: png_check_sig failed");
        // PNGReadWrite.cpp:111  return false;
        return false;
    }

    // PNGReadWrite.cpp:114  PNGDescr dsc;
    let _dsc = PNGDescr::new();
    // PNGReadWrite.cpp:115-121  dsc.png = png_create_read_struct(...); if(!dsc.png) { ...; return false; }
    // PNGReadWrite.cpp:123-128  dsc.info = png_create_info_struct(...); if(!dsc.info) { ...; return false; }
    // PNGReadWrite.cpp:130  png_set_read_fn(dsc.png, &in_buf, png_read_callback);
    // PNGReadWrite.cpp:133  png_set_sig_bytes(dsc.png, PNG_SIG_BYTES);
    // PNGReadWrite.cpp:135  png_read_info(dsc.png, dsc.info);
    let dec = match read_png_after_signature(in_buf) {
        Some(d) => d,
        None => {
            log::error!("decode_colored_png: png_create_read_struct failed");
            return false;
        }
    };

    // PNGReadWrite.cpp:137  out_img.cols = png_get_image_width(dsc.png, dsc.info);
    out_img.cols = dec.width;
    // PNGReadWrite.cpp:138  out_img.rows = png_get_image_height(dsc.png, dsc.info);
    out_img.rows = dec.height;
    // PNGReadWrite.cpp:139  size_t color_type = png_get_color_type(dsc.png, dsc.info);
    let color_type = dec.color_type;
    // PNGReadWrite.cpp:140  size_t bit_depth  = png_get_bit_depth(dsc.png, dsc.info);
    let bit_depth = dec.bit_depth;
    // PNGReadWrite.cpp:141  unsigned long rowbytes = png_get_rowbytes(dsc.png, dsc.info);
    let rowbytes = dec.rowbytes;

    // PNGReadWrite.cpp:143  switch(color_type)
    match color_type {
        // PNGReadWrite.cpp:145      case PNG_COLOR_TYPE_RGB:
        // PNGReadWrite.cpp:146          out_img.bytes_per_pixel = 3;
        // PNGReadWrite.cpp:147          break;
        PNG_COLOR_TYPE_RGB => {
            out_img.bytes_per_pixel = 3;
        }
        // PNGReadWrite.cpp:148      case PNG_COLOR_TYPE_RGB_ALPHA:
        // PNGReadWrite.cpp:149          out_img.bytes_per_pixel = 4;
        // PNGReadWrite.cpp:150          break;
        PNG_COLOR_TYPE_RGB_ALPHA => {
            out_img.bytes_per_pixel = 4;
        }
        // PNGReadWrite.cpp:151      default: //not supported currently
        // PNGReadWrite.cpp:152          png_destroy_read_struct(&dsc.png, &dsc.info, NULL);
        // PNGReadWrite.cpp:153          return false;
        _ => {
            return false;
        }
    }

    // PNGReadWrite.cpp:156  BOOST_LOG_TRIVIAL(info) << ... png's cols ... rows ... color_type ... bit_depth ... bytes_per_pixel ... rowbytes ...
    log::info!(
        "png's cols {}, rows {}, color_type {}, bit_depth {}, bytes_per_pixel {}, rowbytes {}",
        out_img.cols,
        out_img.rows,
        color_type,
        bit_depth,
        out_img.bytes_per_pixel,
        rowbytes
    );
    // PNGReadWrite.cpp:157  out_img.buf.resize(out_img.rows * rowbytes);
    out_img.buf.resize(out_img.rows * rowbytes, 0);

    // PNGReadWrite.cpp:159  int filter_type = png_get_filter_type(dsc.png, dsc.info);
    // PNGReadWrite.cpp:160  int compression_type = png_get_compression_type(dsc.png, dsc.info);
    // PNGReadWrite.cpp:161  int interlace_type = png_get_interlace_type(dsc.png, dsc.info);
    // The codec only handles the standard PNG defaults here.
    let filter_type = 0;
    let compression_type = 0;
    let interlace_type = 0;
    // PNGReadWrite.cpp:162  BOOST_LOG_TRIVIAL(info) << ... filter_type ... compression_type ... interlace_type ... rowbytes ...
    log::info!(
        "filter_type {}, compression_type {}, interlace_type {}, rowbytes {}",
        filter_type,
        compression_type,
        interlace_type,
        rowbytes
    );

    // PNGReadWrite.cpp:164  auto readbuf = static_cast<png_bytep>(out_img.buf.data());
    // PNGReadWrite.cpp:165  for (size_t r = out_img.rows; r > 0; r--)
    // PNGReadWrite.cpp:167      png_read_row(dsc.png, readbuf + (r - 1) * rowbytes, nullptr);
    // NOTE: this vertically flips the image (reads bottom row first).
    let mut r = out_img.rows;
    let mut src_row = 0usize;
    while r > 0 {
        let dst = (r - 1) * rowbytes;
        let src = src_row * rowbytes;
        out_img.buf[dst..dst + rowbytes].copy_from_slice(&dec.pixels[src..src + rowbytes]);
        src_row += 1;
        r -= 1;
    }

    // PNGReadWrite.cpp:170  png_read_end(dsc.png, dsc.info);
    // PNGReadWrite.cpp:171  png_destroy_read_struct(&dsc.png, &dsc.info, NULL);

    // PNGReadWrite.cpp:173  return true;
    true
}

// PNGReadWrite.cpp:176  bool decode_colored_png(const ReadBuf &in_buf, ImageColorscale &out_img)
pub fn decode_colored_png(in_buf: &ReadBuf, out_img: &mut ImageColorscale) -> bool {
    // PNGReadWrite.cpp:178  struct ReadBufStream stream{in_buf};
    let mut stream = ReadBufStream::new(in_buf);

    // PNGReadWrite.cpp:180  return decode_colored_png(stream, out_img);
    decode_colored_png_stream(&mut stream, out_img)
}

// PNGReadWrite.hpp:73-78  template<class Img> bool decode_png(const ReadBuf &in_buf, Img &out_img)
// (header-only template) — grayscale specialization for the ReadBuf overload.
pub fn decode_png_buf(in_buf: &ReadBuf, out_img: &mut ImageGreyscale) -> bool {
    // PNGReadWrite.hpp:75   struct ReadBufStream stream{in_buf};
    let mut stream = ReadBufStream::new(in_buf);
    // PNGReadWrite.hpp:77   return decode_png(stream, out_img);
    decode_png(&mut stream, out_img)
}

// PNGReadWrite.cpp:184  // Down to earth function to store a packed RGB image to file. ...
// PNGReadWrite.cpp:185  // Based on https://www.lemoda.net/c/write-png/
// PNGReadWrite.cpp:186  // png_color_type is PNG_COLOR_TYPE_RGB or PNG_COLOR_TYPE_GRAY
// PNGReadWrite.cpp:187  //FIXME maybe better to use tdefl_write_image_to_png_file_in_memory() instead?
// PNGReadWrite.cpp:188  static bool write_rgb_or_gray_to_file(const char *file_name_utf8, size_t width, size_t height, int png_color_type, const uint8_t *data, bool flip = false)
fn write_rgb_or_gray_to_file(
    file_name_utf8: &str,
    width: usize,
    height: usize,
    png_color_type: i32,
    data: &[u8],
    flip: bool,
) -> bool {
    // PNGReadWrite.cpp:190  bool result = false;
    let mut result = false;

    // PNGReadWrite.cpp:193-195  png_ptr / info_ptr / row_pointers forward declarations (libpng handles).

    // PNGReadWrite.cpp:197  FILE *fp = boost::nowide::fopen(file_name_utf8, "wb");
    // PNGReadWrite.cpp:198  if (! fp) { BOOST_LOG_TRIVIAL(error) << ...; goto fopen_failed; }
    let mut fp = match std::fs::File::create(file_name_utf8) {
        Ok(f) => f,
        Err(_) => {
            log::error!(
                "write_png_file: File could not be opened for writing: {}",
                file_name_utf8
            );
            // PNGReadWrite.cpp:263  fopen_failed:
            // PNGReadWrite.cpp:264  return result;
            return result;
        }
    };

    // PNGReadWrite.cpp:203-219  png_create_write_struct / png_create_info_struct / setjmp error handling.

    // PNGReadWrite.cpp:222-230  png_set_IHDR(..., 8 /*depth*/, png_color_type, PNG_INTERLACE_NONE,
    //                           PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

    // PNGReadWrite.cpp:233  row_pointers = ::png_malloc(png_ptr, height * sizeof(png_byte*));
    // PNGReadWrite.cpp:234-245  build per-row buffers, copying (with optional vertical flip):
    // PNGReadWrite.cpp:235      int line_width = width;
    let mut line_width: usize = width;
    // PNGReadWrite.cpp:236      if (png_color_type == PNG_COLOR_TYPE_RGB)
    // PNGReadWrite.cpp:237          line_width *= 3;
    if png_color_type == PNG_COLOR_TYPE_RGB {
        line_width *= 3;
    // PNGReadWrite.cpp:238      else if (png_color_type == PNG_COLOR_TYPE_RGBA)
    // PNGReadWrite.cpp:239          line_width *= 4;
    } else if png_color_type == PNG_COLOR_TYPE_RGBA {
        line_width *= 4;
    }
    // PNGReadWrite.cpp:240-244  for (y) { row = png_malloc(line_width); row_pointers[y] = row;
    //                            memcpy(row, data + line_width * (flip ? (height-1-y) : y), line_width); }
    let mut image_top_first: Vec<u8> = Vec::with_capacity(line_width * height);
    for y in 0..height {
        let src = line_width * if flip { height - 1 - y } else { y };
        image_top_first.extend_from_slice(&data[src..src + line_width]);
    }

    // PNGReadWrite.cpp:247-250  png_init_io(png_ptr, fp); png_set_rows(...);
    //                           png_write_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, nullptr);
    // PNGReadWrite.cpp:252-254  free row_pointers (no-op in Rust; buffers owned by Vec).
    if let Some(bytes) = encode_png(width, height, png_color_type, &image_top_first) {
        if fp.write_all(&bytes).is_ok() {
            // PNGReadWrite.cpp:256  result = true;
            result = true;
        }
    }

    // PNGReadWrite.cpp:258-260  png_failure / png_create_info_struct_failed:
    //                           ::png_destroy_write_struct(&png_ptr, &info_ptr);
    // PNGReadWrite.cpp:261-262  png_create_write_struct_failed: ::fclose(fp);
    drop(fp);
    // PNGReadWrite.cpp:264  return result;
    result
}

// PNGReadWrite.cpp:267  bool write_gl_rgba_to_file(const char* file_name_utf8, size_t width, size_t height, const uint8_t* data_rgb)
pub fn write_gl_rgba_to_file(
    file_name_utf8: &str,
    width: usize,
    height: usize,
    data_rgb: &[u8],
) -> bool {
    // PNGReadWrite.cpp:269  return write_rgb_or_gray_to_file(file_name_utf8, width, height, PNG_COLOR_TYPE_RGBA, data_rgb, true);
    write_rgb_or_gray_to_file(file_name_utf8, width, height, PNG_COLOR_TYPE_RGBA, data_rgb, true)
}

// PNGReadWrite.cpp:272  bool write_rgb_to_file(const char *file_name_utf8, size_t width, size_t height, const uint8_t *data_rgb)
pub fn write_rgb_to_file(file_name_utf8: &str, width: usize, height: usize, data_rgb: &[u8]) -> bool {
    // PNGReadWrite.cpp:274  return write_rgb_or_gray_to_file(file_name_utf8, width, height, PNG_COLOR_TYPE_RGB, data_rgb);
    write_rgb_or_gray_to_file(file_name_utf8, width, height, PNG_COLOR_TYPE_RGB, data_rgb, false)
}

// PNGReadWrite.cpp:277  bool write_rgb_to_file(const std::string &file_name_utf8, size_t width, size_t height, const uint8_t *data_rgb)
// PNGReadWrite.cpp:282  bool write_rgb_to_file(const std::string &file_name_utf8, size_t width, size_t height, const std::vector<uint8_t> &data_rgb)
// (string + vector overloads collapse to the &str / &[u8] form above in Rust)
//
// PNGReadWrite.cpp:284  assert(width * height * 3 == data_rgb.size());
pub fn write_rgb_to_file_vec(
    file_name_utf8: &str,
    width: usize,
    height: usize,
    data_rgb: &[u8],
) -> bool {
    debug_assert!(width * height * 3 == data_rgb.len());
    write_rgb_to_file(file_name_utf8, width, height, data_rgb)
}

// PNGReadWrite.cpp:288  bool write_gray_to_file(const char *file_name_utf8, size_t width, size_t height, const uint8_t *data_gray)
pub fn write_gray_to_file(
    file_name_utf8: &str,
    width: usize,
    height: usize,
    data_gray: &[u8],
) -> bool {
    // PNGReadWrite.cpp:290  return write_rgb_or_gray_to_file(file_name_utf8, width, height, PNG_COLOR_TYPE_GRAY, data_gray);
    write_rgb_or_gray_to_file(file_name_utf8, width, height, PNG_COLOR_TYPE_GRAY, data_gray, false)
}

// PNGReadWrite.cpp:293  bool write_gray_to_file(const std::string &..., const uint8_t *data_gray)
// PNGReadWrite.cpp:298  bool write_gray_to_file(const std::string &..., const std::vector<uint8_t> &data_gray)
// PNGReadWrite.cpp:300  assert(width * height == data_gray.size());
pub fn write_gray_to_file_vec(
    file_name_utf8: &str,
    width: usize,
    height: usize,
    data_gray: &[u8],
) -> bool {
    debug_assert!(width * height == data_gray.len());
    write_gray_to_file(file_name_utf8, width, height, data_gray)
}

// PNGReadWrite.cpp:304  // Scaled variants are mostly useful for debugging purposes, ...
// PNGReadWrite.cpp:305  // Scaling is done by multiplying rows and columns without any smoothing ...
// PNGReadWrite.cpp:306  // png_color_type is PNG_COLOR_TYPE_RGB or PNG_COLOR_TYPE_GRAY
// PNGReadWrite.cpp:307  static bool write_rgb_or_gray_to_file_scaled(const char *file_name_utf8, size_t width, size_t height, int png_color_type, const uint8_t *data, size_t scale)
fn write_rgb_or_gray_to_file_scaled(
    file_name_utf8: &str,
    width: usize,
    height: usize,
    png_color_type: i32,
    data: &[u8],
    scale: usize,
) -> bool {
    // PNGReadWrite.cpp:309  if (scale <= 1)
    if scale <= 1 {
        // PNGReadWrite.cpp:310  return write_rgb_or_gray_to_file(file_name_utf8, width, height, png_color_type, data);
        write_rgb_or_gray_to_file(file_name_utf8, width, height, png_color_type, data, false)
    } else {
        // PNGReadWrite.cpp:312  size_t pixel_bytes = png_color_type == PNG_COLOR_TYPE_RGB ? 3 : 1;
        let pixel_bytes: usize = if png_color_type == PNG_COLOR_TYPE_RGB { 3 } else { 1 };
        // PNGReadWrite.cpp:313  size_t line_width  = width * pixel_bytes;
        let line_width: usize = width * pixel_bytes;
        // PNGReadWrite.cpp:314  std::vector<uint8_t> scaled(line_width * height * scale * scale);
        let mut scaled: Vec<u8> = vec![0u8; line_width * height * scale * scale];
        // PNGReadWrite.cpp:315  uint8_t *dst = scaled.data();
        let mut dst: usize = 0;
        // PNGReadWrite.cpp:316  for (size_t r = 0; r < height; ++ r) {
        for r in 0..height {
            // PNGReadWrite.cpp:317  for (size_t repr = 0; repr < scale; ++ repr) {
            for _repr in 0..scale {
                // PNGReadWrite.cpp:318  const uint8_t *row = data + line_width * r;
                let mut row: usize = line_width * r;
                // PNGReadWrite.cpp:319  for (size_t c = 0; c < width; ++ c) {
                for _c in 0..width {
                    // PNGReadWrite.cpp:320  for (size_t repc = 0; repc < scale; ++ repc)
                    for _repc in 0..scale {
                        // PNGReadWrite.cpp:321  for (size_t b = 0; b < pixel_bytes; ++ b)
                        for b in 0..pixel_bytes {
                            // PNGReadWrite.cpp:322  *dst ++ = row[b];
                            scaled[dst] = data[row + b];
                            dst += 1;
                        }
                    }
                    // PNGReadWrite.cpp:323  row += pixel_bytes;
                    row += pixel_bytes;
                }
            }
        }
        // PNGReadWrite.cpp:327  return write_rgb_or_gray_to_file(file_name_utf8, width * scale, height * scale, png_color_type, scaled.data());
        write_rgb_or_gray_to_file(file_name_utf8, width * scale, height * scale, png_color_type, &scaled, false)
    }
}

// PNGReadWrite.cpp:331  bool write_rgb_to_file_scaled(const char *file_name_utf8, size_t width, size_t height, const uint8_t *data_rgb, size_t scale)
pub fn write_rgb_to_file_scaled(
    file_name_utf8: &str,
    width: usize,
    height: usize,
    data_rgb: &[u8],
    scale: usize,
) -> bool {
    // PNGReadWrite.cpp:333  return write_rgb_or_gray_to_file_scaled(file_name_utf8, width, height, PNG_COLOR_TYPE_RGB, data_rgb, scale);
    write_rgb_or_gray_to_file_scaled(file_name_utf8, width, height, PNG_COLOR_TYPE_RGB, data_rgb, scale)
}

// PNGReadWrite.cpp:336  bool write_rgb_to_file_scaled(const std::string &..., const uint8_t *data_rgb, size_t scale)
// PNGReadWrite.cpp:341  bool write_rgb_to_file_scaled(const std::string &..., const std::vector<uint8_t> &data_rgb, size_t scale)
// PNGReadWrite.cpp:343  assert(width * height * 3 == data_rgb.size());
pub fn write_rgb_to_file_scaled_vec(
    file_name_utf8: &str,
    width: usize,
    height: usize,
    data_rgb: &[u8],
    scale: usize,
) -> bool {
    debug_assert!(width * height * 3 == data_rgb.len());
    write_rgb_to_file_scaled(file_name_utf8, width, height, data_rgb, scale)
}

// PNGReadWrite.cpp:347  bool write_gray_to_file_scaled(const char *file_name_utf8, size_t width, size_t height, const uint8_t *data_gray, size_t scale)
pub fn write_gray_to_file_scaled(
    file_name_utf8: &str,
    width: usize,
    height: usize,
    data_gray: &[u8],
    scale: usize,
) -> bool {
    // PNGReadWrite.cpp:349  return write_rgb_or_gray_to_file_scaled(file_name_utf8, width, height, PNG_COLOR_TYPE_GRAY, data_gray, scale);
    write_rgb_or_gray_to_file_scaled(file_name_utf8, width, height, PNG_COLOR_TYPE_GRAY, data_gray, scale)
}

// PNGReadWrite.cpp:352  bool write_gray_to_file_scaled(const std::string &..., const uint8_t *data_gray, size_t scale)
// PNGReadWrite.cpp:357  bool write_gray_to_file_scaled(const std::string &..., const std::vector<uint8_t> &data_gray, size_t scale)
// PNGReadWrite.cpp:359  assert(width * height == data_gray.size());
pub fn write_gray_to_file_scaled_vec(
    file_name_utf8: &str,
    width: usize,
    height: usize,
    data_gray: &[u8],
    scale: usize,
) -> bool {
    debug_assert!(width * height == data_gray.len());
    write_gray_to_file_scaled(file_name_utf8, width, height, data_gray, scale)
}

// PNGReadWrite.cpp:363  }} // namespace Slic3r::png

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_rgb() {
        // 2x2 RGB image.
        let w = 2usize;
        let h = 2usize;
        let data: Vec<u8> = vec![
            10, 20, 30, 40, 50, 60, // row 0
            70, 80, 90, 100, 110, 120, // row 1
        ];
        let path = std::env::temp_dir().join("png_rw_rgb_test.png");
        let p = path.to_str().unwrap();
        assert!(write_rgb_to_file(p, w, h, &data));

        let bytes = std::fs::read(p).unwrap();
        let rb = ReadBuf {
            buf: &bytes,
            sz: bytes.len(),
        };
        assert!(is_png(&rb));

        let mut img = ImageColorscale::default();
        assert!(decode_colored_png(&rb, &mut img));
        assert_eq!(img.cols, w);
        assert_eq!(img.rows, h);
        assert_eq!(img.bytes_per_pixel, 3);
        // decode_colored_png flips vertically; bottom-first read => row order reversed.
        // Reconstructed buffer[(rows-1-r)] == original row r.
        assert_eq!(&img.buf[0..6], &data[6..12]);
        assert_eq!(&img.buf[6..12], &data[0..6]);
        let _ = std::fs::remove_file(p);
    }

    #[test]
    fn roundtrip_gray() {
        let w = 3usize;
        let h = 2usize;
        let data: Vec<u8> = vec![1, 2, 3, 4, 5, 6];
        let path = std::env::temp_dir().join("png_rw_gray_test.png");
        let p = path.to_str().unwrap();
        assert!(write_gray_to_file(p, w, h, &data));

        let bytes = std::fs::read(p).unwrap();
        let rb = ReadBuf {
            buf: &bytes,
            sz: bytes.len(),
        };
        let mut img = ImageGreyscale::default();
        assert!(decode_png_buf(&rb, &mut img));
        assert_eq!(img.cols, w);
        assert_eq!(img.rows, h);
        assert_eq!(img.buf, data);
        let _ = std::fs::remove_file(p);
    }

    #[test]
    fn not_png() {
        let bytes = [0u8, 1, 2, 3, 4, 5, 6, 7];
        let rb = ReadBuf {
            buf: &bytes,
            sz: bytes.len(),
        };
        assert!(!is_png(&rb));
    }
}
