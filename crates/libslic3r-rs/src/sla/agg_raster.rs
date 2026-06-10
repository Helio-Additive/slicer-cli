//! Faithful partial port of BambuStudio `src/libslic3r/SLA/AGGRaster.hpp`.
//!
//! C++ Reference:
//! - SLA/AGGRaster.hpp
//!
//! The C++ `AGGRaster` is a class template over an AGG pixel renderer
//! (AGGRaster.hpp:35-38). The only instantiation in libslic3r is
//! `_RasterGrayscaleAA = AGGRaster<agg::pixfmt_gray8,
//! agg::renderer_scanline_aa_solid>` (AGGRaster.hpp:181-182), for which
//! `TColor` is `agg::gray8` (a single `uint8_t` gray value), `TValue` is
//! `uint8_t`, `TPixel` is one byte and `num_components == 1`. The Rust port
//! models that single instantiation concretely.
//!
//! PORTED (the raster's data layer): the pixel buffer, resolution, scaled
//! pixel dimensions, trafo, foreground color state, `clear`, `read_pixel`,
//! `trafo()`, `resolution()`, `pixel_dimensions()`, `encode()`, `getPx`/`getPy`.
//!
//! BLOCKED SYMBOLS (the AGG scanline rasterization kernel — the vendored AGG
//! C++ library `agg_rasterizer_scanline_aa`, `agg_scanline_p`,
//! `agg_renderer_scanline`, `agg_path_storage`, gamma LUTs — is a native C++
//! dependency that has no Rust port yet):
//! - `AGGRaster::flipy` / `flipx` (AGGRaster.hpp:63-71) — operate on
//!   `agg::path_storage`.
//! - `AGGRaster::to_path` / `_to_path` / `_to_path_flpxy`
//!   (AGGRaster.hpp:75-112) — build `agg::path_storage`.
//! - `AGGRaster::_draw` / `draw` (AGGRaster.hpp:114-122, 164) — run
//!   `agg::render_scanlines` on the rasterizer.
//! - The `GammaFn` constructor parameter (AGGRaster.hpp:125-131, 153) — it is
//!   forwarded solely to `m_rasterizer.gamma(gammafn)`, part of the blocked
//!   kernel; the Rust constructors omit it.
//! - `RasterGrayscaleAAGammaPower` (AGGRaster.hpp:210-218) — only forwards
//!   `agg::gamma_power(gamma)` into the blocked kernel.
//! - The `sla::RasterBase` trait impl — requires `draw`.

use crate::geometry::{ExPolygon, Point, Polygon, Polygons};
use crate::sla::raster_base::{EncodedRaster, PixelDim, RasterEncoder, Resolution, Trafo};

// libslic3r.h:58  static constexpr double SCALING_FACTOR = 0.00001;
// NOTE: the crate-level `crate::SCALING_FACTOR` is defined as the *reciprocal*
// (100_000.0, units per mm); the C++ formulas below use the C++ constant
// (mm per unit), reproduced here verbatim.
const SCALING_FACTOR: f64 = 0.00001;

// AGGRaster.hpp:22  inline const Polygon& contour(const ExPolygon& p) { return p.contour; }
#[inline]
pub fn contour(p: &ExPolygon) -> &Polygon {
    &p.contour
}

// AGGRaster.hpp:23  inline const Polygons& holes(const ExPolygon& p) { return p.holes; }
#[inline]
pub fn holes(p: &ExPolygon) -> &Polygons {
    &p.holes
}

// AGGRaster.hpp:27-30  template<class Color> struct Colors {
//     static const Color White;
//     static const Color Black;
// };
//
// Instantiated for the gray8 color type used by `RasterGrayscaleAA`
// (`TColor::value_type == uint8_t`).
pub struct Colors;

impl Colors {
    // AGGRaster.hpp:32  template<class Color> const Color Colors<Color>::White = Color{255};
    pub const WHITE: u8 = 255;
    // AGGRaster.hpp:33  template<class Color> const Color Colors<Color>::Black = Color{0};
    pub const BLACK: u8 = 0;
}

// AGGRaster.hpp:35-39
// template<class PixelRenderer,
//          template<class> class Renderer,
//          class Rasterizer = agg::rasterizer_scanline_aa<>,
//          class Scanline   = agg::scanline_p8>
// class AGGRaster: public RasterBase
//
// Concrete gray8 instantiation (see module docs). The AGG kernel members
// `m_rbuf`, `m_pixrenderer`, `m_raw_renderer`, `m_scanlines`, `m_rasterizer`
// (AGGRaster.hpp:52-61) are views/algorithms over `m_buf`; the only piece of
// their state that outlives a `draw` call is the renderer's fill color
// (AGGRaster.hpp:150), kept here as `m_foreground`.
#[derive(Debug, Clone)]
pub struct AGGRaster {
    // AGGRaster.hpp:48  Resolution m_resolution;
    m_resolution: Resolution,
    // AGGRaster.hpp:49  PixelDim m_pxdim_scaled;    // used for scaled coordinate polygons
    m_pxdim_scaled: PixelDim,
    // AGGRaster.hpp:51  std::vector<TPixel> m_buf;  (TPixel == uint8_t for gray8)
    m_buf: Vec<u8>,
    // AGGRaster.hpp:59  Trafo m_trafo;
    m_trafo: Trafo,
    // AGGRaster.hpp:150  m_renderer.color(foreground);  (renderer state, see above)
    #[allow(dead_code)]
    m_foreground: u8,
}

impl AGGRaster {
    // AGGRaster.hpp:73  double getPx(const Point &p) { return p(0) * m_pxdim_scaled.w_mm; }
    #[allow(dead_code)]
    #[inline]
    fn get_px(&self, p: &Point) -> f64 {
        p.x as f64 * self.m_pxdim_scaled.w_mm
    }

    // AGGRaster.hpp:74  double getPy(const Point &p) { return p(1) * m_pxdim_scaled.h_mm; }
    #[allow(dead_code)]
    #[inline]
    fn get_py(&self, p: &Point) -> f64 {
        p.y as f64 * self.m_pxdim_scaled.h_mm
    }

    // AGGRaster.hpp:125-154
    // template<class GammaFn>
    // AGGRaster(const Resolution &res,
    //           const PixelDim &  pd,
    //           const Trafo &     trafo,
    //           const TColor &    foreground,
    //           const TColor &    background,
    //           GammaFn &&        gammafn)
    //
    // (The `GammaFn` parameter is omitted: AGGRaster.hpp:153
    // `m_rasterizer.gamma(gammafn);` belongs to the blocked AGG kernel.)
    pub fn new(
        res: &Resolution,
        pd: &PixelDim,
        trafo: &Trafo,
        foreground: u8,
        background: u8,
    ) -> Self {
        // AGGRaster.hpp:132  m_resolution(res)
        let m_resolution = *res;
        // AGGRaster.hpp:133  m_pxdim_scaled(SCALING_FACTOR, SCALING_FACTOR)
        let mut m_pxdim_scaled = PixelDim::new(SCALING_FACTOR, SCALING_FACTOR);
        // AGGRaster.hpp:134  m_buf(res.pixels())
        let m_buf = vec![0u8; res.pixels()];
        // AGGRaster.hpp:135-141  m_rbuf / m_pixrenderer / m_raw_renderer /
        // m_renderer wire the AGG view chain over m_buf (blocked kernel state).
        // AGGRaster.hpp:142  m_trafo(trafo)
        let m_trafo = *trafo;

        // AGGRaster.hpp:144-145
        // Visual Studio compiler gives warnings about possible division by zero.
        // assert(pd.w_mm != 0 && pd.h_mm != 0);
        debug_assert!(pd.w_mm != 0. && pd.h_mm != 0.);
        // AGGRaster.hpp:146-149
        if pd.w_mm != 0. && pd.h_mm != 0. {
            m_pxdim_scaled.w_mm /= pd.w_mm;
            m_pxdim_scaled.h_mm /= pd.h_mm;
        }

        let mut this = Self {
            m_resolution,
            m_pxdim_scaled,
            m_buf,
            m_trafo,
            // AGGRaster.hpp:150  m_renderer.color(foreground);
            m_foreground: foreground,
        };
        // AGGRaster.hpp:151  clear(background);
        this.clear(background);

        // AGGRaster.hpp:153  m_rasterizer.gamma(gammafn);  (BLOCKED, see ctor docs)

        this
    }

    // AGGRaster.hpp:156  Trafo trafo() const override { return m_trafo; }
    pub fn trafo(&self) -> Trafo {
        self.m_trafo
    }

    // AGGRaster.hpp:157  Resolution resolution() const { return m_resolution; }
    pub fn resolution(&self) -> Resolution {
        self.m_resolution
    }

    // AGGRaster.hpp:158-162
    // PixelDim   pixel_dimensions() const
    // {
    //     return {SCALING_FACTOR / m_pxdim_scaled.w_mm,
    //             SCALING_FACTOR / m_pxdim_scaled.h_mm};
    // }
    pub fn pixel_dimensions(&self) -> PixelDim {
        PixelDim::new(
            SCALING_FACTOR / self.m_pxdim_scaled.w_mm,
            SCALING_FACTOR / self.m_pxdim_scaled.h_mm,
        )
    }

    // AGGRaster.hpp:164  void draw(const ExPolygon &poly) override { _draw(poly); }
    // BLOCKED: requires the AGG scanline rasterization kernel (see module docs).

    // AGGRaster.hpp:166-169
    // EncodedRaster encode(RasterEncoder encoder) const override
    // {
    //     return encoder(m_buf.data(), m_resolution.width_px, m_resolution.height_px, 1);
    // }
    pub fn encode(&self, mut encoder: RasterEncoder) -> EncodedRaster {
        encoder(
            &self.m_buf,
            self.m_resolution.width_px,
            self.m_resolution.height_px,
            1,
        )
    }

    // AGGRaster.hpp:171  void clear(const TColor color) { m_raw_renderer.clear(color); }
    //
    // `agg::renderer_base::clear` fills every pixel of the attached buffer
    // with `color` (one byte per pixel for gray8).
    pub fn clear(&mut self, color: u8) {
        for px in self.m_buf.iter_mut() {
            *px = color;
        }
    }
}

// AGGRaster.hpp:174-180
// /*
//  * Captures an anti-aliased monochrome canvas where vectorial
//  * polygons can be rasterized. Fill color is always white and the background is
//  * black. Contours are anti-aliased.
//  *
//  * A gamma function can be specified at compile time to make it more flexible.
//  */
// AGGRaster.hpp:181-182
// using _RasterGrayscaleAA =
//     AGGRaster<agg::pixfmt_gray8, agg::renderer_scanline_aa_solid>;
//
// AGGRaster.hpp:184  class RasterGrayscaleAA : public _RasterGrayscaleAA
// (C++ public inheritance -> composition + explicit delegation in Rust.)
#[derive(Debug, Clone)]
pub struct RasterGrayscaleAA {
    base: AGGRaster,
}

impl RasterGrayscaleAA {
    // AGGRaster.hpp:189-196
    // template<class GammaFn>
    // RasterGrayscaleAA(const Resolution &res,
    //                   const PixelDim &  pd,
    //                   const RasterBase::Trafo &     trafo,
    //                   GammaFn &&                    fn)
    //     : Base(res, pd, trafo, Colors<TColor>::White, Colors<TColor>::Black,
    //            std::forward<GammaFn>(fn))
    //
    // (`GammaFn` omitted — blocked, see module docs.)
    pub fn new(res: &Resolution, pd: &PixelDim, trafo: &Trafo) -> Self {
        Self {
            base: AGGRaster::new(res, pd, trafo, Colors::WHITE, Colors::BLACK),
        }
    }

    // AGGRaster.hpp:198-205
    // uint8_t read_pixel(size_t col, size_t row) const
    // {
    //     static_assert(std::is_same<TValue, uint8_t>::value, "Not grayscale pix");
    //
    //     uint8_t px;
    //     Base::m_buf[row * Base::resolution().width_px + col].get(px);
    //     return px;
    // }
    pub fn read_pixel(&self, col: usize, row: usize) -> u8 {
        self.base.m_buf[row * self.base.resolution().width_px + col]
    }

    // AGGRaster.hpp:207  void clear() { Base::clear(Colors<TColor>::Black); }
    pub fn clear(&mut self) {
        self.base.clear(Colors::BLACK);
    }

    // --- Base class (public) interface, inherited in C++ -------------------

    // AGGRaster.hpp:156 (inherited)
    pub fn trafo(&self) -> Trafo {
        self.base.trafo()
    }

    // AGGRaster.hpp:157 (inherited)
    pub fn resolution(&self) -> Resolution {
        self.base.resolution()
    }

    // AGGRaster.hpp:158-162 (inherited)
    pub fn pixel_dimensions(&self) -> PixelDim {
        self.base.pixel_dimensions()
    }

    // AGGRaster.hpp:166-169 (inherited)
    pub fn encode(&self, encoder: RasterEncoder) -> EncodedRaster {
        self.base.encode(encoder)
    }
}

// AGGRaster.hpp:210-218  class RasterGrayscaleAAGammaPower: public RasterGrayscaleAA
// BLOCKED: its constructor only adds `agg::gamma_power(gamma)`, which feeds
// the blocked AGG rasterizer kernel (see module docs).
