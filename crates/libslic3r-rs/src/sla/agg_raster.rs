//! Faithful 1:1 line-by-line port of BambuStudio `src/libslic3r/SLA/AGGRaster.hpp`.
//!
//! C++ Reference:
//! - SLA/AGGRaster.hpp
//!
//! The C++ `AGGRaster` is a class template over an AGG pixel renderer
//! (AGGRaster.hpp:35-38). The only instantiation in libslic3r is
//! `_RasterGrayscaleAA = AGGRaster<agg::pixfmt_gray8,
//! agg::renderer_scanline_aa_solid>` (AGGRaster.hpp:181-182), for which
//! `TColor` is `agg::gray8`, `TValue` is `uint8_t`, `TPixel` is one byte and
//! `num_components == 1`. The Rust port models that single instantiation
//! concretely on top of the faithful AGG kernel port in `crate::sla::agg`.
//!
//! Ownership note: in C++ the members `m_rbuf`, `m_pixrenderer`,
//! `m_raw_renderer` and `m_renderer` (AGGRaster.hpp:52-57) form a chain of
//! non-owning views over `m_buf`. Such self-referential members are not
//! expressible in safe Rust; the view chain is therefore reconstructed
//! transiently inside `clear()` and `_draw()`. This is behavior-preserving:
//! the only state those views carry across calls is the scanline renderer's
//! fill color (set once at AGGRaster.hpp:150), which is kept here as
//! `m_renderer_color`, and `renderer_base`'s clip box, which is always the
//! full window (set at construction, never modified).

use crate::geometry::{ExPolygon, Point, Polygon, Polygons};
use crate::sla::agg::color_gray::Gray8;
use crate::sla::agg::gamma_functions::{GammaFunction, GammaPower};
use crate::sla::agg::path_storage::PathStorage;
use crate::sla::agg::pixfmt_gray::PixfmtGray8;
use crate::sla::agg::rasterizer_scanline_aa::RasterizerScanlineAa;
use crate::sla::agg::renderer_base::RendererBase;
use crate::sla::agg::renderer_scanline::{render_scanlines, RendererScanlineAaSolid};
use crate::sla::agg::rendering_buffer::RenderingBuffer;
use crate::sla::agg::scanline_p::ScanlineP8;
use crate::sla::raster_base::{
    EncodedRaster, PixelDim, RasterBase, RasterEncoder, Resolution, Trafo,
};

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
// Instantiated for the gray8 color type used by `RasterGrayscaleAA`. The C++
// initializer `Color{255}` calls `gray8T(unsigned v_, unsigned a_=base_mask)`,
// so both colors are fully opaque (a == 255).
pub struct Colors;

impl Colors {
    // AGGRaster.hpp:32  template<class Color> const Color Colors<Color>::White = Color{255};
    pub const WHITE: Gray8 = Gray8 { v: 255, a: 255 };
    // AGGRaster.hpp:33  template<class Color> const Color Colors<Color>::Black = Color{0};
    pub const BLACK: Gray8 = Gray8 { v: 0, a: 255 };
}

// AGGRaster.hpp:35-39
// template<class PixelRenderer,
//          template<class /*agg::renderer_base<PixelRenderer>*/> class Renderer,
//          class Rasterizer = agg::rasterizer_scanline_aa<>,
//          class Scanline   = agg::scanline_p8>
// class AGGRaster: public RasterBase
pub struct AGGRaster {
    // AGGRaster.hpp:48  Resolution m_resolution;
    m_resolution: Resolution,
    // AGGRaster.hpp:49  PixelDim m_pxdim_scaled;    // used for scaled coordinate polygons
    m_pxdim_scaled: PixelDim,

    // AGGRaster.hpp:51  std::vector<TPixel> m_buf;  (TPixel == one byte for gray8)
    m_buf: Vec<u8>,
    // AGGRaster.hpp:52  agg::rendering_buffer m_rbuf;
    // AGGRaster.hpp:54  PixelRenderer m_pixrenderer;
    // AGGRaster.hpp:56  agg::renderer_base<PixelRenderer> m_raw_renderer;
    // AGGRaster.hpp:57  Renderer<agg::renderer_base<PixelRenderer>> m_renderer;
    // (non-owning views over m_buf; reconstructed transiently in clear()/_draw(),
    //  see the module docs. The renderer's persistent color state lives here:)
    m_renderer_color: Gray8,

    // AGGRaster.hpp:59  Trafo m_trafo;
    m_trafo: Trafo,
    // AGGRaster.hpp:60  Scanline m_scanlines;
    m_scanlines: ScanlineP8,
    // AGGRaster.hpp:61  Rasterizer m_rasterizer;
    m_rasterizer: RasterizerScanlineAa,
}

impl AGGRaster {
    // AGGRaster.hpp:63-66
    // void flipy(agg::path_storage &path) const
    // {
    //     path.flip_y(0, double(m_resolution.height_px));
    // }
    fn flipy(&self, path: &mut PathStorage) {
        path.flip_y(0.0, self.m_resolution.height_px as f64);
    }

    // AGGRaster.hpp:68-71
    // void flipx(agg::path_storage &path) const
    // {
    //     path.flip_x(0, double(m_resolution.width_px));
    // }
    fn flipx(&self, path: &mut PathStorage) {
        path.flip_x(0.0, self.m_resolution.width_px as f64);
    }

    // AGGRaster.hpp:73  double getPx(const Point &p) { return p(0) * m_pxdim_scaled.w_mm; }
    #[inline]
    fn get_px(&self, p: &Point) -> f64 {
        p.x as f64 * self.m_pxdim_scaled.w_mm
    }

    // AGGRaster.hpp:74  double getPy(const Point &p) { return p(1) * m_pxdim_scaled.h_mm; }
    #[inline]
    fn get_py(&self, p: &Point) -> f64 {
        p.y as f64 * self.m_pxdim_scaled.h_mm
    }

    // AGGRaster.hpp:75  agg::path_storage to_path(const Polygon &poly) { return to_path(poly.points); }
    fn to_path_polygon(&self, poly: &Polygon) -> PathStorage {
        self.to_path(&poly.points)
    }

    // AGGRaster.hpp:77-87
    // template<class PointVec> agg::path_storage _to_path(const PointVec& v)
    // {
    //     agg::path_storage path;
    //
    //     auto it = v.begin();
    //     path.move_to(getPx(*it), getPy(*it));
    //     while(++it != v.end()) path.line_to(getPx(*it), getPy(*it));
    //     path.line_to(getPx(v.front()), getPy(v.front()));
    //
    //     return path;
    // }
    #[allow(non_snake_case)]
    fn _to_path(&self, v: &[Point]) -> PathStorage {
        let mut path = PathStorage::new();

        let mut it = v.iter();
        let first = it.next().unwrap();
        path.move_to(self.get_px(first), self.get_py(first));
        for p in it {
            path.line_to(self.get_px(p), self.get_py(p));
        }
        path.line_to(self.get_px(&v[0]), self.get_py(&v[0]));

        path
    }

    // AGGRaster.hpp:89-99
    // template<class PointVec> agg::path_storage _to_path_flpxy(const PointVec& v)
    // {
    //     agg::path_storage path;
    //
    //     auto it = v.begin();
    //     path.move_to(getPy(*it), getPx(*it));
    //     while(++it != v.end()) path.line_to(getPy(*it), getPx(*it));
    //     path.line_to(getPy(v.front()), getPx(v.front()));
    //
    //     return path;
    // }
    #[allow(non_snake_case)]
    fn _to_path_flpxy(&self, v: &[Point]) -> PathStorage {
        let mut path = PathStorage::new();

        let mut it = v.iter();
        let first = it.next().unwrap();
        path.move_to(self.get_py(first), self.get_px(first));
        for p in it {
            path.line_to(self.get_py(p), self.get_px(p));
        }
        path.line_to(self.get_py(&v[0]), self.get_px(&v[0]));

        path
    }

    // AGGRaster.hpp:101-112
    // template<class PointVec> agg::path_storage to_path(const PointVec &v)
    // {
    //     auto path = m_trafo.flipXY ? _to_path_flpxy(v) : _to_path(v);
    //
    //     path.translate_all_paths(m_trafo.center_x * m_pxdim_scaled.w_mm,
    //                              m_trafo.center_y * m_pxdim_scaled.h_mm);
    //
    //     if(m_trafo.mirror_x) flipx(path);
    //     if(m_trafo.mirror_y) flipy(path);
    //
    //     return path;
    // }
    fn to_path(&self, v: &[Point]) -> PathStorage {
        let mut path = if self.m_trafo.flip_xy {
            self._to_path_flpxy(v)
        } else {
            self._to_path(v)
        };

        path.translate_all_paths(
            self.m_trafo.center_x as f64 * self.m_pxdim_scaled.w_mm,
            self.m_trafo.center_y as f64 * self.m_pxdim_scaled.h_mm,
        );

        if self.m_trafo.mirror_x {
            self.flipx(&mut path);
        }
        if self.m_trafo.mirror_y {
            self.flipy(&mut path);
        }

        path
    }

    // AGGRaster.hpp:114-122
    // template<class P> void _draw(const P &poly)
    // {
    //     m_rasterizer.reset();
    //
    //     m_rasterizer.add_path(to_path(contour(poly)));
    //     for(auto& h : holes(poly)) m_rasterizer.add_path(to_path(h));
    //
    //     agg::render_scanlines(m_rasterizer, m_scanlines, m_renderer);
    // }
    #[allow(non_snake_case)]
    fn _draw(&mut self, poly: &ExPolygon) {
        self.m_rasterizer.reset();

        let mut path = self.to_path_polygon(contour(poly));
        self.m_rasterizer.add_path(&mut path, 0);
        for h in holes(poly) {
            let mut hpath = self.to_path_polygon(h);
            self.m_rasterizer.add_path(&mut hpath, 0);
        }

        // agg::render_scanlines(m_rasterizer, m_scanlines, m_renderer);
        // (reconstruct the C++ member view chain m_rbuf -> m_pixrenderer ->
        //  m_raw_renderer -> m_renderer over m_buf; see module docs)
        let stride = (self.m_resolution.width_px * PixfmtGray8::NUM_COMPONENTS as usize) as i32;
        let rbuf = RenderingBuffer::new(
            &mut self.m_buf,
            self.m_resolution.width_px as u32,
            self.m_resolution.height_px as u32,
            stride,
        );
        let pixrenderer = PixfmtGray8::new(rbuf);
        let mut raw_renderer = RendererBase::new(pixrenderer);
        let mut renderer = RendererScanlineAaSolid::new(&mut raw_renderer);
        renderer.color(&self.m_renderer_color);
        render_scanlines(&mut self.m_rasterizer, &mut self.m_scanlines, &mut renderer);
    }

    // AGGRaster.hpp:125-154
    // template<class GammaFn>
    // AGGRaster(const Resolution &res,
    //           const PixelDim &  pd,
    //           const Trafo &     trafo,
    //           const TColor &    foreground,
    //           const TColor &    background,
    //           GammaFn &&        gammafn)
    //     : m_resolution(res)
    //     , m_pxdim_scaled(SCALING_FACTOR, SCALING_FACTOR)
    //     , m_buf(res.pixels())
    //     , m_rbuf(reinterpret_cast<TValue *>(m_buf.data()),
    //              unsigned(res.width_px),
    //              unsigned(res.height_px),
    //              int(res.width_px *PixelRenderer::num_components))
    //     , m_pixrenderer(m_rbuf)
    //     , m_raw_renderer(m_pixrenderer)
    //     , m_renderer(m_raw_renderer)
    //     , m_trafo(trafo)
    pub fn new<GammaFn: GammaFunction>(
        res: &Resolution,
        pd: &PixelDim,
        trafo: &Trafo,
        foreground: &Gray8,
        background: &Gray8,
        gammafn: GammaFn,
    ) -> Self {
        let mut this = Self {
            m_resolution: *res,
            m_pxdim_scaled: PixelDim::new(SCALING_FACTOR, SCALING_FACTOR),
            m_buf: vec![0u8; res.pixels()],
            m_renderer_color: Gray8::default(),
            m_trafo: *trafo,
            m_scanlines: ScanlineP8::new(),
            m_rasterizer: RasterizerScanlineAa::new(),
        };

        // AGGRaster.hpp:144-145
        // Visual Studio compiler gives warnings about possible division by zero.
        // assert(pd.w_mm != 0 && pd.h_mm != 0);
        debug_assert!(pd.w_mm != 0. && pd.h_mm != 0.);
        // AGGRaster.hpp:146-149
        if pd.w_mm != 0. && pd.h_mm != 0. {
            this.m_pxdim_scaled.w_mm /= pd.w_mm;
            this.m_pxdim_scaled.h_mm /= pd.h_mm;
        }
        // AGGRaster.hpp:150  m_renderer.color(foreground);
        this.m_renderer_color = *foreground;
        // AGGRaster.hpp:151  clear(background);
        this.clear(background);

        // AGGRaster.hpp:153  m_rasterizer.gamma(gammafn);
        this.m_rasterizer.gamma(&gammafn);

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
    pub fn draw(&mut self, poly: &ExPolygon) {
        self._draw(poly);
    }

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
    pub fn clear(&mut self, color: &Gray8) {
        // (reconstruct the m_rbuf -> m_pixrenderer -> m_raw_renderer view
        //  chain over m_buf; see module docs)
        let stride = (self.m_resolution.width_px * PixfmtGray8::NUM_COMPONENTS as usize) as i32;
        let rbuf = RenderingBuffer::new(
            &mut self.m_buf,
            self.m_resolution.width_px as u32,
            self.m_resolution.height_px as u32,
            stride,
        );
        let pixrenderer = PixfmtGray8::new(rbuf);
        let mut raw_renderer = RendererBase::new(pixrenderer);
        raw_renderer.clear(color);
    }
}

// AGGRaster.hpp:39  class AGGRaster: public RasterBase
impl RasterBase for AGGRaster {
    // AGGRaster.hpp:164
    fn draw(&mut self, poly: &ExPolygon) {
        AGGRaster::draw(self, poly);
    }

    // AGGRaster.hpp:156
    fn trafo(&self) -> Trafo {
        AGGRaster::trafo(self)
    }

    // AGGRaster.hpp:166-169
    fn encode(&self, encoder: RasterEncoder) -> EncodedRaster {
        AGGRaster::encode(self, encoder)
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
    // {}
    pub fn new<GammaFn: GammaFunction>(
        res: &Resolution,
        pd: &PixelDim,
        trafo: &Trafo,
        gamma_fn: GammaFn,
    ) -> Self {
        Self {
            base: AGGRaster::new(res, pd, trafo, &Colors::WHITE, &Colors::BLACK, gamma_fn),
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
        self.base.clear(&Colors::BLACK);
    }

    // --- Base class (public) interface, inherited in C++ -------------------

    // AGGRaster.hpp:164 (inherited)
    pub fn draw(&mut self, poly: &ExPolygon) {
        self.base.draw(poly);
    }

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

// (inherited RasterBase conformance, via _RasterGrayscaleAA -> AGGRaster)
impl RasterBase for RasterGrayscaleAA {
    fn draw(&mut self, poly: &ExPolygon) {
        RasterGrayscaleAA::draw(self, poly);
    }

    fn trafo(&self) -> Trafo {
        RasterGrayscaleAA::trafo(self)
    }

    fn encode(&self, encoder: RasterEncoder) -> EncodedRaster {
        RasterGrayscaleAA::encode(self, encoder)
    }
}

// AGGRaster.hpp:210-218
// class RasterGrayscaleAAGammaPower: public RasterGrayscaleAA
pub struct RasterGrayscaleAAGammaPower {
    base: RasterGrayscaleAA,
}

impl RasterGrayscaleAAGammaPower {
    // AGGRaster.hpp:212-217
    // RasterGrayscaleAAGammaPower(const Resolution &res,
    //                             const PixelDim &  pd,
    //                             const RasterBase::Trafo &     trafo,
    //                             double                        gamma = 1.)
    //     : RasterGrayscaleAA(res, pd, trafo, agg::gamma_power(gamma))
    // {}
    pub fn new(res: &Resolution, pd: &PixelDim, trafo: &Trafo, gamma: f64) -> Self {
        Self {
            base: RasterGrayscaleAA::new(res, pd, trafo, GammaPower::new_with(gamma)),
        }
    }

    /// C++ default argument `gamma = 1.`.
    pub fn new_default_gamma(res: &Resolution, pd: &PixelDim, trafo: &Trafo) -> Self {
        Self::new(res, pd, trafo, 1.)
    }

    // --- Inherited (public) interface ---------------------------------------

    // AGGRaster.hpp:198-205 (inherited)
    pub fn read_pixel(&self, col: usize, row: usize) -> u8 {
        self.base.read_pixel(col, row)
    }

    // AGGRaster.hpp:207 (inherited)
    pub fn clear(&mut self) {
        self.base.clear();
    }

    // AGGRaster.hpp:164 (inherited)
    pub fn draw(&mut self, poly: &ExPolygon) {
        self.base.draw(poly);
    }

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

// (inherited RasterBase conformance, via RasterGrayscaleAA)
impl RasterBase for RasterGrayscaleAAGammaPower {
    fn draw(&mut self, poly: &ExPolygon) {
        RasterGrayscaleAAGammaPower::draw(self, poly);
    }

    fn trafo(&self) -> Trafo {
        RasterGrayscaleAAGammaPower::trafo(self)
    }

    fn encode(&self, encoder: RasterEncoder) -> EncodedRaster {
        RasterGrayscaleAAGammaPower::encode(self, encoder)
    }
}
