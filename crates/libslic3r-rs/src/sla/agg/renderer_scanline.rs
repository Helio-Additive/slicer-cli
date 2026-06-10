//! Faithful port of the vendored AGG header `src/agg/agg_renderer_scanline.h`
//! (Anti-Grain Geometry 2.4, as bundled with BambuStudio) — the solid-color
//! anti-aliased scanline renderer used by `SLA/AGGRaster.hpp`
//! (`renderer_scanline_aa_solid<renderer_base<pixfmt_gray8>>`) and the
//! generic `render_scanlines` driver.
//!
//! C++ Reference:
//! - agg/agg_renderer_scanline.h
//!
//! The C++ functions are templates; the Rust port instantiates them for the
//! gray8 pipeline (`RasterizerScanlineAa` + `ScanlineP8` + `RendererBase`),
//! the only instantiation used by libslic3r. The span-generator renderers
//! (`render_scanline_aa`, `renderer_scanline_bin`, ...) have no users here
//! and are not ported.

use super::color_gray::Gray8;
use super::rasterizer_scanline_aa::RasterizerScanlineAa;
use super::renderer_base::RendererBase;
use super::scanline_p::ScanlineP8;

// agg_renderer_scanline.h:27-55 (approx)
// //================================================render_scanline_aa_solid
// template<class Scanline, class BaseRenderer, class ColorT>
// void render_scanline_aa_solid(const Scanline& sl,
//                               BaseRenderer& ren,
//                               const ColorT& color)
pub fn render_scanline_aa_solid(sl: &ScanlineP8, ren: &mut RendererBase, color: &Gray8) {
    // int y = sl.y();
    let y: i32 = sl.y();
    // unsigned num_spans = sl.num_spans();
    let mut num_spans: u32 = sl.num_spans();
    // typename Scanline::const_iterator span = sl.begin();
    let spans = sl.begin();
    let covers = sl.covers();
    let mut span_idx: usize = 0;

    loop {
        let span = &spans[span_idx];
        // int x = span->x;
        let x: i32 = span.x as i32;
        if span.len > 0 {
            // ren.blend_solid_hspan(x, y, (unsigned)span->len, color, span->covers);
            ren.blend_solid_hspan(x, y, span.len as i32, color, &covers[span.covers..]);
        } else {
            // ren.blend_hline(x, y, (unsigned)(x - span->len - 1), color, *(span->covers));
            ren.blend_hline(x, y, x - span.len as i32 - 1, color, covers[span.covers]);
        }
        // if(--num_spans == 0) break;
        num_spans -= 1;
        if num_spans == 0 {
            break;
        }
        // ++span;
        span_idx += 1;
    }
}

// agg_renderer_scanline.h:107-138
// //==============================================renderer_scanline_aa_solid
// template<class BaseRenderer> class renderer_scanline_aa_solid
pub struct RendererScanlineAaSolid<'a, 'b> {
    // agg_renderer_scanline.h:136  base_ren_type* m_ren;
    m_ren: &'b mut RendererBase<'a>,
    // agg_renderer_scanline.h:137  color_type m_color;
    m_color: Gray8,
}

impl<'a, 'b> RendererScanlineAaSolid<'a, 'b> {
    // agg_renderer_scanline.h:116
    // explicit renderer_scanline_aa_solid(base_ren_type& ren) : m_ren(&ren) {}
    pub fn new(ren: &'b mut RendererBase<'a>) -> Self {
        Self {
            m_ren: ren,
            m_color: Gray8::default(),
        }
    }

    // agg_renderer_scanline.h:123  void color(const color_type& c) { m_color = c; }
    pub fn color(&mut self, c: &Gray8) {
        self.m_color = *c;
    }

    // agg_renderer_scanline.h:124  const color_type& color() const { return m_color; }
    pub fn get_color(&self) -> &Gray8 {
        &self.m_color
    }

    // agg_renderer_scanline.h:127  void prepare() {}
    pub fn prepare(&mut self) {}

    // agg_renderer_scanline.h:130-133
    // template<class Scanline> void render(const Scanline& sl)
    // {
    //     render_scanline_aa_solid(sl, *m_ren, m_color);
    // }
    pub fn render(&mut self, sl: &ScanlineP8) {
        render_scanline_aa_solid(sl, self.m_ren, &self.m_color);
    }
}

// agg_renderer_scanline.h:438-451
// //=====================================================render_scanlines
// template<class Rasterizer, class Scanline, class Renderer>
// void render_scanlines(Rasterizer& ras, Scanline& sl, Renderer& ren)
// {
//     if(ras.rewind_scanlines())
//     {
//         sl.reset(ras.min_x(), ras.max_x());
//         ren.prepare();
//         while(ras.sweep_scanline(sl))
//         {
//             ren.render(sl);
//         }
//     }
// }
pub fn render_scanlines(
    ras: &mut RasterizerScanlineAa,
    sl: &mut ScanlineP8,
    ren: &mut RendererScanlineAaSolid,
) {
    if ras.rewind_scanlines() {
        sl.reset(ras.min_x(), ras.max_x());
        ren.prepare();
        while ras.sweep_scanline(sl) {
            ren.render(sl);
        }
    }
}
