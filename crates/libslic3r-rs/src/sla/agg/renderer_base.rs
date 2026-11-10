//! Faithful port of the vendored AGG header `src/agg/agg_renderer_base.h`
//! (Anti-Grain Geometry 2.4, as bundled with BambuStudio) — the
//! `renderer_base<pixfmt_gray8>` instantiation.
//!
//! C++ Reference:
//! - agg/agg_renderer_base.h
//!
//! Only the members exercised by the SLA grayscale raster pipeline are ported:
//! the constructor (full-window clip box), `clear`, `blend_hline` and
//! `blend_solid_hspan` (called by the solid scanline renderer).

use super::basics::{CoverType, RectI};
use super::color_gray::Gray8;
use super::pixfmt_gray::PixfmtGray8;

// agg_renderer_base.h:30  template<class PixelFormat> class renderer_base
#[derive(Debug)]
pub struct RendererBase<'a> {
    // agg_renderer_base.h (private)  pixfmt_type* m_ren;
    m_ren: PixfmtGray8<'a>,
    // agg_renderer_base.h (private)  rect_i m_clip_box;
    m_clip_box: RectI,
}

impl<'a> RendererBase<'a> {
    // agg_renderer_base.h:39-42
    // explicit renderer_base(pixfmt_type& ren) :
    //     m_ren(&ren),
    //     m_clip_box(0, 0, ren.width() - 1, ren.height() - 1)
    // {}
    pub fn new(ren: PixfmtGray8<'a>) -> Self {
        let clip_box = RectI::new(0, 0, ren.width() as i32 - 1, ren.height() as i32 - 1);
        Self {
            m_ren: ren,
            m_clip_box: clip_box,
        }
    }

    // agg_renderer_base.h:54  unsigned width()  const { return m_ren->width();  }
    #[inline]
    pub fn width(&self) -> u32 {
        self.m_ren.width()
    }

    // agg_renderer_base.h:55  unsigned height() const { return m_ren->height(); }
    #[inline]
    pub fn height(&self) -> u32 {
        self.m_ren.height()
    }

    // agg_renderer_base.h (accessors)
    // const rect_i& clip_box() const { return m_clip_box;    }
    // int           xmin()     const { return m_clip_box.x1; }
    // int           ymin()     const { return m_clip_box.y1; }
    // int           xmax()     const { return m_clip_box.x2; }
    // int           ymax()     const { return m_clip_box.y2; }
    #[inline]
    pub fn clip_box(&self) -> &RectI {
        &self.m_clip_box
    }
    #[inline]
    pub fn xmin(&self) -> i32 {
        self.m_clip_box.x1
    }
    #[inline]
    pub fn ymin(&self) -> i32 {
        self.m_clip_box.y1
    }
    #[inline]
    pub fn xmax(&self) -> i32 {
        self.m_clip_box.x2
    }
    #[inline]
    pub fn ymax(&self) -> i32 {
        self.m_clip_box.y2
    }

    // agg_renderer_base.h:124-134
    // void clear(const color_type& c)
    // {
    //     unsigned y;
    //     if(width())
    //     {
    //         for(y = 0; y < height(); y++)
    //         {
    //             m_ren->copy_hline(0, y, width(), c);
    //         }
    //     }
    // }
    pub fn clear(&mut self, c: &Gray8) {
        if self.width() != 0 {
            for y in 0..self.height() {
                let w = self.width();
                self.m_ren.copy_hline(0, y as i32, w, c);
            }
        }
    }

    // agg_renderer_base.h:222-235
    // void blend_hline(int x1, int y, int x2, const color_type& c, cover_type cover)
    // {
    //     if(x1 > x2) { int t = x2; x2 = x1; x1 = t; }
    //     if(y  > ymax()) return;
    //     if(y  < ymin()) return;
    //     if(x1 > xmax()) return;
    //     if(x2 < xmin()) return;
    //
    //     if(x1 < xmin()) x1 = xmin();
    //     if(x2 > xmax()) x2 = xmax();
    //
    //     m_ren->blend_hline(x1, y, x2 - x1 + 1, c, cover);
    // }
    pub fn blend_hline(&mut self, mut x1: i32, y: i32, mut x2: i32, c: &Gray8, cover: CoverType) {
        if x1 > x2 {
            std::mem::swap(&mut x1, &mut x2);
        }
        if y > self.ymax() {
            return;
        }
        if y < self.ymin() {
            return;
        }
        if x1 > self.xmax() {
            return;
        }
        if x2 < self.xmin() {
            return;
        }

        if x1 < self.xmin() {
            x1 = self.xmin();
        }
        if x2 > self.xmax() {
            x2 = self.xmax();
        }

        self.m_ren
            .blend_hline(x1, y, (x2 - x1 + 1) as u32, c, cover as u32);
    }

    // agg_renderer_base.h:275-295
    // void blend_solid_hspan(int x, int y, int len, const color_type& c, const cover_type* covers)
    // {
    //     if(y > ymax()) return;
    //     if(y < ymin()) return;
    //
    //     if(x < xmin())
    //     {
    //         len -= xmin() - x;
    //         if(len <= 0) return;
    //         covers += xmin() - x;
    //         x = xmin();
    //     }
    //     if(x + len > xmax())
    //     {
    //         len = xmax() - x + 1;
    //         if(len <= 0) return;
    //     }
    //     m_ren->blend_solid_hspan(x, y, len, c, covers);
    // }
    pub fn blend_solid_hspan(&mut self, mut x: i32, y: i32, mut len: i32, c: &Gray8, covers: &[u8]) {
        if y > self.ymax() {
            return;
        }
        if y < self.ymin() {
            return;
        }

        let mut covers_off = 0usize; // covers pointer offset
        if x < self.xmin() {
            len -= self.xmin() - x;
            if len <= 0 {
                return;
            }
            covers_off = (self.xmin() - x) as usize;
            x = self.xmin();
        }
        if x + len > self.xmax() {
            len = self.xmax() - x + 1;
            if len <= 0 {
                return;
            }
        }
        self.m_ren
            .blend_solid_hspan(x, y, len as u32, c, &covers[covers_off..]);
    }
}
