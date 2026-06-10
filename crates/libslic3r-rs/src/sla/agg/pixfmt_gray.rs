//! Faithful port of the vendored AGG header `src/agg/agg_pixfmt_gray.h`
//! (Anti-Grain Geometry 2.4, as bundled with BambuStudio) — the
//! `pixfmt_gray8 = pixfmt_alpha_blend_gray<blender_gray8, rendering_buffer>`
//! instantiation (Step = 1, Offset = 0).
//!
//! C++ Reference:
//! - agg/agg_pixfmt_gray.h
//!
//! Only the members exercised by the SLA grayscale raster pipeline
//! (`renderer_base::clear` -> `copy_hline`, and the scanline renderer ->
//! `blend_hline` / `blend_solid_hspan`) are ported. `pixel_type` for gray8 is
//! a single byte; raw pixel pointers become byte indices into the rendering
//! buffer slice.

use super::basics::{CoverType, COVER_MASK};
use super::color_gray::Gray8;
use super::rendering_buffer::RenderingBuffer;

// agg_pixfmt_gray.h:33-56  template<class ColorT> struct blender_gray
pub struct BlenderGray8;

impl BlenderGray8 {
    // agg_pixfmt_gray.h:42-44
    // Blend pixels using the non-premultiplied form of Alvy-Ray Smith's
    // compositing function. Since the render buffer is opaque we skip the
    // initial premultiply and final demultiply.

    // agg_pixfmt_gray.h:46-50
    // static AGG_INLINE void blend_pix(value_type* p,
    //     value_type cv, value_type alpha, cover_type cover)
    // {
    //     blend_pix(p, cv, color_type::mult_cover(alpha, cover));
    // }
    #[inline]
    pub fn blend_pix_cover(p: &mut u8, cv: u8, alpha: u8, cover: CoverType) {
        Self::blend_pix(p, cv, Gray8::mult_cover(alpha, cover));
    }

    // agg_pixfmt_gray.h:52-56
    // static AGG_INLINE void blend_pix(value_type* p,
    //     value_type cv, value_type alpha)
    // {
    //     *p = color_type::lerp(*p, cv, alpha);
    // }
    #[inline]
    pub fn blend_pix(p: &mut u8, cv: u8, alpha: u8) {
        *p = Gray8::lerp(*p, cv, alpha);
    }
}

// agg_pixfmt_gray.h:124-126
// template<class Blender, class RenBuf, unsigned Step = 1, unsigned Offset = 0>
// class pixfmt_alpha_blend_gray
//
// agg_pixfmt_gray.h:726
// typedef pixfmt_alpha_blend_gray<blender_gray8, rendering_buffer> pixfmt_gray8;
#[derive(Debug)]
pub struct PixfmtGray8<'a> {
    // agg_pixfmt_gray.h (private)  rbuf_type* m_rbuf;
    m_rbuf: RenderingBuffer<'a>,
}

impl<'a> PixfmtGray8<'a> {
    // agg_pixfmt_gray.h:138-143  enum { num_components = 1, pix_width = ..., pix_step = Step, pix_offset = Offset }
    pub const NUM_COMPONENTS: u32 = 1;
    pub const PIX_WIDTH: u32 = 1; // sizeof(value_type) * Step
    pub const PIX_STEP: u32 = 1;
    pub const PIX_OFFSET: u32 = 0;

    // agg_pixfmt_gray.h:283-286 (approx)
    // explicit pixfmt_alpha_blend_gray(rbuf_type& rb) : m_rbuf(&rb) {}
    pub fn new(rb: RenderingBuffer<'a>) -> Self {
        Self { m_rbuf: rb }
    }

    // AGG_INLINE unsigned width()  const { return m_rbuf->width();  }
    #[inline]
    pub fn width(&self) -> u32 {
        self.m_rbuf.width()
    }

    // AGG_INLINE unsigned height() const { return m_rbuf->height(); }
    #[inline]
    pub fn height(&self) -> u32 {
        self.m_rbuf.height()
    }

    // pix_value_ptr(x, y, len) -> (pixel_type*)(m_rbuf->row_ptr(x, y, len)
    //                                           + sizeof(value_type) * (x * pix_step + pix_offset))
    // (byte index of the pixel at (x, y); gray8 Step = 1, Offset = 0)
    #[inline]
    fn pix_value_index(&self, x: i32, y: i32) -> usize {
        self.m_rbuf.row_index(y) + (x as usize) * Self::PIX_STEP as usize
    }

    // agg_pixfmt_gray.h:191-195 (private)
    // AGG_INLINE void blend_pix(pixel_type* p, value_type v, value_type a, unsigned cover)
    // {
    //     blender_type::blend_pix(p->c, v, a, cover);
    // }
    // agg_pixfmt_gray.h:205-207 (color overload forwards c.v, c.a)
    #[inline]
    fn blend_pix(&mut self, p: usize, c: &Gray8, cover: u32) {
        BlenderGray8::blend_pix_cover(&mut self.m_rbuf.m_buf[p], c.v, c.a, cover as CoverType);
    }

    // agg_pixfmt_gray.h:361-372
    // AGG_INLINE void copy_hline(int x, int y, unsigned len, const color_type& c)
    // {
    //     pixel_type* p = pix_value_ptr(x, y, len);
    //     do
    //     {
    //         p->set(c);
    //         p = p->next();
    //     }
    //     while(--len);
    // }
    #[inline]
    pub fn copy_hline(&mut self, x: i32, y: i32, mut len: u32, c: &Gray8) {
        let mut p = self.pix_value_index(x, y);
        loop {
            // pixel_type::set(const color_type&) -> c[0] = color.v  (agg_pixfmt_gray.h:148-156)
            self.m_rbuf.m_buf[p] = c.v;
            p += Self::PIX_STEP as usize;
            len -= 1;
            if len == 0 {
                break;
            }
        }
    }

    // agg_pixfmt_gray.h:389-417
    // void blend_hline(int x, int y, unsigned len, const color_type& c, int8u cover)
    pub fn blend_hline(&mut self, x: i32, y: i32, mut len: u32, c: &Gray8, cover: u32) {
        if !c.is_transparent() {
            let mut p = self.pix_value_index(x, y);

            if c.is_opaque() && cover == COVER_MASK {
                loop {
                    self.m_rbuf.m_buf[p] = c.v; // p->set(c)
                    p += Self::PIX_STEP as usize;
                    len -= 1;
                    if len == 0 {
                        break;
                    }
                }
            } else {
                loop {
                    self.blend_pix(p, c, cover);
                    p += Self::PIX_STEP as usize;
                    len -= 1;
                    if len == 0 {
                        break;
                    }
                }
            }
        }
    }

    // agg_pixfmt_gray.h:449-474
    // void blend_solid_hspan(int x, int y, unsigned len, const color_type& c, const int8u* covers)
    pub fn blend_solid_hspan(&mut self, x: i32, y: i32, mut len: u32, c: &Gray8, covers: &[u8]) {
        if !c.is_transparent() {
            let mut p = self.pix_value_index(x, y);
            let mut covers_idx = 0usize;

            loop {
                if c.is_opaque() && covers[covers_idx] as u32 == COVER_MASK {
                    self.m_rbuf.m_buf[p] = c.v; // p->set(c)
                } else {
                    self.blend_pix(p, c, covers[covers_idx] as u32);
                }
                p += Self::PIX_STEP as usize;
                covers_idx += 1; // ++covers
                len -= 1;
                if len == 0 {
                    break;
                }
            }
        }
    }
}
