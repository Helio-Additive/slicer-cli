//! Faithful port of the vendored AGG header `src/agg/agg_rasterizer_sl_clip.h`
//! (Anti-Grain Geometry 2.4, as bundled with BambuStudio) — the
//! `rasterizer_sl_clip_int = rasterizer_sl_clip<ras_conv_int>` instantiation
//! (the default `Clip` parameter of `rasterizer_scanline_aa<>`).
//!
//! C++ Reference:
//! - agg/agg_rasterizer_sl_clip.h
//!
//! The other conversion policies (`ras_conv_int_sat`, `ras_conv_int_3x`,
//! `ras_conv_dbl`, `ras_conv_dbl_3x`; agg_rasterizer_sl_clip.h:42-99) and
//! `rasterizer_sl_no_clip` (:310-333) have no users here and are not ported.

use super::basics::{iround, RectI, POLY_SUBPIXEL_SCALE};
use super::clip_liang_barsky::{clipping_flags, clipping_flags_y};
use super::rasterizer_cells_aa::RasterizerCellsAa;

// agg_rasterizer_sl_clip.h:22-26  enum poly_max_coord_e
pub const POLY_MAX_COORD: i32 = (1 << 30) - 1; //----poly_max_coord

// agg_rasterizer_sl_clip.h:28-40  struct ras_conv_int
pub struct RasConvInt;

impl RasConvInt {
    // typedef int coord_type;

    // agg_rasterizer_sl_clip.h:32-35
    // static AGG_INLINE int mul_div(double a, double b, double c)
    // {
    //     return iround(a * b / c);
    // }
    #[inline]
    pub fn mul_div(a: f64, b: f64, c: f64) -> i32 {
        iround(a * b / c)
    }

    // agg_rasterizer_sl_clip.h:36  static int xi(int v) { return v; }
    #[inline]
    pub fn xi(v: i32) -> i32 {
        v
    }

    // agg_rasterizer_sl_clip.h:37  static int yi(int v) { return v; }
    #[inline]
    pub fn yi(v: i32) -> i32 {
        v
    }

    // agg_rasterizer_sl_clip.h:38
    // static int upscale(double v) { return iround(v * poly_subpixel_scale); }
    #[inline]
    pub fn upscale(v: f64) -> i32 {
        iround(v * POLY_SUBPIXEL_SCALE as f64)
    }

    // agg_rasterizer_sl_clip.h:39  static int downscale(int v) { return v; }
    #[inline]
    pub fn downscale(v: i32) -> i32 {
        v
    }
}

// agg_rasterizer_sl_clip.h:105-106
// template<class Conv> class rasterizer_sl_clip
// agg_rasterizer_sl_clip.h:342
// typedef rasterizer_sl_clip<ras_conv_int> rasterizer_sl_clip_int;
#[derive(Debug)]
pub struct RasterizerSlClipInt {
    // agg_rasterizer_sl_clip.h:300  rect_type m_clip_box;
    m_clip_box: RectI,
    // agg_rasterizer_sl_clip.h:301-302  coord_type m_x1; coord_type m_y1;
    m_x1: i32,
    m_y1: i32,
    // agg_rasterizer_sl_clip.h:303  unsigned m_f1;
    m_f1: u32,
    // agg_rasterizer_sl_clip.h:304  bool m_clipping;
    m_clipping: bool,
}

impl Default for RasterizerSlClipInt {
    fn default() -> Self {
        Self::new()
    }
}

impl RasterizerSlClipInt {
    // agg_rasterizer_sl_clip.h:114-120
    // rasterizer_sl_clip() :
    //     m_clip_box(0,0,0,0),
    //     m_x1(0), m_y1(0), m_f1(0),
    //     m_clipping(false)
    // {}
    pub fn new() -> Self {
        Self {
            m_clip_box: RectI::new(0, 0, 0, 0),
            m_x1: 0,
            m_y1: 0,
            m_f1: 0,
            m_clipping: false,
        }
    }

    // agg_rasterizer_sl_clip.h:123-126
    // void reset_clipping()
    // {
    //     m_clipping = false;
    // }
    pub fn reset_clipping(&mut self) {
        self.m_clipping = false;
    }

    // agg_rasterizer_sl_clip.h:129-134
    // void clip_box(coord_type x1, coord_type y1, coord_type x2, coord_type y2)
    // {
    //     m_clip_box = rect_type(x1, y1, x2, y2);
    //     m_clip_box.normalize();
    //     m_clipping = true;
    // }
    pub fn clip_box(&mut self, x1: i32, y1: i32, x2: i32, y2: i32) {
        self.m_clip_box = RectI::new(x1, y1, x2, y2);
        self.m_clip_box.normalize();
        self.m_clipping = true;
    }

    // agg_rasterizer_sl_clip.h:137-142
    // void move_to(coord_type x1, coord_type y1)
    // {
    //     m_x1 = x1;
    //     m_y1 = y1;
    //     if(m_clipping) m_f1 = clipping_flags(x1, y1, m_clip_box);
    // }
    pub fn move_to(&mut self, x1: i32, y1: i32) {
        self.m_x1 = x1;
        self.m_y1 = y1;
        if self.m_clipping {
            self.m_f1 = clipping_flags(x1, y1, &self.m_clip_box);
        }
    }

    // agg_rasterizer_sl_clip.h:146-198
    // template<class Rasterizer>
    // AGG_INLINE void line_clip_y(Rasterizer& ras,
    //                             coord_type x1, coord_type y1,
    //                             coord_type x2, coord_type y2,
    //                             unsigned   f1, unsigned   f2) const
    #[inline]
    fn line_clip_y(
        &self,
        ras: &mut RasterizerCellsAa,
        x1: i32,
        y1: i32,
        x2: i32,
        y2: i32,
        f1: u32,
        f2: u32,
    ) {
        let f1 = f1 & 10;
        let f2 = f2 & 10;
        if (f1 | f2) == 0 {
            // Fully visible
            ras.line(
                RasConvInt::xi(x1),
                RasConvInt::yi(y1),
                RasConvInt::xi(x2),
                RasConvInt::yi(y2),
            );
        } else {
            if f1 == f2 {
                // Invisible by Y
                return;
            }

            let mut tx1 = x1;
            let mut ty1 = y1;
            let mut tx2 = x2;
            let mut ty2 = y2;

            if (f1 & 8) != 0 {
                // y1 < clip.y1
                tx1 = x1
                    + RasConvInt::mul_div(
                        (self.m_clip_box.y1 - y1) as f64,
                        (x2 - x1) as f64,
                        (y2 - y1) as f64,
                    );
                ty1 = self.m_clip_box.y1;
            }

            if (f1 & 2) != 0 {
                // y1 > clip.y2
                tx1 = x1
                    + RasConvInt::mul_div(
                        (self.m_clip_box.y2 - y1) as f64,
                        (x2 - x1) as f64,
                        (y2 - y1) as f64,
                    );
                ty1 = self.m_clip_box.y2;
            }

            if (f2 & 8) != 0 {
                // y2 < clip.y1
                tx2 = x1
                    + RasConvInt::mul_div(
                        (self.m_clip_box.y1 - y1) as f64,
                        (x2 - x1) as f64,
                        (y2 - y1) as f64,
                    );
                ty2 = self.m_clip_box.y1;
            }

            if (f2 & 2) != 0 {
                // y2 > clip.y2
                tx2 = x1
                    + RasConvInt::mul_div(
                        (self.m_clip_box.y2 - y1) as f64,
                        (x2 - x1) as f64,
                        (y2 - y1) as f64,
                    );
                ty2 = self.m_clip_box.y2;
            }
            ras.line(
                RasConvInt::xi(tx1),
                RasConvInt::yi(ty1),
                RasConvInt::xi(tx2),
                RasConvInt::yi(ty2),
            );
        }
    }

    // agg_rasterizer_sl_clip.h:203-296
    // template<class Rasterizer>
    // void line_to(Rasterizer& ras, coord_type x2, coord_type y2)
    pub fn line_to(&mut self, ras: &mut RasterizerCellsAa, x2: i32, y2: i32) {
        if self.m_clipping {
            let f2 = clipping_flags(x2, y2, &self.m_clip_box);

            if (self.m_f1 & 10) == (f2 & 10) && (self.m_f1 & 10) != 0 {
                // Invisible by Y
                self.m_x1 = x2;
                self.m_y1 = y2;
                self.m_f1 = f2;
                return;
            }

            let x1 = self.m_x1;
            let y1 = self.m_y1;
            let f1 = self.m_f1;
            let y3: i32;
            let y4: i32;
            let f3: u32;
            let f4: u32;

            match ((f1 & 5) << 1) | (f2 & 5) {
                0 => {
                    // Visible by X
                    self.line_clip_y(ras, x1, y1, x2, y2, f1, f2);
                }

                1 => {
                    // x2 > clip.x2
                    y3 = y1
                        + RasConvInt::mul_div(
                            (self.m_clip_box.x2 - x1) as f64,
                            (y2 - y1) as f64,
                            (x2 - x1) as f64,
                        );
                    f3 = clipping_flags_y(y3, &self.m_clip_box);
                    self.line_clip_y(ras, x1, y1, self.m_clip_box.x2, y3, f1, f3);
                    self.line_clip_y(
                        ras,
                        self.m_clip_box.x2,
                        y3,
                        self.m_clip_box.x2,
                        y2,
                        f3,
                        f2,
                    );
                }

                2 => {
                    // x1 > clip.x2
                    y3 = y1
                        + RasConvInt::mul_div(
                            (self.m_clip_box.x2 - x1) as f64,
                            (y2 - y1) as f64,
                            (x2 - x1) as f64,
                        );
                    f3 = clipping_flags_y(y3, &self.m_clip_box);
                    self.line_clip_y(
                        ras,
                        self.m_clip_box.x2,
                        y1,
                        self.m_clip_box.x2,
                        y3,
                        f1,
                        f3,
                    );
                    self.line_clip_y(ras, self.m_clip_box.x2, y3, x2, y2, f3, f2);
                }

                3 => {
                    // x1 > clip.x2 && x2 > clip.x2
                    self.line_clip_y(
                        ras,
                        self.m_clip_box.x2,
                        y1,
                        self.m_clip_box.x2,
                        y2,
                        f1,
                        f2,
                    );
                }

                4 => {
                    // x2 < clip.x1
                    y3 = y1
                        + RasConvInt::mul_div(
                            (self.m_clip_box.x1 - x1) as f64,
                            (y2 - y1) as f64,
                            (x2 - x1) as f64,
                        );
                    f3 = clipping_flags_y(y3, &self.m_clip_box);
                    self.line_clip_y(ras, x1, y1, self.m_clip_box.x1, y3, f1, f3);
                    self.line_clip_y(
                        ras,
                        self.m_clip_box.x1,
                        y3,
                        self.m_clip_box.x1,
                        y2,
                        f3,
                        f2,
                    );
                }

                6 => {
                    // x1 > clip.x2 && x2 < clip.x1
                    y3 = y1
                        + RasConvInt::mul_div(
                            (self.m_clip_box.x2 - x1) as f64,
                            (y2 - y1) as f64,
                            (x2 - x1) as f64,
                        );
                    y4 = y1
                        + RasConvInt::mul_div(
                            (self.m_clip_box.x1 - x1) as f64,
                            (y2 - y1) as f64,
                            (x2 - x1) as f64,
                        );
                    f3 = clipping_flags_y(y3, &self.m_clip_box);
                    f4 = clipping_flags_y(y4, &self.m_clip_box);
                    self.line_clip_y(
                        ras,
                        self.m_clip_box.x2,
                        y1,
                        self.m_clip_box.x2,
                        y3,
                        f1,
                        f3,
                    );
                    self.line_clip_y(
                        ras,
                        self.m_clip_box.x2,
                        y3,
                        self.m_clip_box.x1,
                        y4,
                        f3,
                        f4,
                    );
                    self.line_clip_y(
                        ras,
                        self.m_clip_box.x1,
                        y4,
                        self.m_clip_box.x1,
                        y2,
                        f4,
                        f2,
                    );
                }

                8 => {
                    // x1 < clip.x1
                    y3 = y1
                        + RasConvInt::mul_div(
                            (self.m_clip_box.x1 - x1) as f64,
                            (y2 - y1) as f64,
                            (x2 - x1) as f64,
                        );
                    f3 = clipping_flags_y(y3, &self.m_clip_box);
                    self.line_clip_y(
                        ras,
                        self.m_clip_box.x1,
                        y1,
                        self.m_clip_box.x1,
                        y3,
                        f1,
                        f3,
                    );
                    self.line_clip_y(ras, self.m_clip_box.x1, y3, x2, y2, f3, f2);
                }

                9 => {
                    // x1 < clip.x1 && x2 > clip.x2
                    y3 = y1
                        + RasConvInt::mul_div(
                            (self.m_clip_box.x1 - x1) as f64,
                            (y2 - y1) as f64,
                            (x2 - x1) as f64,
                        );
                    y4 = y1
                        + RasConvInt::mul_div(
                            (self.m_clip_box.x2 - x1) as f64,
                            (y2 - y1) as f64,
                            (x2 - x1) as f64,
                        );
                    f3 = clipping_flags_y(y3, &self.m_clip_box);
                    f4 = clipping_flags_y(y4, &self.m_clip_box);
                    self.line_clip_y(
                        ras,
                        self.m_clip_box.x1,
                        y1,
                        self.m_clip_box.x1,
                        y3,
                        f1,
                        f3,
                    );
                    self.line_clip_y(
                        ras,
                        self.m_clip_box.x1,
                        y3,
                        self.m_clip_box.x2,
                        y4,
                        f3,
                        f4,
                    );
                    self.line_clip_y(
                        ras,
                        self.m_clip_box.x2,
                        y4,
                        self.m_clip_box.x2,
                        y2,
                        f4,
                        f2,
                    );
                }

                12 => {
                    // x1 < clip.x1 && x2 < clip.x1
                    self.line_clip_y(
                        ras,
                        self.m_clip_box.x1,
                        y1,
                        self.m_clip_box.x1,
                        y2,
                        f1,
                        f2,
                    );
                }

                _ => {}
            }
            self.m_f1 = f2;
        } else {
            ras.line(
                RasConvInt::xi(self.m_x1),
                RasConvInt::yi(self.m_y1),
                RasConvInt::xi(x2),
                RasConvInt::yi(y2),
            );
        }
        self.m_x1 = x2;
        self.m_y1 = y2;
    }
}
