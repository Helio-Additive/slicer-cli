//! Faithful port of the vendored AGG header
//! `src/agg/agg_rasterizer_scanline_aa.h` (Anti-Grain Geometry 2.4, as
//! bundled with BambuStudio) — `rasterizer_scanline_aa<>` with the default
//! template parameters (`Clip = rasterizer_sl_clip_int`,
//! `Scanline = scanline_p8` at the use sites).
//!
//! C++ Reference:
//! - agg/agg_rasterizer_scanline_aa.h
//! - agg/agg_rasterizer_cells_aa.h (scanline_hit_test, used by hit_test)
//!
//! `sweep_scanline` is a template over the scanline container in C++; the
//! Rust port makes it generic over the `Scanline` trait below (implemented by
//! `ScanlineP8` and `ScanlineHitTest`), mirroring the C++ duck typing.

use super::basics::{
    is_close, is_move_to, is_stop, is_vertex, uround, FillingRule, POLY_SUBPIXEL_SHIFT,
};
use super::gamma_functions::GammaFunction;
use super::path_storage::VertexSource;
use super::rasterizer_cells_aa::RasterizerCellsAa;
use super::rasterizer_sl_clip::{RasConvInt, RasterizerSlClipInt};
use super::scanline_p::ScanlineP8;

/// The scanline-container interface consumed by `sweep_scanline`
/// (duck-typed in C++; see agg_rasterizer_scanline_aa.h:203-258 and the
/// `scanline_hit_test` consumer in agg_rasterizer_cells_aa.h:714-736).
pub trait Scanline {
    fn reset_spans(&mut self);
    fn add_cell(&mut self, x: i32, cover: u32);
    fn add_span(&mut self, x: i32, len: u32, cover: u32);
    fn finalize(&mut self, y: i32);
    fn num_spans(&self) -> u32;
}

impl Scanline for ScanlineP8 {
    fn reset_spans(&mut self) {
        ScanlineP8::reset_spans(self)
    }
    fn add_cell(&mut self, x: i32, cover: u32) {
        ScanlineP8::add_cell(self, x, cover)
    }
    fn add_span(&mut self, x: i32, len: u32, cover: u32) {
        ScanlineP8::add_span(self, x, len, cover)
    }
    fn finalize(&mut self, y: i32) {
        ScanlineP8::finalize(self, y)
    }
    fn num_spans(&self) -> u32 {
        ScanlineP8::num_spans(self)
    }
}

// agg_rasterizer_scanline_aa.h:73-79  enum status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Status {
    StatusInitial,
    #[allow(dead_code)]
    StatusMoveTo,
    StatusLineTo,
    StatusClosed,
}

// agg_rasterizer_scanline_aa.h:86-93  enum aa_scale_e
pub const AA_SHIFT: i32 = 8;
pub const AA_SCALE: i32 = 1 << AA_SHIFT;
pub const AA_MASK: i32 = AA_SCALE - 1;
pub const AA_SCALE2: i32 = AA_SCALE * 2;
pub const AA_MASK2: i32 = AA_SCALE2 - 1;

// agg_rasterizer_scanline_aa.h:71
// template<class Clip=rasterizer_sl_clip_int> class rasterizer_scanline_aa
pub struct RasterizerScanlineAa {
    // agg_rasterizer_scanline_aa.h:272  rasterizer_cells_aa<cell_aa> m_outline;
    m_outline: RasterizerCellsAa,
    // agg_rasterizer_scanline_aa.h:273  clip_type m_clipper;
    m_clipper: RasterizerSlClipInt,
    // agg_rasterizer_scanline_aa.h:274  int m_gamma[aa_scale];
    m_gamma: [i32; AA_SCALE as usize],
    // agg_rasterizer_scanline_aa.h:275  filling_rule_e m_filling_rule;
    m_filling_rule: FillingRule,
    // agg_rasterizer_scanline_aa.h:276  bool m_auto_close;
    m_auto_close: bool,
    // agg_rasterizer_scanline_aa.h:277-278  coord_type m_start_x; coord_type m_start_y;
    m_start_x: i32,
    m_start_y: i32,
    // agg_rasterizer_scanline_aa.h:279  unsigned m_status;
    m_status: Status,
    // agg_rasterizer_scanline_aa.h:280  int m_scan_y;
    m_scan_y: i32,
}

impl Default for RasterizerScanlineAa {
    fn default() -> Self {
        Self::new()
    }
}

impl RasterizerScanlineAa {
    // agg_rasterizer_scanline_aa.h:96-107
    // rasterizer_scanline_aa() :
    //     m_outline(),
    //     m_clipper(),
    //     m_filling_rule(fill_non_zero),
    //     m_auto_close(true),
    //     m_start_x(0),
    //     m_start_y(0),
    //     m_status(status_initial)
    // {
    //     int i;
    //     for(i = 0; i < aa_scale; i++) m_gamma[i] = i;
    // }
    pub fn new() -> Self {
        let mut gamma = [0i32; AA_SCALE as usize];
        for (i, g) in gamma.iter_mut().enumerate() {
            *g = i as i32;
        }
        Self {
            m_outline: RasterizerCellsAa::new(),
            m_clipper: RasterizerSlClipInt::new(),
            m_gamma: gamma,
            m_filling_rule: FillingRule::FillNonZero,
            m_auto_close: true,
            m_start_x: 0,
            m_start_y: 0,
            m_status: Status::StatusInitial,
            m_scan_y: 0,
        }
    }

    // agg_rasterizer_scanline_aa.h:110-121
    // template<class GammaF>
    // rasterizer_scanline_aa(const GammaF& gamma_function) : ...
    // { gamma(gamma_function); }
    pub fn new_with_gamma<F: GammaFunction>(gamma_function: &F) -> Self {
        let mut rast = Self::new();
        rast.gamma(gamma_function);
        rast
    }

    // agg_rasterizer_scanline_aa.h:295-300
    // void rasterizer_scanline_aa<Clip>::reset()
    // {
    //     m_outline.reset();
    //     m_status = status_initial;
    // }
    pub fn reset(&mut self) {
        self.m_outline.reset();
        self.m_status = Status::StatusInitial;
    }

    // agg_rasterizer_scanline_aa.h:319-325
    // void rasterizer_scanline_aa<Clip>::reset_clipping()
    pub fn reset_clipping(&mut self) {
        self.reset();
        self.m_clipper.reset_clipping();
    }

    // agg_rasterizer_scanline_aa.h:310-317
    // void rasterizer_scanline_aa<Clip>::clip_box(double x1, double y1, double x2, double y2)
    // {
    //     reset();
    //     m_clipper.clip_box(conv_type::upscale(x1), conv_type::upscale(y1),
    //                        conv_type::upscale(x2), conv_type::upscale(y2));
    // }
    pub fn clip_box(&mut self, x1: f64, y1: f64, x2: f64, y2: f64) {
        self.reset();
        self.m_clipper.clip_box(
            RasConvInt::upscale(x1),
            RasConvInt::upscale(y1),
            RasConvInt::upscale(x2),
            RasConvInt::upscale(y2),
        );
    }

    // agg_rasterizer_scanline_aa.h:303-307
    // void rasterizer_scanline_aa<Clip>::filling_rule(filling_rule_e filling_rule)
    pub fn filling_rule(&mut self, filling_rule: FillingRule) {
        self.m_filling_rule = filling_rule;
    }

    // agg_rasterizer_scanline_aa.h:128  void auto_close(bool flag) { m_auto_close = flag; }
    pub fn auto_close(&mut self, flag: bool) {
        self.m_auto_close = flag;
    }

    // agg_rasterizer_scanline_aa.h:131-138
    // template<class GammaF> void gamma(const GammaF& gamma_function)
    // {
    //     int i;
    //     for(i = 0; i < aa_scale; i++)
    //     {
    //         m_gamma[i] = uround(gamma_function(double(i) / aa_mask) * aa_mask);
    //     }
    // }
    pub fn gamma<F: GammaFunction>(&mut self, gamma_function: &F) {
        for i in 0..AA_SCALE as usize {
            self.m_gamma[i] =
                uround(gamma_function.call(i as f64 / AA_MASK as f64) * AA_MASK as f64) as i32;
        }
    }

    // agg_rasterizer_scanline_aa.h:141-144
    // unsigned apply_gamma(unsigned cover) const { return m_gamma[cover]; }
    pub fn apply_gamma(&self, cover: u32) -> u32 {
        self.m_gamma[cover as usize] as u32
    }

    // agg_rasterizer_scanline_aa.h:328-336
    // void rasterizer_scanline_aa<Clip>::close_polygon()
    // {
    //     if(m_status == status_line_to)
    //     {
    //         m_clipper.line_to(m_outline, m_start_x, m_start_y);
    //         m_status = status_closed;
    //     }
    // }
    pub fn close_polygon(&mut self) {
        if self.m_status == Status::StatusLineTo {
            self.m_clipper
                .line_to(&mut self.m_outline, self.m_start_x, self.m_start_y);
            self.m_status = Status::StatusClosed;
        }
    }

    // agg_rasterizer_scanline_aa.h:339-347
    // void rasterizer_scanline_aa<Clip>::move_to(int x, int y)
    pub fn move_to(&mut self, x: i32, y: i32) {
        if self.m_outline.sorted() {
            self.reset();
        }
        if self.m_auto_close {
            self.close_polygon();
        }
        self.m_start_x = RasConvInt::downscale(x);
        self.m_start_y = RasConvInt::downscale(y);
        self.m_clipper.move_to(self.m_start_x, self.m_start_y);
        self.m_status = Status::StatusMoveTo;
    }

    // agg_rasterizer_scanline_aa.h:350-357
    // void rasterizer_scanline_aa<Clip>::line_to(int x, int y)
    pub fn line_to(&mut self, x: i32, y: i32) {
        self.m_clipper.line_to(
            &mut self.m_outline,
            RasConvInt::downscale(x),
            RasConvInt::downscale(y),
        );
        self.m_status = Status::StatusLineTo;
    }

    // agg_rasterizer_scanline_aa.h:360-368
    // void rasterizer_scanline_aa<Clip>::move_to_d(double x, double y)
    // {
    //     if(m_outline.sorted()) reset();
    //     if(m_auto_close) close_polygon();
    //     m_clipper.move_to(m_start_x = conv_type::upscale(x),
    //                       m_start_y = conv_type::upscale(y));
    //     m_status = status_move_to;
    // }
    pub fn move_to_d(&mut self, x: f64, y: f64) {
        if self.m_outline.sorted() {
            self.reset();
        }
        if self.m_auto_close {
            self.close_polygon();
        }
        self.m_start_x = RasConvInt::upscale(x);
        self.m_start_y = RasConvInt::upscale(y);
        self.m_clipper.move_to(self.m_start_x, self.m_start_y);
        self.m_status = Status::StatusMoveTo;
    }

    // agg_rasterizer_scanline_aa.h:371-378
    // void rasterizer_scanline_aa<Clip>::line_to_d(double x, double y)
    // {
    //     m_clipper.line_to(m_outline,
    //                       conv_type::upscale(x),
    //                       conv_type::upscale(y));
    //     m_status = status_line_to;
    // }
    pub fn line_to_d(&mut self, x: f64, y: f64) {
        self.m_clipper.line_to(
            &mut self.m_outline,
            RasConvInt::upscale(x),
            RasConvInt::upscale(y),
        );
        self.m_status = Status::StatusLineTo;
    }

    // agg_rasterizer_scanline_aa.h:381-398
    // void rasterizer_scanline_aa<Clip>::add_vertex(double x, double y, unsigned cmd)
    // {
    //     if(is_move_to(cmd))
    //     {
    //         move_to_d(x, y);
    //     }
    //     else
    //     if(is_vertex(cmd))
    //     {
    //         line_to_d(x, y);
    //     }
    //     else
    //     if(is_close(cmd))
    //     {
    //         close_polygon();
    //     }
    // }
    pub fn add_vertex(&mut self, x: f64, y: f64, cmd: u32) {
        if is_move_to(cmd) {
            self.move_to_d(x, y);
        } else if is_vertex(cmd) {
            self.line_to_d(x, y);
        } else if is_close(cmd) {
            self.close_polygon();
        }
    }

    // agg_rasterizer_scanline_aa.h:401-410
    // void rasterizer_scanline_aa<Clip>::edge(int x1, int y1, int x2, int y2)
    pub fn edge(&mut self, x1: i32, y1: i32, x2: i32, y2: i32) {
        if self.m_outline.sorted() {
            self.reset();
        }
        self.m_clipper
            .move_to(RasConvInt::downscale(x1), RasConvInt::downscale(y1));
        self.m_clipper.line_to(
            &mut self.m_outline,
            RasConvInt::downscale(x2),
            RasConvInt::downscale(y2),
        );
        self.m_status = Status::StatusMoveTo;
    }

    // agg_rasterizer_scanline_aa.h:413-423
    // void rasterizer_scanline_aa<Clip>::edge_d(double x1, double y1, double x2, double y2)
    pub fn edge_d(&mut self, x1: f64, y1: f64, x2: f64, y2: f64) {
        if self.m_outline.sorted() {
            self.reset();
        }
        self.m_clipper
            .move_to(RasConvInt::upscale(x1), RasConvInt::upscale(y1));
        self.m_clipper.line_to(
            &mut self.m_outline,
            RasConvInt::upscale(x2),
            RasConvInt::upscale(y2),
        );
        self.m_status = Status::StatusMoveTo;
    }

    // agg_rasterizer_scanline_aa.h:158-171
    // template<class VertexSource>
    // void add_path(VertexSource &&vs, unsigned path_id=0)
    // {
    //     double x;
    //     double y;
    //
    //     unsigned cmd;
    //     vs.rewind(path_id);
    //     if(m_outline.sorted()) reset();
    //     while(!is_stop(cmd = vs.vertex(&x, &y)))
    //     {
    //         add_vertex(x, y, cmd);
    //     }
    // }
    pub fn add_path<VS: VertexSource>(&mut self, vs: &mut VS, path_id: u32) {
        let mut x: f64 = 0.0;
        let mut y: f64 = 0.0;

        let mut cmd: u32;
        vs.rewind(path_id);
        if self.m_outline.sorted() {
            self.reset();
        }
        loop {
            cmd = vs.vertex(&mut x, &mut y);
            if is_stop(cmd) {
                break;
            }
            self.add_vertex(x, y, cmd);
        }
    }

    // agg_rasterizer_scanline_aa.h:174-177
    pub fn min_x(&self) -> i32 {
        self.m_outline.min_x()
    }
    pub fn min_y(&self) -> i32 {
        self.m_outline.min_y()
    }
    pub fn max_x(&self) -> i32 {
        self.m_outline.max_x()
    }
    pub fn max_y(&self) -> i32 {
        self.m_outline.max_y()
    }

    // agg_rasterizer_scanline_aa.h:426-431
    // void rasterizer_scanline_aa<Clip>::sort()
    // {
    //     if(m_auto_close) close_polygon();
    //     m_outline.sort_cells();
    // }
    pub fn sort(&mut self) {
        if self.m_auto_close {
            self.close_polygon();
        }
        self.m_outline.sort_cells();
    }

    // agg_rasterizer_scanline_aa.h:434-445
    // AGG_INLINE bool rasterizer_scanline_aa<Clip>::rewind_scanlines()
    // {
    //     if(m_auto_close) close_polygon();
    //     m_outline.sort_cells();
    //     if(m_outline.total_cells() == 0)
    //     {
    //         return false;
    //     }
    //     m_scan_y = m_outline.min_y();
    //     return true;
    // }
    pub fn rewind_scanlines(&mut self) -> bool {
        if self.m_auto_close {
            self.close_polygon();
        }
        self.m_outline.sort_cells();
        if self.m_outline.total_cells() == 0 {
            return false;
        }
        self.m_scan_y = self.m_outline.min_y();
        true
    }

    // agg_rasterizer_scanline_aa.h:449-462
    // AGG_INLINE bool rasterizer_scanline_aa<Clip>::navigate_scanline(int y)
    pub fn navigate_scanline(&mut self, y: i32) -> bool {
        if self.m_auto_close {
            self.close_polygon();
        }
        self.m_outline.sort_cells();
        if self.m_outline.total_cells() == 0
            || y < self.m_outline.min_y()
            || y > self.m_outline.max_y()
        {
            return false;
        }
        self.m_scan_y = y;
        true
    }

    // agg_rasterizer_scanline_aa.h:185-200
    // AGG_INLINE unsigned calculate_alpha(int area) const
    // {
    //     int cover = area >> (poly_subpixel_shift*2 + 1 - aa_shift);
    //
    //     if(cover < 0) cover = -cover;
    //     if(m_filling_rule == fill_even_odd)
    //     {
    //         cover &= aa_mask2;
    //         if(cover > aa_scale)
    //         {
    //             cover = aa_scale2 - cover;
    //         }
    //     }
    //     if(cover > aa_mask) cover = aa_mask;
    //     return m_gamma[cover];
    // }
    #[inline]
    pub fn calculate_alpha(&self, area: i32) -> u32 {
        let mut cover: i32 = area >> (POLY_SUBPIXEL_SHIFT * 2 + 1 - AA_SHIFT);

        if cover < 0 {
            cover = -cover;
        }
        if self.m_filling_rule == FillingRule::FillEvenOdd {
            cover &= AA_MASK2;
            if cover > AA_SCALE {
                cover = AA_SCALE2 - cover;
            }
        }
        if cover > AA_MASK {
            cover = AA_MASK;
        }
        self.m_gamma[cover as usize] as u32
    }

    // agg_rasterizer_scanline_aa.h:203-258
    // template<class Scanline> bool sweep_scanline(Scanline& sl)
    pub fn sweep_scanline<S: Scanline>(&mut self, sl: &mut S) -> bool {
        loop {
            if self.m_scan_y > self.m_outline.max_y() {
                return false;
            }
            sl.reset_spans();
            let mut num_cells: u32 = self.m_outline.scanline_num_cells(self.m_scan_y as u32);
            // const cell_aa* const* cells = m_outline.scanline_cells(m_scan_y);
            // (cell indices; `cells_pos` mirrors the C++ `cells` pointer)
            let mut cells_pos: usize = 0;
            let mut cover: i32 = 0;

            while num_cells != 0 {
                let cells = self.m_outline.scanline_cells(self.m_scan_y as u32);
                // const cell_aa* cur_cell = *cells;
                let mut cur_cell = self.m_outline.cell(cells[cells_pos]);
                let mut x: i32 = cur_cell.x;
                let mut area: i32 = cur_cell.area;
                let alpha: u32;

                cover += cur_cell.cover;

                // accumulate all cells with the same X
                loop {
                    num_cells -= 1;
                    if num_cells == 0 {
                        break;
                    }
                    // cur_cell = *++cells;
                    cells_pos += 1;
                    cur_cell = self.m_outline.cell(cells[cells_pos]);
                    if cur_cell.x != x {
                        break;
                    }
                    area += cur_cell.area;
                    cover += cur_cell.cover;
                }

                let cur_cell_x = cur_cell.x;

                if area != 0 {
                    alpha =
                        self.calculate_alpha((cover << (POLY_SUBPIXEL_SHIFT + 1)) - area);
                    if alpha != 0 {
                        sl.add_cell(x, alpha);
                    }
                    x += 1;
                }

                if num_cells != 0 && cur_cell_x > x {
                    let alpha = self.calculate_alpha(cover << (POLY_SUBPIXEL_SHIFT + 1));
                    if alpha != 0 {
                        sl.add_span(x, (cur_cell_x - x) as u32, alpha);
                    }
                }
            }

            if sl.num_spans() != 0 {
                break;
            }
            self.m_scan_y += 1;
        }

        sl.finalize(self.m_scan_y);
        self.m_scan_y += 1;
        true
    }

    // agg_rasterizer_cells_aa.h:714-736  class scanline_hit_test
    // agg_rasterizer_scanline_aa.h:465-472
    // bool rasterizer_scanline_aa<Clip>::hit_test(int tx, int ty)
    // {
    //     if(!navigate_scanline(ty)) return false;
    //     scanline_hit_test sl(tx);
    //     sweep_scanline(sl);
    //     return sl.hit();
    // }
    pub fn hit_test(&mut self, tx: i32, ty: i32) -> bool {
        if !self.navigate_scanline(ty) {
            return false;
        }
        let mut sl = ScanlineHitTest::new(tx);
        self.sweep_scanline(&mut sl);
        sl.hit()
    }
}

// agg_rasterizer_cells_aa.h:715-736  class scanline_hit_test
pub struct ScanlineHitTest {
    // agg_rasterizer_cells_aa.h:734-735  int m_x; bool m_hit;
    m_x: i32,
    m_hit: bool,
}

impl ScanlineHitTest {
    // agg_rasterizer_cells_aa.h:718  scanline_hit_test(int x) : m_x(x), m_hit(false) {}
    pub fn new(x: i32) -> Self {
        Self { m_x: x, m_hit: false }
    }

    // agg_rasterizer_cells_aa.h:731  bool hit() const { return m_hit; }
    pub fn hit(&self) -> bool {
        self.m_hit
    }
}

impl Scanline for ScanlineHitTest {
    // agg_rasterizer_cells_aa.h:720  void reset_spans() {}
    fn reset_spans(&mut self) {}

    // agg_rasterizer_cells_aa.h:722-725
    // void add_cell(int x, int)
    // {
    //     if(m_x == x) m_hit = true;
    // }
    fn add_cell(&mut self, x: i32, _cover: u32) {
        if self.m_x == x {
            self.m_hit = true;
        }
    }

    // agg_rasterizer_cells_aa.h:726-729
    // void add_span(int x, int len, int)
    // {
    //     if(m_x >= x && m_x < x+len) m_hit = true;
    // }
    fn add_span(&mut self, x: i32, len: u32, _cover: u32) {
        if self.m_x >= x && self.m_x < x + len as i32 {
            self.m_hit = true;
        }
    }

    // agg_rasterizer_cells_aa.h:721  void finalize(int) {}
    fn finalize(&mut self, _y: i32) {}

    // agg_rasterizer_cells_aa.h:730  unsigned num_spans() const { return 1; }
    fn num_spans(&self) -> u32 {
        1
    }
}
