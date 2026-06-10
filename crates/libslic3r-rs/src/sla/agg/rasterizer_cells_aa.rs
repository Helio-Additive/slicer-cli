//! Faithful port of the vendored AGG header `src/agg/agg_rasterizer_cells_aa.h`
//! (Anti-Grain Geometry 2.4, as bundled with BambuStudio) plus the `cell_aa`
//! struct from `agg_rasterizer_scanline_aa_nogamma.h:45-66`.
//!
//! C++ Reference:
//! - agg/agg_rasterizer_cells_aa.h
//! - agg/agg_rasterizer_scanline_aa_nogamma.h (cell_aa)
//!
//! The C++ class is a template over the cell type; only the `cell_aa`
//! instantiation (no style) is used. Cells are stored in 4096-cell blocks in
//! C++; the blocks are contiguous append-only storage, so they collapse to a
//! flat `Vec<CellAa>` indexed by the running cell count. The block COUNTERS
//! (`m_num_blocks`, `m_curr_block`) are kept because `add_curr_cell` drops
//! cells once `cell_block_limit` blocks have been allocated — that observable
//! cap is reproduced exactly. Sorted cell pointers (`cell_type**`) become
//! `u32` indices into the flat cell array.

use super::basics::{POLY_SUBPIXEL_MASK, POLY_SUBPIXEL_SCALE, POLY_SUBPIXEL_SHIFT};

// agg_rasterizer_scanline_aa_nogamma.h:42-44
// A pixel cell. There're no constructors defined and it was done
// intentionally in order to avoid extra overhead when allocating an
// array of cells.
// agg_rasterizer_scanline_aa_nogamma.h:45-66  struct cell_aa
#[derive(Debug, Clone, Copy)]
pub struct CellAa {
    pub x: i32,
    pub y: i32,
    pub cover: i32,
    pub area: i32,
}

impl CellAa {
    // agg_rasterizer_scanline_aa_nogamma.h:52-58
    // void initial()
    // {
    //     x = std::numeric_limits<int>::max();
    //     y = std::numeric_limits<int>::max();
    //     cover = 0;
    //     area  = 0;
    // }
    pub fn initial(&mut self) {
        self.x = i32::MAX;
        self.y = i32::MAX;
        self.cover = 0;
        self.area = 0;
    }

    // agg_rasterizer_scanline_aa_nogamma.h:60  void style(const cell_aa&) {}
    #[inline]
    pub fn style(&mut self, _style_cell: &CellAa) {}

    // agg_rasterizer_scanline_aa_nogamma.h:62-65
    // int not_equal(int ex, int ey, const cell_aa&) const
    // {
    //     return ((unsigned)ex - (unsigned)x) | ((unsigned)ey - (unsigned)y);
    // }
    #[inline]
    pub fn not_equal(&self, ex: i32, ey: i32, _style_cell: &CellAa) -> u32 {
        ((ex as u32).wrapping_sub(self.x as u32)) | ((ey as u32).wrapping_sub(self.y as u32))
    }
}

impl Default for CellAa {
    fn default() -> Self {
        // (C++ cells are allocated uninitialized; the default is irrelevant to
        // behavior — every cell is fully overwritten before use)
        Self {
            x: 0,
            y: 0,
            cover: 0,
            area: 0,
        }
    }
}

// agg_rasterizer_cells_aa.h:47-54  enum cell_block_scale_e
const CELL_BLOCK_SHIFT: u32 = 12;
const CELL_BLOCK_SIZE: u32 = 1 << CELL_BLOCK_SHIFT;
const CELL_BLOCK_MASK: u32 = CELL_BLOCK_SIZE - 1;
#[allow(dead_code)]
const CELL_BLOCK_POOL: u32 = 256;
const CELL_BLOCK_LIMIT: u32 = 1024;

// agg_rasterizer_cells_aa.h:56-60  struct sorted_y
#[derive(Debug, Clone, Copy, Default)]
struct SortedY {
    start: u32,
    num: u32,
}

// agg_rasterizer_cells_aa.h:45  template<class Cell> class rasterizer_cells_aa
#[derive(Debug)]
pub struct RasterizerCellsAa {
    // agg_rasterizer_cells_aa.h:107  unsigned m_num_blocks;
    m_num_blocks: u32,
    // agg_rasterizer_cells_aa.h:109  unsigned m_curr_block;
    m_curr_block: u32,
    // agg_rasterizer_cells_aa.h:110  unsigned m_num_cells;
    m_num_cells: u32,
    // agg_rasterizer_cells_aa.h:111-112  cell_type** m_cells; cell_type* m_curr_cell_ptr;
    // (flat storage; the cell at index m_num_cells is the next write slot)
    m_cells: Vec<CellAa>,
    // agg_rasterizer_cells_aa.h:113  pod_vector<cell_type*> m_sorted_cells;
    m_sorted_cells: Vec<u32>,
    // agg_rasterizer_cells_aa.h:114  pod_vector<sorted_y> m_sorted_y;
    m_sorted_y: Vec<SortedY>,
    // agg_rasterizer_cells_aa.h:115  cell_type m_curr_cell;
    m_curr_cell: CellAa,
    // agg_rasterizer_cells_aa.h:116  cell_type m_style_cell;
    m_style_cell: CellAa,
    // agg_rasterizer_cells_aa.h:117-120  int m_min_x, m_min_y, m_max_x, m_max_y;
    m_min_x: i32,
    m_min_y: i32,
    m_max_x: i32,
    m_max_y: i32,
    // agg_rasterizer_cells_aa.h:121  bool m_sorted;
    m_sorted: bool,
}

impl Default for RasterizerCellsAa {
    fn default() -> Self {
        Self::new()
    }
}

impl RasterizerCellsAa {
    // agg_rasterizer_cells_aa.h:144-162
    // rasterizer_cells_aa<Cell>::rasterizer_cells_aa() :
    //     m_num_blocks(0), m_max_blocks(0), m_curr_block(0), m_num_cells(0),
    //     m_cells(0), m_curr_cell_ptr(0), m_sorted_cells(), m_sorted_y(),
    //     m_min_x(INT_MAX), m_min_y(INT_MAX), m_max_x(INT_MIN), m_max_y(INT_MIN),
    //     m_sorted(false)
    // {
    //     m_style_cell.initial();
    //     m_curr_cell.initial();
    // }
    pub fn new() -> Self {
        let mut style_cell = CellAa::default();
        let mut curr_cell = CellAa::default();
        style_cell.initial();
        curr_cell.initial();
        Self {
            m_num_blocks: 0,
            m_curr_block: 0,
            m_num_cells: 0,
            m_cells: Vec::new(),
            m_sorted_cells: Vec::new(),
            m_sorted_y: Vec::new(),
            m_curr_cell: curr_cell,
            m_style_cell: style_cell,
            m_min_x: i32::MAX,
            m_min_y: i32::MAX,
            m_max_x: i32::MIN,
            m_max_y: i32::MIN,
            m_sorted: false,
        }
    }

    // agg_rasterizer_cells_aa.h:165-177
    // void rasterizer_cells_aa<Cell>::reset()
    pub fn reset(&mut self) {
        self.m_num_cells = 0;
        self.m_curr_block = 0;
        self.m_curr_cell.initial();
        self.m_style_cell.initial();
        self.m_sorted = false;
        self.m_min_x = i32::MAX;
        self.m_min_y = i32::MAX;
        self.m_max_x = i32::MIN;
        self.m_max_y = i32::MIN;
    }

    // agg_rasterizer_cells_aa.h:73-76
    pub fn min_x(&self) -> i32 {
        self.m_min_x
    }
    pub fn min_y(&self) -> i32 {
        self.m_min_y
    }
    pub fn max_x(&self) -> i32 {
        self.m_max_x
    }
    pub fn max_y(&self) -> i32 {
        self.m_max_y
    }

    // agg_rasterizer_cells_aa.h:80-83
    // unsigned total_cells() const { return m_num_cells; }
    pub fn total_cells(&self) -> u32 {
        self.m_num_cells
    }

    // agg_rasterizer_cells_aa.h:85-88
    // unsigned scanline_num_cells(unsigned y) const
    // { return m_sorted_y[y - m_min_y].num; }
    pub fn scanline_num_cells(&self, y: u32) -> u32 {
        self.m_sorted_y[(y as i32 - self.m_min_y) as usize].num
    }

    // agg_rasterizer_cells_aa.h:90-93
    // const cell_type* const* scanline_cells(unsigned y) const
    // { return m_sorted_cells.data() + m_sorted_y[y - m_min_y].start; }
    // (returns the sorted cell-index slice for the scanline; pair it with
    // `cell(idx)` to dereference)
    pub fn scanline_cells(&self, y: u32) -> &[u32] {
        let sy = &self.m_sorted_y[(y as i32 - self.m_min_y) as usize];
        &self.m_sorted_cells[sy.start as usize..(sy.start + sy.num) as usize]
    }

    /// Dereference a sorted cell index (C++ dereferences the `cell_type*`).
    #[inline]
    pub fn cell(&self, idx: u32) -> &CellAa {
        &self.m_cells[idx as usize]
    }

    // agg_rasterizer_cells_aa.h:95  bool sorted() const { return m_sorted; }
    pub fn sorted(&self) -> bool {
        self.m_sorted
    }

    // agg_rasterizer_cells_aa.h:180-193
    // AGG_INLINE void rasterizer_cells_aa<Cell>::add_curr_cell()
    // {
    //     if(m_curr_cell.area | m_curr_cell.cover)
    //     {
    //         if((m_num_cells & cell_block_mask) == 0)
    //         {
    //             if(m_num_blocks >= cell_block_limit) return;
    //             allocate_block();
    //         }
    //         *m_curr_cell_ptr++ = m_curr_cell;
    //         ++m_num_cells;
    //     }
    // }
    #[inline]
    fn add_curr_cell(&mut self) {
        if (self.m_curr_cell.area | self.m_curr_cell.cover) != 0 {
            if (self.m_num_cells & CELL_BLOCK_MASK) == 0 {
                if self.m_num_blocks >= CELL_BLOCK_LIMIT {
                    return;
                }
                self.allocate_block();
            }
            self.m_cells[self.m_num_cells as usize] = self.m_curr_cell;
            self.m_num_cells += 1;
        }
    }

    // agg_rasterizer_cells_aa.h:196-208
    // AGG_INLINE void rasterizer_cells_aa<Cell>::set_curr_cell(int x, int y)
    // {
    //     if(m_curr_cell.not_equal(x, y, m_style_cell))
    //     {
    //         add_curr_cell();
    //         m_curr_cell.style(m_style_cell);
    //         m_curr_cell.x     = x;
    //         m_curr_cell.y     = y;
    //         m_curr_cell.cover = 0;
    //         m_curr_cell.area  = 0;
    //     }
    // }
    #[inline]
    fn set_curr_cell(&mut self, x: i32, y: i32) {
        if self.m_curr_cell.not_equal(x, y, &self.m_style_cell) != 0 {
            self.add_curr_cell();
            let style_cell = self.m_style_cell;
            self.m_curr_cell.style(&style_cell);
            self.m_curr_cell.x = x;
            self.m_curr_cell.y = y;
            self.m_curr_cell.cover = 0;
            self.m_curr_cell.area = 0;
        }
    }

    // agg_rasterizer_cells_aa.h:211-307
    // AGG_INLINE void rasterizer_cells_aa<Cell>::render_hline(int ey,
    //                                                         int x1, int y1,
    //                                                         int x2, int y2)
    #[inline]
    fn render_hline(&mut self, ey: i32, x1: i32, y1: i32, x2: i32, y2: i32) {
        // agg_rasterizer_cells_aa.h:216-219
        let ex1: i32 = x1 >> POLY_SUBPIXEL_SHIFT;
        let ex2: i32 = x2 >> POLY_SUBPIXEL_SHIFT;
        let fx1: i32 = x1 & POLY_SUBPIXEL_MASK;
        let fx2: i32 = x2 & POLY_SUBPIXEL_MASK;

        // agg_rasterizer_cells_aa.h:221-223
        // int delta, p, first;  long long dx;  int incr, lift, mod, rem;
        let mut delta: i32;
        let mut p: i32;
        let mut first: i32;
        let mut incr: i32;

        // agg_rasterizer_cells_aa.h:225-230
        // trivial case. Happens often
        if y1 == y2 {
            self.set_curr_cell(ex2, ey);
            return;
        }

        // agg_rasterizer_cells_aa.h:232-239
        // everything is located in a single cell.  That is easy!
        if ex1 == ex2 {
            delta = y2 - y1;
            self.m_curr_cell.cover += delta;
            self.m_curr_cell.area += (fx1 + fx2) * delta;
            return;
        }

        // agg_rasterizer_cells_aa.h:241-245
        // ok, we'll have to render a run of adjacent cells on the same
        // hline...
        p = (POLY_SUBPIXEL_SCALE - fx1) * (y2 - y1);
        first = POLY_SUBPIXEL_SCALE;
        incr = 1;

        // agg_rasterizer_cells_aa.h:247  dx = (long long)x2 - (long long)x1;
        let mut dx: i64 = x2 as i64 - x1 as i64;

        // agg_rasterizer_cells_aa.h:249-255
        if dx < 0 {
            p = fx1 * (y2 - y1);
            first = 0;
            incr = -1;
            dx = -dx;
        }

        // agg_rasterizer_cells_aa.h:257-258
        delta = (p as i64 / dx) as i32;
        let mut mod_: i32 = (p as i64 % dx) as i32;

        // agg_rasterizer_cells_aa.h:260-264
        if mod_ < 0 {
            delta -= 1;
            mod_ += dx as i32;
        }

        // agg_rasterizer_cells_aa.h:266-267
        self.m_curr_cell.cover += delta;
        self.m_curr_cell.area += (fx1 + first) * delta;

        // agg_rasterizer_cells_aa.h:269-271
        let mut ex1 = ex1 + incr;
        self.set_curr_cell(ex1, ey);
        let mut y1 = y1 + delta;

        // agg_rasterizer_cells_aa.h:273-303
        if ex1 != ex2 {
            p = POLY_SUBPIXEL_SCALE * (y2 - y1 + delta);
            let mut lift: i32 = (p as i64 / dx) as i32;
            let mut rem: i32 = (p as i64 % dx) as i32;

            // agg_rasterizer_cells_aa.h:279-283
            if rem < 0 {
                lift -= 1;
                rem += dx as i32;
            }

            // agg_rasterizer_cells_aa.h:285
            mod_ -= dx as i32;

            // agg_rasterizer_cells_aa.h:287-302
            while ex1 != ex2 {
                delta = lift;
                mod_ += rem;
                if mod_ >= 0 {
                    mod_ -= dx as i32;
                    delta += 1;
                }

                self.m_curr_cell.cover += delta;
                self.m_curr_cell.area += POLY_SUBPIXEL_SCALE * delta;
                y1 += delta;
                ex1 += incr;
                self.set_curr_cell(ex1, ey);
            }
        }
        // agg_rasterizer_cells_aa.h:304-306
        delta = y2 - y1;
        self.m_curr_cell.cover += delta;
        self.m_curr_cell.area += (fx2 + POLY_SUBPIXEL_SCALE - first) * delta;
    }

    // agg_rasterizer_cells_aa.h:310-314
    // AGG_INLINE void rasterizer_cells_aa<Cell>::style(const cell_type& style_cell)
    // { m_style_cell.style(style_cell); }
    pub fn style(&mut self, style_cell: &CellAa) {
        self.m_style_cell.style(style_cell);
    }

    // agg_rasterizer_cells_aa.h:317-466
    // void rasterizer_cells_aa<Cell>::line(int x1, int y1, int x2, int y2)
    pub fn line(&mut self, x1: i32, y1: i32, x2: i32, y2: i32) {
        // agg_rasterizer_cells_aa.h:320  enum dx_limit_e { dx_limit = 16384 << poly_subpixel_shift };
        const DX_LIMIT: i64 = (16384 << POLY_SUBPIXEL_SHIFT) as i64;

        // agg_rasterizer_cells_aa.h:322  long long dx = (long long)x2 - (long long)x1;
        let dx: i64 = x2 as i64 - x1 as i64;

        // agg_rasterizer_cells_aa.h:324-330
        // NOTE (faithful quirk): the vendored source does NOT `return` after
        // splitting; it falls through and renders the full line as well.
        if dx >= DX_LIMIT || dx <= -DX_LIMIT {
            let cx: i32 = ((x1 as i64 + x2 as i64) >> 1) as i32;
            let cy: i32 = ((y1 as i64 + y2 as i64) >> 1) as i32;
            self.line(x1, y1, cx, cy);
            self.line(cx, cy, x2, y2);
        }

        // agg_rasterizer_cells_aa.h:332  long long dy = (long long)y2 - (long long)y1;
        let mut dy: i64 = y2 as i64 - y1 as i64;
        // agg_rasterizer_cells_aa.h:333-338
        let ex1: i32 = x1 >> POLY_SUBPIXEL_SHIFT;
        let ex2: i32 = x2 >> POLY_SUBPIXEL_SHIFT;
        let mut ey1: i32 = y1 >> POLY_SUBPIXEL_SHIFT;
        let ey2: i32 = y2 >> POLY_SUBPIXEL_SHIFT;
        let fy1: i32 = y1 & POLY_SUBPIXEL_MASK;
        let fy2: i32 = y2 & POLY_SUBPIXEL_MASK;

        // agg_rasterizer_cells_aa.h:340-342
        // int x_from, x_to;  int rem, mod, lift, delta, first, incr;  long long p;
        let mut x_from: i32;
        let mut x_to: i32;
        let mut mod_: i32;
        let mut delta: i32;
        let mut first: i32;
        let mut incr: i32;
        let mut p: i64;

        // agg_rasterizer_cells_aa.h:344-351
        if ex1 < self.m_min_x {
            self.m_min_x = ex1;
        }
        if ex1 > self.m_max_x {
            self.m_max_x = ex1;
        }
        if ey1 < self.m_min_y {
            self.m_min_y = ey1;
        }
        if ey1 > self.m_max_y {
            self.m_max_y = ey1;
        }
        if ex2 < self.m_min_x {
            self.m_min_x = ex2;
        }
        if ex2 > self.m_max_x {
            self.m_max_x = ex2;
        }
        if ey2 < self.m_min_y {
            self.m_min_y = ey2;
        }
        if ey2 > self.m_max_y {
            self.m_max_y = ey2;
        }

        // agg_rasterizer_cells_aa.h:353
        self.set_curr_cell(ex1, ey1);

        // agg_rasterizer_cells_aa.h:355-360
        // everything is on a single hline
        if ey1 == ey2 {
            self.render_hline(ey1, x1, fy1, x2, fy2);
            return;
        }

        // agg_rasterizer_cells_aa.h:362-365
        // Vertical line - we have to calculate start and end cells,
        // and then - the common values of the area and coverage for
        // all cells of the line. We know exactly there's only one
        // cell, so, we don't have to call render_hline().
        incr = 1;
        // agg_rasterizer_cells_aa.h:367-405
        if dx == 0 {
            let ex: i32 = x1 >> POLY_SUBPIXEL_SHIFT;
            let two_fx: i32 = (x1 - (ex << POLY_SUBPIXEL_SHIFT)) << 1;
            let area: i32;

            first = POLY_SUBPIXEL_SCALE;
            if dy < 0 {
                first = 0;
                incr = -1;
            }

            x_from = x1;
            let _ = x_from; // (x_from is dead in this branch in C++ too)

            // render_hline(ey1, x_from, fy1, x_from, first);
            delta = first - fy1;
            self.m_curr_cell.cover += delta;
            self.m_curr_cell.area += two_fx * delta;

            ey1 += incr;
            self.set_curr_cell(ex, ey1);

            delta = first + first - POLY_SUBPIXEL_SCALE;
            area = two_fx * delta;
            while ey1 != ey2 {
                // render_hline(ey1, x_from, poly_subpixel_scale - first, x_from, first);
                self.m_curr_cell.cover = delta;
                self.m_curr_cell.area = area;
                ey1 += incr;
                self.set_curr_cell(ex, ey1);
            }
            // render_hline(ey1, x_from, poly_subpixel_scale - first, x_from, fy2);
            delta = fy2 - POLY_SUBPIXEL_SCALE + first;
            self.m_curr_cell.cover += delta;
            self.m_curr_cell.area += two_fx * delta;
            return;
        }

        // agg_rasterizer_cells_aa.h:407-409
        // ok, we have to render several hlines
        p = ((POLY_SUBPIXEL_SCALE - fy1) as i64) * dx;
        first = POLY_SUBPIXEL_SCALE;

        // agg_rasterizer_cells_aa.h:411-417
        if dy < 0 {
            p = fy1 as i64 * dx;
            first = 0;
            incr = -1;
            dy = -dy;
        }

        // agg_rasterizer_cells_aa.h:419-420
        delta = (p / dy) as i32;
        mod_ = (p % dy) as i32;

        // agg_rasterizer_cells_aa.h:422-426
        if mod_ < 0 {
            delta -= 1;
            mod_ += dy as i32;
        }

        // agg_rasterizer_cells_aa.h:428-429
        x_from = x1 + delta;
        self.render_hline(ey1, x1, fy1, x_from, first);

        // agg_rasterizer_cells_aa.h:431-432
        ey1 += incr;
        self.set_curr_cell(x_from >> POLY_SUBPIXEL_SHIFT, ey1);

        // agg_rasterizer_cells_aa.h:434-464
        if ey1 != ey2 {
            p = POLY_SUBPIXEL_SCALE as i64 * dx;
            let mut lift: i32 = (p / dy) as i32;
            let mut rem: i32 = (p % dy) as i32;

            // agg_rasterizer_cells_aa.h:440-444
            if rem < 0 {
                lift -= 1;
                rem += dy as i32;
            }
            mod_ -= dy as i32;

            while ey1 != ey2 {
                delta = lift;
                mod_ += rem;
                if mod_ >= 0 {
                    mod_ -= dy as i32;
                    delta += 1;
                }

                x_to = x_from + delta;
                self.render_hline(ey1, x_from, POLY_SUBPIXEL_SCALE - first, x_to, first);
                x_from = x_to;

                ey1 += incr;
                self.set_curr_cell(x_from >> POLY_SUBPIXEL_SHIFT, ey1);
            }
        }
        // agg_rasterizer_cells_aa.h:465
        self.render_hline(ey1, x_from, POLY_SUBPIXEL_SCALE - first, x2, fy2);
    }

    // agg_rasterizer_cells_aa.h:469-494
    // void rasterizer_cells_aa<Cell>::allocate_block()
    // {
    //     if(m_curr_block >= m_num_blocks)
    //     {
    //         ... allocate a new 4096-cell block ...
    //         m_cells[m_num_blocks++] = pod_allocator<cell_type>::allocate(cell_block_size);
    //     }
    //     m_curr_cell_ptr = m_cells[m_curr_block++];
    // }
    fn allocate_block(&mut self) {
        if self.m_curr_block >= self.m_num_blocks {
            // (the pointer-array pool growth (m_max_blocks/cell_block_pool)
            // is a memory-management detail; Vec handles it)
            self.m_num_blocks += 1;
            self.m_cells.resize(
                (self.m_num_blocks * CELL_BLOCK_SIZE) as usize,
                CellAa::default(),
            );
        }
        self.m_curr_block += 1;
        // m_curr_cell_ptr is implicit: cells are appended at m_num_cells
    }

    // agg_rasterizer_cells_aa.h:625-710
    // void rasterizer_cells_aa<Cell>::sort_cells()
    pub fn sort_cells(&mut self) {
        if self.m_sorted {
            return; // Perform sort only the first time.
        }

        // agg_rasterizer_cells_aa.h:630-634
        self.add_curr_cell();
        self.m_curr_cell.x = i32::MAX;
        self.m_curr_cell.y = i32::MAX;
        self.m_curr_cell.cover = 0;
        self.m_curr_cell.area = 0;

        // agg_rasterizer_cells_aa.h:636
        if self.m_num_cells == 0 {
            return;
        }

        // agg_rasterizer_cells_aa.h:650-651
        // Allocate the array of cell pointers
        self.m_sorted_cells
            .resize(self.m_num_cells as usize, 0u32);

        // agg_rasterizer_cells_aa.h:653-655
        // Allocate and zero the Y array
        self.m_sorted_y.clear();
        self.m_sorted_y.resize(
            (self.m_max_y - self.m_min_y + 1) as usize,
            SortedY::default(),
        );

        // agg_rasterizer_cells_aa.h:657-672
        // Create the Y-histogram (count the numbers of cells for each Y)
        // (the per-block iteration collapses to a flat scan over the cells)
        for nc in 0..self.m_num_cells as usize {
            let cell_y = self.m_cells[nc].y;
            self.m_sorted_y[(cell_y - self.m_min_y) as usize].start += 1;
        }

        // agg_rasterizer_cells_aa.h:674-681
        // Convert the Y-histogram into the array of starting indexes
        let mut start: u32 = 0;
        for i in 0..self.m_sorted_y.len() {
            let v = self.m_sorted_y[i].start;
            self.m_sorted_y[i].start = start;
            start += v;
        }

        // agg_rasterizer_cells_aa.h:683-698
        // Fill the cell pointer array sorted by Y
        for nc in 0..self.m_num_cells as usize {
            let cell_y = self.m_cells[nc].y;
            let curr_y = &mut self.m_sorted_y[(cell_y - self.m_min_y) as usize];
            self.m_sorted_cells[(curr_y.start + curr_y.num) as usize] = nc as u32;
            curr_y.num += 1;
        }

        // agg_rasterizer_cells_aa.h:700-708
        // Finally arrange the X-arrays
        for i in 0..self.m_sorted_y.len() {
            let curr_y = self.m_sorted_y[i];
            if curr_y.num != 0 {
                qsort_cells(
                    &mut self.m_sorted_cells
                        [curr_y.start as usize..(curr_y.start + curr_y.num) as usize],
                    &self.m_cells,
                );
            }
        }
        self.m_sorted = true;
    }
}

// agg_rasterizer_cells_aa.h:499-504
// template <class T> static AGG_INLINE void swap_cells(T* a, T* b)
// (slice::swap below)

// agg_rasterizer_cells_aa.h:508-511  enum { qsort_threshold = 9 };
const QSORT_THRESHOLD: isize = 9;

// agg_rasterizer_cells_aa.h:515-621
// template<class Cell> void qsort_cells(Cell** start, unsigned num)
//
// The C++ version sorts an array of cell POINTERS comparing `(*ptr)->x`; the
// Rust version sorts the corresponding array of cell INDICES comparing
// `cells[idx].x` — identical comparisons, identical swaps, identical order.
fn qsort_cells(start: &mut [u32], cells: &[CellAa]) {
    #[inline]
    fn x_of(start: &[u32], cells: &[CellAa], i: isize) -> i32 {
        cells[start[i as usize] as usize].x
    }

    // Cell**  stack[80];  Cell*** top;  Cell** limit;  Cell** base;
    // (pointers into the array become isize indices)
    let mut stack: [isize; 80] = [0; 80];
    let mut top: usize = 0;
    let mut limit: isize = start.len() as isize;
    let mut base: isize = 0;

    loop {
        let len: isize = limit - base;

        let mut i: isize;
        let mut j: isize;
        let pivot: isize;

        if len > QSORT_THRESHOLD {
            // agg_rasterizer_cells_aa.h:536-539
            // we use base + len/2 as the pivot
            pivot = base + len / 2;
            start.swap(base as usize, pivot as usize);

            i = base + 1;
            j = limit - 1;

            // agg_rasterizer_cells_aa.h:544-558
            // now ensure that *i <= *base <= *j
            if x_of(start, cells, j) < x_of(start, cells, i) {
                start.swap(i as usize, j as usize);
            }
            if x_of(start, cells, base) < x_of(start, cells, i) {
                start.swap(base as usize, i as usize);
            }
            if x_of(start, cells, j) < x_of(start, cells, base) {
                start.swap(base as usize, j as usize);
            }

            // agg_rasterizer_cells_aa.h:560-572
            loop {
                let x: i32 = x_of(start, cells, base);
                loop {
                    i += 1;
                    if x_of(start, cells, i) >= x {
                        break;
                    }
                }
                loop {
                    j -= 1;
                    if x >= x_of(start, cells, j) {
                        break;
                    }
                }

                if i > j {
                    break;
                }

                start.swap(i as usize, j as usize);
            }

            // agg_rasterizer_cells_aa.h:574
            start.swap(base as usize, j as usize);

            // agg_rasterizer_cells_aa.h:576-589
            // now, push the largest sub-array
            if j - base > limit - i {
                stack[top] = base;
                stack[top + 1] = j;
                base = i;
            } else {
                stack[top] = i;
                stack[top + 1] = limit;
                limit = j;
            }
            top += 2;
        } else {
            // agg_rasterizer_cells_aa.h:591-607
            // the sub-array is small, perform insertion sort
            j = base;
            i = j + 1;

            while i < limit {
                while x_of(start, cells, j + 1) < x_of(start, cells, j) {
                    start.swap((j + 1) as usize, j as usize);
                    if j == base {
                        break;
                    }
                    j -= 1;
                }
                j = i;
                i += 1;
            }

            // agg_rasterizer_cells_aa.h:609-618
            if top > 0 {
                top -= 2;
                base = stack[top];
                limit = stack[top + 1];
            } else {
                break;
            }
        }
    }
}
