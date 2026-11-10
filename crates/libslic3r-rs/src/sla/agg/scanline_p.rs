//! Faithful port of the vendored AGG header `src/agg/agg_scanline_p.h`
//! (Anti-Grain Geometry 2.4, as bundled with BambuStudio) — `scanline_p8`,
//! a general purpose scanline container with packed spans.
//!
//! C++ Reference:
//! - agg/agg_scanline_p.h
//!
//! The C++ container uses raw pointers into its internal `m_covers` /
//! `m_spans` arrays (`m_cover_ptr`, `m_cur_span`, `span::covers`). The Rust
//! port replaces each pointer with the corresponding index into the same
//! arrays; all arithmetic and span-packing logic is otherwise identical.
//! `scanline32_p8` (agg_scanline_p.h:181-322) has no users here and is not
//! ported.

// agg_scanline_p.h:42  class scanline_p8
// agg_scanline_p.h:46-47
// typedef int8u       cover_type;
// typedef int16       coord_type;

// agg_scanline_p.h:50-55
// struct span
// {
//     coord_type        x;
//     coord_type        len; // If negative, it's a solid span, covers is valid
//     const cover_type* covers;
// };
#[derive(Debug, Clone, Copy, Default)]
pub struct Span {
    pub x: i16,
    pub len: i16, // If negative, it's a solid span, covers is valid
    pub covers: usize, // index into ScanlineP8::m_covers
}

#[derive(Debug, Default)]
pub struct ScanlineP8 {
    // agg_scanline_p.h:166  int m_last_x;
    m_last_x: i32,
    // agg_scanline_p.h:167  int m_y;
    m_y: i32,
    // agg_scanline_p.h:168  pod_array<cover_type> m_covers;
    m_covers: Vec<u8>,
    // agg_scanline_p.h:169  cover_type* m_cover_ptr;  (index into m_covers)
    m_cover_ptr: usize,
    // agg_scanline_p.h:170  pod_array<span> m_spans;
    m_spans: Vec<Span>,
    // agg_scanline_p.h:171  span* m_cur_span;  (index into m_spans)
    m_cur_span: usize,
}

impl ScanlineP8 {
    // agg_scanline_p.h:60-67
    // scanline_p8() :
    //     m_last_x(0x7FFFFFF0),
    //     m_covers(),
    //     m_cover_ptr(0),
    //     m_spans(),
    //     m_cur_span(0)
    // {}
    pub fn new() -> Self {
        Self {
            m_last_x: 0x7FFF_FFF0,
            m_y: 0,
            m_covers: Vec::new(),
            m_cover_ptr: 0,
            m_spans: Vec::new(),
            m_cur_span: 0,
        }
    }

    // agg_scanline_p.h:70-82
    // void reset(int min_x, int max_x)
    // {
    //     unsigned max_len = max_x - min_x + 3;
    //     if(max_len > m_spans.size())
    //     {
    //         m_spans.resize(max_len);
    //         m_covers.resize(max_len);
    //     }
    //     m_last_x    = 0x7FFFFFF0;
    //     m_cover_ptr = &m_covers[0];
    //     m_cur_span  = &m_spans[0];
    //     m_cur_span->len = 0;
    // }
    pub fn reset(&mut self, min_x: i32, max_x: i32) {
        let max_len = (max_x - min_x + 3) as usize;
        if max_len > self.m_spans.len() {
            self.m_spans.resize(max_len, Span::default());
            self.m_covers.resize(max_len, 0);
        }
        self.m_last_x = 0x7FFF_FFF0;
        self.m_cover_ptr = 0;
        self.m_cur_span = 0;
        self.m_spans[self.m_cur_span].len = 0;
    }

    // agg_scanline_p.h:85-101
    // void add_cell(int x, unsigned cover)
    // {
    //     *m_cover_ptr = (cover_type)cover;
    //     if(x == m_last_x+1 && m_cur_span->len > 0)
    //     {
    //         m_cur_span->len++;
    //     }
    //     else
    //     {
    //         m_cur_span++;
    //         m_cur_span->covers = m_cover_ptr;
    //         m_cur_span->x = (int16)x;
    //         m_cur_span->len = 1;
    //     }
    //     m_last_x = x;
    //     m_cover_ptr++;
    // }
    pub fn add_cell(&mut self, x: i32, cover: u32) {
        self.m_covers[self.m_cover_ptr] = cover as u8;
        if x == self.m_last_x + 1 && self.m_spans[self.m_cur_span].len > 0 {
            self.m_spans[self.m_cur_span].len += 1;
        } else {
            self.m_cur_span += 1;
            self.m_spans[self.m_cur_span].covers = self.m_cover_ptr;
            self.m_spans[self.m_cur_span].x = x as i16;
            self.m_spans[self.m_cur_span].len = 1;
        }
        self.m_last_x = x;
        self.m_cover_ptr += 1;
    }

    // agg_scanline_p.h:104-120
    // void add_cells(int x, unsigned len, const cover_type* covers)
    pub fn add_cells(&mut self, x: i32, len: u32, covers: &[u8]) {
        // memcpy(m_cover_ptr, covers, len * sizeof(cover_type));
        self.m_covers[self.m_cover_ptr..self.m_cover_ptr + len as usize]
            .copy_from_slice(&covers[..len as usize]);
        if x == self.m_last_x + 1 && self.m_spans[self.m_cur_span].len > 0 {
            self.m_spans[self.m_cur_span].len += len as i16;
        } else {
            self.m_cur_span += 1;
            self.m_spans[self.m_cur_span].covers = self.m_cover_ptr;
            self.m_spans[self.m_cur_span].x = x as i16;
            self.m_spans[self.m_cur_span].len = len as i16;
        }
        self.m_cover_ptr += len as usize;
        self.m_last_x = x + len as i32 - 1;
    }

    // agg_scanline_p.h:123-140
    // void add_span(int x, unsigned len, unsigned cover)
    // {
    //     if(x == m_last_x+1 &&
    //        m_cur_span->len < 0 &&
    //        cover == *m_cur_span->covers)
    //     {
    //         m_cur_span->len -= (int16)len;
    //     }
    //     else
    //     {
    //         *m_cover_ptr = (cover_type)cover;
    //         m_cur_span++;
    //         m_cur_span->covers = m_cover_ptr++;
    //         m_cur_span->x      = (int16)x;
    //         m_cur_span->len    = (int16)(-int(len));
    //     }
    //     m_last_x = x + len - 1;
    // }
    pub fn add_span(&mut self, x: i32, len: u32, cover: u32) {
        if x == self.m_last_x + 1
            && self.m_spans[self.m_cur_span].len < 0
            && cover == self.m_covers[self.m_spans[self.m_cur_span].covers] as u32
        {
            self.m_spans[self.m_cur_span].len -= len as i16;
        } else {
            self.m_covers[self.m_cover_ptr] = cover as u8;
            self.m_cur_span += 1;
            self.m_spans[self.m_cur_span].covers = self.m_cover_ptr;
            self.m_cover_ptr += 1;
            self.m_spans[self.m_cur_span].x = x as i16;
            self.m_spans[self.m_cur_span].len = -(len as i32) as i16;
        }
        self.m_last_x = x + len as i32 - 1;
    }

    // agg_scanline_p.h:143-146
    // void finalize(int y) { m_y = y; }
    pub fn finalize(&mut self, y: i32) {
        self.m_y = y;
    }

    // agg_scanline_p.h:149-155
    // void reset_spans()
    // {
    //     m_last_x    = 0x7FFFFFF0;
    //     m_cover_ptr = &m_covers[0];
    //     m_cur_span  = &m_spans[0];
    //     m_cur_span->len = 0;
    // }
    pub fn reset_spans(&mut self) {
        self.m_last_x = 0x7FFF_FFF0;
        self.m_cover_ptr = 0;
        self.m_cur_span = 0;
        self.m_spans[self.m_cur_span].len = 0;
    }

    // agg_scanline_p.h:158  int y() const { return m_y; }
    #[inline]
    pub fn y(&self) -> i32 {
        self.m_y
    }

    // agg_scanline_p.h:159
    // unsigned num_spans() const { return unsigned(m_cur_span - &m_spans[0]); }
    #[inline]
    pub fn num_spans(&self) -> u32 {
        self.m_cur_span as u32
    }

    // agg_scanline_p.h:160  const_iterator begin() const { return &m_spans[1]; }
    // (returns the spans starting at index 1, plus the covers array the
    // `span::covers` indices refer to)
    #[inline]
    pub fn begin(&self) -> &[Span] {
        &self.m_spans[1..]
    }

    /// The covers array referenced by `Span::covers` (C++ `span::covers` is a
    /// raw pointer into this storage).
    #[inline]
    pub fn covers(&self) -> &[u8] {
        &self.m_covers
    }
}
