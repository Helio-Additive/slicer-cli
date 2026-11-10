//! Faithful port of the vendored AGG header `src/agg/agg_rendering_buffer.h`
//! (Anti-Grain Geometry 2.4, as bundled with BambuStudio) — the
//! `row_accessor<int8u>` instantiation (`agg::rendering_buffer`).
//!
//! C++ Reference:
//! - agg/agg_rendering_buffer.h
//!
//! The C++ `rendering_buffer` is a non-owning view (raw pointer + stride) over
//! the raster's pixel memory. The Rust port borrows the buffer as a mutable
//! byte slice; raw row pointers become byte offsets into that slice.

// agg_rendering_buffer.h:28-29  template<class T> class row_accessor
// agg_rendering_buffer.h:300(approx)  typedef row_accessor<int8u> rendering_buffer;
#[derive(Debug)]
pub struct RenderingBuffer<'a> {
    // agg_rendering_buffer.h  T* m_buf;    // Pointer to renrdering buffer
    pub(crate) m_buf: &'a mut [u8],
    // agg_rendering_buffer.h  T* m_start;  // Pointer to first pixel depending on stride
    // (kept as a byte offset from `m_buf`)
    m_start: usize,
    // agg_rendering_buffer.h  unsigned m_width;   // Width in pixels
    m_width: u32,
    // agg_rendering_buffer.h  unsigned m_height;  // Height in pixels
    m_height: u32,
    // agg_rendering_buffer.h  int m_stride;       // Number of bytes per row. Can be < 0
    m_stride: i32,
}

impl<'a> RenderingBuffer<'a> {
    // agg_rendering_buffer.h:45-53
    // row_accessor(T* buf, unsigned width, unsigned height, int stride)
    // { attach(buf, width, height, stride); }
    pub fn new(buf: &'a mut [u8], width: u32, height: u32, stride: i32) -> Self {
        let mut rb = Self {
            m_buf: buf,
            m_start: 0,
            m_width: 0,
            m_height: 0,
            m_stride: 0,
        };
        rb.attach(width, height, stride);
        rb
    }

    // agg_rendering_buffer.h:56-67
    // void attach(T* buf, unsigned width, unsigned height, int stride)
    // {
    //     m_buf = m_start = buf;
    //     m_width = width;
    //     m_height = height;
    //     m_stride = stride;
    //     if(stride < 0)
    //     {
    //         m_start = m_buf - int(height - 1) * stride;
    //     }
    // }
    pub fn attach(&mut self, width: u32, height: u32, stride: i32) {
        self.m_start = 0;
        self.m_width = width;
        self.m_height = height;
        self.m_stride = stride;
        if stride < 0 {
            self.m_start = (-((height as i32 - 1) * stride)) as usize;
        }
    }

    // agg_rendering_buffer.h:70-73 (accessors)
    // AGG_INLINE unsigned width()  const { return m_width;  }
    #[inline]
    pub fn width(&self) -> u32 {
        self.m_width
    }

    // AGG_INLINE unsigned height() const { return m_height; }
    #[inline]
    pub fn height(&self) -> u32 {
        self.m_height
    }

    // AGG_INLINE int stride() const { return m_stride; }
    #[inline]
    pub fn stride(&self) -> i32 {
        self.m_stride
    }

    // agg_rendering_buffer.h:75-78
    // AGG_INLINE unsigned stride_abs() const
    #[inline]
    pub fn stride_abs(&self) -> u32 {
        if self.m_stride < 0 {
            (-self.m_stride) as u32
        } else {
            self.m_stride as u32
        }
    }

    // agg_rendering_buffer.h:81-86
    // AGG_INLINE T* row_ptr(int, int y, unsigned) { return m_start + y * m_stride; }
    // AGG_INLINE T* row_ptr(int y) { return m_start + y * m_stride; }
    // (returns the byte offset of row `y` within the buffer slice)
    #[inline]
    pub fn row_index(&self, y: i32) -> usize {
        (self.m_start as i64 + y as i64 * self.m_stride as i64) as usize
    }
}
