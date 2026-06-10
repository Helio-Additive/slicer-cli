//! Faithful Rust port of the subset of the vendored AGG library
//! (Anti-Grain Geometry 2.4, `src/agg/` in BambuStudio) required by the SLA
//! grayscale rasterizer (`SLA/AGGRaster.hpp` / `SLA/RasterBase.cpp`).
//!
//! One module per vendored header:
//!
//! | Rust module           | C++ header                                  |
//! |-----------------------|---------------------------------------------|
//! | `basics`              | agg_basics.h                                |
//! | `clip_liang_barsky`   | agg_clip_liang_barsky.h (flag helpers)      |
//! | `color_gray`          | agg_color_gray.h (gray8)                    |
//! | `gamma_functions`     | agg_gamma_functions.h                       |
//! | `path_storage`        | agg_path_storage.h (path_storage)           |
//! | `pixfmt_gray`         | agg_pixfmt_gray.h (pixfmt_gray8)            |
//! | `rasterizer_cells_aa` | agg_rasterizer_cells_aa.h (+ cell_aa)       |
//! | `rasterizer_scanline_aa` | agg_rasterizer_scanline_aa.h             |
//! | `rasterizer_sl_clip`  | agg_rasterizer_sl_clip.h (int conv)         |
//! | `renderer_base`       | agg_renderer_base.h                         |
//! | `renderer_scanline`   | agg_renderer_scanline.h (aa solid)          |
//! | `rendering_buffer`    | agg_rendering_buffer.h (row_accessor)       |
//! | `scanline_p`          | agg_scanline_p.h (scanline_p8)              |
//!
//! All integer fixed-point arithmetic (24.8 subpixel coordinates, 8-bit
//! covers, gray8 lerp blending) is ported bit-exactly. wasm-safe: pure Rust,
//! no native dependencies.

pub mod basics;
pub mod clip_liang_barsky;
pub mod color_gray;
pub mod gamma_functions;
pub mod path_storage;
pub mod pixfmt_gray;
pub mod rasterizer_cells_aa;
pub mod rasterizer_scanline_aa;
pub mod rasterizer_sl_clip;
pub mod renderer_base;
pub mod renderer_scanline;
pub mod rendering_buffer;
pub mod scanline_p;
