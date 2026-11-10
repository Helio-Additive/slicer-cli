//! Faithful port of the vendored AGG header `src/agg/agg_path_storage.h`
//! (Anti-Grain Geometry 2.4, as bundled with BambuStudio) — the
//! `path_storage = path_base<vertex_block_storage<double>>` instantiation
//! (agg_path_storage.h:1561), restricted to the members exercised by
//! `SLA/AGGRaster.hpp` and the scanline rasterizer's `add_path`.
//!
//! C++ Reference:
//! - agg/agg_path_storage.h
//!
//! `vertex_block_storage` keeps vertices in 8-bit-command + double-pair
//! blocks; the block allocation scheme (agg_path_storage.h:298-345) is a
//! memory-management detail and collapses to growable `Vec`s. All observable
//! behavior (commands, coordinates, iteration order) is identical.
//!
//! Curve/arc path commands, `concat_path`/`join_path`, polygon orientation
//! helpers and the serialization adaptors (agg_path_storage.h:360-600,
//! 939-1160, 1250-1386, 1457-1582) have no users in the SLA raster path and
//! are not ported.

use super::basics::{is_stop, is_vertex, PATH_CMD_LINE_TO, PATH_CMD_MOVE_TO, PATH_CMD_STOP};

/// The vertex-source interface consumed by
/// `rasterizer_scanline_aa::add_path` (duck-typed in C++).
pub trait VertexSource {
    // void rewind(unsigned path_id);
    fn rewind(&mut self, path_id: u32);
    // unsigned vertex(double* x, double* y);
    fn vertex(&mut self, x: &mut f64, y: &mut f64) -> u32;
}

// agg_path_storage.h:31  class vertex_block_storage
#[derive(Debug, Clone, Default)]
pub struct VertexBlockStorage {
    // agg_path_storage.h:76  unsigned m_total_vertices;  (== coords.len())
    // (block arrays m_coord_blocks / m_cmd_blocks collapse to flat Vecs)
    coords: Vec<[f64; 2]>,
    cmds: Vec<u8>,
}

impl VertexBlockStorage {
    // agg_path_storage.h:117-124  vertex_block_storage() : ... {}
    pub fn new() -> Self {
        Self::default()
    }

    // agg_path_storage.h:155-159
    // inline void vertex_block_storage<T,S,P>::remove_all()
    // {
    //     m_total_vertices = 0;
    // }
    pub fn remove_all(&mut self) {
        self.coords.clear();
        self.cmds.clear();
    }

    // agg_path_storage.h:162-171
    // inline void vertex_block_storage<T,S,P>::add_vertex(double x, double y, unsigned cmd)
    // {
    //     T* coord_ptr = 0;
    //     *storage_ptrs(&coord_ptr) = (int8u)cmd;
    //     coord_ptr[0] = T(x);
    //     coord_ptr[1] = T(y);
    //     m_total_vertices++;
    // }
    pub fn add_vertex(&mut self, x: f64, y: f64, cmd: u32) {
        self.cmds.push(cmd as u8);
        self.coords.push([x, y]);
    }

    // agg_path_storage.h:174-181
    // inline void vertex_block_storage<T,S,P>::modify_vertex(unsigned idx, double x, double y)
    pub fn modify_vertex(&mut self, idx: u32, x: f64, y: f64) {
        let pv = &mut self.coords[idx as usize];
        pv[0] = x;
        pv[1] = y;
    }

    // agg_path_storage.h:184-195
    // inline void vertex_block_storage<T,S,P>::modify_vertex(unsigned idx, double x, double y, unsigned cmd)
    pub fn modify_vertex_cmd(&mut self, idx: u32, x: f64, y: f64, cmd: u32) {
        let pv = &mut self.coords[idx as usize];
        pv[0] = x;
        pv[1] = y;
        self.cmds[idx as usize] = cmd as u8;
    }

    // agg_path_storage.h:198-203
    // inline void vertex_block_storage<T,S,P>::modify_command(unsigned idx, unsigned cmd)
    pub fn modify_command(&mut self, idx: u32, cmd: u32) {
        self.cmds[idx as usize] = cmd as u8;
    }

    // agg_path_storage.h:206-221
    // inline void vertex_block_storage<T,S,P>::swap_vertices(unsigned v1, unsigned v2)
    pub fn swap_vertices(&mut self, v1: u32, v2: u32) {
        self.coords.swap(v1 as usize, v2 as usize);
        self.cmds.swap(v1 as usize, v2 as usize);
    }

    // agg_path_storage.h:224-229
    // inline unsigned vertex_block_storage<T,S,P>::last_command() const
    // {
    //     if(m_total_vertices) return command(m_total_vertices - 1);
    //     return path_cmd_stop;
    // }
    pub fn last_command(&self) -> u32 {
        if self.total_vertices() != 0 {
            return self.command(self.total_vertices() - 1);
        }
        PATH_CMD_STOP
    }

    // agg_path_storage.h:232-237
    // inline unsigned vertex_block_storage<T,S,P>::last_vertex(double* x, double* y) const
    pub fn last_vertex(&self, x: &mut f64, y: &mut f64) -> u32 {
        if self.total_vertices() != 0 {
            return self.vertex(self.total_vertices() - 1, x, y);
        }
        PATH_CMD_STOP
    }

    // agg_path_storage.h:240-245
    // inline unsigned vertex_block_storage<T,S,P>::prev_vertex(double* x, double* y) const
    pub fn prev_vertex(&self, x: &mut f64, y: &mut f64) -> u32 {
        if self.total_vertices() > 1 {
            return self.vertex(self.total_vertices() - 2, x, y);
        }
        PATH_CMD_STOP
    }

    // agg_path_storage.h:248-257  inline double vertex_block_storage<T,S,P>::last_x() const
    pub fn last_x(&self) -> f64 {
        if self.total_vertices() != 0 {
            return self.coords[(self.total_vertices() - 1) as usize][0];
        }
        0.0
    }

    // agg_path_storage.h:260-269  inline double vertex_block_storage<T,S,P>::last_y() const
    pub fn last_y(&self) -> f64 {
        if self.total_vertices() != 0 {
            return self.coords[(self.total_vertices() - 1) as usize][1];
        }
        0.0
    }

    // agg_path_storage.h:272-276
    // inline unsigned vertex_block_storage<T,S,P>::total_vertices() const
    #[inline]
    pub fn total_vertices(&self) -> u32 {
        self.coords.len() as u32
    }

    // agg_path_storage.h:279-288
    // inline unsigned vertex_block_storage<T,S,P>::vertex(unsigned idx, double* x, double* y) const
    #[inline]
    pub fn vertex(&self, idx: u32, x: &mut f64, y: &mut f64) -> u32 {
        let pv = &self.coords[idx as usize];
        *x = pv[0];
        *y = pv[1];
        self.cmds[idx as usize] as u32
    }

    // agg_path_storage.h:291-295
    // inline unsigned vertex_block_storage<T,S,P>::command(unsigned idx) const
    #[inline]
    pub fn command(&self, idx: u32) -> u32 {
        self.cmds[idx as usize] as u32
    }
}

// agg_path_storage.h:608  template<class VertexContainer> class path_base
// agg_path_storage.h:1561  typedef path_base<vertex_block_storage<double> > path_storage;
#[derive(Debug, Clone, Default)]
pub struct PathStorage {
    // agg_path_storage.h (private)  VertexContainer m_vertices;
    m_vertices: VertexBlockStorage,
    // agg_path_storage.h (private)  unsigned m_iterator;
    m_iterator: u32,
}

impl PathStorage {
    // agg_path_storage.h:613 (approx)  path_base() : m_vertices(), m_iterator(0) {}
    pub fn new() -> Self {
        Self::default()
    }

    // void remove_all() { m_vertices.remove_all(); m_iterator = 0; }
    pub fn remove_all(&mut self) {
        self.m_vertices.remove_all();
        self.m_iterator = 0;
    }

    // agg_path_storage.h:885-893
    // unsigned path_base<VC>::start_new_path()
    // {
    //     if(!is_stop(m_vertices.last_command()))
    //     {
    //         m_vertices.add_vertex(0.0, 0.0, path_cmd_stop);
    //     }
    //     return m_vertices.total_vertices();
    // }
    pub fn start_new_path(&mut self) -> u32 {
        if !is_stop(self.m_vertices.last_command()) {
            self.m_vertices.add_vertex(0.0, 0.0, PATH_CMD_STOP);
        }
        self.m_vertices.total_vertices()
    }

    // agg_path_storage.h:913-917
    // inline void path_base<VC>::move_to(double x, double y)
    // {
    //     m_vertices.add_vertex(x, y, path_cmd_move_to);
    // }
    pub fn move_to(&mut self, x: f64, y: f64) {
        self.m_vertices.add_vertex(x, y, PATH_CMD_MOVE_TO);
    }

    // agg_path_storage.h:928-932
    // inline void path_base<VC>::line_to(double x, double y)
    // {
    //     m_vertices.add_vertex(x, y, path_cmd_line_to);
    // }
    pub fn line_to(&mut self, x: f64, y: f64) {
        self.m_vertices.add_vertex(x, y, PATH_CMD_LINE_TO);
    }

    // agg_path_storage.h:1166-1170
    // inline unsigned path_base<VC>::total_vertices() const
    #[inline]
    pub fn total_vertices(&self) -> u32 {
        self.m_vertices.total_vertices()
    }

    // agg_path_storage.h:1201-1205
    // inline unsigned path_base<VC>::vertex(unsigned idx, double* x, double* y) const
    // (the indexed overload of `vertex`)
    #[inline]
    pub fn vertex_at(&self, idx: u32, x: &mut f64, y: &mut f64) -> u32 {
        self.m_vertices.vertex(idx, x, y)
    }

    // agg_path_storage.h:1208-1212
    // inline unsigned path_base<VC>::command(unsigned idx) const
    #[inline]
    pub fn command(&self, idx: u32) -> u32 {
        self.m_vertices.command(idx)
    }

    // agg_path_storage.h:1215-1219
    // void path_base<VC>::modify_vertex(unsigned idx, double x, double y)
    pub fn modify_vertex(&mut self, idx: u32, x: f64, y: f64) {
        self.m_vertices.modify_vertex(idx, x, y);
    }

    // agg_path_storage.h:1388-1402
    // void path_base<VC>::flip_x(double x1, double x2)
    // {
    //     unsigned i;
    //     double x, y;
    //     for(i = 0; i < m_vertices.total_vertices(); i++)
    //     {
    //         unsigned cmd = m_vertices.vertex(i, &x, &y);
    //         if(is_vertex(cmd))
    //         {
    //             m_vertices.modify_vertex(i, x2 - x + x1, y);
    //         }
    //     }
    // }
    pub fn flip_x(&mut self, x1: f64, x2: f64) {
        let mut x = 0.0f64;
        let mut y = 0.0f64;
        let mut i = 0u32;
        while i < self.m_vertices.total_vertices() {
            let cmd = self.m_vertices.vertex(i, &mut x, &mut y);
            if is_vertex(cmd) {
                self.m_vertices.modify_vertex(i, x2 - x + x1, y);
            }
            i += 1;
        }
    }

    // agg_path_storage.h:1404-1418
    // void path_base<VC>::flip_y(double y1, double y2)
    pub fn flip_y(&mut self, y1: f64, y2: f64) {
        let mut x = 0.0f64;
        let mut y = 0.0f64;
        let mut i = 0u32;
        while i < self.m_vertices.total_vertices() {
            let cmd = self.m_vertices.vertex(i, &mut x, &mut y);
            if is_vertex(cmd) {
                self.m_vertices.modify_vertex(i, x, y2 - y + y1);
            }
            i += 1;
        }
    }

    // agg_path_storage.h:1439-1455
    // void path_base<VC>::translate_all_paths(double dx, double dy)
    // {
    //     unsigned idx;
    //     unsigned num_ver = m_vertices.total_vertices();
    //     for(idx = 0; idx < num_ver; idx++)
    //     {
    //         double x, y;
    //         if(is_vertex(m_vertices.vertex(idx, &x, &y)))
    //         {
    //             x += dx;
    //             y += dy;
    //             m_vertices.modify_vertex(idx, x, y);
    //         }
    //     }
    // }
    pub fn translate_all_paths(&mut self, dx: f64, dy: f64) {
        let num_ver = self.m_vertices.total_vertices();
        for idx in 0..num_ver {
            let mut x = 0.0f64;
            let mut y = 0.0f64;
            if is_vertex(self.m_vertices.vertex(idx, &mut x, &mut y)) {
                x += dx;
                y += dy;
                self.m_vertices.modify_vertex(idx, x, y);
            }
        }
    }
}

impl VertexSource for PathStorage {
    // agg_path_storage.h:1235-1240
    // inline void path_base<VC>::rewind(unsigned path_id)
    // {
    //     m_iterator = path_id;
    // }
    fn rewind(&mut self, path_id: u32) {
        self.m_iterator = path_id;
    }

    // agg_path_storage.h:1242-1248
    // inline unsigned path_base<VC>::vertex(double* x, double* y)
    // {
    //     if(m_iterator >= m_vertices.total_vertices()) return path_cmd_stop;
    //     return m_vertices.vertex(m_iterator++, x, y);
    // }
    fn vertex(&mut self, x: &mut f64, y: &mut f64) -> u32 {
        if self.m_iterator >= self.m_vertices.total_vertices() {
            return PATH_CMD_STOP;
        }
        let cmd = self.m_vertices.vertex(self.m_iterator, x, y);
        self.m_iterator += 1;
        cmd
    }
}
