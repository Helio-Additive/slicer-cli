//! Marching squares contour extraction over a 2D raster.
//!
//! Faithful 1:1 port of `MarchingSquares.hpp` (header-only template library)
//! from BambuStudio's libslic3r. The C++ original lives in namespace
//! `marchsq` and uses templates to abstract over the raster type and the
//! parallel execution policy. In Rust we model the raster abstraction with the
//! [`RasterTraits`] trait (mirroring the C++ `_RasterTraits` specialization
//! point) and replace the `ExecutionPolicy` template with serial loops (the
//! C++ default `_Loop` is a serial `for_each`).
//!
//! C++ Reference:
//! - MarchingSquares.hpp

// coord_t -> i64, but the C++ here uses `long` for grid/raster coordinates.
// We mirror `long` with i64 to match 64-bit platforms. Isovalues use the
// raster's own ValueType (generic).

// MarchingSquares.hpp:13-23
/// Marks a square in the grid
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Coord {
    pub r: i64,
    pub c: i64,
}

impl Coord {
    // MarchingSquares.hpp:16  Coord() = default;
    #[inline]
    pub fn new() -> Self {
        Coord { r: 0, c: 0 }
    }

    // MarchingSquares.hpp:17  explicit Coord(long s) : r(s), c(s) {}
    #[inline]
    pub fn splat(s: i64) -> Self {
        Coord { r: s, c: s }
    }

    // MarchingSquares.hpp:18  Coord(long _r, long _c): r(_r), c(_c) {}
    #[inline]
    pub fn rc(r: i64, c: i64) -> Self {
        Coord { r, c }
    }

    // MarchingSquares.hpp:20  size_t seq(const Coord &res) const { return r * res.c + c; }
    #[inline]
    pub fn seq(&self, res: &Coord) -> usize {
        (self.r * res.c + self.c) as usize
    }
}

// MarchingSquares.hpp:21-22  operator+= / operator+
impl std::ops::AddAssign for Coord {
    #[inline]
    fn add_assign(&mut self, b: Coord) {
        self.r += b.r;
        self.c += b.c;
    }
}

impl std::ops::Add for Coord {
    type Output = Coord;
    #[inline]
    fn add(self, b: Coord) -> Coord {
        let mut a = self;
        a += b;
        a
    }
}

// MarchingSquares.hpp:26  using Ring = std::vector<Coord>;
/// Closed ring of cell coordinates
pub type Ring = Vec<Coord>;

// MarchingSquares.hpp:28-40
// Specialize this trait to register a raster type for the Marching squares alg.
// (C++ `_RasterTraits<T>` specialization point.)
pub trait RasterTraits {
    // The type of pixel cell in the raster
    type ValueType: Copy + PartialOrd;

    // Value at a given position
    fn get(&self, row: usize, col: usize) -> Self::ValueType;

    // Number of rows and cols of the raster
    fn rows(&self) -> usize;
    fn cols(&self) -> usize;
}

// __impl namespace begins. MarchingSquares.hpp:50

// MarchingSquares.hpp:53  TRasterValue<T>
type TRasterValue<T> = <T as RasterTraits>::ValueType;

// MarchingSquares.hpp:55-58
#[inline]
fn rows<T: RasterTraits>(raster: &T) -> usize {
    raster.rows()
}

// MarchingSquares.hpp:60-63
#[inline]
fn cols<T: RasterTraits>(raster: &T) -> usize {
    raster.cols()
}

// MarchingSquares.hpp:65-68
#[inline]
fn isoval<T: RasterTraits>(rst: &T, crd: &Coord) -> TRasterValue<T> {
    rst.get(crd.r as usize, crd.c as usize)
}

// MarchingSquares.hpp:70-74  for_each (serial default _Loop)
#[inline]
fn for_each<It, F>(items: It, mut f: F)
where
    It: IntoIterator,
    F: FnMut(It::Item, usize),
{
    for (idx, item) in items.into_iter().enumerate() {
        f(item, idx);
    }
}

// MarchingSquares.hpp:76-86
// Type of squares (tiles) depending on which vertices are inside an ROI
// The vertices would be marked a, b, c, d in counter clockwise order from the
// bottom left vertex of a square.
// d --- c
// |     |
// |     |
// a --- b
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SquareTag {
    //     0, 1, 2,  3, 4,  5,  6,   7, 8,  9, 10,  11, 12,  13,  14,  15
    None = 0,
    A = 1,
    B = 2,
    Ab = 3,
    C = 4,
    Ac = 5,
    Bc = 6,
    Abc = 7,
    D = 8,
    Ad = 9,
    Bd = 10,
    Abd = 11,
    Cd = 12,
    Acd = 13,
    Bcd = 14,
    Full = 15,
}

impl SquareTag {
    // From the low nibble of a tag byte (always < 16 by construction).
    #[inline]
    fn from_u8(v: u8) -> SquareTag {
        match v {
            0 => SquareTag::None,
            1 => SquareTag::A,
            2 => SquareTag::B,
            3 => SquareTag::Ab,
            4 => SquareTag::C,
            5 => SquareTag::Ac,
            6 => SquareTag::Bc,
            7 => SquareTag::Abc,
            8 => SquareTag::D,
            9 => SquareTag::Ad,
            10 => SquareTag::Bd,
            11 => SquareTag::Abd,
            12 => SquareTag::Cd,
            13 => SquareTag::Acd,
            14 => SquareTag::Bcd,
            15 => SquareTag::Full,
            _ => unreachable!(),
        }
    }
}

// MarchingSquares.hpp:88-91
// template<class E> constexpr std::underlying_type_t<E> _t(E e)
// Returns the underlying integer value of an enum.
#[inline]
fn t_tag(e: SquareTag) -> u8 {
    e as u8
}

#[inline]
fn t_dir(e: Dir) -> u8 {
    e as u8
}

// MarchingSquares.hpp:93
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Dir {
    Left = 0,
    Down = 1,
    Right = 2,
    Up = 3,
    None = 4,
}

impl Dir {
    // Construct a Dir from a small integer (used when a Dir is smuggled
    // through Coord::c for ambiguous cases). C++ uses `Dir(ringvertex.c)`.
    #[inline]
    fn from_i64(v: i64) -> Dir {
        match v {
            0 => Dir::Left,
            1 => Dir::Down,
            2 => Dir::Right,
            3 => Dir::Up,
            4 => Dir::None,
            _ => Dir::None,
        }
    }
}

// MarchingSquares.hpp:95-112
static NEXT_CCW: [Dir; 16] = [
    /* 00 */ Dir::None, // SquareTag::none (empty square, nowhere to go)
    /* 01 */ Dir::Left, // SquareTag::a
    /* 02 */ Dir::Down, // SquareTag::b
    /* 03 */ Dir::Left, // SquareTag::ab
    /* 04 */ Dir::Right, // SquareTag::c
    /* 05 */ Dir::None, // SquareTag::ac   (ambiguous case)
    /* 06 */ Dir::Down, // SquareTag::bc
    /* 07 */ Dir::Left, // SquareTag::abc
    /* 08 */ Dir::Up,   // SquareTag::d
    /* 09 */ Dir::Up,   // SquareTag::ad
    /* 10 */ Dir::None, // SquareTag::bd   (ambiguous case)
    /* 11 */ Dir::Up,   // SquareTag::abd
    /* 12 */ Dir::Right, // SquareTag::cd
    /* 13 */ Dir::Right, // SquareTag::acd
    /* 14 */ Dir::Down, // SquareTag::bcd
    /* 15 */ Dir::None, // SquareTag::full (full covered, nowhere to go)
];

// MarchingSquares.hpp:114-131
static PREV_CCW: [u8; 16] = [
    /* 00 */ 1 << (Dir::None as u8),
    /* 01 */ 1 << (Dir::Up as u8),
    /* 02 */ 1 << (Dir::Left as u8),
    /* 03 */ 1 << (Dir::Left as u8),
    /* 04 */ 1 << (Dir::Down as u8),
    /* 05 */ (1 << (Dir::Up as u8)) | (1 << (Dir::Down as u8)),
    /* 06 */ 1 << (Dir::Down as u8),
    /* 07 */ 1 << (Dir::Down as u8),
    /* 08 */ 1 << (Dir::Right as u8),
    /* 09 */ 1 << (Dir::Up as u8),
    /* 10 */ (1 << (Dir::Left as u8)) | (1 << (Dir::Right as u8)),
    /* 11 */ 1 << (Dir::Left as u8),
    /* 12 */ 1 << (Dir::Right as u8),
    /* 13 */ 1 << (Dir::Up as u8),
    /* 14 */ 1 << (Dir::Right as u8),
    /* 15 */ 1 << (Dir::None as u8),
];

// MarchingSquares.hpp:133-135
const DIRMASKS: [u8; 5] = [
    /*left: */ 0x01, /*down*/ 0x12, /*right */ 0x21, /*up*/ 0x10, /*none*/ 0x00,
];

// MarchingSquares.hpp:137-141
#[inline]
fn step(crd: &Coord, d: Dir) -> Coord {
    let dd = DIRMASKS[d as u8 as usize];
    Coord {
        r: crd.r - 1 + (dd & 0x0f) as i64,
        c: crd.c - 1 + (dd >> 4) as i64,
    }
}

// MarchingSquares.hpp:143  template<class Rst> class Grid
struct Grid<'a, Rst: RasterTraits> {
    m_rst: &'a Rst,
    m_cellsize: Coord,
    m_res_1: Coord,
    m_window: Coord,
    m_gridsize: Coord,
    #[allow(dead_code)]
    m_grid_1: Coord,
    m_tags: Vec<u8>, // Assign tags to each square
}

// MarchingSquares.hpp:241-254
// Two cell iterators representing an edge of a square. This is then
// used for binary search for the first active pixel on the edge.
struct CellIt<'a, Rst: RasterTraits> {
    crd: Coord,
    dir: Dir,
    grid: Option<&'a Rst>,
}

// Manual Clone/Copy to avoid the derive adding a spurious `Rst: Copy` bound
// (all fields are trivially copyable: Coord, Dir, and a reference).
impl<'a, Rst: RasterTraits> Clone for CellIt<'a, Rst> {
    #[inline]
    fn clone(&self) -> Self {
        *self
    }
}
impl<'a, Rst: RasterTraits> Copy for CellIt<'a, Rst> {}

impl<'a, Rst: RasterTraits> CellIt<'a, Rst> {
    // MarchingSquares.hpp:244  TRasterValue<Rst> operator*() const
    #[inline]
    fn deref(&self) -> TRasterValue<Rst> {
        isoval(self.grid.unwrap(), &self.crd)
    }

    // MarchingSquares.hpp:245  CellIt& operator++()
    #[inline]
    fn inc(&mut self) {
        self.crd = step(&self.crd, self.dir);
    }

    // MarchingSquares.hpp:247  operator!=
    #[inline]
    fn ne(&self, it: &CellIt<'a, Rst>) -> bool {
        self.crd.r != it.crd.r || self.crd.c != it.crd.c
    }
}

// MarchingSquares.hpp:258  struct Edge { CellIt from, to; };
struct Edge<'a, Rst: RasterTraits> {
    from: CellIt<'a, Rst>,
    to: CellIt<'a, Rst>,
}

// Faithful port of std::lower_bound over a forward-iterator range
// [first, last) where elements are isovalues obtained by dereferencing the
// CellIt. Returns the iterator to the first element not less than `value`.
// MarchingSquares.hpp:401  std::lower_bound(e.from, e.to, isov)
fn lower_bound<'a, Rst: RasterTraits>(
    first: CellIt<'a, Rst>,
    last: CellIt<'a, Rst>,
    value: TRasterValue<Rst>,
) -> CellIt<'a, Rst> {
    // std::distance for forward iterators: linear count of ++ until == last.
    let mut count: i64 = 0;
    {
        let mut it = first;
        while it.ne(&last) {
            it.inc();
            count += 1;
        }
    }

    let mut first = first;
    while count > 0 {
        let mut it = first;
        let step_n = count / 2;
        // std::advance(it, step)
        for _ in 0..step_n {
            it.inc();
        }
        if it.deref() < value {
            it.inc();
            first = it;
            count -= step_n + 1;
        } else {
            count = step_n;
        }
    }

    first
}

impl<'a, Rst: RasterTraits> Grid<'a, Rst> {
    // MarchingSquares.hpp:148-151
    #[inline]
    fn rastercoord(&self, crd: &Coord) -> Coord {
        Coord {
            r: (crd.r - 1) * self.m_window.r,
            c: (crd.c - 1) * self.m_window.c,
        }
    }

    // MarchingSquares.hpp:153-156
    #[inline]
    fn bl(&self, crd: &Coord) -> Coord {
        self.tl(crd) + Coord { r: self.m_res_1.r, c: 0 }
    }
    #[inline]
    fn br(&self, crd: &Coord) -> Coord {
        self.tl(crd)
            + Coord {
                r: self.m_res_1.r,
                c: self.m_res_1.c,
            }
    }
    #[inline]
    fn tr(&self, crd: &Coord) -> Coord {
        self.tl(crd) + Coord { r: 0, c: self.m_res_1.c }
    }
    #[inline]
    fn tl(&self, crd: &Coord) -> Coord {
        self.rastercoord(crd)
    }

    // MarchingSquares.hpp:158-162
    #[inline]
    fn is_within(&self, crd: &Coord) -> bool {
        let r = rows(self.m_rst) as i64;
        let c = cols(self.m_rst) as i64;
        crd.r >= 0 && crd.r < r && crd.c >= 0 && crd.c < c
    }

    // MarchingSquares.hpp:164-177
    // Calculate the tag for a cell (or square). The cell coordinates mark the
    // top left vertex of a square in the raster. v is the isovalue
    fn get_tag_for_cell(&self, cell: &Coord, v: TRasterValue<Rst>) -> u8 {
        let sqr = [self.bl(cell), self.br(cell), self.tr(cell), self.tl(cell)];

        let b0 = (self.is_within(&sqr[0]) && isoval(self.m_rst, &sqr[0]) >= v) as u8;
        let b1 = (self.is_within(&sqr[1]) && isoval(self.m_rst, &sqr[1]) >= v) as u8;
        let b2 = (self.is_within(&sqr[2]) && isoval(self.m_rst, &sqr[2]) >= v) as u8;
        let b3 = (self.is_within(&sqr[3]) && isoval(self.m_rst, &sqr[3]) >= v) as u8;

        let t = b0 + (b1 << 1) + (b2 << 2) + (b3 << 3);

        debug_assert!(t < 16);
        t
    }

    // MarchingSquares.hpp:179-183
    // Get a cell coordinate from a sequential index
    #[inline]
    fn coord(&self, i: usize) -> Coord {
        Coord {
            r: (i as i64) / self.m_gridsize.c,
            c: (i as i64) % self.m_gridsize.c,
        }
    }

    // MarchingSquares.hpp:185
    #[inline]
    fn seq(&self, crd: &Coord) -> usize {
        crd.seq(&self.m_gridsize)
    }

    // MarchingSquares.hpp:187-193
    fn is_visited(&self, idx: usize, d: Dir) -> bool {
        let t = self.get_tag(idx);
        let reff: u8 = if d == Dir::None {
            PREV_CCW[t_tag(t) as usize]
        } else {
            1u8 << t_dir(d)
        };
        t == SquareTag::Full
            || t == SquareTag::None
            || ((self.m_tags[idx] & 0xf0) >> 4) == reff
    }

    // MarchingSquares.hpp:195-198
    #[inline]
    fn set_visited(&mut self, idx: usize, d: Dir) {
        self.m_tags[idx] |= (1u8 << t_dir(d)) << 4;
    }

    // MarchingSquares.hpp:200-204
    #[inline]
    fn is_ambiguous(&self, idx: usize) -> bool {
        let t = self.get_tag(idx);
        t == SquareTag::Ac || t == SquareTag::Bd
    }

    // MarchingSquares.hpp:206-214
    // Search for a new starting square
    fn search_start_cell(&self, mut i: usize) -> usize {
        // Skip ambiguous tags as starting tags due to unknown previous
        // direction.
        while (i < self.m_tags.len()) && (self.is_visited(i, Dir::None) || self.is_ambiguous(i)) {
            i += 1;
        }

        i
    }

    // MarchingSquares.hpp:216
    #[inline]
    fn get_tag(&self, idx: usize) -> SquareTag {
        SquareTag::from_u8(self.m_tags[idx] & 0x0f)
    }

    // MarchingSquares.hpp:218-239
    fn next_dir(&self, prev: Dir, tag: SquareTag) -> Dir {
        // Treat ambiguous cases as two separate regions in one square.
        match tag {
            SquareTag::Ac => match prev {
                Dir::Down => Dir::Right,
                Dir::Up => Dir::Left,
                _ => {
                    debug_assert!(false);
                    Dir::None
                }
            },
            SquareTag::Bd => match prev {
                Dir::Right => Dir::Up,
                Dir::Left => Dir::Down,
                _ => {
                    debug_assert!(false);
                    Dir::None
                }
            },
            _ => NEXT_CCW[tag as u8 as usize],
        }
    }

    // MarchingSquares.hpp:260-305
    fn _edge(&self, ringvertex: &Coord) -> Edge<'a, Rst> {
        let idx = ringvertex.r as usize;
        let cell = self.coord(idx);
        let tg = self.m_tags[ringvertex.r as usize];
        let t = SquareTag::from_u8(tg & 0x0f);

        match t {
            // MarchingSquares.hpp:268-271
            SquareTag::A | SquareTag::Ab | SquareTag::Abc => Edge {
                from: CellIt {
                    crd: self.tl(&cell),
                    dir: Dir::Down,
                    grid: Some(self.m_rst),
                },
                to: CellIt {
                    crd: self.bl(&cell),
                    dir: Dir::None,
                    grid: None,
                },
            },
            // MarchingSquares.hpp:272-275
            SquareTag::B | SquareTag::Bc | SquareTag::Bcd => Edge {
                from: CellIt {
                    crd: self.bl(&cell),
                    dir: Dir::Right,
                    grid: Some(self.m_rst),
                },
                to: CellIt {
                    crd: self.br(&cell),
                    dir: Dir::None,
                    grid: None,
                },
            },
            // MarchingSquares.hpp:276-277
            SquareTag::C => Edge {
                from: CellIt {
                    crd: self.br(&cell),
                    dir: Dir::Up,
                    grid: Some(self.m_rst),
                },
                to: CellIt {
                    crd: self.tr(&cell),
                    dir: Dir::None,
                    grid: None,
                },
            },
            // MarchingSquares.hpp:278-283
            SquareTag::Ac => match Dir::from_i64(ringvertex.c) {
                Dir::Left => Edge {
                    from: CellIt {
                        crd: self.tl(&cell),
                        dir: Dir::Down,
                        grid: Some(self.m_rst),
                    },
                    to: CellIt {
                        crd: self.bl(&cell),
                        dir: Dir::None,
                        grid: None,
                    },
                },
                Dir::Right => Edge {
                    from: CellIt {
                        crd: self.br(&cell),
                        dir: Dir::Up,
                        grid: Some(self.m_rst),
                    },
                    to: CellIt {
                        crd: self.tr(&cell),
                        dir: Dir::None,
                        grid: None,
                    },
                },
                _ => {
                    debug_assert!(false);
                    // Fall through to default empty edge (C++ has no return here,
                    // hits the function-final `return {};`).
                    Edge {
                        from: CellIt {
                            crd: Coord::new(),
                            dir: Dir::None,
                            grid: None,
                        },
                        to: CellIt {
                            crd: Coord::new(),
                            dir: Dir::None,
                            grid: None,
                        },
                    }
                }
            },
            // MarchingSquares.hpp:284-287
            SquareTag::D | SquareTag::Ad | SquareTag::Abd => Edge {
                from: CellIt {
                    crd: self.tr(&cell),
                    dir: Dir::Left,
                    grid: Some(self.m_rst),
                },
                to: CellIt {
                    crd: self.tl(&cell),
                    dir: Dir::None,
                    grid: None,
                },
            },
            // MarchingSquares.hpp:288-293
            SquareTag::Bd => match Dir::from_i64(ringvertex.c) {
                Dir::Down => Edge {
                    from: CellIt {
                        crd: self.bl(&cell),
                        dir: Dir::Right,
                        grid: Some(self.m_rst),
                    },
                    to: CellIt {
                        crd: self.br(&cell),
                        dir: Dir::None,
                        grid: None,
                    },
                },
                Dir::Up => Edge {
                    from: CellIt {
                        crd: self.tr(&cell),
                        dir: Dir::Left,
                        grid: Some(self.m_rst),
                    },
                    to: CellIt {
                        crd: self.tl(&cell),
                        dir: Dir::None,
                        grid: None,
                    },
                },
                _ => {
                    debug_assert!(false);
                    Edge {
                        from: CellIt {
                            crd: Coord::new(),
                            dir: Dir::None,
                            grid: None,
                        },
                        to: CellIt {
                            crd: Coord::new(),
                            dir: Dir::None,
                            grid: None,
                        },
                    }
                }
            },
            // MarchingSquares.hpp:294-296
            SquareTag::Cd | SquareTag::Acd => Edge {
                from: CellIt {
                    crd: self.br(&cell),
                    dir: Dir::Up,
                    grid: Some(self.m_rst),
                },
                to: CellIt {
                    crd: self.tr(&cell),
                    dir: Dir::None,
                    grid: None,
                },
            },
            // MarchingSquares.hpp:297-301
            SquareTag::Full | SquareTag::None => {
                let crd = self.tl(&cell)
                    + Coord {
                        r: self.m_cellsize.r / 2,
                        c: self.m_cellsize.c / 2,
                    };
                Edge {
                    from: CellIt {
                        crd,
                        dir: Dir::None,
                        grid: Some(self.m_rst),
                    },
                    to: CellIt {
                        crd,
                        dir: Dir::None,
                        grid: None,
                    },
                }
            }
        }
    }

    // MarchingSquares.hpp:307-327
    fn edge(&self, ringvertex: &Coord) -> Edge<'a, Rst> {
        let r = rows(self.m_rst) as i64;
        let c = cols(self.m_rst) as i64;
        let r_1 = r - 1;
        let c_1 = c - 1;

        let mut e = self._edge(ringvertex);
        e.to.dir = e.from.dir;
        e.to.inc(); // ++e.to

        e.from.crd.r = e.from.crd.r.min(r_1);
        e.from.crd.r = e.from.crd.r.max(0i64);
        e.from.crd.c = e.from.crd.c.min(c_1);
        e.from.crd.c = e.from.crd.c.max(0i64);

        e.to.crd.r = e.to.crd.r.min(r);
        e.to.crd.r = e.to.crd.r.max(0i64);
        e.to.crd.c = e.to.crd.c.min(c);
        e.to.crd.c = e.to.crd.c.max(0i64);

        e
    }

    // MarchingSquares.hpp:330-339  explicit Grid(...)
    fn new(rst: &'a Rst, cellsz: Coord, overlap: Coord) -> Grid<'a, Rst> {
        let m_cellsize = cellsz;
        let m_res_1 = Coord {
            r: m_cellsize.r - 1,
            c: m_cellsize.c - 1,
        };
        let m_window = Coord {
            r: if overlap.r < cellsz.r {
                cellsz.r - overlap.r
            } else {
                cellsz.r
            },
            c: if overlap.c < cellsz.c {
                cellsz.c - overlap.c
            } else {
                cellsz.c
            },
        };
        let m_gridsize = Coord {
            r: 2 + (rows(rst) as i64 - overlap.r) / m_window.r,
            c: 2 + (cols(rst) as i64 - overlap.c) / m_window.c,
        };
        let m_tags = vec![0u8; (m_gridsize.r * m_gridsize.c) as usize];

        Grid {
            m_rst: rst,
            m_cellsize,
            m_res_1,
            m_window,
            m_gridsize,
            m_grid_1: Coord::new(),
            m_tags,
        }
    }

    // MarchingSquares.hpp:341-351
    // Go through the cells and mark them with the appropriate tag.
    fn tag_grid(&mut self, isov: TRasterValue<Rst>) {
        // parallel for r
        let n = self.m_tags.len();
        // for_each over the tag indices; mirrors the C++ closure that writes
        // `tag = get_tag_for_cell(coord(idx), isoval)`.
        for_each(0..n, |idx, _seq| {
            let cell = self.coord(idx);
            self.m_tags[idx] = self.get_tag_for_cell(&cell, isov);
        });
    }

    // MarchingSquares.hpp:353-386
    // Scan for the rings on the tagged grid. Each ring vertex stores the
    // sequential index of the cell and the next direction (Dir).
    // This info can be used later to calculate the exact raster coordinate.
    fn scan_rings(&mut self) -> Vec<Ring> {
        let mut rings: Vec<Ring> = Vec::new();
        let mut startidx: usize = 0;
        loop {
            startidx = self.search_start_cell(startidx);
            if startidx >= self.m_tags.len() {
                break;
            }
            let mut ring: Ring = Ring::new();

            let mut idx = startidx;
            let mut prev = Dir::None;
            let mut next = self.next_dir(prev, self.get_tag(idx));

            while next != Dir::None && !self.is_visited(idx, prev) {
                let ringvertex = Coord {
                    r: idx as i64,
                    c: next as u8 as i64,
                };
                ring.push(ringvertex);
                self.set_visited(idx, prev);

                idx = self.seq(&step(&self.coord(idx), next));
                prev = next;
                next = self.next_dir(next, self.get_tag(idx));
            }

            // To prevent infinite loops in case of degenerate input
            if next == Dir::None {
                self.m_tags[startidx] = t_tag(SquareTag::None);
            }

            if ring.len() > 1 {
                ring.pop();
                rings.push(ring);
            }
        }

        rings
    }

    // MarchingSquares.hpp:388-405
    // Calculate the exact raster position from the cells which store the
    // sequantial index of the square and the next direction
    fn interpolate_rings(&self, rings: &mut [Ring], isov: TRasterValue<Rst>) {
        for_each(rings.iter_mut(), |ring, _seq| {
            for ringvertex in ring.iter_mut() {
                let e = self.edge(ringvertex);

                let found = lower_bound(e.from, e.to, isov);
                *ringvertex = found.crd;
            }
        });
    }
}

// MarchingSquares.hpp:408-431
pub fn execute_with_policy<Raster: RasterTraits>(
    raster: &Raster,
    isov: TRasterValue<Raster>,
    mut windowsize: Coord,
) -> Vec<Ring> {
    if rows(raster) == 0 || cols(raster) == 0 {
        return Vec::new();
    }

    let ratio: usize = cols(raster) / rows(raster);

    if windowsize.r == 0 {
        windowsize.r = 2;
    }
    if windowsize.c == 0 {
        windowsize.c = std::cmp::max(2i64, windowsize.r * ratio as i64);
    }

    let overlap = Coord::splat(1);

    let mut grid: Grid<Raster> = Grid::new(raster, windowsize, overlap);

    grid.tag_grid(isov);
    let mut rings: Vec<Ring> = grid.scan_rings();
    grid.interpolate_rings(&mut rings, isov);

    rings
}

// MarchingSquares.hpp:433-439
pub fn execute<Raster: RasterTraits>(
    raster: &Raster,
    isov: TRasterValue<Raster>,
    windowsize: Coord,
) -> Vec<Ring> {
    execute_with_policy(raster, isov, windowsize)
}
