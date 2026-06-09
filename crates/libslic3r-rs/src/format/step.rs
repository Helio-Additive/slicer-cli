//! 1:1 port of `libslic3r/Format/STEP.{hpp,cpp}` (BambuStudio).
//!
//! STEP file loading.
//!
//! The C++ implementation depends heavily on OpenCascade (OCCT) -- a native CAD
//! kernel -- for STEP/BREP parsing and meshing (`STEPCAFControl_Reader`,
//! `BRepMesh_IncrementalMesh`, `XCAFDoc_*`, `TopoDS_*`, etc.). OCCT is a
//! system/dylib dependency and is NOT wasm-safe, so it is intentionally not
//! added here.
//!
//! This module faithfully ports every portable symbol line-by-line:
//!   * `StepPreProcessor` encoding detection / GBK->UTF8 conversion
//!     (`preprocess`, `isUtf8File`, `isUtf8`, `isGBK`, `preNum`).
//!   * `StepProgressIncdicator`, `NamedSolid`, the `Step` control flow and
//!     progress callbacks, and the `Step_Status` / `EncodedType` enums.
//!
//! The OCCT-backed symbols (`getNamedSolids`, the OCCT document handling in the
//! `Step` constructors / `load` / `mesh` / `get_triangle_num*` /
//! `clean_mesh_data`) are BLOCKED on the native CAD kernel and faithfully
//! return the C++ error/empty paths rather than fabricating geometry.

use crate::model::Model;
use crate::utils::{decode_path, temporary_dir};
use crate::{Error, Result};

use log::error;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};

// STEP.cpp:16-20
//   #ifdef _WIN32
//   #define DIR_SEPARATOR '\\'
//   #else
//   #define DIR_SEPARATOR '/'
//   #endif
#[cfg(windows)]
const DIR_SEPARATOR: char = '\\';
#[cfg(not(windows))]
const DIR_SEPARATOR: char = '/';

// ---------------------------------------------------------------------------
// Constants (STEP.hpp:20-24)
// ---------------------------------------------------------------------------

// STEP.hpp:20  const int LOAD_STEP_STAGE_READ_FILE = 0;
pub const LOAD_STEP_STAGE_READ_FILE: i32 = 0;
// STEP.hpp:21  const int LOAD_STEP_STAGE_GET_SOLID = 1;
pub const LOAD_STEP_STAGE_GET_SOLID: i32 = 1;
// STEP.hpp:22  const int LOAD_STEP_STAGE_GET_MESH = 2;
pub const LOAD_STEP_STAGE_GET_MESH: i32 = 2;
// STEP.hpp:23  const int LOAD_STEP_STAGE_NUM = 3;
pub const LOAD_STEP_STAGE_NUM: i32 = 3;
// STEP.hpp:24  const int LOAD_STEP_STAGE_UNIT_NUM = 5;
pub const LOAD_STEP_STAGE_UNIT_NUM: usize = 5;

// ---------------------------------------------------------------------------
// Callback types (STEP.hpp:26-27)
// ---------------------------------------------------------------------------

// STEP.hpp:26
//   typedef std::function<void(int load_stage, int current, int total, bool& cancel)> ImportStepProgressFn;
pub type ImportStepProgressFn = Box<dyn Fn(i32, i32, i32, &mut bool) + Send>;

// STEP.hpp:27
//   typedef std::function<void(bool isUtf8)> StepIsUtf8Fn;
pub type StepIsUtf8Fn = Box<dyn Fn(bool) + Send>;

// ---------------------------------------------------------------------------
// EncodedType (STEP.hpp:56-61)
// ---------------------------------------------------------------------------

// STEP.hpp:56-61
//   enum class EncodedType : unsigned char { UTF8, GBK, OTHER };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodedType {
    Utf8,
    Gbk,
    Other,
}

// ---------------------------------------------------------------------------
// Step_Status (STEP.hpp:91-97)
// ---------------------------------------------------------------------------

// STEP.hpp:91-97
//   enum class Step_Status { LOAD_SUCCESS, LOAD_ERROR, CANCEL, MESH_SUCCESS, MESH_ERROR };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepStatus {
    LoadSuccess,
    LoadError,
    Cancel,
    MeshSuccess,
    MeshError,
}

// ---------------------------------------------------------------------------
// NamedSolid (STEP.hpp:29-37)
// ---------------------------------------------------------------------------

// STEP.hpp:29-37
//   struct NamedSolid {
//       NamedSolid(const TopoDS_Shape& s, const std::string& n) : solid{s}, name{n} {}
//       const TopoDS_Shape solid;
//       const std::string  name;
//       int tri_face_cout = 0;
//   };
//
// `solid` is an OCCT `TopoDS_Shape` (native CAD kernel handle). We retain the
// name and the meshed triangle count; the opaque solid handle has no portable
// representation and is held as `()`.
#[derive(Debug, Clone)]
pub struct NamedSolid {
    /// The solid shape (OCCT `TopoDS_Shape`, BLOCKED on the native CAD kernel).
    pub solid: (),
    /// `const std::string name;`
    pub name: String,
    /// `int tri_face_cout = 0;`
    pub tri_face_cout: i32,
}

impl NamedSolid {
    // STEP.hpp:31-33  NamedSolid(const TopoDS_Shape& s, const std::string& n)
    pub fn new(name: String) -> Self {
        Self {
            solid: (),
            name,
            tri_face_cout: 0,
        }
    }
}

// ---------------------------------------------------------------------------
// StepProgressIncdicator (STEP.hpp:74-86)
// ---------------------------------------------------------------------------

// STEP.hpp:74-86
//   class StepProgressIncdicator : public Message_ProgressIndicator { ... };
//
// In C++ this holds a reference to an external `std::atomic<bool>& should_stop`
// and overrides `UserBreak()` / `Show()`. We model the same: borrow the stop
// flag so `UserBreak()` observes external cancellation.
#[derive(Debug)]
pub struct StepProgressIncdicator<'a> {
    // STEP.hpp:85  std::atomic<bool>& should_stop;
    should_stop: &'a AtomicBool,
}

impl<'a> StepProgressIncdicator<'a> {
    // STEP.hpp:77  StepProgressIncdicator(std::atomic<bool>& stop_flag) : should_stop(stop_flag){}
    pub fn new(stop_flag: &'a AtomicBool) -> Self {
        Self {
            should_stop: stop_flag,
        }
    }

    // STEP.hpp:79  Standard_Boolean UserBreak() override { return should_stop.load(); }
    pub fn user_break(&self) -> bool {
        self.should_stop.load(Ordering::Relaxed)
    }

    // STEP.hpp:81-83  void Show(const Message_ProgressScope&, const Standard_Boolean) override
    //   { std::cout << "Progress: " << GetPosition() << "%" << std::endl; }
    pub fn show(&self, position: f64) {
        println!("Progress: {}%", position);
    }
}

// ---------------------------------------------------------------------------
// StepPreProcessor (STEP.hpp:55-72, STEP.cpp:43-167)
// ---------------------------------------------------------------------------

// STEP.hpp:55-72
//   class StepPreProcessor { ... EncodedType m_encode_type = EncodedType::UTF8; };
#[derive(Debug)]
pub struct StepPreProcessor {
    // STEP.hpp:71  EncodedType m_encode_type = EncodedType::UTF8;
    m_encode_type: EncodedType,
}

impl Default for StepPreProcessor {
    fn default() -> Self {
        Self::new()
    }
}

impl StepPreProcessor {
    pub fn new() -> Self {
        Self {
            // STEP.hpp:71  default is UTF8 for most step file.
            m_encode_type: EncodedType::Utf8,
        }
    }

    // STEP.cpp:43  bool StepPreProcessor::preprocess(const char* path, std::string &output_path)
    pub fn preprocess(&mut self, path: &str, output_path: &mut String) -> Result<bool> {
        // STEP.cpp:45-49
        //   boost::nowide::ifstream infile(path);
        //   if (!infile.good()) {
        //       throw Slic3r::RuntimeError(...);
        //       return false;
        //   }
        let content = std::fs::read(path).map_err(|_| {
            Error::IO("Load step file failed.\nCannot open file for reading.\n".into())
        })?;

        // STEP.cpp:51-52
        //   boost::filesystem::path temp_path(temporary_dir());
        //   std::string temp_step_path = temp_path.string() + "/temp.step";
        let temp_step_path = format!("{}/temp.step", temporary_dir());
        // STEP.cpp:53  boost::nowide::remove(temp_step_path.c_str());
        let _ = std::fs::remove_file(&temp_step_path);
        // STEP.cpp:54  boost::nowide::ofstream temp_file(temp_step_path, std::ios::app);
        let mut temp_bytes: Vec<u8> = Vec::new();

        // STEP.cpp:55-78
        //   std::string temp_line;
        //   while (std::getline(infile, temp_line)) { ... }
        //
        // std::getline splits on '\n' and strips the delimiter (keeps a trailing
        // '\r' if present). Operate on raw bytes per line so GBK byte sequences
        // are not corrupted (a lossy UTF-8 decode would destroy them).
        for line in split_getline_lines(&content) {
            // STEP.cpp:57  if (m_encode_type == EncodedType::UTF8) {
            if self.m_encode_type == EncodedType::Utf8 {
                // STEP.cpp:58-61  continue to judge whether is other type
                if Self::is_utf8_bytes(line) {
                    // STEP.cpp:60  do nothing, but must be checked before checking GBK
                }
                // STEP.cpp:62-65  not utf8, then maybe GBK
                else if Self::is_gbk_bytes(line) {
                    self.m_encode_type = EncodedType::Gbk;
                }
                // STEP.cpp:66-70  not UTF8 and not GBK -> special encoded type we can't handle
                else {
                    self.m_encode_type = EncodedType::Other;
                }
            }
            // STEP.cpp:72-77
            if self.m_encode_type == EncodedType::Gbk {
                // STEP.cpp:73-75  transform to UTF8 format if is GBK
                //   todo: use gbkToUtf8 function to replace
                //   temp_file << decode_path(temp_line.c_str()) << std::endl;
                let line_str = String::from_utf8_lossy(line);
                temp_bytes.extend_from_slice(decode_path(&line_str).as_bytes());
                temp_bytes.push(b'\n');
            } else {
                // STEP.cpp:77  temp_file << temp_line.c_str() << std::endl;
                temp_bytes.extend_from_slice(line);
                temp_bytes.push(b'\n');
            }
        }
        // STEP.cpp:79  temp_file.close();
        // STEP.cpp:80  infile.close();
        // STEP.cpp:81-86
        //   if (m_encode_type == EncodedType::GBK) {
        //       output_path = temp_step_path;
        //   } else {
        //       boost::nowide::remove(temp_step_path.c_str());
        //       output_path = std::string(path);
        //   }
        if self.m_encode_type == EncodedType::Gbk {
            std::fs::write(&temp_step_path, &temp_bytes)
                .map_err(|e| Error::IO(format!("Failed to write temp step file: {}", e)))?;
            *output_path = temp_step_path;
        } else {
            let _ = std::fs::remove_file(&temp_step_path);
            *output_path = path.to_string();
        }

        // STEP.cpp:88  return true;
        Ok(true)
    }

    // STEP.cpp:91  bool StepPreProcessor::isUtf8File(const char* path)
    pub fn is_utf8_file(path: &str) -> Result<bool> {
        // STEP.cpp:93-97
        //   boost::nowide::ifstream infile(path);
        //   if (!infile.good()) { throw ...; return false; }
        let content = std::fs::read(path).map_err(|_| {
            Error::IO("Load step file failed.\nCannot open file for reading.\n".into())
        })?;

        // STEP.cpp:99-105
        //   std::string temp_line;
        //   while (std::getline(infile, temp_line)) {
        //       if (!isUtf8(temp_line)) { infile.close(); return false; }
        //   }
        for line in split_getline_lines(&content) {
            if !Self::is_utf8_bytes(line) {
                // STEP.cpp:102-103  infile.close(); return false;
                return Ok(false);
            }
        }

        // STEP.cpp:107-108  infile.close(); return true;
        Ok(true)
    }

    // STEP.cpp:111  bool StepPreProcessor::isUtf8(const std::string str)
    //
    // Operates over the raw bytes of the (possibly non-UTF-8) string, matching
    // the C++ which indexes the `std::string`'s underlying chars.
    pub fn is_utf8_bytes(str: &[u8]) -> bool {
        // STEP.cpp:113-114  size_t num = 0; int i = 0;
        let mut num: usize;
        let mut i: usize = 0;
        // STEP.cpp:115  while (i < str.length()) {
        while i < str.len() {
            // STEP.cpp:116-117  if ((str[i] & 0x80) == 0x00) { i++; }
            if (str[i] & 0x80) == 0x00 {
                i += 1;
            } else {
                num = Self::pre_num(str[i]);
                // STEP.cpp:118  } else if ((num = preNum(str[i])) > 2) {
                if num > 2 {
                    // STEP.cpp:119  i++;
                    i += 1;
                    // STEP.cpp:120-124  for (int j = 0; j < num - 1; j++) { ... }
                    let mut j = 0;
                    while j < num - 1 {
                        // STEP.cpp:121-122  if ((str[i] & 0xc0) != 0x80) return false;
                        if i >= str.len() || (str[i] & 0xc0) != 0x80 {
                            return false;
                        }
                        // STEP.cpp:123  i++;
                        i += 1;
                        j += 1;
                    }
                } else {
                    // STEP.cpp:125-127  } else { return false; }
                    return false;
                }
            }
        }
        // STEP.cpp:129  return true;
        true
    }

    /// Convenience wrapper matching the public `static bool isUtf8(const std::string)`
    /// signature for callers that hold a `&str` (already-valid UTF-8 sequences).
    pub fn is_utf8(s: &str) -> bool {
        Self::is_utf8_bytes(s.as_bytes())
    }

    // STEP.cpp:132  bool StepPreProcessor::isGBK(const std::string str)
    //
    // The C++ relies on `std::string`'s NUL terminator: `str[length()]` reads
    // '\0' (0x00) which fails the `>= 0x40` check and returns false. We replicate
    // that by treating an out-of-range `str[i+1]` as 0x00.
    fn is_gbk_bytes(str: &[u8]) -> bool {
        // STEP.cpp:133  size_t i = 0;
        let mut i: usize = 0;
        // STEP.cpp:134  while (i < str.length()) {
        while i < str.len() {
            // STEP.cpp:135-137  if (str[i] <= 0x7f) { i++; continue; }
            if str[i] <= 0x7f {
                i += 1;
                continue;
            } else {
                // STEP.cpp:139-146
                //   if (str[i] >= 0x81 && str[i] <= 0xfe &&
                //       str[i + 1] >= 0x40 && str[i + 1] <= 0xfe && str[i + 1] != 0xf7) {
                //       i += 2; continue;
                //   }
                let next: u8 = if i + 1 < str.len() { str[i + 1] } else { 0x00 };
                if str[i] >= 0x81
                    && str[i] <= 0xfe
                    && next >= 0x40
                    && next <= 0xfe
                    && next != 0xf7
                {
                    i += 2;
                    continue;
                } else {
                    // STEP.cpp:147-149  else { return false; }
                    return false;
                }
            }
        }
        // STEP.cpp:152  return true;
        true
    }

    // STEP.cpp:155  int StepPreProcessor::preNum(const unsigned char byte)
    fn pre_num(byte: u8) -> usize {
        // STEP.cpp:156-157  unsigned char mask = 0x80; int num = 0;
        let mut mask: u8 = 0x80;
        let mut num: usize = 0;
        // STEP.cpp:158  for (int i = 0; i < 8; i++) {
        for _ in 0..8 {
            // STEP.cpp:159-161  if ((byte & mask) == mask) { mask = mask >> 1; num++; }
            if (byte & mask) == mask {
                mask >>= 1;
                num += 1;
            } else {
                // STEP.cpp:162-164  else { break; }
                break;
            }
        }
        // STEP.cpp:166  return num;
        num
    }
}

// ---------------------------------------------------------------------------
// getNamedSolids (STEP.cpp:169-227)  -- BLOCKED on OCCT native CAD kernel.
// ---------------------------------------------------------------------------
//
// STEP.cpp:169-227
//   static void getNamedSolids(const TopLoc_Location& location, const std::string& prefix,
//                              unsigned int& id, const Handle(XCAFDoc_ShapeTool) shapeTool,
//                              const TDF_Label label, std::vector<NamedSolid>& namedSolids,
//                              bool isSplitCompound = false);
//
// Recursively walks the XCAF shape tree (`shapeTool->IsReference`,
// `GetReferredShape`, `GetComponents`, `GetShape`), applies the accumulated
// `TopLoc_Location` via `BRepBuilderAPI_Transform`, and emits `TopoDS_Solid` /
// `TopoDS_Compound` / `TopoDS_CompSolid` entries. Every operation requires the
// OpenCascade CAD kernel (a native, non-wasm-safe dylib dependency), so this
// function cannot be ported without that backend.

// ---------------------------------------------------------------------------
// Step (STEP.hpp:88-121, STEP.cpp:413-743)
// ---------------------------------------------------------------------------

// STEP.hpp:88-121  class Step { ... };
//
// The OCCT document handles (`m_app`, `m_doc`, `m_shape_tool`) are part of the
// native CAD kernel and have no portable representation; the geometry-bearing
// methods are blocked on that backend (see per-method notes below).
pub struct Step {
    // STEP.hpp:114  std::string m_path;
    m_path: String,
    // STEP.hpp:115  ImportStepProgressFn m_stepFn;
    m_step_fn: Option<ImportStepProgressFn>,
    // STEP.hpp:116  StepIsUtf8Fn m_utf8Fn;
    m_utf8_fn: Option<StepIsUtf8Fn>,
    // STEP.hpp:120  std::vector<NamedSolid> m_name_solids;
    m_name_solids: Vec<NamedSolid>,
    // STEP.hpp:111  std::atomic<bool> m_stop_mesh;
    pub m_stop_mesh: AtomicBool,
}

impl Step {
    // STEP.cpp:413-419
    //   Step::Step(fs::path path, ImportStepProgressFn stepFn, StepIsUtf8Fn isUtf8Fn) :
    //       m_stepFn(stepFn), m_utf8Fn(isUtf8Fn) {
    //       m_path = path.string();
    //       m_app->NewDocument(TCollection_ExtendedString("BinXCAF"), m_doc);
    //   }
    //
    // The `m_app->NewDocument(...)` OCCT document allocation is blocked on the
    // native CAD kernel; the portable members are initialised here.
    pub fn from_path_buf(
        path: PathBuf,
        step_fn: Option<ImportStepProgressFn>,
        utf8_fn: Option<StepIsUtf8Fn>,
    ) -> Self {
        Self {
            // STEP.cpp:417  m_path = path.string();
            m_path: path.to_string_lossy().into_owned(),
            m_step_fn: step_fn,
            m_utf8_fn: utf8_fn,
            m_name_solids: Vec::new(),
            m_stop_mesh: AtomicBool::new(false),
        }
    }

    // STEP.cpp:421-427
    //   Step::Step(std::string path, ImportStepProgressFn stepFn, StepIsUtf8Fn isUtf8Fn) :
    //       m_path(path), m_stepFn(stepFn), m_utf8Fn(isUtf8Fn) {
    //       m_app->NewDocument(TCollection_ExtendedString("BinXCAF"), m_doc);
    //   }
    pub fn from_path(
        path: impl Into<String>,
        step_fn: Option<ImportStepProgressFn>,
        utf8_fn: Option<StepIsUtf8Fn>,
    ) -> Self {
        Self {
            // STEP.cpp:422  m_path(path)
            m_path: path.into(),
            m_step_fn: step_fn,
            m_utf8_fn: utf8_fn,
            m_name_solids: Vec::new(),
            m_stop_mesh: AtomicBool::new(false),
        }
    }

    // STEP.cpp:434  void Step::update_process(int load_stage, int current, int total, bool& cancel)
    pub fn update_process(&self, load_stage: i32, current: i32, total: i32, cancel: &mut bool) {
        // STEP.cpp:436-438  if (m_stepFn) { m_stepFn(load_stage, current, total, cancel); }
        if let Some(ref f) = self.m_step_fn {
            f(load_stage, current, total, cancel);
        }
    }

    // STEP.cpp:441  Step::Step_Status Step::load()
    //
    // BLOCKED past the UTF-8 gate: the body spins a worker thread running
    // `STEPCAFControl_Reader::ReadFile/Transfer`, `XCAFDoc_DocumentTool::ShapeTool`,
    // `GetFreeShapes` and `getNamedSolids` -- all OCCT CAD-kernel calls. The
    // portable preamble (the UTF-8 check + utf8 callback) is preserved; the OCCT
    // transfer cannot run, so we return `LOAD_ERROR` exactly as the C++ does when
    // the transfer fails.
    pub fn load(&mut self) -> StepStatus {
        // STEP.cpp:443-446
        //   if (!StepPreProcessor::isUtf8File(m_path.c_str()) && m_utf8Fn) {
        //       m_utf8Fn(false);
        //       return Step_Status::LOAD_ERROR;
        //   }
        match StepPreProcessor::is_utf8_file(&self.m_path) {
            Ok(false) => {
                if let Some(ref f) = self.m_utf8_fn {
                    f(false);
                    return StepStatus::LoadError;
                }
            }
            Ok(true) => {}
            Err(_) => {
                // C++ throws RuntimeError from isUtf8File when the file cannot be
                // opened; surface that as a load error.
                return StepStatus::LoadError;
            }
        }

        // STEP.cpp:447-505  OCCT STEPCAFControl_Reader transfer + getNamedSolids.
        error!(
            "STEP loading requires OpenCascade (OCCT) bindings which are not available. File: {}",
            self.m_path
        );
        StepStatus::LoadError
    }

    // STEP.cpp:508  Step::Step_Status Step::mesh(Model* model, bool& is_cancel, bool isSplitCompound,
    //                                            double linear_defletion, double angle_defletion)
    //
    // BLOCKED on OCCT: the worker thread runs `getNamedSolids`,
    // `BRepMesh_IncrementalMesh`, `BRep_Tool::Triangulation`, `TopExp_Explorer`
    // and the `stl_*` triangulation copy -- all CAD-kernel calls. No solids can
    // be meshed, so no `ModelVolume` is ever added and the empty-object cleanup
    // path is taken, returning `MESH_ERROR` exactly as the C++ does for an empty
    // result. We do not add/delete a `ModelObject` here because the meshing that
    // populates it cannot run; mutating `model` would fabricate state the C++
    // never reaches when the transfer produces nothing.
    pub fn mesh(
        &mut self,
        _model: &mut Model,
        is_cancel: &mut bool,
        _is_split_compound: bool,
        _linear_defletion: f64,
        _angle_defletion: f64,
    ) -> StepStatus {
        // STEP.cpp:521-524
        //   ModelObject* new_object = model->add_object();
        //   const char* last_slash = strrchr(m_path.c_str(), DIR_SEPARATOR);
        //   new_object->name.assign((last_slash == nullptr) ? m_path.c_str() : last_slash + 1);
        //   new_object->input_file = m_path.c_str();
        //
        // Compute the object name (path basename) the way the C++ does, so the
        // portable bit is exercised even though the volume-populating mesh is
        // blocked on OCCT.
        let last_slash = self.m_path.rfind(DIR_SEPARATOR);
        let _new_name = match last_slash {
            None => self.m_path.clone(),
            Some(pos) => self.m_path[pos + 1..].to_string(),
        };

        // STEP.cpp:526-672  OCCT worker thread: getNamedSolids + BRepMesh +
        // triangulation copy + add_volume. Cannot run without the CAD kernel.
        error!("STEP meshing requires OpenCascade (OCCT) bindings");
        *is_cancel = false;

        // STEP.cpp:674-678
        //   if (new_object->volumes.size() == 0) {
        //       model->delete_object(new_object);
        //       return Step_Status::MESH_ERROR;
        //   }
        // No volumes were produced -> MESH_ERROR.
        StepStatus::MeshError
    }

    // STEP.cpp:682  void Step::clean_mesh_data()
    //
    // BLOCKED on OCCT: `BRepTools::Clean(name_solid.solid)` operates on the
    // native `TopoDS_Shape`. With no solids materialised the loop is empty.
    pub fn clean_mesh_data(&mut self) {
        // STEP.cpp:684-686
        //   for (const auto& name_solid : m_name_solids) {
        //       BRepTools::Clean(name_solid.solid);
        //   }
        for _name_solid in &self.m_name_solids {
            // BRepTools::Clean -- OCCT, blocked.
        }
    }

    // STEP.cpp:689  unsigned int Step::get_triangle_num(double linear_defletion, double angle_defletion)
    //
    // BLOCKED on OCCT: meshes each solid with `BRepMesh_IncrementalMesh` and sums
    // `BRep_Tool::Triangulation(...)->NbTriangles()`. No solids -> 0, matching the
    // C++ result for an empty / cancelled mesh.
    pub fn get_triangle_num(&mut self, _linear_defletion: f64, _angle_defletion: f64) -> u32 {
        // STEP.cpp:691  unsigned int tri_num = 0;
        let tri_num: u32 = 0;
        // STEP.cpp:692-714  try { clean_mesh_data(); BRepMesh ... } catch { return 0; }
        self.clean_mesh_data();
        // STEP.cpp:716  return tri_num;
        tri_num
    }

    // STEP.cpp:719  unsigned int Step::get_triangle_num_tbb(double linear_defletion, double angle_defletion)
    //
    // BLOCKED on OCCT: parallel `BRepMesh_IncrementalMesh` per solid plus
    // `BRep_Tool::Triangulation`. No solids -> 0.
    pub fn get_triangle_num_tbb(&mut self, _linear_defletion: f64, _angle_defletion: f64) -> u32 {
        // STEP.cpp:721  unsigned int tri_num = 0;
        let mut tri_num: u32 = 0;
        // STEP.cpp:722  clean_mesh_data();
        self.clean_mesh_data();
        // STEP.cpp:723-738  tbb::parallel_for ... m_name_solids[i].tri_face_cout = solids_tri_num;
        //   (OCCT meshing, blocked; tri_face_cout stays 0.)
        // STEP.cpp:739-741  for (int i = 0; ...) tri_num += m_name_solids[i].tri_face_cout;
        for i in 0..self.m_name_solids.len() {
            tri_num += self.m_name_solids[i].tri_face_cout as u32;
        }
        // STEP.cpp:742  return tri_num;
        tri_num
    }

    /// Access the extracted named solids (`m_name_solids`).
    pub fn name_solids(&self) -> &[NamedSolid] {
        &self.m_name_solids
    }
}

impl Drop for Step {
    // STEP.cpp:429-432  Step::~Step() { m_app->Close(m_doc); }
    //
    // `m_app->Close(m_doc)` releases the OCCT document; nothing portable to do.
    fn drop(&mut self) {}
}

/// Split a byte buffer into lines the way `std::getline(infile, line)` does:
/// split on `'\n'` and drop the trailing delimiter (a preceding `'\r'` is kept,
/// matching the C++ behaviour). A trailing line without a final newline still
/// yields a line; a final newline does not produce an extra empty line.
fn split_getline_lines(content: &[u8]) -> impl Iterator<Item = &[u8]> {
    let mut slices: Vec<&[u8]> = Vec::new();
    let mut start = 0;
    let mut idx = 0;
    while idx < content.len() {
        if content[idx] == b'\n' {
            slices.push(&content[start..idx]);
            start = idx + 1;
        }
        idx += 1;
    }
    if start < content.len() {
        slices.push(&content[start..]);
    }
    slices.into_iter()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[test]
    fn test_step_status_variants() {
        assert_ne!(StepStatus::LoadSuccess, StepStatus::LoadError);
        assert_ne!(StepStatus::MeshSuccess, StepStatus::MeshError);
        assert_eq!(StepStatus::Cancel, StepStatus::Cancel);
    }

    #[test]
    fn test_encoded_type_variants() {
        assert_ne!(EncodedType::Utf8, EncodedType::Gbk);
        assert_ne!(EncodedType::Gbk, EncodedType::Other);
    }

    #[test]
    fn test_is_utf8() {
        assert!(StepPreProcessor::is_utf8("Hello world"));
        assert!(StepPreProcessor::is_utf8(""));
        assert!(StepPreProcessor::is_utf8("UTF-8: \u{00e9}\u{00e8}\u{00ea}"));
    }

    #[test]
    fn test_pre_num() {
        assert_eq!(StepPreProcessor::pre_num(0b1100_0000), 2);
        assert_eq!(StepPreProcessor::pre_num(0b1110_0000), 3);
        assert_eq!(StepPreProcessor::pre_num(0b1111_0000), 4);
        assert_eq!(StepPreProcessor::pre_num(0b0111_1111), 0);
    }

    #[test]
    fn test_is_utf8_file() {
        let dir = std::env::temp_dir().join("test_step_utf8");
        std::fs::create_dir_all(&dir).unwrap();
        let file_path = dir.join("test.step");
        {
            let mut f = std::fs::File::create(&file_path).unwrap();
            writeln!(f, "ISO-10303-21;").unwrap();
            writeln!(f, "HEADER;").unwrap();
        }
        assert!(StepPreProcessor::is_utf8_file(file_path.to_str().unwrap()).unwrap());
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn test_step_load_nonexistent() {
        let mut step = Step::from_path("/nonexistent/file.step", None, None);
        assert_eq!(step.load(), StepStatus::LoadError);
    }

    #[test]
    fn test_named_solid() {
        let ns = NamedSolid::new("Part1".to_string());
        assert_eq!(ns.name, "Part1");
        assert_eq!(ns.tri_face_cout, 0);
    }

    #[test]
    fn test_step_progress_indicator() {
        let flag = AtomicBool::new(false);
        let ind = StepProgressIncdicator::new(&flag);
        assert!(!ind.user_break());
        flag.store(true, Ordering::Relaxed);
        assert!(ind.user_break());
    }
}
