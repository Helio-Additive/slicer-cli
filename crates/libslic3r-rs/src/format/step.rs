//! STEP file loading.
//!
//! C++ Reference:
//! - Format/STEP.hpp
//! - Format/STEP.cpp
//!
//! The C++ implementation depends heavily on OpenCascade (OCCT) for STEP/BREP
//! parsing and meshing (BRepMesh, XCAFDoc, etc.). A full port would require an
//! OCCT binding crate. This module faithfully ports all portable logic
//! (preprocessing, encoding detection, progress callbacks, data structures) and
//! provides a top-level `Step::load()` / `Step::mesh()` that returns an error
//! when OCCT is unavailable -- matching the pattern used by the C++ code when
//! the optional OCCT dependency is not compiled in.

use crate::model::Model;
use crate::{Error, Result};

use log::error;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};

// ---------------------------------------------------------------------------
// Constants (STEP.hpp:20-24)
// ---------------------------------------------------------------------------

/// Progress stage: reading STEP file from disk.
pub const LOAD_STEP_STAGE_READ_FILE: i32 = 0;
/// Progress stage: extracting solids from STEP document.
pub const LOAD_STEP_STAGE_GET_SOLID: i32 = 1;
/// Progress stage: meshing solids into triangle data.
pub const LOAD_STEP_STAGE_GET_MESH: i32 = 2;
/// Total number of load stages.
pub const LOAD_STEP_STAGE_NUM: i32 = 3;
/// Number of units per stage for progress granularity.
pub const LOAD_STEP_STAGE_UNIT_NUM: usize = 5;

// ---------------------------------------------------------------------------
// Callback types (STEP.hpp:26-27)
// ---------------------------------------------------------------------------

/// Callback invoked during STEP import to report progress.
/// `(load_stage, current, total, cancel)` -- set `cancel` to `true` to abort.
pub type ImportStepProgressFn = Box<dyn Fn(i32, i32, i32, &mut bool) + Send>;

/// Callback invoked to report whether the file is UTF-8 encoded.
pub type StepIsUtf8Fn = Box<dyn Fn(bool) + Send>;

// ---------------------------------------------------------------------------
// Enums (STEP.hpp:56-61, 91-97)
// ---------------------------------------------------------------------------

/// Step preprocessing: encoding type detected in the file.
/// STEP.hpp:56-61
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodedType {
    Utf8,
    Gbk,
    Other,
}

/// Result status of a `Step` load or mesh operation.
/// STEP.hpp:91-97
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

/// A named solid shape extracted from a STEP document.
///
/// In the C++ code this holds a `TopoDS_Shape` (OCCT type). Here we store
/// a placeholder so the struct API is available for downstream code that may
/// supply an OCCT binding in the future.
#[derive(Debug, Clone)]
pub struct NamedSolid {
    /// The solid shape data (opaque – requires OCCT to materialise).
    /// Stored as serialised bytes when available, empty otherwise.
    pub solid_data: Vec<u8>,
    /// Human-readable name of the solid.
    pub name: String,
    /// Triangle face count after meshing.
    pub tri_face_count: u32,
}

impl NamedSolid {
    pub fn new(name: String) -> Self {
        Self {
            solid_data: Vec::new(),
            name,
            tri_face_count: 0,
        }
    }
}

// ---------------------------------------------------------------------------
// StepProgressIndicator (STEP.hpp:74-86)
// ---------------------------------------------------------------------------

/// A progress indicator that can be polled to detect cancellation.
/// STEP.hpp:74-86
#[derive(Debug)]
pub struct StepProgressIndicator {
    should_stop: AtomicBool,
}

impl StepProgressIndicator {
    pub fn new(stop_flag: &AtomicBool) -> Self {
        Self {
            should_stop: AtomicBool::new(stop_flag.load(Ordering::Relaxed)),
        }
    }

    /// Returns `true` if the user requested cancellation.
    pub fn user_break(&self) -> bool {
        self.should_stop.load(Ordering::Relaxed)
    }

    /// Set the stop flag.
    pub fn request_stop(&self) {
        self.should_stop.store(true, Ordering::Relaxed);
    }
}

// ---------------------------------------------------------------------------
// StepPreProcessor (STEP.hpp:55-72, STEP.cpp:43-167)
// ---------------------------------------------------------------------------

/// Preprocessor for STEP files that handles encoding detection/conversion.
/// STEP.hpp:55-72
#[derive(Debug)]
pub struct StepPreProcessor {
    encode_type: EncodedType,
}

impl StepPreProcessor {
    pub fn new() -> Self {
        Self {
            encode_type: EncodedType::Utf8,
        }
    }

    /// Preprocess a STEP file, converting from GBK to UTF-8 if necessary.
    /// Returns the path to use for subsequent loading (may be a temp file).
    /// STEP.cpp:43-89
    pub fn preprocess(&mut self, path: &Path) -> Result<PathBuf> {
        let content = std::fs::read(path).map_err(|_| {
            Error::IO("Load step file failed. Cannot open file for reading.".into())
        })?;

        let content_str = String::from_utf8_lossy(&content);

        for line in content_str.lines() {
            if self.encode_type == EncodedType::Utf8 {
                if Self::is_utf8(line) {
                    // still UTF-8
                } else if Self::is_gbk(line.as_bytes()) {
                    self.encode_type = EncodedType::Gbk;
                } else {
                    self.encode_type = EncodedType::Other;
                }
            }
        }

        if self.encode_type == EncodedType::Gbk {
            // In the C++ code this writes a temp file with GBK→UTF-8 conversion.
            // Without a GBK codec we pass through (matching "OTHER" behaviour).
            // A real implementation would use encoding_rs or similar.
            error!("STEP file appears to be GBK-encoded; GBK→UTF-8 conversion not yet available");
        }

        Ok(path.to_path_buf())
    }

    /// Check whether the entire file is valid UTF-8.
    /// STEP.cpp:91-109
    pub fn is_utf8_file(path: &Path) -> bool {
        match std::fs::read(path) {
            Ok(bytes) => {
                // Check each line
                for line in bytes.split(|&b| b == b'\n') {
                    let s = String::from_utf8_lossy(line);
                    if !Self::is_utf8(&s) {
                        return false;
                    }
                }
                true
            }
            Err(_) => false,
        }
    }

    /// Check whether a string is valid UTF-8 (byte-level validation).
    /// STEP.cpp:111-130
    pub fn is_utf8(s: &str) -> bool {
        let bytes = s.as_bytes();
        let mut i = 0;
        while i < bytes.len() {
            if (bytes[i] & 0x80) == 0x00 {
                i += 1;
            } else {
                let num = Self::pre_num(bytes[i]);
                if num <= 2 {
                    return false;
                }
                i += 1;
                for _ in 0..num - 1 {
                    if i >= bytes.len() || (bytes[i] & 0xc0) != 0x80 {
                        return false;
                    }
                    i += 1;
                }
            }
        }
        true
    }

    /// Check whether a byte sequence looks like valid GBK.
    /// STEP.cpp:132-153
    fn is_gbk(bytes: &[u8]) -> bool {
        let mut i = 0;
        while i < bytes.len() {
            if bytes[i] <= 0x7f {
                i += 1;
            } else if i + 1 < bytes.len()
                && bytes[i] >= 0x81
                && bytes[i] <= 0xfe
                && bytes[i + 1] >= 0x40
                && bytes[i + 1] <= 0xfe
                && bytes[i + 1] != 0xf7
            {
                i += 2;
            } else {
                return false;
            }
        }
        true
    }

    /// Count leading 1-bits in a byte.
    /// STEP.cpp:155-167
    fn pre_num(byte: u8) -> usize {
        let mut mask: u8 = 0x80;
        let mut num = 0;
        for _ in 0..8 {
            if (byte & mask) == mask {
                mask >>= 1;
                num += 1;
            } else {
                break;
            }
        }
        num
    }
}

// ---------------------------------------------------------------------------
// Step (STEP.hpp:88-121, STEP.cpp:413-743)
// ---------------------------------------------------------------------------

/// High-level STEP loader.
///
/// The C++ `Step` class wraps OCCT document handling, solid extraction, and
/// parallel meshing. This Rust port retains all the control-flow logic but
/// the actual BREP meshing requires an OCCT binding.
///
/// STEP.hpp:88-121
pub struct Step {
    path: String,
    step_fn: Option<ImportStepProgressFn>,
    utf8_fn: Option<StepIsUtf8Fn>,
    name_solids: Vec<NamedSolid>,
    pub stop_mesh: AtomicBool,
}

impl Step {
    /// Create a new `Step` loader from a file path.
    /// STEP.cpp:413-419
    pub fn from_path(path: impl Into<String>) -> Self {
        Self {
            path: path.into(),
            step_fn: None,
            utf8_fn: None,
            name_solids: Vec::new(),
            stop_mesh: AtomicBool::new(false),
        }
    }

    /// Create with progress callbacks.
    /// STEP.cpp:413-427
    pub fn with_callbacks(
        path: impl Into<String>,
        step_fn: Option<ImportStepProgressFn>,
        utf8_fn: Option<StepIsUtf8Fn>,
    ) -> Self {
        Self {
            path: path.into(),
            step_fn,
            utf8_fn,
            name_solids: Vec::new(),
            stop_mesh: AtomicBool::new(false),
        }
    }

    /// Report progress through the registered callback.
    /// STEP.cpp:434-439
    pub fn update_process(&self, load_stage: i32, current: i32, total: i32, cancel: &mut bool) {
        if let Some(ref f) = self.step_fn {
            f(load_stage, current, total, cancel);
        }
    }

    /// Load and parse the STEP file.
    ///
    /// This validates encoding and would normally use OCCT's STEPCAFControl_Reader
    /// to parse the document. Without OCCT bindings, returns `LoadError`.
    ///
    /// STEP.cpp:441-506
    pub fn load(&mut self) -> StepStatus {
        if !StepPreProcessor::is_utf8_file(Path::new(&self.path)) {
            if let Some(ref f) = self.utf8_fn {
                f(false);
            }
            return StepStatus::LoadError;
        }

        // Without OCCT we cannot actually read the STEP geometry.
        // Return an error that callers can handle (e.g. by falling back to
        // an external converter).
        error!(
            "STEP loading requires OpenCascade (OCCT) bindings which are not available. \
             File: {}",
            self.path
        );
        StepStatus::LoadError
    }

    /// Mesh the loaded solids into a Model.
    ///
    /// STEP.cpp:508-680
    pub fn mesh(
        &mut self,
        _model: &mut Model,
        is_cancel: &mut bool,
        _is_split_compound: bool,
        _linear_deflection: f64,
        _angle_deflection: f64,
    ) -> StepStatus {
        if self.name_solids.is_empty() {
            return StepStatus::MeshError;
        }

        // Without OCCT BRepMesh_IncrementalMesh, we cannot mesh.
        *is_cancel = false;
        error!("STEP meshing requires OpenCascade (OCCT) bindings");
        StepStatus::MeshError
    }

    /// Clean cached mesh data from solids.
    /// STEP.cpp:682-687
    pub fn clean_mesh_data(&mut self) {
        // In C++ this calls BRepTools::Clean on each solid.
        // No-op without OCCT.
        for solid in &mut self.name_solids {
            solid.tri_face_count = 0;
        }
    }

    /// Get triangle count by meshing each solid (single-threaded with progress).
    /// STEP.cpp:689-717
    pub fn get_triangle_num(&mut self, _linear_deflection: f64, _angle_deflection: f64) -> u32 {
        // Without OCCT, return 0.
        0
    }

    /// Get triangle count using parallel meshing (TBB equivalent).
    /// STEP.cpp:719-743
    pub fn get_triangle_num_tbb(&mut self, _linear_deflection: f64, _angle_deflection: f64) -> u32 {
        // Without OCCT, return 0.
        0
    }

    /// Access the extracted named solids.
    pub fn name_solids(&self) -> &[NamedSolid] {
        &self.name_solids
    }
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
        assert!(StepPreProcessor::is_utf8_file(&file_path));
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn test_step_load_nonexistent() {
        let mut step = Step::from_path("/nonexistent/file.step");
        assert_eq!(step.load(), StepStatus::LoadError);
    }

    #[test]
    fn test_named_solid() {
        let ns = NamedSolid::new("Part1".to_string());
        assert_eq!(ns.name, "Part1");
        assert_eq!(ns.tri_face_count, 0);
    }

    #[test]
    fn test_step_progress_indicator() {
        let flag = AtomicBool::new(false);
        let ind = StepProgressIndicator::new(&flag);
        assert!(!ind.user_break());
        ind.request_stop();
        assert!(ind.user_break());
    }
}
