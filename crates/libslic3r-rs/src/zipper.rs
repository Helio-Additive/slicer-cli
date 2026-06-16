//! ZIP archive creation utilities
//!
//! C++ Reference:
//! - Zipper.hpp (lines 1-96)
//! - Zipper.cpp (lines 1-150)
//!
//! This module provides a high-level interface for creating ZIP archives.
//! It wraps the `zip` crate to provide an API similar to the C++ Zipper class.
//!
//! The C++ implementation uses the miniz library (`mz_zip_writer_add_mem`);
//! this Rust version uses the pure-Rust `zip` crate (deflate via `miniz_oxide`)
//! so the port stays wasm-safe with no native/dylib dependency.
//!
//! DIVERGENCE (documented, not a logic bug): the local-file-header layout and
//! the high-level control flow are mirrored 1:1, and the miniz compression
//! levels are mapped faithfully (MZ_NO_COMPRESSION -> Stored,
//! MZ_BEST_SPEED -> deflate level 1, MZ_BEST_COMPRESSION -> deflate level 9).
//! However the raw deflate bitstream produced by `miniz_oxide` is NOT
//! guaranteed byte-identical to miniz's `tdefl` for the compressed (FAST /
//! TIGHT) levels. Stored entries (NO_COMPRESSION) are byte-identical.
//! Achieving byte-exact deflate output for the compressed levels would require
//! vendoring miniz itself (a native dependency), which is intentionally avoided.

use crate::{Error, Result};
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;
use zip::write::{FileOptions, ZipWriter};
use zip::CompressionMethod;

/// Compression levels for ZIP entries
///
/// Zipper.hpp:14-18
/// C++: enum e_compression {
/// C++:     NO_COMPRESSION,
/// C++:     FAST_COMPRESSION,
/// C++:     TIGHT_COMPRESSION
/// C++: };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Compression {
    /// No compression (store only)
    /// Zipper.hpp:15
    None,

    /// Fast compression (lower compression ratio, faster)
    /// Zipper.hpp:16
    Fast,

    /// Maximum compression (higher ratio, slower)
    /// Zipper.hpp:17
    Tight,
}

impl Default for Compression {
    fn default() -> Self {
        Self::Fast
    }
}

impl Compression {
    /// Convert to zip crate's CompressionMethod
    fn to_zip_method(self) -> CompressionMethod {
        match self {
            Compression::None => CompressionMethod::Stored,
            Compression::Fast => CompressionMethod::Deflated,
            Compression::Tight => CompressionMethod::Deflated,
        }
    }

    /// Get compression level (0-9 for Deflate)
    fn level(self) -> Option<i32> {
        match self {
            Compression::None => None,
            Compression::Fast => Some(1),  // Fast but less compression
            Compression::Tight => Some(9), // Maximum compression
        }
    }
}

/// ZIP archive writer
///
/// Zipper.hpp:10-92
/// C++: class Zipper {
/// C++: private:
/// C++:     class Impl;
/// C++:     std::unique_ptr<Impl> m_impl;
/// C++:     std::string m_data;
/// C++:     std::string m_entry;
/// C++:     e_compression m_compression;
/// C++: public:
/// C++:     explicit Zipper(const std::string& zipfname,
/// C++:                     e_compression level = FAST_COMPRESSION);
/// C++:     ~Zipper();
/// C++:     // ... methods
/// C++: };
pub struct Zipper {
    /// The underlying ZIP writer
    /// Zipper.cpp:24 (m_impl wraps the archive)
    writer: Option<ZipWriter<BufWriter<File>>>,

    /// Filename of the ZIP archive
    /// Zipper.cpp:24
    filename: String,

    /// Current entry name (if any)
    /// Zipper.hpp:24
    current_entry: Option<String>,

    /// Buffer for accumulating data before writing to entry
    /// Zipper.hpp:23
    buffer: Vec<u8>,

    /// Compression level for new entries
    /// Zipper.hpp:25
    compression: Compression,
}

impl Zipper {
    /// Create a new ZIP archive
    ///
    /// Zipper.hpp:29-30
    /// C++: explicit Zipper(const std::string& zipfname,
    /// C++:                 e_compression level = FAST_COMPRESSION);
    ///
    /// Zipper.cpp:44-54
    /// C++: Zipper::Zipper(const std::string &zipfname, e_compression compression)
    /// C++: {
    /// C++:     m_impl.reset(new Impl());
    /// C++:     m_compression = compression;
    /// C++:     m_impl->m_zipname = zipfname;
    /// C++:     memset(&m_impl->arch, 0, sizeof(m_impl->arch));
    /// C++:     if (!open_zip_writer(&m_impl->arch, zipfname)) {
    /// C++:         m_impl->blow_up();
    /// C++:     }
    /// C++: }
    pub fn new<P: AsRef<Path>>(path: P, compression: Compression) -> Result<Self> {
        let path = path.as_ref();
        let file = File::create(path).map_err(|e| {
            Error::IO(format!(
                "Failed to create ZIP file '{}': {}",
                path.display(),
                e
            ))
        })?;

        let writer = ZipWriter::new(BufWriter::new(file));

        Ok(Self {
            writer: Some(writer),
            filename: path.display().to_string(),
            current_entry: None,
            buffer: Vec::new(),
            compression,
        })
    }

    /// Create a ZIP archive with default (fast) compression
    pub fn with_default_compression<P: AsRef<Path>>(path: P) -> Result<Self> {
        Self::new(path, Compression::Fast)
    }

    /// Whether the archive is still open for writing.
    ///
    /// miniz_extension.hpp:27-30 (MZ_Archive::is_alive) / Zipper.cpp:36-39
    /// C++: bool is_alive()
    /// C++: {
    /// C++:     return arch.m_zip_mode != MZ_ZIP_MODE_WRITING_HAS_BEEN_FINALIZED;
    /// C++: }
    ///
    /// In this Rust port the writer is consumed (taken) by `finalize()`, after
    /// which the archive has been finalized; `writer` therefore being `None` is
    /// the equivalent of `m_zip_mode == MZ_ZIP_MODE_WRITING_HAS_BEEN_FINALIZED`.
    fn is_alive(&self) -> bool {
        self.writer.is_some()
    }

    /// Add a new entry (file) to the archive
    ///
    /// Zipper.hpp:48
    /// C++: void add_entry(const std::string& name);
    ///
    /// Zipper.cpp:87-93
    /// C++: void Zipper::add_entry(const std::string &name)
    /// C++: {
    /// C++:     if(!m_impl->is_alive()) return;
    /// C++:     finish_entry(); // finish previous business
    /// C++:     m_entry = name;
    /// C++: }
    pub fn add_entry(&mut self, name: &str) -> Result<()> {
        // Zipper.cpp:89
        if !self.is_alive() {
            return Ok(());
        }

        // Zipper.cpp:91 - finish previous business
        self.finish_entry()?;

        // Zipper.cpp:92 - store the new entry name
        self.current_entry = Some(name.to_string());

        Ok(())
    }

    /// Add a new entry with immediate data
    ///
    /// Zipper.hpp:51
    /// C++: void add_entry(const std::string& name, const void* data, size_t bytes);
    ///
    /// Zipper.cpp:95-112
    /// C++: void Zipper::add_entry(const std::string &name, const void *data, size_t l)
    /// C++: {
    /// C++:     if(!m_impl->is_alive()) return;
    /// C++:     finish_entry();
    /// C++:     mz_uint cmpr = MZ_NO_COMPRESSION;
    /// C++:     switch (m_compression) {
    /// C++:     case NO_COMPRESSION: cmpr = MZ_NO_COMPRESSION; break;
    /// C++:     case FAST_COMPRESSION: cmpr = MZ_BEST_SPEED; break;
    /// C++:     case TIGHT_COMPRESSION: cmpr = MZ_BEST_COMPRESSION; break;
    /// C++:     }
    /// C++:     if(!mz_zip_writer_add_mem(&m_impl->arch, name.c_str(), data, l, cmpr))
    /// C++:         m_impl->blow_up();
    /// C++:     m_entry.clear();
    /// C++:     m_data.clear();
    /// C++: }
    pub fn add_entry_with_data(&mut self, name: &str, data: &[u8]) -> Result<()> {
        // Zipper.cpp:97
        if !self.is_alive() {
            return Ok(());
        }

        // Zipper.cpp:99 - finish previous business
        self.finish_entry()?;

        // Zipper.cpp:100-105 - select compression level from m_compression.
        // The miniz level constants are folded into `create_file_options`:
        //   NO_COMPRESSION   -> MZ_NO_COMPRESSION    (store, level 0)
        //   FAST_COMPRESSION -> MZ_BEST_SPEED        (deflate, level 1)
        //   TIGHT_COMPRESSION-> MZ_BEST_COMPRESSION  (deflate, level 9)
        let options = self.create_file_options();

        // Zipper.cpp:107-108 - mz_zip_writer_add_mem; blow up on failure.
        if let Some(ref mut writer) = self.writer {
            writer.start_file(name, options).map_err(|e| {
                Error::IO(format!(
                    "Failed to start ZIP entry '{}' in '{}': {}",
                    name, self.filename, e
                ))
            })?;

            writer.write_all(data).map_err(|e| {
                Error::IO(format!(
                    "Failed to write data to ZIP entry '{}': {}",
                    name, e
                ))
            })?;
        }

        // Zipper.cpp:110-111 - m_entry.clear(); m_data.clear();
        self.current_entry = None;
        self.buffer.clear();

        Ok(())
    }

    /// Write data to the current entry
    ///
    /// Zipper.hpp:54-76 (operator<< templates)
    /// C++: template<class T> inline
    /// C++: typename std::enable_if<std::is_arithmetic<T>::value, Zipper&>::type
    /// C++: operator<<(T &&val) {
    /// C++:     return this->operator<<(std::to_string(std::forward<T>(val)));
    /// C++: }
    /// C++:
    /// C++: template<class T> inline
    /// C++: typename std::enable_if<!std::is_arithmetic<T>::value, Zipper&>::type
    /// C++: operator<<(T &&val) {
    /// C++:     if(m_data.empty()) m_data = std::forward<T>(val);
    /// C++:     else m_data.append(val);
    /// C++:     return *this;
    /// C++: }
    pub fn write(&mut self, data: &[u8]) -> Result<&mut Self> {
        self.buffer.extend_from_slice(data);
        Ok(self)
    }

    /// Write a string to the current entry
    pub fn write_str(&mut self, s: &str) -> Result<&mut Self> {
        self.write(s.as_bytes())
    }

    /// Finish the current entry and flush buffered data
    ///
    /// Zipper.hpp:78-87
    /// C++: /// Finishing an entry means that subsequent writes will no longer be
    /// C++: /// appended to the previous entry. They will be written into the internal
    /// C++: /// buffer and ones an entry is added, the buffer will bind to the new entry
    /// C++: /// If the buffer was written, but no entry was added, the buffer will be
    /// C++: /// cleared after this call.
    /// C++: ///
    /// C++: /// This method will throw a runtime exception if an error occures. The
    /// C++: /// entry will still be open (with the data intact) but the state of the
    /// C++: /// file is up to minz after the erroneous write.
    /// C++: void finish_entry();
    ///
    /// Zipper.cpp:114-135
    /// C++: void Zipper::finish_entry()
    /// C++: {
    /// C++:     if(!m_impl->is_alive()) return;
    /// C++:     if(!m_data.empty() && !m_entry.empty()) {
    /// C++:         mz_uint compression = MZ_NO_COMPRESSION;
    /// C++:         switch (m_compression) {
    /// C++:         case NO_COMPRESSION: compression = MZ_NO_COMPRESSION; break;
    /// C++:         case FAST_COMPRESSION: compression = MZ_BEST_SPEED; break;
    /// C++:         case TIGHT_COMPRESSION: compression = MZ_BEST_COMPRESSION; break;
    /// C++:         }
    /// C++:         if(!mz_zip_writer_add_mem(&m_impl->arch, m_entry.c_str(),
    /// C++:                                   m_data.c_str(),
    /// C++:                                   m_data.size(),
    /// C++:                                   compression)) m_impl->blow_up();
    /// C++:     }
    /// C++:     m_data.clear();
    /// C++:     m_entry.clear();
    /// C++: }
    pub fn finish_entry(&mut self) -> Result<()> {
        // Zipper.cpp:116
        if !self.is_alive() {
            return Ok(());
        }

        // Zipper.cpp:118 - if(!m_data.empty() && !m_entry.empty())
        // Mirror C++ exactly: both the data buffer AND the entry name must be
        // non-empty. `Some("")` (an entry added with an empty name) must NOT
        // produce a write, matching `!m_entry.empty()` being false.
        if let Some(ref entry_name) = self.current_entry {
            if !self.buffer.is_empty() && !entry_name.is_empty() {
                // Zipper.cpp:119-125 - select compression (folded into options).
                let options = self.create_file_options();

                if let Some(ref mut writer) = self.writer {
                    // Zipper.cpp:127-130 - mz_zip_writer_add_mem; blow up on failure.
                    writer.start_file(entry_name, options).map_err(|e| {
                        Error::IO(format!(
                            "Failed to start ZIP entry '{}' in '{}': {}",
                            entry_name, self.filename, e
                        ))
                    })?;

                    writer.write_all(&self.buffer).map_err(|e| {
                        Error::IO(format!(
                            "Failed to write buffered data to ZIP entry '{}': {}",
                            entry_name, e
                        ))
                    })?;
                }
            }
        }

        // Zipper.cpp:133-134 - m_data.clear(); m_entry.clear();
        self.current_entry = None;
        self.buffer.clear();

        Ok(())
    }

    /// Finalize the ZIP archive
    ///
    /// Zipper.hpp:85
    /// C++: void finalize();
    ///
    /// Zipper.cpp:137-143
    /// C++: void Zipper::finalize()
    /// C++: {
    /// C++:     finish_entry();
    /// C++:     if(m_impl->is_alive()) if(!mz_zip_writer_finalize_archive(&m_impl->arch))
    /// C++:         m_impl->blow_up();
    /// C++: }
    pub fn finalize(&mut self) -> Result<()> {
        // Zipper.cpp:139 - finish any pending entry
        self.finish_entry()?;

        // Zipper.cpp:141-142 - if(m_impl->is_alive()) finalize the archive.
        // Taking the writer leaves us in the finalized state (is_alive() == false).
        if let Some(mut writer) = self.writer.take() {
            writer.finish().map_err(|e| {
                Error::IO(format!(
                    "Failed to finalize ZIP archive '{}': {}",
                    self.filename, e
                ))
            })?;
        }

        Ok(())
    }

    /// Get the filename of the ZIP archive
    ///
    /// Zipper.hpp:87
    /// C++: const std::string & get_filename() const;
    ///
    /// Zipper.cpp:145-148
    /// C++: const std::string &Zipper::get_filename() const
    /// C++: {
    /// C++:     return m_impl->m_zipname;
    /// C++: }
    pub fn filename(&self) -> &str {
        &self.filename
    }

    /// Create FileOptions with appropriate compression settings
    fn create_file_options(&self) -> FileOptions {
        let mut options =
            FileOptions::default().compression_method(self.compression.to_zip_method());

        // Set compression level if using Deflate
        if let Some(level) = self.compression.level() {
            options = options.compression_level(Some(level as i32));
        }

        options
    }
}

impl Drop for Zipper {
    /// Ensure the archive is finalized on drop
    ///
    /// Zipper.cpp:56-71
    /// C++: Zipper::~Zipper()
    /// C++: {
    /// C++:     if(m_impl->is_alive()) {
    /// C++:         // Flush the current entry if not finished yet.
    /// C++:         try { finish_entry(); } catch(...) {
    /// C++:             BOOST_LOG_TRIVIAL(error) << m_impl->formatted_errorstr();
    /// C++:         }
    /// C++:         if(!mz_zip_writer_finalize_archive(&m_impl->arch))
    /// C++:             BOOST_LOG_TRIVIAL(error) << m_impl->formatted_errorstr();
    /// C++:     }
    /// C++:     // The file should be closed no matter what...
    /// C++:     if(!close_zip_writer(&m_impl->arch))
    /// C++:         BOOST_LOG_TRIVIAL(error) << m_impl->formatted_errorstr();
    /// C++: }
    fn drop(&mut self) {
        // Best effort finalization on drop
        let _ = self.finalize();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Read;
    use zip::ZipArchive;

    #[test]
    fn test_zipper_create() {
        let temp_dir = std::env::temp_dir();
        let zip_path = temp_dir.join("test_create.zip");

        let result = Zipper::new(&zip_path, Compression::Fast);
        assert!(result.is_ok());

        let zipper = result.unwrap();
        assert_eq!(zipper.filename(), zip_path.display().to_string());

        std::fs::remove_file(&zip_path).ok();
    }

    #[test]
    fn test_zipper_add_entry_with_data() {
        let temp_dir = std::env::temp_dir();
        let zip_path = temp_dir.join("test_add_entry.zip");

        let mut zipper = Zipper::new(&zip_path, Compression::None).unwrap();

        let data = b"Hello, World!";
        zipper.add_entry_with_data("test.txt", data).unwrap();
        zipper.finalize().unwrap();

        // Verify the ZIP contents
        let file = File::open(&zip_path).unwrap();
        let mut archive = ZipArchive::new(file).unwrap();

        assert_eq!(archive.len(), 1);

        let mut entry = archive.by_name("test.txt").unwrap();
        let mut contents = Vec::new();
        entry.read_to_end(&mut contents).unwrap();

        assert_eq!(contents, data);

        std::fs::remove_file(&zip_path).ok();
    }

    #[test]
    fn test_zipper_buffered_write() {
        let temp_dir = std::env::temp_dir();
        let zip_path = temp_dir.join("test_buffered.zip");

        let mut zipper = Zipper::new(&zip_path, Compression::Fast).unwrap();

        zipper.add_entry("data.txt").unwrap();
        zipper.write_str("Line 1\n").unwrap();
        zipper.write_str("Line 2\n").unwrap();
        zipper.write_str("Line 3\n").unwrap();
        zipper.finish_entry().unwrap();
        zipper.finalize().unwrap();

        // Verify contents
        let file = File::open(&zip_path).unwrap();
        let mut archive = ZipArchive::new(file).unwrap();

        let mut entry = archive.by_name("data.txt").unwrap();
        let mut contents = String::new();
        entry.read_to_string(&mut contents).unwrap();

        assert_eq!(contents, "Line 1\nLine 2\nLine 3\n");

        std::fs::remove_file(&zip_path).ok();
    }

    #[test]
    fn test_zipper_multiple_entries() {
        let temp_dir = std::env::temp_dir();
        let zip_path = temp_dir.join("test_multiple.zip");

        let mut zipper = Zipper::new(&zip_path, Compression::Tight).unwrap();

        zipper
            .add_entry_with_data("file1.txt", b"First file")
            .unwrap();
        zipper
            .add_entry_with_data("file2.txt", b"Second file")
            .unwrap();
        zipper
            .add_entry_with_data("file3.txt", b"Third file")
            .unwrap();
        zipper.finalize().unwrap();

        // Verify all entries exist
        let file = File::open(&zip_path).unwrap();
        let mut archive = ZipArchive::new(file).unwrap();

        assert_eq!(archive.len(), 3);
        assert!(archive.by_name("file1.txt").is_ok());
        assert!(archive.by_name("file2.txt").is_ok());
        assert!(archive.by_name("file3.txt").is_ok());

        std::fs::remove_file(&zip_path).ok();
    }

    #[test]
    fn test_zipper_compression_levels() {
        let temp_dir = std::env::temp_dir();

        // Create test data (repetitive for better compression)
        let data = b"AAAABBBBCCCCDDDD".repeat(100);

        for (name, compression) in [
            ("none.zip", Compression::None),
            ("fast.zip", Compression::Fast),
            ("tight.zip", Compression::Tight),
        ] {
            let zip_path = temp_dir.join(name);
            let mut zipper = Zipper::new(&zip_path, compression).unwrap();
            zipper.add_entry_with_data("data.bin", &data).unwrap();
            zipper.finalize().unwrap();

            // Verify the file was created
            assert!(zip_path.exists());

            std::fs::remove_file(&zip_path).ok();
        }
    }

    #[test]
    fn test_zipper_auto_finalize_on_drop() {
        let temp_dir = std::env::temp_dir();
        let zip_path = temp_dir.join("test_drop.zip");

        {
            let mut zipper = Zipper::new(&zip_path, Compression::Fast).unwrap();
            zipper.add_entry_with_data("test.txt", b"data").unwrap();
            // Drop without explicit finalize
        }

        // Should still be valid
        let file = File::open(&zip_path).unwrap();
        let archive = ZipArchive::new(file);
        assert!(archive.is_ok());

        std::fs::remove_file(&zip_path).ok();
    }

    #[test]
    fn test_zipper_empty_archive() {
        let temp_dir = std::env::temp_dir();
        let zip_path = temp_dir.join("test_empty.zip");

        let mut zipper = Zipper::new(&zip_path, Compression::None).unwrap();
        zipper.finalize().unwrap();

        // Empty ZIP should still be valid
        let file = File::open(&zip_path).unwrap();
        let archive = ZipArchive::new(file).unwrap();
        assert_eq!(archive.len(), 0);

        std::fs::remove_file(&zip_path).ok();
    }
}
