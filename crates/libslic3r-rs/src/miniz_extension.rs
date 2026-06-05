//! Faithful 1:1 port of `src/libslic3r/miniz_extension.cpp` (+ `.hpp`).
//!
//! C++ Reference:
//! - miniz_extension.hpp
//! - miniz_extension.cpp
//!
//! This file is a thin wrapper around the miniz C library's `mz_zip_archive`
//! type. The error-string mapping and the small accessor helpers
//! (`MZ_Archive::get_errorstr`, `MZ_Archive::is_alive`) are pure logic and are
//! ported faithfully below.
//!
//! BLOCKED (native dependency, see notes): the file-open/close helpers
//! `open_zip`, `close_zip`, `open_zip_reader`, `open_zip_writer`,
//! `close_zip_reader`, `close_zip_writer` are intrinsically tied to the miniz C
//! API (`mz_zip_reader_init_cfile` / `mz_zip_writer_init_cfile` /
//! `mz_zip_get_cfile` / `mz_zip_reader_end` / `mz_zip_writer_end`) operating on
//! a raw `FILE*`. miniz is a native C dependency that this crate intentionally
//! avoids (the pure-Rust `zip` crate is used instead — see `zipper.rs` and
//! `threemf.rs`). There is no `mz_zip_archive` "init on an open cfile" surface
//! in the `zip` crate, so these helpers cannot be faithfully reproduced without
//! vendoring miniz. They are NOT stubbed here; they are documented as blocked.

// miniz_extension.cpp:1   #include <exception>
// miniz_extension.cpp:3   #include "miniz_extension.hpp"
// miniz_extension.cpp:9   #include "I18N.hpp"

// miniz_extension.cpp:11-13
//! macro used to mark string used at localization,
//! return same string
// #define L(s) Slic3r::I18N::translate(s)
//
// In this parity port `L(s)` maps to `crate::i18n::translate(s)`, which is the
// identity at runtime unless a translation callback is registered
// (mirrors I18N::translate, see I18N.hpp).
use crate::i18n::translate as l;

// miniz_extension.cpp:15  namespace Slic3r {

/// miniz `mz_zip_mode` enum (miniz.h:989-994).
///
/// Needed by `MZ_Archive::is_alive` which compares against
/// `MZ_ZIP_MODE_WRITING_HAS_BEEN_FINALIZED`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum mz_zip_mode {
    /// miniz.h:990
    MZ_ZIP_MODE_INVALID = 0,
    /// miniz.h:991
    MZ_ZIP_MODE_READING = 1,
    /// miniz.h:992
    MZ_ZIP_MODE_WRITING = 2,
    /// miniz.h:993
    MZ_ZIP_MODE_WRITING_HAS_BEEN_FINALIZED = 3,
}

/// miniz error codes — `mz_zip_error` (miniz.h:1018-1053).
///
/// Be sure to update `MZ_Archive::get_errorstr()` if you add or modify this
/// enum (mirrors the matching note in miniz.h:1018). The discriminants match
/// miniz exactly (implicit `+1` increments from `MZ_ZIP_NO_ERROR = 0`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum mz_zip_error {
    /// miniz.h:1020
    MZ_ZIP_NO_ERROR = 0,
    /// miniz.h:1021
    MZ_ZIP_UNDEFINED_ERROR,
    /// miniz.h:1022
    MZ_ZIP_TOO_MANY_FILES,
    /// miniz.h:1023
    MZ_ZIP_FILE_TOO_LARGE,
    /// miniz.h:1024
    MZ_ZIP_UNSUPPORTED_METHOD,
    /// miniz.h:1025
    MZ_ZIP_UNSUPPORTED_ENCRYPTION,
    /// miniz.h:1026
    MZ_ZIP_UNSUPPORTED_FEATURE,
    /// miniz.h:1027
    MZ_ZIP_FAILED_FINDING_CENTRAL_DIR,
    /// miniz.h:1028
    MZ_ZIP_NOT_AN_ARCHIVE,
    /// miniz.h:1029
    MZ_ZIP_INVALID_HEADER_OR_CORRUPTED,
    /// miniz.h:1030
    MZ_ZIP_UNSUPPORTED_MULTIDISK,
    /// miniz.h:1031
    MZ_ZIP_DECOMPRESSION_FAILED,
    /// miniz.h:1032
    MZ_ZIP_COMPRESSION_FAILED,
    /// miniz.h:1033
    MZ_ZIP_UNEXPECTED_DECOMPRESSED_SIZE,
    /// miniz.h:1034
    MZ_ZIP_CRC_CHECK_FAILED,
    /// miniz.h:1035
    MZ_ZIP_UNSUPPORTED_CDIR_SIZE,
    /// miniz.h:1036
    MZ_ZIP_ALLOC_FAILED,
    /// miniz.h:1037
    MZ_ZIP_FILE_OPEN_FAILED,
    /// miniz.h:1038
    MZ_ZIP_FILE_CREATE_FAILED,
    /// miniz.h:1039
    MZ_ZIP_FILE_WRITE_FAILED,
    /// miniz.h:1040
    MZ_ZIP_FILE_READ_FAILED,
    /// miniz.h:1041
    MZ_ZIP_FILE_CLOSE_FAILED,
    /// miniz.h:1042
    MZ_ZIP_FILE_SEEK_FAILED,
    /// miniz.h:1043
    MZ_ZIP_FILE_STAT_FAILED,
    /// miniz.h:1044
    MZ_ZIP_INVALID_PARAMETER,
    /// miniz.h:1045
    MZ_ZIP_INVALID_FILENAME,
    /// miniz.h:1046
    MZ_ZIP_BUF_TOO_SMALL,
    /// miniz.h:1047
    MZ_ZIP_INTERNAL_ERROR,
    /// miniz.h:1048
    MZ_ZIP_FILE_NOT_FOUND,
    /// miniz.h:1049
    MZ_ZIP_ARCHIVE_TOO_LARGE,
    /// miniz.h:1050
    MZ_ZIP_VALIDATION_FAILED,
    /// miniz.h:1051
    MZ_ZIP_WRITE_CALLBACK_FAILED,
    /// miniz.h:1052
    MZ_ZIP_TOTAL_ERRORS,
}

// miniz_extension.cpp:17-64  -- anonymous namespace helpers.
//
// `open_zip` / `close_zip` (and therefore the four public adapters
// `open_zip_reader` / `open_zip_writer` / `close_zip_reader` /
// `close_zip_writer`, miniz_extension.cpp:66-77) operate directly on the
// native miniz `mz_zip_archive` struct + a C `FILE*`:
//
// miniz_extension.cpp:18-51
//   bool open_zip(mz_zip_archive *zip, const char *fname, bool isread)
//   {
//       if (!zip) return false;
//       const char *mode = isread ? "rb" : "wb";
//       FILE *f = nullptr;
//   #if defined(_MSC_VER) || defined(__MINGW64__)
//       f = boost::nowide::fopen(fname, mode);
//   #elif defined(__GNUC__) && defined(_LARGEFILE64_SOURCE)
//       f = fopen64(fname, mode);
//   #else
//       f = fopen(fname, mode);
//   #endif
//       if (!f) {
//           zip->m_last_error = MZ_ZIP_FILE_OPEN_FAILED;
//           return false;
//       }
//       bool res = false;
//       if (isread)
//       {
//           res = mz_zip_reader_init_cfile(zip, f, 0, 0);
//           if (!res)
//               // if we get here it means we tried to open a non-zip file
//               // we need to close the file here because the call to
//               // mz_zip_get_cfile() made into close_zip() returns a null pointer
//               // see: https://github.com/prusa3d/PrusaSlicer/issues/3536
//               fclose(f);
//       }
//       else
//           res = mz_zip_writer_init_cfile(zip, f, 0);
//       return res;
//   }
//
// miniz_extension.cpp:53-63
//   bool close_zip(mz_zip_archive *zip, bool isread)
//   {
//       bool ret = false;
//       if (zip) {
//           FILE *f = mz_zip_get_cfile(zip);
//           ret     = bool(isread ? mz_zip_reader_end(zip)
//                             : mz_zip_writer_end(zip));
//           if (f) fclose(f);
//       }
//       return ret;
//   }
//
// miniz_extension.cpp:66-77
//   bool open_zip_reader(mz_zip_archive *zip, const std::string &fname)
//   { return open_zip(zip, fname.c_str(), true); }
//   bool open_zip_writer(mz_zip_archive *zip, const std::string &fname)
//   { return open_zip(zip, fname.c_str(), false); }
//   bool close_zip_reader(mz_zip_archive *zip) { return close_zip(zip, true); }
//   bool close_zip_writer(mz_zip_archive *zip) { return close_zip(zip, false); }
//
// BLOCKED: these require the native miniz `mz_zip_archive` cfile API. The crate
// uses the pure-Rust `zip` crate (`zip::ZipArchive` for reading, `ZipWriter`
// for writing — see `threemf.rs` and `zipper.rs`), which exposes no equivalent
// "init the archive on an already-open FILE*" surface. Porting them faithfully
// would mean vendoring miniz (a native, non-wasm-safe dependency), which is
// intentionally avoided. Left as documented blocked symbols rather than fakes.

/// C++ class `MZ_Archive` (miniz_extension.hpp:14-31).
///
/// In C++ this owns a full miniz `mz_zip_archive arch;`. Here we model only the
/// two members actually read by the ported accessors — `m_last_error` and
/// `m_zip_mode` — because the full `mz_zip_archive` struct is the native miniz
/// state that this crate does not depend on. The constructor and both accessors
/// are ported faithfully.
#[derive(Debug, Clone)]
#[allow(non_camel_case_types)]
pub struct MZ_Archive {
    /// `arch.m_last_error` (miniz.h:1064). Mirrors the `mz_zip_archive` member
    /// read by `get_errorstr()`.
    /// miniz_extension.hpp:16
    pub last_error: mz_zip_error,

    /// `arch.m_zip_mode`. Mirrors the `mz_zip_archive` member read by
    /// `is_alive()`.
    /// miniz_extension.hpp:16
    pub zip_mode: mz_zip_mode,
}

impl MZ_Archive {
    /// miniz_extension.cpp:79-82
    /// C++: MZ_Archive::MZ_Archive() { mz_zip_zero_struct(&arch); }
    ///
    /// `mz_zip_zero_struct` zeroes the whole `mz_zip_archive`; for the modelled
    /// members that means `m_last_error == MZ_ZIP_NO_ERROR (0)` and
    /// `m_zip_mode == MZ_ZIP_MODE_INVALID (0)`.
    pub fn new() -> Self {
        Self {
            last_error: mz_zip_error::MZ_ZIP_NO_ERROR,
            zip_mode: mz_zip_mode::MZ_ZIP_MODE_INVALID,
        }
    }

    /// miniz_extension.cpp:84-157
    /// C++: std::string MZ_Archive::get_errorstr(mz_zip_error mz_err)
    pub fn get_errorstr_for(mz_err: mz_zip_error) -> String {
        // miniz_extension.cpp:86  switch (mz_err)
        match mz_err {
            // miniz_extension.cpp:88-89
            mz_zip_error::MZ_ZIP_NO_ERROR => "no error".to_string(),
            // miniz_extension.cpp:90-91
            mz_zip_error::MZ_ZIP_UNDEFINED_ERROR => l("undefined error"),
            // miniz_extension.cpp:92-93
            mz_zip_error::MZ_ZIP_TOO_MANY_FILES => l("too many files"),
            // miniz_extension.cpp:94-95
            mz_zip_error::MZ_ZIP_FILE_TOO_LARGE => l("file too large"),
            // miniz_extension.cpp:96-97
            mz_zip_error::MZ_ZIP_UNSUPPORTED_METHOD => l("unsupported method"),
            // miniz_extension.cpp:98-99
            mz_zip_error::MZ_ZIP_UNSUPPORTED_ENCRYPTION => l("unsupported encryption"),
            // miniz_extension.cpp:100-101
            mz_zip_error::MZ_ZIP_UNSUPPORTED_FEATURE => l("unsupported feature"),
            // miniz_extension.cpp:102-103
            mz_zip_error::MZ_ZIP_FAILED_FINDING_CENTRAL_DIR => l("failed finding central directory"),
            // miniz_extension.cpp:104-105
            mz_zip_error::MZ_ZIP_NOT_AN_ARCHIVE => l("not a ZIP archive"),
            // miniz_extension.cpp:106-107
            mz_zip_error::MZ_ZIP_INVALID_HEADER_OR_CORRUPTED => l("invalid header or corrupted"),
            // miniz_extension.cpp:108-109
            mz_zip_error::MZ_ZIP_UNSUPPORTED_MULTIDISK => l("unsupported multidisk"),
            // miniz_extension.cpp:110-111
            mz_zip_error::MZ_ZIP_DECOMPRESSION_FAILED => l("decompression failed"),
            // miniz_extension.cpp:112-113
            mz_zip_error::MZ_ZIP_COMPRESSION_FAILED => l("compression failed"),
            // miniz_extension.cpp:114-115
            mz_zip_error::MZ_ZIP_UNEXPECTED_DECOMPRESSED_SIZE => l("unexpected decompressed size"),
            // miniz_extension.cpp:116-117
            mz_zip_error::MZ_ZIP_CRC_CHECK_FAILED => l("CRC check failed"),
            // miniz_extension.cpp:118-119
            mz_zip_error::MZ_ZIP_UNSUPPORTED_CDIR_SIZE => l("unsupported central directory size"),
            // miniz_extension.cpp:120-121
            mz_zip_error::MZ_ZIP_ALLOC_FAILED => l("allocation failed"),
            // miniz_extension.cpp:122-123
            mz_zip_error::MZ_ZIP_FILE_OPEN_FAILED => l("file open failed"),
            // miniz_extension.cpp:124-125
            mz_zip_error::MZ_ZIP_FILE_CREATE_FAILED => l("file create failed"),
            // miniz_extension.cpp:126-127
            mz_zip_error::MZ_ZIP_FILE_WRITE_FAILED => l("file write failed"),
            // miniz_extension.cpp:128-129
            mz_zip_error::MZ_ZIP_FILE_READ_FAILED => l("file read failed"),
            // miniz_extension.cpp:130-131
            mz_zip_error::MZ_ZIP_FILE_CLOSE_FAILED => l("file close failed"),
            // miniz_extension.cpp:132-133
            mz_zip_error::MZ_ZIP_FILE_SEEK_FAILED => l("file seek failed"),
            // miniz_extension.cpp:134-135
            mz_zip_error::MZ_ZIP_FILE_STAT_FAILED => l("file stat failed"),
            // miniz_extension.cpp:136-137
            mz_zip_error::MZ_ZIP_INVALID_PARAMETER => l("invalid parameter"),
            // miniz_extension.cpp:138-139
            mz_zip_error::MZ_ZIP_INVALID_FILENAME => l("invalid filename"),
            // miniz_extension.cpp:140-141
            mz_zip_error::MZ_ZIP_BUF_TOO_SMALL => l("buffer too small"),
            // miniz_extension.cpp:142-143
            mz_zip_error::MZ_ZIP_INTERNAL_ERROR => l("internal error"),
            // miniz_extension.cpp:144-145
            mz_zip_error::MZ_ZIP_FILE_NOT_FOUND => l("file not found"),
            // miniz_extension.cpp:146-147
            mz_zip_error::MZ_ZIP_ARCHIVE_TOO_LARGE => l("archive too large"),
            // miniz_extension.cpp:148-149
            mz_zip_error::MZ_ZIP_VALIDATION_FAILED => l("validation failed"),
            // miniz_extension.cpp:150-151
            mz_zip_error::MZ_ZIP_WRITE_CALLBACK_FAILED => l("write callback failed"),
            // miniz_extension.cpp:152-153  default: break;
            // (MZ_ZIP_TOTAL_ERRORS and any other value falls through.)
            mz_zip_error::MZ_ZIP_TOTAL_ERRORS => "unknown error".to_string(),
        }
        // miniz_extension.cpp:156  return "unknown error";
        // (handled above via the catch-all `MZ_ZIP_TOTAL_ERRORS` arm; in C++ the
        //  function returns "unknown error" for any value not matched by the
        //  switch.)
    }

    /// miniz_extension.hpp:22-25
    /// C++: std::string get_errorstr() const
    /// C++: { return get_errorstr(arch.m_last_error) + "!"; }
    pub fn get_errorstr(&self) -> String {
        Self::get_errorstr_for(self.last_error) + "!"
    }

    /// miniz_extension.hpp:27-30
    /// C++: bool is_alive() const
    /// C++: { return arch.m_zip_mode != MZ_ZIP_MODE_WRITING_HAS_BEEN_FINALIZED; }
    pub fn is_alive(&self) -> bool {
        self.zip_mode != mz_zip_mode::MZ_ZIP_MODE_WRITING_HAS_BEEN_FINALIZED
    }
}

impl Default for MZ_Archive {
    fn default() -> Self {
        Self::new()
    }
}

// miniz_extension.cpp:159  } // namespace Slic3r

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constructor_zeroes_struct() {
        // mz_zip_zero_struct -> m_last_error == 0, m_zip_mode == 0
        let a = MZ_Archive::new();
        assert_eq!(a.last_error, mz_zip_error::MZ_ZIP_NO_ERROR);
        assert_eq!(a.zip_mode, mz_zip_mode::MZ_ZIP_MODE_INVALID);
    }

    #[test]
    fn errorstr_known_values() {
        crate::i18n::clear_translate_callback();
        assert_eq!(
            MZ_Archive::get_errorstr_for(mz_zip_error::MZ_ZIP_NO_ERROR),
            "no error"
        );
        assert_eq!(
            MZ_Archive::get_errorstr_for(mz_zip_error::MZ_ZIP_NOT_AN_ARCHIVE),
            "not a ZIP archive"
        );
        assert_eq!(
            MZ_Archive::get_errorstr_for(mz_zip_error::MZ_ZIP_FILE_OPEN_FAILED),
            "file open failed"
        );
    }

    #[test]
    fn errorstr_member_appends_bang() {
        crate::i18n::clear_translate_callback();
        let mut a = MZ_Archive::new();
        a.last_error = mz_zip_error::MZ_ZIP_CRC_CHECK_FAILED;
        assert_eq!(a.get_errorstr(), "CRC check failed!");
    }

    #[test]
    fn is_alive_matches_zip_mode() {
        let mut a = MZ_Archive::new();
        // INVALID, READING, WRITING are all "alive"
        assert!(a.is_alive());
        a.zip_mode = mz_zip_mode::MZ_ZIP_MODE_READING;
        assert!(a.is_alive());
        a.zip_mode = mz_zip_mode::MZ_ZIP_MODE_WRITING;
        assert!(a.is_alive());
        // only the finalized mode is not alive
        a.zip_mode = mz_zip_mode::MZ_ZIP_MODE_WRITING_HAS_BEEN_FINALIZED;
        assert!(!a.is_alive());
    }
}
