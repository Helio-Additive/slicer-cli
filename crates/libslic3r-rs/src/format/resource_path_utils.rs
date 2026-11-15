//! Port of `libslic3r/Format/ResourcePathUtils.hpp` (namespace `Slic3r::resource_path`).
//!
//! Resolves a resource path that came from outside (a texture reference of a glTF/FBX material, a
//! `file://` URI of a 3MF descriptor) to an existing file, case-insensitively where the exact
//! name does not exist.
//!
//! The C++ works on `boost::filesystem::path`. Its decomposition of a path into elements is not
//! that of `std::path::Path` (a trailing separator is an element ".", "." elements are kept,
//! `//net` is a root name on every platform), so the part of it these functions use is ported in
//! [`boost_path`], on the bytes of the path.
//!
//! C++ reference: `references/BambuStudio/src/libslic3r/Format/ResourcePathUtils.hpp`.

use std::path::{Path, PathBuf};

/// The part of `boost::filesystem::path` (Boost.Filesystem version 3, the default of the Boost the
/// engine is built with) that ResourcePathUtils.hpp uses, on the bytes of the path.
///
/// FIDELITY-NOTE: the POSIX rules are compared with a built Boost (1.90) in
/// `tests/resource_path_utils_native_parity.rs`. The Windows rules (a drive letter and a colon is a
/// root name, a path is absolute only with a root name and a root directory) are written from
/// Boost's documentation of the same version and are not compared with a build; the `\\?\` and
/// `\\.\` prefixes are not recognised. On Windows a path that is not valid Unicode is converted
/// with replacement characters.
#[doc(hidden)]
pub mod boost_path {
    use std::path::{Path, PathBuf};

    /// The bytes of a path: `path::string()`.
    pub fn string(path: &Path) -> Vec<u8> {
        #[cfg(unix)]
        {
            use std::os::unix::ffi::OsStrExt;
            path.as_os_str().as_bytes().to_vec()
        }
        #[cfg(not(unix))]
        {
            path.to_string_lossy().as_bytes().to_vec()
        }
    }

    /// `boost::filesystem::path(std::string)`
    pub fn path(bytes: &[u8]) -> PathBuf {
        #[cfg(unix)]
        {
            use std::os::unix::ffi::OsStrExt;
            PathBuf::from(std::ffi::OsStr::from_bytes(bytes))
        }
        #[cfg(not(unix))]
        {
            PathBuf::from(String::from_utf8_lossy(bytes).into_owned())
        }
    }

    fn is_separator(b: u8) -> bool {
        b == b'/' || (cfg!(windows) && b == b'\\')
    }

    /// The length of the root name at the start of `s`, 0 when there is none.
    fn root_name_size(s: &[u8]) -> usize {
        // `//net`: exactly two separators, then a name up to the next separator.
        if s.len() >= 2 && is_separator(s[0]) && is_separator(s[1]) && (s.len() == 2 || !is_separator(s[2])) {
            let mut end = 2;
            while end < s.len() && !is_separator(s[end]) {
                end += 1;
            }
            return end;
        }
        // A drive: Windows only.
        if cfg!(windows) && s.len() >= 2 && s[0].is_ascii_alphabetic() && s[1] == b':' {
            return 2;
        }
        0
    }

    /// The elements `for (const path &part : p)` visits: the root name, the root directory as "/",
    /// then every name; separators that follow each other count once, and a trailing separator
    /// is an element ".".
    pub fn elements(s: &[u8]) -> Vec<Vec<u8>> {
        let mut out: Vec<Vec<u8>> = Vec::new();
        let mut pos = root_name_size(s);
        if pos > 0 {
            out.push(s[..pos].to_vec());
        }
        if pos < s.len() && is_separator(s[pos]) {
            out.push(b"/".to_vec());
            while pos < s.len() && is_separator(s[pos]) {
                pos += 1;
            }
        }
        while pos < s.len() {
            let start = pos;
            while pos < s.len() && !is_separator(s[pos]) {
                pos += 1;
            }
            out.push(s[start..pos].to_vec());
            if pos < s.len() {
                while pos < s.len() && is_separator(s[pos]) {
                    pos += 1;
                }
                if pos == s.len() {
                    out.push(b".".to_vec());
                }
            }
        }
        out
    }

    /// `path::root_name()`
    pub fn root_name(s: &[u8]) -> Vec<u8> {
        s[..root_name_size(s)].to_vec()
    }

    /// `path::root_directory()`: "/" when there is one.
    pub fn root_directory(s: &[u8]) -> Vec<u8> {
        let pos = root_name_size(s);
        if pos < s.len() && is_separator(s[pos]) {
            b"/".to_vec()
        } else {
            Vec::new()
        }
    }

    /// `path::is_absolute()`: a root directory, and on Windows a root name too.
    pub fn is_absolute(s: &[u8]) -> bool {
        !root_directory(s).is_empty() && (!cfg!(windows) || root_name_size(s) > 0)
    }

    /// `path::filename()`: the last element.
    pub fn filename(s: &[u8]) -> Vec<u8> {
        elements(s).pop().unwrap_or_default()
    }

    /// `lhs / rhs` (`path::operator/=`): a separator between the two unless one is already there.
    pub fn append(lhs: &[u8], rhs: &[u8]) -> Vec<u8> {
        let mut out = lhs.to_vec();
        if rhs.is_empty() {
            return out;
        }
        if !out.is_empty() && !is_separator(out[out.len() - 1]) && !is_separator(rhs[0]) {
            out.push(b'/');
        }
        out.extend_from_slice(rhs);
        out
    }
}

/// `boost::filesystem::exists(p)`.
///
/// FIDELITY-NOTE: the C++ overload throws `filesystem_error` when the status cannot be read for a
/// reason other than the file not being there (no permission to search a directory of the path);
/// here that is "does not exist".
fn exists(path: &[u8]) -> bool {
    boost_path::path(path).exists()
}

/// ResourcePathUtils.hpp:16-23
pub fn ascii_lower_copy(value: &str) -> String {
    // `std::tolower` on every byte, in the "C" locale: only A-Z change.
    value.to_ascii_lowercase()
}

/// `ascii_lower_copy` on the bytes of a path element.
fn ascii_lower_bytes(value: &[u8]) -> Vec<u8> {
    value.to_ascii_lowercase()
}

/// ResourcePathUtils.hpp:25-30 on the bytes of the path.
fn portable_bytes(value: &[u8]) -> Vec<u8> {
    value.iter().map(|b| if *b == b'\\' { b'/' } else { *b }).collect()
}

/// ResourcePathUtils.hpp:25-30
pub fn portable_path_copy(value: &Path) -> PathBuf {
    boost_path::path(&portable_bytes(&boost_path::string(value)))
}

/// ResourcePathUtils.hpp:32-38
pub fn hex_digit_value(ch: u8) -> i32 {
    if ch.is_ascii_digit() {
        return i32::from(ch - b'0');
    }
    if (b'a'..=b'f').contains(&ch) {
        return i32::from(ch - b'a') + 10;
    }
    if (b'A'..=b'F').contains(&ch) {
        return i32::from(ch - b'A') + 10;
    }
    -1
}

/// ResourcePathUtils.hpp:40-60
///
/// Byte-level percent decoding. Per RFC 3986 the %XX byte stream is expected to be
/// UTF-8 when produced from URIs / Assimp aiString; this function performs no
/// transcoding, so the result is bytes.
pub fn percent_decode_copy(value: &str) -> Vec<u8> {
    let value = value.as_bytes();
    let mut decoded: Vec<u8> = Vec::with_capacity(value.len());
    let mut i = 0;
    while i < value.len() {
        if value[i] == b'%' && i + 2 < value.len() {
            let hi = hex_digit_value(value[i + 1]);
            let lo = hex_digit_value(value[i + 2]);
            if hi >= 0 && lo >= 0 {
                decoded.push(((hi << 4) | lo) as u8);
                i += 3;
                continue;
            }
        }
        decoded.push(value[i]);
        i += 1;
    }
    decoded
}

/// ResourcePathUtils.hpp:62-78
pub fn strip_file_uri_prefix_copy(value: &str) -> String {
    let lower = ascii_lower_copy(value);
    if !lower.starts_with("file://") {
        return value.to_string();
    }

    // The seven bytes matched are ASCII, so 7 is a character boundary.
    let mut path: String = value[7..].to_string();
    if ascii_lower_copy(&path).starts_with("localhost/") {
        path.drain(.."localhost".len());
    } else if !path.is_empty() && !path.starts_with('/') {
        path = format!("//{path}");
    }

    // file:///C:/... should become C:/..., while file:///tmp/... keeps /tmp/...
    let bytes = path.as_bytes();
    if bytes.len() >= 3 && bytes[0] == b'/' && bytes[1].is_ascii_alphabetic() && bytes[2] == b':' {
        path.remove(0);
    }
    path
}

/// ResourcePathUtils.hpp:80-93
pub fn file_uri_has_remote_authority(value: &str) -> bool {
    let lower = ascii_lower_copy(value);
    if !lower.starts_with("file://") {
        return false;
    }

    let path = &value[7..];
    if path.is_empty() || path.starts_with('/') {
        return false;
    }

    let authority = match path.find('/') {
        Some(slash) => &path[..slash],
        None => path,
    };
    ascii_lower_copy(authority) != "localhost"
}

/// ResourcePathUtils.hpp:95-102 on the bytes of the path.
fn looks_like_windows_absolute_bytes(path: &[u8]) -> bool {
    let portable = portable_bytes(path);
    portable.len() >= 3 && portable[0].is_ascii_alphabetic() && portable[1] == b':' && portable[2] == b'/'
}

/// ResourcePathUtils.hpp:95-102
pub fn looks_like_windows_absolute_path(path: &Path) -> bool {
    looks_like_windows_absolute_bytes(&boost_path::string(path))
}

/// ResourcePathUtils.hpp:104-108
pub fn filename_from_portable_path(value: &Path) -> PathBuf {
    let portable = portable_bytes(&boost_path::string(value));
    boost_path::path(&boost_path::filename(&portable))
}

/// ResourcePathUtils.hpp:110-136 on the bytes of the paths; an empty result is the C++ `{}`.
fn find_child_case_insensitive_bytes(directory: &[u8], requested_name: &[u8], context: &str) -> Vec<u8> {
    let directory_path = boost_path::path(directory);
    if !directory_path.exists() || !directory_path.is_dir() {
        return Vec::new();
    }

    let requested_lower = ascii_lower_bytes(&boost_path::filename(requested_name));
    let mut matches: Vec<Vec<u8>> = Vec::new();

    // `directory_iterator(directory, ec)`: a directory that cannot be read has no entries, and the
    // loop ends at the first entry that fails.
    if let Ok(entries) = std::fs::read_dir(&directory_path) {
        for entry in entries {
            let Ok(entry) = entry else {
                break;
            };
            let name = boost_path::string(Path::new(&entry.file_name()));
            if ascii_lower_bytes(&name) == requested_lower {
                // `it->path()`: the directory as it was given, and the name.
                matches.push(boost_path::append(directory, &name));
            }
        }
    }

    if matches.len() == 1 {
        return matches.swap_remove(0);
    }

    if matches.len() > 1 {
        log::warn!(
            "{}: ambiguous case-insensitive resource match for {:?} in {:?}",
            context,
            String::from_utf8_lossy(requested_name),
            String::from_utf8_lossy(directory)
        );
    }

    Vec::new()
}

/// ResourcePathUtils.hpp:110-136
///
/// The entry of `directory` whose name is `requested_name` when case is ignored (ASCII), if there
/// is exactly one; an empty path otherwise.
pub fn find_child_case_insensitive(directory: &Path, requested_name: &Path, context: &str) -> PathBuf {
    boost_path::path(&find_child_case_insensitive_bytes(
        &boost_path::string(directory),
        &boost_path::string(requested_name),
        context,
    ))
}

/// ResourcePathUtils.hpp:138-181 on the bytes of the path.
fn resolve_existing_path_case_insensitive_bytes(requested_path: &[u8], context: &str) -> Vec<u8> {
    let normalized_path = portable_bytes(requested_path);

    if normalized_path.is_empty() {
        return Vec::new();
    }

    if exists(&normalized_path) {
        return normalized_path;
    }

    let mut current: Vec<u8> = Vec::new();
    let mut initialized = false;

    let root_name = boost_path::root_name(&normalized_path);
    let root_directory = boost_path::root_directory(&normalized_path);
    for part in boost_path::elements(&normalized_path) {
        if part == root_name || part == root_directory {
            current = boost_path::append(&current, &part);
            initialized = true;
            continue;
        }

        if !initialized {
            current = match std::env::current_dir() {
                Ok(dir) => boost_path::string(&dir),
                Err(_) => Vec::new(),
            };
            initialized = true;
        }

        let exact = boost_path::append(&current, &part);
        if exists(&exact) {
            current = exact;
            continue;
        }

        let matched = find_child_case_insensitive_bytes(&current, &part, context);
        if matched.is_empty() {
            return Vec::new();
        }

        log::info!(
            "{}: resolved resource path case-insensitively from {:?} to {:?}",
            context,
            String::from_utf8_lossy(&exact),
            String::from_utf8_lossy(&matched)
        );
        current = matched;
    }

    if exists(&current) {
        current
    } else {
        Vec::new()
    }
}

/// ResourcePathUtils.hpp:138-181
///
/// `context` is the C++ default argument "resource_path" unless the caller names its own.
pub fn resolve_existing_path_case_insensitive(requested_path: &Path, context: &str) -> PathBuf {
    boost_path::path(&resolve_existing_path_case_insensitive_bytes(&boost_path::string(requested_path), context))
}

/// ResourcePathUtils.hpp:183-190 on the bytes of the paths.
fn resolve_existing_relative_path_case_insensitive_bytes(base_dir: &[u8], resource_path: &[u8], context: &str) -> Vec<u8> {
    let requested =
        if boost_path::is_absolute(resource_path) { resource_path.to_vec() } else { boost_path::append(base_dir, resource_path) };
    resolve_existing_path_case_insensitive_bytes(&requested, context)
}

/// ResourcePathUtils.hpp:183-190
pub fn resolve_existing_relative_path_case_insensitive(base_dir: &Path, resource_path: &Path, context: &str) -> PathBuf {
    boost_path::path(&resolve_existing_relative_path_case_insensitive_bytes(
        &boost_path::string(base_dir),
        &boost_path::string(resource_path),
        context,
    ))
}

/// ResourcePathUtils.hpp:192-236
///
/// Resolve a resource path that originated outside our own code (e.g. a glTF/FBX
/// material texture reference or a file:// URI inside a 3MF descriptor).
///
/// `raw_path` is UTF-8 regardless of host platform: file URIs are UTF-8 by spec, and Assimp
/// aiString uses UTF-8 internally. The C++ default arguments are `context = "resource_path"` and
/// `allow_basename_fallback = true`.
pub fn resolve_external_resource_path(
    base_dir: &Path,
    raw_path: &str,
    context: &str,
    allow_basename_fallback: bool,
) -> PathBuf {
    if raw_path.is_empty() {
        return PathBuf::new();
    }
    let base_dir = boost_path::string(base_dir);

    let remote_file_uri = file_uri_has_remote_authority(raw_path);
    let decoded_path = percent_decode_copy(&strip_file_uri_prefix_copy(raw_path));
    let requested = portable_bytes(&decoded_path);

    let mut resolved = if boost_path::is_absolute(&requested) || looks_like_windows_absolute_bytes(&requested) {
        resolve_existing_path_case_insensitive_bytes(&requested, context)
    } else {
        resolve_existing_relative_path_case_insensitive_bytes(&base_dir, &requested, context)
    };
    if !resolved.is_empty() {
        return boost_path::path(&resolved);
    }

    if !allow_basename_fallback || remote_file_uri {
        return PathBuf::new();
    }

    // `filename_from_portable_path(requested)`
    let basename = boost_path::filename(&portable_bytes(&requested));
    if basename.is_empty() {
        return PathBuf::new();
    }

    resolved = resolve_existing_relative_path_case_insensitive_bytes(&base_dir, &basename, context);
    if !resolved.is_empty() {
        log::info!(
            "{}: resolved resource by basename from {:?} to {:?}",
            context,
            String::from_utf8_lossy(&requested),
            String::from_utf8_lossy(&resolved)
        );
    }
    boost_path::path(&resolved)
}
