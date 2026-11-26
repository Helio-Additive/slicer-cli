#include "cli_project_archive.hpp"

#include <boost/algorithm/string.hpp>
#include "libslic3r/miniz_extension.hpp"
#include "libslic3r_version.h"

namespace slicer_cli {

/// The input's kind by its final extension, any case, as both official
/// CLIs and their loaders tell files apart (boost::algorithm::iends_with:
/// BambuStudio.cpp 1873 and Model.cpp 326 at 5873b5f; OrcaSlicer.cpp 1773
/// and Model.cpp 353 at f92a7c0). A folder named "x.stl" or a file named
/// "a.3mf.bak" no longer counts.
bool input_is_stl(const std::string& path) {
    return boost::algorithm::iends_with(path, ".stl");
}

bool input_is_3mf(const std::string& path) {
    return boost::algorithm::iends_with(path, ".3mf");
}

/// Reads one member of a zip archive (case-insensitive name, either slash).
/// False when the archive or the member cannot be read.
bool read_zip_member(const std::string& archive, const std::string& member,
                            std::string& content, size_t max_size) {
    mz_zip_archive zip;
    mz_zip_zero_struct(&zip);
    if (!Slic3r::open_zip_reader(&zip, archive))
        return false;
    bool found = false;
    const mz_uint count = mz_zip_reader_get_num_files(&zip);
    for (mz_uint file_idx = 0; file_idx < count && !found; ++file_idx) {
        mz_zip_archive_file_stat stat;
        if (!mz_zip_reader_file_stat(&zip, file_idx, &stat)) continue;
        std::string name(stat.m_filename);
        std::replace(name.begin(), name.end(), '\\', '/');
        if (!boost::algorithm::iequals(name, member)) continue;
        if (stat.m_uncomp_size > max_size) break;
        content.assign(stat.m_uncomp_size, '\0');
        found = mz_zip_reader_extract_to_mem(&zip, file_idx, content.data(), content.size(), 0);
    }
    Slic3r::close_zip_reader(&zip);
    return found;
}

/// The archive opens as a ZIP and `member` decompresses with a matching CRC.
/// The bytes stream to a discarding sink, so any size is checked without
/// holding it in memory.
bool zip_member_readable(const std::string& archive, const std::string& member) {
    mz_zip_archive zip;
    mz_zip_zero_struct(&zip);
    if (!Slic3r::open_zip_reader(&zip, archive))
        return false;
    bool readable = false;
    const mz_uint count = mz_zip_reader_get_num_files(&zip);
    for (mz_uint file_idx = 0; file_idx < count; ++file_idx) {
        mz_zip_archive_file_stat stat;
        if (!mz_zip_reader_file_stat(&zip, file_idx, &stat)) continue;
        std::string name(stat.m_filename);
        std::replace(name.begin(), name.end(), '\\', '/');
        if (!boost::algorithm::iequals(name, member)) continue;
        auto discard = [](void*, mz_uint64, const void*, size_t n) -> size_t { return n; };
        readable = mz_zip_reader_extract_to_callback(&zip, file_idx, discard, nullptr, 0);
        break;
    }
    Slic3r::close_zip_reader(&zip);
    return readable;
}

/// Number of plates a Bambu/Orca project declares (one <plate> element each in
/// Metadata/model_settings.config, which is what load_bbs_3mf builds its
/// PlateData list from); 0 when the file declares none.
int count_3mf_plates(const std::string& path) {
    std::string settings;
    if (!read_zip_member(path, "Metadata/model_settings.config", settings))
        return 0;
    // Every <plate> start tag, by XML name rules: the name ends at
    // whitespace, '>' or '/', so `<plate >` and a tag with attributes count
    // and `<plates>` does not; comments are skipped.
    int plates = 0;
    for (size_t pos = 0; (pos = settings.find('<', pos)) != std::string::npos; ++pos) {
        if (settings.compare(pos, 4, "<!--") == 0) {
            const size_t end = settings.find("-->", pos + 4);
            if (end == std::string::npos) break;
            pos = end + 2;
            continue;
        }
        if (settings.compare(pos, 6, "<plate") != 0 || pos + 6 >= settings.size()) continue;
        const char next = settings[pos + 6];
        if (next == '>' || next == '/' || next == ' ' || next == '\t' || next == '\r' || next == '\n')
            ++plates;
    }
    return plates;
}

/// Text of every <metadata name="KEY">VALUE</metadata> in a 3MF model part.
std::map<std::string, std::string> model_metadata(const std::string& model_xml) {
    std::map<std::string, std::string> out;
    size_t pos = 0;
    while ((pos = model_xml.find("<metadata", pos)) != std::string::npos) {
        const size_t tag_end = model_xml.find('>', pos);
        if (tag_end == std::string::npos) break;
        const std::string tag = model_xml.substr(pos, tag_end - pos);
        pos = tag_end + 1;
        if (!tag.empty() && tag.back() == '/') continue;
        const size_t n = tag.find("name=\"");
        if (n == std::string::npos) continue;
        const size_t n_end = tag.find('"', n + 6);
        if (n_end == std::string::npos) continue;
        const size_t close = model_xml.find("</metadata>", pos);
        if (close == std::string::npos) break;
        out[tag.substr(n + 6, n_end - n - 6)] = model_xml.substr(pos, close - pos);
        pos = close;
    }
    return out;
}

/// The file version this engine's 3MF loader reads: OrcaSlicer takes the
/// OrcaSlicer tag when present, else the Application version after
/// "BambuStudio-" or "OrcaSlicer-" (bbs_3mf.cpp 1541-1549, 4172-4188 at
/// f92a7c0); BambuStudio reads only "BambuStudio-" (bbs_3mf.cpp 1481 at
/// 5873b5f).
boost::optional<Slic3r::Semver> engine_file_version(const std::map<std::string, std::string>& meta) {
    const auto app = meta.find("Application");
#ifdef ENGINE_SNAPMAKER
    // Snapmaker Orca writes Application "Snapmaker_Orca-<version>"
    // (references/SnapmakerOrca/src/libslic3r/Format/bbs_3mf.cpp:6645-6647 at
    // bac6fd5) and reads that prefix back (3830-3836).
    if (app != meta.end()) {
        if (boost::starts_with(app->second, "BambuStudio-"))   return Slic3r::Semver::parse(app->second.substr(12));
        if (boost::starts_with(app->second, "Snapmaker_Orca-")) return Slic3r::Semver::parse(app->second.substr(15));
    }
#elif defined(ENGINE_ORCA)
    if (const auto orca = meta.find("OrcaSlicer"); orca != meta.end())
        if (auto v = Slic3r::Semver::parse(orca->second)) return v;
    if (app != meta.end()) {
        if (boost::starts_with(app->second, "BambuStudio-")) return Slic3r::Semver::parse(app->second.substr(12));
        if (boost::starts_with(app->second, "OrcaSlicer-"))  return Slic3r::Semver::parse(app->second.substr(11));
    }
#else
    if (app != meta.end() && boost::starts_with(app->second, "BambuStudio-"))
        return Slic3r::Semver::parse(app->second.substr(12));
#endif
    return boost::none;
}

std::string engine_version_text() {
#ifdef ENGINE_ORCA
    return SoftFever_VERSION;
#else
    return SLIC3R_VERSION;
#endif
}

/// The official version gate (BambuStudio.cpp 1906-1911 at 5873b5f,
/// OrcaSlicer.cpp 1806-1810 at f92a7c0): a file whose major.minor is newer
/// than the version it is compared against.
static bool file_newer_than_version(const Slic3r::Semver& file_version, const std::string& version) {
    const auto engine = Slic3r::Semver::parse(version);
    if (!engine) return false;
    return engine->maj() < file_version.maj() ||
           (engine->maj() == file_version.maj() && engine->min() < file_version.min());
}

static bool file_newer_than_engine(const Slic3r::Semver& file_version) {
    return file_newer_than_version(file_version, engine_version_text());
}

/// What the slice does with a 3MF of this version (without
/// --allow-newer-file): the one rule the slice gate and --engine-info share.
FileVersionVerdict file_version_verdict(const Slic3r::Semver& file_version,
                                               const std::map<std::string, std::string>& meta) {
#ifdef ENGINE_ORCA
    const auto app = meta.find("Application");
#ifdef ENGINE_SNAPMAKER
    // A Snapmaker Orca project is this engine's own: it writes Application
    // "Snapmaker_Orca-<version>" (references/SnapmakerOrca/src/libslic3r/Format/bbs_3mf.cpp:6645-6647
    // at bac6fd5), so it is compared with this engine's version, not the base.
    const bool orca_made = (app != meta.end() && boost::starts_with(app->second, "Snapmaker_Orca-")) ||
                           meta.count("OrcaSlicer") != 0 ||
                           (app != meta.end() && boost::starts_with(app->second, "OrcaSlicer-"));
#else
    const bool orca_made = meta.count("OrcaSlicer") != 0 ||
                           (app != meta.end() && boost::starts_with(app->second, "OrcaSlicer-"));
#endif
    if (!orca_made)
        return file_newer_than_version(file_version, SLIC3R_VERSION) ? FileVersionVerdict::WarnedNewerThanBase
                                                                     : FileVersionVerdict::Ok;
#else
    (void)meta;
#endif
    return file_newer_than_engine(file_version) ? FileVersionVerdict::Refused : FileVersionVerdict::Ok;
}

} // namespace slicer_cli
