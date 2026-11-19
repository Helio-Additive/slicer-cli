#pragma once

#include <map>
#include <string>
#include "libslic3r/Semver.hpp"

namespace slicer_cli {

/// On the Orca build the maker decides which number the version is in (G2):
/// a project not made by OrcaSlicer is compared with SLIC3R_VERSION, the
/// Bambu base this Orca release is built on (version.inc, 02.06.00.51), and
/// a newer one is only warned about, as the desktop loads it (Plater.cpp
/// 6094-6157 at 31f6803); an OrcaSlicer project is compared with this
/// engine's own version and a newer one is refused, as the official CLI
/// refuses it. The Bambu build compares every file with its own version.
enum class FileVersionVerdict { Ok, WarnedNewerThanBase, Refused };

bool input_is_stl(const std::string& path);
bool input_is_3mf(const std::string& path);
bool read_zip_member(const std::string& archive, const std::string& member,
                            std::string& content, size_t max_size = 64ULL * 1024 * 1024);
bool zip_member_readable(const std::string& archive, const std::string& member);
int count_3mf_plates(const std::string& path);
std::map<std::string, std::string> model_metadata(const std::string& model_xml);
boost::optional<Slic3r::Semver> engine_file_version(const std::map<std::string, std::string>& meta);
std::string engine_version_text();
FileVersionVerdict file_version_verdict(const Slic3r::Semver& file_version,
                                               const std::map<std::string, std::string>& meta);

} // namespace slicer_cli
