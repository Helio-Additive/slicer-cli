#pragma once

#include <boost/filesystem/path.hpp>

namespace slicer_cli {

boost::filesystem::path staging_owner_marker(const boost::filesystem::path& folder);
void write_owner_marker(const boost::filesystem::path& folder);
void remove_staging_folder(const boost::filesystem::path& folder);
boost::filesystem::path make_owned_temp_dir(const char* prefix);
void remove_owned_temp_dir(const boost::filesystem::path& dir);
void stage_file_copy(const boost::filesystem::path& src,
                            const boost::filesystem::path& dst);
void stage_vendor_dir(const boost::filesystem::path& src,
                             const boost::filesystem::path& dst);
void configure_engine_resources(const char* argv0, bool quiet);

} // namespace slicer_cli
