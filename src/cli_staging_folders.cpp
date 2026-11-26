#include "cli_staging_folders.hpp"
#include "cli_preset_staging.hpp"

#include <boost/filesystem.hpp>
#include <algorithm>
#include <cstring>

namespace slicer_cli {

std::string RunStagingFolders::add(std::string path) {
    boost::system::error_code mk;
    boost::filesystem::create_directories(path, mk);   // the engine would create it itself (_extract_file_from_archive, bbs_3mf.cpp 2914-2919)
    write_owner_marker(boost::filesystem::path(path));   // whole, never empty: write_owner_marker
    const size_t n = (size_t)count;
    if (n < kMax) {
        const size_t room = sizeof paths[n] - 1;
        const size_t len  = std::min(path.size(), room);
        std::memcpy(paths[n], path.data(), len);
        paths[n][len] = '\0';
        count = n + 1;
    }
    return path;
}

RunStagingFolders::~RunStagingFolders() {
    for (size_t i = 0; i < (size_t)count; ++i)
        remove_staging_folder(boost::filesystem::path(paths[i]));   // markers first
}

} // namespace slicer_cli
