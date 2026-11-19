#include "cli_preset_staging.hpp"

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/algorithm/string.hpp>
#include "libslic3r/Utils.hpp"
#include "cli_printer_catalog.hpp"
#include "cli_snapmaker.hpp"
#include <iostream>
#ifdef _WIN32
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#else
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>
#endif

namespace slicer_cli {

/// The ownership marker of a staging folder: the folder's own path with
/// ".owner" appended, so the marker sits beside the folder and no archive
/// member can land on it (add()).
boost::filesystem::path staging_owner_marker(const boost::filesystem::path& folder) {
    boost::filesystem::path marker = folder;
    marker += ".owner";
    return marker;
}

/// Writes this run's PID as `folder`'s ownership marker, all at once: the PID
/// goes to <folder>.owner.tmp, which is then renamed onto <folder>.owner
/// (rename(2) on POSIX; MoveFileExW with MOVEFILE_REPLACE_EXISTING on Windows,
/// boost 1.90 operations.cpp 236). A marker opened and filled in place is
/// empty for a moment, and another run's sweep reading it then sees no PID,
/// takes it for a damaged marker and removes a live folder. The sweep reads
/// directories only, so it never takes the .tmp file itself, and
/// remove_staging_folder takes a stray one with its folder.
void write_owner_marker(const boost::filesystem::path& folder) {
    boost::filesystem::path tmp = staging_owner_marker(folder);
    tmp += ".tmp";
    {
        boost::filesystem::ofstream out(tmp, std::ios::out | std::ios::trunc);
        out << Slic3r::get_current_pid();
    }
    boost::system::error_code ec;
    boost::filesystem::rename(tmp, staging_owner_marker(folder), ec);
    if (ec)
        boost::filesystem::remove(tmp, ec);
}

/// Removes a staging folder with its marker and any marker left half-written
/// (<folder>.owner.tmp), the markers first: a kill between the removals
/// leaves a folder with no marker, which the sweep's age rule reclaims, never a
/// bare marker no sweep takes.
void remove_staging_folder(const boost::filesystem::path& folder) {
    boost::system::error_code ignored;
    boost::filesystem::path tmp = staging_owner_marker(folder);
    tmp += ".tmp";
    boost::filesystem::remove(staging_owner_marker(folder), ignored);
    boost::filesystem::remove(tmp, ignored);
    boost::filesystem::remove_all(folder, ignored);
}

/// A temp folder of this run's own under temp_directory_path(), named
/// <prefix><8 random characters>, with its ownership marker (<folder>.owner,
/// this run's PID) written as the folder is made, as RunStagingFolders::add
/// writes the load's: a run ended by SIGTERM or SIGKILL skips the destructor
/// that removes the folder, and the next run's sweep
/// (sweep_stale_staging_folders, whose prefix list holds every prefix this is
/// called with) reclaims the folder of a PID that is gone. Throws, as
/// temp_directory_path does on a bad TMPDIR/TMP.
boost::filesystem::path make_owned_temp_dir(const char* prefix) {
    namespace fs = boost::filesystem;
    const fs::path dir = fs::temp_directory_path() / fs::unique_path(std::string(prefix) + "%%%%%%%%");
    fs::create_directories(dir);
    write_owner_marker(dir);
    return dir;
}

/// Removes a folder make_owned_temp_dir made (remove_staging_folder).
void remove_owned_temp_dir(const boost::filesystem::path& dir) {
    if (!dir.empty())
        remove_staging_folder(dir);
}

// Preset staging, both engines: the Bambu 3MF preset rebuild and the
// preset-by-name path (--printer-preset ...) stage a profiles tree the same way.
/// Copy one preset file into the staging tree.
///
/// boost::filesystem::copy_file delegates to the copy_file_range syscall on
/// Linux (selected once from uname in Boost's static init). That syscall
/// reports EXDEV — "Invalid cross-device link" — when source and destination
/// are on different filesystems and the destination has no native copy path:
/// the packaged vendor JSONs on the root filesystem (or a user profiles dir on
/// btrfs) against a TMPDIR on tmpfs (/dev/shm, a tmpfs /tmp) is exactly that
/// case. A stream copy moves the bytes through user space and works there.
///
/// A destination that even the stream copy cannot write is a real staging
/// failure: it throws, and the caller stops the slice instead of continuing on
/// the flat 3MF config.
void stage_file_copy(const boost::filesystem::path& src,
                            const boost::filesystem::path& dst) {
    boost::system::error_code copy_ec;
    boost::filesystem::copy_file(src, dst, copy_ec);
    if (!copy_ec)
        return;

    const std::string why = copy_ec.message();
    // Only a failed data copy falls back. copy_file creates the destination
    // with O_EXCL first and leaves it behind when copy_file_range then fails
    // (Boost.Filesystem 1.84 src/operations.cpp 2847-2934): that file is ours
    // to fill. Any other error, an existing destination (EEXIST) above all,
    // keeps copy_file's refusal to overwrite.
    namespace errc = boost::system::errc;
    const bool data_copy_failed = copy_ec == errc::cross_device_link || copy_ec == errc::not_supported ||
                                  copy_ec == errc::operation_not_supported || copy_ec == errc::invalid_argument ||
                                  copy_ec == errc::function_not_supported;
    if (!data_copy_failed)
        throw std::runtime_error("cannot copy " + src.string() + " to " + dst.string() + ": " + why);
    // The file copy_file created, never a link put in its place.
    boost::system::error_code status_ec;
    if (boost::filesystem::symlink_status(dst, status_ec).type() != boost::filesystem::regular_file)
        throw std::runtime_error("cannot write " + dst.string() + " (copy_file: " + why + ")");
    // Boost's path-taking streams open the native path: the wide path on
    // Windows/MSVC, where path::string() would narrow it to the ANSI code page
    // and lose characters outside it. BambuStudio opens a file it copies by
    // size the same way (src/libslic3r/Format/bbs_3mf.cpp:6668 @5873b5f;
    // OrcaSlicer bbs_3mf.cpp:6385 @31f6803).
    boost::filesystem::ifstream in(src, std::ios::binary);
    boost::filesystem::ofstream out(dst, std::ios::binary | std::ios::trunc);
    if (!in.is_open())
        throw std::runtime_error("cannot read " + src.string() + " (copy_file: " + why + ")");
    if (!out.is_open())
        throw std::runtime_error("cannot write " + dst.string() + " (copy_file: " + why + ")");
    // `out << in.rdbuf()` reads through the stream buffer and never sets
    // `in`'s state, and a filebuf need not tell a read error apart from EOF,
    // so a source read that fails part-way could pass as a short, "complete"
    // copy. Copy in a read loop and count the bytes, then compare the count
    // against the source size — the same size check BambuStudio's check_copy()
    // makes on a copied file (src/libslic3r/utils.cpp:977-1006 @5873b5f).
    std::vector<char> buf(64 * 1024);
    std::uintmax_t copied = 0;
    for (;;) {
        in.read(buf.data(), static_cast<std::streamsize>(buf.size()));
        const std::streamsize got = in.gcount();
        if (got > 0) {
            out.write(buf.data(), got);
            copied += static_cast<std::uintmax_t>(got);
        }
        if (!in || !out)
            break;
    }
    out.close();
    if (out.fail())
        throw std::runtime_error("write failed for " + dst.string() + " (copy_file: " + why + ")");
    if (in.bad())
        throw std::runtime_error("read failed for " + src.string() + " (copy_file: " + why + ")");
    boost::system::error_code size_ec;
    const std::uintmax_t src_size = boost::filesystem::file_size(src, size_ec);
    if (size_ec)
        throw std::runtime_error("cannot size " + src.string() + ": " + size_ec.message()
                                 + " (copy_file: " + why + ")");
    if (copied != src_size)
        throw std::runtime_error("read failed for " + src.string() + ": copied "
                                 + std::to_string(copied) + " of " + std::to_string(src_size)
                                 + " bytes (copy_file: " + why + ")");
}

/// Stage one vendor directory under data_dir/system for the PresetBundle
/// rebuild: a directory symlink where the platform grants one, otherwise a
/// copy. Windows refuses CreateSymbolicLink without the symlink privilege or
/// Developer Mode, and boost then throws; a staging failure must not send the
/// slice to the flat 3MF config.
void stage_vendor_dir(const boost::filesystem::path& src,
                             const boost::filesystem::path& dst) {
    const boost::filesystem::path real_src = boost::filesystem::canonical(src);
    try {
        boost::filesystem::create_directory_symlink(real_src, dst);
        return;
    } catch (const boost::filesystem::filesystem_error&) {
        boost::filesystem::remove(dst);
    }
    boost::filesystem::create_directories(dst);
    for (boost::filesystem::recursive_directory_iterator it(real_src), end; it != end; ++it) {
        const boost::filesystem::path target = dst / boost::filesystem::relative(it->path(), real_src);
        if (boost::filesystem::is_directory(it->path()))
            boost::filesystem::create_directories(target);
        else
            stage_file_copy(it->path(), target);
    }
}
// (end of preset staging helpers)

/// The resources root this engine's libslic3r reads at slice time, or empty
/// when no known layout is present. The two engines' resource files differ
/// (OrcaSlicer's flush/flush_data_standard.txt and info/*.json are not
/// BambuStudio's bytes), so each engine has its own root:
///   Bambu: the parent of the Bambu profiles tree (resources/ in the package,
///          references/BambuStudio/resources in a checkout);
///   Orca:  resources/orca in the package, references/OrcaSlicer/resources in
///          a checkout (build/ or build/<config>/).
static boost::filesystem::path engine_resources_root(const boost::filesystem::path& exe_dir) {
#ifdef ENGINE_ORCA
#ifdef ENGINE_SNAPMAKER
    return slicer_cli::snapmaker_resources_root(exe_dir);
#else
    for (const auto& p : std::vector<boost::filesystem::path>{
        exe_dir / ".." / "references" / "OrcaSlicer" / "resources",
        exe_dir / ".." / ".." / "references" / "OrcaSlicer" / "resources",
        exe_dir / ".." / "resources" / "orca",
        exe_dir / "resources" / "orca",
    }) {
        if (boost::filesystem::exists(p / "info") && boost::filesystem::is_directory(p / "info"))
            return boost::filesystem::canonical(p);
    }
    return boost::filesystem::path();
#endif // ENGINE_SNAPMAKER
#else
    const boost::filesystem::path profiles_dir = engine_profiles_dir(exe_dir);
    if (profiles_dir.empty())
        return boost::filesystem::path();
    return boost::filesystem::canonical(profiles_dir / "..");
#endif
}

/// Point libslic3r at this engine's resources root. The line printed here is
/// the only runtime evidence of which root was chosen; the package tests grep
/// it. It goes to stderr: this runs before the CLI options are read, and the
/// layout-plan subcommand's stdout must stay exactly one JSON document
/// (tests/test_diagnostic_events.sh layout-plan-stdout-stays-one-json-document).
///
/// `quiet` drops the line: `--layout-plan` owns both streams as JSON documents
/// (stdout for the plan, stderr for LayoutErrorV1 — layout_plan.hpp), so that
/// mode configures the root silently.
void configure_engine_resources(const char* argv0, bool quiet) {
    const boost::filesystem::path root = engine_resources_root(engine_executable_dir(argv0));
    if (root.empty()) {
        if (!quiet)
            std::cerr << "  Engine resources: <not found>\n";
        return;
    }
    Slic3r::set_resources_dir(root.string());
    if (!quiet)
        std::cerr << "  Engine resources: " << root.string() << "\n";
}

} // namespace slicer_cli
