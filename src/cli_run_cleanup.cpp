#include "cli_run_cleanup.hpp"

#include "cli_run_context.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/miniz_extension.hpp"
#include <set>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <chrono>
#include <csignal>
#include "cli_preset_staging.hpp"
#include "cli_events.hpp"
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

/// True for the names this program stages under, and (the sweep's guard)
/// nothing else: temp_directory_path() is shared, and another program's
/// folder is never this run's to remove. A folder's ownership marker carries
/// the same prefix — it is the folder's path plus ".owner" — so the sweep
/// takes directories only.
static bool is_staging_folder_name(const std::string& name) {
    // RunStagingFolders::add's three, and make_owned_temp_dir's.
    for (const char* prefix : {"slicer_cli_load-", "slicer_cli_preset-", "slicer_cli_export-", "slicer_cli_percent-",
                               "slicer_cli_named-", "slicer_cli_plate-", "slicer_cli_list-", "slicer_cli_project-",
                               "slicer_cli_presets-"})
        if (boost::algorithm::starts_with(name, prefix))
            return true;
    return false;
}

/// True when `name` is one of the file names this package's two binaries run
/// under: the Bambu engine's slicer_cli and the OrcaSlicer engine's
/// slicer_cli-orcaslicer, the names the CI package and the updater's engine
/// convention give them (slicer-cli-ci.yml 1390, 1438-1439), each with the
/// ".exe" suffix Windows keeps. get_process_name returns the executable's own
/// base name but stops ON the separator in front of it — the loop advances to
/// the last '/' ('\\' on Windows) and returns that pointer (utils.cpp
/// 1189-1221) — so the separator is dropped here; the comparison itself is
/// case-insensitive, Windows file names being.
static bool is_engine_binary_name(std::string name) {
    while (!name.empty() && (name.front() == '/' || name.front() == '\\'))
        name.erase(name.begin());
    // Every build knows all three program names: a Bambu or Orca run's temp
    // sweep must recognise (and so never delete) a live Snapmaker run's folders.
    for (const char* binary : {"slicer_cli", "slicer_cli-orcaslicer", "slicer_cli-snapmaker"}) {
        const std::string base(binary);
        if (boost::algorithm::iequals(name, base) || boost::algorithm::iequals(name, base + ".exe"))
            return true;
    }
    return false;
}

/// True when `pid` is a running process of this package — the desktop's test
/// for "this backup folder still has its owner, keep it"
/// (has_restore_data, bbs_3mf.cpp 9447-9466 / OrcaSlicer 9013-9031:
/// get_process_name(pid) == get_process_name(0); on Windows a live pid of
/// another program, or a dead one that OpenProcess refuses, gives a different
/// name or none).
/// The desktop has one program to compare with; this package ships two
/// binaries, and one host may run both against the same TMPDIR, so a live run
/// of EITHER engine owns its folder. Comparing the two names alone made a run
/// of one remove the live folder of a run of the other: the loader still had
/// that folder in its hands, and read back a settings file the sweep had
/// taken away ("parse error at 1:1", rc 255).
static bool staging_owner_alive(int pid) {
    if (pid <= 0)
        return false;
#ifndef _WIN32
    // On Linux and macOS get_process_name() reports a PID that is gone with
    // perror() (utils.cpp 1216-1220 / 1263-1272), which would put engine text
    // on this run's stderr; a PID that is not running cannot be ours, so the
    // probe comes first and the call is only made for a live PID.
    if (::kill(pid, 0) != 0 && errno != EPERM)
        return false;
#endif
    const std::string name = Slic3r::get_process_name(pid);
    // No name at all is not an owner: on Windows OpenProcess refuses a pid
    // this process may not query, live or not.
    if (name.empty())
        return false;
    return name == Slic3r::get_process_name(0) || is_engine_binary_name(name);
}

/// How old a marker-less staging folder must be before the sweep removes it.
/// Every folder this program makes carries an ownership marker beside it, so a
/// folder without one is either from a build older than this fix or is inside
/// the moment between add()'s create_directories and its marker write; the age
/// is what separates the two. An hour is far longer than that window — and
/// than the interval between a folder appearing and the next run sweeping — so
/// a folder this recent is left alone, and the alternative (removing every
/// marker-less folder on sight) is not available: it would delete a live run's
/// folder in that window, which is the one thing this sweep must never do.
static constexpr double kLocklessFolderMinAgeSeconds = 3600.0;

/// Removes the staging folders of runs that are gone, so a tmpfs /tmp does not
/// hold the folders of every killed run. A folder is removed only when its
/// ownership marker names a PID of no binary of this package that is still
/// running (the desktop's own staleness test, for either engine's), or when it
/// has no marker and is over
/// kLocklessFolderMinAgeSeconds old. A live run's folder is left alone, so a
/// second run in the same TMPDIR never disturbs the first.
void sweep_stale_staging_folders() {
    namespace bfs = boost::filesystem;
    boost::system::error_code ec;
    const bfs::path tmp = bfs::temp_directory_path(ec);
    if (ec)
        return;
    const std::string self = Slic3r::get_process_name(0);
    if (self.empty())   // no name of this program to compare against: remove nothing
        return;
    bfs::directory_iterator it(tmp, ec), end;
    if (ec)
        return;
    for (; it != end; it.increment(ec)) {
        if (ec)
            break;
        const bfs::path      folder = it->path();
        if (!is_staging_folder_name(folder.filename().string()))
            continue;
        boost::system::error_code sub;
        if (!bfs::is_directory(folder, sub) || sub)
            continue;   // the markers of other runs' folders sit beside them
        const bfs::path           marker = staging_owner_marker(folder);
        if (bfs::exists(marker, sub) && !sub) {
            int pid = 0;
            {
                boost::filesystem::ifstream in(marker);
                in >> pid;   // a damaged marker leaves pid 0: not an owner, remove
            }
            if (staging_owner_alive(pid))
                continue;
            remove_staging_folder(folder);   // the markers before the folder
            continue;
        }
        // No marker, or one a killed run left half-written (<folder>.owner.tmp,
        // which is taken with the folder): the age rule.
        const std::time_t when = bfs::last_write_time(folder, sub);
        if (sub)
            continue;
        if (std::difftime(std::time(nullptr), (std::time_t)when) < kLocklessFolderMinAgeSeconds)
            continue;
        remove_staging_folder(folder);
    }
}

/// The folder this run's 3MF loads stage their temporary files in. The
/// engine's loader extracts Metadata/project_settings.config to
/// <backup>/_temp_3.config and the embedded presets to <backup>/_temp_2.config
/// and then parses them back (bbs_3mf.cpp
/// _extract_project_config_from_archive 2729 and
/// _extract_project_embedded_presets_from_archive 2758 at BambuStudio
/// 5873b5f; the same functions at 2636 and 2665 in OrcaSlicer 31f6803). One
/// folder for the whole run: the shared $TMPDIR/slicer_cli_backup name let
/// parallel runs read each other's half-written file ("load_from_json: parse
/// /tmp/slicer_cli_backup/_temp_3.config got a parse_error"), and a folder per
/// load would litter the temp dir. "detach" first, so set_backup_path does not
/// remove the folder it replaces (Model::set_backup_path, Model.cpp 1165-1179
/// / 1028-1042) — as f747602 does for the --export-3mf staging folder.
const std::string& run_backup_path(CliRunContext& context) {
    static const std::string path =
        context.run_staging.add((boost::filesystem::temp_directory_path() /
                         boost::filesystem::unique_path("slicer_cli_load-%%%%%%%%")).string());
    return path;
}

/// Gives `model` this run's own backup folder (run_backup_path).
void set_run_backup_path(CliRunContext& context, Slic3r::Model& model) {
    model.set_backup_path("detach");
    model.set_backup_path(run_backup_path(context));
}

/// Creates, inside the run's staging folder, the directory of every file the
/// 3MF holds — nothing else — so the loader's extraction has somewhere to put
/// them.
/// The loader writes an archive entry to <backup>/<its own archive path>
/// (_extract_file_from_archive): the BambuStudio build creates that path's
/// parents first (bbs_3mf.cpp 2914-2919 at 5873b5f), the OrcaSlicer build does
/// not (bbs_3mf.cpp 2798-2818 at 31f6803), so on OrcaSlicer an entry nested
/// below Metadata/ or 3D/Objects/ — a project's Metadata/helio/... files, say —
/// fails with "Error while extract file to temp directory" (2816). Measured on
/// the same corpus 40fd25289c325186, --slice 1, per-run folder: 18 of them on
/// the OrcaSlicer build of 397e505, 0 on the BambuStudio build of the same
/// commit. The folder itself was never missing — only those parents were.
///
/// A member whose path is rooted (a root name or root directory, an X: drive
/// path or a //server UNC path, after the '\\' -> '/' replace) is skipped: no
/// folder is made for it and an event names it. boost's path::operator/ drops
/// backup_root for a rooted right-hand side, so a folder made for one would land
/// outside the run's own folder. The load goes on, as the desktop's does: both
/// loaders extract only members under Metadata/ and Auxiliaries/ (bbs_3mf.cpp
/// 2036-2054 at 5873b5f), and the OrcaSlicer one also skips an absolute path
/// (is_path_within_root, bbs_3mf.cpp 104-118 and 2798-2807 at 31f6803), so a
/// rooted member is never extracted. A member with a ".." component stays a
/// skip, as before.
void create_archive_dirs(const std::string& archive, const std::string& backup_root) {
    mz_zip_archive zip;
    mz_zip_zero_struct(&zip);
    if (!Slic3r::open_zip_reader(&zip, archive))
        return;   // not a 3MF (an STL, a missing file): the load reports it, as before
    std::vector<std::string> dirs;
    const mz_uint count = mz_zip_reader_get_num_files(&zip);
    for (mz_uint i = 0; i < count; ++i) {
        mz_zip_archive_file_stat stat;
        if (!mz_zip_reader_file_stat(&zip, i, &stat))
            continue;
        std::string name(stat.m_filename);
        std::replace(name.begin(), name.end(), '\\', '/');
        // boost on Linux reads neither '//server/x' nor 'X:/x' as rooted (both
        // are relative POSIX paths), so both are named here; the boost root
        // checks cover the rest, and Windows.
        const boost::filesystem::path member(name);
        const char c0 = name.empty() ? '\0' : name[0];
        const bool drive_letter = (c0 >= 'A' && c0 <= 'Z') || (c0 >= 'a' && c0 <= 'z');
        if (name.compare(0, 2, "//") == 0 || (name.size() >= 2 && name[1] == ':' && drive_letter) ||
            member.has_root_name() || member.has_root_directory()) {
            // Once per member per run: --downward-check reads the archive once per plate.
            static std::set<std::string> reported;
            if (reported.insert(archive + '\n' + name).second)
                emit_event({{"event","model_loaded"}, {"tag","ThreeMfMemberSkipped"}, {"path", archive}, {"member", name},
                            {"message", "The 3MF member '" + name + "' has a rooted path; it was skipped, as the desktop skips it, "
                                        "and nothing was written for it outside the run's folder."}});
            continue;
        }
        // the loader refuses a path that leaves the backup folder (bbs_3mf.cpp
        // 2909-2912 / 2804-2807): never create one here either. Like its
        // is_path_within_root (OrcaSlicer bbs_3mf.cpp 112-117), a component
        // that IS ".." is refused, not a name that merely holds two dots
        // (Metadata/rev..1/data stays inside the folder).
        bool parent_step = false;
        for (const boost::filesystem::path& part : member)
            if (part == "..") {
                parent_step = true;
                break;
            }
        if (name.empty() || name[0] == '/' || parent_step)
            continue;
        const size_t slash = name.find_last_of('/');
        if (slash == std::string::npos)
            continue;
        dirs.push_back(name.substr(0, slash));
    }
    mz_zip_reader_end(&zip);
    // Every member is read before the first folder is made, so a rooted member
    // anywhere in the archive leaves no folder behind it.
    for (const std::string& dir : dirs) {
        boost::system::error_code ec;
        boost::filesystem::create_directories(boost::filesystem::path(backup_root) / dir, ec);
    }
}

/// Takes `model` off the run's staging folder once nothing loads into it any
/// more.
/// ~Model removes the folder a model still holds (Slic3r::remove_backup(*this,
/// true), Model.cpp 203-210 / OrcaSlicer 176-183), and that removal is not
/// only queued for the desktop: the manager's own worker thread runs it
/// (_BBS_Backup_Manager starts it in its constructor, bbs_3mf.cpp 9140-9144 /
/// OrcaSlicer 8715-8719; process_task runs RemoveBackup, 9212-9235 / 8788-8803
/// — the log shows the task for this run's own folder on that second thread).
/// Every model of the run holds the one folder of run_backup_path, each
/// plate's copy of the run's model (std::make_unique<Slic3r::Model>(model))
/// included, so a model that dies before the run's work ends takes the folder
/// the run is still using with it. "detach" clears backup_path, so ~Model
/// queues nothing (Model.cpp 1165-1179 / OrcaSlicer 1028-1042) and the folder
/// stands until the run removes it at exit.
void detach_run_backup_path(Slic3r::Model& model) {
    model.set_backup_path("detach");
}

} // namespace slicer_cli
