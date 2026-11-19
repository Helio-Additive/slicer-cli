#pragma once

#include <cstddef>
#include <string>

namespace slicer_cli {

/// The staging folders this run made: the 3MF load's backup folder and the
/// --export-3mf one. Both are handed to the engine, and 397e505's comment here
/// claimed the engine's removal of them "runs as a UI task the command line
/// never drains" — it does not: ~Model queues a RemoveBackup task
/// (Model.cpp 203-210 / OrcaSlicer 176-183, remove_backup at bbs_3mf.cpp 9030
/// / OrcaSlicer 8604) and the backup manager's worker thread runs it
/// (bbs_3mf.cpp 9140-9144 / OrcaSlicer 8715-8719 start the thread, 9212-9235 /
/// 8788-8803 is the RemoveBackup it runs). A model that still holds this
/// folder when it dies therefore removes it mid-run; detach_run_backup_path
/// takes every model off it, and the run removes it itself as the process
/// ends. The list is a member, not a function-local static: this object's
/// destructor must outlive it. A crash or a kill leaves the folders, as it
/// leaves any temp file — up to the input's embedded G-code, so the sweep
/// below exists: the folders of runs that are gone go at the start of the
/// next run. Nothing else removes them on the way out: a SIGTERM or SIGHUP
/// ends this process through its default action, exactly as SIGKILL does, and
/// both leave the folder to that sweep. A handler would have to remove the
/// folders from signal context, and the process holds ~30 threads: fork()
/// there runs glibc's atfork handlers and takes the malloc arena locks, so a
/// signal that lands while any thread holds one blocks the handler on a
/// futex for ever instead of ending the run ("stale" is judged from the
/// ownership marker below, so the folder the hang would have left is still
/// reclaimed).
///
/// An engine backup folder is marked as owned by the run's PID, and the
/// desktop's own test of "this folder still has its owner" is the rule the
/// sweep follows: the folder is stale once that PID is not a running process
/// of a binary of this package — either engine's, see is_engine_binary_name
/// (the desktop's test, has_restore_data, bbs_3mf.cpp 9447-9466 /
/// OrcaSlicer 9013-9031, compares with its own one program). The desktop writes the PID inside the folder, as
/// lock.txt, when it makes the folder for a load that has no backup path yet
/// (Model.cpp 1123-1130 / OrcaSlicer 986-993); this program's marker is
/// <folder>.owner beside it instead, for the reason add() gives, and add()
/// writes it as the folder is made, so the folder carries its marker from the
/// moment it exists — a run killed before its load reached a backup path
/// would otherwise leave a folder only the age rule below could reclaim.

struct RunStagingFolders {
    /// The 3MF load's folder and the --export-3mf one, in a fixed array: the
    /// set is bounded by the run's steps, and a fixed size keeps add() off the
    /// heap while the run holds its staging folders.
    static constexpr size_t kMax = 4;
    char paths[kMax][4096];   ///< TMPDIR + a relative path, well inside 4096
    size_t count = 0;         ///< Paths registered so far.

    /// Creates `path` with this run's ownership marker, registers it, returns
    /// it.
    /// The marker is `<path>.owner` — a SIBLING of the folder, never a member:
    /// the loader extracts an archive's files under their own names
    /// (_extract_file_from_archive, bbs_3mf.cpp 2904-2919), so a project whose
    /// root holds an entry named lock.txt overwrote a marker kept at
    /// <path>/lock.txt with a PID the archive chose, and the sweep of a second
    /// run then read a dead PID out of a live run's folder and removed that
    /// folder under it. An archive member cannot reach a sibling: extraction
    /// refuses a path that leaves the folder (bbs_3mf.cpp 2909-2912 /
    /// 2804-2807).
    std::string add(std::string path);
    ~RunStagingFolders();
};

} // namespace slicer_cli
