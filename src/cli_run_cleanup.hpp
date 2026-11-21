#pragma once

#include <string>

namespace Slic3r { class Model; }

namespace slicer_cli {

struct CliRunContext;
void sweep_stale_staging_folders();
const std::string& run_backup_path(CliRunContext& context);
void set_run_backup_path(CliRunContext& context, Slic3r::Model& model);
void create_archive_dirs(const std::string& archive, const std::string& backup_root);

void detach_run_backup_path(Slic3r::Model& model);

/// Detaches `model` when the scope that loaded into the run's folder ends, on
/// every path out of it.
struct RunBackupDetach {
    Slic3r::Model& model;
    ~RunBackupDetach() { detach_run_backup_path(model); }
};


} // namespace slicer_cli
