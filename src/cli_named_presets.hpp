#pragma once

#include <string>
#include "libslic3r/PrintConfig.hpp"
#include <boost/filesystem/path.hpp>

struct CliOptions;
namespace Slic3r { class DynamicPrintConfig; class Model; }
namespace Slic3r { class PresetBundle; class Preset; class ConfigBase; }

namespace slicer_cli {

struct CliRunContext;
boost::filesystem::path this_engine_profiles_dir(const std::string& argv0);
void apply_file_inherits(const std::string& argv0, const std::string& file, const char* kind,
                                Slic3r::DynamicPrintConfig& config, bool verbose);
std::string config_id_text(const Slic3r::DynamicPrintConfig& config, const char* key);
#ifdef ENGINE_ORCA
std::string config_value_text(const Slic3r::ConfigBase& config, const char* key);
#endif
bool project_presets_shipped(Slic3r::PresetBundle& bundle, const std::string& printer, const std::string& process);
bool load_system_presets(const std::string& argv0, Slic3r::PresetBundle& bundle,
                                const boost::filesystem::path& staging, std::string& error,
                                bool document_mode = false);
bool stage_named_presets_for_project(CliOptions& o, const std::string& dir, int& code, std::string& error);
bool desktop_plate_type_for_printer(const CliOptions& o, const std::string& printer_name,
                                           Slic3r::BedType& out, std::string& why);
bool resolve_named_presets(CliRunContext& context, const CliOptions& o, Slic3r::DynamicPrintConfig& out,
                                  int& code, std::string& error);
int run_list_presets(const CliOptions& o, const std::string& printer_name);

} // namespace slicer_cli
