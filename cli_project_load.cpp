#include "cli_project_load.hpp"

#include <algorithm>

#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/libslic3r.h"

namespace slicer_cli {

#ifdef ENGINE_BAMBU
void project_load_tower_inside_plate(Slic3r::DynamicPrintConfig& config, int plate_index, int plate_count,
                                     const Slic3r::BoundingBoxf& reach, int plate_filaments,
                                     const std::function<Slic3r::Vec3d(int)>& tower_size) {
    using namespace Slic3r;
    if (plate_index < 0)
        return;
    if (plate_index >= plate_count)
        return;
    project_load_tower_positions_per_plate(config, plate_count);
    auto* wipe_x = config.option<ConfigOptionFloats>("wipe_tower_x");
    auto* wipe_y = config.option<ConfigOptionFloats>("wipe_tower_y");
    if (wipe_x == nullptr || wipe_y == nullptr || (int)wipe_x->values.size() <= plate_index ||
        (int)wipe_y->values.size() <= plate_index)
        return;
    Vec3d tower = tower_size(plate_filaments);
    if (is_approx(tower(0), 0.0) || is_approx(tower(1), 0.0))
        tower = tower_size(2);
    if (is_approx(tower(0), 0.0) || is_approx(tower(1), 0.0))
        return;
    // The build volume is the reach area widened by SceneEpsilon
    // (PartPlate::get_build_volume, PartPlate.cpp 3711-3733): a tower the app
    // put on the low edge is saved at 15 - 0.0001.
    const double m = WIPE_TOWER_MARGIN, eps = BuildVolume::SceneEpsilon;
    const double lo_x = reach.min.x() - eps + m, lo_y = reach.min.y() - eps + m;
    const double hi_x = std::max(lo_x, reach.max.x() + eps - tower(0) - m);
    const double hi_y = std::max(lo_y, reach.max.y() + eps - tower(1) - m);
    ConfigOptionFloat wt_x_opt(std::clamp((float)wipe_x->values[plate_index], (float)lo_x, (float)hi_x));
    ConfigOptionFloat wt_y_opt(std::clamp((float)wipe_y->values[plate_index], (float)lo_y, (float)hi_y));
    wipe_x->set_at(&wt_x_opt, plate_index, 0);
    wipe_y->set_at(&wt_y_opt, plate_index, 0);
}
#endif

void project_load_tower_positions_per_plate(Slic3r::DynamicPrintConfig& config, int plate_count) {
    using namespace Slic3r;
    for (const char* key : {"wipe_tower_x", "wipe_tower_y"})
        if (auto* opt = config.option<ConfigOptionFloats>(key); opt != nullptr && !opt->values.empty() &&
                                                                 (int)opt->values.size() < plate_count)
            opt->values.resize(plate_count, opt->values.front());
}

// Two lists that hold one same value, repeated a different number of times
// (one value where the other holds it once per extruder or variant).
static bool same_value_repeated(const Slic3r::ConfigOption* a, const Slic3r::ConfigOption* b) {
    using namespace Slic3r;
    const auto* va = dynamic_cast<const ConfigOptionVectorBase*>(a);
    const auto* vb = dynamic_cast<const ConfigOptionVectorBase*>(b);
    if (va == nullptr || vb == nullptr)
        return false;
    const std::vector<std::string> sa = va->vserialize(), sb = vb->vserialize();
    if (sa.empty() || sb.empty())
        return false;
    const auto all_are = [](const std::vector<std::string>& v, const std::string& x) {
        return std::all_of(v.begin(), v.end(), [&x](const std::string& e) { return e == x; });
    };
    return all_are(sa, sa.front()) && all_are(sb, sa.front());
}

void keep_replaced_printed_differently(std::vector<std::string>& replaced,
                                       const std::map<std::string, std::string>& file_values,
                                       const Slic3r::DynamicPrintConfig& printed,
                                       const std::set<std::string>& command_line) {
    using namespace Slic3r;
    replaced.erase(std::remove_if(replaced.begin(), replaced.end(),
                                  [&](const std::string& key) {
                                      if (command_line.count(key) != 0)
                                          return true;
                                      const ConfigOption* used = printed.option(key);
                                      const auto file = file_values.find(key);
                                      if (used == nullptr || file == file_values.end())
                                          return true;
                                      std::unique_ptr<ConfigOption> stated(used->clone());
                                      try {
                                          if (!stated->deserialize(file->second))
                                              return false;
                                      } catch (const std::exception&) {
                                          return false;
                                      }
                                      return *stated == *used || same_value_repeated(stated.get(), used);
                                  }),
                   replaced.end());
}

std::vector<std::string> settings_replaced_by_presets(const Slic3r::DynamicPrintConfig& file,
                                                      const Slic3r::DynamicPrintConfig& loaded) {
    using namespace Slic3r;
    std::vector<std::string> replaced;
    for (const std::string& key : file.diff(loaded))
        if (key != "different_settings_to_system" && key != "inherits_group" &&
            !same_value_repeated(file.option(key), loaded.option(key)))
            replaced.push_back(key);
    return replaced;
}

} // namespace slicer_cli
