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
    auto* wipe_x = config.option<ConfigOptionFloats>("wipe_tower_x", true);
    auto* wipe_y = config.option<ConfigOptionFloats>("wipe_tower_y", true);
    // One position per plate; a plate the file has none for takes the first
    // plate's (PartPlate.cpp 5146-5148). --project-settings-as-saved leaves
    // such a list as it is, as the official command line does.
    wipe_x->values.resize(plate_count, wipe_x->values.empty() ? 0.0 : wipe_x->values.front());
    wipe_y->values.resize(plate_count, wipe_y->values.empty() ? 0.0 : wipe_y->values.front());
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

std::vector<std::string> settings_replaced_by_presets(const Slic3r::DynamicPrintConfig& file,
                                                      const Slic3r::DynamicPrintConfig& loaded) {
    using namespace Slic3r;
    const auto same_value_repeated = [](const ConfigOption* a, const ConfigOption* b) {
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
    };
    std::vector<std::string> replaced;
    for (const std::string& key : file.diff(loaded))
        if (key != "different_settings_to_system" && key != "inherits_group" &&
            !same_value_repeated(file.option(key), loaded.option(key)))
            replaced.push_back(key);
    return replaced;
}

} // namespace slicer_cli
