// cli_layer_ranges.cpp — see cli_layer_ranges.hpp.
#include "cli_layer_ranges.hpp"

#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"

namespace slicer_cli {

namespace {

/// The height a range keeps when it states none: the object's own layer height,
/// else the project's (the object inherits the process's layer height when it
/// states none, PrintObject.cpp object_config_from_model_object at both pins).
double range_layer_height(const Slic3r::ModelObject& object, const Slic3r::DynamicPrintConfig& project) {
    if (object.config.has("layer_height"))
        return object.config.opt_float("layer_height");
    return project.opt_float("layer_height");
}

} // namespace

std::vector<FilledRangeHeight> fill_range_layer_heights(Slic3r::Model&                    model,
                                                        const Slic3r::DynamicPrintConfig& project) {
    std::vector<FilledRangeHeight> filled;
    for (Slic3r::ModelObject* object : model.objects) {
        if (object->layer_config_ranges.empty())
            continue;
        const double height = range_layer_height(*object, project);
        if (!(height > 0.))
            continue;
        for (auto& [range, config] : object->layer_config_ranges) {
            if (config.has("layer_height"))
                continue;
            config.set_key_value("layer_height", new Slic3r::ConfigOptionFloat(height));
            filled.push_back({object->name, range.first, range.second, height});
        }
    }
    return filled;
}

} // namespace slicer_cli
