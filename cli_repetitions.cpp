// cli_repetitions.cpp — see cli_repetitions.hpp.
#include "cli_repetitions.hpp"

#include "cli_events.hpp"

namespace slicer_cli {
namespace {

// PartPlate.cpp 68-74 at 5873b5f (the same values at 31f6803).
constexpr float kWipeTowerDefaultX   = 165.f;
constexpr float kWipeTowerDefaultY   = 250.f;
constexpr float kI3WipeTowerDefaultX = 0.f;
constexpr float kI3WipeTowerDefaultY = 250.f;

int printable_objects(const Slic3r::Model& model) {
    int n = 0;
    for (const Slic3r::ModelObject* object : model.objects)
        for (const Slic3r::ModelInstance* inst : object->instances)
            n += inst->printable ? 1 : 0;
    return n;
}

} // namespace

void duplicate_all_instance(Slic3r::Model& model, unsigned dup_count, const std::set<int>& skip_ids) {
    // Every instance of the plate in turn (obj_to_instance_set, in (object,
    // instance) order): a skipped one is made unprintable and not copied, any
    // other is copied (PartPlate::duplicate_all_instance, PartPlate.cpp
    // 2820-2855 at 5873b5f; 2739-2775 at 31f6803). The official copies the
    // instance's whole object, so an object with several instances repeats
    // its other instances too, skipped ones included; each copy here holds
    // only the instance it copies, which gives the official result for an
    // object of one instance and applies --skip-objects per instance.
    std::vector<std::pair<size_t, size_t>> old_list;
    for (size_t obj_id = 0; obj_id < model.objects.size(); ++obj_id)
        for (size_t inst_id = 0; inst_id < model.objects[obj_id]->instances.size(); ++inst_id)
            old_list.emplace_back(obj_id, inst_id);
    for (const auto& [obj_id, inst_id] : old_list) {
        Slic3r::ModelObject* object = model.objects[obj_id];
        Slic3r::ModelInstance* instance = object->instances[inst_id];
        if (!skip_ids.empty() && skip_ids.count(instance->loaded_id)) {
            instance->printable = false;
            continue;
        }
        for (unsigned index = 0; index < dup_count; ++index) {
            Slic3r::ModelObject* copy = model.add_object(*object);
            copy->name = object->name + "_" + std::to_string(index + 1);
            for (int other = int(copy->instances.size()) - 1; other >= 0; --other)
                if (size_t(other) != inst_id)
                    copy->delete_instance(size_t(other));
        }
    }
    for (Slic3r::ModelObject* object : model.objects)
        for (Slic3r::ModelInstance* inst : object->instances) {
            if (!inst->printable)
                continue;
            inst->loaded_id = int(inst->id().id);
            while (!skip_ids.empty() && skip_ids.count(inst->loaded_id))
                inst->loaded_id++;
        }
}

RepetitionsResult arrange_repetitions(Slic3r::Model& model, Slic3r::DynamicPrintConfig& config, int plate_index,
                                      int duplicate_count, const std::set<int>& skip_ids, const ArrangeAttempt& arrange) {
    RepetitionsResult result;
    const Slic3r::Model original_model = model;
    const Slic3r::DynamicPrintConfig original_config = config;
    const int originals = printable_objects(model);
    int low = 0, up = duplicate_count;
    bool first_run = true;
    for (;;) {
        if (!first_run) {
            model  = original_model;
            config = original_config;
        }
        first_run = false;
        duplicate_all_instance(model, unsigned(duplicate_count), skip_ids);
        // The copies start the tower at the printer's default corner
        // (BambuStudio.cpp 5773-5781; OrcaSlicer.cpp 5033-5041).
        {
            float x = kWipeTowerDefaultX, y = kWipeTowerDefaultY;
            if (const auto* s = config.option<Slic3r::ConfigOptionEnum<Slic3r::PrinterStructure>>("printer_structure");
                s && s->value == Slic3r::PrinterStructure::psI3) {
                x = kI3WipeTowerDefaultX;
                y = kI3WipeTowerDefaultY;
            }
            Slic3r::ConfigOptionFloat wx(x), wy(y);
            config.option<Slic3r::ConfigOptionFloats>("wipe_tower_x", true)->set_at(&wx, plate_index, 0);
            config.option<Slic3r::ConfigOptionFloats>("wipe_tower_y", true)->set_at(&wy, plate_index, 0);
        }
        const bool single_object = printable_objects(model) == duplicate_count + 1 && originals == 1;
        int landed = 0;
        const bool all_landed = arrange(model, config, landed);
        if (all_landed) {
            if (single_object || duplicate_count == up || duplicate_count == up - 1) {
                result.copies_kept = duplicate_count;
                break;
            }
            // Multiple objects: try more (BambuStudio.cpp 6111-6123).
            low = duplicate_count;
            duplicate_count = (up + low) / 2;
            continue;
        }
        if (single_object) {
            // Keep the copies that landed (BambuStudio.cpp 6084-6110).
            if (landed <= 1) {
                model  = original_model;
                config = original_config;
                result.restored = true;
                break;
            }
            result.copies_kept = landed - 1;
            break;
        }
        // Multiple objects: fewer copies (BambuStudio.cpp 6014-6081).
        if (duplicate_count == 1) {
            duplicate_count = 0;
        } else if (duplicate_count == low) {
            up = duplicate_count;
            low--;
            duplicate_count--;
        } else {
            up = duplicate_count;
            duplicate_count = (up + low) / 2;
        }
        if (duplicate_count == 0) {
            model  = original_model;
            config = original_config;
            result.restored = true;
            break;
        }
    }
    emit({{"event", "arranged"}, {"tag", "RepetitionsPlaced"}, {"copies", result.copies_kept},
          {"message", result.restored ? std::string("--repetitions: no copy fitted beside the original; the plate is printed as it was")
                                      : "--repetitions: " + std::to_string(result.copies_kept) + " cop" +
                                            (result.copies_kept == 1 ? "y" : "ies") + " placed beside the original"}});
    return result;
}

} // namespace slicer_cli
