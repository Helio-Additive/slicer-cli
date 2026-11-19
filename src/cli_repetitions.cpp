// cli_repetitions.cpp — see cli_repetitions.hpp.
#include "cli_repetitions.hpp"

#include <algorithm>

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

/// The arrange's own result — the offset and the rotation of every instance —
/// from `from` onto `to`, object by object and instance by instance, which the
/// two hold in the same order: this is what the arrange writes
/// (ModelInstance::apply_arrange_result sets the offset and rotates the
/// instance, Model.cpp 4528-4535 at 5873b5f; apply_arrange_polys calls it per
/// instance, ModelArrange.cpp 29-41). It lets the search below run on a copy
/// and land on the model itself without the model being replaced.
void copy_arrange_result(const Slic3r::Model& from, Slic3r::Model& to) {
    const size_t objects = std::min(from.objects.size(), to.objects.size());
    for (size_t k = 0; k < objects; ++k) {
        const Slic3r::ModelObject* source = from.objects[k];
        Slic3r::ModelObject* destination = to.objects[k];
        const size_t instances = std::min(source->instances.size(), destination->instances.size());
        for (size_t i = 0; i < instances; ++i) {
            destination->instances[i]->set_offset(source->instances[i]->get_offset());
            destination->instances[i]->set_rotation(source->instances[i]->get_rotation());
        }
        destination->invalidate_bounding_box();
    }
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
    // The search runs on a copy of the model, and only the iteration it settles
    // on is run once more on the model itself. A retry used to put the saved
    // model back with `model = original_model`, which frees every object and
    // instance the model held: the caller's PlateScope (main.cpp) and the run's
    // plate lists still name them (a plate of a multi-plate project that is not
    // the whole model), so the assignment was a use-after-free — ~PlateScope
    // wrote through the freed objects, and the next plate's members were gone.
    // Nothing here replaces the model now: it is only added to (the copies) and
    // moved (the arrange).
    const Slic3r::Model original_model = model;
    const Slic3r::DynamicPrintConfig original_config = config;
    Slic3r::Model work;
    const int originals = printable_objects(model);
    const auto set_tower_entry = [&](Slic3r::DynamicPrintConfig& c) {
        // The copies start the tower at the printer's default corner
        // (BambuStudio.cpp 5773-5781; OrcaSlicer.cpp 5033-5041).
        float x = kWipeTowerDefaultX, y = kWipeTowerDefaultY;
        if (const auto* s = c.option<Slic3r::ConfigOptionEnum<Slic3r::PrinterStructure>>("printer_structure");
            s && s->value == Slic3r::PrinterStructure::psI3) {
            x = kI3WipeTowerDefaultX;
            y = kI3WipeTowerDefaultY;
        }
        Slic3r::ConfigOptionFloat wx(x), wy(y);
        c.option<Slic3r::ConfigOptionFloats>("wipe_tower_x", true)->set_at(&wx, plate_index, 0);
        c.option<Slic3r::ConfigOptionFloats>("wipe_tower_y", true)->set_at(&wy, plate_index, 0);
    };
    int chosen = 0;   // the duplicate count the search settled on
    int low = 0, up = duplicate_count;
    for (;;) {
        work   = original_model;
        config = original_config;
        duplicate_all_instance(work, unsigned(duplicate_count), skip_ids);
        set_tower_entry(config);
        const bool single_object = printable_objects(work) == duplicate_count + 1 && originals == 1;
        int landed = 0;
        const bool all_landed = arrange(work, config, landed);
        if (all_landed) {
            if (single_object || duplicate_count == up || duplicate_count == up - 1) {
                result.copies_kept = duplicate_count;
                chosen = duplicate_count;
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
                result.restored = true;
                break;
            }
            result.copies_kept = landed - 1;
            chosen = duplicate_count;
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
            result.restored = true;
            break;
        }
    }
    // The settled iteration, on the model itself: the same copies (the same
    // add_object order, so the two lists line up) and the arrange's own result
    // copied over, which is what that iteration left on the copy. `config` is
    // that iteration's too: it holds the tower the arrange placed the copies
    // around (arrange_on_bed writes it), so it is kept as it is. With no copy
    // placed the plate is printed as it was, its own tower included, as the
    // official puts the saved config back (BambuStudio.cpp 6084-6110,
    // 6014-6081).
    if (result.restored) {
        config = original_config;
    } else {
        duplicate_all_instance(model, unsigned(chosen), skip_ids);
        copy_arrange_result(work, model);
    }
    emit({{"event", "arranged"}, {"tag", "RepetitionsPlaced"}, {"copies", result.copies_kept},
          {"message", result.restored ? std::string("--repetitions: no copy fitted beside the original; the plate is printed as it was")
                                      : "--repetitions: " + std::to_string(result.copies_kept) + " cop" +
                                            (result.copies_kept == 1 ? "y" : "ies") + " placed beside the original"}});
    return result;
}

} // namespace slicer_cli
