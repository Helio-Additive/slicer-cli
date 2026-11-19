#include "cli_bed_fit.hpp"

#include <cmath>
#include <limits>
#include <locale>
#include <sstream>
#include <iomanip>

#include <boost/multiprecision/integer.hpp>
#include <boost/rational.hpp>

#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/PrintConfig.hpp"
// Bambu's model headers need to precede the libnest2d Point operators on Clang.
#include "libslic3r/Model.hpp"
#include <libnest2d/backends/libslic3r/geometries.hpp>
#include <libnest2d/utils/rotcalipers.hpp>

namespace slicer_cli {

std::optional<double> fitting_z_rotation(const Slic3r::ModelObject& object, size_t instance_index,
                                         const Slic3r::BoundingBoxf& bed) {
    using namespace Slic3r;
    if (instance_index >= object.instances.size())
        return std::nullopt;
    ExPolygon shape(object.convex_hull_2d(object.instances[instance_index]->get_matrix()));
    if (!shape.contour.is_valid())
        return std::nullopt;
    shape.contour.make_counter_clockwise();

    std::optional<double> best;
    auto try_degrees = [&](double degrees) {
        degrees = std::fmod(degrees, 180.);
        if (degrees < 0.)
            degrees += 180.;
        if (best && *best <= degrees)
            return;
        ExPolygon rotated = shape;
        libnest2d::sl::rotate(rotated, Geometry::deg2rad(degrees));
        const auto box = libnest2d::sl::boundingBox(rotated);
        if (unscale_(box.width()) <= bed.size().x() + EPSILON &&
            unscale_(box.height()) <= bed.size().y() + EPSILON)
            best = degrees;
    };
    for (int degrees = 0; degrees < 180; ++degrees)
        try_degrees(degrees);

    // Include the arranger's minimum-area orientation, which need not be an
    // integer angle (BambuStudio Arrange.cpp 969-973; OrcaSlicer Arrange.cpp
    // 995-1004).
    using Unit = libnest2d::TCompute<ExPolygon>;
    using Ratio = boost::rational<boost::multiprecision::int128_t>;
    try {
        const double angle = libnest2d::minAreaBoundingBox<ExPolygon, Unit, Ratio>(shape).angleToX();
        try_degrees(angle * 180. / M_PI);
    } catch (const std::exception&) {
    }
    return best;
}

namespace {
bool rectangular_bed(const Slic3r::DynamicPrintConfig& config, Slic3r::BoundingBoxf& bed) {
    using namespace Slic3r;
    const auto* area = config.option<ConfigOptionPoints>("printable_area");
    if (!area || area->values.size() < 3)
        return false;
    for (const Vec2d& point : area->values)
        bed.merge(point);
    // The bounding box proves a planar fit only on a rectangular bed.
#ifndef ENGINE_SNAPMAKER
    const BuildVolume build_volume(area->values, 0., {}, {});
#else
    const BuildVolume build_volume(area->values, 0.);
#endif
    return build_volume.type() == decltype(build_volume.type())::Rectangle;
}
} // namespace

std::optional<double> oversized_fitting_z_rotation(const Slic3r::ModelObject& object, size_t instance_index,
                                                   const Slic3r::DynamicPrintConfig& config) {
    using namespace Slic3r;
    BoundingBoxf bed;
    if (!rectangular_bed(config, bed))
        return std::nullopt;
    const Vec3d  size   = object.instance_bounding_box(instance_index).size();
    const double height = config.has("printable_height") ? config.opt_float("printable_height") : 0.;
    if (height > 0. && size.z() > height + EPSILON)
        return std::nullopt;
    if (size.x() <= bed.size().x() + EPSILON && size.y() <= bed.size().y() + EPSILON)
        return std::nullopt;
    return fitting_z_rotation(object, instance_index, bed);
}

std::string full_size_turn_advice(double degrees, const Slic3r::ModelObject& object, size_t instance_index,
                                  const Slic3r::DynamicPrintConfig& config, bool rotation_given, bool arranged) {
    using namespace Slic3r;
    std::ostringstream formatted;
    formatted.imbue(std::locale::classic());
    formatted << std::setprecision(std::numeric_limits<double>::max_digits10) << degrees;
    const std::string angle = formatted.str();
    const std::string fit   = " It fits at full size rotated " + angle + " degrees about Z.";

    // --rotate turns every object, each volume before the instance transform.
    // A mirror across X or Y reverses the world-frame turn.
    const Model* model = object.get_model();
    bool flag_alone = !rotation_given && model != nullptr && model->objects.size() == 1 &&
                      object.instances.size() == 1 && object.volumes.size() == 1;
    if (flag_alone) {
        const Eigen::Matrix3d linear       = object.instances[instance_index]->get_matrix().linear();
        const Eigen::Matrix3d quarter_turn = (Eigen::Matrix3d() << 0, -1, 0, 1, 0, 0, 0, 0, 1).finished();
        flag_alone = (linear * quarter_turn).isApprox(quarter_turn * linear);
    }
    BoundingBoxf bed;
    if (flag_alone && arranged) {
        // The arrange places the turned part; the turn fits the bed's box.
    } else if (flag_alone && rectangular_bed(config, bed)) {
        // The part turned the way the flag turns it, at the position it keeps.
        Model   scratch;
        ModelObject* turned = scratch.add_object(object);
        turned->rotate(Geometry::deg2rad(degrees), Z);
        const BoundingBoxf3 box = turned->instance_bounding_box(instance_index);
        flag_alone = box.min.x() >= bed.min.x() - EPSILON && box.min.y() >= bed.min.y() - EPSILON &&
                     box.max.x() <= bed.max.x() + EPSILON && box.max.y() <= bed.max.y() + EPSILON;
    } else {
        flag_alone = false;
    }
    if (flag_alone)
        return fit + " Use --rotate " + angle + ", or give --arrange 1 --allow-rotations.";
    return fit + " Give --arrange 1 --allow-rotations.";
}

} // namespace slicer_cli
