#include "cli_bed_fit.hpp"

#include <cmath>

#include <boost/multiprecision/integer.hpp>
#include <boost/rational.hpp>
#include <libnest2d/backends/libslic3r/geometries.hpp>
#include <libnest2d/utils/rotcalipers.hpp>

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Model.hpp"

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
    auto try_angle = [&](double angle) {
        double degrees = std::fmod(angle * 180. / M_PI, 180.);
        if (degrees < 0.)
            degrees += 180.;
        if (best && *best <= degrees)
            return;
        ExPolygon rotated = shape;
        libnest2d::sl::rotate(rotated, angle);
        const auto box = libnest2d::sl::boundingBox(rotated);
        if (unscale_(box.width()) <= bed.size().x() + EPSILON &&
            unscale_(box.height()) <= bed.size().y() + EPSILON)
            best = degrees;
    };
    for (int degrees = 0; degrees < 180; ++degrees)
        try_angle(degrees * M_PI / 180.);

    // Include the arranger's minimum-area orientation, which need not be an
    // integer angle (BambuStudio Arrange.cpp 969-973; OrcaSlicer Arrange.cpp
    // 995-1004).
    using Unit = libnest2d::TCompute<ExPolygon>;
    using Ratio = boost::rational<boost::multiprecision::int128_t>;
    try {
        try_angle(libnest2d::minAreaBoundingBox<ExPolygon, Unit, Ratio>(shape).angleToX());
    } catch (const std::exception&) {
    }
    return best;
}

} // namespace slicer_cli
