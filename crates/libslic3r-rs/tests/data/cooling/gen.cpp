// BambuStudio 56a57d0 GCode/CoolingBuffer.cpp 259-350. Emit adjustments in
// their original input order after the sorted tier views have been processed.
#include "libslic3r/GCode/CoolingBuffer.hpp"
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>

static unsigned bits(float value)
{
    unsigned result;
    static_assert(sizeof(result) == sizeof(value));
    std::memcpy(&result, &value, sizeof(result));
    return result;
}

int main()
{
    using namespace Slic3r;
    std::array<int, 3> order = {0, 1, 2};
    do {
        for (int disabled : {-1, 0, 1, 2}) {
            std::vector<PerExtruderAdjustments> adjustments;
            for (int id : order) {
                adjustments.emplace_back();
                auto &a = adjustments.back();
                a.extruder_id = id;
                a.cooling_slow_down_enabled = id != disabled;
                a.slow_down_layer_time = std::array<float, 3>{12.f, 3.f, 8.f}[id];
                a.slow_down_min_speed = 10.f;
                a.cooling_slowdown_logic = cslUniformCooling;
                a.lines.emplace_back(CoolingLine::TYPE_G1 | CoolingLine::TYPE_ADJUSTABLE, 0, 1);
                auto &line = a.lines.back();
                line.length = std::array<float, 3>{150.f, 100.f, 50.f}[id];
                line.feedrate = line.origin_feedrate = 50.f;
                line.time = line.length / 50.f;
                line.time_max = line.length / 10.f;
            }
            CoolingBuffer buffer;
            float total = buffer.calculate_layer_slowdown(adjustments);
            std::printf("%d %d %d %d %08x", order[0], order[1], order[2], disabled, bits(total));
            for (const auto &a : adjustments) std::printf(" %08x", bits(a.elapsed_time_total()));
            std::puts("");
        }
    } while (std::next_permutation(order.begin(), order.end()));
}
