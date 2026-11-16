// Reference lines for crates/libslic3r-rs/src/perf_trace.rs, taken from the C++ at the pin.
//
// The program compiles references/BambuStudio/src/libslic3r/PerfTrace.cpp as it is, with
// stub/libslic3r/Log.hpp in the place of the engine's Log.hpp: every record is written as
// `LOG <level> <message>` and every flush_logs() as `FLUSH`.
//
// Build and run, from the repository root:
//   D=crates/libslic3r-rs/tests/data/perf_trace
//   clang++ -std=c++17 -O2 -I $D/stub -I references/BambuStudio/src/libslic3r \
//       -I references/BambuStudio/src $D/gen.cpp -o /tmp/perf_gen
//   /tmp/perf_gen > $D/vectors.txt
//
// Output: `STEP <name>` before each step of main() below, then what the step logged. The number
// of milliseconds in a "cost" line is the time the step took (the test compares it as a number,
// not by its value).

#include "PerfTrace.cpp"

#include <thread>

using namespace Slic3r;

int main()
{
    printf("STEP mark\n");
    perf_mark("startup");
    printf("STEP mark_empty\n");
    perf_mark("");
    printf("STEP mark_text\n");
    perf_mark("load project: a b [c] 100%");
    printf("STEP timer\n");
    {
        PerfTimer timer("load");
    }
    printf("STEP timer_disabled\n");
    {
        PerfTimer timer("quiet", false);
    }
    printf("STEP timer_enabled\n");
    {
        PerfTimer timer("loud", true);
    }
    printf("STEP trace\n");
    {
        PERF_TRACE("scope");
        perf_mark("inside the scope");
    }
    printf("STEP trace_if_true\n");
    {
        PERF_TRACE_IF(true, std::string("3mf: ") + "add model");
        perf_mark("inside");
    }
    printf("STEP trace_if_false\n");
    {
        PERF_TRACE_IF(false, "3mf: add thumbnails");
        perf_mark("inside");
    }
    printf("STEP nested\n");
    {
        PERF_TRACE("outer");
        {
            PERF_TRACE("inner");
        }
        perf_mark("between");
    }
    printf("STEP sleep\n");
    {
        PERF_TRACE("sleep");
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    return 0;
}
