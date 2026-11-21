#include <boost/log/core.hpp>

// Engine static configuration constructors log before main(). Keep those
// records off stdout so strict-JSON commands are valid from the first byte.
// Isolate the MSVC initialization segment: it may be selected only once per
// translation unit and must not affect unrelated globals in the CLI driver.
#ifdef _MSC_VER
#pragma init_seg(lib)
#endif

namespace {
struct StartupLogSilencer {
    /// Disables startup logging without using standard streams.
    StartupLogSilencer() { boost::log::core::get()->set_logging_enabled(false); }
};

#if defined(_MSC_VER)
StartupLogSilencer startup_log_silencer;
#elif defined(__GNUC__) || defined(__clang__)
StartupLogSilencer startup_log_silencer __attribute__((init_priority(101)));
#else
StartupLogSilencer startup_log_silencer;
#endif
}
