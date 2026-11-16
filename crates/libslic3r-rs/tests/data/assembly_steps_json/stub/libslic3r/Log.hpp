// Stand-in for libslic3r/Log.hpp, for the generators under crates/libslic3r-rs/tests/data that
// compile a libslic3r source file by itself: BOOST_LOG_TRIVIAL takes what is streamed into it and
// writes nothing. (Log.hpp is `#pragma once`, so it cannot be kept out by defining a guard; this
// directory is put before the engine's on the include path instead.)
#pragma once

#include <string>

namespace Slic3r {
struct GenNullLog
{
    template<class T> GenNullLog &operator<<(const T &) { return *this; }
};
inline void flush_logs() {}
} // namespace Slic3r

#undef BOOST_LOG_TRIVIAL
#define BOOST_LOG_TRIVIAL(lvl) ::Slic3r::GenNullLog()
