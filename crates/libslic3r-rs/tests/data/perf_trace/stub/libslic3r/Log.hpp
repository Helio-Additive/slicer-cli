// Stand-in for libslic3r/Log.hpp, for gen.cpp of this directory: BOOST_LOG_TRIVIAL writes the
// record as a line `LOG <level> <message>` to stdout and flush_logs() writes `FLUSH`, so that the
// order of the two can be read back. (Log.hpp is `#pragma once`, so it cannot be kept out by
// defining a guard; this directory is put before the engine's on the include path instead.)
#pragma once

#include <cstdio>
#include <sstream>
#include <string>

namespace Slic3r {
struct GenLog
{
    const char        *level;
    std::ostringstream text;
    explicit GenLog(const char *level) : level(level) {}
    ~GenLog() { printf("LOG %s %s\n", level, text.str().c_str()); }
    template<class T> GenLog &operator<<(const T &v)
    {
        text << v;
        return *this;
    }
};
inline void flush_logs() { printf("FLUSH\n"); }
} // namespace Slic3r

#undef BOOST_LOG_TRIVIAL
#define BOOST_LOG_TRIVIAL(lvl) ::Slic3r::GenLog(#lvl)
