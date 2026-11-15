// Reference vectors of tests/log_native_parity.rs: Bambu Studio's Log.cpp at the pin, with the
// LogSink.cpp that slicer_cli compiles (the override layer's) and the pin's Desensitize.cpp, over
// Boost.Log.
//
// Build and run, from the repository root (macOS arm64 clang is the reference build; PREFIX is where
// Boost is installed; STUB is a directory holding an empty libslic3r_version.h, the header CMake
// generates; BBL_RELEASE_TO_PUBLIC is 1 as the build of slicer_cli sets it):
//
//   clang++ -w -std=c++17 -O2 -ffp-contract=on -DBOOST_LOG_DYN_LINK -DBBL_RELEASE_TO_PUBLIC=1 \
//       -I $STUB -I references/BambuStudio/src/libslic3r -I references/BambuStudio/src \
//       -I $PREFIX/include crates/libslic3r-rs/tests/data/log/gen.cpp \
//       libslic3r/bambustudio/libslic3r/LogSink.cpp references/BambuStudio/src/libslic3r/Desensitize.cpp \
//       -L $PREFIX/lib -lboost_log -lboost_log_setup -lboost_thread -lboost_filesystem -o log_gen
//   ./log_gen <an empty scratch directory> > crates/libslic3r-rs/tests/data/log/vectors.txt
//
// Log.cpp is included below unchanged. The log files are written under the scratch directory.
//
// Format of vectors.txt, one step per line; a step with a result has it after `->`. A string is `~`
// followed by the text, with '%' and every byte outside the printable ASCII characters
// percent-encoded. A value is `<type>=<payload>`: i int, u unsigned, l long long, b bool, d double
// (bits), f float (bits), c char, sc signed char, uc unsigned char, s string, p char pointer (`null`
// or a string), j json (its text).
//
//   NAME name -> level_string_to_boost          STR n -> get_string_logging_level
//   SETLVL n -> get_logging_level, then log_enabled for trace .. fatal
//   KV key value -> KV(key, value)
//   EXC kind -> current_exception_message       (none, std, other)
//   VALID -> is_log_trivial_valid
//   DEFREC level n values -> what Boost.Log's default sink prints for the record (`-` if filtered)
//   SETPATH dir level enc_type enc_key_url enc_key_host_env     set_log_path_and_level
//   UPDATE enc_type enc_key_url enc_key_host_env                update_log_sink
//   REC level n values                                          one record through LogStream
//   FILE dir name lines        a log file after flush_logs, then its lines as `LINE text`
// In the printed lines the digits of the time stamp are `D`, those of the thread id `H`, and the
// source location is `[L]`; in a file name the date, the time and the process id are `*`.

#include "Log.cpp"

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <variant>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

using namespace Slic3r;

static uint64_t g_seed = 0xD1B54A32D192ED03ull;
static uint64_t rnd()
{
    g_seed ^= g_seed << 13;
    g_seed ^= g_seed >> 7;
    g_seed ^= g_seed << 17;
    return g_seed;
}
static int  rint_(int n) { return int(rnd() % uint64_t(n)); } // 0 .. n-1
static bool chance(int percent) { return rint_(100) < percent; }
template<class T, size_t N> static T pick(const T (&items)[N]) { return items[rint_(int(N))]; }

static std::string tok(const std::string &s)
{
    std::string out = "~";
    for (unsigned char c : s) {
        if (c <= ' ' || c == '%' || c >= 0x7f) {
            char buf[4];
            std::snprintf(buf, sizeof(buf), "%%%02X", c);
            out += buf;
        } else
            out += char(c);
    }
    return out;
}

using Value = std::variant<int, unsigned, long long, bool, double, float, char, signed char, unsigned char, std::string, const char *, nlohmann::json>;

static const char *const k_texts[] = {"", "plain", "two words", "a=b", "say \"hi\"", "back\\slash", "tab\there", "line\nbreak", "cr\rhere",
                                      "\xc3\xa9t\xc3\xa9", "\xe4\xb8\xad\xe6\x96\x87", "0.4", "N7", "trailing ", " leading", "x", "192.168.1.20",
                                      "01S00A123456789", "secret-token-value", "ab", "abcdefg", "https://example.com/a?b=c", "end\n", "end\r\n\n"};
static const char *const k_jsons[] = {"{}", "[]", "null", "true", "17", "\"text\"", "{\"a\":1,\"b\":[1,2,3]}", "{\"dev_id\":\"01S00A123456789\",\"n\":2}",
                                      "{\"token\":\"secret\",\"nested\":{\"dev_ip\":\"192.168.1.20\",\"ok\":true}}", "[{\"sn\":\"0123456789\"},{\"x\":1.5}]",
                                      "{\"uid\":\"1234567\",\"url\":\"https://example.com\"}", "{\"text\":\"a b\\n\"}"};
// The jsons Desensitize::json_log masks without an error. For the others it writes an info record of its
// own ("Error processing JSON for cross-talk"), which crate::desensitize sends to another logger; they
// are used for KV only, where the level filters that record out.
static const char *const k_jsons_masked[] = {"{}", "[]", "null", "{\"dev_id\":\"01S00A123456789\",\"n\":2}",
                                             "{\"token\":\"secret\",\"nested\":{\"dev_ip\":\"192.168.1.20\",\"ok\":true}}",
                                             "{\"uid\":\"1234567\",\"url\":\"https://example.com\"}", "{\"text\":\"a b\\n\"}"};
static const double k_doubles[] = {0.0, -0.0, 1.0, -1.0, 0.1, 0.4, 0.5, 1.5, 2.5, 100.0, 123456.0, 1234567.0, 999999.0, 999999.5, 9999995.0, 0.0001,
                                   0.00001, 0.000012345678, 1e21, 1e-21, 1e100, 1e-100, 123456789.0, 0.3, 1.0 / 3.0, 2.0 / 3.0, 1e5, 1e6, 1e-4, 1e-5,
                                   99999.95, 0.00009999995, 3.14159265358979, 2.718281828e10, 5e-324, 1.7976931348623157e308};

static Value rnd_value(bool in_record = true)
{
    switch (rint_(13)) {
    case 0: return Value(std::in_place_type<int>, chance(50) ? rint_(200) - 100 : int(rnd()));
    case 1: return Value(std::in_place_type<unsigned>, unsigned(rnd()));
    case 2: return Value(std::in_place_type<long long>, chance(20) ? (chance(50) ? INT64_MIN : INT64_MAX) : (long long) rnd());
    case 3: return Value(std::in_place_type<bool>, chance(50));
    case 4: {
        double v;
        const int r = rint_(100);
        if (r < 50)
            v = pick(k_doubles);
        else if (r < 60)
            v = chance(50) ? std::numeric_limits<double>::infinity() : -std::numeric_limits<double>::infinity();
        else if (r < 65)
            v = std::numeric_limits<double>::quiet_NaN();
        else if (r < 85)
            v = double(int64_t(rnd() % 2000001) - 1000000) / double(1 + rint_(100000));
        else {
            uint64_t bits = rnd();
            std::memcpy(&v, &bits, sizeof(v));
            if (std::isnan(v)) v = 1.25;
        }
        return Value(std::in_place_type<double>, v);
    }
    case 5: {
        float v = chance(50) ? float(pick(k_doubles)) : float(int(rnd() % 20001) - 10000) / float(1 + rint_(1000));
        return Value(std::in_place_type<float>, v);
    }
    case 6: return Value(std::in_place_type<char>, char('!' + rint_(94)));
    case 7: return Value(std::in_place_type<signed char>, (signed char) (rint_(256) - 128));
    case 8: return Value(std::in_place_type<unsigned char>, (unsigned char) rint_(256));
    case 9:
    case 10: return Value(std::in_place_type<std::string>, pick(k_texts));
    case 11: return Value(std::in_place_type<const char *>, chance(25) ? nullptr : pick(k_texts));
    default: return Value(std::in_place_type<nlohmann::json>, nlohmann::json::parse(in_record ? pick(k_jsons_masked) : pick(k_jsons)));
    }
}

static std::string bits64(double v)
{
    uint64_t u;
    std::memcpy(&u, &v, sizeof(u));
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%016llx", (unsigned long long) u);
    return buf;
}

static std::string bits32(float v)
{
    uint32_t u;
    std::memcpy(&u, &v, sizeof(u));
    char buf[9];
    std::snprintf(buf, sizeof(buf), "%08x", u);
    return buf;
}

static std::string value_token(const Value &value)
{
    struct Visitor
    {
        std::string operator()(int v) const { return "i=" + std::to_string(v); }
        std::string operator()(unsigned v) const { return "u=" + std::to_string(v); }
        std::string operator()(long long v) const { return "l=" + std::to_string(v); }
        std::string operator()(bool v) const { return std::string("b=") + (v ? "1" : "0"); }
        std::string operator()(double v) const { return "d=" + bits64(v); }
        std::string operator()(float v) const { return "f=" + bits32(v); }
        std::string operator()(char v) const { return std::string("c=") + v; }
        std::string operator()(signed char v) const { return "sc=" + std::to_string(int(v)); }
        std::string operator()(unsigned char v) const { return "uc=" + std::to_string(int(v)); }
        std::string operator()(const std::string &v) const { return "s=" + tok(v); }
        std::string operator()(const char *v) const { return v ? "p=" + tok(v) : std::string("p=null"); }
        std::string operator()(const nlohmann::json &v) const { return "j=" + tok(v.dump()); }
    };
    return std::visit(Visitor(), value);
}

static std::string values_tokens(const std::vector<Value> &values)
{
    std::string out = std::to_string(values.size());
    for (const Value &value : values) out += " " + value_token(value);
    return out;
}

// One record, as BOOST_LOG_TRIVIAL makes it: the level check, then a LogStream that takes the values.
static void record(LogLevel level, const std::vector<Value> &values)
{
    if (!log_detail::log_enabled(level))
        return;
    log_detail::LogStream stream(level, __FILE__, __LINE__, __func__);
    for (const Value &value : values) std::visit([&stream](const auto &v) { stream << v; }, value);
}

// The digits of the time stamp become D, those of the thread id H, and the location becomes [L]. A line
// that does not start a record (the rest of a message with a line break in it) stays as it is.
static std::string normalized(std::string line, bool file_format)
{
    // file: `<time> [<thread>] [<level>] [<location>] <message>`; default sink: `[<time>] [<thread>] [<level>]<spaces> [<location>] <message>`
    const size_t time_begin   = file_format ? 0 : 1;
    const size_t thread_begin = time_begin + 26 + (file_format ? 1 : 2);
    if (line.size() < thread_begin + 3 || line.compare(thread_begin, 3, "[0x") != 0 || line[time_begin + 4] != '-' || line[time_begin + 10] != ' ')
        return line;
    for (size_t i = time_begin; i < time_begin + 26; ++i)
        if (line[i] >= '0' && line[i] <= '9') line[i] = 'D';
    const size_t thread_end = line.find(']', thread_begin);
    if (thread_end == std::string::npos)
        return line;
    for (size_t i = thread_begin + 3; i < thread_end; ++i) line[i] = 'H';
    const size_t level_end = line.find(']', thread_end + 1);
    const size_t loc_begin = level_end == std::string::npos ? std::string::npos : line.find('[', level_end);
    const size_t loc_end   = loc_begin == std::string::npos ? std::string::npos : line.find(']', loc_begin);
    if (loc_end != std::string::npos)
        line.replace(loc_begin, loc_end - loc_begin + 1, "[L]");
    return line;
}

static const char *const k_levels[] = {"trace", "debug", "info", "warning", "error", "fatal"};

static std::vector<Value> rnd_values()
{
    std::vector<Value> values;
    const int          n = 1 + rint_(5);
    for (int i = 0; i < n; ++i) values.push_back(rnd_value());
    return values;
}

// `studio_%a_%b_%d_%H_%M_%S_<pid>`: the seven fields after `studio` become `*`.
static std::string normalized_file_name(const std::string &name)
{
    std::string out;
    size_t      field = 0, pos = 0;
    while (pos < name.size()) {
        size_t end = name.find('_', pos);
        if (field >= 1 && field <= 7) {
            // the process id ends at '_' or '.'
            if (field == 7) end = name.find_first_of("_.", pos);
            out += "*";
        } else {
            if (end != std::string::npos && field > 7) end = std::string::npos;
            out += name.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
        }
        if (end == std::string::npos) break;
        if (field > 7) { out += name.substr(end); break; }
        out += name[end];
        pos = end + 1;
        ++field;
        if (field == 8) { out += name.substr(pos); break; }
    }
    return out;
}

static void dump_dir(const std::string &label, const std::string &dir)
{
    std::vector<std::pair<std::string, std::string>> files; // normalized name, path
    for (const auto &entry : boost::filesystem::directory_iterator(dir))
        if (boost::filesystem::is_regular_file(entry.path()))
            files.emplace_back(normalized_file_name(entry.path().filename().string()), entry.path().string());
    std::sort(files.begin(), files.end());
    for (const auto &[name, path] : files) {
        std::ifstream            in(path, std::ios::binary);
        std::vector<std::string> lines;
        for (std::string line; std::getline(in, line);) lines.push_back(line);
        std::printf("FILE %s %s %zu\n", label.c_str(), tok(name).c_str(), lines.size());
        for (const std::string &line : lines) std::printf("LINE %s\n", tok(normalized(line, true)).c_str());
    }
}

static void records(int count)
{
    for (int i = 0; i < count; ++i) {
        if (chance(12)) {
            const int n = rint_(7);
            set_logging_level(unsigned(n));
            std::printf("SETLVL %d -> %u", n, get_logging_level());
            for (int l = 0; l < 6; ++l) std::printf(" %d", log_detail::log_enabled(LogLevel(l)) ? 1 : 0);
            std::printf("\n");
        }
        const int                level  = rint_(6);
        const std::vector<Value> values = rnd_values();
        std::printf("REC %s %s\n", k_levels[level], values_tokens(values).c_str());
        record(LogLevel(level), values);
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: log_gen <empty scratch directory>\n");
        return 1;
    }
    const std::string scratch = argv[1];

    // The level functions.
    for (const char *name : {"fatal", "error", "warning", "info", "debug", "trace", "Fatal", "", "warn", "trace ", "verbose"})
        std::printf("NAME %s -> %u\n", tok(name).c_str(), level_string_to_boost(name));
    for (unsigned n = 0; n < 9; ++n) std::printf("STR %u -> %s\n", n, tok(get_string_logging_level(n)).c_str());
    std::printf("VALID -> %d\n", is_log_trivial_valid() ? 1 : 0);
    // the level a program starts with
    std::printf("GETLVL -> %u\n", get_logging_level());
    for (unsigned n : {0u, 1u, 2u, 3u, 4u, 5u, 6u, 77u, 1u}) {
        set_logging_level(n);
        std::printf("SETLVL %u -> %u", n, get_logging_level());
        for (int l = 0; l < 6; ++l) std::printf(" %d", log_detail::log_enabled(LogLevel(l)) ? 1 : 0);
        std::printf("\n");
    }

    // KV: every registered key and a few others, with values of every type.
    static const char *const k_keys[] = {"dev_id", "sn", "dev_ip", "dev_name", "uid", "access_code", "token", "password", "authkey", "ttcode_enc",
                                         "url", "region", "model_id", "seq", "ret", "task_id", "plate", "extruder_id", "nozzle_diameter", "file_count",
                                         "app_version", "gpu", "other", "", "a b", "k=v", "my_dev_id"};
    for (const char *key : k_keys)
        for (int i = 0; i < 26; ++i) {
            const Value       value = rnd_value(false);
            const std::string text  = std::visit([key](const auto &v) { return KV(key, v); }, value);
            std::printf("KV %s %s -> %s\n", tok(key).c_str(), value_token(value).c_str(), tok(text).c_str());
        }

    // current_exception_message
    std::printf("EXC none -> %s\n", tok(current_exception_message()).c_str());
    try {
        throw std::runtime_error("what it says");
    } catch (...) {
        std::printf("EXC std -> %s\n", tok(current_exception_message()).c_str());
    }
    try {
        throw 7;
    } catch (...) {
        std::printf("EXC other -> %s\n", tok(current_exception_message()).c_str());
    }

    // Without a sink Boost.Log prints the record on stdout; it is caught in a file to show it here.
    set_logging_level(3);
    std::printf("SETLVL 3 -> %u", get_logging_level());
    for (int l = 0; l < 6; ++l) std::printf(" %d", log_detail::log_enabled(LogLevel(l)) ? 1 : 0);
    std::printf("\n");
    boost::filesystem::create_directories(scratch);
    const std::string capture = scratch + "/stdout.txt";
    for (int i = 0; i < 40; ++i) {
        const int                level  = rint_(6);
        const std::vector<Value> values = rnd_values();
        std::fflush(stdout);
        const int saved = dup(1);
        const int fd    = open(capture.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
        dup2(fd, 1);
        record(LogLevel(level), values);
        std::fflush(stdout);
        dup2(saved, 1);
        close(fd);
        close(saved);
        std::ifstream in(capture, std::ios::binary);
        std::string   text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        std::string   shown = "-";
        if (!text.empty()) {
            // one line, ended by the sink
            if (text.back() == '\n') text.pop_back();
            shown = tok(normalized(text, false));
        }
        std::printf("DEFREC %s %s -> %s\n", k_levels[level], values_tokens(values).c_str(), shown.c_str());
    }
    boost::filesystem::remove(capture);

    // The log file.
    const std::string dir_a = scratch + "/a";
    const std::string dir_b = scratch + "/b/sub/dir";
    LogEncOptions     enc;
    enc.enc_type = LogEncOptions::LOG_ENC_NONE;
    std::printf("SETPATH a 3 %d %s %s\n", int(enc.enc_type), tok(enc.enc_key_url).c_str(), tok(enc.enc_key_host_env).c_str());
    set_log_path_and_level(dir_a, 3, enc);
    std::printf("VALID -> %d\n", is_log_trivial_valid() ? 1 : 0);
    std::printf("GETLVL -> %u\n", get_logging_level());
    records(120);

    auto update = [](LogEncOptions::LogEncType type, const char *url, const char *env) {
        LogEncOptions options;
        options.enc_type         = type;
        options.enc_key_url      = url;
        options.enc_key_host_env = env;
        std::printf("UPDATE %d %s %s\n", int(type), tok(url).c_str(), tok(env).c_str());
        update_log_sink(options);
    };
    update(LogEncOptions::LOG_ENC_NONE, "", "");  // the same options: nothing changes
    records(20);
    update(LogEncOptions::LOG_ENC_AES_256_CBC, "", "cn");
    records(60);
    update(LogEncOptions::LOG_ENC_AES_256_CBC, "", "dc");
    records(40);
    update(LogEncOptions::LOG_ENC_AES_256_CBC, "https://example.com/key", "us");
    update(LogEncOptions::LOG_ENC_AES_256_CBC, "https://example.com/key", "us");
    records(40);
    update(LogEncOptions::LOG_ENC_NONE, "", "cn");
    records(30);

    // A second call adds a second sink; the first one stays, so both files get the records.
    LogEncOptions enc_b;
    std::printf("SETPATH b 5 %d %s %s\n", int(enc_b.enc_type), tok(enc_b.enc_key_url).c_str(), tok(enc_b.enc_key_host_env).c_str());
    set_log_path_and_level(dir_b, 5, enc_b);
    std::printf("GETLVL -> %u\n", get_logging_level());
    records(60);

    flush_logs();
    // flush_logs() flushes the newest sink; the older one is flushed when the program ends, so its file is
    // read after all sinks are gone.
    boost::log::core::get()->flush();
    dump_dir("a", dir_a);
    dump_dir("b", dir_b);
    return 0;
}
