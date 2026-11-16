// Generator of the vectors of tests/print_base_native_parity.rs: libslic3r's PrintBase.hpp /
// PrintBase.cpp at the pin (references/BambuStudio/src/libslic3r): the PrintState state machine, a
// PrintBaseWithState and a PrintObjectBaseWithState over a test step enum driven by random operation
// sequences, and the output file name functions over small models.
//
// Build, with REPO = this repository, BUILD = a built tree of the Bambu Studio engine
// (docs/building.md, -DENGINE=bambu) and DATA = this directory: compile with the compile line the
// build uses for a libslic3r source (its defines and include directories, the override layer
// $REPO/libslic3r/bambustudio before the pin, -std=c++17 -O3 -DNDEBUG and the forced include of
// cgal_54_compat.hpp), then link with the build's libslic3r_core and the libraries it links:
//
//   c++ <the build's defines and include directories> -std=c++17 -O3 -DNDEBUG \
//       -include $REPO/libslic3r/bambustudio/cgal_54_compat.hpp -c $DATA/gen.cpp -o gen.o
//   c++ gen.o $BUILD/libslic3r_core.a <the libraries libslic3r_core links> -o gen
//   ./gen > vectors.txt && gzip -9 -n -c vectors.txt > $DATA/vectors.txt.gz
//
// The run is deterministic (a fixed LCG); the output file names use neither the time nor the
// version placeholders. Timestamps are written relative to g_last_timestamp at the start of the
// sequence ("z" for a step never stamped). An object ID is written as "P" (the print's), "O" (the
// object's) or "-" (another).
//
// Line format: tokens separated by one space; a text token is the hexadecimal of its bytes ("-" for
// the empty text), a double its 16 hexadecimal digits, an integer is decimal. A sequence line is
//   <kind> <ops...> => <results...>
// where the operations and their results are listed in the same order.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#define private public
#define protected public
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/Model.hpp"
#undef protected
#undef private

using namespace Slic3r;

static std::string out;
static void tok(const std::string &s)
{
    if (!out.empty() && out.back() != '\n')
        out += ' ';
    out += s;
}
static void ti(long long v) { tok(std::to_string(v)); }
static void td(double v)
{
    uint64_t b;
    std::memcpy(&b, &v, 8);
    char buf[32];
    snprintf(buf, sizeof(buf), "%016llx", (unsigned long long) b);
    tok(buf);
}
static std::string hex(const std::string &s)
{
    if (s.empty())
        return "-";
    static const char *d = "0123456789abcdef";
    std::string r;
    for (unsigned char c : s) {
        r += d[c >> 4];
        r += d[c & 15];
    }
    return r;
}
static void ts(const std::string &s) { tok(hex(s)); }
static void endl_()
{
    out += '\n';
    fputs(out.c_str(), stdout);
    fflush(stdout);
    out.clear();
}

static uint64_t g_seed = 0x2545f4914f6cdd1dull;
static uint32_t rnd()
{
    g_seed = g_seed * 6364136223846793005ull + 1442695040888963407ull;
    return uint32_t(g_seed >> 33);
}
static int    ri(int lo, int hi) { return lo + int(rnd() % uint32_t(hi - lo + 1)); }
static double rd(double lo, double hi) { return lo + (hi - lo) * (double(rnd()) / 2147483648.0); }
static bool   rb(int pct = 50) { return ri(0, 99) < pct; }

enum TestStep { tsA, tsB, tsC, tsD, tsCount };

static const std::vector<std::string> g_messages{"first warning", "second", "", "third one", "first warning"};

// ---------------------------------------------------------------------------------------------
// PrintState
// ---------------------------------------------------------------------------------------------

static size_t g_base = 0;
static void tstamp(size_t t)
{
    if (t == 0)
        tok("z");
    else
        ti((long long) (t - g_base));
}

template<class State> static void write_state(const State &st)
{
    ti((int) st.state);
    tstamp(st.timestamp);
}

static void write_warnings(const PrintStateBase::StateWithWarnings &st)
{
    write_state(st);
    ti((long long) st.warnings.size());
    for (auto &w : st.warnings) {
        ti((int) w.level);
        ti(w.current);
        ts(w.message);
        ti(w.message_id);
    }
}

static void case_print_state()
{
    for (int it = 0; it < 300; ++it) {
        PrintState<TestStep, tsCount> ps;
        std::mutex mtx;
        g_base = PrintStateBase::g_last_timestamp;
        std::string ops, res;
        int n = ri(1, 30);
        std::string line_in = "print_state";
        std::string line_out;
        auto add_in = [&](const std::string &s) { line_in += " " + s; };
        auto add_out = [&](const std::string &s) { line_out += (line_out.empty() ? "" : " ") + s; };
        for (int k = 0; k < n; ++k) {
            int op = ri(0, 9);
            TestStep step = TestStep(ri(0, tsCount - 1));
            out.clear();
            if (op == 0) {
                bool cancel = rb(10);
                add_in("S " + std::to_string(step) + " " + std::to_string(cancel));
                try {
                    bool r = ps.set_started(step, mtx, [cancel]() { if (cancel) throw CanceledException(); });
                    ti(r);
                } catch (CanceledException &) {
                    tok("!C");
                }
            } else if (op == 1) {
                if (ps.m_step_active < 0 || ps.m_state[ps.m_step_active].state != PrintStateBase::STARTED) {
                    --k;
                    continue;
                }
                TestStep s = TestStep(ps.m_step_active);
                bool cancel = rb(10);
                add_in("D " + std::to_string(s) + " " + std::to_string(cancel));
                try {
                    auto r = ps.set_done(s, mtx, [cancel]() { if (cancel) throw CanceledException(); });
                    tstamp(r.first);
                    ti(r.second);
                } catch (CanceledException &) {
                    tok("!C");
                }
            } else if (op == 2) {
                add_in("I " + std::to_string(step));
                int calls = 0;
                bool r = ps.invalidate(step, [&calls]() { ++calls; });
                ti(r);
                ti(calls);
            } else if (op == 3) {
                std::vector<TestStep> steps;
                for (int j = 0; j < ri(0, 3); ++j)
                    steps.push_back(TestStep(ri(0, tsCount - 1)));
                std::string s = "M " + std::to_string(steps.size());
                for (auto x : steps)
                    s += " " + std::to_string(x);
                add_in(s);
                int calls = 0;
                bool r = ps.invalidate_multiple(steps.begin(), steps.end(), [&calls]() { ++calls; });
                ti(r);
                ti(calls);
            } else if (op == 4) {
                add_in("A");
                int calls = 0;
                bool r = ps.invalidate_all([&calls]() { ++calls; });
                ti(r);
                ti(calls);
            } else if (op == 5) {
                if (ps.m_step_active < 0 || ps.m_state[ps.m_step_active].state != PrintStateBase::STARTED) {
                    --k;
                    continue;
                }
                int level = ri(0, 1), mi = ri(0, (int) g_messages.size() - 1), id = rb(50) ? 0 : ri(1, 3);
                add_in("W " + std::to_string(level) + " " + std::to_string(mi) + " " + std::to_string(id));
                auto r = ps.active_step_add_warning(PrintStateBase::WarningLevel(level), g_messages[mi], id, mtx);
                ti((int) r.first);
                ti(r.second);
            } else if (op == 6) {
                add_in("T " + std::to_string(step));
                write_state(ps.state_with_timestamp(step, mtx));
                write_state(ps.state_with_timestamp_unguarded(step));
            } else if (op == 7) {
                add_in("X " + std::to_string(step));
                write_warnings(ps.state_with_warnings(step, mtx));
            } else {
                add_in("Q " + std::to_string(step));
                ti(ps.is_started(step, mtx));
                ti(ps.is_done(step, mtx));
                ti(ps.is_started_unguarded(step));
                ti(ps.is_done_unguarded(step));
            }
            add_out(out);
            out.clear();
        }
        fputs((line_in + " => " + line_out + "\n").c_str(), stdout);
    }
}

// ---------------------------------------------------------------------------------------------
// PrintBaseWithState / PrintObjectBaseWithState
// ---------------------------------------------------------------------------------------------

class TestPrint : public PrintBaseWithState<TestStep, tsCount>
{
public:
    PrinterTechnology technology() const noexcept override { return ptFFF; }
    void clear() override {}
    bool empty() const override { return true; }
    std::vector<ObjectID> print_object_ids() const override { return {}; }
    ApplyStatus apply(const Model &, DynamicPrintConfig, bool) override { return APPLY_STATUS_UNCHANGED; }
    void process(std::unordered_map<std::string, long long> *, bool) override {}
    bool finished() const override { return false; }
    std::string output_filename(const std::string &filename_base = std::string()) const override
    {
        return this->PrintBase::output_filename("{input_filename_base}_{num_objects}.gcode", ".gcode", filename_base, nullptr);
    }
};

class TestObject : public PrintObjectBaseWithState<TestPrint, TestStep, tsCount>
{
public:
    TestObject(TestPrint *print) : PrintObjectBaseWithState(print, nullptr) {}
};

static std::vector<PrintBase::SlicingStatus> g_status;

static void write_status(const PrintBase &print, const TestObject &obj)
{
    ti((long long) g_status.size());
    for (auto &s : g_status) {
        ti(s.percent);
        ts(s.text);
        ti(s.flags);
        tok(s.warning_object_id == print.id() ? "P" : s.warning_object_id == obj.id() ? "O" : "-");
        ti(s.warning_step);
        ti((int) s.message_type);
        ti((int) s.warning_level);
    }
    g_status.clear();
}

static void case_print_with_state()
{
    for (int it = 0; it < 300; ++it) {
        TestPrint  print;
        TestObject obj(&print);
        g_status.clear();
        bool with_cb = rb(85);
        if (with_cb)
            print.set_status_callback([](const PrintBase::SlicingStatus &s) { g_status.push_back(s); });
        int cancel_calls = 0;
        print.set_cancel_callback([&cancel_calls]() { ++cancel_calls; });
        g_base = PrintStateBase::g_last_timestamp;
        std::string line_in = std::string("print_with_state ") + (with_cb ? "1" : "0");
        std::string line_out;
        int n = ri(1, 30);
        for (int k = 0; k < n; ++k) {
            int  op   = ri(0, 19);
            bool on_obj = rb(40);
            TestStep step = TestStep(ri(0, tsCount - 1));
            out.clear();
            auto *ps = on_obj ? &obj.m_state : &print.m_state;
            std::string in;
            if (op == 0) {
                in = "S " + std::to_string(on_obj) + " " + std::to_string(step);
                try {
                    ti(on_obj ? obj.set_started(step) : print.set_started(step));
                } catch (CanceledException &) {
                    tok("!C");
                }
            } else if (op == 1) {
                if (ps->m_step_active < 0 || ps->m_state[ps->m_step_active].state != PrintStateBase::STARTED) {
                    --k;
                    continue;
                }
                TestStep s = TestStep(ps->m_step_active);
                in = "D " + std::to_string(on_obj) + " " + std::to_string(s);
                try {
                    tstamp(on_obj ? obj.set_done(s) : print.set_done(s));
                } catch (CanceledException &) {
                    tok("!C");
                }
            } else if (op == 2) {
                in = "I " + std::to_string(on_obj) + " " + std::to_string(step);
                ti(on_obj ? obj.invalidate_step(step) : print.invalidate_step(step));
            } else if (op == 3) {
                std::vector<TestStep> steps;
                for (int j = 0; j < ri(0, 3); ++j)
                    steps.push_back(TestStep(ri(0, tsCount - 1)));
                in = "M " + std::to_string(on_obj) + " " + std::to_string(steps.size());
                for (auto x : steps)
                    in += " " + std::to_string(x);
                ti(on_obj ? obj.invalidate_steps(steps.begin(), steps.end()) : print.invalidate_steps(steps.begin(), steps.end()));
            } else if (op == 4) {
                int which = on_obj ? ri(0, 1) : 0;
                in = "A " + std::to_string(on_obj) + " " + std::to_string(which);
                ti(on_obj ? (which ? obj.invalidate_all_steps_without_cancel() : obj.invalidate_all_steps()) : print.invalidate_all_steps());
            } else if (op == 5) {
                if (ps->m_step_active < 0 || ps->m_state[ps->m_step_active].state != PrintStateBase::STARTED) {
                    --k;
                    continue;
                }
                int level = ri(0, 1), mi = ri(0, (int) g_messages.size() - 1), id = rb(50) ? 0 : ri(1, 3);
                in = "W " + std::to_string(on_obj) + " " + std::to_string(level) + " " + std::to_string(mi) + " " + std::to_string(id);
                if (on_obj)
                    obj.active_step_add_warning(PrintStateBase::WarningLevel(level), g_messages[mi], PrintStateBase::SlicingNotificationType(id));
                else
                    print.active_step_add_warning(PrintStateBase::WarningLevel(level), g_messages[mi], PrintStateBase::SlicingNotificationType(id));
            } else if (op == 6) {
                in = "T " + std::to_string(on_obj) + " " + std::to_string(step);
                write_state(on_obj ? obj.step_state_with_timestamp(step) : print.step_state_with_timestamp(step));
                write_warnings(on_obj ? obj.step_state_with_warnings(step) : print.step_state_with_warnings(step));
            } else if (op == 7) {
                in = "Q " + std::to_string(on_obj) + " " + std::to_string(step);
                ti(on_obj ? obj.is_step_done(step) : print.is_step_done(step));
                ti(on_obj ? obj.is_step_started_unguarded(step) : print.is_step_started_unguarded(step));
                ti(on_obj ? obj.is_step_done_unguarded(step) : print.is_step_done_unguarded(step));
            } else if (op == 8) {
                in = "CU";
                print.cancel();
            } else if (op == 9) {
                in = "CI";
                print.cancel_internal();
            } else if (op == 10 || op == 11) {
                in = "R";
                print.restart();
            } else if (op == 12) {
                in = "CS";
                ti((int) print.cancel_status());
                ti(print.canceled());
                try {
                    if (on_obj)
                        obj.throw_if_canceled();
                    else
                        print.throw_if_canceled();
                    ti(0);
                } catch (CanceledException &) {
                    tok("!C");
                }
                PrintTryCancel tc = print.make_try_cancel();
                try {
                    tc();
                    ti(0);
                } catch (CanceledException &) {
                    tok("!C");
                }
            } else if (op == 13) {
                int percent = ri(-1, 100), mi = ri(0, (int) g_messages.size() - 1), flags = ri(0, 63), wstep = ri(-1, 3);
                in = "ST " + std::to_string(percent) + " " + std::to_string(mi) + " " + std::to_string(flags) + " " + std::to_string(wstep);
                print.set_status(percent, g_messages[mi], unsigned(flags), wstep);
            } else if (op == 14) {
                int idx = ri(-1, 9);
                in = "PI " + std::to_string(idx);
                print.set_plate_index(idx);
                ti(print.get_plate_index());
            } else if (op == 15) {
                int nc = ri(0, 1);
                in = "NC " + std::to_string(nc);
                print.set_no_check_flag(nc);
                ti(print.get_no_check_flag());
            } else if (op == 16) {
                int mi = ri(0, (int) g_messages.size() - 1);
                in = "PN " + std::to_string(mi);
                print.set_plate_name(g_messages[mi]);
                ts(print.get_plate_name());
            } else if (op == 17) {
                in = "CC";
                print.call_cancel_callback();
            } else if (op == 18) {
                int level = ri(0, 1), mi = ri(0, (int) g_messages.size() - 1), wstep = ri(-1, 3), id = ri(0, 6);
                in = "UW " + std::to_string(on_obj) + " " + std::to_string(level) + " " + std::to_string(mi) + " " + std::to_string(wstep) + " " + std::to_string(id);
                if (on_obj)
                    obj.status_update_warnings(&print, wstep, PrintStateBase::WarningLevel(level), g_messages[mi], PrintStateBase::SlicingNotificationType(id));
                else
                    print.status_update_warnings(wstep, PrintStateBase::WarningLevel(level), g_messages[mi], nullptr, PrintStateBase::SlicingNotificationType(id));
            } else {
                in = "SD";
                if (rb(50))
                    print.set_status_default();
                else
                    print.set_status_silent();
                in += print.m_status_callback ? " 1" : " 0";
            }
            ti(cancel_calls);
            write_status(print, obj);
            line_in += " " + in;
            line_out += (line_out.empty() ? "" : " ") + out;
            out.clear();
        }
        fputs((line_in + " => " + line_out + "\n").c_str(), stdout);
    }
}

// ---------------------------------------------------------------------------------------------
// Output file names
// ---------------------------------------------------------------------------------------------

static void write_cfg_keys(const DynamicConfig &cfg)
{
    for (const char *k : {"num_objects", "num_instances"})
        ti(cfg.opt_int(k));
    auto *scale = cfg.option<ConfigOptionStrings>("scale");
    ti((long long) scale->values.size());
    for (auto &s : scale->values)
        ts(s);
    for (const char *k : {"input_filename", "input_filename_base"}) {
        auto *o = cfg.option<ConfigOptionString>(k);
        if (o)
            ts(o->value);
        else
            tok("none");
    }
}

static void case_output_filename()
{
    const std::vector<std::string> names{"", "Benchy", "part.one", "my model"};
    const std::vector<std::string> files{"", "/models/3DBenchy.stl", "dir/sub/thing.3mf", "noext", "a.b.c.obj", "/x/y/"};
    const std::vector<std::string> formats{"", "{input_filename_base}", "[input_filename_base]_{num_objects}",
                                           "out_{num_instances}.3mf", "noext", "{scale[0]}", "{input_filename}",
                                           "{input_filename_base}.gcode", "plate", "{num_objects}x{num_instances}.bgcode"};
    const std::vector<std::string> exts{".gcode", ".3mf", ".bgcode", "gcode"};
    for (int it = 0; it < 400; ++it) {
        TestPrint print;
        int nobj = ri(0, 3);
        for (int o = 0; o < nobj; ++o) {
            ModelObject *mo = print.m_model.add_object();
            mo->name        = names[ri(0, (int) names.size() - 1)];
            mo->input_file  = files[ri(0, (int) files.size() - 1)];
            mo->printable   = rb(85);
            int ninst       = ri(0, 3);
            for (int k = 0; k < ninst; ++k) {
                ModelInstance *mi = mo->add_instance();
                mi->set_scaling_factor(Vec3d(rb(50) ? 1.0 : rd(0.2, 3.0), rb(50) ? 1.0 : rd(0.2, 3.0), rb(70) ? 1.0 : rd(0.2, 3.0)));
                mi->printable = rb(85);
                if (rb(10))
                    mi->print_volume_state = ModelInstancePVS_Partly_Outside;
            }
        }
        std::string format = formats[ri(0, (int) formats.size() - 1)];
        std::string ext    = exts[ri(0, (int) exts.size() - 1)];
        std::string base   = rb(30) ? std::string("given_base") : std::string();
        bool        ovr    = rb(25);
        DynamicConfig override_cfg;
        override_cfg.set_key_value("input_filename_base", new ConfigOptionString("ovr"));
        override_cfg.set_key_value("num_objects", new ConfigOptionInt(77));
        {
            // An empty format reads input_filename_base without a test (a null dereference when no
            // object names a file): give a base then.
            DynamicConfig probe;
            print.update_object_placeholders(probe, ext);
            if (format.empty() && base.empty() && !ovr && !probe.has("input_filename_base"))
                base = "given_base";
        }

        tok("output_filename");
        ti((long long) print.m_model.objects.size());
        for (auto *mo : print.m_model.objects) {
            ts(mo->name);
            ts(mo->input_file);
            ti((long long) mo->instances.size());
            for (auto *mi : mo->instances) {
                ti(mi->is_printable());
                td(mi->get_scaling_factor(X));
                td(mi->get_scaling_factor(Y));
                td(mi->get_scaling_factor(Z));
            }
        }
        ts(print.m_model.propose_export_file_name_and_path());
        ts(format);
        ts(ext);
        ts(base);
        ti(ovr);
        tok("=>");
        DynamicConfig cfg;
        print.update_object_placeholders(cfg, ext);
        write_cfg_keys(cfg);
        try {
            ts(print.PrintBase::output_filename(format, ext, base, ovr ? &override_cfg : nullptr));
        } catch (const Slic3r::PlaceholderParserError &e) {
            tok("!P");
            ts(e.what());
        }
        for (const char *path : {"", "/", "/nonexistent-dir/file.gcode"}) {
            try {
                ts(print.output_filepath(path, base));
            } catch (const Slic3r::PlaceholderParserError &e) {
                tok("!P");
                ts(e.what());
            }
        }
        endl_();
    }
}

int main()
{
    case_print_state();
    case_print_with_state();
    case_output_filename();
    return 0;
}
