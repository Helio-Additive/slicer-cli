// layout_spike — thin wrapper calling arrange_harness
#include <cstdio>
extern int run_arrange_spike(const char* profiles_dir);
int main(int argc, char** argv) {
    if (argc < 2) { std::fprintf(stderr, "Usage: layout_spike <profiles-dir>\n"); return 2; }
    return run_arrange_spike(argv[1]);
}
