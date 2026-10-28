#!/bin/bash
# final-verify.sh — clean Snapmaker rebuild, relink all three programs, run every test, the parity check and the package.
set -u; W="$(cd "$(dirname "$0")/../.." && pwd)"; cd "$W"; L=decision-log/final; mkdir -p $L
export CCACHE_BASEDIR="$HOME/Projects/Snapmaker" CCACHE_DIR="$W/deps-prefix/ccache"
step(){ echo "$(date '+%H:%M:%S') $*" | tee -a $L/steps.txt; }
step "reconfigure snapmaker with CGAL 5.4"; rm -rf build-snapmaker; decision-log/scripts/configure.sh snapmaker build-snapmaker > $L/configure-snapmaker.log 2>&1 || step "CONFIGURE FAILED"
for e in snapmaker orca bambu; do s=$(date +%s); /usr/bin/lockf -t 3600 deps-prefix/build.lock cmake --build build-$e --target slicer_cli -j8 > $L/build-$e.log 2>&1; step "build $e rc=$? $(( $(date +%s)-s ))s"; done
for e in snapmaker orca bambu; do for t in test_diagnostic_events test_excluded_features; do bash tests/$t.sh build-$e/slicer_cli > $L/$t-$e.log 2>&1; step "$t $e rc=$? $(grep -E 'Passed:|passed' $L/$t-$e.log | tail -1)"; done; done
bash tests/test_nozzle_map_provenance.sh build-bambu/slicer_cli > $L/nozzle-bambu.log 2>&1; step "test_nozzle_map_provenance bambu rc=$? $(tail -1 $L/nozzle-bambu.log)"
cd work/u1-tests; bash run_u1_tests.sh ../../build-snapmaker/slicer_cli out-final > ../../$L/u1-final.log 2>&1; cd "$W"; step "u1 suite: $(tail -1 $L/u1-final.log)"
: > $L/parity.txt
for g in work/u1-tests/out-smapp/*/*.gcode; do c=$(basename $(dirname $g)); b=work/u1-tests/out-final/$c/$(basename $g); [ -f "$b" ] && echo "$c/$(basename $g): $(python3 -I work/u1-tests/compare_gcode.py "$g" "$b" | sed -n 3p)" >> $L/parity.txt; done
step "parity: $(grep -c IDENTICAL $L/parity.txt) identical of $(grep -c '' $L/parity.txt)"
build-orca/slicer_cli work/models/cube20.stl --printer-preset "Snapmaker U1 (0.4 nozzle)" --process-preset "0.20 Standard @Snapmaker U1 (0.4 nozzle)" -o work/out-orca/cube20-final.gcode > /dev/null 2>&1
step "orca baseline unchanged: $(python3 -I work/u1-tests/compare_gcode.py work/out-orca/cube20.gcode work/out-orca/cube20-final.gcode | sed -n 3p)"
PKG_NO_LOCK=1 RUN_TESTS=1 bash scripts/package-local-macos.sh pkg-local > $L/package.log 2>&1; step "package rc=$? $(grep -iE 'passed|failed' $L/package.log | tail -3 | tr '\n' ' ')"
rm -rf work/pkgcheck && mkdir -p work/pkgcheck && tar -xzf $(ls pkg-local/release/*.tar.gz | head -1) -C work/pkgcheck; P=work/pkgcheck/slicer-cli
mkdir -p work/u1-tests/out-pkg/four_colour; "$P/slicer_cli-snapmaker" work/u1-tests/models/four_colour.3mf --slice 0 --outputdir work/u1-tests/out-pkg/four_colour > $L/pkg-four.log 2>&1
step "packaged snapmaker four_colour vs Snapmaker CLI: $(python3 -I work/u1-tests/compare_gcode.py work/u1-tests/out-smapp/four_colour/plate_1.gcode work/u1-tests/out-pkg/four_colour/plate_1.gcode | sed -n 3p)"
step DONE
