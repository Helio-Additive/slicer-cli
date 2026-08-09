# A/B TEMPLATE — R673. EVERY arm CHECKS THE EXIT CODE BEFORE READING THE OUTPUT.
# R672 lost a round because a failed slice left the PREVIOUS run's .gcode in place
# and the script hashed it anyway, reporting "byte-identical" for a crashed arm.
cd /Users/alex/Code/Helio-Additive/worktrees/slicer-cli/lofty-dawn/slicer-cli
D=/Users/alex/.claude/jobs/e2b92bdb/tmp
GATE=${GATE:-ARACHNE_SIMPLIFY_SCALED}
for arm in ON OFF; do
  if [ "$arm" = "ON" ]; then export $GATE=1; else unset $GATE; fi
  rm -f tests/.tmp/nu3mf/majorasmask.gcode
  ./target/release/slicer-cli slice --engine rust --config tests/configs/nu3mf.jsonnet > $D/ab_$arm.log 2>&1
  rc=$?
  if [ $rc -ne 0 ] || [ ! -f tests/.tmp/nu3mf/majorasmask.gcode ]; then
    printf "%-3s SLICE FAILED rc=%s — NO OUTPUT, arm is UNUSABLE\n" "$arm" "$rc"
    grep -i "panicked at" $D/ab_$arm.log | tail -2
    continue
  fi
  cp tests/.tmp/nu3mf/majorasmask.gcode $D/ab_maj_$arm.gcode
  printf "%-3s rc=0 maj=%s\n" "$arm" "$(shasum -a256 $D/ab_maj_$arm.gcode|cut -c1-8)"
done
echo ok
