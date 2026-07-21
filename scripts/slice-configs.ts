#!/usr/bin/env bun
// Slice every job config in tests/configs/ through BOTH engines into a single
// timestamped run directory — one subdirectory per config, each holding both
// G-codes (or an <engine>.error.txt on failure), the exact config that was
// sliced, and the resolved config.
//
//   tests/.tmp/<YYYY-MM-DD_HH-MM-SS>/
//     <config>/
//       bambu.gcode        bambu.resolved.config  bambu.config.json
//       rust.gcode         rust.resolved.config   rust.config.json
//       (or bambu.error.txt / rust.error.txt when that engine fails)
//     summary.txt
//
// "bambu" = the C++ BambuStudio libslic3r engine (`--engine bambu`).
// "rust"  = the in-process Rust engine (`--engine rust`).
//
// Pure artifact generation — no assertions. Run everything with
// `just slice-configs`, or a subset directly:
//   bun run scripts/slice-configs.ts stl-file-config nu3mf

import { Jsonnet } from "@hanazuki/node-jsonnet";
import { mkdir, readdir, writeFile } from "node:fs/promises";
import { basename, join } from "node:path";

const cwd = process.cwd();
const cli = join(cwd, "target", "debug", "slicer-cli");
const bambuBinary = process.env.BAMBUSTUDIO_SLICER ?? "libslic3r/bambustudio/build/slicer_cli";

const ENGINES = [
  { label: "bambu", flag: "bambu" },
  { label: "rust", flag: "rust" },
] as const;

// Optional positional args restrict the run to those config names (basename
// without .jsonnet), e.g. `bun run scripts/slice-configs.ts stl-file-config`.
const only = new Set(process.argv.slice(2));

function timestamp(): string {
  const d = new Date();
  const p = (n: number) => String(n).padStart(2, "0");
  return (
    `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())}` +
    `_${p(d.getHours())}-${p(d.getMinutes())}-${p(d.getSeconds())}`
  );
}

async function run(command: string, args: string[]) {
  const proc = Bun.spawn([command, ...args], { cwd, stdout: "pipe", stderr: "pipe" });
  const [stdout, stderr, exitCode] = await Promise.all([
    new Response(proc.stdout).text(),
    new Response(proc.stderr).text(),
    proc.exited,
  ]);
  return { exitCode, stdout, stderr };
}

const runDir = join(cwd, "tests", ".tmp", timestamp());
await mkdir(runDir, { recursive: true });
console.log(`Slicing configs → ${runDir}\n`);

const configsDir = join(cwd, "tests", "configs");
const jsonnetFiles = (await readdir(configsDir)).filter((f) => f.endsWith(".jsonnet")).sort();

const jsonnet = new Jsonnet();
const summary: string[] = [];

for (const file of jsonnetFiles) {
  const name = basename(file, ".jsonnet");
  if (only.size > 0 && !only.has(name)) continue;

  let job: unknown;
  try {
    job = JSON.parse(await jsonnet.evaluateFile(join(configsDir, file)));
  } catch {
    continue; // not evaluable to JSON
  }
  // Only job configs (they have an `input`); skip preset fragments.
  if (typeof job !== "object" || job === null || !("input" in job)) continue;

  const outDir = join(runDir, name);
  await mkdir(outDir, { recursive: true });

  for (const { label, flag } of ENGINES) {
    // Redirect this engine's outputs into the config's run subdirectory.
    const cfg = {
      ...(job as Record<string, unknown>),
      output: {
        gcode: join(outDir, `${label}.gcode`),
        resolved_config: join(outDir, `${label}.resolved.config`),
      },
    };
    const cfgPath = join(outDir, `${label}.config.json`);
    await writeFile(cfgPath, `${JSON.stringify(cfg, null, 2)}\n`);

    const res = await run(cli, [
      "slice",
      "--config",
      cfgPath,
      "--engine",
      flag,
      "--bambu-binary",
      bambuBinary,
    ]);

    if (res.exitCode === 0) {
      console.log(`  ✓ ${name}/${label}.gcode`);
      summary.push(`${name}/${label}: ok`);
    } else {
      await writeFile(
        join(outDir, `${label}.error.txt`),
        `exit code: ${res.exitCode}\n\n=== stdout ===\n${res.stdout}\n=== stderr ===\n${res.stderr}\n`,
      );
      console.log(`  ✗ ${name}/${label} — exit ${res.exitCode} (see ${label}.error.txt)`);
      summary.push(`${name}/${label}: FAILED (exit ${res.exitCode})`);
    }
  }
}

await writeFile(join(runDir, "summary.txt"), `${summary.join("\n")}\n`);
console.log(`\nDone — ${summary.length} slice(s). Artifacts + summary.txt under:\n  ${runDir}`);
