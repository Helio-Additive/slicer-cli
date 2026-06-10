import { beforeAll, describe, expect, test } from "bun:test";
import { Jsonnet } from "@hanazuki/node-jsonnet";
import { createHash } from "node:crypto";
import { createServer } from "node:http";
import { mkdir, readFile, rm, writeFile } from "node:fs/promises";
import { basename, dirname, join } from "node:path";

type TestMode = "devbox" | "docker";

interface CommandResult {
  exitCode: number;
  stdout: string;
  stderr: string;
}

interface SliceOptions {
  dryRun?: boolean;
  nativeBinary?: string;
}

interface Triangle {
  a: number;
  b: number;
  c: number;
}

interface Vertex {
  x: number;
  y: number;
  z: number;
}

const mode = testMode();
const dockerImage = process.env.SLICER_CLI_TEST_IMAGE ?? "slicer-cli:local";
const root = join(
  process.cwd(),
  "tests",
  ".tmp",
  `${process.pid}-${Date.now()}-${Math.random().toString(16).slice(2)}`,
);
let built = false;

const base64Output = "tests/base64-job-config.out.gcode";
const local3mfOutput = "tests/3mf-job-local.out.gcode";
const http3mfOutput = "tests/3mf-job-http.out.gcode";

const expectedBase64Sha256 = "f420aee59b626162516bd63a7d80d20c28fc38a7721dca1ece1d8318cb16e236";
const expectedBase64SizeBytes = 3_022_221;
const expected3mfSha256 = "6c0ed1dffc75516bf2ffd18e3c47d72f8eae0779fab9116177a16e8c01475891";
const expected3mfSizeBytes = 3_014_167;

beforeAll(async () => {
  await mkdir(root, { recursive: true });
  if (!built && mode === "devbox") {
    await run("cargo", ["build"]);
    built = true;
  }
});

describe("Slicing tests", () => {
  test("slices STL with machine, filament, and process configs loaded from files", async () => {
    const result = await slice("tests/configs/stl-file-config.jsonnet");

    expect(result.exitCode).toBe(0);
    expect(result.stdout).toContain("tests/.tmp/stl-file-config/out.gcode");
    expect(result.stdout).toContain("_downloads/3DBenchy.stl");
  });

  test("slices STL from a Jsonnet job config with imported Bambu presets", async () => {
    const result = await slice("tests/configs/stl-inline-config.jsonnet");

    expect(result.exitCode).toBe(0);
    expect(result.stdout).toContain("--config");
    expect(result.stdout).toContain("tests/.tmp/stl-inline-config/benchy.gcode");
    expect(result.stdout).toContain("_downloads/3DBenchy.stl");
  });

  test("rejects STL jobs missing one of machine, filament, and process", async () => {
    const result = await slice("tests/configs/stl-invalid-config.jsonnet");

    expect(result.exitCode).toBe(1);
    expect(result.stderr).toContain("machine, filament, and process");
  });

  test("accepts the entire slice job config as base64 JSON", async () => {
    await rm(base64Output, { force: true });
    const json = await new Jsonnet().evaluateFile("tests/configs/base64-job-config.jsonnet");
    const config = Buffer.from(json).toString("base64");

    const result = await slice(config, {
      dryRun: false,
    });

    expect(result.exitCode).toBe(0);
    await expectGcodeOutput(base64Output, expectedBase64SizeBytes, expectedBase64Sha256);
  }, 30_000);

  test("slices a local prepared 3MF generated from the shared STL", async () => {
    await rm(local3mfOutput, { force: true });
    const generated = await generatePrepared3mf();
    await expectPrepared3mf(generated);

    const config = await jsonFixture("local-3mf-job.json", {
      input: {
        type: "3mf",
        model: generated.prepared3mf,
      },
      output: { gcode: local3mfOutput },
    });

    const result = await slice(config, {
      dryRun: false,
    });

    expect(result.exitCode).toBe(0);
    await expectGcodeOutput(local3mfOutput, expected3mfSizeBytes, expected3mfSha256);
  }, 30_000);

  test("downloads an HTTP prepared 3MF generated from the shared STL, then slices it", async () => {
    await rm(http3mfOutput, { force: true });
    const generated = await generatePrepared3mf();
    await expectPrepared3mf(generated);

    const fixture = await startHttpFixture(generated.prepared3mf);
    if (fixture === null) {
      console.warn("skipping HTTP fixture test because this environment cannot bind localhost");
      return;
    }
    const { port, server } = fixture;

    try {
      const config = await jsonFixture("http-3mf-job.json", {
        input: {
          type: "3mf",
          model: `http://127.0.0.1:${port}/remote.3mf`,
        },
        output: { gcode: http3mfOutput },
      });

      const result = await slice(config, {
        dryRun: false,
      });

      expect(result.exitCode).toBe(0);
      await expectGcodeOutput(http3mfOutput, expected3mfSizeBytes, expected3mfSha256);
    } finally {
      await new Promise<void>((resolve, reject) => {
        server.close((error) => (error ? reject(error) : resolve()));
      });
    }
  }, 30_000);
});

describe("Preset tests", () => {
  test("resolves printer, filament, and process preset names from a profile root", async () => {
    const profileRoot = join(root, "named-presets");
    const output = join(root, "resolved-presets.json");
    await mkdir(join(profileRoot, "machine"), { recursive: true });
    await mkdir(join(profileRoot, "filament"), { recursive: true });
    await mkdir(join(profileRoot, "process"), { recursive: true });
    await writeFile(
      join(profileRoot, "machine", "printer.json"),
      JSON.stringify(
        {
          name: "Test Printer",
          nozzle_diameter: ["0.4"],
          bed_temperature: [55],
        },
        null,
        2,
      ),
    );
    await writeFile(
      join(profileRoot, "filament", "filament.json"),
      JSON.stringify(
        {
          name: "Test PLA",
          filament_type: ["PLA"],
          filament_colour: ["#00ff00"],
        },
        null,
        2,
      ),
    );
    await writeFile(
      join(profileRoot, "process", "process.json"),
      JSON.stringify(
        {
          name: "0.16mm Test Process",
          layer_height: 0.16,
          sparse_infill_density: "15%",
        },
        null,
        2,
      ),
    );

    const result = await presets({
      machine: "Test Printer",
      filament: "Test PLA",
      process: "0.16mm Test Process",
      output,
      profileRoot,
    });

    expect(result.exitCode).toBe(0);
    const resolved = JSON.parse(await readFile(output, "utf8"));
    expect(resolved.nozzle_diameter).toEqual(["0.4"]);
    expect(resolved.bed_temperature).toEqual([55]);
    expect(resolved.filament_type).toEqual(["PLA"]);
    expect(resolved.filament_colour).toEqual(["#00ff00"]);
    expect(resolved.layer_height).toBe(0.16);
    expect(resolved.sparse_infill_density).toBe("15%");
    expect(resolved.name).toBeUndefined();
  });
});

describe("Profile catalog tests", () => {
  test("lists profiles by kind to stdout", async () => {
    const profileRoot = join(root, "profile-catalog");
    await mkdir(join(profileRoot, "BBL", "machine"), { recursive: true });
    await mkdir(join(profileRoot, "BBL", "filament", "Polymaker"), { recursive: true });
    await mkdir(join(profileRoot, "BBL", "process"), { recursive: true });
    await writeFile(
      join(profileRoot, "BBL", "machine", "printer.json"),
      JSON.stringify({ name: "Catalog Printer", type: "machine" }, null, 2),
    );
    await writeFile(
      join(profileRoot, "BBL", "filament", "Polymaker", "pla.json"),
      JSON.stringify({ name: "Catalog PLA", type: "filament" }, null, 2),
    );
    await writeFile(
      join(profileRoot, "BBL", "process", "standard.json"),
      JSON.stringify({ name: "Catalog Process", type: "process" }, null, 2),
    );

    const result = await profilesList({ kind: "filament", profileRoot });

    expect(result.exitCode).toBe(0);
    const profiles = JSON.parse(result.stdout);
    expect(profiles).toEqual([
      {
        name: "Catalog PLA",
        kind: "filament",
        path: join(profileRoot, "BBL", "filament", "Polymaker", "pla.json"),
        vendor: "BBL",
      },
    ]);
  });
});

async function slice(config: string, options: SliceOptions = {}): Promise<CommandResult> {
  const dryRun = options.dryRun ?? true;

  if (mode === "docker") {
    const args = [
      "run",
      "--rm",
      "--volume",
      `${process.cwd()}:${process.cwd()}`,
      "--workdir",
      process.cwd(),
      dockerImage,
      "slice",
      "--config",
      config,
    ];
    if (dryRun) {
      args.push("--dry-run");
    }
    return run("docker", args);
  }

  const nativeBinary =
    options.nativeBinary ??
    (dryRun ? "/bin/echo" : (process.env.BAMBUSTUDIO_SLICER ?? "libslic3r/bambustudio/build/slicer_cli"));
  const args = ["slice", "--config", config, "--native-binary", nativeBinary];
  if (dryRun) {
    args.push("--dry-run");
  }

  return run(join(process.cwd(), "target", "debug", "slicer-cli"), args);
}

async function presets(options: {
  machine: string;
  filament: string;
  process: string;
  output: string;
  profileRoot: string;
}): Promise<CommandResult> {
  const args = [
    "presets",
    "--machine",
    options.machine,
    "--filament",
    options.filament,
    "--process",
    options.process,
    "--output",
    options.output,
    "--profile-root",
    options.profileRoot,
  ];

  if (mode === "docker") {
    return run("docker", [
      "run",
      "--rm",
      "--volume",
      `${process.cwd()}:${process.cwd()}`,
      "--workdir",
      process.cwd(),
      dockerImage,
      ...args,
    ]);
  }

  return run(join(process.cwd(), "target", "debug", "slicer-cli"), args);
}

async function profilesList(options: {
  kind: "machine" | "filament" | "process";
  profileRoot: string;
}): Promise<CommandResult> {
  const args = ["profiles", "list", "--kind", options.kind, "--profile-root", options.profileRoot];

  if (mode === "docker") {
    return run("docker", [
      "run",
      "--rm",
      "--volume",
      `${process.cwd()}:${process.cwd()}`,
      "--workdir",
      process.cwd(),
      dockerImage,
      ...args,
    ]);
  }

  return run(join(process.cwd(), "target", "debug", "slicer-cli"), args);
}

async function jsonFixture(name: string, value: unknown): Promise<string> {
  return fixture(name, `${JSON.stringify(value, null, 2)}\n`);
}

async function fixture(name: string, contents: string): Promise<string> {
  const path = join(root, `${Date.now()}-${basename(name)}`);
  await writeFile(path, contents);
  return path;
}

async function run(command: string, args: string[]): Promise<CommandResult> {
  const proc = Bun.spawn([command, ...args], {
    cwd: process.cwd(),
    stderr: "pipe",
    stdout: "pipe",
  });
  const [stdout, stderr, exitCode] = await Promise.all([
    new Response(proc.stdout).text(),
    new Response(proc.stderr).text(),
    proc.exited,
  ]);
  return { exitCode, stdout, stderr };
}

async function generatePrepared3mf(): Promise<{ prepared3mf: string; resolvedConfig: string }> {
  const dir = join(root, "3mf-job");
  await mkdir(dir, { recursive: true });

  const sourceJob = JSON.parse(await new Jsonnet().evaluateFile("tests/configs/3mf-source-config.jsonnet"));
  const resolvedConfig = join(dir, "project_settings.config");
  sourceJob.output.gcode = join(dir, "source.gcode");
  sourceJob.output.resolved_config = resolvedConfig;

  const sourceConfig = await jsonFixture("prepared-3mf-source.json", sourceJob);
  const resolved = await slice(sourceConfig);
  expect(resolved.exitCode).toBe(0);

  const prepared3mf = join(dir, "remote.3mf");
  await writePrepared3mfFromStl("_downloads/3DBenchy.stl", resolvedConfig, prepared3mf);
  return { prepared3mf, resolvedConfig };
}

async function expectPrepared3mf(generated: { prepared3mf: string; resolvedConfig: string }): Promise<void> {
  const prepared3mf = await readFile(generated.prepared3mf);
  expect(prepared3mf.subarray(0, 4).toString("hex")).toBe("504b0304");
  expect(prepared3mf.toString("utf8")).toContain("Metadata/project_settings.config");
  expect(prepared3mf.toString("utf8")).toContain("3D/3dmodel.model");
  expect(await readFile(generated.resolvedConfig, "utf8")).toContain('"print_settings_id"');
}

async function expectGcodeOutput(path: string, expectedSizeBytes: number, expectedSha256: string): Promise<void> {
  const bytes = await readFile(path);
  const gcode = bytes.toString("utf8");
  expect(gcode).toContain("; HEADER_BLOCK_START");
  expect(gcode).toContain("; BambuStudio");
  expect(bytes.byteLength).toBe(expectedSizeBytes);
  expect(createHash("sha256").update(bytes).digest("hex")).toBe(expectedSha256);
}

async function startHttpFixture(prepared3mf: string): Promise<{
  port: number;
  server: ReturnType<typeof createServer>;
} | null> {
  const body = await readFile(prepared3mf);
  for (let port = 39_120; port < 39_140; port += 1) {
    const server = createServer((_request, response) => {
      response.writeHead(200, {
        "content-length": body.byteLength,
        "content-type": "model/3mf",
      });
      response.end(body);
    });
    const started = await new Promise<boolean>((resolve) => {
      server.once("error", () => resolve(false));
      server.listen(port, "127.0.0.1", () => resolve(true));
    });
    if (started) {
      return { port, server };
    }
  }
  return null;
}

async function writePrepared3mfFromStl(stlPath: string, projectConfigPath: string, outputPath: string): Promise<void> {
  const [stl, projectConfig] = await Promise.all([readFile(stlPath), readFile(projectConfigPath)]);
  const mesh = parseBinaryStl(stl);
  const model = modelXml(mesh.vertices, mesh.triangles);

  await mkdir(dirname(outputPath), { recursive: true });
  await writeFile(
    outputPath,
    zipStore({
      "_rels/.rels": relationshipsXml(),
      "[Content_Types].xml": contentTypesXml(),
      "3D/3dmodel.model": model,
      "Metadata/project_settings.config": projectConfig,
    }),
  );
}

function parseBinaryStl(stl: Buffer): { vertices: Vertex[]; triangles: Triangle[] } {
  if (stl.byteLength < 84) {
    throw new Error("STL is too small to be a binary STL");
  }

  const triangleCount = stl.readUInt32LE(80);
  const expectedSize = 84 + triangleCount * 50;
  if (expectedSize > stl.byteLength) {
    throw new Error(`binary STL is truncated: expected ${expectedSize} bytes, got ${stl.byteLength}`);
  }

  const vertices: Vertex[] = [];
  const triangles: Triangle[] = [];
  const vertexIds = new Map<string, number>();

  for (let triangleIndex = 0; triangleIndex < triangleCount; triangleIndex += 1) {
    const offset = 84 + triangleIndex * 50 + 12;
    const indices: number[] = [];

    for (let vertexIndex = 0; vertexIndex < 3; vertexIndex += 1) {
      const vertexOffset = offset + vertexIndex * 12;
      const vertex = {
        x: stl.readFloatLE(vertexOffset),
        y: stl.readFloatLE(vertexOffset + 4),
        z: stl.readFloatLE(vertexOffset + 8),
      };
      const key = `${vertex.x.toFixed(6)},${vertex.y.toFixed(6)},${vertex.z.toFixed(6)}`;
      let id = vertexIds.get(key);
      if (id === undefined) {
        id = vertices.length;
        vertexIds.set(key, id);
        vertices.push(vertex);
      }
      indices.push(id);
    }

    triangles.push({ a: indices[0], b: indices[1], c: indices[2] });
  }

  return { vertices, triangles };
}

function modelXml(vertices: Vertex[], triangles: Triangle[]): Buffer {
  const lines = [
    '<?xml version="1.0" encoding="UTF-8"?>',
    '<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" xmlns:BambuStudio="http://schemas.bambulab.com/package/2021">',
    '  <metadata name="Application">BambuStudio-2.0.0.0</metadata>',
    '  <metadata name="BambuStudio:3mfVersion">1</metadata>',
    "  <resources>",
    '    <object id="1" name="3DBenchy" type="model">',
    "      <mesh>",
    "        <vertices>",
  ];

  for (const vertex of vertices) {
    lines.push(`          <vertex x="${xmlNumber(vertex.x)}" y="${xmlNumber(vertex.y)}" z="${xmlNumber(vertex.z)}" />`);
  }

  lines.push("        </vertices>", "        <triangles>");

  for (const triangle of triangles) {
    lines.push(`          <triangle v1="${triangle.a}" v2="${triangle.b}" v3="${triangle.c}" />`);
  }

  lines.push(
    "        </triangles>",
    "      </mesh>",
    "    </object>",
    "  </resources>",
    "  <build>",
    '    <item objectid="1" printable="1" />',
    "  </build>",
    "</model>",
    "",
  );

  return Buffer.from(lines.join("\n"));
}

function relationshipsXml(): Buffer {
  return Buffer.from(`<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Target="/3D/3dmodel.model" Id="rel-1" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>
`);
}

function contentTypesXml(): Buffer {
  return Buffer.from(`<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
  <Default Extension="config" ContentType="application/octet-stream"/>
</Types>
`);
}

function zipStore(files: Record<string, Buffer>): Buffer {
  const localParts: Buffer[] = [];
  const centralParts: Buffer[] = [];
  let offset = 0;

  for (const [name, data] of Object.entries(files)) {
    const nameBytes = Buffer.from(name);
    const crc = crc32(data);

    const local = Buffer.alloc(30);
    local.writeUInt32LE(0x04034b50, 0);
    local.writeUInt16LE(20, 4);
    local.writeUInt16LE(0, 6);
    local.writeUInt16LE(0, 8);
    local.writeUInt16LE(0, 10);
    local.writeUInt16LE(0, 12);
    local.writeUInt32LE(crc, 14);
    local.writeUInt32LE(data.byteLength, 18);
    local.writeUInt32LE(data.byteLength, 22);
    local.writeUInt16LE(nameBytes.byteLength, 26);
    local.writeUInt16LE(0, 28);
    localParts.push(local, nameBytes, data);

    const central = Buffer.alloc(46);
    central.writeUInt32LE(0x02014b50, 0);
    central.writeUInt16LE(20, 4);
    central.writeUInt16LE(20, 6);
    central.writeUInt16LE(0, 8);
    central.writeUInt16LE(0, 10);
    central.writeUInt16LE(0, 12);
    central.writeUInt16LE(0, 14);
    central.writeUInt32LE(crc, 16);
    central.writeUInt32LE(data.byteLength, 20);
    central.writeUInt32LE(data.byteLength, 24);
    central.writeUInt16LE(nameBytes.byteLength, 28);
    central.writeUInt16LE(0, 30);
    central.writeUInt16LE(0, 32);
    central.writeUInt16LE(0, 34);
    central.writeUInt16LE(0, 36);
    central.writeUInt32LE(0, 38);
    central.writeUInt32LE(offset, 42);
    centralParts.push(central, nameBytes);

    offset += local.byteLength + nameBytes.byteLength + data.byteLength;
  }

  const centralDirectory = Buffer.concat(centralParts);
  const end = Buffer.alloc(22);
  end.writeUInt32LE(0x06054b50, 0);
  end.writeUInt16LE(0, 4);
  end.writeUInt16LE(0, 6);
  end.writeUInt16LE(Object.keys(files).length, 8);
  end.writeUInt16LE(Object.keys(files).length, 10);
  end.writeUInt32LE(centralDirectory.byteLength, 12);
  end.writeUInt32LE(offset, 16);
  end.writeUInt16LE(0, 20);

  return Buffer.concat([...localParts, centralDirectory, end]);
}

function crc32(data: Buffer): number {
  let crc = 0xffffffff;
  for (const byte of data) {
    crc = (crc >>> 8) ^ crcTable[(crc ^ byte) & 0xff];
  }
  return (crc ^ 0xffffffff) >>> 0;
}

function xmlNumber(value: number): string {
  return Number.isInteger(value) ? String(value) : value.toFixed(6).replace(/0+$/, "").replace(/\.$/, "");
}

function testMode(): TestMode {
  const value = process.env.SLICER_CLI_TEST_MODE ?? "devbox";
  if (value === "devbox" || value === "docker") {
    return value;
  }
  throw new Error(`Unsupported SLICER_CLI_TEST_MODE: ${value}`);
}

const crcTable = new Uint32Array(256);
for (let n = 0; n < 256; n += 1) {
  let c = n;
  for (let k = 0; k < 8; k += 1) {
    c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
  }
  crcTable[n] = c >>> 0;
}
