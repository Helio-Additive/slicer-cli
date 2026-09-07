#!/usr/bin/env python3
"""Exercise the production stdout writer without fatal-signal interception without an engine build."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = r'''
#include "diagnostic_output.hpp"
#include <iostream>
#include <thread>
#include <vector>
int main() {
    std::vector<std::thread> writers;
    for (int t = 0; t < 3; ++t) {
        writers.emplace_back([t] {
            for (int i = 0; i < 200; ++i) {
                std::cout << "partial text " << i;
                std::printf("printf text %d", i);
                slicer_cli::diagnostics::write_event(
                    "{\"thread\":" + std::to_string(t) + ",\"index\":" +
                    std::to_string(i) + ",\"payload\":\"" + std::string(16384, 'x') + "\"}");
            }
        });
    }
    for (auto& writer : writers) writer.join();
}
'''
with tempfile.TemporaryDirectory(prefix="diagnostic-transport-") as tmp:
    source = Path(tmp) / "probe.cpp"
    binary = Path(tmp) / "probe"
    source.write_text(SOURCE)
    subprocess.run(shlex.split(os.environ.get("CXX", "c++")) +
                   ["-std=c++17", "-pthread", "-I", str(ROOT), str(source), "-o", str(binary)],
                   check=True)
    output = subprocess.run([str(binary)], capture_output=True, check=True, timeout=30).stdout.decode()
    prefix = "[[SLICER_EVENT]] "
    events = []
    for line in output.splitlines():
        if prefix in line:
            assert line.startswith(prefix), repr(line[:100])
            event = json.loads(line[len(prefix):])
            assert event["payload"] == "x" * 16384
            events.append((event["thread"], event["index"]))
    assert set(events) == {(t, i) for t in range(3) for i in range(200)}
    assert len(events) == 600
    print("PASS: 600 complete events amid concurrent cout/printf, including >PIPE_BUF payloads")
