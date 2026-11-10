// cli_list_flags.hpp — the program's flags as one JSON document.
//
// --list-flags prints every flag this program takes, with what it does, what
// value it takes and which binaries of the package take it, the way
// --list-settings prints the settings table: a caller that drives this binary
// (a worker, an agent) reads the contract from the binary it runs instead of
// from the documentation, and a flag added in a release is a visible change
// of this document (tests/test_list_flags.sh, schemas/list-flags.schema.json).
#pragma once

#include <string>

namespace slicer_cli {

/// Prints the document on stdout and returns the exit code (0). `engine` is
/// the name result.json uses (bambustudio, orcaslicer, snapmaker).
int run_list_flags(const std::string& engine, const std::string& engine_version);

} // namespace slicer_cli
