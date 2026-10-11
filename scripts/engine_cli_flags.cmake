# engine_cli_flags.cmake — one engine's command-line flags, read from its own
# option tables and written as a C++ header.
#
# slicer-cli builds one engine per build tree, and its walk over a refused
# command line has to read a flag's value the way that flag's own engine does —
# including the flags of the engine this build does not compile. Those are not
# in any table of this binary, so they are read from the other engine's own
# registrations at configure time (CMakeLists.txt, "The other engine's
# command-line flags").
#
# What is read, from <engine>/src/libslic3r/PrintConfig.cpp:
#
#   - the options of the config defs the engine's command line reads, which are
#     the ones DynamicPrintAndCLIConfig holds: PrintConfigDef and the CLI defs
#     CLIActionsConfigDef, CLITransformConfigDef, CLIMiscConfigDef. The file's
#     other defs (ReadWriteSlicingStatesConfigDef and the like) are G-code
#     variables, not flags, and are left out;
#   - each option's registration: this->add("key", coType), or OrcaSlicer's
#     new_def("key", coType, ...) macro. The flag's name is the key with dashes
#     unless the definition sets `cli` itself (ConfigOptionDef::cli_args:
#     Config.cpp 242-259 at BambuStudio 5873b5f, 238-255 at OrcaSlicer
#     31f6803); `cli = ConfigOptionDef::nocli` means the option is no flag;
#   - whether the flag reads a value, by the engine's own read_cli: one value
#     for every flag except a coBool or coBools one (Config.cpp 1660-1782 at
#     BambuStudio 5873b5f, 1591-1713 at OrcaSlicer 31f6803);
#   - a name the engine builds in a loop (this->add("machine_max_speed_" +
#     axis.name, coFloats)) is expanded over the string literals of the list
#     the loop runs over, as they are written above it.
#
# A registration this scan cannot read stops the configure: a flag the walk
# does not know is read as a model file, and naming one is what this table is
# for. The one exception is a registration over a key it builds from a string
# set (add_nullable(opt_key, ...) in a loop over filament_extruder_override_
# keys and the like): its names are right there in the set, but its type comes
# from the option the set entry names, in code the scan does not follow. Those
# are listed at configure time and left out — a key both engines have is in
# this binary's own table anyway, and one only the other engine has falls back
# to the walk's old reading (a model file, which keeps a file rather than
# writing over it).
#
# Usage: cmake -DENGINE_SRC=<PrintConfig.cpp> -DAPP_NAME=<name> -DOUT=<header> -P engine_cli_flags.cmake

if(NOT DEFINED ENGINE_SRC OR NOT DEFINED APP_NAME OR NOT DEFINED OUT)
  message(FATAL_ERROR "usage: cmake -DENGINE_SRC=<PrintConfig.cpp> -DAPP_NAME=<name> -DOUT=<header> -P engine_cli_flags.cmake")
endif()
if(NOT EXISTS "${ENGINE_SRC}")
  message(FATAL_ERROR
    "The ${APP_NAME} engine's source is not checked out: ${ENGINE_SRC} is missing.\n"
    "slicer-cli reads the command line of both engines, so both engine submodules are\n"
    "needed: git submodule update --init --recursive")
endif()

set(RE_CTOR       [[^([A-Za-z_][A-Za-z0-9_]*)::([A-Za-z_][A-Za-z0-9_]*)\(\)[ \t]*(const[ \t]*)?\{?[ \t]*$]])
set(RE_ADD        [[this->add\("([A-Za-z0-9_]+)"[ \t]*,[ \t]*(co[A-Za-z0-9_]+)\)]])
set(RE_NEWDEF     [[new_def\("([A-Za-z0-9_]+)"[ \t]*,[ \t]*(co[A-Za-z0-9_]+)]])
set(RE_COMPUTED   [[this->add\("([A-Za-z0-9_]+)"[ \t]*\+[^,]*,[ \t]*(co[A-Za-z0-9_]+)\)]])
set(RE_ENTRY      [[^[ \t]*\{[ \t]*"([^"]+)"[ \t]*,[ \t]*]])
set(RE_CLI        [[>[ \t]*cli[ \t]*=[ \t]*"([^"]*)"]])
set(RE_NOCLI      [[>[ \t]*cli[ \t]*=[ \t]*ConfigOptionDef::nocli]])
set(RE_OTHERADD   [[this->add\(|this->add_nullable\(]])
set(RE_VARIABLE   [[this->add_nullable\([A-Za-z_][A-Za-z0-9_]*[ \t]*,|this->add\([A-Za-z_][A-Za-z0-9_]*[ \t]*,]])

# The config defs the engine's command line reads (DynamicPrintAndCLIConfig).
set(command_defs PrintConfigDef CLIActionsConfigDef CLITransformConfigDef CLIMiscConfigDef)


file(STRINGS "${ENGINE_SRC}" lines)
set(entries "")
set(context "")
set(key "")
set(type "")
set(names "")
set(nocli FALSE)
set(parsed_literal_adds 0)
set(literal_registrations 0)
set(out_of_scope_registrations 0)
set(over_sets "")
# The string literals of the most recent { "value", ... } list: the axis names
# a loop-built option name is expanded over.
set(literals "")
set(run "")

# One entry per flag name: the option's key, its name, and whether the engine
# reads a value for it. A nocli option is no flag.
macro(flush_entry)
  if(NOT key STREQUAL "")
    if(nocli)
      # cli = ConfigOptionDef::nocli: this option is no command-line flag.
    else()
      if(type MATCHES "^coBools?$")
        set(takes_value "false")
      else()
        set(takes_value "true")
      endif()
      if(names STREQUAL "")
        string(REPLACE "_" "-" dashed "${key}")
        list(APPEND entries "    { \"${key}\", \"${dashed}\", ${takes_value} },")
      else()
        # The names the definition sets itself (export-gcode|gcode|g).
        string(REPLACE "|" ";" cli_names "${names}")
        foreach(cli_name IN LISTS cli_names)
          list(APPEND entries "    { \"${key}\", \"${cli_name}\", ${takes_value} },")
        endforeach()
      endif()
    endif()
  endif()
  set(key "")
  set(names "")
  set(nocli FALSE)
endmacro()

foreach(line IN LISTS lines)
  string(REGEX REPLACE "^[ \t]+" "" trimmed "${line}")
  if(trimmed MATCHES "^//")
    continue()
  endif()
  # A preprocessor line: OrcaSlicer's new_def macro is defined here, its body
  # joined into one line (file(STRINGS) follows the backslash continuations).
  # Its invocations are the registrations, and they are read below.
  if(trimmed MATCHES "^#")
    continue()
  endif()
  if(line MATCHES [[this->add\("]])
    math(EXPR literal_registrations "${literal_registrations} + 1")
  endif()
  if(line MATCHES "${RE_CTOR}")
    set(ctor_class "${CMAKE_MATCH_1}")
    set(ctor_method "${CMAKE_MATCH_2}")
    if(ctor_class STREQUAL ctor_method)
      flush_entry()
      set(context "${ctor_class}")
      continue()
    endif()
  endif()
  list(FIND command_defs "${context}" in_command_defs)
  if(in_command_defs EQUAL -1)
    if(line MATCHES [[this->add\("]])
      math(EXPR out_of_scope_registrations "${out_of_scope_registrations} + 1")
    endif()
    continue()
  endif()
  # The match groups are read out at once: a second regex replaces
  # CMAKE_MATCH_<n>, and an OR evaluates both sides.
  set(matched_key "")
  set(matched_type "")
  set(matched_prefix "")
  if(line MATCHES "${RE_ADD}")
    set(matched_key "${CMAKE_MATCH_1}")
    set(matched_type "${CMAKE_MATCH_2}")
  elseif(line MATCHES "${RE_NEWDEF}")
    set(matched_key "${CMAKE_MATCH_1}")
    set(matched_type "${CMAKE_MATCH_2}")
  elseif(line MATCHES "${RE_COMPUTED}")
    set(matched_prefix "${CMAKE_MATCH_1}")
    set(matched_type "${CMAKE_MATCH_2}")
  endif()
  if(NOT matched_key STREQUAL "")
    flush_entry()
    set(key "${matched_key}")
    set(type "${matched_type}")
    math(EXPR parsed_literal_adds "${parsed_literal_adds} + 1")
  elseif(NOT matched_prefix STREQUAL "")
    flush_entry()
    math(EXPR parsed_literal_adds "${parsed_literal_adds} + 1")
    if(literals STREQUAL "")
      message(FATAL_ERROR "a name built in a loop, with no list of literals above it:\n  ${trimmed}")
    endif()
    if(matched_type MATCHES "^coBools?$")
      set(takes_value "false")
    else()
      set(takes_value "true")
    endif()
    foreach(literal IN LISTS literals)
      set(built "${matched_prefix}${literal}")
      string(REPLACE "_" "-" dashed "${built}")
      list(APPEND entries "    { \"${built}\", \"${dashed}\", ${takes_value} },")
    endforeach()
  elseif(line MATCHES "${RE_ENTRY}")
    list(APPEND run "${CMAKE_MATCH_1}")
  else()
    if(NOT run STREQUAL "")
      set(literals "${run}")
      set(run "")
    endif()
    if(line MATCHES "${RE_OTHERADD}")
      if(NOT line MATCHES "${RE_VARIABLE}")
        message(FATAL_ERROR "an option registration this scan does not read:\n  ${trimmed}")
      endif()
      flush_entry()
      # The text is reported as one line: a C++ statement holds ';', which a
      # CMake list would split.
      string(REPLACE ";" "" over_text "${trimmed}")
      list(APPEND over_sets "${over_text}")
      continue()
    endif()
    if(key STREQUAL "")
      continue()
    endif()
    if(line MATCHES "${RE_NOCLI}")
      set(nocli TRUE)
    elseif(line MATCHES "${RE_CLI}")
      set(names "${CMAKE_MATCH_1}")
    endif()
  endif()
endforeach()
flush_entry()

math(EXPR in_scope_registrations "${literal_registrations} - ${out_of_scope_registrations}")
if(NOT in_scope_registrations EQUAL parsed_literal_adds)
  message(FATAL_ERROR
    "${ENGINE_SRC}: ${in_scope_registrations} option registrations with a literal name in the config defs the command line reads, ${parsed_literal_adds} read")
endif()
list(LENGTH entries entry_count)
list(LENGTH over_sets over_set_count)
if(over_set_count GREATER 0)
  message(STATUS "engine_cli_flags: ${APP_NAME}: ${over_set_count} registrations over a set of keys are not in the table (their names are in the set, their type is read from another option):")
  foreach(over_line IN LISTS over_sets)
    message(STATUS "  ${over_line}")
  endforeach()
endif()

file(WRITE "${OUT}" "// engine_cli_flags.hpp — the command-line flags of the engine this build does
// not compile, generated at configure time by scripts/engine_cli_flags.cmake
// from that engine's own option registrations (${APP_NAME}'s
// src/libslic3r/PrintConfig.cpp). Do not edit: an engine bump rewrites it.
//
// The walk over a refused command line reads a flag's value the way the flag's
// own engine does, and a flag of the other engine is in none of this binary's
// tables: its name and whether it takes a value come from here. The rule is
// the other engine's own read_cli: one value for every flag except a coBool or
// coBools one (Config.cpp 1660-1782 at BambuStudio 5873b5f, 1591-1713 at
// OrcaSlicer 31f6803); a nocli option is no flag (ConfigOptionDef::cli_args,
// Config.cpp 242-259 at 5873b5f, 238-255 at 31f6803).
#pragma once

namespace slicer_cli {

/// One command-line flag of the other engine: the option's key (for the flags
/// that read a file, file_options), its name without dashes, and whether the
/// engine's read_cli reads a value for it.
struct OtherEngineFlag {
    const char* key;
    const char* name;
    bool        takes_value;
};

/// ${entry_count} flags of the ${APP_NAME} engine.
inline constexpr OtherEngineFlag kOtherEngineFlags[] = {
")
foreach(entry IN LISTS entries)
  file(APPEND "${OUT}" "${entry}\n")
endforeach()
file(APPEND "${OUT}" "};

} // namespace slicer_cli
")
