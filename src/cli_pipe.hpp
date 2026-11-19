// cli_pipe.hpp — --pipe NAME: the official progress pipe.
//
// Port of cli_callback_mgr_t (BambuStudio.cpp 244-437 at 5873b5f;
// OrcaSlicer.cpp 220-410 at 31f6803). Linux only, as in both official
// command lines (#if __linux__): the run opens the named pipe (a FIFO the
// caller made and reads) and writes one JSON line per progress step:
//   {"plate_index":N,"plate_count":N,"plate_percent":P,"total_percent":T,
//    "message":"..."}   ("warning" in place of "message" for a slicing warning)
// A pipe no reader opens within 1 s (50 tries, 20 ms apart) is left unused,
// and the run goes on, as the official does.
#pragma once

#include <string>

namespace slicer_cli {

/// True on the systems where the official command lines have --pipe.
bool pipe_supported();

/// cli_callback_mgr_t::start: open the pipe and start the writer.
bool pipe_start(const std::string& name);

/// cli_callback_mgr_t::is_started.
bool pipe_started();

/// cli_callback_mgr_t::set_plate_info: the plate now sliced (1-based; 0 after
/// the plates) and how many this run slices.
void pipe_set_plate_info(int index, int count);

/// cli_status_callback / cli_callback_mgr_t::update: one progress step, or a
/// warning when warning_step is not -1.
void pipe_update(int percent, const std::string& message, int warning_step = -1);

/// cli_callback_mgr_t::stop: write what is queued, stop the writer, close the pipe.
void pipe_stop();

} // namespace slicer_cli
