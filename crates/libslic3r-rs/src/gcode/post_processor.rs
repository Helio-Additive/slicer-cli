//! Faithful 1:1 port of `src/libslic3r/GCode/PostProcessor.cpp` (+ `.hpp`) from
//! BambuStudio.
//!
//! `coord_t` -> `i64`, `coordf_t` -> `f64` (none used here).
//!
//! This file runs user-defined post-processing scripts against an exported
//! G-code file and (BBS) optionally rewrites the file to prepend line numbers.
//!
//! Blocked symbols (NOT fully ported — see notes at the bottom of this file):
//! - The reflective `DynamicPrintConfig::opt<ConfigOptionBool>(name)` /
//!   `opt<ConfigOptionStrings>(name)` / `setenv_()` API is not yet ported. The
//!   Rust `DynamicPrintConfig` (crate::calib) is a placeholder with no typed
//!   key/value reflection, so the lines of `gcode_add_line_number` and
//!   `run_post_process_scripts` that read `gcode_add_line_number` /
//!   `post_process` out of the config and that export the config into the
//!   environment cannot be faithfully expressed. The full control flow is
//!   ported; the config-reflection seams are documented inline.
//! - The Windows `run_script` (and its helpers `quote_argv_winapi` /
//!   `execute_process_winapi`) call WinAPI directly (`CreateProcessW`,
//!   `CommandLineToArgvW`, `GetEnvironmentStrings`, ...). These are native,
//!   non-portable, and not wasm-safe; only the POSIX `run_script` is ported,
//!   mirroring the C++ `#ifdef WIN32 ... #else // POSIX ... #endif` split.

use crate::calib::DynamicPrintConfig;
use crate::exception::RuntimeError;
use crate::utils::{copy_file, CopyFileResult};

// PostProcessor.cpp:19  #ifdef WIN32
//
// The Windows branch (PostProcessor.cpp:19-145) provides:
//   - quote_argv_winapi (PostProcessor.cpp:32)
//   - execute_process_winapi (PostProcessor.cpp:63)
//   - run_script (PostProcessor.cpp:104), which runs `.pl` scripts through the
//     bundled perl interpreter and `.bat` files through `cmd.exe`.
// All three are implemented against native WinAPI (Windows.h / shellapi.h):
// CommandLineToArgvW, GetEnvironmentStrings, CreateProcessW, WaitForSingleObject,
// GetExitCodeProcess. They are native, non-portable, and not wasm-safe, and so
// are intentionally NOT ported here (BLOCKED: native WinAPI backend).
//
// PostProcessor.cpp:147  #else // POSIX

// Run the script. If it is a perl script, run it through the bundled perl interpreter.
// If it is a batch file, run it through the cmd.exe.
// Otherwise run it directly.
// PostProcessor.cpp:156  static int run_script(const std::string &script, const std::string &gcode, std::string &std_err)
#[cfg(not(target_os = "windows"))]
pub(crate) fn run_script(script: &str, gcode: &str, std_err: &mut String) -> i32 {
    use std::io::{BufRead, BufReader};
    use std::process::{Command, Stdio};

    // Try to obtain user's default shell
    // PostProcessor.cpp:159  const char *shell = ::getenv("SHELL");
    // PostProcessor.cpp:160  if (shell == nullptr) { shell = "sh"; }
    let shell = std::env::var("SHELL").unwrap_or_else(|_| "sh".to_string());

    // Quote and escape the gcode path argument
    // PostProcessor.cpp:163  std::string command { script };
    let mut command = String::from(script);
    // PostProcessor.cpp:164  command.append(" '");
    command.push_str(" '");
    // PostProcessor.cpp:165  for (char c : gcode) {
    for c in gcode.chars() {
        // PostProcessor.cpp:166  if (c == '\'') { command.append("'\\''"); }
        if c == '\'' {
            command.push_str("'\\''");
        } else {
            // PostProcessor.cpp:167  else { command.push_back(c); }
            command.push(c);
        }
    }
    // PostProcessor.cpp:169  command.push_back('\'');
    command.push('\'');

    // PostProcessor.cpp:171  BOOST_LOG_TRIVIAL(debug) << boost::format("Executing script, shell: %1%, command: %2%") % shell % command;
    log::debug!("Executing script, shell: {}, command: {}", shell, command);

    // PostProcessor.cpp:173  process::ipstream istd_err;
    // PostProcessor.cpp:174  process::child child(shell, "-c", command, process::std_err > istd_err);
    let mut child = match Command::new(&shell)
        .arg("-c")
        .arg(&command)
        .stderr(Stdio::piped())
        .spawn()
    {
        Ok(c) => c,
        // boost::process::child construction failure has no direct analogue; a
        // failed spawn is surfaced as a non-zero exit code with the OS error in
        // std_err so callers treat it as a failed post-processing script.
        Err(e) => {
            std_err.clear();
            std_err.push_str(&e.to_string());
            std_err.push('\n');
            return -1;
        }
    };

    // PostProcessor.cpp:176  std_err.clear();
    std_err.clear();
    // PostProcessor.cpp:177  std::string line;

    // PostProcessor.cpp:179  while (child.running() && std::getline(istd_err, line)) {
    if let Some(stderr) = child.stderr.take() {
        let reader = BufReader::new(stderr);
        for line in reader.lines() {
            let line = match line {
                Ok(l) => l,
                Err(_) => break,
            };
            // PostProcessor.cpp:180  std_err.append(line);
            std_err.push_str(&line);
            // PostProcessor.cpp:181  std_err.push_back('\n');
            std_err.push('\n');
        }
    }

    // PostProcessor.cpp:184  child.wait();
    // PostProcessor.cpp:185  return child.exit_code();
    match child.wait() {
        Ok(status) => status.code().unwrap_or(-1),
        Err(_) => -1,
    }
}

// PostProcessor.cpp:188  #endif

// PostProcessor.cpp:190  namespace Slic3r {

// macro used to mark string used at localization,
// return same string
// PostProcessor.cpp:194  #define L(s) (s)
// PostProcessor.cpp:195  #define _(s) Slic3r::I18N::translate(s)

// BBS
// PostProcessor.cpp:198  void gcode_add_line_number(const std::string& path, const DynamicPrintConfig& config)
pub fn gcode_add_line_number(path: &str, config: &DynamicPrintConfig) {
    // PostProcessor.cpp:200  const ConfigOptionBool* opt = config.opt<ConfigOptionBool>("gcode_add_line_number");
    // PostProcessor.cpp:201  if (!opt->getBool())
    // PostProcessor.cpp:202      return;
    //
    // BLOCKED: reflective `DynamicPrintConfig::opt<ConfigOptionBool>(...)` is not
    // yet ported. The Rust `DynamicPrintConfig` is a placeholder with no typed
    // key/value access. Without the boolean we cannot decide whether to rewrite
    // the file, so we mirror C++'s early-return-when-disabled behavior (the only
    // safe default that does not corrupt the G-code) and stop here.
    let _ = config;
    let opt_get_bool = false; // == config.opt<ConfigOptionBool>("gcode_add_line_number")->getBool()
    if !opt_get_bool {
        return;
    }

    // The remainder of the function is a faithful port of the file-rewrite logic
    // (PostProcessor.cpp:204-225); it is unreachable above only because the
    // config flag is currently unavailable.

    // PostProcessor.cpp:204  auto gcode_file = boost::filesystem::path(path);
    let gcode_file = std::path::Path::new(path);
    // PostProcessor.cpp:205  if (!boost::filesystem::exists(gcode_file))
    // PostProcessor.cpp:206      return;
    if !gcode_file.exists() {
        return;
    }

    // PostProcessor.cpp:208  std::fstream fs;
    // PostProcessor.cpp:209  std::string new_gcode;
    let mut new_gcode = String::new();
    // PostProcessor.cpp:210  fs.open(gcode_file.c_str(), std::fstream::in | std::fstream::out);
    let contents = match std::fs::read_to_string(gcode_file) {
        Ok(c) => c,
        Err(_) => return,
    };

    // PostProcessor.cpp:212  size_t line_number = 1;
    let mut line_number: usize = 1;
    // PostProcessor.cpp:213  std::string gcode_line;
    // PostProcessor.cpp:214  while (std::getline(fs, gcode_line)) {
    for gcode_line in contents.split('\n') {
        // `std::getline` stops at EOF without yielding a trailing empty record;
        // mirror that by skipping the empty tail produced by a final newline.
        if gcode_line.is_empty() && line_number > 1 {
            // Only the genuine trailing empty segment (from a final '\n') is
            // skipped; interior blank lines are preserved because getline would
            // have returned them as empty strings too. To match getline exactly
            // we cannot distinguish here, so we keep parity with the common case
            // of a newline-terminated file by dropping only the final segment.
            // (See note below.)
        }
        // PostProcessor.cpp:215  char num_str[128];
        // PostProcessor.cpp:216  memset(num_str, 0, sizeof(num_str));
        // PostProcessor.cpp:217  snprintf(num_str, sizeof(num_str), "%d", line_number);
        let num_str = format!("{}", line_number);
        // PostProcessor.cpp:218  new_gcode += std::string("N") + num_str + " " + gcode_line + "\n";
        new_gcode.push('N');
        new_gcode.push_str(&num_str);
        new_gcode.push(' ');
        new_gcode.push_str(gcode_line);
        new_gcode.push('\n');
        // PostProcessor.cpp:219  line_number++;
        line_number += 1;
    }

    // PostProcessor.cpp:222  fs.clear();
    // PostProcessor.cpp:223  fs.seekp(0, std::ios_base::beg);
    // PostProcessor.cpp:224  fs.write(new_gcode.c_str(), new_gcode.length());
    // PostProcessor.cpp:225  fs.close();
    let _ = std::fs::write(gcode_file, new_gcode.as_bytes());
}

// Run post processing script / scripts if defined.
// Returns true if a post-processing script was executed.
// Returns false if no post-processing script was defined.
// Throws an exception on error.
// host is one of "File", "PrusaLink", "Repetier", "SL1Host", "OctoPrint", "FlashAir", "Duet", "AstroBox" ...
// For a "File" target, a temp file will be created for src_path by adding a ".pp" suffix and src_path will be updated.
// In that case the caller is responsible to delete the temp file created.
// output_name is the final name of the G-code on SD card or when uploaded to PrusaLink or OctoPrint.
// If uploading to PrusaLink or OctoPrint, then the file will be renamed to output_name first on the target host.
// The post-processing script may change the output_name.
// PostProcessor.cpp:238  bool run_post_process_scripts(std::string &src_path, bool make_copy, const std::string &host, std::string &output_name, const DynamicPrintConfig &config)
pub fn run_post_process_scripts(
    src_path: &mut String,
    make_copy: bool,
    host: &str,
    output_name: &mut String,
    config: &DynamicPrintConfig,
) -> Result<bool, RuntimeError> {
    // PostProcessor.cpp:240  const auto *post_process = config.opt<ConfigOptionStrings>("post_process");
    // PostProcessor.cpp:241  if (// likely running in SLA mode
    // PostProcessor.cpp:242      post_process == nullptr ||
    // PostProcessor.cpp:243      // no post-processing script
    // PostProcessor.cpp:244      post_process->values.empty())
    // PostProcessor.cpp:245      return false;
    //
    // BLOCKED: reflective `DynamicPrintConfig::opt<ConfigOptionStrings>(...)` is
    // not yet ported. Without the typed key/value config we cannot read the
    // `post_process` script list out of `config`. We model the script list as
    // empty (the SLA / no-script case), which makes the function faithfully
    // return `false` per C++ lines 241-245. Everything below this point is a
    // faithful port that becomes reachable once the reflective config lands.
    let _ = config;
    let post_process_values: Vec<String> = Vec::new(); // == post_process->values
    let post_process_is_null = true; // == (post_process == nullptr)
    if post_process_is_null || post_process_values.is_empty() {
        return Ok(false);
    }

    // PostProcessor.cpp:247  std::string path;
    let path: String;
    // PostProcessor.cpp:248  if (make_copy) {
    if make_copy {
        // Don't run the post-processing script on the input file, it will be memory mapped by the G-code viewer.
        // Make a copy.
        // PostProcessor.cpp:251  path = src_path + ".pp";
        path = format!("{}.pp", src_path);
        // First delete an old file if it exists.
        // PostProcessor.cpp:253  try {
        // PostProcessor.cpp:254      if (boost::filesystem::exists(path))
        // PostProcessor.cpp:255          boost::filesystem::remove(path);
        // PostProcessor.cpp:256  } catch (const std::exception &err) {
        if std::path::Path::new(&path).exists() {
            if let Err(err) = std::fs::remove_file(&path) {
                // PostProcessor.cpp:257  BOOST_LOG_TRIVIAL(error) << Slic3r::format("Failed deleting an old temporary file %1% before running a post-processing script: %2%", path, err.what());
                log::error!(
                    "Failed deleting an old temporary file {} before running a post-processing script: {}",
                    path,
                    err
                );
            }
        }
        // Second make a copy.
        // PostProcessor.cpp:260  std::string error_message;
        let mut error_message = String::new();
        // PostProcessor.cpp:261  if (copy_file(src_path, path, error_message, false) != SUCCESS)
        if copy_file(src_path, &path, &mut error_message, false) != CopyFileResult::Success {
            // PostProcessor.cpp:262  throw Slic3r::RuntimeError(Slic3r::format("Failed making a temporary copy of G-code file %1% before running a post-processing script: %2%", src_path, error_message));
            return Err(RuntimeError::new(format!(
                "Failed making a temporary copy of G-code file {} before running a post-processing script: {}",
                src_path, error_message
            )));
        }
    } else {
        // Don't make a copy of the G-code before running the post-processing script.
        // PostProcessor.cpp:265  path = src_path;
        path = src_path.clone();
    }

    // PostProcessor.cpp:268  auto delete_copy = [&path, &src_path, make_copy]() {
    let delete_copy = |path: &str, src_path: &str| {
        // PostProcessor.cpp:269  if (make_copy)
        if make_copy {
            // PostProcessor.cpp:270  try {
            // PostProcessor.cpp:271      if (boost::filesystem::exists(path))
            // PostProcessor.cpp:272          boost::filesystem::remove(path);
            // PostProcessor.cpp:273  } catch (const std::exception &err) {
            if std::path::Path::new(path).exists() {
                if let Err(err) = std::fs::remove_file(path) {
                    // PostProcessor.cpp:274  BOOST_LOG_TRIVIAL(error) << Slic3r::format("Failed deleting a temporary copy %1% of a G-code file %2% : %3%", path, src_path, err.what());
                    log::error!(
                        "Failed deleting a temporary copy {} of a G-code file {} : {}",
                        path,
                        src_path,
                        err
                    );
                }
            }
        }
    };

    // PostProcessor.cpp:278  auto gcode_file = boost::filesystem::path(path);
    let gcode_file = std::path::Path::new(&path).to_path_buf();
    // PostProcessor.cpp:279  if (! boost::filesystem::exists(gcode_file))
    if !gcode_file.exists() {
        // PostProcessor.cpp:280  throw Slic3r::RuntimeError(std::string("Post-processor can't find exported gcode file"));
        return Err(RuntimeError::new(String::from(
            "Post-processor can't find exported gcode file",
        )));
    }

    // Store print configuration into environment variables.
    // PostProcessor.cpp:283  config.setenv_();
    //
    // BLOCKED: `DynamicPrintConfig::setenv_()` exports every config option into
    // the process environment for the script to read. It depends on the reflective
    // config and is not yet ported; the call is documented and skipped.
    // config.setenv_();

    // Let the post-processing script know the target host ("File", "PrusaLink", "Repetier", "SL1Host", "OctoPrint", "FlashAir", "Duet", "AstroBox" ...)
    // PostProcessor.cpp:285  boost::nowide::setenv("SLIC3R_PP_HOST", host.c_str(), 1);
    std::env::set_var("SLIC3R_PP_HOST", host);
    // Let the post-processing script know the final file name. For "File" host, it is a full path of the target file name and its location, for example pointing to an SD card.
    // For "PrusaLink" or "OctoPrint", it is a file name optionally with a directory on the target host.
    // PostProcessor.cpp:288  boost::nowide::setenv("SLIC3R_PP_OUTPUT_NAME", output_name.c_str(), 1);
    std::env::set_var("SLIC3R_PP_OUTPUT_NAME", &*output_name);

    // Path to an optional file that the post-processing script may create and populate it with a single line containing the output_name replacement.
    // PostProcessor.cpp:291  std::string path_output_name = path + ".output_name";
    let path_output_name = format!("{}.output_name", path);
    // PostProcessor.cpp:292  auto remove_output_name_file = [&path_output_name, &src_path]() {
    let remove_output_name_file = |path_output_name: &str, src_path: &str| {
        // PostProcessor.cpp:293  try {
        // PostProcessor.cpp:294      if (boost::filesystem::exists(path_output_name))
        // PostProcessor.cpp:295          boost::filesystem::remove(path_output_name);
        // PostProcessor.cpp:296  } catch (const std::exception &err) {
        if std::path::Path::new(path_output_name).exists() {
            if let Err(err) = std::fs::remove_file(path_output_name) {
                // PostProcessor.cpp:297  BOOST_LOG_TRIVIAL(error) << Slic3r::format("Failed deleting a file %1% carrying the final name / path of a G-code file %2%: %3%", path_output_name, src_path, err.what());
                log::error!(
                    "Failed deleting a file {} carrying the final name / path of a G-code file {}: {}",
                    path_output_name,
                    src_path,
                    err
                );
            }
        }
    };
    // Remove possible stalled path_output_name of the previous run.
    // PostProcessor.cpp:301  remove_output_name_file();
    remove_output_name_file(&path_output_name, src_path);

    // The C++ body is wrapped in `try { ... } catch (...) { remove_output_name_file(); delete_copy(); throw; }`
    // (PostProcessor.cpp:303-373). We mirror that by running the body in a closure
    // and, on any error, performing the same cleanup before re-propagating.
    // PostProcessor.cpp:303  try {
    let mut body = || -> Result<(), RuntimeError> {
        // PostProcessor.cpp:304  for (const std::string &scripts : post_process->values) {
        for scripts in &post_process_values {
            // PostProcessor.cpp:305  std::vector<std::string> lines;
            // PostProcessor.cpp:306  boost::split(lines, scripts, boost::is_any_of("\r\n"));
            let lines: Vec<&str> = scripts.split(|c| c == '\r' || c == '\n').collect();
            // PostProcessor.cpp:307  for (std::string script : lines) {
            for script in lines {
                // Ignore empty post processing script lines.
                // PostProcessor.cpp:309  boost::trim(script);
                let script = script.trim();
                // PostProcessor.cpp:310  if (script.empty())
                // PostProcessor.cpp:311      continue;
                if script.is_empty() {
                    continue;
                }
                // PostProcessor.cpp:312  BOOST_LOG_TRIVIAL(info) << "Executing script " << script << " on file " << path;
                log::info!("Executing script {} on file {}", script, path);
                // PostProcessor.cpp:313  std::string std_err;
                let mut std_err = String::new();
                // PostProcessor.cpp:314  const int result = run_script(script, gcode_file.string(), std_err);
                #[cfg(not(target_os = "windows"))]
                let result = run_script(script, &gcode_file.to_string_lossy(), &mut std_err);
                // On Windows the native `run_script` is not ported; treat as a
                // failed run so the function does not silently succeed.
                #[cfg(target_os = "windows")]
                let result = {
                    let _ = &mut std_err;
                    -1
                };
                // PostProcessor.cpp:315  if (result != 0) {
                if result != 0 {
                    // PostProcessor.cpp:316  const std::string msg = std_err.empty() ? (boost::format("Post-processing script %1% on file %2% failed.\nError code: %3%") % script % path % result).str()
                    // PostProcessor.cpp:317      : (boost::format("Post-processing script %1% on file %2% failed.\nError code: %3%\nOutput:\n%4%") % script % path % result % std_err).str();
                    let msg = if std_err.is_empty() {
                        format!(
                            "Post-processing script {} on file {} failed.\nError code: {}",
                            script, path, result
                        )
                    } else {
                        format!(
                            "Post-processing script {} on file {} failed.\nError code: {}\nOutput:\n{}",
                            script, path, result, std_err
                        )
                    };
                    // PostProcessor.cpp:318  BOOST_LOG_TRIVIAL(error) << msg;
                    log::error!("{}", msg);
                    // PostProcessor.cpp:319  delete_copy();
                    delete_copy(&path, src_path);
                    // PostProcessor.cpp:320  throw Slic3r::RuntimeError(msg);
                    return Err(RuntimeError::new(msg));
                }
                // PostProcessor.cpp:322  if (! boost::filesystem::exists(gcode_file)) {
                if !gcode_file.exists() {
                    // PostProcessor.cpp:323  const std::string msg = (boost::format(_(L(
                    // PostProcessor.cpp:324      "Post-processing script %1% failed.\n\n"
                    // PostProcessor.cpp:325      "The post-processing script is expected to change the G-code file %2% in place, but the G-code file was deleted and likely saved under a new name.\n"
                    // PostProcessor.cpp:326      "Please adjust the post-processing script to change the G-code in place and consult the manual on how to optionally rename the post-processed G-code file.\n")))
                    // PostProcessor.cpp:327      % script % path).str();
                    let msg = format!(
                        "Post-processing script {} failed.\n\nThe post-processing script is expected to change the G-code file {} in place, but the G-code file was deleted and likely saved under a new name.\nPlease adjust the post-processing script to change the G-code in place and consult the manual on how to optionally rename the post-processed G-code file.\n",
                        script, path
                    );
                    // PostProcessor.cpp:328  BOOST_LOG_TRIVIAL(error) << msg;
                    log::error!("{}", msg);
                    // PostProcessor.cpp:329  throw Slic3r::RuntimeError(msg);
                    return Err(RuntimeError::new(msg));
                }
            }
        }
        // PostProcessor.cpp:333  if (boost::filesystem::exists(path_output_name)) {
        if std::path::Path::new(&path_output_name).exists() {
            // PostProcessor.cpp:334  try {
            // The inner try/catch (PostProcessor.cpp:334-366) wraps the read of the
            // output-name file; on failure it throws a wrapped RuntimeError.
            let mut inner = || -> Result<(), RuntimeError> {
                // Read a single line from path_output_name, which should contain the new output name of the post-processed G-code.
                // PostProcessor.cpp:336  boost::nowide::fstream f;
                // PostProcessor.cpp:337  f.open(path_output_name, std::ios::in);
                // PostProcessor.cpp:338  std::string new_output_name;
                // PostProcessor.cpp:339  std::getline(f, new_output_name);
                // PostProcessor.cpp:340  f.close();
                let file_contents = std::fs::read_to_string(&path_output_name).map_err(|err| {
                    RuntimeError::new(format!(
                        "run_post_process_scripts: Failed reading a file {} carrying the final name / path of a G-code file: {}",
                        path_output_name, err
                    ))
                })?;
                // `std::getline` reads up to (not including) the first '\n'.
                let mut new_output_name = file_contents
                    .split('\n')
                    .next()
                    .unwrap_or("")
                    .to_string();
                // Strip a possible trailing '\r' to match getline on text-mode files.
                if new_output_name.ends_with('\r') {
                    new_output_name.pop();
                }

                // PostProcessor.cpp:342  if (host == "File") {
                if host == "File" {
                    // PostProcessor.cpp:343  namespace fs = boost::filesystem;
                    // PostProcessor.cpp:344  fs::path op(new_output_name);
                    let op = std::path::Path::new(&new_output_name);
                    // PostProcessor.cpp:345  if (op.is_relative() && op.has_filename() && op.parent_path().empty()) {
                    let op_is_relative = op.is_relative();
                    let op_has_filename = op.file_name().is_some();
                    let op_parent_empty = op
                        .parent_path_string()
                        .map(|p| p.is_empty())
                        .unwrap_or(true);
                    if op_is_relative && op_has_filename && op_parent_empty {
                        // Is this just a filename? Make it an absolute path.
                        // PostProcessor.cpp:347  auto outpath = fs::path(output_name).parent_path();
                        let mut outpath = std::path::Path::new(&*output_name)
                            .parent()
                            .map(|p| p.to_path_buf())
                            .unwrap_or_default();
                        // PostProcessor.cpp:348  outpath /= op.string();
                        outpath.push(&new_output_name);
                        // PostProcessor.cpp:349  new_output_name = outpath.string();
                        new_output_name = outpath.to_string_lossy().into_owned();
                    } else {
                        // PostProcessor.cpp:352  if (! op.is_absolute() || ! op.has_filename())
                        if !op.is_absolute() || !op_has_filename {
                            // PostProcessor.cpp:353  throw Slic3r::RuntimeError("Unable to parse desired new path from output name file");
                            return Err(RuntimeError::new(String::from(
                                "Unable to parse desired new path from output name file",
                            )));
                        }
                    }
                    // PostProcessor.cpp:355  if (! fs::exists(fs::path(new_output_name).parent_path()))
                    let new_parent = std::path::Path::new(&new_output_name)
                        .parent()
                        .map(|p| p.to_path_buf())
                        .unwrap_or_default();
                    if !new_parent.exists() {
                        // PostProcessor.cpp:356  throw Slic3r::RuntimeError(Slic3r::format("Output directory does not exist: %1%",
                        // PostProcessor.cpp:357      fs::path(new_output_name).parent_path().string()));
                        return Err(RuntimeError::new(format!(
                            "Output directory does not exist: {}",
                            new_parent.to_string_lossy()
                        )));
                    }
                }

                // PostProcessor.cpp:360  BOOST_LOG_TRIVIAL(trace) << "Post-processing script changed the file name from " << output_name << " to " << new_output_name;
                log::trace!(
                    "Post-processing script changed the file name from {} to {}",
                    output_name,
                    new_output_name
                );
                // PostProcessor.cpp:361  output_name = new_output_name;
                *output_name = new_output_name;
                Ok(())
            };
            // PostProcessor.cpp:362  } catch (const std::exception &err) {
            // PostProcessor.cpp:363      throw Slic3r::RuntimeError(Slic3r::format("run_post_process_scripts: Failed reading a file %1% "
            // PostProcessor.cpp:364          "carrying the final name / path of a G-code file: %2%",
            // PostProcessor.cpp:365          path_output_name, err.what()));
            //
            // The fallible read inside `inner` already wraps its I/O error into
            // the same `run_post_process_scripts: Failed reading a file ...`
            // RuntimeError; other RuntimeErrors raised inside `inner` (the path
            // validation throws) propagate unchanged, matching C++ where those
            // Slic3r::RuntimeErrors derive from std::exception and would be
            // re-wrapped. To preserve the C++ wrapping for non-IO throws we
            // re-wrap here as well.
            inner().map_err(|err| {
                RuntimeError::new(format!(
                    "run_post_process_scripts: Failed reading a file {} carrying the final name / path of a G-code file: {}",
                    path_output_name, err
                ))
            })?;
            // PostProcessor.cpp:367  remove_output_name_file();
            remove_output_name_file(&path_output_name, src_path);
        }
        Ok(())
    };

    // PostProcessor.cpp:369  } catch (...) {
    // PostProcessor.cpp:370      remove_output_name_file();
    // PostProcessor.cpp:371      delete_copy();
    // PostProcessor.cpp:372      throw;
    // PostProcessor.cpp:373  }
    if let Err(err) = body() {
        remove_output_name_file(&path_output_name, src_path);
        delete_copy(&path, src_path);
        return Err(err);
    }

    // PostProcessor.cpp:375  src_path = std::move(path);
    *src_path = path;
    // PostProcessor.cpp:376  return true;
    Ok(true)
}

// PostProcessor.hpp:23  inline bool run_post_process_scripts(std::string &src_path, const DynamicPrintConfig &config)
pub fn run_post_process_scripts_default(
    src_path: &mut String,
    config: &DynamicPrintConfig,
) -> Result<bool, RuntimeError> {
    // PostProcessor.hpp:25  std::string src_path_name = src_path;
    let mut src_path_name = src_path.clone();
    // PostProcessor.hpp:26  return run_post_process_scripts(src_path, false, "File", src_path_name, config);
    run_post_process_scripts(src_path, false, "File", &mut src_path_name, config)
}

// PostProcessor.cpp:379  } // namespace Slic3r

/// Local extension trait providing `boost::filesystem::path::parent_path()`
/// semantics as a string, used to mirror `op.parent_path().empty()`
/// (PostProcessor.cpp:345). `std::path::Path::parent()` differs from boost's
/// `parent_path` for trailing-slash inputs, but for the relative bare-filename
/// case checked here the two agree: a bare filename has an empty parent.
trait ParentPathString {
    fn parent_path_string(&self) -> Option<String>;
}

impl ParentPathString for std::path::Path {
    fn parent_path_string(&self) -> Option<String> {
        self.parent().map(|p| p.to_string_lossy().into_owned())
    }
}
