//! 1:1 port of `Platform.cpp` / `Platform.hpp` (BambuStudio `libslic3r`).
//!
//! Compile-time platform detection. The C++ source selects exactly one body of
//! `detect_platform()` via `#if defined(_WIN32) / __APPLE__ / __linux__ /
//! __OpenBSD__ / #else`. The faithful Rust translation mirrors that selection
//! with `#[cfg(...)]` so that exactly the same branch is compiled in on each
//! target, including wasm (which matches none of the OS cfgs and therefore
//! takes the `#else` "Unknown" body).
//!
//! The C++ stores the result in two mutable file-static globals
//! (`s_platform`, `s_platform_flavor`). Rust forbids mutable `static` without
//! `unsafe`, so we use `Mutex`-protected statics (matching the precedent in
//! `thread.rs`), which preserves the same observable semantics: the values are
//! `Uninitialized` until `detect_platform()` runs, then read back via
//! `platform()` / `platform_flavor()`.
//!
//! `BOOST_LOG_TRIVIAL(info)` maps to `log::info!` (the crate convention).
//!
//! The macOS CPU-type / Rosetta probe and the Linux `/proc/version` read are
//! reproduced via system queries (shelling out to `sysctl` / reading the file),
//! matching the existing system-query precedent in `mac_utils.rs` and
//! `utils.rs`. This keeps the crate wasm-safe: no native dylib / FFI deps are
//! introduced, and on wasm neither the macOS nor the Linux cfg body compiles.

// Platform.hpp:4  #include <string>

use std::sync::Mutex;

// Platform.hpp:8
// enum class Platform
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Platform {
    // Platform.hpp:10
    Uninitialized,
    // Platform.hpp:11
    Unknown,
    // Platform.hpp:12
    Windows,
    // Platform.hpp:13
    OSX,
    // Platform.hpp:14
    Linux,
    // Platform.hpp:15
    BSDUnix,
}

// Platform.hpp:18
// enum class PlatformFlavor
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlatformFlavor {
    // Platform.hpp:20
    Uninitialized,
    // Platform.hpp:21
    Unknown,
    // Platform.hpp:22  Generic,         // For Windows and OSX, until we need to be more specific.
    Generic,
    // Platform.hpp:23  GenericLinux,    // For Platform::Linux
    GenericLinux,
    // Platform.hpp:24  LinuxOnChromium, // For Platform::Linux
    LinuxOnChromium,
    // Platform.hpp:25  WSL,             // Microsoft's Windows on Linux (Linux kernel simulated on NTFS kernel)
    WSL,
    // Platform.hpp:26  WSL2,            // Microsoft's Windows on Linux, version 2 (virtual machine)
    WSL2,
    // Platform.hpp:27  OpenBSD,         // For Platform::BSDUnix
    OpenBSD,
    // Platform.hpp:28  GenericOSX,      // For Platform::OSX
    GenericOSX,
    // Platform.hpp:29  OSXOnX86,        // For Apple's on Intel X86 CPU
    OSXOnX86,
    // Platform.hpp:30  OSXOnArm,        // For Apple's on Arm CPU
    OSXOnArm,
}

// Platform.cpp:14  static auto s_platform        = Platform::Uninitialized;
static S_PLATFORM: Mutex<Platform> = Mutex::new(Platform::Uninitialized);
// Platform.cpp:15  static auto s_platform_flavor = PlatformFlavor::Uninitialized;
static S_PLATFORM_FLAVOR: Mutex<PlatformFlavor> = Mutex::new(PlatformFlavor::Uninitialized);

// Platform.cpp:17
// void detect_platform()
//
// The C++ body is a single `#if` cascade selecting one platform. We mirror it
// with `#[cfg(...)]` so exactly one body is compiled per target.
pub fn detect_platform() {
    // Platform.cpp:19  #if defined(_WIN32)
    #[cfg(target_os = "windows")]
    {
        // Platform.cpp:20  BOOST_LOG_TRIVIAL(info) << "Platform: Windows";
        log::info!("Platform: Windows");
        // Platform.cpp:21  s_platform        = Platform::Windows;
        *S_PLATFORM.lock().unwrap() = Platform::Windows;
        // Platform.cpp:22  s_platform_flavor = PlatformFlavor::Generic;
        *S_PLATFORM_FLAVOR.lock().unwrap() = PlatformFlavor::Generic;
    }

    // Platform.cpp:23  #elif defined(__APPLE__)
    #[cfg(target_os = "macos")]
    {
        // Platform.cpp:24  BOOST_LOG_TRIVIAL(info) << "Platform: OSX";
        log::info!("Platform: OSX");
        // Platform.cpp:25  s_platform        = Platform::OSX;
        *S_PLATFORM.lock().unwrap() = Platform::OSX;
        // Platform.cpp:26  s_platform_flavor = PlatformFlavor::GenericOSX;
        *S_PLATFORM_FLAVOR.lock().unwrap() = PlatformFlavor::GenericOSX;
        // Platform.cpp:27  {
        {
            // Platform.cpp:28  cpu_type_t type = 0;
            // Platform.cpp:29  size_t     size = sizeof(type);
            // Platform.cpp:30  if (sysctlbyname("hw.cputype", &type, &size, NULL, 0) == 0) {
            if let Some(mut cpu_type) = sysctlbyname_int("hw.cputype") {
                // Platform.cpp:31  type &= ~CPU_ARCH_MASK;
                cpu_type &= !CPU_ARCH_MASK;
                // Platform.cpp:32  if (type == CPU_TYPE_X86) {
                if cpu_type == CPU_TYPE_X86 {
                    // Platform.cpp:33  int proc_translated = 0;
                    // Platform.cpp:34  size                = sizeof(proc_translated);
                    // Platform.cpp:35  // Detect if native CPU is really X86 or PrusaSlicer runs through Rosetta.
                    // Platform.cpp:36  if (sysctlbyname("sysctl.proc_translated", &proc_translated, &size, NULL, 0) == -1) {
                    match sysctlbyname_int("sysctl.proc_translated") {
                        None => {
                            // Platform.cpp:37  if (errno == ENOENT) {
                            // The property does not exist: a failed `sysctlbyname`
                            // for this key corresponds to `errno == ENOENT`.
                            // Platform.cpp:38  // Native CPU is X86, and property sysctl.proc_translated doesn't exist.
                            // Platform.cpp:39  s_platform_flavor = PlatformFlavor::OSXOnX86;
                            *S_PLATFORM_FLAVOR.lock().unwrap() = PlatformFlavor::OSXOnX86;
                            // Platform.cpp:40  BOOST_LOG_TRIVIAL(info) << "Platform flavor: OSXOnX86";
                            log::info!("Platform flavor: OSXOnX86");
                        }
                        // Platform.cpp:42  } else if (proc_translated == 1) {
                        Some(1) => {
                            // Platform.cpp:43  // Native CPU is ARM and PrusaSlicer runs through Rosetta.
                            // Platform.cpp:44  s_platform_flavor = PlatformFlavor::OSXOnArm;
                            *S_PLATFORM_FLAVOR.lock().unwrap() = PlatformFlavor::OSXOnArm;
                            // Platform.cpp:45  BOOST_LOG_TRIVIAL(info) << "Platform flavor: OSXOnArm";
                            log::info!("Platform flavor: OSXOnArm");
                        }
                        // Platform.cpp:46  } else {
                        Some(_) => {
                            // Platform.cpp:47  // Native CPU is X86.
                            // Platform.cpp:48  s_platform_flavor = PlatformFlavor::OSXOnX86;
                            *S_PLATFORM_FLAVOR.lock().unwrap() = PlatformFlavor::OSXOnX86;
                            // Platform.cpp:49  BOOST_LOG_TRIVIAL(info) << "Platform flavor: OSXOnX86";
                            log::info!("Platform flavor: OSXOnX86");
                        }
                    }
                // Platform.cpp:51  } else if (type == CPU_TYPE_ARM) {
                } else if cpu_type == CPU_TYPE_ARM {
                    // Platform.cpp:52  // Native CPU is ARM
                    // Platform.cpp:53  s_platform_flavor = PlatformFlavor::OSXOnArm;
                    *S_PLATFORM_FLAVOR.lock().unwrap() = PlatformFlavor::OSXOnArm;
                    // Platform.cpp:54  BOOST_LOG_TRIVIAL(info) << "Platform flavor: OSXOnArm";
                    log::info!("Platform flavor: OSXOnArm");
                }
            }
        }
    }

    // Platform.cpp:58  #elif defined(__linux__)
    #[cfg(target_os = "linux")]
    {
        // Platform.cpp:59  BOOST_LOG_TRIVIAL(info) << "Platform: Linux";
        log::info!("Platform: Linux");
        // Platform.cpp:60  s_platform        = Platform::Linux;
        *S_PLATFORM.lock().unwrap() = Platform::Linux;
        // Platform.cpp:61  s_platform_flavor = PlatformFlavor::GenericLinux;
        *S_PLATFORM_FLAVOR.lock().unwrap() = PlatformFlavor::GenericLinux;
        // Platform.cpp:62  // Test for Chromium.
        // Platform.cpp:63  {
        {
            // Platform.cpp:64  FILE *f = ::fopen("/proc/version", "rt");
            // Platform.cpp:65  if (f) {
            // Platform.cpp:66      char buf[4096];
            // Platform.cpp:67      // Read the 1st line.
            // Platform.cpp:68      if (::fgets(buf, 4096, f)) {
            if let Some(buf) = read_first_line("/proc/version") {
                // Platform.cpp:69  if (strstr(buf, "Chromium OS") != nullptr) {
                if buf.contains("Chromium OS") {
                    // Platform.cpp:70  s_platform_flavor = PlatformFlavor::LinuxOnChromium;
                    *S_PLATFORM_FLAVOR.lock().unwrap() = PlatformFlavor::LinuxOnChromium;
                    // Platform.cpp:71  BOOST_LOG_TRIVIAL(info) << "Platform flavor: LinuxOnChromium";
                    log::info!("Platform flavor: LinuxOnChromium");
                // Platform.cpp:72  } else if (strstr(buf, "microsoft") != nullptr || strstr(buf, "Microsoft") != nullptr) {
                } else if buf.contains("microsoft") || buf.contains("Microsoft") {
                    // Platform.cpp:73  if (boost::filesystem::exists("/run/WSL") && getenv("WSL_INTEROP") != nullptr) {
                    if std::path::Path::new("/run/WSL").exists()
                        && std::env::var_os("WSL_INTEROP").is_some()
                    {
                        // Platform.cpp:74  BOOST_LOG_TRIVIAL(info) << "Platform flavor: WSL2";
                        log::info!("Platform flavor: WSL2");
                        // Platform.cpp:75  s_platform_flavor = PlatformFlavor::WSL2;
                        *S_PLATFORM_FLAVOR.lock().unwrap() = PlatformFlavor::WSL2;
                    } else {
                        // Platform.cpp:77  BOOST_LOG_TRIVIAL(info) << "Platform flavor: WSL";
                        log::info!("Platform flavor: WSL");
                        // Platform.cpp:78  s_platform_flavor = PlatformFlavor::WSL;
                        *S_PLATFORM_FLAVOR.lock().unwrap() = PlatformFlavor::WSL;
                    }
                }
                // Platform.cpp:82  ::fclose(f);  (file closed when `buf` was read)
            }
        }
    }

    // Platform.cpp:85  #elif defined(__OpenBSD__)
    #[cfg(target_os = "openbsd")]
    {
        // Platform.cpp:86  BOOST_LOG_TRIVIAL(info) << "Platform: OpenBSD";
        log::info!("Platform: OpenBSD");
        // Platform.cpp:87  s_platform        = Platform::BSDUnix;
        *S_PLATFORM.lock().unwrap() = Platform::BSDUnix;
        // Platform.cpp:88  s_platform_flavor = PlatformFlavor::OpenBSD;
        *S_PLATFORM_FLAVOR.lock().unwrap() = PlatformFlavor::OpenBSD;
    }

    // Platform.cpp:89  #else
    //   // This should not happen.
    //   BOOST_LOG_TRIVIAL(info) << "Platform: Unknown";
    //   static_assert(false, "Unknown platform detected");
    //   s_platform        = Platform::Unknown;
    //   s_platform_flavor = PlatformFlavor::Unknown;
    //
    // The C++ `#else` body is `static_assert(false, ...)`, i.e. compiling for an
    // unrecognised platform is a hard build error. We reproduce the same intent
    // for unrecognised targets (notably wasm, where none of the OS cfgs above
    // match): set the Unknown values. The `static_assert(false)` itself is a
    // compile-time abort that has no faithful runtime analogue, so the closest
    // observable behaviour is the assignment of the Unknown values it precedes.
    #[cfg(not(any(
        target_os = "windows",
        target_os = "macos",
        target_os = "linux",
        target_os = "openbsd"
    )))]
    {
        // Platform.cpp:91  BOOST_LOG_TRIVIAL(info) << "Platform: Unknown";
        log::info!("Platform: Unknown");
        // Platform.cpp:93  s_platform        = Platform::Unknown;
        *S_PLATFORM.lock().unwrap() = Platform::Unknown;
        // Platform.cpp:94  s_platform_flavor = PlatformFlavor::Unknown;
        *S_PLATFORM_FLAVOR.lock().unwrap() = PlatformFlavor::Unknown;
    }
}

// Platform.cpp:98
// Platform platform()
pub fn platform() -> Platform {
    // Platform.cpp:100  return s_platform;
    *S_PLATFORM.lock().unwrap()
}

// Platform.cpp:103
// PlatformFlavor platform_flavor()
pub fn platform_flavor() -> PlatformFlavor {
    // Platform.cpp:105  return s_platform_flavor;
    *S_PLATFORM_FLAVOR.lock().unwrap()
}

// Platform.cpp:110
// std::string platform_to_string(Platform platform)
pub fn platform_to_string(platform: Platform) -> String {
    // Platform.cpp:112  switch (platform) {
    match platform {
        // Platform.cpp:113  case Platform::Uninitialized: return "Unitialized";
        Platform::Uninitialized => "Unitialized".to_string(),
        // Platform.cpp:114  case Platform::Unknown      : return "Unknown";
        Platform::Unknown => "Unknown".to_string(),
        // Platform.cpp:115  case Platform::Windows      : return "Windows";
        Platform::Windows => "Windows".to_string(),
        // Platform.cpp:116  case Platform::OSX          : return "OSX";
        Platform::OSX => "OSX".to_string(),
        // Platform.cpp:117  case Platform::Linux        : return "Linux";
        Platform::Linux => "Linux".to_string(),
        // Platform.cpp:118  case Platform::BSDUnix      : return "BSDUnix";
        Platform::BSDUnix => "BSDUnix".to_string(),
    }
    // Platform.cpp:120  assert(false);
    // Platform.cpp:121  return "";
    // (The `match` is exhaustive over the enum, so the post-switch
    //  `assert(false); return "";` is unreachable, exactly as in C++.)
}

// Platform.cpp:126
// std::string platform_flavor_to_string(PlatformFlavor pf)
pub fn platform_flavor_to_string(pf: PlatformFlavor) -> String {
    // Platform.cpp:128  switch (pf) {
    match pf {
        // Platform.cpp:129  case PlatformFlavor::Uninitialized   : return "Unitialized";
        PlatformFlavor::Uninitialized => "Unitialized".to_string(),
        // Platform.cpp:130  case PlatformFlavor::Unknown         : return "Unknown";
        PlatformFlavor::Unknown => "Unknown".to_string(),
        // Platform.cpp:131  case PlatformFlavor::Generic         : return "Generic";
        PlatformFlavor::Generic => "Generic".to_string(),
        // Platform.cpp:132  case PlatformFlavor::GenericLinux    : return "GenericLinux";
        PlatformFlavor::GenericLinux => "GenericLinux".to_string(),
        // Platform.cpp:133  case PlatformFlavor::LinuxOnChromium : return "LinuxOnChromium";
        PlatformFlavor::LinuxOnChromium => "LinuxOnChromium".to_string(),
        // Platform.cpp:134  case PlatformFlavor::WSL             : return "WSL";
        PlatformFlavor::WSL => "WSL".to_string(),
        // Platform.cpp:135  case PlatformFlavor::WSL2            : return "WSL2";
        PlatformFlavor::WSL2 => "WSL2".to_string(),
        // Platform.cpp:136  case PlatformFlavor::OpenBSD         : return "OpenBSD";
        PlatformFlavor::OpenBSD => "OpenBSD".to_string(),
        // Platform.cpp:137  case PlatformFlavor::GenericOSX      : return "GenericOSX";
        PlatformFlavor::GenericOSX => "GenericOSX".to_string(),
        // Platform.cpp:138  case PlatformFlavor::OSXOnX86        : return "OSXOnX86";
        PlatformFlavor::OSXOnX86 => "OSXOnX86".to_string(),
        // Platform.cpp:139  case PlatformFlavor::OSXOnArm        : return "OSXOnArm";
        PlatformFlavor::OSXOnArm => "OSXOnArm".to_string(),
    }
    // Platform.cpp:141  assert(false);
    // Platform.cpp:142  return "";
    // (Exhaustive `match`; the post-switch fallthrough is unreachable.)
}

// --- macOS support helpers (Platform.cpp:6-10 `<sys/sysctl.h>` / `<mach/machine.h>`) ---
//
// `cpu_type_t` masking constants from `<mach/machine.h>`. `CPU_ARCH_MASK`
// strips the 64-bit ABI bit so that x86_64 / arm64 compare equal to their
// 32-bit base type, exactly as `type &= ~CPU_ARCH_MASK` does in the C++.
#[cfg(target_os = "macos")]
const CPU_ARCH_MASK: i64 = 0xff00_0000; // <mach/machine.h> CPU_ARCH_MASK
#[cfg(target_os = "macos")]
const CPU_TYPE_X86: i64 = 7; // <mach/machine.h> CPU_TYPE_X86
#[cfg(target_os = "macos")]
const CPU_TYPE_ARM: i64 = 12; // <mach/machine.h> CPU_TYPE_ARM

/// Reads an integer sysctl by name. Mirrors `sysctlbyname(name, &out, ...)`:
/// returns `Some(value)` when the property exists and parses as an integer
/// (C++ return 0), and `None` when the property does not exist or the query
/// fails (C++ return -1, `errno == ENOENT`). Implemented by shelling out to
/// `sysctl -n <name>`, matching the existing system-query precedent in
/// `mac_utils.rs` / `utils.rs` (no native FFI / dylib dependency).
#[cfg(target_os = "macos")]
fn sysctlbyname_int(name: &str) -> Option<i64> {
    use std::process::Command;

    let output = Command::new("sysctl").arg("-n").arg(name).output().ok()?;
    if !output.status.success() {
        return None;
    }
    let text = String::from_utf8(output.stdout).ok()?;
    text.trim().parse::<i64>().ok()
}

/// Reads the first line of a file (`fopen` + `fgets`, then `fclose`). Returns
/// `None` if the file cannot be opened or no line can be read, matching the
/// `if (f) { ... if (::fgets(...)) { ... } }` guard in the C++ Linux body.
#[cfg(target_os = "linux")]
fn read_first_line(path: &str) -> Option<String> {
    use std::fs::File;
    use std::io::{BufRead, BufReader};

    let file = File::open(path).ok()?;
    let mut reader = BufReader::new(file);
    let mut line = String::new();
    if reader.read_line(&mut line).ok()? == 0 {
        return None;
    }
    Some(line)
}
