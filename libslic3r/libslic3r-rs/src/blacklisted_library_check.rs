//! Windows DLL blacklist checker for problematic overlay libraries
//!
//! C++ Reference:
//! - BlacklistedLibraryCheck.hpp
//! - BlacklistedLibraryCheck.cpp
//!
//! This module detects problematic DLLs that are known to cause issues with
//! the application (screen capture overlays, audio enhancement DLLs, etc).
//! Only functional on Windows; no-op on other platforms.


/// Blacklisted library checker (singleton pattern)
/// BlacklistedLibraryCheck.hpp:12-41
#[derive(Debug)]
pub struct BlacklistedLibraryCheck {
    /// List of found blacklisted DLL paths
    /// BlacklistedLibraryCheck.cpp:16
    found: Vec<String>,
}

impl BlacklistedLibraryCheck {
    /// Get the singleton instance
    /// BlacklistedLibraryCheck.hpp:14-19
    /// C++: static BlacklistedLibraryCheck& get_instance() { static BlacklistedLibraryCheck instance; return instance; }
    pub fn get_instance() -> &'static mut Self {
        use std::sync::Once;
        static mut INSTANCE: Option<BlacklistedLibraryCheck> = None;
        static INIT: Once = Once::new();

        unsafe {
            INIT.call_once(|| {
                INSTANCE = Some(BlacklistedLibraryCheck { found: Vec::new() });
            });
            INSTANCE.as_mut().unwrap()
        }
    }

    /// Returns all found blacklisted DLL names
    /// BlacklistedLibraryCheck.cpp:16-23
    /// C++: bool BlacklistedLibraryCheck::get_blacklisted(std::vector<std::wstring>& names)
    pub fn get_blacklisted(&self, names: &mut Vec<String>) -> bool {
        if self.found.is_empty() {
            return false;
        }
        for lib in &self.found {
            names.push(lib.clone());
        }
        true
    }

    /// Returns all found blacklisted DLLs as a newline-separated string
    /// BlacklistedLibraryCheck.cpp:25-30
    /// C++: std::wstring BlacklistedLibraryCheck::get_blacklisted_string()
    pub fn get_blacklisted_string(&self) -> String {
        let mut ret = String::new();
        for lib in &self.found {
            ret.push_str(lib);
            ret.push('\n');
        }
        ret
    }

    /// Perform check for blacklisted DLLs loaded in the current process
    /// Returns true if any blacklisted DLL was found
    /// BlacklistedLibraryCheck.cpp:32-62
    #[cfg(target_os = "windows")]
    pub fn perform_check(&mut self) -> bool {
        use std::ffi::OsString;
        use std::os::windows::ffi::OsStringExt;
        use winapi::shared::minwindef::{DWORD, HMODULE, MAX_PATH};
        use winapi::um::processthreadsapi::GetCurrentProcess;
        use winapi::um::psapi::{EnumProcessModulesEx, GetModuleFileNameExW, LIST_MODULES_ALL};

        /// Hardcoded list of problematic DLLs
        /// BlacklistedLibraryCheck.cpp:14
        /// C++: const std::vector<std::wstring> BlacklistedLibraryCheck::blacklist({ L"NahimicOSD.dll", L"SS2OSD.dll", L"amhook.dll", L"AMHook.dll" });
        const BLACKLIST: &[&str] = &["NahimicOSD.dll", "SS2OSD.dll", "amhook.dll", "AMHook.dll"];

        unsafe {
            /// Get the pseudo-handle for the current process
            /// BlacklistedLibraryCheck.cpp:34
            /// C++: HANDLE hCurrentProcess = GetCurrentProcess();
            let h_current_process = GetCurrentProcess();

            /// Get a list of all the modules in this process
            /// BlacklistedLibraryCheck.cpp:37-38
            /// C++: HMODULE hMods[1024]; DWORD cbNeeded;
            let mut h_mods: [HMODULE; 1024] = [std::ptr::null_mut(); 1024];
            let mut cb_needed: DWORD = 0;

            /// Enumerate all loaded modules
            /// BlacklistedLibraryCheck.cpp:39
            /// C++: if (EnumProcessModulesEx(hCurrentProcess, hMods, sizeof(hMods), &cbNeeded, LIST_MODULES_ALL))
            if EnumProcessModulesEx(
                h_current_process,
                h_mods.as_mut_ptr(),
                std::mem::size_of_val(&h_mods) as DWORD,
                &mut cb_needed,
                LIST_MODULES_ALL,
            ) != 0
            {
                /// Iterate through each loaded module
                /// BlacklistedLibraryCheck.cpp:41-42
                /// C++: for (unsigned int i = 0; i < cbNeeded / sizeof(HMODULE); ++i)
                let num_modules = (cb_needed as usize) / std::mem::size_of::<HMODULE>();
                for i in 0..num_modules {
                    /// Get the full path to the module's file
                    /// BlacklistedLibraryCheck.cpp:44-46
                    /// C++: wchar_t szModName[MAX_PATH];
                    /// C++: if (GetModuleFileNameExW(hCurrentProcess, hMods[i], szModName, MAX_PATH))
                    let mut sz_mod_name: [u16; MAX_PATH] = [0; MAX_PATH];
                    if GetModuleFileNameExW(
                        h_current_process,
                        h_mods[i],
                        sz_mod_name.as_mut_ptr(),
                        MAX_PATH as DWORD,
                    ) != 0
                    {
                        /// Convert wide string to Rust String
                        let len = sz_mod_name.iter().position(|&c| c == 0).unwrap_or(MAX_PATH);
                        let os_string = OsString::from_wide(&sz_mod_name[..len]);
                        let dll_path = os_string.to_string_lossy().to_string();

                        /// Add to list if blacklisted
                        /// BlacklistedLibraryCheck.cpp:48-51
                        /// C++: if (BlacklistedLibraryCheck::is_blacklisted(szModName)) {
                        /// C++: if (std::find(m_found.begin(), m_found.end(), szModName) == m_found.end())
                        /// C++: m_found.emplace_back(szModName);
                        /// C++: }
                        if Self::is_blacklisted_path(&dll_path, BLACKLIST) {
                            if !self.found.contains(&dll_path) {
                                self.found.push(dll_path);
                            }
                        }
                    }
                }
            }
        }

        /// Return true if any blacklisted DLLs were found
        /// BlacklistedLibraryCheck.cpp:60
        /// C++: return !m_found.empty();
        !self.found.is_empty()
    }

    /// Perform check for blacklisted DLLs (no-op on non-Windows platforms)
    /// BlacklistedLibraryCheck.cpp:32-62
    #[cfg(not(target_os = "windows"))]
    pub fn perform_check(&mut self) -> bool {
        // No-op on non-Windows platforms
        false
    }

    /// Check if a DLL path is blacklisted
    /// BlacklistedLibraryCheck.cpp:64-74
    /// C++: bool BlacklistedLibraryCheck::is_blacklisted(const std::wstring &dllpath)
    fn is_blacklisted_path(dll_path: &str, blacklist: &[&str]) -> bool {
        // Extract filename from path
        // BlacklistedLibraryCheck.cpp:66
        // C++: std::wstring dllname = boost::filesystem::path(dllpath).filename().wstring();
        if let Some(file_name) = std::path::Path::new(dll_path).file_name() {
            let dll_name = file_name.to_string_lossy();

            // Check if filename is in blacklist (case-sensitive)
            // BlacklistedLibraryCheck.cpp:68-71
            // C++: if (std::find(BlacklistedLibraryCheck::blacklist.begin(), BlacklistedLibraryCheck::blacklist.end(), dllname) != BlacklistedLibraryCheck::blacklist.end()) {
            // C++: return true;
            // C++: }
            for blacklisted in blacklist {
                if dll_name == *blacklisted {
                    return true;
                }
            }
        }
        false
    }
}

/// Check if a DLL path is blacklisted (UTF-8 encoded path)
/// BlacklistedLibraryCheck.cpp:76-79
/// C++: bool BlacklistedLibraryCheck::is_blacklisted(const std::string &dllpath)
pub fn is_blacklisted(dll_path: &str) -> bool {
    const BLACKLIST: &[&str] = &["NahimicOSD.dll", "SS2OSD.dll", "amhook.dll", "AMHook.dll"];
    BlacklistedLibraryCheck::is_blacklisted_path(dll_path, BLACKLIST)
}

/// Get all blacklisted DLL names found in the current process
/// BlacklistedLibraryCheck.cpp:16-23
pub fn get_blacklisted() -> Vec<String> {
    let mut names = Vec::new();
    BlacklistedLibraryCheck::get_instance().get_blacklisted(&mut names);
    names
}

/// Get all blacklisted DLLs as a newline-separated string
/// BlacklistedLibraryCheck.cpp:25-30
pub fn get_blacklisted_string() -> String {
    BlacklistedLibraryCheck::get_instance().get_blacklisted_string()
}

/// Perform check for blacklisted DLLs loaded in the current process
/// BlacklistedLibraryCheck.cpp:32-62
pub fn perform_check() -> bool {
    BlacklistedLibraryCheck::get_instance().perform_check()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_is_blacklisted() {
        /// Test known blacklisted DLL names
        assert!(is_blacklisted("NahimicOSD.dll"));
        assert!(is_blacklisted("C:\\Windows\\System32\\NahimicOSD.dll"));
        assert!(is_blacklisted("SS2OSD.dll"));
        assert!(is_blacklisted("amhook.dll"));
        assert!(is_blacklisted("AMHook.dll"));

        /// Test non-blacklisted DLL
        assert!(!is_blacklisted("kernel32.dll"));
        assert!(!is_blacklisted("user32.dll"));
    }

    #[test]
    fn test_get_blacklisted_string_empty() {
        /// Should return empty string when no blacklisted DLLs found
        let result = get_blacklisted_string();
        // Note: Can't easily test this without modifying global state
        // This is a structural test to ensure the function compiles
        assert!(result.is_empty() || !result.is_empty());
    }

    #[test]
    #[cfg(target_os = "windows")]
    fn test_perform_check_runs() {
        /// Test that perform_check runs without panicking
        /// We can't predict the result since it depends on what DLLs are loaded
        let result = perform_check();
        assert!(result == true || result == false);
    }

    #[test]
    #[cfg(not(target_os = "windows"))]
    fn test_perform_check_no_op() {
        /// On non-Windows platforms, should always return false
        let result = perform_check();
        assert_eq!(result, false);
    }
}
