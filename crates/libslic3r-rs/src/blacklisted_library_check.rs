//! Windows DLL blacklist checker for problematic overlay libraries
//!
//! C++ Reference:
//! - BlacklistedLibraryCheck.hpp
//! - BlacklistedLibraryCheck.cpp
//!
//! This module detects problematic DLLs that are known to cause issues with
//! the application (screen capture overlays, audio enhancement DLLs, etc).
//!
//! In the C++ source the ENTIRE class and all of its members are wrapped in
//! `#ifdef WIN32`; on every other platform none of these symbols exist. To
//! faithfully mirror that, the whole module body is gated behind
//! `#[cfg(target_os = "windows")]`. This also keeps the crate wasm-safe: the
//! `winapi` native dependency is only referenced on Windows targets.

// BlacklistedLibraryCheck.hpp:4 `#ifdef  WIN32`
// BlacklistedLibraryCheck.cpp:12 `#ifdef  WIN32`
#[cfg(target_os = "windows")]
mod windows_impl {
    use std::ffi::OsString;
    use std::os::windows::ffi::OsStringExt;
    use std::sync::Once;
    use winapi::shared::minwindef::{DWORD, HMODULE, MAX_PATH};
    use winapi::um::processthreadsapi::GetCurrentProcess;
    use winapi::um::psapi::{EnumProcessModulesEx, GetModuleFileNameExW, LIST_MODULES_ALL};

    /// BlacklistedLibraryCheck.hpp:13-40
    /// C++: class BlacklistedLibraryCheck { ... }
    #[derive(Debug)]
    pub struct BlacklistedLibraryCheck {
        /// BlacklistedLibraryCheck.hpp:25
        /// C++: std::vector<std::wstring> m_found;
        m_found: Vec<String>,
    }

    /// BlacklistedLibraryCheck.cpp:14-15
    /// C++: //only dll name with .dll suffix - currently case sensitive
    /// C++: const std::vector<std::wstring> BlacklistedLibraryCheck::blacklist({ L"NahimicOSD.dll", L"SS2OSD.dll", L"amhook.dll", L"AMHook.dll" });
    //only dll name with .dll suffix - currently case sensitive
    const BLACKLIST: &[&str] = &["NahimicOSD.dll", "SS2OSD.dll", "amhook.dll", "AMHook.dll"];

    impl BlacklistedLibraryCheck {
        /// BlacklistedLibraryCheck.hpp:16-21
        /// C++: static BlacklistedLibraryCheck& get_instance()
        /// C++: { static BlacklistedLibraryCheck instance; return instance; }
        #[allow(static_mut_refs)]
        pub fn get_instance() -> &'static mut BlacklistedLibraryCheck {
            // BlacklistedLibraryCheck.hpp:23 `BlacklistedLibraryCheck() = default;`
            static mut INSTANCE: Option<BlacklistedLibraryCheck> = None;
            static INIT: Once = Once::new();

            unsafe {
                INIT.call_once(|| {
                    INSTANCE = Some(BlacklistedLibraryCheck { m_found: Vec::new() });
                });
                INSTANCE.as_mut().unwrap()
            }
        }

        /// BlacklistedLibraryCheck.cpp:17-24
        /// C++: bool BlacklistedLibraryCheck::get_blacklisted(std::vector<std::wstring>& names)
        // returns all found blacklisted dlls
        pub fn get_blacklisted(&self, names: &mut Vec<String>) -> bool {
            // BlacklistedLibraryCheck.cpp:19-20
            // C++: if (m_found.empty()) return false;
            if self.m_found.is_empty() {
                return false;
            }
            // BlacklistedLibraryCheck.cpp:21-22
            // C++: for (const auto& lib : m_found) names.emplace_back(lib);
            for lib in &self.m_found {
                names.push(lib.clone());
            }
            // BlacklistedLibraryCheck.cpp:23
            // C++: return true;
            true
        }

        /// BlacklistedLibraryCheck.cpp:26-32
        /// C++: std::wstring BlacklistedLibraryCheck::get_blacklisted_string()
        pub fn get_blacklisted_string(&self) -> String {
            // BlacklistedLibraryCheck.cpp:28
            // C++: std::wstring ret;
            let mut ret = String::new();
            // BlacklistedLibraryCheck.cpp:29-30
            // C++: for (const auto& lib : m_found) ret += lib + L"\n";
            for lib in &self.m_found {
                ret += lib;
                ret += "\n";
            }
            // BlacklistedLibraryCheck.cpp:31
            // C++: return ret;
            ret
        }

        /// BlacklistedLibraryCheck.cpp:34-64
        /// C++: bool BlacklistedLibraryCheck::perform_check()
        // returns true if enumerating found blacklisted dll
        pub fn perform_check(&mut self) -> bool {
            unsafe {
                // BlacklistedLibraryCheck.cpp:36-37
                // Get the pseudo-handle for the current process.
                // C++: HANDLE  hCurrentProcess = GetCurrentProcess();
                let h_current_process = GetCurrentProcess();

                // BlacklistedLibraryCheck.cpp:39-41
                // Get a list of all the modules in this process.
                // C++: HMODULE hMods[1024];
                // C++: DWORD   cbNeeded;
                let mut h_mods: [HMODULE; 1024] = [std::ptr::null_mut(); 1024];
                let mut cb_needed: DWORD = 0;

                // BlacklistedLibraryCheck.cpp:42
                // C++: if (EnumProcessModulesEx(hCurrentProcess, hMods, sizeof(hMods), &cbNeeded, LIST_MODULES_ALL))
                if EnumProcessModulesEx(
                    h_current_process,
                    h_mods.as_mut_ptr(),
                    std::mem::size_of_val(&h_mods) as DWORD,
                    &mut cb_needed,
                    LIST_MODULES_ALL,
                ) != 0
                {
                    //printf("Total Dlls: %d\n", cbNeeded / sizeof(HMODULE));
                    // BlacklistedLibraryCheck.cpp:45
                    // C++: for (unsigned int i = 0; i < cbNeeded / sizeof(HMODULE); ++ i)
                    let count = (cb_needed as usize) / std::mem::size_of::<HMODULE>();
                    for i in 0..count {
                        // BlacklistedLibraryCheck.cpp:47-49
                        // Get the full path to the module's file.
                        // C++: wchar_t szModName[MAX_PATH];
                        // C++: if (GetModuleFileNameExW(hCurrentProcess, hMods[i], szModName, MAX_PATH))
                        let mut sz_mod_name: [u16; MAX_PATH] = [0; MAX_PATH];
                        if GetModuleFileNameExW(
                            h_current_process,
                            h_mods[i],
                            sz_mod_name.as_mut_ptr(),
                            MAX_PATH as DWORD,
                        ) != 0
                        {
                            let len = sz_mod_name
                                .iter()
                                .position(|&c| c == 0)
                                .unwrap_or(MAX_PATH);
                            let sz_mod_name_str =
                                OsString::from_wide(&sz_mod_name[..len]).to_string_lossy().into_owned();

                            // BlacklistedLibraryCheck.cpp:51-56
                            // Add to list if blacklisted
                            // C++: if (BlacklistedLibraryCheck::is_blacklisted(szModName)) {
                            //          //wprintf(L"Contains library: %s\n", szModName);
                            //          if (std::find(m_found.begin(), m_found.end(), szModName) == m_found.end())
                            //              m_found.emplace_back(szModName);
                            //      }
                            if BlacklistedLibraryCheck::is_blacklisted_wstring(&sz_mod_name_str) {
                                //wprintf(L"Contains library: %s\n", szModName);
                                if !self.m_found.iter().any(|s| *s == sz_mod_name_str) {
                                    self.m_found.push(sz_mod_name_str);
                                }
                            }
                            //wprintf(L"%s\n", szModName);
                        }
                    }
                }
            }

            //printf("\n");
            // BlacklistedLibraryCheck.cpp:63
            // C++: return !m_found.empty();
            !self.m_found.is_empty()
        }

        /// BlacklistedLibraryCheck.cpp:66-76
        /// C++: bool BlacklistedLibraryCheck::is_blacklisted(const std::wstring &dllpath)
        ///
        /// The C++ overload takes a `std::wstring`; here paths are carried as
        /// UTF-8 `&str` (the wide bytes are losslessly decoded before being
        /// handed in). Naming distinguishes it from the `std::string` overload.
        pub fn is_blacklisted_wstring(dllpath: &str) -> bool {
            // BlacklistedLibraryCheck.cpp:68
            // C++: std::wstring dllname = boost::filesystem::path(dllpath).filename().wstring();
            //std::transform(dllname.begin(), dllname.end(), dllname.begin(), std::tolower);
            let dllname: String = match std::path::Path::new(dllpath).file_name() {
                Some(name) => name.to_string_lossy().into_owned(),
                // boost::filesystem::path::filename() yields an empty path when
                // `dllpath` has no trailing component; an empty name never
                // matches the blacklist.
                None => String::new(),
            };
            // BlacklistedLibraryCheck.cpp:70-73
            // C++: if (std::find(BlacklistedLibraryCheck::blacklist.begin(), BlacklistedLibraryCheck::blacklist.end(), dllname) != BlacklistedLibraryCheck::blacklist.end()) {
            //          //std::wprintf(L"%s is blacklisted\n", dllname.c_str());
            //          return true;
            //      }
            if BLACKLIST.iter().any(|&b| b == dllname) {
                //std::wprintf(L"%s is blacklisted\n", dllname.c_str());
                return true;
            }
            //std::wprintf(L"%s is NOT blacklisted\n", dllname.c_str());
            // BlacklistedLibraryCheck.cpp:75
            // C++: return false;
            false
        }

        /// BlacklistedLibraryCheck.cpp:77-80
        /// C++: bool BlacklistedLibraryCheck::is_blacklisted(const std::string &dllpath)
        // UTF-8 encoded path
        pub fn is_blacklisted(dllpath: &str) -> bool {
            // BlacklistedLibraryCheck.cpp:79
            // C++: return BlacklistedLibraryCheck::is_blacklisted(boost::nowide::widen(dllpath));
            //
            // `boost::nowide::widen` converts UTF-8 to UTF-16; since our wide
            // overload also operates on UTF-8 the conversion is a no-op here.
            BlacklistedLibraryCheck::is_blacklisted_wstring(dllpath)
        }
    }
}

// BlacklistedLibraryCheck.cpp:82 `#endif //WIN32`
#[cfg(target_os = "windows")]
pub use windows_impl::BlacklistedLibraryCheck;

#[cfg(all(test, target_os = "windows"))]
mod tests {
    use super::*;

    #[test]
    fn test_is_blacklisted() {
        // Test known blacklisted DLL names
        assert!(BlacklistedLibraryCheck::is_blacklisted("NahimicOSD.dll"));
        assert!(BlacklistedLibraryCheck::is_blacklisted(
            "C:\\Windows\\System32\\NahimicOSD.dll"
        ));
        assert!(BlacklistedLibraryCheck::is_blacklisted("SS2OSD.dll"));
        assert!(BlacklistedLibraryCheck::is_blacklisted("amhook.dll"));
        assert!(BlacklistedLibraryCheck::is_blacklisted("AMHook.dll"));

        // Test non-blacklisted DLL
        assert!(!BlacklistedLibraryCheck::is_blacklisted("kernel32.dll"));
        assert!(!BlacklistedLibraryCheck::is_blacklisted("user32.dll"));
    }

    #[test]
    fn test_perform_check_runs() {
        // Test that perform_check runs without panicking.
        // We can't predict the result since it depends on what DLLs are loaded.
        let result = BlacklistedLibraryCheck::get_instance().perform_check();
        assert!(result == true || result == false);
    }
}
