//! 1:1 port of `GCodeSender.cpp` / `GCodeSender.hpp`.
//!
//! The C++ `GCodeSender` streams G-code to a physical 3D printer over a serial
//! port. It is built entirely on native, NON-wasm-safe machinery:
//!   * `boost::asio::serial_port` + `boost::asio::io_service` async I/O,
//!   * a `boost::thread` background reader/writer thread,
//!   * OS-specific custom-baud handling via `ioctl`/`termios`/`termios2`,
//!     `IOKit/serial/ioss.h` (`IOSSIOSPEED`) on macOS and `EscapeCommFunction`
//!     (`SETDTR`/`CLRDTR`) on Windows,
//!   * `TIOCMGET`/`TIOCMSET` line-control (DTR toggling) in `set_DTR`/`reset`.
//!
//! None of the serial transport is wasm-safe and none of it participates in
//! G-code *generation*; it is a runtime device-communication module. Per the
//! port rules we therefore faithfully port everything that is tractable and
//! wasm-safe — the message queues, the response parser (`on_read`), and the
//! outgoing-line formatter (`do_send`: comment stripping, `N<n>` line numbers,
//! and the XOR checksum) which is the only piece that shapes the bytes sent to
//! the printer — and we mark the native serial transport as blocked (it is
//! documented and present as no-ops / errors, NOT faked with a fake serial
//! port). See the module-level `divergences` notes.
//!
//! `coord_t`/`coordf_t` do not appear in this file. `size_t` -> `usize`,
//! `int` -> `i32`, `unsigned int` -> `u32`.
//!
//! C++ Reference:
//! - GCodeSender.hpp (74 lines)
//! - GCodeSender.cpp (581 lines)

use std::collections::VecDeque;
use std::sync::Mutex;

// GCodeSender.cpp:50
const KEEP_SENT: usize = 20;

/// 1:1 port of C++ class `GCodeSender`.
/// GCodeSender.hpp:16
///
/// The Boost.ASIO `io_service`, `serial_port`, the `background_thread`, and the
/// `read_buffer`/`write_buffer` streambufs are native-only and are NOT modeled
/// here (no wasm-safe serial backend exists). The pure state that drives the
/// protocol — queues, flags, counters, and the temperature/log mirrors — is
/// ported faithfully. The three C++ `boost::mutex` guards (`error_mutex`,
/// `queue_mutex`, `log_mutex`) become `std::sync::Mutex`.
pub struct GCodeSender {
    // GCodeSender.hpp:38-41
    //   asio::io_service io;                 -- native, blocked
    //   asio::serial_port serial;            -- native, blocked
    //   boost::thread background_thread;     -- native, blocked
    //   boost::asio::streambuf read_buffer, write_buffer; -- native, blocked
    /// GCodeSender.hpp:42 -- whether the serial socket is connected
    open: bool,
    /// GCodeSender.hpp:43 -- whether the printer is online
    connected: bool,
    /// GCodeSender.hpp:44
    /// guarded by `error_mutex`
    error: Mutex<bool>,

    // this mutex guards queue, priqueue, can_send, queue_paused, sent, last_sent
    // GCodeSender.hpp:47-48
    /// GCodeSender.hpp:48 -- `queue_mutex` guarding the fields below.
    queue_mutex: Mutex<QueueState>,

    // this mutex guards log, T, B
    // GCodeSender.hpp:56-57
    /// GCodeSender.hpp:57 -- `log_mutex` guarding `log`, `T`, `B`.
    log_mutex: Mutex<LogState>,
}

/// Fields guarded by `queue_mutex`.
/// GCodeSender.hpp:49-54
struct QueueState {
    /// GCodeSender.hpp:49
    queue: VecDeque<String>,
    /// GCodeSender.hpp:50
    priqueue: VecDeque<String>,
    /// GCodeSender.hpp:51
    can_send: bool,
    /// GCodeSender.hpp:52
    queue_paused: bool,
    /// GCodeSender.hpp:53
    sent: usize,
    /// GCodeSender.hpp:54
    last_sent: VecDeque<String>,
}

/// Fields guarded by `log_mutex`.
/// GCodeSender.hpp:58-59
struct LogState {
    /// GCodeSender.hpp:58
    log: VecDeque<String>,
    /// GCodeSender.hpp:59
    t: String,
    /// GCodeSender.hpp:59
    b: String,
}

impl Default for GCodeSender {
    fn default() -> Self {
        Self::new()
    }
}

impl GCodeSender {
    /// GCodeSender.cpp:54-61
    /// C++: GCodeSender::GCodeSender()
    ///     : io(), serial(io), can_send(false), sent(0), open(false), error(false),
    ///       connected(false), queue_paused(false)
    pub fn new() -> Self {
        // GCodeSender.cpp:58-60 -- DEBUG_SERIAL: std::srand(std::time(nullptr));
        // DEBUG_SERIAL is disabled (//#define DEBUG_SERIAL), so this is a no-op.
        Self {
            open: false,    // GCodeSender.cpp:55
            connected: false, // GCodeSender.cpp:56
            error: Mutex::new(false), // GCodeSender.cpp:55
            queue_mutex: Mutex::new(QueueState {
                queue: VecDeque::new(),
                priqueue: VecDeque::new(),
                can_send: false, // GCodeSender.cpp:55
                queue_paused: false, // GCodeSender.cpp:56
                sent: 0,         // GCodeSender.cpp:55
                last_sent: VecDeque::new(),
            }),
            log_mutex: Mutex::new(LogState {
                log: VecDeque::new(),
                t: String::new(),
                b: String::new(),
            }),
        }
    }

    // GCodeSender.cpp:63-66
    // C++: GCodeSender::~GCodeSender() { this->disconnect(); }
    // Rust's Drop would call disconnect(); see `Drop` impl below.

    /// GCodeSender.cpp:68-119
    /// C++: bool GCodeSender::connect(std::string devname, unsigned int baud_rate)
    ///
    /// NATIVE-BLOCKED: opens a `boost::asio::serial_port`, configures parity /
    /// character size / flow control / stop bits / baud rate, posts `do_read`
    /// to the io_service, and spawns the background `io_service::run` thread.
    /// There is no wasm-safe serial backend, so this transport cannot be ported;
    /// it always reports a connection failure (mirrors the C++ catch path which
    /// returns `false`).
    #[allow(unused_variables)]
    pub fn connect(&mut self, devname: String, baud_rate: u32) -> bool {
        // GCodeSender.cpp:71
        self.disconnect();

        // GCodeSender.cpp:73
        self.set_error_status(false);
        // GCodeSender.cpp:74-94 -- serial.open / set_option(...) / set_baud_rate /
        // reset(); on boost::system::system_error -> set_error_status(true); return false.
        // The serial port is native and unavailable here, so the open fails just
        // as the C++ catch branch does.
        self.set_error_status(true); // GCodeSender.cpp:92
        false // GCodeSender.cpp:93
        // NOTE: the success tail (GCodeSender.cpp:96-118) — resetting `sent`,
        // clearing `last_sent`, io.post(do_read), spawning background_thread,
        // returning true — is unreachable without a real serial port.
    }

    /// GCodeSender.cpp:121-159
    /// C++: void GCodeSender::set_baud_rate(unsigned int baud_rate)
    ///
    /// NATIVE-BLOCKED: sets the Boost.ASIO baud-rate option and, when the
    /// requested speed is unsupported, falls back to OS-specific custom-baud
    /// handling via `ioctl`/`termios`/`termios2` / `IOSSIOSPEED`. Requires a
    /// native serial handle; not portable to wasm.
    #[allow(unused_variables)]
    fn set_baud_rate(&mut self, baud_rate: u32) {
        // native serial handle unavailable; no-op.
    }

    /// GCodeSender.cpp:161-181
    /// C++: void GCodeSender::disconnect()
    pub fn disconnect(&mut self) {
        // GCodeSender.cpp:164
        if !self.open {
            return;
        }
        // GCodeSender.cpp:165-166
        self.open = false;
        self.connected = false;
        // GCodeSender.cpp:167-169 -- io.post(do_close); background_thread.join(); io.reset();
        // NATIVE-BLOCKED: no io_service / background thread; the close is a no-op.
        self.do_close();
        // GCodeSender.cpp:170-180 -- commented-out error check / DEBUG_SERIAL trailer.
    }

    /// GCodeSender.cpp:183-187
    /// C++: bool GCodeSender::is_connected() const
    pub fn is_connected(&self) -> bool {
        // GCodeSender.cpp:186
        self.connected
    }

    /// GCodeSender.cpp:189-199
    /// C++: bool GCodeSender::wait_connected(unsigned int timeout = 3) const
    ///
    /// NATIVE-BLOCKED in spirit: the C++ version busy-waits on `this->connected`
    /// (which only flips inside the background reader thread `on_read`) up to
    /// `timeout` seconds, sleeping 100ms between polls. Without the reader thread
    /// `connected` never changes, so we faithfully report the current flag rather
    /// than spin forever. `timeout` is unused here for the same reason.
    #[allow(unused_variables)]
    pub fn wait_connected(&self, timeout: u32) -> bool {
        // GCodeSender.cpp:192-198
        self.connected
    }

    /// GCodeSender.cpp:201-206
    /// C++: size_t GCodeSender::queue_size() const
    pub fn queue_size(&self) -> usize {
        // GCodeSender.cpp:204
        let q = self.queue_mutex.lock().unwrap();
        // GCodeSender.cpp:205
        q.queue.len()
    }

    /// GCodeSender.cpp:208-213
    /// C++: void GCodeSender::pause_queue()
    pub fn pause_queue(&self) {
        // GCodeSender.cpp:211
        let mut q = self.queue_mutex.lock().unwrap();
        // GCodeSender.cpp:212
        q.queue_paused = true;
    }

    /// GCodeSender.cpp:215-223
    /// C++: void GCodeSender::resume_queue()
    pub fn resume_queue(&self) {
        {
            // GCodeSender.cpp:219
            let mut q = self.queue_mutex.lock().unwrap();
            // GCodeSender.cpp:220
            q.queue_paused = false;
        }
        // GCodeSender.cpp:222
        self.send();
    }

    /// GCodeSender.cpp:225-239
    /// C++: void GCodeSender::purge_queue(bool priority = false)
    pub fn purge_queue(&self, priority: bool) {
        // GCodeSender.cpp:228
        let mut q = self.queue_mutex.lock().unwrap();
        // GCodeSender.cpp:229
        if priority {
            // GCodeSender.cpp:230-232 -- clear priority queue (swap with empty list)
            q.priqueue.clear();
        } else {
            // GCodeSender.cpp:233-237 -- clear queue (swap with empty queue)
            q.queue.clear();
            q.queue_paused = false;
        }
    }

    /// GCodeSender.cpp:241-253
    /// C++: std::vector<std::string> GCodeSender::purge_log()
    // purge log and return its contents
    pub fn purge_log(&self) -> Vec<String> {
        // GCodeSender.cpp:245
        let mut log = self.log_mutex.lock().unwrap();
        // GCodeSender.cpp:246-247
        let mut retval: Vec<String> = Vec::with_capacity(log.log.len());
        // GCodeSender.cpp:248-251
        while let Some(front) = log.log.pop_front() {
            retval.push(front);
        }
        // GCodeSender.cpp:252
        retval
    }

    /// GCodeSender.cpp:255-260
    /// C++: std::string GCodeSender::getT() const
    #[allow(non_snake_case)]
    pub fn getT(&self) -> String {
        // GCodeSender.cpp:258
        let log = self.log_mutex.lock().unwrap();
        // GCodeSender.cpp:259
        log.t.clone()
    }

    /// GCodeSender.cpp:262-267
    /// C++: std::string GCodeSender::getB() const
    #[allow(non_snake_case)]
    pub fn getB(&self) -> String {
        // GCodeSender.cpp:265
        let log = self.log_mutex.lock().unwrap();
        // GCodeSender.cpp:266
        log.b.clone()
    }

    /// GCodeSender.cpp:269-278
    /// C++: void GCodeSender::do_close()
    ///
    /// NATIVE-BLOCKED: cancels and closes the Boost.ASIO serial port, setting
    /// the error status on failure. No serial port exists here, so only the
    /// initial `set_error_status(false)` is faithfully reproduced.
    fn do_close(&mut self) {
        // GCodeSender.cpp:272
        self.set_error_status(false);
        // GCodeSender.cpp:273-277 -- serial.cancel(ec)/serial.close(ec); on error
        // set_error_status(true). No native serial port; nothing to cancel/close.
    }

    /// GCodeSender.cpp:280-285
    /// C++: void GCodeSender::set_error_status(bool e)
    fn set_error_status(&self, e: bool) {
        // GCodeSender.cpp:283
        let mut err = self.error.lock().unwrap();
        // GCodeSender.cpp:284
        *err = e;
    }

    /// GCodeSender.cpp:287-292
    /// C++: bool GCodeSender::error_status() const
    pub fn error_status(&self) -> bool {
        // GCodeSender.cpp:290
        let err = self.error.lock().unwrap();
        // GCodeSender.cpp:291
        *err
    }

    /// GCodeSender.cpp:294-309
    /// C++: void GCodeSender::do_read()
    ///
    /// NATIVE-BLOCKED: issues `boost::asio::async_read_until(serial, read_buffer,
    /// '\n', on_read)`. There is no serial port / io_service to read from.
    fn do_read(&mut self) {
        // GCodeSender.cpp:298-308 -- async_read_until(...): no native transport.
    }

    /// GCodeSender.cpp:311-423
    /// C++: void GCodeSender::on_read(const boost::system::error_code& error,
    ///                                size_t bytes_transferred)
    ///
    /// The transport-error / async-read parts are native-blocked, but the line
    /// parsing — connection detection, `ok`/`resend`/`wait` handling, the log
    /// push, and the `T:`/`B:` temperature scrape — is pure string logic and is
    /// ported faithfully. We expose it as a function taking the already-read
    /// `line` so the (wasm-safe) protocol parsing is exercisable and parity-
    /// checkable; the C++ entry reads the line from `read_buffer` via
    /// `std::getline`. `error`/`bytes_transferred` are inapplicable without a
    /// real transport.
    fn on_read(&mut self, line: &str) {
        // GCodeSender.cpp:315
        self.set_error_status(false);
        // GCodeSender.cpp:316-333 -- transport error handling (native-blocked):
        //   on error, possibly retry (OS X bug 45), else do_close + set_error_status.
        // GCodeSender.cpp:335-337 -- std::istream is(&read_buffer); std::getline(is, line);
        //   The caller supplies `line` (native read replaced).

        // C++ keeps a mutable copy of `line` (it is trimmed in the resend path).
        let mut line = line.to_string();

        // GCodeSender.cpp:338
        if !line.is_empty() {
            // GCodeSender.cpp:339-341 -- DEBUG_SERIAL trace (disabled).

            // note that line might contain \r at its end
            // parse incoming line
            // GCodeSender.cpp:345-349
            if !self.connected
                && (line.starts_with("start")
                    || line.starts_with("Grbl ")
                    || line.starts_with("ok")
                    || line.contains("T:"))
            {
                // GCodeSender.cpp:350
                self.connected = true;
                {
                    // GCodeSender.cpp:352-353
                    let mut q = self.queue_mutex.lock().unwrap();
                    q.can_send = true;
                }
                // GCodeSender.cpp:355
                self.send();
            } else if line.starts_with("ok") {
                // GCodeSender.cpp:356
                {
                    // GCodeSender.cpp:358-359
                    let mut q = self.queue_mutex.lock().unwrap();
                    q.can_send = true;
                }
                // GCodeSender.cpp:361
                self.send();
            } else if starts_with_ci(&line, "resend") || starts_with_ci(&line, "rs") {
                // GCodeSender.cpp:362-363
                //   boost::istarts_with(line, "resend")  // Marlin uses "Resend: "
                //   || boost::istarts_with(line, "rs")

                // extract the first number from line
                // GCodeSender.cpp:365 -- trim_left_if(line, !is_digit())
                trim_left_if_not_digit(&mut line);
                // GCodeSender.cpp:366 -- lexical_cast<size_t>(line.substr(0, first_not_of("0123456789")))
                let end = find_first_not_of(&line, b"0123456789", 0).unwrap_or(line.len());
                let toresend: usize = line[0..end].parse::<usize>().unwrap_or(0);

                // GCodeSender.cpp:368-370 -- DEBUG_SERIAL trace (disabled).

                // GCodeSender.cpp:372
                let (do_resend, oldest) = {
                    let mut q = self.queue_mutex.lock().unwrap();
                    // C++ reads `sent`/`last_sent` here both for the condition and the body.
                    if toresend > q.sent - q.last_sent.len() && toresend <= q.sent {
                        // GCodeSender.cpp:373-393 (inside queue_mutex lock)

                        // GCodeSender.cpp:376
                        let lines_to_resend = q.sent - toresend + 1;
                        // GCodeSender.cpp:377-379 -- DEBUG_SERIAL trace (disabled).

                        // move the unsent lines to priqueue
                        // GCodeSender.cpp:381-385
                        //   priqueue.insert(priqueue.begin(),
                        //       last_sent.begin() + last_sent.size() - lines_to_resend,
                        //       last_sent.end());
                        let start = q.last_sent.len() - lines_to_resend;
                        let to_move: Vec<String> =
                            q.last_sent.iter().skip(start).cloned().collect();
                        // insert at the beginning, preserving order
                        for s in to_move.into_iter().rev() {
                            q.priqueue.push_front(s);
                        }

                        // we can empty last_sent because it's not useful anymore
                        // GCodeSender.cpp:388
                        q.last_sent.clear();

                        // start resending with the requested line number
                        // GCodeSender.cpp:391-392
                        q.sent = toresend - 1;
                        q.can_send = true;
                        (true, 0)
                    } else {
                        // GCodeSender.cpp:396 -- printf oldest = sent - last_sent.size()
                        (false, q.sent - q.last_sent.len())
                    }
                };
                if do_resend {
                    // GCodeSender.cpp:394
                    self.send();
                } else {
                    // GCodeSender.cpp:395-397
                    // printf("Cannot resend %zu (oldest we have is %zu)\n", toresend, oldest);
                    let _ = oldest;
                }
            } else if line.starts_with("wait") {
                // GCodeSender.cpp:398-399 -- ignore
            } else {
                // push any other line into the log
                // GCodeSender.cpp:401-403
                let mut log = self.log_mutex.lock().unwrap();
                log.log.push_back(line.clone());
            }

            // parse temperature info
            // GCodeSender.cpp:406-420
            {
                // GCodeSender.cpp:408
                let pos = line.find("T:");
                if let Some(pos) = pos {
                    // GCodeSender.cpp:409 -- pos != npos && line.size() > pos + 2
                    if line.len() > pos + 2 {
                        // we got temperature info
                        // GCodeSender.cpp:411
                        let mut log = self.log_mutex.lock().unwrap();
                        // GCodeSender.cpp:412
                        //   T = line.substr(pos+2, first_not_of("0123456789.", pos+2) - (pos+2))
                        let t_end = find_first_not_of(&line, b"0123456789.", pos + 2)
                            .unwrap_or(line.len());
                        log.t = line[pos + 2..t_end].to_string();

                        // GCodeSender.cpp:414
                        let bpos = line.find("B:");
                        if let Some(bpos) = bpos {
                            // GCodeSender.cpp:415 -- pos != npos && line.size() > pos + 2
                            if line.len() > bpos + 2 {
                                // we got bed temperature info
                                // GCodeSender.cpp:417
                                let b_end =
                                    find_first_not_of(&line, b"0123456789.", bpos + 2)
                                        .unwrap_or(line.len());
                                log.b = line[bpos + 2..b_end].to_string();
                            }
                        }
                    }
                }
            }
        }
        // GCodeSender.cpp:422
        self.do_read();
    }

    /// GCodeSender.cpp:425-440
    /// C++: void GCodeSender::send(const std::vector<std::string> &lines, bool priority = false)
    // append lines to queue
    pub fn send_lines(&self, lines: &[String], priority: bool) {
        {
            // GCodeSender.cpp:430
            let mut q = self.queue_mutex.lock().unwrap();
            // GCodeSender.cpp:431-437
            for line in lines.iter() {
                if priority {
                    q.priqueue.push_back(line.clone());
                } else {
                    q.queue.push_back(line.clone());
                }
            }
        }
        // GCodeSender.cpp:439
        self.send();
    }

    /// GCodeSender.cpp:442-455
    /// C++: void GCodeSender::send(const std::string &line, bool priority = false)
    // append line to queue
    pub fn send_line(&self, line: &str, priority: bool) {
        {
            // GCodeSender.cpp:447
            let mut q = self.queue_mutex.lock().unwrap();
            // GCodeSender.cpp:448-452
            if priority {
                q.priqueue.push_back(line.to_string());
            } else {
                q.queue.push_back(line.to_string());
            }
        }
        // GCodeSender.cpp:454
        self.send();
    }

    /// GCodeSender.cpp:457-461
    /// C++: void GCodeSender::send()
    ///
    /// NATIVE-BLOCKED transport: the C++ version posts `do_send` to the
    /// io_service (`io.post(boost::bind(&GCodeSender::do_send, this))`) so it
    /// runs on the background thread. Without an io_service there is nothing to
    /// post to; `do_send` is the wasm-safe formatter and is exposed directly.
    fn send(&self) {
        // GCodeSender.cpp:460 -- io.post(do_send): no io_service available.
    }

    /// GCodeSender.cpp:463-530
    /// C++: void GCodeSender::do_send()
    ///
    /// This is the only routine that shapes the bytes written to the printer:
    /// it dequeues (priqueue first, then queue unless paused), strips trailing
    /// `;` comments and surrounding whitespace, prepends the `N<line_num>` line
    /// number, computes the XOR checksum over the whole `N... ` string, and
    /// appends `*<cs>\n`. All of that is pure and ported faithfully. The final
    /// `async_write` to the serial port is native-blocked; we return the formed
    /// line so the formatting is observable/parity-checkable instead.
    ///
    /// Returns `Some(full_line)` when a line was formatted and "written", or
    /// `None` when nothing was sent (cannot send, or only blank/comment lines).
    pub fn do_send(&self) -> Option<String> {
        // GCodeSender.cpp:466
        let mut q = self.queue_mutex.lock().unwrap();

        // printer is not connected or we're still waiting for the previous ack
        // GCodeSender.cpp:469
        if !q.can_send {
            return None;
        }

        // GCodeSender.cpp:471
        let mut line = String::new();
        // GCodeSender.cpp:472
        while !q.priqueue.is_empty() || (!q.queue.is_empty() && !q.queue_paused) {
            // GCodeSender.cpp:473-479
            if !q.priqueue.is_empty() {
                line = q.priqueue.pop_front().unwrap();
            } else {
                line = q.queue.pop_front().unwrap();
            }

            // strip comments
            // GCodeSender.cpp:482-484
            if let Some(comment_pos) = line.find(';') {
                line.truncate(comment_pos);
            }
            // GCodeSender.cpp:485 -- boost::algorithm::trim(line)
            let trimmed = line.trim().to_string();
            line = trimmed;

            // if line is not empty, send it
            // GCodeSender.cpp:488
            if !line.is_empty() {
                break;
            }
            // if line is empty, process next item in queue
            // GCodeSender.cpp:489
        }
        // GCodeSender.cpp:491
        if line.is_empty() {
            return None;
        }

        // compute full line
        // GCodeSender.cpp:494
        q.sent += 1;
        // GCodeSender.cpp:495-500
        // #ifndef DEBUG_SERIAL: const auto line_num = this->sent;
        let line_num = q.sent;
        // GCodeSender.cpp:501
        let mut full_line = format!("N{} {}", line_num, line);

        // calculate checksum
        // GCodeSender.cpp:504-506
        let mut cs: i32 = 0;
        for it in full_line.bytes() {
            cs ^= it as i32;
        }

        // write line to device
        // GCodeSender.cpp:509-511
        full_line.push('*');
        full_line.push_str(&cs.to_string());
        full_line.push('\n');

        // GCodeSender.cpp:513-515 -- DEBUG_SERIAL trace (disabled).

        // GCodeSender.cpp:517
        q.last_sent.push_back(line);
        // GCodeSender.cpp:518
        q.can_send = false;

        // GCodeSender.cpp:520-522
        while q.last_sent.len() > KEEP_SENT {
            q.last_sent.pop_front();
        }

        // GCodeSender.cpp:524-529
        //   std::ostream os(&write_buffer); os << full_line;
        //   boost::asio::async_write(serial, write_buffer, on_write);
        // NATIVE-BLOCKED: no serial port to write to. Return the formed line so
        // the (parity-relevant) formatting is observable.
        Some(full_line)
    }

    /// GCodeSender.cpp:532-546
    /// C++: void GCodeSender::on_write(const boost::system::error_code& error,
    ///                                 size_t bytes_transferred)
    ///
    /// NATIVE-BLOCKED transport-completion handler: on success it chains the next
    /// `do_send`; on error it closes the port and sets the error status. Without
    /// a real `async_write` there is no completion to handle, so we faithfully
    /// reproduce the success path's `set_error_status(false)` + `do_send`.
    #[allow(dead_code)]
    fn on_write(&mut self) {
        // GCodeSender.cpp:536
        self.set_error_status(false);
        // GCodeSender.cpp:537-543 -- if error { if open { do_close(); set_error_status(true) } return }
        // GCodeSender.cpp:545
        self.do_send();
    }

    /// GCodeSender.cpp:548-567
    /// C++: void GCodeSender::set_DTR(bool on)
    ///
    /// NATIVE-BLOCKED: toggles the serial DTR line via `EscapeCommFunction`
    /// (`SETDTR`/`CLRDTR`) on Windows or `ioctl(fd, TIOCMGET/TIOCMSET, ...)`
    /// with the `TIOCM_DTR` bit elsewhere. Requires a native serial handle.
    #[allow(unused_variables)]
    fn set_dtr(&mut self, on: bool) {
        // native serial handle unavailable; no-op.
    }

    /// GCodeSender.cpp:569-578
    /// C++: void GCodeSender::reset()
    ///
    /// NATIVE-BLOCKED: pulses DTR low/high/low with 200ms/200ms/500ms sleeps to
    /// hardware-reset the printer's controller. Depends on `set_DTR` (native).
    fn reset(&mut self) {
        // GCodeSender.cpp:572-577
        self.set_dtr(false);
        // std::this_thread::sleep_for(200ms);
        self.set_dtr(true);
        // std::this_thread::sleep_for(200ms);
        self.set_dtr(false);
        // std::this_thread::sleep_for(500ms);
    }
}

/// GCodeSender.cpp:63-66
/// C++: GCodeSender::~GCodeSender() { this->disconnect(); }
impl Drop for GCodeSender {
    fn drop(&mut self) {
        self.disconnect();
    }
}

/// Case-insensitive prefix test, faithful to `boost::istarts_with`.
/// Used at GCodeSender.cpp:362-363.
fn starts_with_ci(haystack: &str, prefix: &str) -> bool {
    let h = haystack.as_bytes();
    let p = prefix.as_bytes();
    if h.len() < p.len() {
        return false;
    }
    h.iter()
        .zip(p.iter())
        .all(|(a, b)| a.eq_ignore_ascii_case(b))
}

/// Faithful to `boost::algorithm::trim_left_if(line, !boost::algorithm::is_digit())`:
/// erase the leading run of non-digit characters in place.
/// Used at GCodeSender.cpp:365.
fn trim_left_if_not_digit(line: &mut String) {
    let pos = line
        .bytes()
        .position(|b| b.is_ascii_digit())
        .unwrap_or(line.len());
    line.drain(0..pos);
}

/// Faithful to `std::string::find_first_not_of(chars, start)`: index of the
/// first byte at or after `start` that is NOT in `chars`, or `None` (npos).
/// Used at GCodeSender.cpp:366, 412, 417.
fn find_first_not_of(s: &str, chars: &[u8], start: usize) -> Option<usize> {
    let bytes = s.as_bytes();
    if start >= bytes.len() {
        return None;
    }
    bytes[start..]
        .iter()
        .position(|b| !chars.contains(b))
        .map(|p| p + start)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn do_send_formats_line_number_and_checksum() {
        // GCodeSender.cpp:463-530 -- verify N<n> prefix and XOR checksum + *cs\n.
        let s = GCodeSender::new();
        {
            let mut q = s.queue_mutex.lock().unwrap();
            q.can_send = true;
            q.queue.push_back("G1 X1 ; comment".to_string());
        }
        let out = s.do_send().expect("a line should be formatted");
        // comment stripped + trimmed -> "G1 X1"; first line number is 1.
        let body = "N1 G1 X1";
        let mut cs: i32 = 0;
        for b in body.bytes() {
            cs ^= b as i32;
        }
        assert_eq!(out, format!("{}*{}\n", body, cs));
    }

    #[test]
    fn do_send_skips_blank_and_comment_only_lines() {
        // GCodeSender.cpp:482-490 -- comment-only / blank lines are skipped.
        let s = GCodeSender::new();
        {
            let mut q = s.queue_mutex.lock().unwrap();
            q.can_send = true;
            q.queue.push_back("   ; only a comment".to_string());
            q.queue.push_back("".to_string());
            q.queue.push_back("M104 S200".to_string());
        }
        let out = s.do_send().expect("the non-blank line is sent");
        assert!(out.starts_with("N1 M104 S200*"));
    }

    #[test]
    fn on_read_scrapes_temperatures() {
        // GCodeSender.cpp:406-420 -- T:/B: scrape stops at first non [0-9.] char.
        let mut s = GCodeSender::new();
        s.on_read("ok T:201.5 /210.0 B:59.8 /60.0");
        assert_eq!(s.getT(), "201.5");
        assert_eq!(s.getB(), "59.8");
    }

    #[test]
    fn on_read_detects_connection() {
        // GCodeSender.cpp:345-355 -- "start" marks the printer online + can_send.
        let mut s = GCodeSender::new();
        assert!(!s.is_connected());
        s.on_read("start");
        assert!(s.is_connected());
        assert!(s.queue_mutex.lock().unwrap().can_send);
    }
}
