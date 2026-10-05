// cli_pipe.cpp — see cli_pipe.hpp.
#include "cli_pipe.hpp"

#if defined(__linux__) || defined(__LINUX__)
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>

#include <fcntl.h>
#include <unistd.h>

#include <nlohmann/json.hpp>
#endif

namespace slicer_cli {

#if defined(__linux__) || defined(__LINUX__)
namespace {

constexpr size_t kPipeBufferSize = 512;   // PIPE_BUFFER_SIZE

// cli_callback_mgr_t (BambuStudio.cpp 246-424; OrcaSlicer.cpp 222-397).
struct CliCallbackMgr {
    int                     m_plate_count{0};
    int                     m_plate_index{0};
    int                     m_progress{0};
    int                     m_total_progress{0};
    std::string             m_message;
    int                     m_warning_step{-1};
    bool                    m_exit{false};
    bool                    m_data_ready{false};
    bool                    m_started{false};
    std::thread             m_thread;
    std::mutex              m_mutex;
    std::condition_variable m_condition;
    int                     m_pipe_fd{-1};

    // A run that ends without stop() (any return from main) still joins the
    // writer: a joinable std::thread at exit would end the process.
    ~CliCallbackMgr() { stop(); }

    bool is_started() {
        std::lock_guard<std::mutex> lck(m_mutex);
        return m_started;
    }

    void set_plate_info(int index, int count) {
        std::lock_guard<std::mutex> lck(m_mutex);
        m_plate_count = count;
        m_plate_index = index;
        m_progress    = 0;
    }

    void notify() {
        if (m_pipe_fd < 0)
            return;
        nlohmann::json j;
        j["plate_index"]   = m_plate_index;
        j["plate_count"]   = m_plate_count;
        j["plate_percent"] = m_progress;
        j["total_percent"] = m_total_progress;
        if (m_warning_step >= 0)
            j["warning"] = m_message;
        else
            j["message"] = m_message;
        const std::string notify_message = j.dump(-1, ' ', false, nlohmann::json::error_handler_t::replace);
        char pipe_message[kPipeBufferSize] = {0};
        std::snprintf(pipe_message, kPipeBufferSize, "%s\n", notify_message.c_str());
        const ssize_t ret = ::write(m_pipe_fd, pipe_message, std::strlen(pipe_message));
        (void)ret;
    }

    void thread_proc() {
        std::unique_lock<std::mutex> lck(m_mutex);
        m_started    = true;
        m_data_ready = false;
        lck.unlock();
        m_condition.notify_one();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        while (true) {
            lck.lock();
            m_condition.wait(lck, [this]() { return m_data_ready || m_exit; });
            if (m_data_ready) {
                notify();
                m_data_ready = false;
            }
            if (m_exit)
                break;
            lck.unlock();
            m_condition.notify_one();
        }
        lck.unlock();
    }

    void update(int percent, const std::string& message, int warning_step) {
        std::unique_lock<std::mutex> lck(m_mutex);
        if (!m_started)
            return;
        if (m_progress >= percent && warning_step == -1)
            return;   // already reported
        const int old_total_progress = m_total_progress;
        if (warning_step == -1) {
            m_progress = percent;
            if (m_plate_count <= 1 && m_plate_index >= 1)
                m_total_progress = 3 + 0.9 * m_progress;
            else if (m_plate_count > 1 && m_plate_index >= 1)
                m_total_progress = 3 + ((float) (m_plate_index - 1) * 90) / m_plate_count + ((float) m_progress * 0.9) / m_plate_count;
            else
                m_total_progress = m_progress;
        }
        if (m_total_progress < old_total_progress)
            m_total_progress = old_total_progress;
        m_message      = message;
        m_warning_step = warning_step;
        m_data_ready   = true;
        lck.unlock();
        m_condition.notify_one();
    }

    bool start(const std::string& pipe_name) {
        int retry_count = 0;
        m_pipe_fd = ::open(pipe_name.c_str(), O_WRONLY | O_NONBLOCK);
        while (m_pipe_fd < 0) {
            retry_count++;
            if (retry_count >= 50)
                return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            m_pipe_fd = ::open(pipe_name.c_str(), O_WRONLY | O_NONBLOCK);
        }
        std::unique_lock<std::mutex> lck(m_mutex);
        m_thread = std::thread([this] { this->thread_proc(); });
        m_condition.wait(lck, [this]() { return m_started; });
        lck.unlock();
        m_condition.notify_one();
        return true;
    }

    void stop() {
        std::unique_lock<std::mutex> lck(m_mutex);
        if (!m_started)
            return;
        m_exit = true;
        lck.unlock();
        m_condition.notify_one();
        if (m_thread.joinable())
            m_thread.join();
        lck.lock();
        m_started = false;
        lck.unlock();
        if (m_pipe_fd > 0) {
            ::close(m_pipe_fd);
            m_pipe_fd = -1;
        }
    }
};

CliCallbackMgr& mgr() {
    static CliCallbackMgr m;
    return m;
}

} // namespace

bool pipe_supported() { return true; }
bool pipe_start(const std::string& name) { return mgr().start(name); }
bool pipe_started() { return mgr().is_started(); }
void pipe_set_plate_info(int index, int count) { mgr().set_plate_info(index, count); }
void pipe_update(int percent, const std::string& message, int warning_step) {
    mgr().update(percent, message, warning_step);
}
void pipe_stop() { mgr().stop(); }

#else

bool pipe_supported() { return false; }
bool pipe_start(const std::string&) { return false; }
bool pipe_started() { return false; }
void pipe_set_plate_info(int, int) {}
void pipe_update(int, const std::string&, int) {}
void pipe_stop() {}

#endif

} // namespace slicer_cli
