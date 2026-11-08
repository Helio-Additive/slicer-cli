// slicer-cli override layer. The log-file sink of BambuStudio v02.09.01.70
// (src/libslic3r/LogSink.cpp) without its encrypted-file path: that path
// needs the desktop's slic3r/Utils/Http.hpp and BBLUtil.hpp (the AES key is
// fetched from Bambu's servers), which a headless libslic3r does not carry.
// Log.cpp links LogSinkUtil::get_log_filename_format, LogSinkBackend and
// LogSinkBackend::is_consuming_on_this_thread; the CLI never calls
// set_log_path_and_level, so no log file is ever opened. Every function below
// is upstream's, with the LOG_ENC_AES_256_CBC branch of consume() writing
// plain text, as the LOG_ENC_NONE branch does.
#include "LogSink.hpp"

#include <boost/filesystem.hpp>

#include <ctime>
#include <iomanip>
#include <sstream>

#ifdef _WIN32
#include <process.h>
#else
#include <unistd.h>
#endif

namespace Slic3r
{

static std::string s_message_newline = "\n";

namespace {
// True while this thread is inside LogSinkBackend::consume().
thread_local bool t_consuming = false;

// Marks the current thread as consuming for the lifetime of the scope, also when it exits by an exception.
class ConsumingScope
{
public:
    ConsumingScope() { t_consuming = true; }
    ~ConsumingScope() { t_consuming = false; }

    ConsumingScope(const ConsumingScope&)            = delete;
    ConsumingScope& operator=(const ConsumingScope&) = delete;
};

unsigned current_pid()
{
#ifdef _WIN32
    return static_cast<unsigned>(_getpid());
#else
    return static_cast<unsigned>(::getpid());
#endif
}
} // namespace

bool LogSinkBackend::is_consuming_on_this_thread()
{
    return t_consuming;
}

LogSinkBackend::LogSinkBackend(const std::string& base_path, const LogEncOptions& options)
    : m_log_enc_options(options)
{
    set_file_name_pattern(base_path);
    set_rotation_size(100 * 1024 * 1024);
    set_auto_newline_mode(boost::log::sinks::disabled_auto_newline);// newline need to be added before encryption
    set_open_mode(std::ios::binary | std::ios::app);
}

void LogSinkBackend::reset_enc_key_info()
{
    m_log_enc_key.clear();
    m_log_enc_key_iv_base.clear();
    m_log_enc_key_iv.clear();
    m_log_enc_key_tag.clear();
    m_log_enc_key_timestamp = 0;
}

bool LogSinkBackend::update_enc_option(const std::string& base_path, const LogEncOptions& options)
{
    if (m_log_enc_options.enc_type == options.enc_type &&
        m_log_enc_options.enc_key_host_env == options.enc_key_host_env &&
        m_log_enc_options.enc_key_url == options.enc_key_url) {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_log_mutex);

    m_log_enc_options = options;
    reset_enc_key_info();

    try {
        set_file_name_pattern(base_path);
        rotate_file();
    } catch (const std::exception& ) {
        printf("%s, error: %s\n", __FUNCTION__, "set_file_name_pattern or rotate_file exception");
    }

    return true;
}

void LogSinkBackend::consume(const boost::log::record_view& rec, const std::string& formatted_message)
{
    ConsumingScope consuming;

    if (formatted_message.empty()) {
        return;
    }

    // Headless: no encryption key source, so every record is written as plain text.
    return text_file_backend::consume(rec, formatted_message + s_message_newline);
}

// rules
// studio_%a_%b_%d_%H_%M_%S_<pid>[_enc][_cn].log.%N
std::string LogSinkUtil::get_log_filename_format(const std::string& log_dir, const LogEncOptions& enc_opts)
{
    std::time_t       t = std::time(0);
    std::tm* now_time = std::localtime(&t);
    std::stringstream buf;
    buf << std::put_time(now_time, "studio_%a_%b_%d_%H_%M_%S_");
    buf << current_pid();
    if (enc_opts.enc_type == LogEncOptions::LOG_ENC_NONE) {
        buf << ".log.%N";
    } else {
        // default to us
        buf << "_enc";
        if (enc_opts.enc_key_host_env == "cn") {
            buf << "_cn";
        } else if (enc_opts.enc_key_host_env == "dc") {
            buf << "_dc"; // dc means the user doesn't set the region
        } else {
            // no suffix for "us" region
        }

        buf << ".log.%N";
    }

    try {
        auto log_folder = boost::filesystem::path(log_dir);
        if (!boost::filesystem::exists(log_folder)) {
            boost::filesystem::create_directories(log_folder);
        }
        auto base_path = (log_folder / buf.str()).make_preferred();
        return base_path.string();
    } catch (const std::exception& e) {
        printf("%s, error: %s\n", __FUNCTION__, e.what());
    }

    return buf.str();
}

} // namespace Slic3r
