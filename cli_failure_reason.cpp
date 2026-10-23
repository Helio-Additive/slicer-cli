// cli_failure_reason.cpp — see cli_failure_reason.hpp.
#include "cli_failure_reason.hpp"

#include <boost/algorithm/string/trim.hpp>

namespace slicer_cli {

void add_failure_reason(std::string& reasons, const std::string& sentence) {
    if (!sentence.empty())
        reasons += (reasons.empty() ? "" : " ") + sentence;
}

void add_engine_reason(std::string& reasons, const char* what) {
    const std::string text = boost::algorithm::trim_copy(std::string(what ? what : ""));
    if (!text.empty())
        reasons = text + (reasons.empty() ? "" : " " + reasons);
}

} // namespace slicer_cli
