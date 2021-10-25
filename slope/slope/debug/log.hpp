#pragma once
#include "fmt/format.h"

namespace slope::log {

namespace details {
void log_impl(const char* prefix, const char* content);
}

template <class... Ts>
void debug(fmt::format_string<Ts...> format, Ts&&... args) {
    details::log_impl("DEBUG", fmt::format(format, std::forward<Ts>(args)...).c_str());
}

template <class... Ts>
void info(const char* format, Ts&&... args) {
    details::log_impl("INFO", fmt::format(format, std::forward<Ts>(args)...).c_str());
}

template <class... Ts>
void error(const char* format, Ts&&... args) {
    details::log_impl("ERROR", fmt::format(format, std::forward<Ts>(args)...).c_str());
}

template <class... Ts>
void critical(const char* format, Ts&&... args) {
    details::log_impl("CRITICAL", fmt::format(format, std::forward<Ts>(args)...).c_str());
}

} // slope::log