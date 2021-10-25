#include "slope/debug/log.hpp"
#include <cstdio>

namespace slope::log::details {

void log_impl(const char* prefix, const char* content) {
    printf("%s: %s\n", prefix, content);
}

} // slope::log::details