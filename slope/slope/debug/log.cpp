#include "slope/debug/log.hpp"
#include <cstdio>

namespace slope::log {

void info(const char* format, ...) {
    va_list args;
    va_start(args, format);
    printf("[INFO] ");
    printf(format, args);
    va_end(args);
}

} // namespace slope::log