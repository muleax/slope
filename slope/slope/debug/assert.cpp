#include "assert.hpp"
#include <cstdarg>
#include <cstdio>

namespace slope::assert_details {

void assert_handler(const char* expression) {
    printf("Expression failed: %s", expression);
}

void assert_handler(const char* expression, const char* format, ...) {
    va_list args;
    va_start(args, format);
    printf("Expression failed: %s\nMessage: ", expression);
    printf(format, args);
    va_end(args);
}

} // slope::assert_details