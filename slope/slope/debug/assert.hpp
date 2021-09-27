#pragma once

namespace slope::assert_details {

void assert_handler(const char* expression);
void assert_handler(const char* expression, const char* format, ...);

} // slope::assert_details

#define SL_VERIFY(expression, ...)                                                  \
do {                                                                                \
    if (!(expression)) {                                                            \
        if (kw::assert_details::assert_handler(#expression, skip, ##__VA_ARGS__)) { \
            __debugbreak();                                                         \
        }                                                                           \
    }                                                                               \
} while (false)

#ifdef SLOPE_DEBUG
#define SL_ASSERT(expression, ...) SL_VERIFY(expression, ##__VA_ARGS__)
#else
#define SL_ASSERT(expression, ...) ((void)0)
#endif // SLOPE_DEBUG
