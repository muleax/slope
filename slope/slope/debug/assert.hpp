#pragma once
#include <csignal>

namespace slope::assert_details {

void assert_handler(const char* expression);
void assert_handler(const char* expression, const char* format, ...);

inline void raise_impl() {
#ifdef WIN32
    __debugbreak();
#else
    raise(SIGTRAP);
#endif
}

} // slope::assert_details


#define SL_VERIFY(expression, ...)                                          \
    if (!(expression)) {                                                    \
        slope::assert_details::assert_handler(#expression, ##__VA_ARGS__);  \
        slope::assert_details::raise_impl();                                \
    }

#ifdef SLOPE_DEBUG
#define SL_ASSERT(expression, ...) SL_VERIFY(expression, ##__VA_ARGS__)
#else
#define SL_ASSERT(expression, ...) ((void)0)
#endif // SLOPE_DEBUG
