#pragma once
#include <cmath>

namespace slope {

constexpr float PI = M_PI;
constexpr float SQRT2 = M_SQRT2;
constexpr float EPSILON = 1e-6f;

template<class T>
constexpr T sqr(T value) {
    static_assert(std::is_floating_point_v<T>);
    return value * value;
}

template<class T>
constexpr T lerp(T from, T to, T factor) {
    static_assert(std::is_floating_point_v<T>);
    return from + (to - from) * factor;
}

template<class T>
constexpr T clamp(T value, T min, T max) {
    static_assert(std::is_floating_point_v<T>);
    return value < min ? min : (max < value ? max : value);
}

template<class T>
constexpr bool equal(T a, T b, T epsilon = static_cast<T>(EPSILON)) {
    static_assert(std::is_floating_point_v<T>);
    return a - b > -epsilon && a - b < epsilon;
}

template<class T>
constexpr T pi() {
    static_assert(std::is_floating_point_v<T>);
    return static_cast<T>(M_PI);
}

template<class T>
constexpr T degrees(T radians) {
    static_assert(std::is_floating_point_v<T>);
    return radians / pi<T>() * static_cast<T>(180.f);
}

template<class T>
constexpr T radians(T degrees) {
    static_assert(std::is_floating_point_v<T>);
    return degrees / static_cast<T>(180.f) * pi<T>();
}
} // slope