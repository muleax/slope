#pragma once
#include "slope/debug/assert.hpp"
#include <cmath>
#include <limits>

namespace slope {

constexpr double    PI = 3.14159265358979323846264338327950288;
constexpr float     EPSILON = 1e-6f;

constexpr float     FLOAT_MAX = std::numeric_limits<float>::max();

struct Interval {
    float min = FLOAT_MAX;
    float max = -FLOAT_MAX;

    Interval() = default;
    Interval(float in_min, float in_max) : min(in_min), max(in_max) {}
    explicit Interval(float value) : min(value), max(value) {}

    void reset(float value) {
        reset(value, value);
    }

    void reset(float in_min, float in_max) {
        min = in_min;
        max = in_max;
    }

    void extend(float value) {
        SL_ASSERT(min <= max);

        if (value > max)
            max = value;
        else if (value < min)
            min = value;
    }

    bool contains(float value) const {
        return value >= min && value <= max;
    }

    bool intersects(const Interval& other) const {
        return other.max >= min && other.min <= max;
    }
};

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