#pragma once
#include "slope/math/vector3.hpp"
#include "slope/math/math_utils.hpp"
#include "slope/debug/assert.hpp"

namespace slope {

struct AABB {
    vec3 min = vec3{FLOAT_MAX};
    vec3 max = vec3{-FLOAT_MAX};

    AABB() = default;
    AABB(const vec3& in_min, const vec3& in_max) : min(in_min), max(in_max) {}
    explicit AABB(const vec3& point) : min(point), max(point) {}

    void reset(const vec3& min, const vec3& max);
    void reset(const vec3& point);
    void extend(const vec3& v);
    void combine(const AABB& other);
    void combine(const AABB& a, const AABB& b);

    bool intersects(const AABB& other) const;
    bool contains(const AABB& other) const;

    float surface_area() const;
    float volume() const;
};

inline void AABB::reset(const vec3& in_min, const vec3& in_max) {
    min = in_min;
    max = in_max;
}

inline void AABB::reset(const vec3& point) {
    reset(point, point);
}

inline void AABB::extend(const vec3& point) {
    SL_ASSERT(min.x <= max.x && min.y <= max.y && min.z <= max.z);

    for (int i = 0; i < 3; i++) {
        if (point.data[i] < min.data[i])
            min.data[i] = point.data[i];
        else if (point.data[i] > max.data[i])
            max.data[i] = point.data[i];
    }
}

inline void AABB::combine(const AABB& other) {
    for (int i = 0; i < 3; i++) {
        if (other.min.data[i] < min.data[i])
            min.data[i] = other.min.data[i];
        if (other.max.data[i] > max.data[i])
            max.data[i] = other.max.data[i];
    }
}

inline void AABB::combine(const AABB& a, const AABB& b) {
    *this = a;
    combine(b);
}

inline bool AABB::intersects(const AABB& other) const {
    return  (max.x >= other.min.x) && (min.x <= other.max.x) &&
            (max.y >= other.min.y) && (min.y <= other.max.y) &&
            (max.z >= other.min.z) && (min.z <= other.max.z);
}

inline bool AABB::contains(const AABB& other) const {
    return  (min.x <= other.min.x) && (max.x >= other.max.x) &&
            (min.y <= other.min.y) && (max.y >= other.max.y) &&
            (min.z <= other.min.z) && (max.z >= other.max.z);
}

inline float AABB::surface_area() const {
    float dx = max.x - min.x;
    float dy = max.y - min.y;
    float dz = max.z - min.z;
    return 2.f * (dx * (dy + dz) + dy * dz);
}

inline float AABB::volume() const {
    float dx = max.x - min.x;
    float dy = max.y - min.y;
    float dz = max.z - min.z;
    return dx * dy * dz;
}

} // slope
