#pragma once
#include "slope/math/vector3.hpp"
#include <optional>

namespace slope {

struct ContactGeom {
    vec3 p1;
    vec3 p2;
    vec3 normal;
};

struct Plane {
    vec3 normal;
    float dot;

    Plane(const vec3& normal, float dot) : normal(normal), dot(dot) {}
    Plane(const vec3& normal, const vec3& point) : normal(normal), dot(normal.dot(point)) {}

    std::optional<float>    intersect_ray(const vec3& ray_beg, const vec3& ray_dir) const;
    float                   distance(const vec3& p) const;
};

struct LineSegment {
    vec3 begin;
    vec3 end;

    const vec3* data() const { return &begin; }
    vec3* data() { return &begin; }

    const vec3& operator [](int i) const { return data()[i]; }
    vec3& operator [](int i) { return data()[i]; }

    void closest_point(const vec3& other, float& t, vec3& p) const;
    void closest_point(const LineSegment& other, float& t1, float& t2, vec3& p1, vec3& p2) const;
};

vec3 find_orthogonal(const vec3& axis);

inline void find_tangent(vec3& out_u, vec3& out_v, const vec3& normal) {
    out_u = find_orthogonal(normal).normalized();
    out_v = out_u.cross(normal);
}

inline std::optional<float> Plane::intersect_ray(const vec3& ray_beg, const vec3& ray_dir) const {
    float div = dot - ray_beg.dot(normal);
    float rcp = ray_dir.dot(normal);
    if (div * rcp < 0.f)
        return std::nullopt;

    if (fabsf(rcp) < 1e-8f)
        return std::nullopt;

    return div / rcp;
}

inline float Plane::distance(const vec3& p) const
{
    return normal.dot(p) - dot;
}

} // slope
