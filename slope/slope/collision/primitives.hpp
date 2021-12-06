#pragma once
#include "slope/math/vector3.hpp"
#include <optional>

namespace slope {

struct ContactGeom {
    Vec3 p1;
    Vec3 p2;
    Vec3 normal;
};

struct Plane {
    Vec3 normal;
    float dot;

    Plane(const Vec3& normal, float dot) : normal(normal), dot(dot) {}
    Plane(const Vec3& normal, const Vec3& point) : normal(normal), dot(normal.dot(point)) {}

    std::optional<float>    intersect_ray(const Vec3& ray_beg, const Vec3& ray_dir) const;
    float                   distance(const Vec3& p) const;
};

struct LineSegment {
    Vec3 begin;
    Vec3 end;

    const Vec3* data() const { return &begin; }
    Vec3* data() { return &begin; }

    const Vec3& operator [](int i) const { return data()[i]; }
    Vec3& operator [](int i) { return data()[i]; }

    void closest_point(const Vec3& other, float& t, Vec3& p) const;
    void closest_point(const LineSegment& other, float& t1, float& t2, Vec3& p1, Vec3& p2) const;
};

Vec3 find_orthogonal(const Vec3& axis);

inline void find_tangent(Vec3& out_u, Vec3& out_v, const Vec3& normal) {
    out_u = find_orthogonal(normal).normalized();
    out_v = out_u.cross(normal);
}

inline std::optional<float> Plane::intersect_ray(const Vec3& ray_beg, const Vec3& ray_dir) const {
    float div = dot - ray_beg.dot(normal);
    float rcp = ray_dir.dot(normal);
    if (div * rcp < 0.f)
        return std::nullopt;

    if (fabsf(rcp) < 1e-8f)
        return std::nullopt;

    return div / rcp;
}

inline float Plane::distance(const Vec3& p) const
{
    return normal.dot(p) - dot;
}

} // slope
