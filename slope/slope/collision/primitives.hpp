#pragma once
#include "slope/math/vector3.hpp"

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

    bool intersect_ray(float& out_t, const Vec3& ray_beg, const Vec3& ray_dir) const;

    float distance(const Vec3& p) const { return normal.dot(p) - dot; }
};

Vec3 find_orthogonal(const Vec3& normal);

inline void find_tangent(Vec3& out_u, Vec3& out_v, const Vec3& normal) {
    out_u = find_orthogonal(normal).normalized();
    out_v = out_u.cross(normal);
}

inline bool Plane::intersect_ray(float& out_t, const Vec3& ray_beg, const Vec3& ray_dir) const {
    float div = dot - ray_beg.dot(normal);
    float rcp = ray_dir.dot(normal);
    if (div * rcp < 0.f)
        return false;

    if (fabsf(rcp) < 1e-8f)
        return false;

    out_t = div / rcp;
    return true;
}

} // slope
