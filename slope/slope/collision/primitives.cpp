#include "slope/collision/primitives.hpp"

namespace slope {

Vec3 find_orthogonal(const Vec3& normal) {
    int max_idx = 0;
    if (fabs(normal.data[1]) > fabs(normal.data[0]))
        max_idx = 1;
    if (fabs(normal.data[2]) > fabs(normal.data[max_idx]))
        max_idx = 2;

    int next_idx = (max_idx + 1) % 3;
    int prev_idx = (max_idx + 2) % 3;

    Vec3 ort;
    ort.data[max_idx] = -normal.data[next_idx];
    ort[next_idx] = normal.data[max_idx];
    ort[prev_idx] = 0.f;
    return ort;
}

float LineSegment::closest_point(const LineSegment& other, float& t1, float& t2, Vec3& p1, Vec3& p2) const
{
    // TODO: optimize
    constexpr float EPSILON = 1e-6f;
    auto d1 = end - begin;
    auto d2 = other.end - other.begin;
    auto r = begin - other.begin;
    float a = d1.dot(d1);
    float e = d2.dot(d2);
    float f = d2.dot(r);

    if (a <= EPSILON && e <= EPSILON) {
        t1 = t2 = 0.f;
        p1 = begin;
        p2 = other.begin;
        return (p1 - p2).length_squared();
    }

    if (a <= EPSILON) {
        t1 = 0.f;
        t2 = clamp(0.f, f / e, 1.f);
    } else {
        float c = d1.dot(r);
        if (e <= EPSILON) {
            t2 = 0.f;
            t1 = clamp(0.f, -c / a, 1.f);
        } else {
            float b = d1.dot(d2);
            float denom = a*e - b*b;

            if (denom != 0.f) {
                t1 = clamp(0.f, (b*f - c*e) / denom, 1.f);
            } else {
                t1 = 0.f;
            }

            t2 = (b*t1 + f) / e;

            if (t2 < 0.f) {
                t2 = 0.f;
                t1 = clamp(0.f, -c/a, 1.f);
            } else if (t2 > 1.f) {
                t2 = 1.f;
                t1 = clamp(0.f, (b-c)/a, 1.f);
            }
        }
    }

    p1 = begin + d1 * t1;
    p2 = other.begin + d2 * t2;
    return (p1 - p2).length_squared();
}

} // slope
