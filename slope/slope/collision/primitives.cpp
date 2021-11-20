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

} // slope
