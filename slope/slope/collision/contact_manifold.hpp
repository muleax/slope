#pragma once
#include "slope/collision/primitives.hpp"
#include "slope/math/matrix44.hpp"
#include "slope/containers/array.hpp"

namespace slope {

struct ManifoldPoint {
    ContactGeom geom;

    int normal_constr_id = -1;
    int friction1_constr_id = -1;
    int friction2_constr_id = -1;

    float normal_lambda = 0.f;
    float friction1_lambda = 0.f;
    float friction2_lambda = 0.f;

    float penetration = 0.f;
    Vec3 local_point;
};

class ContactManifold {
public:
    static constexpr float CACHE_THRESHOLD = 0.02f;

    auto begin() { return m_points.begin(); }
    auto end() { return m_points.begin() + m_points_size; }

    void begin_update(uint32_t frame_id, const Mat44& shape1_inv_transform);
    void add_contact(const ContactGeom& new_geom);
    void end_update();

    bool is_active(uint32_t frame_id) const { return m_points_size > 0 && m_touch_frame_id == frame_id; }

private:
    static constexpr uint32_t MAX_SIZE = 4;

    struct PointLambdaCache {
        Vec3 local_point;
        float normal_lambda = 0.f;
        float friction1_lambda = 0.f;
        float friction2_lambda = 0.f;
    };

    Array<ManifoldPoint, MAX_SIZE> m_points;
    uint32_t m_points_size = 0;

    Array<PointLambdaCache, MAX_SIZE> m_lambda_cache;
    uint32_t m_lambda_cache_size = 0;

    uint32_t m_touch_frame_id = 0;
    Mat44 m_shape1_inv_transform;
};

} // slope
