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

    auto        begin() { return m_points.begin(); }
    auto        end() { return m_points.begin() + m_point_count; }

    uint32_t    size() const { return m_point_count; }

    void        update_inv_transform(const Mat44& shape1_inv_transform);

    void        begin_update();
    void        end_update();
    void        add_contact(ContactGeom new_geom);
    void        invert_input_order() { m_inverted_input_order = !m_inverted_input_order; };

private:
    static constexpr uint32_t MAX_SIZE = 4;

    struct PointLambdaCache {
        Vec3 local_point;
        float normal_lambda = 0.f;
        float friction1_lambda = 0.f;
        float friction2_lambda = 0.f;
    };

    Array<ManifoldPoint, MAX_SIZE> m_points;
    uint32_t m_point_count = 0;

    Array<PointLambdaCache, MAX_SIZE> m_lambda_cache;
    uint32_t m_lambda_cache_size = 0;

    bool m_inverted_input_order = false;
    Mat44 m_shape1_inv_transform;
};

inline void ContactManifold::update_inv_transform(const Mat44& shape1_inv_transform)
{
    m_shape1_inv_transform = shape1_inv_transform;
}

} // slope
