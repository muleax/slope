#pragma once
#include "slope/collision/shape/collision_shape.hpp"

namespace slope {

class SphereShape : public CollisionShapeImpl<ShapeType::Sphere> {
public:
    explicit SphereShape(float radius) : m_radius(radius) {}

    Vec3 support_point(const Vec3& axis) const final
    {
        // TODO: consider getting rid of normalization
        return m_transform.translation() + axis.normalized() * m_radius;
    }

    void set_transform(const Mat44& matrix) final
    {
        m_transform = matrix;
        Vec3 delta = {m_radius, m_radius, m_radius};
        m_aabb.reset(matrix.translation() - delta, matrix.translation() + delta);
    }

    void set_radius(float value) { m_radius = value; }
    float radius() const { return m_radius; }

private:
    float m_radius = 1.f;
};

} // slope
