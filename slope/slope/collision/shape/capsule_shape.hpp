#pragma once
#include "slope/collision/shape/collision_shape.hpp"
#include "slope/collision/primitives.hpp"

namespace slope {

class CapsuleShape : public CollisionShapeImpl<ShapeType::Capsule> {
public:
    CapsuleShape(float radius, float height);

    Vec3        support(const Vec3& axis, float bloat, bool normalized) const final;
    void        set_transform(const Mat44& matrix) final;

    float       radius() const { return m_radius; }
    float       height() const { return m_height; }
    const auto& segment() const { return m_segment; }

private:
    float m_radius = 1.f;
    float m_height = 1.f;
    LineSegment m_segment;
};

inline Vec3 CapsuleShape::support(const Vec3& axis, float bloat, bool normalized) const
{
    Vec3 norm_axis = normalized ? axis : axis.normalized();
    int support_end = axis.dot(m_segment[0]) > axis.dot(m_segment[1]) ? 0 : 1;
    return m_segment[support_end] + norm_axis * (m_radius + bloat);
}

} // slope
