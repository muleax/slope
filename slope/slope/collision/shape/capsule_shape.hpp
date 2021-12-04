#pragma once
#include "slope/collision/shape/collision_shape.hpp"
#include "slope/collision/primitives.hpp"

namespace slope {

class CapsuleShape : public CollisionShapeImpl<ShapeType::Capsule> {
public:
    CapsuleShape(float radius, float height);

    Vec3                support_point(const Vec3& axis, float bloat) const final;
    void                set_transform(const Mat44& matrix) final;

    float               radius() const { return m_radius; }
    float               height() const { return m_height; }
    const LineSegment&  segment() const { return m_segment; }

private:
    float m_radius = 1.f;
    float m_height = 1.f;
    LineSegment m_segment;
};

inline Vec3 CapsuleShape::support_point(const Vec3& axis, float bloat) const
{
    int support_end = axis.dot(m_segment[0]) > axis.dot(m_segment[1]) ? 0 : 1;
    // TODO: consider getting rid of normalization
    return m_segment[support_end] + axis.normalized() * (m_radius + bloat);
}

} // slope
