#pragma once
#include "slope/collision/shape/collision_shape.hpp"
#include "slope/collision/primitives.hpp"

namespace slope {

class CapsuleShape : public CollisionShapeImpl<ShapeType::Capsule> {
public:
    CapsuleShape(float radius, float height) : m_radius(radius), m_height(height) {}

    Vec3 support_point(const Vec3& axis, float bloat) const final
    {
        int support_end = axis.dot(m_segment[0]) > axis.dot(m_segment[1]) ? 0 : 1;
        // TODO: consider getting rid of normalization
        return m_segment[support_end] + axis.normalized() * (m_radius + bloat);
    }

    void set_transform(const Mat44& matrix) final
    {
        m_transform = matrix;

        Vec3 offset = {0.f, m_height * 0.5f, 0.f};
        m_segment.begin = matrix.apply_point(-offset);
        m_segment.end = matrix.apply_point(offset);

        Vec3 delta = {m_radius, m_radius, m_radius};
        m_aabb.reset(m_segment.begin - delta, m_segment.begin + delta);
        m_aabb.extend(m_segment.end - delta);
        m_aabb.extend(m_segment.end + delta);
    }

    float               radius() const { return m_radius; }
    float               height() const { return m_height; }
    const LineSegment&  segment() const { return m_segment; }

private:
    float m_radius = 1.f;
    float m_height = 1.f;
    LineSegment m_segment;
};

} // slope
