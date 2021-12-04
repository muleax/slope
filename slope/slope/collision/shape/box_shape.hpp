#pragma once
#include "slope/collision/shape/collision_shape.hpp"
#include "slope/containers/array.hpp"
#include "slope/containers/vector.hpp"
#include <memory>

namespace slope {

class BoxShape : public CollisionShapeImpl<ShapeType::Box> {
public:
    explicit BoxShape(const Vec3& dimensions);

    void        set_transform(const Mat44& matrix) final;
    Vec3        support(const Vec3& axis, float bloat) const final { return support_impl(axis, bloat); }
    Vec3        support_normalized(const Vec3& axis, float bloat) const final { return support_impl(axis, bloat); }

    float       get_support_face(const Vec3& axis, Vector<Vec3>& out_support, Vec3& out_face_normal) const;
    Interval    project(const Vec3& axis) const;
    const auto& principal_face_axes() const { return m_principal_axes; };
    const auto& principal_edge_axes() const { return m_principal_axes; };

private:
    Vec3        support_impl(const Vec3& axis, float bloat) const;

    Vec3 m_half_dimensions;
    Array<Vec3, 3> m_extents;
    Array<Vec3, 3> m_principal_axes;
};

inline Interval BoxShape::project(const Vec3& axis) const
{
    float dot = 0.f;
    for (int i = 0; i < 3; i++)
        dot += fabs(axis.dot(m_extents[i]));

    auto pdot = axis.dot(m_transform.translation());
    return { pdot - dot, pdot + dot };
}

inline Vec3 BoxShape::support_impl(const Vec3& axis, float bloat) const
{
    Vec3 offs;
    for (int i = 0; i < 3; i++) {
        float dot = axis.dot(m_extents[i]);
        offs += m_extents[i] * signf(dot);
    }

    return m_transform.translation() + offs;
}

} // slope
