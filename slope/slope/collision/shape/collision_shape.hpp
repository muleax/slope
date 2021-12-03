#pragma once
#include "slope/collision/aabb.hpp"
#include "slope/math/matrix44.hpp"

namespace slope {

enum class ShapeType : int {
    Undefined = 0,
    ConvexPolyhedron,
    Box,
    Sphere,
    Capsule,
    Count
};


class CollisionShape {
public:
    explicit CollisionShape(ShapeType type) : m_type(type) {}
    virtual ~CollisionShape() = default;

    const AABB&     aabb() const { return m_aabb; }
    const Mat44&    transform() const { return m_transform; }
    virtual void    set_transform(const Mat44& matrix) = 0;

    virtual Vec3    support_point(const Vec3& axis, float bloat) const = 0;
    Vec3            support_diff(const CollisionShape* other, const Vec3& axis, float bloat) const;

    ShapeType       type() const { return m_type; }

protected:
    Mat44 m_transform;
    AABB m_aabb;
    ShapeType m_type;
};

template <ShapeType T>
class CollisionShapeImpl : public CollisionShape {
public:
    static constexpr ShapeType Type = T;

    CollisionShapeImpl() : CollisionShape(T) {}
};

inline Vec3 CollisionShape::support_diff(const CollisionShape* other, const Vec3& axis, float bloat) const
{
    return support_point(axis, bloat) - other->support_point(-axis, bloat);
}

} // slope
