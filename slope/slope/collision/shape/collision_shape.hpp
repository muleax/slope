#pragma once
#include "slope/collision/aabb.hpp"
#include "slope/math/matrix44.hpp"
#include "slope/core/typed_object.hpp"

namespace slope {

enum class ShapeKind : int {
    Polyhedron,
    Box,
    Sphere,
    Capsule,
    Count
};

class CollisionShape : public TypedBase<CollisionShape, ShapeKind> {
public:
    using TypedBase::TypedBase;
    virtual ~CollisionShape() = default;

    virtual void    set_transform(const mat44& matrix) = 0;
    virtual vec3    support(const vec3& axis, float bloat, bool normalized) const = 0;

    const mat44&    transform() const { return m_transform; }
    const AABB&     aabb() const { return m_aabb; }
    vec3            support_diff(const CollisionShape* other, const vec3& axis, float bloat, bool normalized) const;

protected:
    mat44 m_transform;
    AABB m_aabb;
    ShapeKind m_kind;
};

inline vec3 CollisionShape::support_diff(const CollisionShape* other, const vec3& axis, float bloat, bool normalized) const
{
    return support(axis, bloat, normalized) - other->support(-axis, bloat, normalized);
}

} // slope
