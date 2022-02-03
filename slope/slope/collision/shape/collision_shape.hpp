#pragma once
#include "slope/collision/aabb.hpp"
#include "slope/math/matrix44.hpp"

namespace slope {

enum class ShapeKind : int {
    Polyhedron,
    Box,
    Sphere,
    Capsule,
    Count
};

class CollisionShape {
public:
    explicit CollisionShape(ShapeKind kind) : m_kind(kind) {}
    virtual ~CollisionShape() = default;

    virtual void    set_transform(const mat44& matrix) = 0;
    virtual vec3    support(const vec3& axis, float bloat, bool normalized) const = 0;

    const mat44&    transform() const { return m_transform; }
    const AABB&     aabb() const { return m_aabb; }
    vec3            support_diff(const CollisionShape* other, const vec3& axis, float bloat, bool normalized) const;

    ShapeKind       kind() const { return m_kind; }

    template<class T>
    bool            is() const;
    template<class T>
    const T*        cast() const;
    template<class T>
    T*              cast();

protected:
    mat44 m_transform;
    AABB m_aabb;
    ShapeKind m_kind;
};

template <ShapeKind T>
class TypedCollisionShape : public CollisionShape {
public:
    static constexpr ShapeKind Kind = T;
    TypedCollisionShape() : CollisionShape(Kind) {}
};

inline vec3 CollisionShape::support_diff(const CollisionShape* other, const vec3& axis, float bloat, bool normalized) const
{
    return support(axis, bloat, normalized) - other->support(-axis, bloat, normalized);
}
template<class T>
bool CollisionShape::is() const
{
    static_assert(std::is_base_of_v<TypedCollisionShape<T::Kind>, T>);
    return m_kind == T::Kind;
}

template<class T>
const T* CollisionShape::cast() const
{
    SL_ASSERT(is<T>());
    return static_cast<const T*>(this);
}

template<class T>
T* CollisionShape::cast()
{
    SL_ASSERT(is<T>());
    return static_cast<T*>(this);
}

} // slope
