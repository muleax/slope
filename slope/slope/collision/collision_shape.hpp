#pragma once
#include "slope/collision/aabb.hpp"
#include "slope/math/matrix44.hpp"

namespace slope {

enum class ShapeType : uint8_t {
    Undefined = 0,
    Box,
    ConvexPolyhedron,
    Sphere,
    Capsule,
    Count
};

class CollisionShape {
public:
    explicit CollisionShape(ShapeType shape_type) : m_shape_type(shape_type) {}
    virtual ~CollisionShape() = default;

    ShapeType       type() const { return m_shape_type; }
    const AABB&     aabb() const { return m_aabb; }
    const Mat44&    transform() const { return m_transform; }
    virtual void    set_transform(const Mat44& matrix) = 0;

    virtual Vec3    support_point(const Vec3& axis) const = 0;
    Vec3            support_diff(const CollisionShape* other, const Vec3& axis) const;

protected:
    Mat44 m_transform;
    AABB m_aabb;
    ShapeType m_shape_type = ShapeType::Undefined;
};


inline Vec3 CollisionShape::support_diff(const CollisionShape* other, const Vec3& axis) const
{
    return support_point(axis) - other->support_point(-axis);
}

class SphereShape : public CollisionShape {
public:
    explicit SphereShape(float radius) :  CollisionShape(ShapeType::Sphere), m_radius(radius) {}

    Vec3 support_point(const Vec3& axis) const final
    {
        // TODO: consider getting rid of normalization
        return m_transform.translation() + axis.normalized() * m_radius;
    }

    void set_transform(const Mat44& matrix) final
    {
        m_transform = matrix;
    }

    void set_radius(float value) { m_radius = value; }
    float radius() const { return m_radius; }

private:
    float m_radius = 1.f;
};

} // slope
