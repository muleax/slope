#pragma once
#include "slope/collision/aabb.hpp"
#include "slope/math/matrix44.hpp"

namespace slope {

enum class ShapeType : uint8_t {
    Undefined = 0,
    Box,
    ConvexPolyhedron,
    Count
};

class CollisionShape {
public:
    explicit CollisionShape(ShapeType shape_type) : m_shape_type(shape_type) {}
    virtual ~CollisionShape() = default;

    ShapeType       type() const { return m_shape_type; }
    const AABB&     aabb() const { return m_aabb; }
    const Mat44&    transform() const { return m_transform; }
    virtual void    set_transform(const Mat44& value) = 0;

protected:
    Mat44 m_transform;
    AABB m_aabb;
    ShapeType m_shape_type = ShapeType::Undefined;
};

} // slope
