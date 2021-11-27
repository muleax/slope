#include "slope/collision/narrowphase.hpp"
#include "slope/collision/primitives.hpp"

namespace slope {
/*
bool NarrowphaseImpl<>::intersect(const CollisionShape* shape1, const CollisionShape* shape2) {

}
*/

/*
void Narrowphase::generate_contacts(
    ContactManifold& manifold, const Vec3& pen_axis,
    const CollisionShape* shape1, const CollisionShape* shape2)
{
    switch (shape1->type()) {

    case ShapeType::ConvexPolyhedron:

        switch (shape2->type()) {

        case ShapeType::ConvexPolyhedron:
            generate_contacts_impl(
                manifold, pen_axis,
                static_cast<const ConvexPolyhedronShape*>(shape1), static_cast<const ConvexPolyhedronShape*>(shape2));
            break;

        case ShapeType::Sphere:
            generate_contacts_impl(
                manifold, pen_axis,
                static_cast<const ConvexPolyhedronShape*>(shape1), static_cast<const SphereShape*>(shape2));
            break;

        default:
            SL_VERIFY(false);
        }
        break;

    case ShapeType::Sphere:

        switch (shape2->type()) {

        case ShapeType::ConvexPolyhedron:
            manifold.invert_input_order();
            generate_contacts_impl(
                manifold, -pen_axis,
                static_cast<const ConvexPolyhedronShape*>(shape2), static_cast<const SphereShape*>(shape1));
            break;

        case ShapeType::Sphere:
            generate_contacts_impl(
                manifold, pen_axis,
                static_cast<const SphereShape*>(shape1), static_cast<const SphereShape*>(shape2));
            break;

        default:
            SL_VERIFY(false);
        }
        break;

    default:
        SL_VERIFY(false);
    }
}
*/
} // slope
