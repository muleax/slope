#pragma once
#include "slope/dynamics/rigid_body.hpp"
#include "slope/collision/shape/collision_shape.hpp"
#include "slope/collision/broadphase/broadphase.hpp"
#include "slope/core/typed_object.hpp"
#include <memory>

namespace slope {

enum class ActorKind : int {
    Static,
    Dynamic,
    Count
};

class BaseActor : public TypedBase<BaseActor, ActorKind> {
public:
    using LinearId = uint32_t;

    using TypedBase::TypedBase;
    virtual ~BaseActor() = default;

    bool                    has_shape() const { return m_shape != nullptr; }
    const CollisionShape&   shape() const { return *m_shape; }
    CollisionShape&         shape() { return *m_shape; }

    void                    set_friction(float value) { m_friction = value; }
    float                   friction() const { return m_friction; }

    void                    set_restitution(float value) { m_restitution = value; }
    float                   restitution() const { return m_restitution; }

    virtual void            set_transform(const mat44& transform) = 0;
    virtual const           mat44& transform() = 0;
    virtual const           mat44& inv_transform() = 0;

protected:
    // Interface for DynamicsWorld
    LinearId        linear_id() const { return m_linear_id; }
    void            set_linear_id(LinearId id) { m_linear_id = id; }
    ProxyId         proxy_id() const { return m_proxy_id; }
    void            set_proxy_id(ProxyId id) { m_proxy_id = id; }
    void            assign_shape(CollisionShape* shape);

    // TODO: reconsider using friend
    friend class DynamicsWorld;

    CollisionShape* m_shape = nullptr;
    float           m_friction = 0.5f;
    float           m_restitution = 0.f;

    LinearId        m_linear_id = 0;
    ProxyId         m_proxy_id = 0;
};

class StaticActor : public TypedObject<BaseActor, ActorKind::Static> {
public:
    void            set_transform(const mat44& transform) final;
    const mat44&    transform() final { return m_transform; }
    const mat44&    inv_transform() final { return m_inv_transform; }

private:
    mat44 m_transform;
    mat44 m_inv_transform;
};

class DynamicActor : public TypedObject<BaseActor, ActorKind::Dynamic> {
public:
    void                set_transform(const mat44& transform) final;
    const mat44&        transform() final { return m_body.transform(); }
    const mat44&        inv_transform() final { return m_body.inv_transform(); }

    const RigidBody&    body() const { return m_body; }
    RigidBody&          body() { return m_body; }

private:
    RigidBody m_body;
};

inline void BaseActor::assign_shape(CollisionShape* shape) {
    m_shape = shape;
    m_shape->set_transform(transform());
}

inline void StaticActor::set_transform(const mat44& transform) {
    m_transform = transform;
    m_inv_transform = transform.inverted_orthonormal();

    if (m_shape)
        m_shape->set_transform(transform);
}

inline void DynamicActor::set_transform(const mat44& transform) {
    m_body.set_transform(transform);

    if (m_shape)
        m_shape->set_transform(transform);
}

} // slope
