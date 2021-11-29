#pragma once
#include "slope/dynamics/rigid_body.hpp"
#include "slope/collision/shape/collision_shape.hpp"
#include <memory>

namespace slope {

class BaseActor {
public:
    explicit BaseActor(int kind) : m_kind(kind) {}
    virtual ~BaseActor() = default;

    void                    set_shape(std::unique_ptr<CollisionShape>&& shape);

    template<class T, class... Args>
    void                    set_shape(Args&&... args);

    bool                    has_shape() const { return m_shape != nullptr; }
    const CollisionShape&   shape() const { return *m_shape; }
    CollisionShape&         shape() { return *m_shape; }

    void                    set_friction(float value) { m_friction = value; }
    float                   friction() const { return m_friction; }

    virtual void            set_transform(const Mat44& transform) = 0;
    virtual const           Mat44& transform() = 0;
    virtual const           Mat44& inv_transform() = 0;

    void                    set_broadphase_proxy_id(int value) { m_broadphase_proxy_id = value; }
    int                     broadphase_proxy_id() const { return m_broadphase_proxy_id; }

    template<class T>
    bool                    is() const;
    int                     kind() const { return m_kind; }

protected:
    std::unique_ptr<CollisionShape> m_shape;
    float m_friction = 0.5f;
    int m_broadphase_proxy_id = -1;
    int m_kind = -1;
};

class StaticActor : public BaseActor {
public:
    static constexpr int Kind = 0;

    StaticActor() : BaseActor(Kind) {}

    void            set_transform(const Mat44& transform) final;
    const Mat44&    transform() final { return m_transform; }
    const Mat44&    inv_transform() final { return m_inv_transform; }

private:
    Mat44 m_transform;
    Mat44 m_inv_transform;
};

class DynamicActor : public BaseActor {
public:
    static constexpr int Kind = 1;

    DynamicActor() : BaseActor(Kind) {}

    void                set_transform(const Mat44& transform) final;
    const Mat44&        transform() final { return m_body.transform(); }
    const Mat44&        inv_transform() final { return m_body.inv_transform(); }

    const RigidBody&    body() const { return m_body; }
    RigidBody&          body() { return m_body; }

private:
    RigidBody m_body;
};

template<class T>
inline bool BaseActor::is() const {
    static_assert(std::is_base_of_v<BaseActor, T>);
    return m_kind == T::Kind;
}

inline void BaseActor::set_shape(std::unique_ptr<CollisionShape>&& shape) {
    m_shape = std::move(shape);
    m_shape->set_transform(transform());
}

template<class T, class... Args>
inline void BaseActor::set_shape(Args&&... args) {
    set_shape(std::make_unique<T>(std::forward<Args>(args)...));
}

inline void StaticActor::set_transform(const Mat44& transform) {
    m_transform = transform;
    m_inv_transform = transform.inverted_orthonormal();

    if (m_shape)
        m_shape->set_transform(transform);
}

inline void DynamicActor::set_transform(const Mat44& transform) {
    m_body.set_transform(transform);

    if (m_shape)
        m_shape->set_transform(transform);
}

} // slope
