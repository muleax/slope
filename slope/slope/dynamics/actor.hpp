#pragma once
#include "slope/dynamics/rigid_body.hpp"
#include "slope/collision/shape/collision_shape.hpp"
#include <memory>

namespace slope {

enum class ActorKind : int {
    Static,
    Dynamic,
    Count
};

class BaseActor {
public:
    explicit BaseActor(ActorKind kind) : m_kind(kind) {}
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

    ActorKind               kind() const { return m_kind; }

    template<class T>
    bool                    is() const;
    template<class T>
    const T*                cast() const;
    template<class T>
    T*                      cast();

protected:
    std::unique_ptr<CollisionShape> m_shape;
    float m_friction = 0.5f;
    ActorKind m_kind;
};

template <ActorKind T>
class TypedActor : public BaseActor {
public:
    static constexpr ActorKind Kind = T;
    TypedActor() : BaseActor(Kind) {}
};

class StaticActor : public TypedActor<ActorKind::Static> {
public:
    void            set_transform(const Mat44& transform) final;
    const Mat44&    transform() final { return m_transform; }
    const Mat44&    inv_transform() final { return m_inv_transform; }

private:
    Mat44 m_transform;
    Mat44 m_inv_transform;
};

class DynamicActor : public TypedActor<ActorKind::Dynamic> {
public:
    void                set_transform(const Mat44& transform) final;
    const Mat44&        transform() final { return m_body.transform(); }
    const Mat44&        inv_transform() final { return m_body.inv_transform(); }

    const RigidBody&    body() const { return m_body; }
    RigidBody&          body() { return m_body; }

private:
    RigidBody m_body;
};

template<class T>
bool BaseActor::is() const
{
    static_assert(std::is_base_of_v<TypedActor<T::Kind>, T>);
    return m_kind == T::Kind;
}

template<class T>
const T* BaseActor::cast() const
{
    SL_ASSERT(is<T>());
    return static_cast<const T*>(this);
}

template<class T>
T* BaseActor::cast()
{
    SL_ASSERT(is<T>());
    return static_cast<T*>(this);
}

inline void BaseActor::set_shape(std::unique_ptr<CollisionShape>&& shape) {
    m_shape = std::move(shape);
    m_shape->set_transform(transform());
}

template<class T, class... Args>
void BaseActor::set_shape(Args&&... args) {
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
