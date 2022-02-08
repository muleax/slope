#pragma once
#include "slope/debug/assert.hpp"

namespace slope {

template <class Impl, class Kind>
class TypedBase {
public:
    using TKind = Kind;

    explicit TypedBase(Kind kind) : m_kind(kind) {}

    Kind kind() const { return m_kind; }

    template<class T>
    bool is() const
    {
        static_assert(std::is_base_of_v<TypedBase<Impl, typename T::TKind>, T>);
        return m_kind == T::s_kind();
    }

    template<class T>
    const T* cast() const
    {
        SL_ASSERT(is<T>());
        return static_cast<const T*>(this);
    }

    template<class T>
    T* cast()
    {
        SL_ASSERT(is<T>());
        return static_cast<T*>(this);
    }

private:
    Kind m_kind;
};

template <class Base, typename Base::TKind K>
class TypedObject : public Base {
public:
    static_assert(std::is_base_of_v<TypedBase<Base, typename Base::TKind>, Base>);

    static constexpr typename Base::TKind s_kind() { return K; }

    TypedObject() : Base(K) {}
};

} // slope
