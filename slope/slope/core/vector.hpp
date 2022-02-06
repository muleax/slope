#pragma once
#include <vector>

namespace slope {

template <class T>
using Vector = std::vector<T>;

template <class T>
class VectorView {
public:
    using Iterator = typename Vector<T>::const_iterator;

    VectorView(const Vector<T>& vector)
        : m_begin(vector.begin()), m_end(vector.end()) {}

    VectorView(Iterator begin, Iterator end)
        : m_begin(begin), m_end(end) {}

    VectorView(Iterator begin, size_t size)
        : m_begin(begin), m_end(begin + size) {}

    const Iterator& begin() const { return m_begin; }
    const Iterator& end() const { return m_end; }

    size_t size() const { return m_end - m_begin; }

    const T* data() const { return &*m_begin; }
    //T* data() { return &*m_begin; }

    const T& operator[](size_t index) const { return *(m_begin + index); }

private:
    Iterator m_begin;
    Iterator m_end;
};

} // slope