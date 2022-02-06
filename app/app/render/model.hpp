#pragma once
#include "app/render/mesh.hpp"
#include "slope/core/vector.hpp"
#include <atomic>

namespace slope::app {

class Model {
public:
    using ModelId = uint32_t;
    static constexpr ModelId INVALID_ID = std::numeric_limits<ModelId>::max();

    explicit Model(Mesh&& mesh);

    ModelId id() const { return m_id; }
    const Mesh& mesh() const { return m_mesh; }

private:
    ModelId m_id = INVALID_ID;
    Mesh m_mesh;

    static std::atomic<ModelId> s_id_counter;
};

using ModelPtr = std::shared_ptr<Model>;

} // slope::app