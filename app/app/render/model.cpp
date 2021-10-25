#include "app/render/model.hpp"

namespace slope::app {

std::atomic<Model::ModelId> Model::s_id_counter = 0;

Model::Model(Mesh&& mesh)
    : m_id(s_id_counter.fetch_add(1))
    , m_mesh(std::move(mesh))
{
}

} // slope::app
