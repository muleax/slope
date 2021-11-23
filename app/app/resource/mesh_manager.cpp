#include "app/resource/mesh_manager.hpp"
#include "slope/debug/log.hpp"
#include <fstream>
#include <sstream>

namespace slope::app {

MeshManager::MeshManager(RenderHandle instancing_buffer) : m_instancing_buffer(instancing_buffer) {}

std::shared_ptr<Mesh> MeshManager::load(const String& path) {
    return load_impl([&path]() {
        return nullptr;
    }, path);
}

bool MeshManager::save(const String& path, std::shared_ptr<Mesh> mesh) {
    return save_impl([&path, &mesh]() {
        return false;
    }, mesh, path);
}

} // slope::app