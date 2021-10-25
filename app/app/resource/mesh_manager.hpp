#pragma once
#include "app/render/mesh.hpp"
#include "app/resource/resource_cache.hpp"

namespace slope::app {

class MeshManager : public ResourceCache<Mesh> {
public:
    MeshManager(RenderHandle instancing_buffer);

    MeshPtr load(const String& path);
    bool    save(const String& path, MeshPtr mesh);

private:
    RenderHandle m_instancing_buffer;
};

} // slope::app