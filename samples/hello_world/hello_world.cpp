#include "app/system/app.hpp"
#include "app/system/resource_manager.hpp"
#include "app/render/mesh.hpp"

int main()
{
    auto& m = slope::ResourceManager::instance();
    auto ret = m.acquire<slope::MeshResource>("resources/brick.dae");

    slope::App().run(640, 480, "Hello world");
    return 0;
}
