#include "resource_manager.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"
#undef STB_IMAGE_IMPLEMENTATION

namespace slope {

ResourceManager ResourceManager::m_instance;

ResourceManager::ResourceManager()
{
}

bool ResourceManager::load_impl(const String& path, IResource& resource) {
    //std::ifstream stream(path);
    //if (!stream) {
    //    return false;
    //}

    if (!resource.load(path)) {
        return false;
    }

    return true;
}

} // slope