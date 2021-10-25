#include "app/resource/texture_manager.hpp"
#include "slope/debug/log.hpp"
#include "stb/stb_image.h"

namespace slope::app {

TextureManager::TexturePtr TextureManager::load(const String& path) {
    TexturePtr cached = get(path);
    if (cached) {
        return cached;
    }
/*
    std::ifstream stream(path);
    if (!stream) {
        log::error("Fail to load mesh {}", path);
        return nullptr;
    }
*/
    // TODO
    return nullptr;
}

bool TextureManager::save(const String& path, TexturePtr mesh) {
    return false;
}

} // slope::app