#pragma once
#include "app/render/texture.hpp"
#include "app/resource/resource_cache.hpp"

namespace slope::app {

class TextureManager : public ResourceCache<Texture> {
public:
    using TexturePtr = ResourceCache::ResourcePtr;

    TexturePtr  load(const String& path);
    bool        save(const String& path, TexturePtr mesh);
};

} // slope::app