#pragma once
#include "app/ecs/system.hpp"

namespace slope::app {

class UIOverlaySystem : public System {
public:
    using System::System;
    void update(float dt) override;
};

} // slope::app
