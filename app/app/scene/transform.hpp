#pragma once
#include "app/ecs/component.hpp"
#include "slope/math/matrix44.hpp"

namespace slope::app {

struct TransformComponent : public Component<TransformComponent> {
    Mat44 transform;
};

} // slope::app