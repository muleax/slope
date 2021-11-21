#include "app/system/utils.hpp"
#define GLFW_INCLUDE_NONE
#include "GLFW/glfw3.h"

namespace slope {

double get_time() {
    return glfwGetTime();
}

} // slope
