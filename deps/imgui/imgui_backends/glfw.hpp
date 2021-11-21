#pragma once
#include "imgui/imgui.h"

// Interface definition for standard GLFW backend (imgui/backends/imgui_impl_glfw.cpp)
// Custom header is required to support Slope's third-party include convention.

struct GLFWwindow;
struct GLFWmonitor;

IMGUI_IMPL_API bool ImGui_ImplGlfw_InitForOpenGL(GLFWwindow* window, bool install_callbacks);
IMGUI_IMPL_API bool ImGui_ImplGlfw_InitForOther(GLFWwindow* window, bool install_callbacks);
IMGUI_IMPL_API void ImGui_ImplGlfw_Shutdown();
IMGUI_IMPL_API void ImGui_ImplGlfw_NewFrame();

IMGUI_IMPL_API void ImGui_ImplGlfw_WindowFocusCallback(GLFWwindow* window, int focused);
IMGUI_IMPL_API void ImGui_ImplGlfw_CursorEnterCallback(GLFWwindow* window, int entered);
IMGUI_IMPL_API void ImGui_ImplGlfw_MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
IMGUI_IMPL_API void ImGui_ImplGlfw_ScrollCallback(GLFWwindow* window, double xoffset, double yoffset);
IMGUI_IMPL_API void ImGui_ImplGlfw_KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
IMGUI_IMPL_API void ImGui_ImplGlfw_CharCallback(GLFWwindow* window, unsigned int c);
IMGUI_IMPL_API void ImGui_ImplGlfw_MonitorCallback(GLFWmonitor* monitor, int event);
