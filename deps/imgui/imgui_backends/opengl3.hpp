#pragma once
#include "imgui/imgui.h"

// Interface definition for standard opengl3 backend (imgui/backends/imgui_impl_opengl3.cpp).
// Custom header is required to support Slope's third-party include convention.

IMGUI_IMPL_API bool ImGui_ImplOpenGL3_Init(const char* glsl_version = nullptr);
IMGUI_IMPL_API void ImGui_ImplOpenGL3_Shutdown();
IMGUI_IMPL_API void ImGui_ImplOpenGL3_NewFrame();
IMGUI_IMPL_API void ImGui_ImplOpenGL3_RenderDrawData(ImDrawData* draw_data);
