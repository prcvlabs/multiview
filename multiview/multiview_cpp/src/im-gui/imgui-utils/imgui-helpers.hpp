
#pragma once

#include "perceive/utils/concepts.hpp"

#include <GL/glew.h>

#include <GL/gl.h>
#include <SDL.h>
#include <imgui.h>

#include <string_view>

namespace perceive::imgui
{
template<std::arithmetic T> ImVec2 to_im_vec2(T w, T h) noexcept
{
   return ImVec2(float(w), float(h));
}

template<std::arithmetic T> ImVec2 to_im_vec2(std::pair<T, T> wh) noexcept
{
   return to_im_vec2(wh.first, wh.second);
}

template<typename... Param>
static void Text(std::string_view fmt, Param&&... params)
{
   ImGui::TextUnformatted(format(fmt, std::forward<Param>(params)...).c_str());
}

inline void* convert_texture(GLuint x) noexcept
{
   return reinterpret_cast<void*>(uintptr_t(x));
}

} // namespace perceive::imgui
