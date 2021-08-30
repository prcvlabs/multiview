
#pragma once

#include <SDL.h>

#include "json/json.h"

namespace perceive::gui
{
///
/// Serializable object that store window positioning
///
struct WindowPositionData
{
   Point2 position = {SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED};
   Point2 size     = {1280, 720}; // width/height

   Json::Value to_json() const noexcept;
   static WindowPositionData from_json(const Json::Value&) noexcept(false);
};

///
/// Parameter block for creating a `SingleWindowApp`
///
struct SingleWindowParams
{
   string glsl_version = "#version 150"s;

   uint32_t sdl_init_flags = SDL_INIT_VIDEO | SDL_INIT_TIMER;

   string window_name           = "Dear ImGui+SDL2+OpenGL3"s;
   WindowPositionData pos       = {};
   SDL_WindowFlags window_flags = SDL_WindowFlags(
       SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);

   string imgui_ini_fname = ""s; // uses default if empty
};

///
/// Initializes: SDL, OpenGL, Glew, ImGUI
/// The destructor cleans up (destroys) SDL, etc.
///
/// SDL uses a global event loop, (i.e., for all windows)
/// and thus it makes sense to handle events elsewhere.
///
struct SingleWindowApp
{
   // Members
   SDL_Window* sdl_window   = nullptr;
   SDL_GLContext gl_context = nullptr; // SDL_GLContext is an opaque pointer

   // Construction
   SingleWindowApp(SingleWindowParams p = {}) noexcept(false);
   SingleWindowApp(const SingleWindowApp&) = delete;
   SingleWindowApp(SingleWindowApp&&)      = default;
   ~SingleWindowApp();
   SingleWindowApp& operator=(const SingleWindowApp&) = delete;
   SingleWindowApp& operator=(SingleWindowApp&&) = default;

   uint32_t id() const noexcept;              // SDL_GetWindowID
   std::pair<int, int> size() const noexcept; // width, height
   void gl_swap_window() const noexcept;      // SDL_GL_SwapWindow
};

} // namespace perceive::gui
