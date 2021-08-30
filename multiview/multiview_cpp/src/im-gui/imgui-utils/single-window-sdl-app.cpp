
#include "single-window-sdl-app.hpp"

#include <SDL.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl.h>

#include <GL/glew.h>

#include "perceive/io/json-io.hpp"

#define This SingleWindowApp

namespace perceive::gui
{
// ---------------------------------------------------------- WindowPositionData
//
Json::Value WindowPositionData::to_json() const noexcept
{
   auto o        = Json::Value{Json::objectValue};
   o["position"] = json_save(position);
   o["size"]     = json_save(size);
   return o;
}

WindowPositionData
WindowPositionData::from_json(const Json::Value& node) noexcept(false)
{
   WindowPositionData x;
   const string_view op = "reading WindowPositionData";
   x.position           = json_load_key<Point2>(node, "position", op);
   x.size               = json_load_key<Point2>(node, "size", op);
   return x;
}

// ---------------------------------------------------- construction/destruction
//
This::This(SingleWindowParams p) noexcept(false)
{
   // Setup SDL
   if(SDL_Init(p.sdl_init_flags) != 0)
      throw std::runtime_error(format("SDL_Init error: {}\n", SDL_GetError()));

   // Decide GL+GLSL versions
   // GL 3.2 + GLSL 150
   SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
   SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,
                       SDL_GL_CONTEXT_PROFILE_CORE);
   SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
   SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);

   // Create window with graphics context
   SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
   SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
   SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

   this->sdl_window = SDL_CreateWindow(p.window_name.c_str(),
                                       p.pos.position.x,
                                       p.pos.position.y,
                                       p.pos.size.x,
                                       p.pos.size.y,
                                       p.window_flags);
   this->gl_context = SDL_GL_CreateContext(sdl_window);
   SDL_GL_MakeCurrent(sdl_window, gl_context);
   SDL_GL_SetSwapInterval(1); // Enable vsync

   // Initialize OpenGL loader
   if(glewInit() != GLEW_OK) {
      // Would be surprised if we could actually
      // safely delete the allocated resources above.
      // Requires knowledge of the underlying libraries, and them
      // being bug free.
      // So... kBAM!
      FATAL(format("glewInit() error"));
   }

   // Setup Dear ImGui context
   IMGUI_CHECKVERSION();
   ImGui::CreateContext();
   ImGuiIO& io = ImGui::GetIO();
   if(!p.imgui_ini_fname.empty()) io.IniFilename = p.imgui_ini_fname.c_str();

   // Setup Dear ImGui style
   ImGui::StyleColorsDark();
   // ImGui::StyleColorsClassic();

   // Setup Platform/Renderer bindings
   ImGui_ImplSDL2_InitForOpenGL(sdl_window, gl_context);
   ImGui_ImplOpenGL3_Init(p.glsl_version.c_str());
}

This::~This()
{
   // Cleanup
   ImGui_ImplOpenGL3_Shutdown();
   ImGui_ImplSDL2_Shutdown();
   ImGui::DestroyContext();
   SDL_GL_DeleteContext(gl_context);
   SDL_DestroyWindow(sdl_window);
   SDL_Quit();
}

// --------------------------------------------------------------------- Getters
//
uint32_t This::id() const noexcept // SDL window id
{
   return SDL_GetWindowID(sdl_window);
}

std::pair<int, int> This::size() const noexcept // width, height
{
   int w = 0, h = 0;
   SDL_GetWindowSize(sdl_window, &w, &h);
   return {w, h};
}

void This::gl_swap_window() const noexcept { SDL_GL_SwapWindow(sdl_window); }

} // namespace perceive::gui

#undef This
