
#include "gl-error.hpp"

#include <GL/gl.h>

#ifdef USE_EGL
#include <EGL/egl.h>
#endif

namespace perceive
{
const char* gl_error_to_cstr(int err) noexcept
{
   switch(err) {
   case GL_NO_ERROR: return "GL_NO_ERROR";
   case GL_INVALID_ENUM: return "GL_INVALID_ENUM";
   case GL_INVALID_VALUE: return "GL_INVALID_VALUE";
   case GL_INVALID_OPERATION: return "GL_INVALID_OPERATION";
   case GL_STACK_OVERFLOW: return "GL_STACK_OVERFLOW";
   case GL_STACK_UNDERFLOW: return "GL_STACK_UNDERFLOW";
   case GL_OUT_OF_MEMORY: return "GL_OUT_OF_MEMORY";
   case GL_INVALID_FRAMEBUFFER_OPERATION:
      return "GL_INVALID_FRAMEBUFFER_OPERATION";
#ifdef GL_CONTEXT_LOST
   case GL_CONTEXT_LOST: return "GL_CONTEXT_LOST";
#endif
   case GL_TABLE_TOO_LARGE: return "GL_TABLE_TOO_LARGE";
#ifdef USE_EGL
   case EGL_SUCCESS: return "EGL_SUCCESS";
   case EGL_NOT_INITIALIZED: return "EGL_BAD_ACCESS";
   case EGL_BAD_ALLOC: return "EGL_BAD_ALLOC";
   case EGL_BAD_ATTRIBUTE: return "EGL_BAD_ATTRIBUTE";
   case EGL_BAD_CONTEXT: return "EGL_BAD_CONTEXT";
   case EGL_BAD_CONFIG: return "EGL_BAD_CONFIG";
   case EGL_BAD_CURRENT_SURFACE: return "EGL_BAD_CURRENT_SURFACE";
   case EGL_BAD_DISPLAY: return "EGL_BAD_DISPLAY";
   case EGL_BAD_SURFACE: return "EGL_BAD_SURFACE";
   case EGL_BAD_MATCH: return "EGL_BAD_MATCH";
   case EGL_BAD_PARAMETER: return "EGL_BAD_PARAMETER";
   case EGL_BAD_NATIVE_PIXMAP: return "EGL_BAD_NATIVE_PIXMAP";
   case EGL_BAD_NATIVE_WINDOW: return "EGL_BAD_NATIVE_WINDOW";
   case EGL_CONTEXT_LOST: return "EGL_CONTEXT_LOST";
#endif
   }
   return "unknown gl-error";
}

void check_and_throw_on_gl_error(const char* preamble) noexcept(false)
{
   auto err = glGetError();
   if(err != GL_NO_ERROR) {
      const auto m = gl_error_to_cstr(int(err));
      if(preamble)
         throw std::runtime_error(format("{:s} {:s}", preamble, m));
      else
         throw std::runtime_error(m);
   }
}

bool check_and_warn_on_gl_error(const char* preamble) noexcept
{
   auto err = glGetError();

   if(err != GL_NO_ERROR) {
      WARN(format("{:s} OpenGl reported error: {:s}",
                  (preamble == nullptr ? "" : preamble),
                  gl_error_to_cstr(int(err))));
      return false;
   }

   return true;
}

} // namespace perceive
