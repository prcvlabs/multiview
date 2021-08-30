
#ifndef USE_EGL
#ifndef USE_GLX
#define USE_GLX
#endif
#endif

#include "../gl-utils.hpp"
#include "platform.hpp"

#include <GL/glew.h>

#include <GL/gl.h>

#ifdef USE_EGL
#include <EGL/egl.h>
#endif

#ifdef USE_GLX
#include <GL/glx.h>
#include <GL/glxext.h>
#include <X11/X.h>
#include <X11/Xlib.h>
#endif

#if defined USE_EGL && defined USE_GLX
#error "cannot define USE_GLX and USE_EGL at the same time"
#endif

namespace perceive
{
// ------------------------------------------------- Gl Setup Windowless Context

#ifdef USE_EGL
void gl_setup_windowless_context()
{
   // -- (1) -- Initialize EGL
   EGLDisplay egl_dpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
   if(egl_dpy == EGL_NO_DISPLAY) FATAL("egldpy == EGL_NO_DISPLAY");

   EGLint major, minor;
   if(eglInitialize(egl_dpy, &major, &minor) == EGL_FALSE) {
      FATAL(format("eglInitialize: {:s}", gl_error_to_cstr(glGetError())));
   } else {
      INFO(format("egl version {}.{}\n", major, minor));
   }

   // -- (2) -- Select an appropriate configuration
   EGLint num_confs;
   EGLConfig egl_cfg;

   static const EGLint conf_attrs[] = {EGL_SURFACE_TYPE,
                                       EGL_PBUFFER_BIT,
                                       EGL_RENDERABLE_TYPE,
                                       EGL_OPENGL_BIT,
                                       EGL_BLUE_SIZE,
                                       8,
                                       EGL_GREEN_SIZE,
                                       8,
                                       EGL_RED_SIZE,
                                       8,
                                       EGL_ALPHA_SIZE,
                                       8,
                                       EGL_DEPTH_SIZE,
                                       24,
                                       EGL_NONE};

   if(eglChooseConfig(egl_dpy, conf_attrs, &egl_cfg, 1, &num_confs)
      == EGL_FALSE) {
      FATAL(format("eglChooseConfig: {:s}", gl_error_to_cstr(glGetError())));
   }

   // -- (3) -- Create a surface
   static const EGLint pbuf_attrs[] = {// EGL_LARGEST_PBUFFER, EGL_NONE
                                       EGL_WIDTH,
                                       1024,
                                       EGL_HEIGHT,
                                       1024,
                                       EGL_NONE};

   EGLSurface egl_surf = eglCreatePbufferSurface(egl_dpy, egl_cfg, pbuf_attrs);
   if(egl_surf == EGL_NO_SURFACE) {
      auto err = eglGetError();
      if(err != EGL_SUCCESS)
         FATAL(format("eglCreatePbufferSurface: {:s}", gl_error_to_cstr(err)));
   }

   // -- (4) -- Bind the API
   eglBindAPI(EGL_OPENGL_API);
   {
      auto err = eglGetError();
      if(err != EGL_SUCCESS)
         FATAL(format("eglBindAPI: {:s}", gl_error_to_cstr(err)));
   }

   // -- (5) -- Create context
   static const EGLint ctx_attrs[]
       = {EGL_CONTEXT_MAJOR_VERSION, 4, EGL_CONTEXT_MINOR_VERSION, 5, EGL_NONE};

   auto egl_ctx = eglCreateContext(egl_dpy, egl_cfg, EGL_NO_CONTEXT, ctx_attrs);
   {
      auto err = eglGetError();
      if(err != EGL_SUCCESS)
         FATAL(format("eglCreateContext: {:s}", gl_error_to_cstr(err)));
   }

   // -- (6) -- Make context current
   if(eglMakeCurrent(egl_dpy, egl_surf, egl_surf, egl_ctx) == EGL_FALSE) {
      auto err = eglGetError();
      if(err != EGL_SUCCESS)
         FATAL(format("eglMakeCurrent: {:s}", gl_error_to_cstr(err)));
   }
}
#endif

#ifdef USE_GLX

void gl_setup_windowless_context()
{
   Display* dpy{nullptr};
   Window root;
   GLint attr[] = {GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None};
   XVisualInfo* vi{nullptr};
   GLXContext glc;

   // open display
   if(not(dpy = XOpenDisplay(NULL))) FATAL("cannot connect to X server");

   // get root window
   root = DefaultRootWindow(dpy);

   // get visual matching attr
   if(not(vi = glXChooseVisual(dpy, 0, attr)))
      FATAL("no appropriate visual found");

   // create a context using the root window
   if(not(glc = glXCreateContext(dpy, vi, NULL, GL_TRUE)))
      FATAL("failed to create context");

   glXMakeCurrent(dpy, root, glc);
}

void gl_setup_windowless_context_B()
{
   Display* disp{nullptr};

   // open display
   if(not(disp = XOpenDisplay(nullptr))) FATAL("cannot connect to X server");

   // get root window
   Window root = DefaultRootWindow(disp);

   static int visual_attribs[] = {GLX_RENDER_TYPE,
                                  GLX_RGBA_BIT,
                                  GLX_DOUBLEBUFFER,
                                  true,
                                  GLX_DEPTH_SIZE,
                                  24,
                                  GLX_RED_SIZE,
                                  1,
                                  GLX_GREEN_SIZE,
                                  1,
                                  GLX_BLUE_SIZE,
                                  1,
                                  None};

   int num_fbc = 0;
   GLXFBConfig* fbc
       = glXChooseFBConfig(disp, DefaultScreen(disp), visual_attribs, &num_fbc);
   if(!fbc) FATAL("glXChooseFBConfig() failed\n");

   // Create old OpenGL context to get correct function pointer for
   // glXCreateContextAttribsARB()
   typedef GLXContext (*glXCreateContextAttribsARBProc)(
       Display*, GLXFBConfig, GLXContext, Bool, const int*);
   XVisualInfo* vi    = glXGetVisualFromFBConfig(disp, fbc[0]);
   GLXContext ctx_old = glXCreateContext(disp, vi, nullptr, GL_TRUE);
   glXCreateContextAttribsARBProc glXCreateContextAttribsARB = 0;
   glXCreateContextAttribsARB
       = reinterpret_cast<glXCreateContextAttribsARBProc>(glXGetProcAddress(
           reinterpret_cast<const GLubyte*>("glXCreateContextAttribsARB")));
   // Destroy old context
   glXMakeCurrent(disp, 0, 0);
   glXDestroyContext(disp, ctx_old);
   if(!glXCreateContextAttribsARB)
      FATAL("glXCreateContextAttribsARB() not found");

   // Set desired minimum OpenGL version
   int context_attribs[] = {GLX_CONTEXT_MAJOR_VERSION_ARB,
                            3,
                            GLX_CONTEXT_MINOR_VERSION_ARB,
                            1,
                            GLX_CONTEXT_FLAGS_ARB,
                            GLX_CONTEXT_DEBUG_BIT_ARB,
                            GLX_CONTEXT_PROFILE_MASK_ARB,
                            GLX_CONTEXT_COMPATIBILITY_PROFILE_BIT_ARB,
                            None};

   // Create modern OpenGL context
   GLXContext glc
       = glXCreateContextAttribsARB(disp, fbc[0], NULL, true, context_attribs);
   if(!glc) FATAL("Failed to create OpenGL context. Exiting.");

   glXMakeCurrent(disp, root, glc);
}
#endif

// ------------------------------------------------------------------- Glew Init

void glew_init()
{
   static bool is_init{false};
   if(!is_init) {
      glewExperimental = GL_TRUE;
      glewInit();
      is_init = true;
      try {
         check_and_throw_on_gl_error(); // glew screws up glError
      } catch(...) {}
   } else {
      LOG_ERR(format("there has been multiple calls to 'glew-init'"));
   }
}

// ---------------------------------------------------- Ensure Glew and Gl Setup

void ensure_glew_and_gl_setup()
{
   static std::once_flag flag1;
   std::call_once(flag1, []() {
      gl_setup_windowless_context();
      glew_init();
   });
}

} // namespace perceive
