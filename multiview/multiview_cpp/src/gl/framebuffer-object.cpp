
#include <stdexcept>

#include "gl/platform/glew-bridge.hpp"

#include "framebuffer-object.hpp"
#include "gl-error.hpp"

#define This FramebufferObject

// These are the functions (and signatures) required from Qt
//
// void glGenFramebuffers         (GLsizei n, GLuint *ids);
// void glBindFramebuffer         (GLenum target,GLuint framebuffer);
// void glDeleteFramebuffers      (GLsizei n,  const GLuint * framebuffers);
//
// void glGenRenderbuffers        (GLsizei n,  GLuint * renderbuffers);
// void glBindRenderbuffer        (GLenum target,  GLuint renderbuffer);
// void glDeleteRenderbuffers     (GLsizei n,  const GLuint * renderbuffers);
//
// void glRenderbufferStorage     (GLenum target,  GLenum internalformat,
//                                 GLsizei width,  GLsizei height);
// void glFramebufferRenderbuffer (GLenum target,  GLenum attachment,
//                                 GLenum renderbuffertarget,
//                                 GLuint renderbuffer);
// void glDrawBuffers             (GLsizei n, const GLenum *bufs);
// GLenum glCheckFramebufferStatus(GLenum target);

namespace perceive
{
// --------------------------------------------------------------- Move Operator

FramebufferObject& This::operator=(FramebufferObject&& o)
{
   destroy();
   std::swap(fbo_, o.fbo_);
   std::swap(rbo_rgba_, o.rbo_rgba_);
   std::swap(rbo_depth_, o.rbo_depth_);
   std::swap(w_, o.w_);
   std::swap(h_, o.h_);
   return *this;
}

// ------------------------------------------------------------------------ Init

static void create_fbo(const unsigned fbo_w,
                       const unsigned fbo_h,
                       GLuint& fbo,
                       GLuint& rbo_rgba,
                       GLuint& rbo_depth)
{
   {
      auto err = glGetError();
      if(err != GL_NO_ERROR) {
         auto msg = format("Checked opengl before creating FBO, and "
                           "found that it was not clear. glGetError "
                           "returned {:s}",
                           gl_error_to_cstr(int(err)));
         throw std::runtime_error(msg);
      }
   }
   Expects(glGetError() == GL_NO_ERROR);

   // Generate frame-buffer object
   glGenFramebuffers(1, &fbo);
   glBindFramebuffer(GL_FRAMEBUFFER, fbo);

   // Generate and bind the depth buffer
   glGenRenderbuffers(1, &rbo_depth);
   glBindRenderbuffer(GL_RENDERBUFFER, rbo_depth);
   glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32, fbo_w, fbo_h);
   glFramebufferRenderbuffer(
       GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbo_depth);

   // Generate and bind the color buffer
   glGenRenderbuffers(1, &rbo_rgba);
   glBindRenderbuffer(GL_RENDERBUFFER, rbo_rgba);
   glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA, fbo_w, fbo_h);
   glFramebufferRenderbuffer(
       GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rbo_rgba);

   // check FBO status
   GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
   if(status != GL_FRAMEBUFFER_COMPLETE)
      FATAL(format("Failed to create frame-buffer object"));

   // Set the attachments (draw buffer) for the (still bound) frame buffer
   GLuint attachments[1] = {GL_COLOR_ATTACHMENT0};
   glDrawBuffers(1, attachments);

   // Throw on any error
   try {
      check_and_throw_on_gl_error();
   } catch(std::exception& e) {
      glDeleteRenderbuffers(1, &rbo_depth);
      glDeleteRenderbuffers(1, &rbo_rgba);
      glDeleteFramebuffers(1, &fbo);
      fbo = rbo_depth = rbo_rgba = 0;
      throw e;
   }
}

// ------------------------------------------------------------------------ Init

void This::init(const unsigned fbo_w, const unsigned fbo_h) noexcept(false)
{
   if(is_valid()) throw std::runtime_error("FBO is already attached");
   GLuint fbo = 0, rbo_rgba = 0, rbo_depth = 0;
   create_fbo(fbo_w, fbo_h, fbo, rbo_rgba, rbo_depth);

   fbo_       = fbo;
   rbo_rgba_  = rbo_rgba;
   rbo_depth_ = rbo_depth;
   w_         = fbo_w;
   h_         = fbo_h;
}

// --------------------------------------------------------------------- Destroy

void This::destroy()
{
   if(fbo_ > 0) {
      check_and_warn_on_gl_error("Before destroying frame buffer, ");

      GLuint v;
      v = rbo_depth_;
      glDeleteRenderbuffers(1, &v);
      v = rbo_rgba_;
      glDeleteRenderbuffers(1, &v);
      v = fbo_;
      glDeleteFramebuffers(1, &v);
      fbo_ = rbo_depth_ = rbo_rgba_ = w_ = h_ = 0;

      check_and_warn_on_gl_error("While destroying frame buffer, ");
   }
   Ensures(fbo_ == 0);
   Ensures(rbo_rgba_ == 0);
   Ensures(rbo_depth_ == 0);
   Ensures(w_ == 0);
   Ensures(h_ == 0);
}

// ------------------------------------------------------------------------ Bind

bool This::bind() const noexcept
{
   if(is_valid()) {
      check_and_warn_on_gl_error("before binding frame buffer, ");
      glBindFramebuffer(GL_FRAMEBUFFER, fbo());
      return check_and_warn_on_gl_error("while binding frame buffer, ");
   }

   WARN("frame buffer object is invalid");

   return false;
}

} // namespace perceive
