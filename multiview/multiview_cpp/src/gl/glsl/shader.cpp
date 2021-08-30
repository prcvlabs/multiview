
#include "gl/gl-error.hpp"
#include "gl/platform/glew-bridge.hpp"
#include "shader.hpp"

#define This Shader

namespace perceive
{
static int to_gl_shader(This::Type type)
{
   switch(type) {
   case This::COMPUTE: return GL_COMPUTE_SHADER;
   case This::VERTEX: return GL_VERTEX_SHADER;
   case This::GEOMETRY: return GL_GEOMETRY_SHADER;
   case This::FRAGMENT: return GL_FRAGMENT_SHADER;
   }
}

// --------------------------------------------------------------------- destroy

void This::destroy()
{
   if(shader_ > 0) {
      glDeleteShader(shader_);
      shader_ = 0;
   }
}

// --------------------------------------------------------------------- compile

void This::compile(Type type, const string& src) noexcept(false)
{
   check_and_warn_on_gl_error("before compiling shader");
   destroy(); // Ensure shader is cleared
   shader_ = glCreateShader(to_gl_shader(type));
   if(shader_ == 0) throw std::runtime_error("failed to create shader");

   Expects(sizeof(GLchar) == sizeof(char));
   const GLchar* s = static_cast<const GLchar*>(&src[0]);
   glShaderSource(shader_, 1, &s, nullptr);
   check_and_throw_on_gl_error("compiling shader");
   glCompileShader(shader_);
   if(!compile_status()) {
      LOG_ERR(format("shader {} compilation failed, printing log", shader_));
      cout << shader_info_log() << endl;
      throw std::runtime_error("shader did not compile");
   }
   check_and_throw_on_gl_error("after compiling shader");
}

// -------------------------------------------------------------- compile status

bool This::compile_status() const noexcept
{
   if(shader_ > 0) {
      GLint val = -1;
      glGetShaderiv(shader_, GL_COMPILE_STATUS, &val);
      return val == GL_TRUE;
   }
   return false;
}

// ------------------------------------------------------------- shader info log

string This::shader_info_log() const
{
   GLint len{-1};
   glGetShaderiv(shader_, GL_INFO_LOG_LENGTH, &len);

   Expects(sizeof(GLchar) == sizeof(char));

   std::vector<GLchar> slog((size_t(len + 1)));

   GLsizei actual_length;
   glGetShaderInfoLog(shader_, len, &actual_length, &slog[0]);

   return string(&slog[0]);
}

} // namespace perceive
