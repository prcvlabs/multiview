
#include "gl/gl-utils.hpp"
#include "glsl-program.hpp"

#define This GlslProgram

#include "gl/platform/glew-bridge.hpp"

namespace perceive
{
// --------------------------------------------------------------------- destroy

void This::destroy()
{
   if(program_ > 0) {
      glDeleteShader(program_);
      program_ = 0;
   }
}

// ------------------------------------------------------------------------- use

void This::use() const
{
   if(program_ == 0) throw std::runtime_error("failed to create program");
   check_and_warn_on_gl_error("before gl-use-program");
   glUseProgram(program_);
   check_and_throw_on_gl_error("using glsl program");
}

// ------------------------------------------------------------ dispatch-execute

void This::dispatch_execute(unsigned x, unsigned y, unsigned z) const
{
   if(program_ == 0) throw std::runtime_error("failed to create program");
   check_and_warn_on_gl_error("before dispatch-execute");
   this->use();
   glDispatchCompute(x, y, z);
   check_and_throw_on_gl_error("dispatch-executing program");
}

// ------------------------------------------------------------------------ link

void This::link(const Shader& shader) noexcept(false)
{
   check_and_warn_on_gl_error("before linking program");
   destroy(); // Ensure shader is cleared
   program_ = glCreateProgram();
   if(program_ == 0) throw std::runtime_error("failed to create program");
   glAttachShader(program_, shader.id());
   check_and_warn_on_gl_error("after attaching shader program");
   glLinkProgram(program_);
   check_and_warn_on_gl_error("linking program");
   if(!link_status()) throw std::runtime_error("linker error");
}

void This::link(const std::vector<unsigned>& shaders) noexcept(false)
{
   check_and_warn_on_gl_error("before linking program");
   destroy(); // Ensure shader is cleared
   program_ = glCreateProgram();
   if(program_ == 0) throw std::runtime_error("failed to create program");
   for(const auto& o : shaders) glAttachShader(program_, o);
   check_and_warn_on_gl_error("after attaching shader program");
   glLinkProgram(program_);
   check_and_throw_on_gl_error("linking program");
   if(!link_status()) throw std::runtime_error("linker error");
}

// ----------------------------------------------------------------- link status

bool This::link_status() const noexcept
{
   check_and_warn_on_gl_error("before checking program link status");
   if(program_ > 0) {
      GLint val = -1;
      glGetProgramiv(program_, GL_LINK_STATUS, &val);
      return val == GL_TRUE;
   }
   return false;
}

// ------------------------------------------------------------ program info log

string This::program_info_log() const
{
   check_and_warn_on_gl_error("before getting program log");
   GLint len{-1};
   glGetProgramiv(program_, GL_INFO_LOG_LENGTH, &len);

   Expects(sizeof(GLchar) == sizeof(char));
   std::vector<GLchar> slog((size_t(len + 1)));

   GLsizei actual_length;
   glGetProgramInfoLog(program_, len, &actual_length, &slog[0]);

   return string(&slog[0]);
}

// -------------------------------------------------------------------- location

int This::location(const char* name) const noexcept(false)
{
   auto ret = loc_nothrow(name);
   if(ret == -1)
      throw std::runtime_error(format("variable not found: '{:s}'", name));
   return ret;
}

// ----------------------------------------------------------------------- check

bool This::check(bool do_throw) const noexcept(false)
{
   auto ret = check_and_warn_on_gl_error("set program variable");
   if(!program_) throw std::runtime_error("program unintialized");
   return ret;
}

int This::loc_nothrow(const char* name) const noexcept
{
   check_and_warn_on_gl_error("before getting program variable location");
   Expects(program_ != 0);
   auto ret = glGetUniformLocation(program_, name);
   if(ret == -1) WARN(format("variable not found: '{:s}'", name));
   check_and_warn_on_gl_error("after getting program variable location");
   return ret;
}

bool This::set_f(int loc, float x) noexcept(false)
{
   check();
   if(loc >= 0) glUniform1f(loc, x);
   return check() and loc >= 0;
}
bool This::set_f(int loc, float x, float y) noexcept(false)
{
   check();
   if(loc >= 0) glUniform2f(loc, x, y);
   return check() and loc >= 0;
}
bool This::set_f(int loc, float x, float y, float z) noexcept(false)
{
   check();
   if(loc >= 0) glUniform3f(loc, x, y, z);
   return check() and loc >= 0;
}
bool This::set_f(int loc, float x, float y, float z, float w) noexcept(false)
{
   check();
   if(loc >= 0) glUniform4f(loc, x, y, z, w);
   return check() and loc >= 0;
}
bool This::set_i(int loc, int x) noexcept(false)
{
   check();
   if(loc >= 0) glUniform1i(loc, x);
   return check() and loc >= 0;
}
bool This::set_i(int loc, int x, int y) noexcept(false)
{
   check();
   if(loc >= 0) glUniform2i(loc, x, y);
   return check() and loc >= 0;
}
bool This::set_i(int loc, int x, int y, int z) noexcept(false)
{
   check();
   if(loc >= 0) glUniform3i(loc, x, y, z);
   return check() and loc >= 0;
}
bool This::set_i(int loc, int x, int y, int z, int w) noexcept(false)
{
   check();
   if(loc >= 0) glUniform4i(loc, x, y, z, w);
   return check() and loc >= 0;
}
bool This::set_u(int loc, unsigned x) noexcept(false)
{
   check();
   if(loc >= 0) glUniform1ui(loc, x);
   return check() and loc >= 0;
}
bool This::set_u(int loc, unsigned x, unsigned y) noexcept(false)
{
   check();
   if(loc >= 0) glUniform2ui(loc, x, y);
   return check() and loc >= 0;
}
bool This::set_u(int loc, unsigned x, unsigned y, unsigned z) noexcept(false)
{
   check();
   if(loc >= 0) glUniform3ui(loc, x, y, z);
   return check() and loc >= 0;
}
bool This::set_u(int loc,
                 unsigned x,
                 unsigned y,
                 unsigned z,
                 unsigned w) noexcept(false)
{
   check();
   if(loc >= 0) glUniform4ui(loc, x, y, z, w);
   return check() and loc >= 0;
}
bool This::set_3x3(int loc, const Matrix3r& K) noexcept(false)
{
   Matrix3f M;
   matrixU_to_V(K, M);
   return set_3x3(loc, M);
}
bool This::set_3x3(int loc, const Matrix3f& K) noexcept(false)
{
   check();
   GLfloat M[9];
   for(int r = 0; r < 3; ++r)
      for(int c = 0; c < 3; ++c) M[r * 3 + c] = K(c, r);
   if(loc >= 0) glUniformMatrix3fv(loc, 1, GL_FALSE, M);
   return check() and loc >= 0;
}
bool This::set_3x3s(int loc, const vector<Matrix3r>& K) noexcept(false)
{
   check();
   vector<Matrix3f> M(K.size());
   std::transform(cbegin(K), cend(K), begin(M), [](const auto& J) {
      Matrix3f Z;
      matrixU_to_V(J, Z);
      return Z;
   });
   return set_3x3s(loc, M);
}
bool This::set_3x3s(int loc, const vector<Matrix3f>& Ks) noexcept(false)
{
   if(loc >= 0) {
      check();
      vector<GLfloat> M(Ks.size() * 9);
      auto dst = &M[0];
      for(const auto& K : Ks)
         for(int r = 0; r < 3; ++r)
            for(int c = 0; c < 3; ++c) *dst++ = K(r, c);
      glUniformMatrix3fv(loc, Ks.size(), GL_TRUE, &M[0]);
   }
   return check() and loc >= 0;
}
bool This::set_3x4s(int loc, const vector<Matrix34r>& Ps) noexcept(false)
{
   if(loc >= 0) {
      check();
      vector<GLfloat> M(Ps.size() * 12);
      auto dst = &M[0];
      for(const auto& P : Ps)
         for(int r = 0; r < 3; ++r)
            for(int c = 0; c < 4; ++c) *dst++ = float(P(r, c));
      // Column major, so 4 cols, and 3 rows (!)
      glUniformMatrix4x3fv(loc, Ps.size(), GL_TRUE, &M[0]);
   }
   return check() and loc >= 0;
}
bool This::set_vec3s(int loc, const vector<Vector3r>& Xs) noexcept(false)
{
   if(loc >= 0) {
      check();
      vector<GLfloat> M(Xs.size() * 3);
      auto dst = &M[0];
      for(const auto& X : Xs)
         for(int c = 0; c < 3; ++c) *dst++ = float(X(c));
      glUniform3fv(loc, Xs.size(), &M[0]);
   }
   return check() and loc >= 0;
}

bool This::set_f(const char* var, float x) noexcept(false)
{
   check();
   return set_f(loc_nothrow(var), x);
}
bool This::set_f(const char* var, float x, float y) noexcept(false)
{
   check();
   return set_f(loc_nothrow(var), x, y);
}
bool This::set_f(const char* var, float x, float y, float z) noexcept(false)
{
   check();
   return set_f(loc_nothrow(var), x, y, z);
}
bool This::set_f(const char* var, float x, float y, float z, float w) noexcept(
    false)
{
   check();
   return set_f(loc_nothrow(var), x, y, z, w);
}
bool This::set_i(const char* var, int x) noexcept(false)
{
   check();
   return set_i(loc_nothrow(var), x);
}
bool This::set_i(const char* var, int x, int y) noexcept(false)
{
   check();
   return set_i(loc_nothrow(var), x, y);
}
bool This::set_i(const char* var, int x, int y, int z) noexcept(false)
{
   check();
   return set_i(loc_nothrow(var), x, y, z);
}
bool This::set_i(const char* var, int x, int y, int z, int w) noexcept(false)
{
   check();
   return set_i(loc_nothrow(var), x, y, z, w);
}
bool This::set_u(const char* var, unsigned x) noexcept(false)
{
   check();
   return set_u(loc_nothrow(var), x);
}
bool This::set_u(const char* var, unsigned x, unsigned y) noexcept(false)
{
   check();
   return set_u(loc_nothrow(var), x, y);
}
bool This::set_u(const char* var,
                 unsigned x,
                 unsigned y,
                 unsigned z) noexcept(false)
{
   check();
   return set_u(loc_nothrow(var), x, y, z);
}
bool This::set_u(const char* var,
                 unsigned x,
                 unsigned y,
                 unsigned z,
                 unsigned w) noexcept(false)
{
   check();
   return set_u(loc_nothrow(var), x, y, z, w);
}
bool This::set_3x3(const char* var, const Matrix3r& K) noexcept(false)
{
   check();
   return set_3x3(loc_nothrow(var), K);
}
bool This::set_3x3(const char* var, const Matrix3f& K) noexcept(false)
{
   check();
   return set_3x3(loc_nothrow(var), K);
}
bool This::set_3x3s(const char* var, const vector<Matrix3r>& K) noexcept(false)
{
   check();
   return set_3x3s(loc_nothrow(var), K);
}
bool This::set_3x3s(const char* var, const vector<Matrix3f>& K) noexcept(false)
{
   check();
   return set_3x3s(loc_nothrow(var), K);
}
bool This::set_3x4s(const char* var, const vector<Matrix34r>& K) noexcept(false)
{
   check();
   return set_3x4s(loc_nothrow(var), K);
}
bool This::set_vec3s(const char* var, const vector<Vector3r>& K) noexcept(false)
{
   check();
   return set_vec3s(loc_nothrow(var), K);
}

} // namespace perceive
