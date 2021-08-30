
#pragma once

#include "shader.hpp"

namespace perceive
{
struct GlslProgram
{
 private:
   unsigned program_{0};
   void destroy();
   bool check(bool do_throw = true) const noexcept(false);
   int loc_nothrow(const char* variable_name) const noexcept;

 public:
   GlslProgram() = default;
   GlslProgram(const Shader& src) noexcept(false) { link(src); }
   ~GlslProgram() { destroy(); }

   GlslProgram(const GlslProgram&) = delete; // No copy
   GlslProgram& operator=(const GlslProgram&) = delete;

   GlslProgram(GlslProgram&& o) { *this = std::move(o); }
   GlslProgram& operator=(GlslProgram&& o)
   {
      std::swap(program_, o.program_);
      return *this;
   }

   unsigned id() const noexcept { return program_; }

   void link(const Shader& shader) noexcept(false);
   void link(const std::vector<unsigned>& shaders) noexcept(false);
   string program_info_log() const;

   bool link_status() const noexcept;

   void use() const; // glUseProgram(id())
   void dispatch_execute(unsigned x, unsigned y, unsigned z) const;

   // NOTE, these functions assume that 'use()' has been called
   int location(const char* variable_name) const noexcept(false);

   bool set_f(int loc, float x) noexcept(false);
   bool set_f(int loc, float x, float y) noexcept(false);
   bool set_f(int loc, float x, float y, float z) noexcept(false);
   bool set_f(int loc, float x, float y, float z, float w) noexcept(false);
   bool set_i(int loc, int x) noexcept(false);
   bool set_i(int loc, int x, int y) noexcept(false);
   bool set_i(int loc, int x, int y, int z) noexcept(false);
   bool set_i(int loc, int x, int y, int z, int w) noexcept(false);
   bool set_u(int loc, unsigned x) noexcept(false);
   bool set_u(int loc, unsigned x, unsigned y) noexcept(false);
   bool set_u(int loc, unsigned x, unsigned y, unsigned z) noexcept(false);
   bool set_u(int loc, unsigned x, unsigned y, unsigned z,
              unsigned w) noexcept(false);
   bool set_3x3(int loc, const Matrix3r& K) noexcept(false);
   bool set_3x3(int loc, const Matrix3f& K) noexcept(false);
   bool set_3x3s(int loc, const vector<Matrix3r>& K) noexcept(false);
   bool set_3x3s(int loc, const vector<Matrix3f>& M) noexcept(false);
   bool set_3x4s(int loc, const vector<Matrix34r>& K) noexcept(false);
   bool set_vec3s(int loc, const vector<Vector3r>& K) noexcept(false);

   bool set_f(const char* var, float x) noexcept(false);
   bool set_f(const char* var, float x, float y) noexcept(false);
   bool set_f(const char* var, float x, float y, float z) noexcept(false);
   bool set_f(const char* var, float x, float y, float z,
              float w) noexcept(false);
   bool set_i(const char* var, int x) noexcept(false);
   bool set_i(const char* var, int x, int y) noexcept(false);
   bool set_i(const char* var, int x, int y, int z) noexcept(false);
   bool set_i(const char* var, int x, int y, int z, int w) noexcept(false);
   bool set_u(const char* var, unsigned x) noexcept(false);
   bool set_u(const char* var, unsigned x, unsigned y) noexcept(false);
   bool set_u(const char* var, unsigned x, unsigned y,
              unsigned z) noexcept(false);
   bool set_u(const char* var, unsigned x, unsigned y, unsigned z,
              unsigned w) noexcept(false);
   bool set_3x3(const char* var, const Matrix3r& K) noexcept(false);
   bool set_3x3(const char* var, const Matrix3f& K) noexcept(false);
   bool set_3x3s(const char* var, const vector<Matrix3r>& K) noexcept(false);
   bool set_3x3s(const char* var, const vector<Matrix3f>& K) noexcept(false);
   bool set_3x4s(const char* var, const vector<Matrix34r>& K) noexcept(false);
   bool set_vec3s(const char* var, const vector<Vector3r>& K) noexcept(false);
};

} // namespace perceive
