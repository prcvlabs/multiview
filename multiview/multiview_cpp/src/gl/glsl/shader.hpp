
#pragma once

namespace perceive
{
struct Shader
{
 private:
   unsigned shader_{0};
   void destroy();

 public:
   enum Type : int { COMPUTE, VERTEX, GEOMETRY, FRAGMENT };

   Shader() = default;
   Shader(Type type, const string& src) noexcept(false) { compile(type, src); }
   ~Shader() { destroy(); }

   Shader(const Shader&) = delete; // No copy
   Shader& operator=(const Shader&) = delete;

   Shader(Shader&& o) { *this = std::move(o); }
   Shader& operator=(Shader&& o)
   {
      std::swap(shader_, o.shader_);
      return *this;
   }

   unsigned id() const noexcept { return shader_; }

   void compile(Type type, const string& src) noexcept(false);
   string shader_info_log() const;

   bool compile_status() const noexcept;
};

} // namespace perceive
