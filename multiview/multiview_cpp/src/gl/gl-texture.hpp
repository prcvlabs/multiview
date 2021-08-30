
#pragma once

#include "perceive/graphics/image-container.hpp"
#include <vector>

namespace perceive
{
struct GlTexture
{
   enum Type : int { FLOAT, UBYTE, INT32 };
   enum Access : int { READONLY, WRITEONLY, READWRITE };

 private:
   unsigned tex_{0};
   unsigned w_{0};
   unsigned h_{0};
   unsigned layers_{0};
   unsigned dims_{0}; // {1, 2, 3, 4}, for float, vec2f, vec3f, vec4f
   Type type_{FLOAT};
   unsigned sizeof_type_{0};
   bool is_3d_{false};

   void destroy();

 public:
   GlTexture() = default;
   GlTexture(Type type, unsigned w, unsigned h, unsigned dims)
   {
      init(type, w, h, dims);
   }
   ~GlTexture() { destroy(); }

   GlTexture(const GlTexture&) = delete; // No copy
   GlTexture& operator=(const GlTexture&) = delete;

   GlTexture(GlTexture&& o) { *this = std::move(o); }
   GlTexture& operator=(GlTexture&& o);

   // ---- Getters ----
   unsigned id() const noexcept { return tex_; }
   unsigned w() const noexcept { return w_; }
   unsigned h() const noexcept { return h_; }
   unsigned layers() const noexcept { return layers_; }
   unsigned dims() const noexcept { return dims_; }

   size_t flat_size() const noexcept { return w() * h() * layers() * dims(); }
   size_t n_bytes() const noexcept { return flat_size() * sizeof_type_; }

   // ---- Initializing ----
   void init(Type type, unsigned w, unsigned h, unsigned dims) noexcept(false);
   void init_3d(Type type, unsigned w, unsigned h, unsigned layers,
                unsigned dims) noexcept(false);

   // ---- Binding ----
   void bind_image_unit(unsigned unit,
                        Access access = READWRITE) noexcept(false);

   // ---- Reading data ---
   void read_data(void* dst) const noexcept(false);
   void read_data(std::vector<float>& dst) const noexcept(false);
   void read_data(std::vector<int32_t>& dst) const noexcept(false);
   void read_data(std::vector<uint8_t>& dst) const noexcept(false);

   // ---- Setting data ---
   void set_data(const void* src) noexcept(false);
   void set_layer(unsigned layer, const void* src) noexcept(false);
   void set_layer(unsigned layer, const ARGBImage& src) noexcept(false);
   void set_layer(unsigned layer, const FloatImage& src) noexcept(false);
   void set_layer(unsigned layer, const FloatField& src) noexcept(false);
   void set_layer(unsigned layer, const Vec4fImage& src) noexcept(false);
};

} // namespace perceive
