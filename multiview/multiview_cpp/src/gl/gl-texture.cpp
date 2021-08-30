
#include "gl-error.hpp"
#include "gl-texture.hpp"
#include "gl/platform/glew-bridge.hpp"

#define This GlTexture

namespace perceive
{
static int dims_to_gl(GlTexture::Type type, unsigned dims)
{
   switch(type) {
   case GlTexture::FLOAT:
      switch(dims) {
      case 1: return GL_R32F;
      case 2: return GL_RG32F;
      case 3: return GL_RGB32F;
      case 4: return GL_RGBA32F;
      }

   case GlTexture::UBYTE:
      switch(dims) {
      case 1: return GL_R8;
      case 2: return GL_RG8;
      case 3: return GL_RGB8;
      case 4: return GL_RGBA8;
      }

   case GlTexture::INT32:
      switch(dims) {
      case 1: return GL_RED;
      case 2: return GL_RG;
      case 3: return GL_RGB;
      case 4: return GL_RGBA;
      }
   }
   Expects(false);
   return 0;
}

static int dims_to_bind(GlTexture::Type type, unsigned dims)
{
   switch(type) {
   case GlTexture::FLOAT:
      switch(dims) {
      case 1: return GL_R32F;
      case 2: return GL_RG32F;
      case 3: return GL_RGB32F;
      case 4: return GL_RGBA32F;
      }

   case GlTexture::UBYTE:
      switch(dims) {
      case 1: return GL_R8;
      case 2: return GL_RG8;
      case 3: return GL_RGB8;
      case 4: return GL_RGBA8;
      }

   case GlTexture::INT32:
      switch(dims) {
      case 1: return GL_R32I;
      case 2: return GL_RG32I;
      case 3: return GL_RGB32I;
      case 4: return GL_RGBA32I;
      }
   }
   Expects(false);
   return 0;
}

static GLenum dims_to_format(unsigned dims)
{
   switch(dims) {
   case 1: return GL_RED;
   case 2: return GL_RG;
   case 3: return GL_RGB;
   case 4: return GL_RGBA;
   }
   Expects(false);
   return 0;
}

static GLenum type_to_gl(GlTexture::Type type)
{
   switch(type) {
   case GlTexture::FLOAT: return GL_FLOAT;
   case GlTexture::UBYTE: return GL_UNSIGNED_BYTE;
   case GlTexture::INT32: return GL_INT;
   }
   Expects(false);
   return 0;
}

static int type_to_sz(GlTexture::Type type)
{
   switch(type) {
   case GlTexture::FLOAT: return 4;
   case GlTexture::UBYTE: return 1;
   case GlTexture::INT32: return 4;
   }
   Expects(false);
   return 0;
}

void This::destroy()
{
   if(tex_ > 0) {
      GLuint tex = tex_;
      glDeleteTextures(1, &tex);
      tex_ = 0;
   }
}

GlTexture& This::operator=(GlTexture&& o)
{
   std::swap(tex_, o.tex_);
   std::swap(w_, o.w_);
   std::swap(h_, o.h_);
   std::swap(dims_, o.dims_);
   return *this;
}

void This::init(Type type,
                unsigned w,
                unsigned h,
                unsigned dims) noexcept(false)
{
   if(dims != 1 and dims != 2 and dims != 4)
      throw std::runtime_error(format("dims = {} not in {1, 2, 4}", dims));

   check_and_warn_on_gl_error("before initializing texture");

   destroy();

   GLuint tex;
   glGenTextures(1, &tex);
   if(tex == 0) throw std::runtime_error("failed to generate texture");

   tex_         = tex;
   type_        = type;
   sizeof_type_ = unsigned(type_to_sz(type));
   is_3d_       = false;
   w_           = w;
   h_           = h;
   layers_      = 1;
   dims_        = dims;

   glActiveTexture(GL_TEXTURE0); // TODO, maybe shouldn't be here
   glBindTexture(GL_TEXTURE_2D, tex_);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
   glTexImage2D(GL_TEXTURE_2D,
                0,
                dims_to_gl(type, dims),
                GLsizei(w),
                GLsizei(h),
                0,
                dims_to_format(dims_),
                type_to_gl(type),
                nullptr);

   check_and_throw_on_gl_error();
}

void This::init_3d(Type type,
                   unsigned w,
                   unsigned h,
                   unsigned layers,
                   unsigned dims) noexcept(false)
{
   if(dims != 1 and dims != 2 and dims != 4)
      throw std::runtime_error(format("dims = {} not in {1, 2, 4}", dims));

   check_and_warn_on_gl_error("before initializing texture");

   destroy();

   GLuint tex;
   glGenTextures(1, &tex);
   if(tex == 0) throw std::runtime_error("failed to generate texture");

   tex_         = tex;
   type_        = type;
   sizeof_type_ = unsigned(type_to_sz(type));
   is_3d_       = true;
   w_           = w;
   h_           = h;
   layers_      = layers;
   dims_        = dims;

   // glActiveTexture(GL_TEXTURE0); // TODO, maybe shouldn't be here
   glBindTexture(GL_TEXTURE_3D, tex_);
   glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
   glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
   glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
   glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
   glTexImage3D(GL_TEXTURE_3D,
                0,
                dims_to_gl(type, dims),
                w,
                h,
                layers,
                0,
                dims_to_format(dims_),
                type_to_gl(type),
                nullptr);

   check_and_warn_on_gl_error("after initializing texture");
   check_and_throw_on_gl_error();
}

void This::bind_image_unit(unsigned unit, Access access0) noexcept(false)
{
   if(tex_ == 0) throw std::runtime_error("no initialized texture");

   check_and_warn_on_gl_error("before binding image unit");

   GLenum access = GL_READ_WRITE;
   if(access0 == READONLY) access = GL_READ_ONLY;
   if(access0 == WRITEONLY) access = GL_WRITE_ONLY;

   auto fmt = dims_to_bind(type_, dims_);

   if(is_3d_) {
      glBindImageTexture(unit, tex_, 0, GL_TRUE, 0, access, fmt);
   } else {
      glBindImageTexture(unit, tex_, 0, GL_FALSE, 0, access, fmt);
   }

   check_and_throw_on_gl_error("after binding image unit");
}

// ------------------------------------------------------------------- Read Data

void This::read_data(void* dst) const noexcept(false)
{
   if(tex_ == 0) throw std::runtime_error("no initialized texture");
   check_and_warn_on_gl_error("before reading texture data");

   if(is_3d_) {
      glBindTexture(GL_TEXTURE_3D, tex_);
      glGetTexImage(
          GL_TEXTURE_3D, 0, dims_to_format(dims_), type_to_gl(type_), dst);
   } else {
      glBindTexture(GL_TEXTURE_2D, tex_);
      glGetTexImage(
          GL_TEXTURE_2D, 0, dims_to_format(dims_), type_to_gl(type_), dst);
   }
}

void This::read_data(std::vector<float>& dst) const noexcept(false)
{
   if(dst.size() != flat_size()) dst.resize(flat_size());
   read_data(&dst[0]);
}

void This::read_data(std::vector<int32_t>& dst) const noexcept(false)
{
   if(dst.size() != flat_size()) dst.resize(flat_size());
   read_data(&dst[0]);
}

void This::read_data(std::vector<uint8_t>& dst) const noexcept(false)
{
   if(dst.size() != flat_size()) dst.resize(flat_size());
   read_data(&dst[0]);
}

// ------------------------------------------------------------------- Set Layer

void This::set_data(const void* src) noexcept(false)
{
   if(tex_ == 0) throw std::runtime_error("no initialized texture");
   check_and_warn_on_gl_error("before setting texture data");

   if(is_3d_) {
      glBindTexture(GL_TEXTURE_3D, tex_);
      glTexSubImage3D(GL_TEXTURE_3D,
                      0,
                      0,
                      0,
                      0,
                      GLsizei(w()),
                      GLsizei(h()),
                      layers(),
                      dims_to_format(dims_),
                      type_to_gl(type_),
                      src);
   } else {
      glBindTexture(GL_TEXTURE_2D, tex_);
      glTexSubImage2D(GL_TEXTURE_2D,
                      0,
                      0,
                      0,
                      GLsizei(w()),
                      GLsizei(h()),
                      dims_to_format(dims_),
                      type_to_gl(type_),
                      src);
   }

   check_and_warn_on_gl_error("after setting texture data");
}

void This::set_layer(unsigned layer, const void* src) noexcept(false)
{
   if(tex_ == 0) throw std::runtime_error("no initialized texture");
   check_and_warn_on_gl_error("before writing texture data");

   if(is_3d_) {
      glBindTexture(GL_TEXTURE_3D, tex_);
      glTexSubImage3D(GL_TEXTURE_3D,
                      0,
                      0,
                      0,
                      layer,
                      GLsizei(w()),
                      GLsizei(h()),
                      1,
                      dims_to_format(dims_),
                      type_to_gl(type_),
                      src);
   } else {
      glBindTexture(GL_TEXTURE_2D, tex_);
      glTexSubImage2D(GL_TEXTURE_2D,
                      0,
                      0,
                      0,
                      GLsizei(w()),
                      GLsizei(h()),
                      dims_to_format(dims_),
                      type_to_gl(type_),
                      src);
   }

   check_and_warn_on_gl_error("after writing texture data");
}

void This::set_layer(unsigned layer, const ARGBImage& src) noexcept(false)
{
   Expects(layer < layers_);
   Expects((type_ == INT32 and dims_ == 1) or (type_ == UBYTE and dims_ == 4));
   set_layer(layer, src.ptr(0));
}

void This::set_layer(unsigned layer, const FloatImage& src) noexcept(false)
{
   Expects(layer < layers_);
   Expects(type_ == FLOAT and dims_ == 1);
   set_layer(layer, src.ptr(0));
}

void This::set_layer(unsigned layer, const FloatField& src) noexcept(false)
{
   Expects(layer < layers_);
   Expects(type_ == FLOAT and dims_ == 2);
   set_layer(layer, src.ptr(0));
}

void This::set_layer(unsigned layer, const Vec4fImage& src) noexcept(false)
{
   Expects(layer < layers_);
   Expects(type_ == FLOAT and dims_ == 4);
   set_layer(layer, src.ptr(0));
}

} // namespace perceive
