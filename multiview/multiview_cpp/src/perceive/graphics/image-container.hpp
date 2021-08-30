
#pragma once

#include <memory>
#include <stdint.h>

#include <opencv2/core.hpp>

#include "colour-set.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/utils/concepts.hpp"
#include "perceive/utils/memory.hpp"

namespace Json
{
class Value;
};

namespace perceive
{
template<typename T = uint8_t> struct ImageContainerT
{
 public:
   using ptr_type   = T*;
   using value_type = T;

   CUSTOM_NEW_DELETE(ImageContainerT)

 private:
   unsigned w{0};
   unsigned h{0};
   unsigned stride{0};
   owned_ptr<T> pix{nullptr};
   bool owns{false}; // pix is managed _only_ if TRUE

 public:
   const unsigned& width{w};
   const unsigned& height{h};
   const unsigned& row_stride{stride};
   const ptr_type& pixels{pix};

 public:
   ImageContainerT() noexcept
       : owns(true)
   {}

   ImageContainerT(bool owns_memory) noexcept
       : owns(owns_memory)
   {}

   template<std::integral I>
   ImageContainerT(I w, I h) noexcept
       : owns(true)
   {
      resize(w, h);
   }

   ~ImageContainerT()
   {
      if(owns) delete[] pix;
   }

   ImageContainerT(const ImageContainerT& rhs)
       : owns(true)
   {
      *this = rhs;
   }

   ImageContainerT(ImageContainerT&& rhs) noexcept
       : owns(true)
   {
      *this = std::move(rhs);
   }

   ImageContainerT& operator=(const ImageContainerT& rhs)
   {
      if(&rhs != this) { // no self-assigment
         if(owns) {
            set_size(rhs.width, rhs.height, rhs.row_stride);
            memcpy(pix, rhs.pix, n_bytes());
         } else {
            w      = rhs.w;
            h      = rhs.h;
            stride = rhs.stride;
            pix    = rhs.pix;
         }
      }
      return *this;
   }

   ImageContainerT& operator=(ImageContainerT&& rhs) noexcept
   {
      if(&rhs != this) { // no self-assigment
         if(owns) delete[] pix;
         owns   = rhs.owns;
         w      = rhs.w;
         h      = rhs.h;
         stride = rhs.stride;
         pix    = rhs.pix;
         if(rhs.owns) rhs.pix = nullptr;
      }
      return *this;
   }

   bool operator==(const ImageContainerT& o) const noexcept
   {
      if(width != o.width or height != o.height) return false;
      for(auto y = 0u; y < height; ++y)
         if(memcmp(row_ptr(y), o.row_ptr(y), row_bytes()) != 0) return false;
      return true;
   }

   bool operator!=(const ImageContainerT& o) const noexcept
   {
      return not(*this == o);
   }

   ImageContainerT& operator+=(const ImageContainerT& o)
   {
      Expects(o.width == width and o.height == height);
      if(this == &o) {
         T* __restrict__ itr = pix;
         const T* end        = itr + height * row_stride;
         while(itr != end) {
            *itr = *itr + *itr;
            ++itr;
         }
      } else {
         T* __restrict__ itr = pix;
         T* __restrict__ src = o.pix;
         const T* end        = itr + height * row_stride;
         while(itr != end) *itr++ += *src++;
      }
      return *this;
   }

   static ImageContainerT load(const char* filename);
   static ImageContainerT load(const std::string& filename)
   {
      return load(filename.c_str());
   }
   void save(const char* filename) const;
   void save(const std::string& filename) const { save(filename.c_str()); }

   std::pair<unsigned, unsigned> dims() const
   {
      return std::pair<unsigned, unsigned>(width, height);
   };

   void copy_to(ImageContainerT& rhs) const
   {
      rhs.set_size(width, height, row_stride);
      memcpy(rhs.data(), pixels, n_bytes());
   }

   void copy_to_while_flipping_y(ImageContainerT& rhs) const
   {
      rhs.set_size(width, height, row_stride);
      for(auto y = 0u; y < height; ++y)
         memcpy(rhs.ptr(height - y - 1), ptr(y), row_bytes());
   }

   Point2 flip_y(const Point2& x) const noexcept
   {
      return Point2(x.x, flip_y(x.y));
   }

   Vector2 flip_y(const Vector2& x) const noexcept
   {
      return Vector2(x.x, flip_y(x.y));
   }

   int flip_y(int y) const noexcept { return int(height) - y - 1; }
   real flip_y(real y) const noexcept { return real(height) - y - 1; }

   size_t size() const noexcept { return width * height; }
   size_t sizeof_T() const noexcept { return sizeof(T); }
   size_t n_bytes() const noexcept { return height * row_stride * sizeof(T); }
   bool owns_memory() const noexcept { return owns; }
   ptr_type& data() noexcept { return pix; }
   const ptr_type& data() const noexcept { return pix; }

   size_t memory_usage() const noexcept
   {
      return sizeof(ImageContainerT) + (owns_memory() ? n_bytes() : size_t(0));
   }

   bool empty() const noexcept { return size() == 0; }

   ptr_type row_ptr(unsigned y) noexcept { return pix + y * row_stride; }
   ptr_type row_ptr(unsigned y) const noexcept { return pix + y * row_stride; }
   ptr_type ptr(unsigned y) noexcept { return pix + y * row_stride; }
   ptr_type ptr(unsigned y) const noexcept { return pix + y * row_stride; }
   ptr_type row_ptr(int y) noexcept { return row_ptr(unsigned(y)); }
   ptr_type row_ptr(int y) const noexcept { return row_ptr(unsigned(y)); }
   ptr_type ptr(int y) noexcept { return ptr(unsigned(y)); }
   ptr_type ptr(int y) const noexcept { return ptr(unsigned(y)); }
   size_t row_bytes() const noexcept { return width * sizeof(T); }
   void zero() noexcept { memset(data(), 0, n_bytes()); }
   ImageContainerT<T>& fill(T val) { return set_all(val); }

   ptr_type begin() noexcept { return pix; }
   ptr_type end() noexcept { return pix + height * row_stride; }
   ptr_type begin() const noexcept { return pix; }
   ptr_type end() const noexcept { return pix + height * row_stride; }
   ptr_type cbegin() const noexcept { return pix; }
   ptr_type cend() const noexcept { return pix + height * row_stride; }

   ImageContainerT<T>& set_all(T val)
   {
      T* itr       = pix;
      const T* end = itr + height * row_stride;
      while(itr != end) *itr++ = val;
      return *this;
   }

   template<std::integral I>
   ImageContainerT<T>& resize(I new_width,
                              I new_height,
                              unsigned new_row_stride
                              = std::numeric_limits<unsigned>::max())
   {
      return set_size(new_width, new_height, new_row_stride);
   }

   template<std::integral I>
   ImageContainerT<T>& set_size(I i_new_width,
                                I i_new_height,
                                unsigned new_row_stride
                                = std::numeric_limits<unsigned>::max())
   {
      if(!owns) throw std::runtime_error("Must own memory to resize");
      static constexpr unsigned u_max = std::numeric_limits<unsigned>::max();

      if constexpr(std::is_signed<I>::value) {
         Expects(i_new_width >= 0);
         Expects(i_new_height >= 0);
      }
      Expects(size_t(i_new_width) < size_t(u_max));
      Expects(size_t(i_new_height) < size_t(u_max));

      const auto new_width  = unsigned(i_new_width);
      const auto new_height = unsigned(i_new_height);

      if(new_row_stride == u_max) new_row_stride = new_width;

      if(h * stride != new_height * new_row_stride) {
         delete[] pix;
         pix = nullptr;
         pix = new T[new_height * new_row_stride];
         if(pix == nullptr)
            throw std::runtime_error(format("failed to allocate {} bytes "
                                            "to image container",
                                            new_height * new_row_stride));
      }
      w      = new_width;
      h      = new_height;
      stride = new_row_stride;
      return *this;
   }

   ImageContainerT<T>& set_pix(unsigned new_width,
                               unsigned new_height,
                               unsigned new_row_stride,
                               const T* new_pix) noexcept(false)
   {
      if(owns) throw std::runtime_error("Must not own to flyweight");
      if(!memory_is_aligned<T>(new_pix))
         throw std::runtime_error("Memory is not correctly aligned");
      w      = new_width;
      h      = new_height;
      stride = new_row_stride;
      pix    = const_cast<T*>(new_pix);
      return *this;
   }

   bool in_bounds(int x, int y) const noexcept
   {
      return unsigned(x) < unsigned(width) && unsigned(y) < unsigned(height);
   }
   bool in_bounds(const Point2& p) const noexcept
   {
      return in_bounds(p.x, p.y);
   }
   bool in_bounds(const Vector2& p) const noexcept
   {
      return in_bounds(int(p.x), int(p.y));
   }

   bool out_of_bounds(int x, int y) const noexcept { return !in_bounds(x, y); }
   bool out_of_bounds(const Point2& p) const noexcept { return !in_bounds(p); }

   AABB bounds() const noexcept { return AABB(0, 0, w, h); }

   T& operator()(int x, int y) noexcept
   {
      assert(in_bounds(x, y));
      return pixels[unsigned(x) + unsigned(y) * row_stride];
   }
   const T& operator()(int x, int y) const noexcept
   {
      return const_cast<ImageContainerT<T>*>(this)->operator()(x, y);
   }

   T& operator()(const Point2& p) noexcept
   {
      return this->operator()(p.x, p.y);
   }
   const T& operator()(const Point2& p) const noexcept
   {
      return this->operator()(p.x, p.y);
   }

   T& operator()(unsigned x, unsigned y) noexcept
   {
      return this->operator()(int(x), int(y));
   }
   const T& operator()(unsigned x, unsigned y) const noexcept
   {
      return this->operator()(int(x), int(y));
   }

   T& operator()(real x, real y) noexcept
   {
      return this->operator()(int(x), int(y));
   }
   const T& operator()(real x, real y) const noexcept
   {
      return this->operator()(int(x), int(y));
   }

   T& operator()(const Vector2& p) noexcept
   {
      return this->operator()(p.x, p.y);
   }
   const T& operator()(const Vector2& p) const noexcept
   {
      return this->operator()(p.x, p.y);
   }

   ImageContainerT<T> cropped(int l, int t, int r, int b) const
   {
      const auto& im = *this;
      if(l < 0) l = 0;
      if(t < 0) t = 0;
      if(l >= int(im.width)) l = im.width;
      if(t >= int(im.height)) t = im.height;
      if(r <= l) r = l;
      if(b <= t) b = t;
      if(r >= int(im.width)) r = im.width;
      if(b >= int(im.height)) b = im.height;

      int w = r - l;
      int h = b - t;

      ImageContainerT<T> ret;
      ret.resize(w, h, w);

      for(int y = 0; y < h; ++y) {
         auto src = im.row_ptr(y + t) + l;
         auto dst = ret.row_ptr(y);
         memcpy(dst, src, w * ret.sizeof_T());
      }

      return ret;
   }

   ImageContainerT<T> cropped(const Point4& bounds) const
   {
      return cropped(bounds[0], bounds[1], bounds[2], bounds[3]);
   }

   auto minmax() const
   {
      T min_val = std::numeric_limits<T>::max();
      T max_val = std::numeric_limits<T>::lowest();
      auto end  = this->cend();
      for(auto ii = cbegin(); ii != end; ++ii) {
         if(*ii < min_val) min_val = *ii;
         if(*ii > max_val) max_val = *ii;
      }
      return std::pair<T, T>(min_val, max_val);
   }

   ImageContainerT<T>& normalize()
   {
      T min_val, max_val;
      std::tie(min_val, max_val) = minmax();
      const auto end             = this->cend();
      const real range_inv       = 1.0 / (max_val - min_val);
      for(auto ii = begin(); ii != end; ++ii) {
         *ii = (*ii - min_val) * range_inv;
      }
      return *this;
   }

   ImageContainerT<T>& normalise() { return normalize(); }

   void vertical_flip()
   {
      using std::swap; // Use ADL to get the right swap
      const auto half_h = height / 2;
      for(auto y = 0u; y < half_h; ++y) {
         auto ii = ptr(y);
         auto jj = ptr(height - y - 1);
         for(auto x = 0u; x < width; ++x) swap(ii[x], jj[x]);
      }
   }
};

template<typename T>
inline bool in_bounds(const ImageContainerT<T>& im, int x, int y) noexcept
{
   return im.in_bounds(x, y);
}

template<typename T>
inline bool in_bounds(const ImageContainerT<T>& im, Point2 x) noexcept
{
   return im.in_bounds(x);
}

typedef ImageContainerT<bool> BinaryImage;
typedef ImageContainerT<uint8_t> GreyImage;
typedef ImageContainerT<uint16_t> Grey16Image;
typedef ImageContainerT<uint32_t> ARGBImage;
typedef ImageContainerT<int16_t> Int16Image;
typedef ImageContainerT<int32_t> IntImage;
typedef ImageContainerT<float> FloatImage;
typedef ImageContainerT<double> DoubleImage;
typedef ImageContainerT<Vector2> Field;
typedef ImageContainerT<Vector2f> FloatField;
typedef ImageContainerT<Vector3f> FloatRayField;
typedef ImageContainerT<Vector3f> Vec3fImage;
typedef ImageContainerT<LAB> LABImage;
typedef ImageContainerT<Vector4f> Vec4fImage;
typedef GreyImage GrayImage;

ARGBImage decode_image(const vector<char>& bytes) noexcept(false);
cv::Mat decode_image_to_cv_mat(const vector<char>& bytes) noexcept(false);

ARGBImage grey_to_colour(const GreyImage& im);
inline ARGBImage grey_to_argb(const GreyImage& im)
{
   return grey_to_colour(im);
}

GreyImage colour_to_grey(const ARGBImage& im);
inline GreyImage argb_to_grey(const ARGBImage& im)
{
   return colour_to_grey(im);
}

ARGBImage field_to_colour(const Field& f);
inline ARGBImage field_to_argb(const Field& f) { return field_to_colour(f); }
GreyImage float_im_to_grey(const FloatImage& f,
                           std::function<float(float)> t = nullptr);
GreyImage float_im_to_grey(const FloatImage& f,
                           float min_val,
                           float max_val,
                           bool invert);
ARGBImage float_im_to_argb(const FloatImage& f,
                           std::function<float(float)> t = nullptr);
ARGBImage float_im_to_argb(const FloatImage& f,
                           float min_val,
                           float max_val,
                           bool invert = false);

ARGBImage float_im_positive_negative_argb(const FloatImage& im,
                                          float min_val,
                                          float max_val) noexcept;

GreyImage binary_im_to_grey(const BinaryImage& im);

// -- Grey16 images
Grey16Image float_im_to_grey16(const FloatImage& f,
                               float min_val,
                               float max_val,
                               bool invert = false);
FloatImage grey16_to_float_im(const Grey16Image& f,
                              float min_val,
                              float max_val,
                              bool invert = false);
ARGBImage grey16_im_to_argb(const Grey16Image& f);

// IF gl-twizzle is true, then input is converted RGBA => ARGB
ARGBImage vec4f_im_to_argb(const Vec4fImage& f, bool gl_twizzle);

Vec3fImage cv_to_LAB_vec3f_im(const cv::Mat& in);
LABImage cv_to_LAB_im(const cv::Mat& in);

ARGBImage int_image_to_argb(const IntImage& in,
                            std::function<uint32_t(int label)> kolour_f
                            = nullptr);
ARGBImage int16_image_to_argb(const Int16Image& in,
                              std::function<uint32_t(int label)> kolour_f
                              = nullptr);

LABImage argb_to_LAB_im(const ARGBImage& in);
Vec3fImage argb_to_LAB_vec3f_im(const ARGBImage& in);

ARGBImage LAB_vec3f_im_to_argb(const Vec3fImage& in);
LABImage LAB_vec3f_im_to_LAB(const Vec3fImage& in);

Vec3fImage LAB_im_to_LAB_vec3f(const LABImage& in);
ARGBImage LAB_im_to_argb(const LABImage& in);

void cv_to_argb(const cv::Mat& in, ARGBImage& argb);
void cv_to_grey(const cv::Mat& in, GreyImage& g);
void cv_to_float_im(const cv::Mat& in, FloatImage& f);
void write_cv_to_ptr(const cv::Mat& in, uint32_t* dst, bool h_to_be32 = false);

void cv_to_cv_grey16(const cv::Mat& in, cv::Mat& g);

inline ARGBImage cv_to_argb(const cv::Mat& in)
{
   ARGBImage argb;
   cv_to_argb(in, argb);
   return argb;
}

inline GreyImage cv_to_grey(const cv::Mat& in)
{
   GreyImage grey;
   cv_to_grey(in, grey);
   return grey;
}

cv::Mat grey_to_cv(const GreyImage& im);
cv::Mat argb_to_cv(const ARGBImage& im);
cv::Mat field_to_cv(const Field& im);
cv::Mat float_im_to_cv(const FloatImage& f,
                       std::function<float(float)> t = nullptr);

void convert_cv_to_cv16(const cv::Mat& in, cv::Mat& out) noexcept;

inline ARGBImage gray_to_color(const GreyImage& im)
{
   return grey_to_colour(im);
}
inline void cv_to_gray(const cv::Mat& in, GreyImage& g) { cv_to_grey(in, g); }

Field make_field(const cv::Mat& mat,
                 const unsigned blur_sz,
                 const real blur_sigma,
                 const bool normalize);

// ----------------------------------------------------------------- set (pixel)

inline void set(ARGBImage& im, int x, int y, uint32_t c)
{
   if(im.in_bounds(x, y)) im(x, y) = c;
}

inline void set(ARGBImage& im, Point2 xy, uint32_t c)
{
   set(im, xy.x, xy.y, c);
}

inline void set(ARGBImage& im, int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
   set(im, x, y, (uint32_t(r) << 16) | (uint32_t(g) << 8) | (uint32_t(b) << 0));
}

inline void set(ARGBImage& im, Point2 xy, uint8_t r, uint8_t g, uint8_t b)
{
   set(im, xy.x, xy.y, r, g, b);
}

inline void set(ARGBImage& im, int x, int y, double r, double g, double b)
{
   set(im, x, y, uint8_t(r * 255.0), uint8_t(g * 255.0), uint8_t(b * 255.0));
}

inline void set(ARGBImage& im, Point2 xy, double r, double g, double b)
{
   set(im,
       xy.x,
       xy.y,
       uint8_t(r * 255.0),
       uint8_t(g * 255.0),
       uint8_t(b * 255.0));
}

inline void set(ARGBImage& im, int x, int y, const Vector3& c)
{
   set(im, x, y, c.x, c.y, c.z);
}

inline void set(ARGBImage& im, Point2 xy, const Vector3& c)
{
   set(im, xy.x, xy.y, c.x, c.y, c.z);
}

// --------------------------------------------------------------------- drawing

void fill_rect(ARGBImage& argb, Point2 top_left, uint32_t k, int w, int h);

void fill_circle(ARGBImage& argb,
                 Point2 p0,
                 uint32_t k,
                 int radius,
                 float alpha = 1.0f);
void fill_circle(cv::Mat& im,
                 Point2 p0,
                 uint32_t k,
                 int radius,
                 float alpha = 1.0f);

void outline_circle(cv::Mat& im, Vector2 p0, uint32_t k, real radius);

void outline_circle(ARGBImage& im, Vector2 p0, uint32_t k, real radius);

void draw_cross(ARGBImage& argb, Point2 p0, uint32_t k, int radius);

void draw_cross_thick(ARGBImage& argb, Point2 p0, uint32_t k, int radius);

void draw_square(ARGBImage& argb,
                 Point2 p0,
                 uint32_t k,
                 int radius,
                 float alpha = 1.0f);

std::pair<Vector2, Vector2> render_circle(const float z,
                                          const bool full) noexcept;

void render_quad_2d(ARGBImage& argb,
                    const array<Vector2, 4>& Xs,
                    uint32_t k) noexcept;

// ----------------------------------------------------------------------

void blit(const cv::Mat& im, ARGBImage& argb, const AABBi& aabb) noexcept;

cv::Mat crop_to_cv(const ARGBImage& argb, const AABBi& aabb) noexcept;

// ---------------------------------------------------------------------- hsplit

template<typename T>
void hsplit(const ImageContainerT<T>& im,
            ImageContainerT<T>& out1,
            ImageContainerT<T>& out2)
{
   out1.resize(im.width / 2, im.height);
   out2.resize(im.width - out1.width, im.height);

   for(auto y = 0u; y < im.height; ++y) {
      const auto src = im.ptr(y);
      auto dst1      = out1.ptr(y);
      auto dst2      = out2.ptr(y);
      memcpy(dst1, src + 0, out1.row_bytes());
      memcpy(dst2, src + out1.width, out2.row_bytes());
   }
}

// ------------------------------------------------------------------------ hcat

template<typename T>
ImageContainerT<T> hcat(const ImageContainerT<T>& a,
                        const ImageContainerT<T>& b)
{
   Expects(a.height == b.height);
   ImageContainerT<T> ret;
   ret.resize(a.width + b.width, a.height);
   for(auto y = 0u; y < a.height; ++y) {
      auto dst  = ret.ptr(y);
      auto src1 = a.ptr(y);
      auto src2 = b.ptr(y);
      memcpy(dst + 0, src1, a.row_bytes());
      memcpy(dst + a.width, src2, b.row_bytes());
   }
   return ret;
}

// ------------------------------------------------------------------------ vcat

template<typename T>
ImageContainerT<T> vcat(const ImageContainerT<T>& a,
                        const ImageContainerT<T>& b)
{
   Expects(a.width == b.width);
   ImageContainerT<T> ret;
   ret.resize(a.width, a.height + b.height);
   for(auto y = 0u; y < a.height; ++y)
      memcpy(ret.ptr(y), a.ptr(y), a.row_bytes());
   for(auto y = 0u; y < b.height; ++y)
      memcpy(ret.ptr(y + a.height), b.ptr(y), b.row_bytes());
   return ret;
}

// ---------------------------------------------------------------------- dilate
//
template<typename T>
inline ImageContainerT<T> dilate(const ImageContainerT<T>& im, const int f)
{
   ImageContainerT<T> i;
   i.resize(im.width * f, im.height * f);
   for(auto y = 0u; y < im.height; ++y) {
      for(auto x = 0u; x < im.width; ++x) {
         const auto k = im(x, y);
         for(auto dy = 0; dy < f; ++dy)
            for(auto dx = 0; dx < f; ++dx) i(x * f + dx, y * f + dy) = k;
      }
   }

   return i;
}

// ---------------------------------------------------------------- to/from json
//
string to_json_string(const ARGBImage& argb);
void from_json_value(ARGBImage& argb, const Json::Value&) noexcept(false);
Json::Value to_json_value(const ARGBImage& argb);

string to_json_string(const LABImage& argb);
void from_json_value(LABImage& argb, const Json::Value&) noexcept(false);
Json::Value to_json_value(const LABImage& argb);

// ------------------------------------------------------------------ make-patch
//
LABImage make_patch(const unsigned w, // patch width (pixels), must be > 1
                    const unsigned h, // patch height (pixels), must be > 0
                    const Vector2& A, // Vector from A->B
                    const Vector2& B,
                    const cv::Mat& im) noexcept(false);

LABImage make_patch(const unsigned w, // patch width (pixels), must be > 1
                    const unsigned h, // patch height (pixels), must be > 0
                    const Vector2& A, // Vector from A->B
                    const Vector2& B,
                    const LABImage& im) noexcept(false);

float dot_product(const LABImage& A, const LABImage& B) noexcept;

} // namespace perceive
