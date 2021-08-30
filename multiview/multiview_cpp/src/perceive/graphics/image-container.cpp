
/**
 * $250 deductable
 * $238.79 co-insurance
 * Claim-id 17184213-01-01
 */

#include "stdinc.hpp"

#include "bresenham.hpp"
#include "colour-set.hpp"
#include "cv-helpers.hpp"
#include "image-container.hpp"

#include "perceive/io/fp-io.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/utils/base64.hpp"
#include "perceive/utils/cuda-spec.hpp"

#include "cuda/perceive/graphics/rgb-to-lab.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace perceive
{
void write_cv_to_ptr(const cv::Mat& in, uint32_t* dst, bool h_to_be32)
{
   auto cols = in.cols;
   auto rows = in.rows;
   if(in.type() == CV_8UC3) {
      for(auto y = 0; y < rows; ++y) {
         const uint8_t* row = in.ptr(y); // BGR format
         for(auto x = 0; x < cols; ++x) {
            uint32_t b = *row++;
            uint32_t g = *row++;
            uint32_t r = *row++;
            *dst       = (r << 16) | (g << 8) | (b << 0);
            if(h_to_be32) *dst = htobe32(*dst);
            dst++;
         }
      }
   } else if(in.type() == CV_8UC1 || in.type() == CV_8U) {
      for(auto y = 0; y < rows; ++y) {
         const uint8_t* row = in.ptr(y);
         for(auto x = 0; x < cols; ++x) {
            uint32_t v = *row++;
            *dst       = (v << 16) | (v << 8) | (v << 0);
            if(h_to_be32) *dst = htobe32(*dst);
            dst++;
         }
      }
   } else {
      throw std::runtime_error("Can only convert CV_8UC1 or CV_8UC3");
   }
}

void cv_to_argb(const cv::Mat& in, ARGBImage& g)
{
   auto cols = in.cols;
   auto rows = in.rows;
   g.set_size(unsigned(in.cols), unsigned(in.rows), unsigned(cols));
   write_cv_to_ptr(in, g.data());
}

void cv_to_grey(const cv::Mat& in, GreyImage& g)
{
   auto cols = in.cols;
   auto rows = in.rows;
   g.set_size(unsigned(cols), unsigned(rows), unsigned(cols));
   uint8_t* dst = g.data();

   if(in.type() == CV_8UC3) {
      for(unsigned y = 0; y < g.height; ++y) {
         const uint8_t* row = in.ptr(int(y)); // BGR format
         for(unsigned x = 0; x < g.width; ++x) {
            float b = *row++;
            float g = *row++;
            float r = *row++;
            float v = 0.21f * r + 0.72f * g + 0.07f * b;
            *dst++  = uint8_t(clamp<float>(v, 0.0f, 255.0f));
         }
      }
   } else if(in.type() == CV_16UC1 || in.type() == CV_16U) {
      for(unsigned y = 0; y < g.height; ++y) {
         const uint16_t* row
             = reinterpret_cast<const uint16_t*>(in.ptr(int(y)));
         for(unsigned x = 0; x < g.width; ++x) *dst++ = uint8_t(*row++ / 255);
      }
   } else if(in.type() == CV_8UC1 || in.type() == CV_8U) {
      for(unsigned y = 0; y < g.height; ++y) {
         const uint8_t* row = in.ptr(int(y));
         for(unsigned x = 0; x < g.width; ++x) *dst++ = *row++;
      }
   } else {
      throw std::runtime_error("Can only convert CV_8UC1 or CV_8UC3");
   }
}

// ----------------------------------------------------- convert to greyscale-16
//
void cv_to_cv_grey16(const cv::Mat& in, cv::Mat& g)
{
   const int w = in.cols;
   const int h = in.rows;

   if(g.cols != w or g.rows != h
      or (g.type() != CV_16U and g.type() != CV_16UC1)) {
      g = cv::Mat(h, w, CV_16U);
   }

   const float uint16_max_f = std::numeric_limits<uint16_t>::max();

   if(in.type() == CV_8UC3) {
      for(auto y = 0; y < h; ++y) {
         const uint8_t* row = in.ptr(y); // BGR format
         uint16_t* dst      = reinterpret_cast<uint16_t*>(g.ptr(y));
         for(auto x = 0; x < w; ++x) {
            float b = *row++;
            float g = *row++;
            float r = *row++;
            float v = 0.21f * r + 0.72f * g + 0.07f * b;
            *dst++  = uint16_t(clamp<float>(v, 0.0f, uint16_max_f));
         }
      }
   } else if(in.type() == CV_16UC1 || in.type() == CV_16U) {
      for(auto y = 0; y < h; ++y) {
         const uint16_t* row = reinterpret_cast<const uint16_t*>(in.ptr(y));
         uint16_t* dst       = reinterpret_cast<uint16_t*>(g.ptr(y));
         for(auto x = 0; x < w; ++x) *dst++ = *row++;
      }
   } else if(in.type() == CV_8UC1 || in.type() == CV_8U) {
      for(auto y = 0; y < h; ++y) {
         const uint8_t* row = in.ptr(y);
         uint16_t* dst      = reinterpret_cast<uint16_t*>(g.ptr(y));
         for(auto x = 0; x < w; ++x) *dst++ = *row++ * 255;
      }
   } else {
      throw std::runtime_error("Can only convert CV_8UC1 or CV_8UC3");
   }
}

void cv_to_float_im(const cv::Mat& in, FloatImage& f)
{
   if(in.type() == CV_32F) {
      f.set_size(unsigned(in.cols), unsigned(in.rows));
      for(auto y = 0u; y < f.height; ++y) {
         const float* row = in.ptr<const float>(int(y)); // BGR format
         float* dst       = f.row_ptr(y);
         for(auto x = 0u; x < f.width; ++x) *dst++ = *row++;
      }
   } else if(in.type() == CV_64F) {
      f.set_size(unsigned(in.cols), unsigned(in.rows));
      for(auto y = 0u; y < f.height; ++y) {
         const double* row = in.ptr<const double>(int(y)); // BGR format
         float* dst        = f.row_ptr(y);
         for(auto x = 0u; x < f.width; ++x) *dst++ = float(*row++);
      }
   } else {
      LOG_ERR(format("types = {}, CV_32F = {}", in.type(), CV_32F));
      throw std::runtime_error("Can only convert CV_32F and CV64F");
   }
}

// ----------------------------------------------------------- int-image-to-argb

ARGBImage int_image_to_argb(const IntImage& in,
                            std::function<uint32_t(int label)> kolour_f)
{
   ARGBImage argb;
   argb.resize(in.width, in.height);

   if(!kolour_f)
      kolour_f = [](int label) { return colour_set_1(unsigned(label)); };

   for(auto y = 0u; y < in.height; ++y)
      for(auto x = 0u; x < in.width; ++x) {
         auto label = in(x, y);
         argb(x, y) = (label == -1) ? k_white : kolour_f(label);
      }

   return argb;
}

ARGBImage int16_image_to_argb(const Int16Image& in,
                              std::function<uint32_t(int label)> kolour_f)
{
   ARGBImage argb;
   argb.resize(in.width, in.height);

   if(!kolour_f)
      kolour_f = [](int label) { return colour_set_1(unsigned(label)); };

   for(auto y = 0u; y < in.height; ++y)
      for(auto x = 0u; x < in.width; ++x) {
         auto label = in(x, y);
         argb(x, y) = (label == -1) ? k_white : kolour_f(label);
      }

   return argb;
}

// ------------------------------------------------------------------- To OpenCv

cv::Mat grey_to_cv(const GreyImage& im)
{
   cv::Mat res(int(im.height), int(im.width), CV_8UC1);
   for(unsigned y = 0; y < im.height; ++y)
      memcpy(res.ptr(int(y)), im.row_ptr(y), im.row_bytes());
   return res;
}

cv::Mat argb_to_cv(const ARGBImage& im)
{
   cv::Mat res(int(im.height), int(im.width), CV_8UC3);

   const uint32_t* src = im.pixels;
   for(unsigned y = 0; y < im.height; ++y) {
      uint8_t* dst = res.ptr(int(y));
      for(unsigned x = 0; x < im.width; ++x) {
         *dst++ = ((*src) >> 0) & 0xff;  // b
         *dst++ = ((*src) >> 8) & 0xff;  // g
         *dst++ = ((*src) >> 16) & 0xff; // r
         src++;
      }
   }

   return res;
}

ARGBImage field_to_colour(const Field& f)
{
   ARGBImage argb;
   cv_to_argb(field_to_cv(f), argb);
   return argb;
}

inline uint32_t
angle_to_colour_360(double radians, double sat = 1.0, double val = 1.0)
{
   auto degrees = to_degrees(angle_normalise2(radians));
   Vector3 hsv(degrees, sat, val);
   return vector3_to_kolour(hsv_to_rgb(hsv));
}

cv::Mat field_to_cv(const Field& im)
{
   cv::Mat res(int(im.height), int(im.width), CV_8UC3);

   // Find the max magnitude
   auto max_mag = 0.0;
   {
      vector<real> mags(im.width * im.height);
      auto src     = im.pixels;
      auto end     = src + (im.height * im.width);
      unsigned pos = 0;
      while(src < end) mags[pos++] = (*src++).norm();
      real av_mag = 0.0;
      for(const auto& X : mags) av_mag += X;
      av_mag /= real(mags.size());
      real sum_abs = 0.0;
      for(const auto& X : mags) sum_abs += fabs(X - av_mag);
      max_mag = av_mag + 3.0 * (sum_abs / real(mags.size()));
   }

   for(unsigned y = 0u; y < im.height; ++y) {
      auto src     = im.ptr(y);
      uint8_t* dst = res.ptr(int(y));
      for(unsigned x = 0u; x < im.width; ++x) {
         const auto& v  = src[x];
         auto quadrance = v.quadrance();
         uint32_t k     = k_black;
         if(quadrance > 1e-9) {
            auto norm      = sqrt(quadrance);
            auto theta     = atan2(v.y, v.x);
            auto intensity = clamp(norm / max_mag, 0.0, 1.0);
            k              = angle_to_colour_360(theta, 1.0, intensity);
         }
         *dst++ = (k >> 0) & 0xff;  // b
         *dst++ = (k >> 8) & 0xff;  // g
         *dst++ = (k >> 16) & 0xff; // r
      }
   }

   return res;
}

// ------------------------------------------------------------------------ Save

template<> void ImageContainerT<bool>::save(const char* filename) const
{
   cv::Mat im(int(height), int(width), CV_8UC1);
   for(unsigned y = 0; y < height; ++y) {
      uint8_t* dst = im.ptr(int(y));
      for(unsigned x = 0; x < width; ++x)
         *dst++ = this->operator()(x, y) ? 255 : 0;
   }

   std::vector<int> compression_params(2);
   compression_params[0] = cv::IMWRITE_PNG_COMPRESSION;
   compression_params[1] = 9;
   imwrite(filename, im, compression_params);
}

template<> void ImageContainerT<uint8_t>::save(const char* filename) const
{
   cv::Mat im(int(height), int(width), CV_8UC1);
   for(unsigned y = 0; y < height; ++y)
      memcpy(im.ptr(int(y)), row_ptr(y), row_bytes());

   std::vector<int> compression_params(2);
   compression_params[0] = cv::IMWRITE_PNG_COMPRESSION;
   compression_params[1] = 9;
   imwrite(filename, im, compression_params);
}

template<> void ImageContainerT<uint32_t>::save(const char* filename) const
{
   cv::Mat im(int(height), int(width), CV_8UC3);

   const uint32_t* src = pix;
   for(unsigned y = 0; y < height; ++y) {
      uint8_t* dst = im.ptr(int(y));
      for(unsigned x = 0; x < width; ++x) {
         *dst++ = ((*src) >> 0) & 0xff;  // b
         *dst++ = ((*src) >> 8) & 0xff;  // g
         *dst++ = ((*src) >> 16) & 0xff; // r
         src++;
      }
   }

   if(!imwrite(filename, im))
      throw std::runtime_error(format("OpenCV failed to save '{}'", filename));
}

template<> void ImageContainerT<float>::save(const char* filename) const
{
   FILE* fp = fopen(filename, "wb");
   if(fp == nullptr)
      throw std::runtime_error(
          format("failed to open '{}' for writing", filename));

   save_uint(fp, width);
   save_uint(fp, height);
   for(auto y = 0u; y < height; ++y)
      for(auto x = 0u; x < width; ++x)
         save_real(fp, real(this->operator()(x, y)));

   fclose(fp);
}

// ------------------------------------------------------------------------ Load

template<> GreyImage GreyImage::load(const char* filename)
{
   cv::Mat im = cv::imread(filename, cv::IMREAD_GRAYSCALE);
   if(im.empty())
      throw std::runtime_error(
          format("OpenCV failed to load image '{}'", filename));
   return cv_to_grey(im);
}

template<> ARGBImage ARGBImage::load(const char* filename)
{
   cv::Mat im = cv::imread(filename, cv::IMREAD_COLOR);
   if(im.empty())
      throw std::runtime_error(
          format("OpenCV failed to load image '{}'", filename));
   return cv_to_argb(im);
}

template<> FloatImage FloatImage::load(const char* filename)
{
   FloatImage g(true);

   FILE* fp = fopen(filename, "rb");
   if(fp == nullptr)
      throw std::runtime_error(
          format("failed to open '{}' for reading", filename));

   unsigned w = 0, h = 0;
   try {
      load_uint(fp, w);
      load_uint(fp, h);
      if(w * h > square(8000))
         throw std::runtime_error(format("cowardly refusing to create "
                                         "[{}x{}] image",
                                         w,
                                         h));
      g.resize(w, h);

      real val{0.0};
      for(auto y = 0u; y < g.height; ++y) {
         for(auto x = 0u; x < g.width; ++x) {
            load_real(fp, val);
            g(x, y) = float(val);
         }
      }
   } catch(std::runtime_error& e) {
      throw std::runtime_error(format("file '{}' corrupt", filename));
   }

   return g;
}

// ----------------------------------------------------------- Force compilation
// -- This anonymous function forces compilation of template code into module --
namespace
{
   static void force_template_inst()
   {
      GreyImage g = GreyImage::load("zap.png");
      g.save("zap.png");

      ARGBImage i = ARGBImage::load("zap.png");
      i.save("zap.png");
   }
} // namespace

// ----------------------------------------------------------------- Colour/Grey

ARGBImage grey_to_colour(const GreyImage& im)
{
   ARGBImage ret;
   ret.set_size(im.width, im.height, im.width);
   uint32_t* dst = ret.data();
   for(unsigned y = 0; y < im.height; ++y) {
      const uint8_t* row = im.row_ptr(y);
      for(unsigned x = 0; x < im.width; ++x) {
         uint32_t g = *row++;
         *dst++     = (g << 16) | (g << 8) | (g << 0);
      }
   }

   return ret;
}

GreyImage colour_to_grey(const ARGBImage& im)
{
   GreyImage ret;
   ret.set_size(im.width, im.height, im.width);
   uint8_t* dst = ret.data();
   for(unsigned y = 0; y < im.height; ++y) {
      const uint32_t* ptr = im.row_ptr(y);
      const uint32_t* end = ptr + im.width;
      while(ptr != end) {
         float r = (*ptr >> 16) & 0xff;
         float g = (*ptr >> 8) & 0xff;
         float b = (*ptr >> 0) & 0xff;
         *dst++
             = uint8_t(std::min(0.299f * r + 0.587f * g + 0.114f * b, 255.0f));
         ptr++;
      }
   }

   return ret;
}

// ----------------------------------------------------------------- Float to CV

template<typename T>
static void
float_im_to_greyT(const FloatImage& f, std::function<float(float)> t, T& g)
{
   if(t == nullptr) t = [&](float x) { return x; };

   auto minmax = Vector2::range();
   for(auto y = 0u; y < f.height; ++y) {
      auto row_ptr = f.row_ptr(y);
      for(auto x = 0u; x < f.width; ++x)
         minmax.union_value(real(t(*row_ptr++)));
   }

   const bool normalize = minmax.x < 0.0 or minmax.y > 1.0;
   const auto range     = !normalize ? 1.0 : (minmax.y - minmax.x);
   const auto minx      = !normalize ? 0.0 : minmax.x;

   for(auto y = 0u; y < f.height; ++y) {
      auto src_ptr = f.row_ptr(y);
      auto dst_ptr = g.ptr(y);
      for(auto x = 0u; x < f.width; ++x)
         *dst_ptr++ = uint8_t(255.0 * (real(t(*src_ptr++)) - minx) / range);
   }
}

GreyImage
float_im_to_grey(const FloatImage& f, float min_val, float max_val, bool invert)
{
   const auto minval    = std::min(min_val, max_val);
   const auto maxval    = std::max(min_val, max_val);
   const auto range     = (max_val - min_val);
   const auto range_inv = (range < 1e-20f) ? 0.0f : (1.0f / range);
   GreyImage g;
   g.resize(f.width, f.height, f.width);
   for(auto y = 0u; y < f.height; ++y) {
      for(auto x = 0u; x < f.width; ++x) {
         auto val = std::clamp<int>(
             int(255.0f * (f(x, y) - min_val) * range_inv), 0, 255);
         g(x, y) = uint8_t((invert) ? (255 - val) : val);
      }
   }
   return g;
}

GreyImage float_im_to_grey(const FloatImage& f, std::function<float(float)> t)
{
   GreyImage g;
   g.resize(f.width, f.height);
   float_im_to_greyT(f, t, g);
   return g;
}

ARGBImage float_im_to_argb(const FloatImage& f, std::function<float(float)> t)
{
   return grey_to_colour(float_im_to_grey(f, t));
}

ARGBImage
float_im_to_argb(const FloatImage& f, float min_val, float max_val, bool invert)
{
   return grey_to_colour(float_im_to_grey(f, min_val, max_val, invert));
}

ARGBImage float_im_positive_negative_argb(const FloatImage& im,
                                          float min_val,
                                          float max_val) noexcept
{
   ARGBImage argb;
   const int w = int(im.width);
   const int h = int(im.height);
   argb.resize(im.width, im.height);

   const real v  = 1.0; // value
   const real h0 = 0.0;
   const real h1 = 127.0;

   for(auto y = 0; y < h; ++y) {
      const float* src = im.row_ptr(unsigned(y));
      const float* end = src + w;
      uint32_t* dst    = argb.ptr(unsigned(y));
      while(src != end) {
         const float x = std::clamp(*src++, min_val, max_val);
         if(x == 0.0f) {
            *dst++ = k_white;
         } else if(x < 0.0f) {
            *dst++ = hsv_to_rgb_uint32(h0, real(x / min_val), v);
         } else if(x > 0.0f) {
            *dst++ = hsv_to_rgb_uint32(h1, real(x / max_val), v);
         } else { // NAN
            *dst++ = k_black;
         }
      }
   }

   return argb;
}

Grey16Image float_im_to_grey16(const FloatImage& f,
                               float min_val,
                               float max_val,
                               bool invert)
{
   constexpr int s_high = std::numeric_limits<uint16_t>::max();

   const auto minval    = std::min(min_val, max_val);
   const auto maxval    = std::max(min_val, max_val);
   const auto range     = (max_val - min_val);
   const auto range_inv = (range < 1e-20f) ? 0.0f : (1.0f / range);
   Grey16Image g;
   g.resize(f.width, f.height);
   for(auto y = 0u; y < f.height; ++y) {
      for(auto x = 0u; x < f.width; ++x) {
         double perc = double(
             std::clamp<float>((f(x, y) - min_val) * range_inv, 0.0f, 1.0f));
         if(invert) perc = 1.0 - perc;
         g(x, y) = uint16_t(s_high * perc);
      }
   }
   return g;
}

GreyImage binary_im_to_grey(const BinaryImage& im)
{
   GreyImage g;
   g.resize(im.width, im.height);
   for(auto y = 0u; y < im.height; ++y)
      for(auto x = 0u; x < im.width; ++x) g(x, y) = im(x, y) ? 255 : 0;
   return g;
}

FloatImage grey16_to_float_im(const Grey16Image& f,
                              float min_val,
                              float max_val,
                              bool invert)
{
   constexpr float s_high = 1.0f / float(std::numeric_limits<uint16_t>::max());

   const auto minval = std::min(min_val, max_val);
   const auto maxval = std::max(min_val, max_val);
   const auto range  = (max_val - min_val);
   FloatImage g;
   g.resize(f.width, f.height);
   for(auto y = 0u; y < f.height; ++y) {
      for(auto x = 0u; x < f.width; ++x) {
         // percent
         float perc = std::clamp<float>(f(x, y) * float(s_high), 0.0f, 1.0f);
         if(invert) perc = 1.0f - perc;
         g(x, y) = float(perc * range + min_val);
      }
   }
   return g;
}

ARGBImage grey16_im_to_argb(const Grey16Image& im)
{
   ARGBImage ret;
   ret.set_size(im.width, im.height);
   uint32_t* dst = ret.data();
   for(unsigned y = 0; y < im.height; ++y) {
      const uint16_t* row = im.row_ptr(y);
      for(unsigned x = 0; x < im.width; ++x) {
         const uint32_t g = (*row++) >> 8;
         *dst++           = (g << 16) | (g << 8) | (g << 0);
      }
   }
   return ret;
}

cv::Mat float_im_to_cv(const FloatImage& f, std::function<float(float)> t)
{
   cv::Mat g(int(f.height), int(f.width), CV_32F);
   for(auto y = 0u; y < f.height; ++y) {
      auto src_ptr = f.row_ptr(y);
      auto dst_ptr = g.ptr<float>(int(y));
      for(auto x = 0u; x < f.width; ++x) {
         float val  = *src_ptr++;
         *dst_ptr++ = t ? t(val) : val;
      }
   }
   return g;
}

ARGBImage vec4f_im_to_argb(const Vec4fImage& f, bool gl_twizzle)
{
   ARGBImage o(f.width, f.height);
   for(auto y = 0u; y < f.height; ++y) {
      const auto src = f.ptr(y);
      const auto dst = o.ptr(y);
      for(auto x = 0u; x < f.width; ++x) {
         const auto k = f(x, y) * 255.0f;
         if(gl_twizzle)
            o(x, y) = make_colour(
                uint8_t(k(3)), uint8_t(k(0)), uint8_t(k(1)), uint8_t(k(2)));
         else
            o(x, y) = make_colour(
                uint8_t(k(0)), uint8_t(k(1)), uint8_t(k(2)), uint8_t(k(3)));
      }
   }
   return o;
}

// --------------------------------------------------------------- make field --

Field make_field(const cv::Mat& mat,
                 const unsigned blur_sz,
                 const real blur_sigma,
                 const bool normalize)
{
   unsigned w = unsigned(mat.cols);
   unsigned h = unsigned(mat.rows);

   const cv::Mat* grey_ptr = &mat;
   cv::Mat grey;
   if(mat.channels() > 1) {
      cv::cvtColor(mat, grey, cv::COLOR_BGR2GRAY);
      grey_ptr = &grey;
   }

   // Blurred image helps stabilize edge energy
   cv::Mat blurred;
   {
      auto sz = cv::Size(int(blur_sz), int(blur_sz));
      cv::GaussianBlur(
          *grey_ptr, blurred, sz, blur_sigma, blur_sigma, cv::BORDER_DEFAULT);
   }

   // Now calculate the edge energy...
   cv::Mat grad_x, grad_y;
   {
      cv::Scharr(blurred, grad_x, CV_32F, 1, 0);
      cv::Scharr(blurred, grad_y, CV_32F, 0, 1);
   }

   // Turn into a field
   Field scharr;
   {
      scharr.resize(w, h, w);
      for(auto y = 0; y < blurred.rows; ++y) {
         auto dst   = scharr.ptr(unsigned(y));
         auto src_x = reinterpret_cast<const float*>(grad_x.ptr(y));
         auto src_y = reinterpret_cast<const float*>(grad_y.ptr(y));
         for(auto x = 0; x < blurred.cols; ++x)
            dst[x] = to_vec2(Vector2f(src_x[x], src_y[x]));
      }
   }

   if(normalize) {
      auto max_norm = 0.0;
      for(auto ii = scharr.begin(); ii != scharr.end(); ++ii) {
         auto norm = ii->norm();
         if(norm > max_norm) max_norm = norm;
      }

      const auto max_norm_inv = 1.0 / max_norm;
      for(auto ii = scharr.begin(); ii != scharr.end(); ++ii)
         *ii *= max_norm_inv;
   }

   return scharr;
}

ARGBImage LAB_im_to_argb(const LABImage& in)
{
   ARGBImage out;
   out.resize(in.width, in.height, in.width);
   for(auto y = 0u; y < in.height; ++y)
      for(auto x = 0u; x < in.width; ++x) out(x, y) = LAB_to_kolour(in(x, y));
   return out;
}

LABImage argb_to_LAB_im(const ARGBImage& in)
{
   LABImage out;
   out.resize(in.width, in.height, in.width);
   for(auto y = 0u; y < in.height; ++y)
      for(auto x = 0u; x < in.width; ++x) out(x, y) = kolour_to_LAB(in(x, y));
   return out;
}

Vec3fImage argb_to_LAB_vec3f_im(const ARGBImage& in)
{
   static cuda::ARGBToLABConversion lab_conversion;

   Vec3fImage out;
   if(cuda::cuda_is_available()) {
      if(in.size() > 0) out = lab_conversion.convert(in);
   } else {
      out.resize(in.width, in.height, in.width);
      for(auto y = 0u; y < in.height; ++y)
         for(auto x = 0u; x < in.width; ++x)
            out(x, y) = LAB_to_LAB_vec3f(kolour_to_LAB(in(x, y)));
   }
   return out;
}

ARGBImage LAB_vec3f_im_to_argb(const Vec3fImage& in)
{
   ARGBImage out;
   out.resize(in.width, in.height, in.width);
   for(auto y = 0u; y < in.height; ++y)
      for(auto x = 0u; x < in.width; ++x)
         out(x, y) = LAB_to_kolour(vec3f_LAB_to_LAB(in(x, y)));
   return out;
}

Vec3fImage cv_to_LAB_vec3f_im(const cv::Mat& in)
{
   Vec3fImage out;
   out.resize(unsigned(in.cols), unsigned(in.rows), unsigned(in.cols));
   if(in.type() == CV_8UC3) {
      for(auto y = 0u; y < out.height; ++y) {
         const cv::Vec3b* row = in.ptr<cv::Vec3b>(int(y));
         for(auto x = 0u; x < out.width; ++x)
            out(x, y) = LAB_to_LAB_vec3f(vec3b_to_LAB(*row++));
      }
   } else if(in.type() == CV_8UC1 || in.type() == CV_8U) {
      for(auto y = 0u; y < out.height; ++y) {
         const uint8_t* row = in.ptr(int(y));
         for(auto x = 0u; x < out.width; ++x)
            out(x, y) = LAB_to_LAB_vec3f(kolour_to_LAB(grey_to_rgb(*row++)));
         //            out(x, y) = to_vec3f(rgb_to_lab(grey_to_rgb(*row++)));
      }
   } else {
      throw std::runtime_error("Can only convert CV_8UC1 or CV_8UC3");
   }

   return out;
}

LABImage cv_to_LAB_im(const cv::Mat& in)
{
   LABImage out;
   out.resize(unsigned(in.cols), unsigned(in.rows), unsigned(in.cols));
   if(in.type() == CV_8UC3) {
      for(auto y = 0u; y < out.height; ++y) {
         const cv::Vec3b* row = in.ptr<cv::Vec3b>(int(y));
         for(auto x = 0u; x < out.width; ++x) out(x, y) = vec3b_to_LAB(*row++);
      }
   } else if(in.type() == CV_8UC1 || in.type() == CV_8U) {
      for(auto y = 0u; y < out.height; ++y) {
         const uint8_t* row = in.ptr(int(y));
         for(auto x = 0u; x < out.width; ++x)
            out(x, y) = kolour_to_LAB(grey_to_rgb(*row++));
      }
   } else {
      throw std::runtime_error("Can only convert CV_8UC1 or CV_8UC3");
   }

   return out;
}

LABImage LAB_vec3f_im_to_LAB(const Vec3fImage& in)
{
   LABImage out;
   out.resize(in.width, in.height, in.width);
   for(auto y = 0u; y < in.height; ++y)
      for(auto x = 0u; x < in.width; ++x)
         out(x, y) = vec3f_LAB_to_LAB(in(x, y));
   return out;
}

Vec3fImage LAB_im_to_LAB_vec3f(const LABImage& in)
{
   Vec3fImage out;
   out.resize(in.width, in.height, in.width);
   for(auto y = 0u; y < in.height; ++y)
      for(auto x = 0u; x < in.width; ++x)
         out(x, y) = LAB_to_LAB_vec3f(in(x, y));
   return out;
}

// ---------------------------------------------------------------- to/from json
//
template<typename T> string to_json_string_T(const T& argb)
{
   return format("{{ \"width\": {}, \"height\": {}, \"stride\": {}, "
                 "\"data\": {} }}",
                 argb.width,
                 argb.height,
                 argb.row_stride,
                 json_encode(base64_encode(argb.data(), argb.n_bytes())));
}

template<typename T> void from_json_value_T(T& argb, const Json::Value& o)
{
   T x;

   const string op = "decoding ARGBImage"s;

   if(o.type() != Json::objectValue)
      throw std::runtime_error("expected object value when decode ARGBImage");
   const auto w       = json_load_key<int>(o, "width", op);
   const auto h       = json_load_key<int>(o, "height", op);
   const auto stride  = json_load_key<int>(o, "stride", op);
   const auto dat_str = json_load_key<string>(o, "data", op);

   if(w < 0) throw std::runtime_error(format("invalid width while {}", op));
   if(h < 0) throw std::runtime_error(format("invalid height while {}", op));
   if(stride < w)
      throw std::runtime_error(format("invalid stride while {}", op));
   if(!is_base64(dat_str))
      throw std::runtime_error(
          format("expected base64 data in 'data' while {}", op));

   x.resize(unsigned(w), unsigned(h), unsigned(stride));

   const auto decode_len = base64_decode_length(dat_str);
   if(decode_len != x.n_bytes())
      throw std::runtime_error(format("invalid image data while {}", op));

   base64_decode(dat_str, x.data());

   argb = x;
}

template<typename T> Json::Value to_json_value_T(const T& argb)
{
   Json::Value o{Json::objectValue};
   o["width"]  = argb.width;
   o["height"] = argb.height;
   o["stride"] = argb.row_stride;
   o["data"]   = base64_encode(argb.data(), argb.n_bytes());
   return o;
}

string to_json_string(const ARGBImage& argb) { return to_json_string_T(argb); }
string to_json_string(const LABImage& argb) { return to_json_string_T(argb); }

void from_json_value(ARGBImage& argb, const Json::Value& o) noexcept(false)
{
   from_json_value_T(argb, o);
}

void from_json_value(LABImage& argb, const Json::Value& o) noexcept(false)
{
   from_json_value_T(argb, o);
}

Json::Value to_json_value(const ARGBImage& argb)
{
   return to_json_value_T(argb);
}

Json::Value to_json_value(const LABImage& argb)
{
   return to_json_value_T(argb);
}

// ------------------------------------------------------------------- draw-rect

void fill_rect(ARGBImage& argb, Point2 top_left, uint32_t k, int w, int h)
{
   for(int dy = 0; dy < h; ++dy) {
      for(int dx = 0; dx < w; ++dx) {
         const auto p = top_left + Point2(dx, dy);
         if(argb.in_bounds(p)) argb(p) = k;
      }
   }
}

// ----------------------------------------------------------------- draw-circle

void fill_circle(ARGBImage& argb,
                 Point2 p0,
                 uint32_t k,
                 int radius,
                 float alpha)
{
   for(int dy = -radius; dy <= radius; ++dy) {
      for(int dx = -radius; dx <= radius; ++dx) {
         if(dx * dx + dy * dy > radius * radius) continue;
         auto p = p0 + Point2(dx, dy);
         if(argb.in_bounds(p)) { argb(p) = blend(k, argb(p), alpha); }
      }
   }
}

void fill_circle(cv::Mat& im, Point2 p0, uint32_t k, int radius, float alpha)
{
   if(im.empty()) return;

   if(im.type() != CV_8UC3) FATAL(format("expected a CV_8UC3 image"));

   for(int dy = -radius; dy <= radius; ++dy) {
      for(int dx = -radius; dx <= radius; ++dx) {
         if(dx * dx + dy * dy > radius * radius) continue;
         auto p = p0 + Point2(dx, dy);
         // TRACE(format("im [{}x{}], in-bounds({}, {}) = {}",
         //              im.cols,
         //              im.rows,
         //              p.x,
         //              p.y,
         //              str(in_bounds(im, p.x, p.y))));
         if(in_bounds(im, p)) {
            const uint32_t t = vec3b_to_rgb(im.at<cv::Vec3b>(p.y, p.x));
            im.at<cv::Vec3b>(p.y, p.x) = rgb_to_vec3b(blend(k, t, alpha));
         }
      }
   }
}

void outline_circle(cv::Mat& im, Vector2 p0, uint32_t k, real radius)
{
   if(im.empty()) return;
   if(im.type() != CV_8UC3) FATAL(format("expected a CV_8UC3 image"));

   auto in_bounds = [&](int x, int y) {
      return (x >= 0) and (y >= 0) and (x < im.cols) and (y < im.rows);
   };

   // std::unordered_set<Point2> seen;

   const cv::Vec3b vec3b_k = rgb_to_vec3b(k);
   const int N             = 40;
   const real step         = 2.0 * M_PI / real(N - 1);
   real theta              = 0.0;
   for(auto i = 0; i < N; ++i) {
      const real theta0 = real(i + 0) * step;
      const real theta1 = real(i + 1) * step;
      const auto X      = Vector2(cos(theta0) * radius, sin(theta0) * radius);
      const auto Y      = Vector2(cos(theta1) * radius, sin(theta1) * radius);
      plot_line_AA(
          X + p0,
          Y + p0,
          [&](int x, int y, float a) {
             if(in_bounds(x, y)) {
                // if(seen.count(Point2{x, y}) == 0) {
                //   seen.emplace(x, y);
                im.at<cv::Vec3b>(y, x) = vec3b_k;
                //= blend(im.at<cv::Vec3b>(y, x), vec3b_k, alpha);
                //}
             }
          },
          false);
   }
}

void outline_circle(ARGBImage& im, Vector2 p0, uint32_t k, real radius)
{
   if(!std::isfinite(radius) || radius < 1.0) return;
   const int N     = 40;
   const real step = 2.0 * M_PI / real(N - 1);
   real theta      = 0.0;
   for(auto i = 0; i < N; ++i) {
      const real theta0 = real(i + 0) * step;
      const real theta1 = real(i + 1) * step;
      const auto X      = Vector2(cos(theta0) * radius, sin(theta0) * radius);
      const auto Y      = Vector2(cos(theta1) * radius, sin(theta1) * radius);
      bresenham(
          X + p0, Y + p0, im.bounds(), [&](int x, int y) { set(im, x, y, k); });
   }
}

// ------------------------------------------------------------------ draw-cross

void draw_cross(ARGBImage& argb, Point2 p0, uint32_t k, int radius)
{
   for(int dxy = -radius; dxy <= radius; ++dxy) {
      auto p = p0 + Point2(dxy, dxy);
      if(argb.in_bounds(p)) argb(p) = k;
   }

   for(int dxy = -radius; dxy <= radius; ++dxy) {
      auto p = p0 + Point2(dxy, -dxy);
      if(argb.in_bounds(p)) argb(p) = k;
   }
}

void draw_cross_thick(ARGBImage& argb, Point2 p0, uint32_t k, int radius)
{
   const auto dx = Point2(radius, 0);
   const auto dy = Point2(0, radius);

   auto f = [&](int x, int y, float a) {
      if(!argb.in_bounds(x, y)) return;
      argb(x, y) = blend(k, argb(x, y), a);
   };

   const auto bounds = aabb_to_aabbi(argb.bounds());
   plot_line_AA(p0 + dx + dy, p0 - dx - dy, bounds, f);
   plot_line_AA(p0 + dx - dy, p0 - dx + dy, bounds, f);
}

// ----------------------------------------------------------------- draw-square

void draw_square(ARGBImage& argb,
                 Point2 p0,
                 uint32_t k,
                 int radius,
                 float alpha)
{
   for(int dxy = -radius; dxy <= radius; ++dxy) {
      auto p = p0 + Point2(dxy, -radius);
      auto q = p0 + Point2(dxy, radius);
      if(argb.in_bounds(p)) argb(p) = blend(k, argb(p), alpha);
      if(argb.in_bounds(q)) argb(q) = blend(k, argb(q), alpha);
   }

   for(int dxy = -radius; dxy <= radius; ++dxy) {
      auto p = p0 + Point2(-radius, dxy);
      auto q = p0 + Point2(radius, dxy);
      if(argb.in_bounds(p)) argb(p) = blend(k, argb(p), alpha);
      if(argb.in_bounds(q)) argb(q) = blend(k, argb(q), alpha);
   }
}

void render_quad_2d(ARGBImage& argb,
                    const array<Vector2, 4>& Xs,
                    uint32_t k) noexcept
{
   for(auto i = 0; i < 4; ++i) {
      const auto a = Xs[size_t(i)];
      const auto b = Xs[size_t((i + 1) % 4)];
      plot_line_AA(a, b, argb.bounds(), [&](int x, int y, float a) {
         if(argb.in_bounds(x, y)) argb(x, y) = blend(k, argb(x, y), a);
      });
   }
}

// ---------------------------------------------------------------- decode-image

ARGBImage decode_image(const vector<char>& bytes) noexcept(false)
{
   return cv_to_argb(decode_image_to_cv_mat(bytes));
}

cv::Mat decode_image_to_cv_mat(const vector<char>& bytes) noexcept(false)
{
   return cv::imdecode(cv::Mat(bytes), cv::IMREAD_COLOR);
}

// ------------------------------------------------------------------ make-patch
//
static LABImage make_patch_(const int image_w,
                            const int image_h,
                            const unsigned w, // patch width (pixels)
                            const unsigned h, // patch height (pixels)
                            const Vector2& A, // Vector from A->B
                            const Vector2& B,
                            std::function<LAB(int x, int y)> f)
{
   Expects(w > 1);
   Expects(h > 0);
   LABImage lab;
   lab.resize(w, h);

   const auto offset_y = (h % 2 == 1) ? real((h - 1) / 2) : 0.5 * real(h);

   const auto N  = (B - A);                       // From A ==> B
   const auto M  = N.clockwise_90().normalised(); // orthogonal to M
   const auto dx = N / (w - 1);
   const auto dy = M; // ie, 1 pixel

   auto calc_pixel = [&](const Vector2& X) -> LAB {
      Vector3f lab{0.0f, 0.0f, 0.0f};
      float sum_weight  = 0.0f;
      const auto qspace = quantize(X);
      for(const auto& [x, weight] : qspace) {
         if(x.x >= 0 && x.y >= 0 && x.x < image_w && x.y < image_h) {
            lab += weight * LAB_to_LAB_vec3f(f(x.x, x.y));
            sum_weight += weight;
         }
      }
      return (sum_weight == 0.0f) ? LAB::make_null()
                                  : vec3f_LAB_to_LAB(lab / sum_weight);
   };

   for(auto y = 0u; y < h; ++y)
      for(auto x = 0u; x < w; ++x)
         lab(x, y) = calc_pixel(A + real(x) * dx + (real(y) - offset_y) * dy);

   return lab;
}

LABImage make_patch(const unsigned w, // patch width (pixels)
                    const unsigned h, // patch height (pixels)
                    const Vector2& A, // Vector from A->B
                    const Vector2& B,
                    const cv::Mat& im) noexcept(false)
{
   Expects(im.type() == CV_8UC3);
   return make_patch_(im.cols, im.rows, w, h, A, B, [&](int x, int y) {
      Expects(in_bounds(im, Point2(x, y)));
      return vec3b_to_LAB(im.at<cv::Vec3b>(y, x));
   });
}

LABImage make_patch(const unsigned w, // patch width (pixels), must be > 1
                    const unsigned h, // patch height (pixels), must be > 0
                    const Vector2& A, // Vector from A->B
                    const Vector2& B,
                    const LABImage& im) noexcept(false)
{
   return make_patch_(
       int(im.width), int(im.height), w, h, A, B, [&](int x, int y) {
          Expects(im.in_bounds(x, y));
          return im(x, y);
       });
}

float dot_product(const LABImage& A, const LABImage& B) noexcept
{
   Expects(A.width == B.width && A.height == B.height);
   const float sz = float(A.width * A.height);
   float out      = 0.0f;
   for(auto y = 0u; y < A.height; ++y) {
      const LAB* row_a = A.row_ptr(y);
      const LAB* row_b = B.row_ptr(y);
      for(auto x = 0u; x < A.width; ++x)
         out += cie1976_distance(*row_a++, *row_b++);
   }
   return out;
}

void blit(const cv::Mat& im, ARGBImage& argb, const AABBi& aabb) noexcept
{
   Expects(im.type() == CV_8UC3);
   const int w = std::min<int>(im.cols, aabb.width());
   const int h = std::min<int>(im.rows, aabb.height());

   for(auto y = 0; y < h; ++y)
      for(auto x = 0; x < w; ++x)
         set(argb,
             x + aabb.left,
             y + aabb.top,
             vec3b_to_rgb(im.at<cv::Vec3b>(y, x)));
}

cv::Mat crop_to_cv(const ARGBImage& argb, const AABBi& aabb) noexcept
{
   AABBi bounds = intersection(aabb, aabb_to_aabbi(argb.bounds()));
   cv::Mat im(bounds.height(), bounds.width(), CV_8UC3);
   const int h = im.rows;
   const int w = im.cols;

   for(auto y = 0; y < h; ++y)
      for(auto x = 0; x < w; ++x) {
         Point2 x0 = {x + aabb.left, y + aabb.top};
         Expects(argb.in_bounds(x0));
         im.at<cv::Vec3b>(y, x) = rgb_to_vec3b(argb(x0));
      }

   return im;
}

} // namespace perceive
