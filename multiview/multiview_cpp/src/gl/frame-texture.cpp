
#include "frame-texture.hpp"

#include <GL/glew.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define This FrameTexture

namespace perceive
{
// ---------------------------------------------------- Construction/Destruction
//
This::~This()
{
   if(tex_id_ != GLuint(-1)) glDeleteTextures(1, &tex_id_);
}

FrameTexture& This::operator=(FrameTexture&& o)
{
   if(this == &o) return *this;

   using std::swap;
   swap(tex_id_, o.tex_id_);
   swap(w_, o.w_);
   swap(h_, o.h_);
   swap(raw_, o.raw_);

   return *this;
}

// ---------------------------------------------------------------------- update
//
void This::update(const cv::Mat& im) noexcept
{
   static_assert(sizeof(GLuint) == sizeof(unsigned));
   Expects(im.type() == CV_8UC3);

   if(tex_id_ == GLuint(-1)) {
      w_ = im.cols;
      h_ = im.rows;
      Expects(w_ > 0);
      Expects(h_ > 0);
      glGenTextures(1, &tex_id_);
      glBindTexture(GL_TEXTURE_2D, tex_id_);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
      // Allocate memory for the image
      glTexImage2D(GL_TEXTURE_2D,
                   0,
                   GL_RGBA8,
                   w_,
                   h_,
                   0,
                   GL_BGRA,
                   GL_UNSIGNED_BYTE,
                   nullptr);
      raw_.resize(size_t(w_ * h_ * 4));
   }

   Expects(im.cols == w_);
   Expects(im.rows == h_);

   {
      uint8_t* __restrict__ dst = &raw_[0];
      for(auto y = 0; y < h_; ++y) {
         const uint8_t* __restrict__ src = im.ptr(y);
         for(auto x = 0; x < w_; ++x) {
            *dst++ = *src++;
            *dst++ = *src++;
            *dst++ = *src++;
            *dst++ = 255;
         }
      }

      Expects(dst == &raw_[0] + raw_.size());
   }

   glBindTexture(GL_TEXTURE_2D, tex_id_);
   glTexSubImage2D(
       GL_TEXTURE_2D, 0, 0, 0, w_, h_, GL_BGRA, GL_UNSIGNED_BYTE, &raw_[0]);
}

} // namespace perceive

#undef This
