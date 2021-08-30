
#pragma once

namespace cv
{
class Mat;
}

namespace perceive
{
struct FrameTexture final
{
 private:
   unsigned tex_id_     = unsigned(-1);
   int w_               = 0;
   int h_               = 0;
   vector<uint8_t> raw_ = {}; // for performing pixel conversion

 public:
   FrameTexture() = default;
   FrameTexture(FrameTexture&& o) { *this = std::move(o); }
   FrameTexture(const FrameTexture&) = delete;
   ~FrameTexture();
   FrameTexture& operator=(const FrameTexture&) = delete;
   FrameTexture& operator                       =(FrameTexture&& o);

   unsigned tex_id() const noexcept { return tex_id_; }
   int w() const noexcept { return w_; }
   int h() const noexcept { return h_; }
   std::pair<int, int> dims() const noexcept { return {w_, h_}; }

   // Lazily creates the texture, as needed
   void update(const cv::Mat& im) noexcept;
};

} // namespace perceive
