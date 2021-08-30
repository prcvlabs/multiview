
#pragma once

namespace perceive
{
struct FramebufferObject
{
 private:
   unsigned fbo_{0};
   unsigned rbo_rgba_{0};
   unsigned rbo_depth_{0};
   unsigned w_{0};
   unsigned h_{0};

 public:
   FramebufferObject() = default;
   FramebufferObject(unsigned w, unsigned h) { init(w, h); }
   FramebufferObject(const FramebufferObject&) = delete; // No copy
   FramebufferObject(FramebufferObject&& o) { *this = std::move(o); }
   ~FramebufferObject() { destroy(); }
   FramebufferObject& operator=(const FramebufferObject&) = delete;
   FramebufferObject& operator=(FramebufferObject&&);

   void init(const unsigned fbo_w, const unsigned fbo_h) noexcept(false);
   void destroy();

   unsigned fbo() const noexcept { return fbo_; }
   unsigned rbo_rgba() const noexcept { return rbo_rgba_; }
   unsigned rbo_depth() const noexcept { return rbo_depth_; }

   unsigned w() const noexcept { return w_; }
   unsigned h() const noexcept { return h_; }

   bool is_valid() const noexcept { return fbo_ != 0; }

   // Binds the frame-buffer to opengl context
   // Prints any error messages, like !is_valid(). Returns TRUE is succeeds.
   bool bind() const noexcept;
};

} // namespace perceive
