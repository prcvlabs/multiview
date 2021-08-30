
#pragma once

#include "config.hpp"

// TODO:
// + Config for parsed commandline arguments
// + Context for SDL/ImGUI/GL resources

namespace perceive::gui
{
class AnnotationApp
{
 private:
   struct Pimpl;
   unique_ptr<Pimpl> pimpl_;
   Config config_;

 public:
   static int run_app(int argc, char** argv) noexcept;

   AnnotationApp(Config config) noexcept(false);
   AnnotationApp(AnnotationApp&&)      = default;
   AnnotationApp(const AnnotationApp&) = delete;
   ~AnnotationApp();
   AnnotationApp& operator=(AnnotationApp&&) = default;
   AnnotationApp& operator=(const AnnotationApp&) = delete;

   const Config& config() const noexcept;

   int exec() noexcept;
};

} // namespace perceive::gui
