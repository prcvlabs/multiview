
#pragma once

#include "perceive/geometry/euclidean-transform.hpp"
#include "perceive/geometry/projective/distorted-camera.hpp"

namespace perceive::graphics
{
struct Fragment
{
   std::array<Vector3f, 3> triangle;
   float alpha     = 1.0f;
   uint32_t kolour = k_white;
   Vector3f centre() const noexcept
   {
      return (triangle[0] + triangle[1] + triangle[2]) / 3.0f;
   }
};

class Fragments
{
 private:
   std::deque<Fragment> fragments_;

 public:
   void push_fragment(Fragment f);

   // Render the fragments onto the image, using the camera
   void render(const DistortedCamera& dcam, ARGBImage& im);
};

} // namespace perceive::graphics
