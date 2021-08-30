
#pragma once

#include "perceive/geometry/vector.hpp"

#include <array>

namespace perceive
{
struct GlFragment
{
   std::array<Vector3f, 3> triangle = {};
   Vector4f kolour                  = {}; // red green blue alpha
   bool is_line                     = false;

   GlFragment()                  = default;
   GlFragment(const GlFragment&) = default;
   GlFragment(GlFragment&&)      = default;
   ~GlFragment()                 = default;
   GlFragment& operator=(const GlFragment&) = default;
   GlFragment& operator=(GlFragment&&) = default;

   GlFragment(const Vector3f& A,
              const Vector3f& B,
              const Vector3f& C,
              const Vector4f& kolour,
              bool is_line = false)
   {
      triangle[0]   = A;
      triangle[1]   = B;
      triangle[2]   = C;
      this->kolour  = kolour;
      this->is_line = is_line;
   }

   GlFragment(const Vector3f& A,
              const Vector3f& B,
              const Vector3f& C,
              const Vector4f& kolour)
   {
      triangle[0]  = A;
      triangle[1]  = B;
      triangle[2]  = Vector3f::nan();
      this->kolour = kolour;
   }

   bool operator<(const GlFragment& o) const noexcept
   {
      return this->z() < o.z();
   }

   Vector3f center() const noexcept { return centre(); }
   Vector3f centre() const noexcept
   {
      return is_line ? ((triangle[0] + triangle[1]) / 2.0f)
                     : ((triangle[0] + triangle[1] + triangle[2]) / 3.0f);
   }

   float z() const noexcept
   {
      return is_line ? ((triangle[0].z + triangle[1].z) / 2.0f)
                     : ((triangle[0].z + triangle[1].z + triangle[2].z) / 3.0f);
   }
};
} // namespace perceive
