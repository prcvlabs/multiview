
#pragma once

#include "aabb.hpp"
#include "vector.hpp"

namespace perceive
{
struct Box
{
   std::array<Vector3, 4> base;
   real height = dNAN;

   Box()                    = default;
   Box(const Box&) noexcept = default;
   Box(Box&&) noexcept      = default;
   ~Box()                   = default;
   Box& operator=(const Box&) noexcept = default;
   Box& operator=(Box&&) noexcept = default;

   static Box make(const AABB& aabb, const real z, const real height)
   {
      Box box;
      box.base[0] = Vector3(aabb.left, aabb.bottom, z);
      box.base[1] = Vector3(aabb.left, aabb.top, z);
      box.base[2] = Vector3(aabb.right, aabb.top, z);
      box.base[3] = Vector3(aabb.right, aabb.bottom, z);
      box.height  = height;
      return box;
   }

   static Box make(const Vector3& C, const real side, const real height)
   {
      Box box;
      const auto dx = Vector3(1.0, 0.0, 0.0);
      const auto dy = Vector3(0.0, 1.0, 0.0);
      box.base[0]   = C - 0.5 * side * dx - 0.5 * side * dy;
      box.base[0]   = C - 0.5 * side * dx + 0.5 * side * dy;
      box.base[0]   = C + 0.5 * side * dx + 0.5 * side * dy;
      box.base[0]   = C + 0.5 * side * dx - 0.5 * side * dy;
      box.height    = height;
      return box;
   }
};

} // namespace perceive
