
#pragma once

#include <deque>

#include "perceive/foundation.hpp"
#include "perceive/geometry/aabb.hpp"
#include "perceive/geometry/vector.hpp"

namespace perceive
{
typedef std::pair<Vector2, uint32_t> LabeledVector2;
typedef std::pair<Vector3, uint32_t> LabeledVector3;

// ------------------------------------------------------------- Spatial-2-Index

class Spatial2Index final
{
 private:
   struct Pimpl;
   Pimpl* pimpl_;

 public:
   CUSTOM_NEW_DELETE(Spatial2Index)

   Spatial2Index();
   Spatial2Index(const Spatial2Index&);
   Spatial2Index(Spatial2Index&&);
   Spatial2Index& operator=(const Spatial2Index&);
   Spatial2Index& operator=(Spatial2Index&&);
   virtual ~Spatial2Index();

   void clear();
   void init(const vector<Vector2>& pts);
   void init(const std::deque<Vector2>& pts);
   size_t size() const noexcept;
   bool empty() const noexcept;

   // Find nearest n points. Inserted onto the end... ret is not cleared
   void query_nearest(const Vector2& P,
                      uint n_points,
                      std::deque<LabeledVector2>& ret) const;
   void query_nearest(const Vector2& P,
                      uint n_points,
                      vector<LabeledVector2>& ret) const;
   LabeledVector2 query_nearest(const Vector2& P) const;

   void query_region(const AABB& aabb, std::deque<LabeledVector2>& ret) const;
   void query_region(const AABB& aabb, vector<LabeledVector2>& ret) const;
   void query_region(const AABB& aabb, vector<Vector2>& ret) const;
};

// ------------------------------------------------------------- Spatial-3-Index

class Spatial3Index final
{
 private:
   struct Pimpl;
   Pimpl* pimpl_;

 public:
   Spatial3Index();
   Spatial3Index(const Spatial3Index&);
   Spatial3Index(Spatial3Index&&);
   Spatial3Index& operator=(const Spatial3Index&);
   Spatial3Index& operator=(Spatial3Index&&);
   virtual ~Spatial3Index();

   void clear();
   void init(const vector<Vector3>& pts);
   void init(const std::deque<Vector3>& pts);

   // Find nearest n points. Inserted onto the end... ret is not cleared
   void query_nearest(const Vector3& P,
                      uint n_points,
                      std::deque<LabeledVector3>& ret) const;
   LabeledVector3 query_nearest(const Vector3& P) const;
};

} // namespace perceive
