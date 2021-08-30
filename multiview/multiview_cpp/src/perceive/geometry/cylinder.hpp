
#pragma once

namespace perceive
{
struct Cylinder
{
   Vector3 X   = Vector3::nan(); // on the base
   real radius = dNAN;
   real height = dNAN;

   Cylinder(const Vector3 X_ = Vector3::nan(),
            real radius_     = dNAN,
            real height_     = dNAN)
       : X(X_)
       , radius(radius_)
       , height(height_)
   {}
   Cylinder(const Cylinder&) noexcept = default;
   Cylinder(Cylinder&&) noexcept      = default;
   ~Cylinder()                        = default;
   Cylinder& operator=(const Cylinder&) noexcept = default;
   Cylinder& operator=(Cylinder&&) noexcept = default;

   Vector3 top() const noexcept { return X + Vector3{0.0, 0.0, height}; }
};

struct ProjectiveFloorCylinder
{
 private:
   Plane top_   = Plane::nan();
   Plane left_  = Plane::nan();
   Plane right_ = Plane::nan();
   Plane C_p3_  = Plane::nan(); // plane that bisects 'left' and 'right'

   // the intersection of the above three planes... should be the camera center
   Vector3 C_ = Vector3::nan();

   bool is_init_ = false;

 public:
   void init(const Plane& left, const Plane& right, const Plane& top);
   void init(const Plane& top, const Plane& C_p3, const Vector3& C);

   bool is_init() const noexcept { return is_init_; }

   const Vector3& C() const noexcept { return C_; }
   const Plane& top() const noexcept { return top_; }
   const Plane& left() const noexcept { return left_; }
   const Plane& right() const noexcept { return right_; }
   const Plane& C_p3() const noexcept { return C_p3_; }

   Cylinder realize(const real height) const noexcept;
};

} // namespace perceive
