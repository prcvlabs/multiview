
#pragma once

#include <opencv2/core.hpp>

namespace perceive
{
struct LAB
{
 private:
   static constexpr int AB_bits      = 11;
   static constexpr int32_t k_max_AB = (1u << AB_bits) - 1;
   static constexpr float k_L        = 100.0f / 255.0f;
   static constexpr float k_AB       = 254.0f / float(k_max_AB);

   uint32_t bits_ = 0; // [L(8) | A(12) | B(12)]

   void set_l_(uint32_t x) noexcept;
   void set_a_(uint32_t x) noexcept;
   void set_b_(uint32_t x) noexcept;
   uint32_t get_l_() const noexcept { return (bits_ & 0xff000000u) >> 24; }
   uint32_t get_a_() const noexcept { return (bits_ & 0x007ff000u) >> 12; }
   uint32_t get_b_() const noexcept { return (bits_ & 0x000007ffu) >> 0; }

 public:
   bool operator==(const LAB& o) const noexcept { return bits_ == o.bits_; }
   bool operator!=(const LAB& o) const noexcept { return !(*this == o); }

   static LAB make_null() noexcept;

   // Red/green/blue in the range [0..1]
   void init(const real r, const real g, const real b) noexcept;
   void init(const uint8_t r, const uint8_t g, const uint8_t b) noexcept;

   Vector3 to_labvec3() const noexcept
   {
      return to_vec3(Vector3f(L(), A(), B()));
   }
   void init_from_labvec3(const Vector3& lab) noexcept;

   // Getters
   float L() const noexcept { return float(get_l_()) * k_L; }
   float A() const noexcept { return float(get_a_()) * k_AB - 127.0f; }
   float B() const noexcept { return float(get_b_()) * k_AB - 127.0f; }

   void set_L(float x) noexcept;
   void set_A(float x) noexcept;
   void set_B(float x) noexcept;

   bool is_null() const noexcept { return bits_ & 0x00800000u; }

   // To/from string
   string to_string() const noexcept(false);
   friend string str(const LAB&) noexcept(false);
};

// Conversions
LAB kolour_to_LAB(uint32_t k) noexcept;
LAB rgb_to_LAB(const Vector3& rgb) noexcept;
LAB hsv_to_LAB(const Vector3& hsv) noexcept;
LAB vec3b_to_LAB(const cv::Vec3b& X) noexcept;
LAB vec3f_LAB_to_LAB(const Vector3f& X) noexcept;

uint32_t LAB_to_kolour(const LAB) noexcept;
Vector3 LAB_to_rgb(const LAB) noexcept;
Vector3 LAB_to_hsv(const LAB) noexcept;
cv::Vec3b LAB_to_vec3b(const LAB) noexcept;
Vector3f LAB_to_LAB_vec3f(const LAB) noexcept;

// Distances
inline float cie1976_distance(const LAB& a, const LAB& b) noexcept
{
   return sqrtf(square(a.L() - b.L()) + square(a.A() - b.A())
                + square(a.B() - b.B()));
}

// Returns a distance in [0..1], close values clamped to 0
float cie1976_normalized_distance(const LAB& lab1, const LAB& lab2) noexcept;

} // namespace perceive
