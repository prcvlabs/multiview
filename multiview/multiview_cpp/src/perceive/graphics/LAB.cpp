
#include "LAB.hpp"

#include "colour-set.hpp"

#include <algorithm>

#define This LAB

namespace perceive
{
constexpr float k_cie1976_mean   = 76.425318f;
constexpr float k_cie1976_stddev = 39.138304f;
constexpr float k_cie2000_mean   = 40.454945f;
constexpr float k_cie2000_stddev = 18.571403f;

// ------------------------------------------------------------------- make-null

LAB This::make_null() noexcept
{
   LAB lab;
   lab.bits_ = 0x00800000u;
   return lab;
}

// --------------------------------------------------------------------- setters

void This::set_l_(uint32_t x) noexcept
{
   bits_ = (bits_ & ~0xff000000u) | ((x & 0x00ffu) << 24);
}

void This::set_a_(uint32_t x) noexcept
{
   bits_ = (bits_ & ~0x007ff000u) | ((x & 0x07ffu) << 12);
}

void This::set_b_(uint32_t x) noexcept
{
   bits_ = (bits_ & ~0x000007ffu) | ((x & 0x07ffu) << 0);
}

void This::set_L(float x) noexcept // Should be in range [0..100]
{
   set_l_(uint32_t(std::clamp(uint32_t(x / k_L), 0u, 255u)));
}

void This::set_A(float x) noexcept
{
   set_a_(uint32_t(std::clamp(int32_t((x + 127.0f) / k_AB), 0, k_max_AB)));
}

void This::set_B(float x) noexcept
{
   set_b_(uint32_t(std::clamp(int32_t((x + 127.0f) / k_AB), 0, k_max_AB)));
}

// ------------------------------------------------------------------- to-string

string This::to_string() const noexcept(false)
{
   return format("L*A*B* = [{:5.2f}, {:5.2f}, {:5.2f}]", L(), A(), B());
}

string str(const LAB& o) noexcept(false) { return o.to_string(); }

// ----------------------------------------------------------------- conversions

static Vector3 LAB_to_rgb(const real L, const real a, const real b) noexcept
{
   auto y = (L + 16.0) / 116.0;
   auto x = a / 500.0 + y;
   auto z = y - b / 200;

   x = 0.95047
       * ((x * x * x > 0.008856) ? x * x * x : (x - 16.0 / 116.0) / 7.787);
   y = 1.00000
       * ((y * y * y > 0.008856) ? y * y * y : (y - 16.0 / 116.0) / 7.787);
   z = 1.08883
       * ((z * z * z > 0.008856) ? z * z * z : (z - 16.0 / 116.0) / 7.787);

   auto rr = x * 3.2406 + y * -1.5372 + z * -0.4986;
   auto gg = x * -0.9689 + y * 1.8758 + z * 0.0415;
   auto bb = x * 0.0557 + y * -0.2040 + z * 1.0570;

   rr = (rr > 0.0031308) ? (1.055 * pow(rr, 1.0 / 2.4) - 0.055) : 12.92 * rr;
   gg = (gg > 0.0031308) ? (1.055 * pow(gg, 1.0 / 2.4) - 0.055) : 12.92 * gg;
   bb = (bb > 0.0031308) ? (1.055 * pow(bb, 1.0 / 2.4) - 0.055) : 12.92 * bb;

   return Vector3(std::max(0.0, std::min(1.0, rr)),
                  std::max(0.0, std::min(1.0, gg)),
                  std::max(0.0, std::min(1.0, bb)));
}

void This::init(const real r, const real g, const real b) noexcept
{
   init(uint8_t(std::clamp(r * 255.0, 0.0, 255.0)),
        uint8_t(std::clamp(g * 255.0, 0.0, 255.0)),
        uint8_t(std::clamp(b * 255.0, 0.0, 255.0)));
}

void This::init(const uint8_t r, const uint8_t g, const uint8_t b) noexcept
{
   auto make_gamma_table = []() {
      vector<real> table(256);
      for(unsigned i = 0; i < 256; ++i) {
         real scaled{(1.0 / 255.0) * static_cast<real>(i)};
         table[i] = (scaled <= 0.04045) ? scaled / 12.92
                                        : pow((scaled + 0.055) / 1.055, 2.4);
      }
      return table;
   };
   static const vector<real> srgb_gamma{make_gamma_table()};

   const real R = srgb_gamma[r];
   const real G = srgb_gamma[g];
   const real B = srgb_gamma[b];

   const real X = R * 0.4124564 + G * 0.3575761 + B * 0.1804375;
   const real Y = R * 0.2126729 + G * 0.7151522 + B * 0.0721750;
   const real Z = R * 0.0193339 + G * 0.1191920 + B * 0.9503041;

   //------------------------
   // XYZ to LAB conversion
   //------------------------
   constexpr real epsilon = 0.008856; // actual CIE standard
   constexpr real kappa   = 7.787;    // actual CIE standard

   constexpr real Xr = 0.950456; // reference white
   constexpr real Yr = 1.0;      // reference white
   constexpr real Zr = 1.088754; // reference white

   const real xr = X / Xr;
   const real yr = Y / Yr;
   const real zr = Z / Zr;

   const auto fx = (xr > epsilon) ? std::cbrt(xr) : (kappa * xr + 16.0) / 116.0;
   const auto fy = (yr > epsilon) ? std::cbrt(yr) : (kappa * yr + 16.0) / 116.0;
   const auto fz = (zr > epsilon) ? std::cbrt(zr) : (kappa * zr + 16.0) / 116.0;

   this->set_L(float(116.0 * fy - 16.0));
   this->set_A(float(500.0 * (fx - fy)));
   this->set_B(float(200.0 * (fy - fz)));
}

void This::init_from_labvec3(const Vector3& lab) noexcept
{
   set_L(float(lab.x));
   set_A(float(lab.y));
   set_B(float(lab.z));
}

LAB kolour_to_LAB(uint32_t k) noexcept
{
   LAB lab;
   lab.init(red(k), green(k), blue(k));
   return lab;
}

LAB rgb_to_LAB(const Vector3& rgb) noexcept
{
   LAB lab;
   lab.init(rgb.x, rgb.y, rgb.z);
   return lab;
}

LAB hsv_to_LAB(const Vector3& hsv) noexcept
{
   return rgb_to_LAB(hsv_to_rgb(hsv));
}

LAB vec3b_to_LAB(const cv::Vec3b& X) noexcept
{
   LAB lab;
   lab.init(X[2], X[1], X[0]);
   return lab;
}

LAB vec3f_LAB_to_LAB(const Vector3f& X) noexcept
{
   LAB lab;
   lab.set_L(X.x);
   lab.set_A(X.y);
   lab.set_B(X.z);
   return lab;
}

uint32_t LAB_to_kolour(const LAB lab) noexcept
{
   return vector3_to_kolour(LAB_to_rgb(lab));
}

Vector3 LAB_to_rgb(const LAB lab) noexcept
{
   return LAB_to_rgb(real(lab.L()), real(lab.A()), real(lab.B()));
}

Vector3 LAB_to_hsv(const LAB lab) noexcept
{
   return rgb_to_hsv(LAB_to_rgb(lab));
}

cv::Vec3b LAB_to_vec3b(const LAB lab) noexcept
{
   return rgb_to_vec3b(LAB_to_kolour(lab));
}

Vector3f LAB_to_LAB_vec3f(const LAB lab) noexcept
{
   return Vector3f(lab.L(), lab.A(), lab.B());
}

// ------------------------------------------------------------------- distances

float cie1976_normalized_distance(const LAB& lab1, const LAB& lab2) noexcept
{
   const float n
       = (cie1976_distance(lab1, lab2) - k_cie1976_mean) / k_cie1976_stddev;
   constexpr float k_min_n = float(-(1.0 * cie1976_mean) / cie1976_stddev);
   constexpr float k_range = 3.0f - k_min_n; // Assume n in [min_n .. 3]

   // Normalize to [0..1], with 0 being no difference
   auto ret = std::clamp((n - k_min_n) / k_range, 0.0f, 1.0f);
   Ensures(ret >= 0.0f);
   Ensures(ret <= 1.0f);
   return ret;
}

} // namespace perceive
