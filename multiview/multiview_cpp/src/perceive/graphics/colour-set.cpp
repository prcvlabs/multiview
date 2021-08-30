
#include <random>

#include "colour-set.hpp"
#include "stdinc.hpp"

namespace perceive
{
// ------------------------------------------------------------------ RGB to HSV

Vector3 rgb_to_hsv(const Vector3& in) noexcept
{
   Vector3 out;
   double min, max, delta;

   min = in.x < in.y ? in.x : in.y;
   min = min < in.z ? min : in.z;

   max = in.x > in.y ? in.x : in.y;
   max = max > in.z ? max : in.z;

   out.z = max; // v
   delta = max - min;
   if(delta < 0.00001) {
      out.y = 0;
      out.x = 0; // undefined, maybe nan?
      return out;
   }
   if(max > 0.0) { // NOTE: if Max is == 0, this divide would cause a crash
      out.y = (delta / max); // s
   } else {
      // if max is 0, then r = g = b = 0
      // s = 0, v is undefined
      out.y = 0.0;
      out.x = dNAN; // its now undefined
      return out;
   }
   if(in.x >= max)                   // > is bogus, just keeps compilor happy
      out.x = (in.y - in.z) / delta; // between yellow & magenta
   else if(in.y >= max)
      out.x = 2.0 + (in.z - in.x) / delta; // between cyan & yellow
   else
      out.x = 4.0 + (in.x - in.y) / delta; // between magenta & cyan

   out.x *= 60.0; // degrees

   if(out.x < 0.0) out.x += 360.0;

   return out;
}

// ------------------------------------------------------------------ HSV to RGB
// rgb hsv2rgb(hsv in)
Vector3 hsv_to_rgb(const Vector3& in) noexcept
{
   double hh, p, q, t, ff;
   long i;
   Vector3 out;

   if(in.y <= 0.0) { // < is bogus, just shuts up warnings
      out.x = in.z;
      out.y = in.z;
      out.z = in.z;
      return out;
   }
   hh = in.x;
   if(hh >= 360.0) hh = 0.0;
   hh /= 60.0;
   i  = long(hh);
   ff = hh - double(i);
   p  = in.z * (1.0 - in.y);
   q  = in.z * (1.0 - (in.y * ff));
   t  = in.z * (1.0 - (in.y * (1.0 - ff)));

   switch(i) {
   case 0:
      out.x = in.z;
      out.y = t;
      out.z = p;
      break;
   case 1:
      out.x = q;
      out.y = in.z;
      out.z = p;
      break;
   case 2:
      out.x = p;
      out.y = in.z;
      out.z = t;
      break;

   case 3:
      out.x = p;
      out.y = q;
      out.z = in.z;
      break;
   case 4:
      out.x = t;
      out.y = p;
      out.z = in.z;
      break;
   case 5:
   default:
      out.x = in.z;
      out.y = p;
      out.z = q;
      break;
   }
   return out;
}

// --------------------------------------------------------------------- HeatMap

// HeatMap::HeatMap()
// {
//    kolours.emplace_back(0, 0, 0);
//    kolours.emplace_back(1, 1, 1);
// }

// Vector3 HeatMap::colour(double value) const
// {
//    const int num_kolours = kolours.size();
//    if(num_kolours == 0)
//       FATAL("Must have at least 1 kolour to interpolate between");

//    int idx1; // |-- Desired kolours between these two indexes in "kolours".
//    int idx2; // |
//    // Fraction between "idx1" and "idx2" where our value is.
//    double fract = 0;

//    if(value <= 0.0) {
//       idx1 = idx2 = 0;
//    } else if(value >= 1.0) {
//       idx1 = idx2 = num_kolours - 1; // accounts for an input >=0
//    } else {
//       value = value * (num_kolours - 1); // Will multiply value by 3.
//       idx1  = floor(value); // Desired kolours will be after this index.
//       idx2  = idx1 + 1;     // ... and before this index (inclusive).
//       // Distance between the two indexes (0-1).
//       fract = value - double(idx1);
//    }

//    Vector3 ret;
//    ret.x = (kolours[idx2][0] - kolours[idx1][0]) * fract + kolours[idx1][0];
//    ret.y = (kolours[idx2][1] - kolours[idx1][1]) * fract + kolours[idx1][1];
//    ret.z = (kolours[idx2][2] - kolours[idx1][2]) * fract + kolours[idx1][2];
//    return ret;
// }

// ------------------------------------------------------------------ RGB to LAB

// sRGB (D65 illuninant assumption) to XYZ conversion
static inline Vector3 rgb_to_xyz(const Vector3& in) noexcept
{
   const auto R = in.x, G = in.y, B = in.z;

   const auto r = (R <= 0.04045) ? R / 12.92 : pow((R + 0.055) / 1.055, 2.4);
   const auto g = (G <= 0.04045) ? G / 12.92 : pow((G + 0.055) / 1.055, 2.4);
   const auto b = (B <= 0.04045) ? B / 12.92 : pow((B + 0.055) / 1.055, 2.4);

   return Vector3(r * 0.4124564 + g * 0.3575761 + b * 0.1804375,
                  r * 0.2126729 + g * 0.7151522 + b * 0.0721750,
                  r * 0.0193339 + g * 0.1191920 + b * 0.9503041);
}

static inline Vector3 xyz_to_lab(const Vector3& XYZ) noexcept
{
   const auto X = XYZ.x, Y = XYZ.y, Z = XYZ.z;

   //------------------------
   // XYZ to LAB conversion
   //------------------------
   constexpr double epsilon = 0.008856; // actual CIE standard
   constexpr double kappa   = 7.787;    // actual CIE standard

   constexpr double Xr = 0.950456; // reference white
   constexpr double Yr = 1.0;      // reference white
   constexpr double Zr = 1.088754; // reference white

   const double xr = X / Xr;
   const double yr = Y / Yr;
   const double zr = Z / Zr;

   const auto fx = (xr > epsilon) ? std::cbrt(xr) : (kappa * xr + 16.0) / 116.0;
   const auto fy = (yr > epsilon) ? std::cbrt(yr) : (kappa * yr + 16.0) / 116.0;
   const auto fz = (zr > epsilon) ? std::cbrt(zr) : (kappa * zr + 16.0) / 116.0;

   return Vector3(116.0 * fy - 16.0, 500.0 * (fx - fy), 200.0 * (fy - fz));
}

Vector3 rgb_to_lab(const Vector3& in) noexcept
{
   return xyz_to_lab(rgb_to_xyz(in));
}

Vector3 rgb_to_lab(uint32_t in) noexcept
{
   auto make_gamma_table = []() {
      vector<real> table(256);
      for(unsigned i = 0; i < 256; ++i) {
         real scaled{(1.0 / 255.0) * static_cast<real>(i)};
         table[i] = (scaled <= 0.04045)
                        ? scaled / 12.92
                        : std::pow((scaled + 0.055) / 1.055, 2.4);
      }
      return table;
   };
   static const vector<real> srgb_gamma{make_gamma_table()};

   const auto R{srgb_gamma[(in >> 16) & 0xff]};
   const auto G{srgb_gamma[(in >> 8) & 0xff]};
   const auto B{srgb_gamma[in & 0xff]};

   Vector3 xyz{R * 0.4124564 + G * 0.3575761 + B * 0.1804375,
               R * 0.2126729 + G * 0.7151522 + B * 0.0721750,
               R * 0.0193339 + G * 0.1191920 + B * 0.9503041};

   return xyz_to_lab(xyz);
}

Vector3 lab_to_rgb(const Vector3& in) noexcept
{
   const auto L = in[0];
   const auto a = in[1];
   const auto b = in[2];

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

real cie2000_compare(const Vector3f& lab1, const Vector3f& lab2) noexcept
{
   return cie2000_compare(to_vec3(lab1), to_vec3(lab2));
}

real cie2000_compare(const Vector3& lab_a, const Vector3& lab_b) noexcept
{
   constexpr auto eps = 1e-5;

   // calculate ci, hi, i=1,2
   auto c1     = sqrt(square(lab_a.y) + square(lab_a.z));
   auto c2     = sqrt(square(lab_b.y) + square(lab_b.z));
   auto meanC  = (c1 + c2) / 2.0;
   auto meanC7 = pow(meanC, 7.0);

   // 0.5*(1-sqrt(meanC^7/(meanC^7+25^7)))
   auto g   = 0.5 * (1 - sqrt(meanC7 / (meanC7 + 6103515625.)));
   auto a1p = lab_a.y * (1 + g);
   auto a2p = lab_b.y * (1 + g);

   c1      = sqrt(square(a1p) + square(lab_a.z));
   c2      = sqrt(square(a2p) + square(lab_b.z));
   auto h1 = fmod(atan2(lab_a.z, a1p) + 2 * M_PI, 2 * M_PI);
   auto h2 = fmod(atan2(lab_b.z, a2p) + 2 * M_PI, 2 * M_PI);

   // compute deltaL, deltaC, deltaH
   auto deltaL = lab_b.x - lab_a.x;
   auto deltaC = c2 - c1;
   auto deltah = 0.0;

   if(c1 * c2 < eps) { deltah = 0; }
   if(abs(h2 - h1) <= M_PI) {
      deltah = h2 - h1;
   } else if(h2 > h1) {
      deltah = h2 - h1 - 2 * M_PI;
   } else {
      deltah = h2 - h1 + 2 * M_PI;
   }

   auto deltaH = 2 * sqrt(c1 * c2) * sin(deltah / 2);

   // calculate CIEDE2000
   auto meanL = (lab_a.x + lab_b.x) / 2;
   meanC      = (c1 + c2) / 2.0;
   meanC7     = pow(meanC, 7.0);
   auto meanH = 0.0;

   if(c1 * c2 < eps) { meanH = h1 + h2; }
   if(abs(h1 - h2) <= M_PI + eps) {
      meanH = (h1 + h2) / 2;
   } else if(h1 + h2 < 2 * M_PI) {
      meanH = (h1 + h2 + 2 * M_PI) / 2;
   } else {
      meanH = (h1 + h2 - 2 * M_PI) / 2;
   }

   auto T = 1 - 0.17 * cos(meanH - to_radians(30.0)) + 0.24 * cos(2 * meanH)
            + 0.32 * cos(3 * meanH + to_radians(6.0))
            - 0.2 * cos(4 * meanH - to_radians(63.0));
   auto sl = 1 + (0.015 * square(meanL - 50)) / sqrt(20 + square(meanL - 50));
   auto sc = 1 + 0.045 * meanC;
   auto sh = 1 + 0.015 * meanC * T;
   auto rc = 2 * sqrt(meanC7 / (meanC7 + 6103515625.));
   auto rt = -sin(to_radians(60 * exp(-square((to_degrees(meanH) - 275) / 25))))
             * rc;

   return sqrt(square(deltaL / sl) + square(deltaC / sc) + square(deltaH / sh)
               + rt * deltaC / sc * deltaH / sh);
}

// ---------------------------------------------------------- cie1976-normalized

template<typename T>
T cie1976_normalizedT(const Vector3T<T>& lab1, const Vector3T<T>& lab2) noexcept
{
   const T n = ((lab1 - lab2).norm() - T(cie1976_mean)) / T(cie1976_stddev);
   constexpr T min_n = T(-cie1976_mean / cie1976_stddev);
   constexpr T range = T(3.0) - min_n; // Assume n in [min_n .. 3]

   // Normalize to [0..1], with 0 being no difference
   auto ret = clamp((n - min_n) / range, T(0.0), T(1.0));
   Ensures(ret >= T(0.0));
   Ensures(ret <= T(1.0));
   return ret;
}

real cie1976_normalized(const Vector3& lab1, const Vector3& lab2) noexcept
{
   return cie1976_normalizedT(lab1, lab2);
}

float cie1976_normalized(const Vector3f& lab1, const Vector3f& lab2) noexcept
{
   return cie1976_normalizedT(lab1, lab2);
}

// -------------------------------------------------------- test LAB conversions

namespace detail
{
   void test_LAB_conversions() noexcept
   {
      INFO("Test LAB conversions, listing 'sorted' crayons");

      uint32_t j = k_black;
      for(auto& k : k_sorted_crayons) {
         auto lab     = rgb_to_lab(k);
         auto old_lab = rgb_to_lab(j);

         cout << format("# RGB ({:3d}, {:3d}, {:3d})  "
                        "==>  LAB ({:10.6f}, {:10.6f}, {:10.6f})  "
                        "==>  \u039474 = {:10.6f}, {:10.6f} "
                        "\u039400 = {:10.6f}, {:10.6f}",
                        red(k),
                        green(k),
                        blue(k),
                        lab.x,
                        lab.y,
                        lab.z,
                        cie1976_compare(lab, old_lab),
                        cie1976_compare(old_lab, lab),
                        cie2000_compare(lab, old_lab),
                        cie2000_compare(old_lab, lab))
              << endl;
         j = k;
      }

      INFO("Calculating mean/variance of cie measures");

      // Set up the vector
      std::vector<uint32_t> ks;
      for(auto& k : k_sorted_crayons) ks.push_back(k);

      std::random_device rd;
      std::mt19937 g(rd());

      std::vector<uint32_t> big_ks;
      const unsigned N = 1000; // the number of times we add the crayons
      big_ks.reserve(N * ks.size());
      for(unsigned n = 0; n < N; ++n) {
         std::shuffle(ks.begin(), ks.end(), g);
         big_ks.insert(big_ks.end(), ks.begin(), ks.end());
      }

      auto calc_av = [&](std::function<real(uint32_t, uint32_t)> f) {
         auto sum = 0.0;
         for(unsigned i = 0; i < big_ks.size(); ++i)
            sum += f(big_ks[i], big_ks[(i + 1) % big_ks.size()]);
         return sum / real(big_ks.size());
      };

      FILE* fp = nullptr;

      auto calc_av_stddev = [&](std::function<real(uint32_t, uint32_t)> f) {
         auto av        = calc_av(f);
         auto sum_sq    = 0.0;
         auto count_inv = 1.0 / real(big_ks.size());
         for(unsigned i = 0; i < big_ks.size(); ++i) {
            auto dist = f(big_ks[i], big_ks[(i + 1) % big_ks.size()]);
            fprintf(fp, "%f\n", dist);
            sum_sq += square(dist - av) * count_inv;
         }
         return std::pair<real, real>(av, sqrt(sum_sq));
      };

      real av = 0.0, stddev = 0.0;
      fp                   = fopen("/tmp/cie1976.csv", "w");
      std::tie(av, stddev) = calc_av_stddev([](uint32_t a, uint32_t b) {
         return cie1976_compare(rgb_to_lab(a), rgb_to_lab(b));
      });
      cout << format("cie1976, mean = {}, stddev = {}", av, stddev) << endl;
      fclose(fp);

      fp                   = fopen("/tmp/cie2000.csv", "w");
      std::tie(av, stddev) = calc_av_stddev([](uint32_t a, uint32_t b) {
         return cie2000_compare(rgb_to_lab(a), rgb_to_lab(b));
      });
      cout << format("cie2000, mean = {}, stddev = {}", av, stddev) << endl;
      fclose(fp);
   }
} // namespace detail

} // namespace perceive
