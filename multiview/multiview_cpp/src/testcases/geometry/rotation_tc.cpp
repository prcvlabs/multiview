
#include <algorithm>
#include <iterator>

#define CATCH_CONFIG_PREFIX_ALL

#include "perceive/contrib/catch.hpp"
#include "perceive/geometry/rotation.hpp"

namespace perceive
{
// -----------------------------------------------------------------------------

CATCH_TEST_CASE("RotationTests", "[rotation_tests]")
{
   std::mt19937 gen;
   std::uniform_real_distribution<double> distribution{0.0, 1.0};
   gen.seed(0);
   auto rand_theta = [&]() { return (2.0 * distribution(gen) - 1.0) * M_PI; };
   auto rand_q     = [&]() {
      return saa_to_quaternion(
          Vector3(rand_theta(), rand_theta(), rand_theta()));
   };
   auto rand_V = [&]() { return rand_q().rotate(Vector3(1, 0, 0)); };

   // ---------------------------------------------------------- calc-rotation-1
   CATCH_SECTION("calc-rotation-1")
   {
      gen.seed(0);
      auto test_it = [&]() {
         const auto q  = rand_q();
         const auto P1 = rand_V();
         const auto P2 = rand_V();
         const auto W1 = q.rotate(P1);
         const auto W2 = q.rotate(P2);

         const auto q_ = calc_rotation(P1, W1, P2, W2);

         const auto V1 = q_.rotate(P1);
         const auto V2 = q_.rotate(P2);
         CATCH_REQUIRE((dot(W1, V1) - 1.0) < 1e-6);
         CATCH_REQUIRE((dot(W2, V2) - 1.0) < 1e-6);
      };

      for(auto i = 0; i < 1000; ++i) test_it();
   }

   // ---------------------------------------------------------- calc-rotation-2
   CATCH_SECTION("calc-rotation-2")
   {
      gen.seed(0);
      auto test_it = [&]() {
         const auto C     = rand_V();
         const auto ray_x = rand_V();
         const auto ray_y = rand_V();
         const auto X     = rand_V();
         const auto N     = rand_V();

         const Vector3 P1 = ray_x.normalised();
         const Vector3 P2 = ray_y.normalised();

         // Find the rotation for W1
         const real cos_t = std::clamp(dot(P1, P2), -1.0, 1.0);
         const real theta = acos(cos_t);
         const auto p3    = Plane(C, X, X + N);
         const auto q     = axis_angle_to_quaternion(Vector4(p3.xyz(), theta));

         const Vector3 W1  = (X - C).normalised();
         const Vector3 W2a = q.rotate(W1);
         const Vector3 W2b = q.inverse_rotate(W1);

         const auto [q0, q1] = calc_rotation(C, ray_x, ray_y, X, N);

         CATCH_REQUIRE((dot(W1, W2a) - cos_t) < 1e-6);
         CATCH_REQUIRE((dot(W1, W2a) - cos_t) < 1e-6);
      };

      for(auto i = 0; i < 1000; ++i) test_it();
   }
}

} // namespace perceive
