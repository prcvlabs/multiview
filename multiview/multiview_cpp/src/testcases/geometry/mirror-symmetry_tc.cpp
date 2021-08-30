
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <deque>
#include <iterator>

#include "perceive/contrib/catch.hpp"
#include "perceive/geometry/projective/mirror-symmetry.hpp"
#include "perceive/geometry/rotation.hpp"

namespace perceive
{
CATCH_TEST_CASE("MirrorSymmetry", "[mirror-symmetry]")
{
   // This code should just finish without tripping the memory sanitizer
   CATCH_SECTION("mirror-symmetry")
   {
      auto test_it = [](const Vector3 C, const Vector3 X, const Vector3 n) {
         const Vector3 A = X + 2.0 * n;
         const Vector3 B = X - 2.0 * n;
         const Vector3 u = (A - C).normalised();
         const Vector3 v = (B - C).normalised();
         const Plane p3  = Plane(n, -dot(n, X));

         const auto [U, V] = recover_mirror_symmetric_pair(C, p3, u, v);

         if(false) {
            const auto cos_theta = std::clamp<real>(u.dot(p3.xyz()), -1.0, 1.0);
            const auto cos_phi   = std::clamp<real>(v.dot(p3.xyz()), -1.0, 1.0);
            const auto theta     = acos(cos_theta);
            const auto phi       = acos(cos_phi);
            const auto ratio     = sin(theta) / sin(phi);
            const auto normU     = (A - C).norm();
            const auto normV     = (B - C).norm();
            INFO(format("FEEDBACK"));
            cout << format("C         = {}", str(C)) << endl;
            cout << format("X         = {}", str(X)) << endl;
            cout << format("p3        = {}", str(p3)) << endl;
            cout << format("n         = {}", str(n)) << endl;
            cout << format("u         = {}", str(u)) << endl;
            cout << format("v         = {}", str(v)) << endl;
            cout << format("cos_theta = {}", cos_theta) << endl;
            cout << format("cos_phi   = {}", cos_phi) << endl;
            cout << format("theta     = {}", to_degrees(theta)) << endl;
            cout << format("phi       = {}", to_degrees(phi)) << endl;
            cout << format("|U|,|V|   = {}, {}\n", normU, normV);
            cout << format("|V|/|U|   = {}\n", normV / normU);
            cout << format("sth/sph   = {}\n", sin(theta) / sin(phi));
            cout << format("|V| (fm)  = {}\n", normU * ratio);
            cout << format("-2d       = {}\n", -2.0 * p3.d());
            cout << format("side(A)   = {}\n", p3.side(A));
            cout << format("side(B)   = {}\n", p3.side(B));
            cout << format("side(C)   = {}\n", p3.side(C));
            cout << format("side(m)   = {}\n", 0.5 * dot(A + B, n) + p3.d());
            cout << format("side(m)   = {}\n", 0.5 * dot(A + B, n) + p3.d());
            cout << format("...       = {}\n",
                           normU * (dot(u, n) + ratio * dot(v, n)));
            cout << vec_feedback(A, U) << endl;
            cout << vec_feedback(B, V) << endl;
         }

         CATCH_REQUIRE(std::fabs(p3.side(X)) < 1e-6);
         CATCH_REQUIRE((A - U).norm() < 1e-6);
         CATCH_REQUIRE((B - V).norm() < 1e-6);
      };

      {
         const Vector3 C = {1.1, 4.0, 0.0};
         const Vector3 X = {2.0, 3.0, 4.0};
         const Vector3 n = Vector3(0.2, 1.0, 0.7).normalised();
         test_it(C, X, n);
      }

      std::mt19937 gen;
      std::uniform_real_distribution<double> distribution{-1.0, 1.0};
      gen.seed(0);
      auto rand_theta = [&]() { return distribution(gen) * M_PI; };
      auto rand_q     = [&]() {
         return saa_to_quaternion(
             Vector3(rand_theta(), rand_theta(), rand_theta()));
      };
      auto random_n
          = [&]() { return rand_q().rotate(Vector3(1, 0, 0)).normalised(); };

      for(auto i = 0; i < 1000; ++i) {
         const auto n  = random_n();
         const auto C  = distribution(gen) * 10.0 * random_n();
         const auto X  = distribution(gen) * 10.0 * random_n();
         const auto p3 = Plane(n, -dot(X, n));

         if(std::fabs(p3.side(C)) < 1e-2) continue; // degenerate case
         test_it(C, X, n);
      }
   }
}

} // namespace perceive
