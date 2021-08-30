
#ifdef USING_OPENGL

#include <algorithm>
#include <iterator>

#include "gl/gl-projection.hpp"
#include "gl/gl-utils.hpp"
#include "perceive/contrib/catch.hpp"
#include "perceive/utils/file-system.hpp"

namespace perceive
{
template<typename T> static real angle_between_vectors(const T& u, const T& v)
{
   auto cos_theta = dot(u, v) / (u.norm() * v.norm());
   return acos(clamp(cos_theta, -1.0, 1.0));
}

TEST_CASE("gl-util-tests", "[test_gl_viewport]")
{
   SECTION("glviewport")
   {
      GlProjection port;

      port.init(0, 0, 640, 480, to_radians(100.0), 0.3, 20.0);

      const auto t  = Vector3{0.94125, -2.86282, -0.65380};
      const auto aa = Vector4{0.49732, -0.20578, -0.84281, to_radians(284.130)};
      port.set_cam_et_from_perceive_cam_et(EuclideanTransform(t, aa, 1.0));

      auto test_X = [&](const Vector3 W) {
         const auto E = port.world_to_eye(W);
         const auto R = E.normalised(); // the ray
         const auto C = port.world_to_clip(W);
         const auto N = port.world_to_NDC(W);
         const auto S = port.world_to_screen(W);

         const auto z = port.convert_z(0.5 * (N.z + 1.0));
         const auto n = port.screen_to_NDC(S);
         const auto e = port.screen_to_eye(S, z);
         const auto r = port.screen_to_ray(S);

         const auto S1 = port.eye_to_screen(e);

         if(false) {
            cout << string(70, '-') << endl;
            cout << format("W   = {:s}", str(W)) << endl;
            cout << format("E   = {:s}", str(E)) << endl;
            cout << format("R   = {:s}", str(R)) << endl;
            cout << format("C   = {:s}", str(C)) << endl;
            cout << format("N   = {:s}", str(N)) << endl;
            cout << format("S   = {:s}", str(S)) << endl;
            cout << "-" << endl;
            cout << format("n   = {:s}", str(n)) << endl;
            cout << format("z   = {}", z) << endl;
            cout << format("r   = {:s}", str(r)) << endl;
            cout << format("e   = {:s}", str(e)) << endl;
            cout << "-" << endl;
            cout << format("S1  = {:s}", str(S1)) << endl;

            cout << endl << endl;
         }

         REQUIRE((n - Vector2(N.x, N.y)).norm() < 1e-9);
         REQUIRE(fabs(z - E.z) < 1e-6);
         REQUIRE((e - E).norm() < 1e-9);
         REQUIRE(angle_between_vectors(R / R(2), r / r(2)) < 1e-3);
         REQUIRE((S1 - S).norm() < 1e-3);
      };

      const auto X1 = Vector3{2.645, 3.430, 0.740};
      const auto X2 = Vector3{2.645, 1.080, 0.740};
      const auto X3 = Vector3{1.640, 1.080, 0.740};
      const auto X4 = Vector3{1.640, 3.430, 0.740};

      test_X(X1);
      test_X(X2);
      test_X(X3);
      test_X(X4);
   }
}

} // namespace perceive

#endif
