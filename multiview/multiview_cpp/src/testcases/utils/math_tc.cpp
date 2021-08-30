
#include <algorithm>
#include <iterator>

#define CATCH_CONFIG_PREFIX_ALL

#include "perceive/contrib/catch.hpp"
#include "perceive/utils/detail/fastexp.hpp"
#include "perceive/utils/math.hpp"
#include "static_math/static_math.h"

namespace perceive
{
// ------------------------------------------------------------ combine mu-sigma

template<typename T>
inline std::pair<T, T>
combine_mu_sigma_(unsigned n0, T u0, T s0, unsigned n1, T u1, T s1) noexcept
{
   const auto n   = n0 + n1;
   const auto v0  = square(s0);
   const auto v1  = square(s1);
   const auto sum = u0 * n0 + u1 * n1;
   const auto ss  = n0 * (v0 + u0 * u0) + n1 * (v1 + u1 * u1);
   const auto u   = sum / T(n);
   const auto s   = ss / T(n) - square(u);
   return std::pair<T, T>(u, sqrt(s));
}

static real phi2_function(real z_score)
{
   constexpr real inv_sqrt_2 = 1.0 / smath::sqrt(2.0);
   return 0.5 * std::erfc(-inv_sqrt_2 * z_score);
}

CATCH_TEST_CASE("MathStuff", "[math_stuff_test_cases]")
{
   CATCH_SECTION("PhiFunction")
   {
      auto test_phi = [&](real z_score, real phi0) {
         auto phi1                 = phi_function(z_score);
         constexpr real inv_sqrt_2 = 1.0 / smath::sqrt(2.0);
         auto err                  = fabs(phi1 - phi0);

         if(err > 1e-9) {
            cout << format("phi({}) = {}, but expected {}, \u0394 = {}",
                           z_score,
                           phi1,
                           phi0,
                           err)
                 << endl;
            CATCH_REQUIRE(false);
         }
      };

      vector<Vector2> data{{{-3.0, 0.00134989803163},
                            {-1.0, 0.158655253931},
                            {0.0, 0.5},
                            {0.5, 0.691462461274},
                            {2.1, 0.982135579437}}};

      for(auto xy : data) test_phi(xy.x, xy.y);
   }

   CATCH_SECTION("IntDiv2")
   {
      auto test_it
          = [&](auto x, auto y, auto z) { CATCH_REQUIRE(int_div2(x, y) == z); };

      test_it(4, 2, 2);
      test_it(4u, 2u, 2u);

      test_it(5, 1, 5);
      test_it(-5, -1, 5);
      test_it(5, -1, -5);
      test_it(-5, 1, -5);

      test_it(5, 2, 3);
      test_it(-5, -2, 3);
      test_it(5, -2, -3);
      test_it(-5, 2, -3);
   }

   CATCH_SECTION("angle-diff")
   {
      auto theta_cmp = [](double t0, double t1) -> bool {
         return (angle_normalise2(t0) - angle_normalise2(t1)) < 1e-9;
      };

      auto run_test = [&](double phi, double theta) {
         if(false) {
            const auto t0 = angle_normalise2(phi);
            auto t1       = angle_normalise2(theta);
            if(t1 < t0) t1 += 2.0 * M_PI;

            auto delta = angle_diff(phi, theta);
            cout << "----------------------------------------------" << endl;
            cout << format(
                "phi   = {} => {}\n", to_degrees(phi), to_degrees(t0));
            cout << format(
                "theta = {} => {}\n", to_degrees(theta), to_degrees(t1));
            cout << format("delta = phi   - theta = {}\n", to_degrees(delta));
            cout << format("phi   = theta + delta = {}\n",
                           to_degrees(angle_normalise2(theta + delta)));
            cout << format("theta = phi   - delta = {}\n",
                           to_degrees(angle_normalise2(phi - delta)));
            cout << endl;
         }

         auto delta = angle_diff(phi, theta);
         CATCH_REQUIRE(theta_cmp(phi, theta + delta));
         CATCH_REQUIRE(theta_cmp(theta, phi - delta));

         // CATCH_REQUIRE(
         //     is_close(angle_normalise2(phi), angle_normalise2(theta +
         //     delta)));
         // CATCH_REQUIRE(is_close(theta, phi - delta));
      };

      run_test(to_radians(80.0), to_radians(-160.0));
      run_test(to_radians(-160), to_radians(80));
      run_test(to_radians(0), to_radians(10));
      run_test(to_radians(0), to_radians(100));
      run_test(to_radians(0), to_radians(190));
      run_test(to_radians(0), to_radians(280));
      run_test(to_radians(0), to_radians(-10));

      for(auto phi = -1000; phi <= 1000; phi += 17)
         for(auto theta = -1000; theta < 1000; theta += 19)
            run_test(to_radians(phi), to_radians(theta));
   }

   CATCH_SECTION("combine-mu-sigma")
   {
      std::mt19937 gen;
      gen.seed(0);
      std::uniform_real_distribution<double> distribution{0.0, 10.0};
      auto g        = [&]() { return distribution(gen); };
      auto rand_vec = [&](unsigned n) {
         vector<real> z(n);
         std::generate(begin(z), end(z), g);
         return z;
      };
      auto join_vecs = [&](const auto& a, const auto& b) {
         vector<real> c = a;
         c.insert(end(c), begin(b), end(b));
         return c;
      };

      auto test_it = [&](unsigned n0, unsigned n1) {
         vector<real> z0 = rand_vec(n0);
         vector<real> z1 = rand_vec(n1);
         vector<real> z2 = join_vecs(z0, z1);

         const auto ss0 = calc_sample_statistics(begin(z0), end(z0));
         const auto ss1 = calc_sample_statistics(begin(z1), end(z1));
         const auto ss2 = calc_sample_statistics(begin(z2), end(z2));

         const auto [mu, sigma] = combine_mu_sigma_(
             n0, ss0.average, ss0.stddev, n1, ss1.average, ss1.stddev);

         const auto mu_good    = std::fabs(mu - ss2.average) < 1e-9;
         const auto sigma_good = std::fabs(sigma - ss2.stddev) < 1e-9;

         // INFO(format("|{} - {}| = {}, |{} - {}| = {}",
         //             mu,
         //             ss2.average,
         //             std::fabs(mu - ss2.average),
         //             sigma,
         //             ss2.stddev,
         //             std::fabs(sigma - ss2.stddev)));

         CATCH_REQUIRE(mu_good);
         CATCH_REQUIRE(sigma_good);
      };

      if(false) {
         vector<real> A = {{1, 5, 4, 67, 1, 3, 3, 1, 7, 9}};
         vector<real> B = {{5, 4, 5, 1, 8, 3, 2}};
         vector<real> C = join_vecs(A, B);
         const auto ss0 = calc_sample_statistics(begin(A), end(A));
         const auto ss1 = calc_sample_statistics(begin(B), end(B));
         const auto ss2 = calc_sample_statistics(begin(C), end(C));

         cout << ss0.to_string() << endl;
         cout << ss1.to_string() << endl;
         cout << ss2.to_string() << endl;

         const auto [mu, var] = combine_mu_sigma_(
             ss0.N, ss0.average, ss0.stddev, ss1.N, ss1.average, ss1.stddev);

         cout << format("mu = {}, std = {}", mu, var) << endl << endl;
      }

      for(auto i = 1u; i < 10; ++i)
         for(auto j = 1u; j < 10; ++j) test_it(i, j);
   }

   CATCH_SECTION("fastexp")
   {
      const auto minval = -20.0f;
      const auto maxval = 20.0f;
      const auto step   = 0.00001f;
      auto val          = 0.0f;

      if(false) {
         profile_thunk(
             [&]() {
                for(auto x = minval; x <= maxval; x += step) val += std::exp(x);
             },
             100,
             "exp              ");

         profile_thunk(
             [&]() {
                for(auto x = minval; x <= maxval; x += step)
                   val -= fastexp<float, ExpApproximation::IEEE>(x);
             },
             100,
             "fastexp -    IEEE");

         profile_thunk(
             [&]() {
                for(auto x = minval; x <= maxval; x += step)
                   val -= fastexp<float, ExpApproximation::PRODUCT, 4>(x);
             },
             100,
             "fastexp - PRODUCT");
         profile_thunk(
             [&]() {
                for(auto x = minval; x <= maxval; x += step)
                   val -= fastexp<float, ExpApproximation::TAYLOR, 2>(x);
             },
             100,
             "fastexp - TAYLOR ");

         cout << val << endl;

         for(float xi = -20.0f; xi < 0.0f; xi += 1.0f) {
            std::vector<float> errs0, errs1, errs2;
            errs0.reserve(10000);
            errs1.reserve(10000);
            errs2.reserve(10000);
            for(float delta = 0.0f; delta < 1.0f; delta += 0.0001f) {
               const float x = xi + delta;
               errs0.push_back(
                   std::fabs(std::exp(x)
                             - fastexp<float, ExpApproximation::IEEE>(x))
                   / std::exp(x));
               errs1.push_back(
                   std::fabs(std::exp(x)
                             - fastexp<float, ExpApproximation::PRODUCT, 4>(x))
                   / std::exp(x));
               errs2.push_back(
                   std::fabs(std::exp(x)
                             - fastexp<float, ExpApproximation::TAYLOR, 4>(x))
                   / std::exp(x));
            }
            const auto stats0
                = calc_sample_statistics(begin(errs0), end(errs0));
            const auto stats1
                = calc_sample_statistics(begin(errs1), end(errs1));
            const auto stats2
                = calc_sample_statistics(begin(errs2), end(errs2));

            WARN(format("x in [{}, {})", xi, xi + 1.0f));
            cout << format("  min: {:5.4f}  med: {:5.4f}  max: {:5.4f}",
                           stats0.min,
                           stats0.median,
                           stats0.max)
                 << endl;
            cout << format("  min: {:5.4f}  med: {:5.4f}  max: {:5.4f}",
                           stats1.min,
                           stats1.median,
                           stats1.max)
                 << endl;
            cout << format("  min: {:5.4f}  med: {:5.4f}  max: {:5.4f}",
                           stats2.min,
                           stats2.median,
                           stats2.max)
                 << endl;
            cout << endl;
         }
      }
   }

   CATCH_SECTION("average-angles")
   {
      vector<float> vals;
      vector<float> shorts;

      std::mt19937 gen;
      gen.seed(0);
      std::uniform_real_distribution<float> distribution(-float(M_PI),
                                                         float(M_PI));

      auto test_short = [&](float ts, size_t sz) {
         shorts.resize(sz);
         for(size_t i = 0; i < sz; ++i)
            shorts[i] = ts + distribution(gen) * 0.01f
                        + (i % 3 != 0 ? 0.0f : float(M_PI));

         const float u = average_short_angles(cbegin(shorts), cend(shorts));
         CATCH_REQUIRE(std::fabs(to_degrees(short_angle_diff(u, ts))) < 0.1f);
      };

      auto test_it = [&](float ts, size_t sz) {
         vals.resize(sz);
         for(size_t i = 0; i < sz; ++i)
            vals[i] = ts + distribution(gen) * 0.01f;

         const float t = average_angles(cbegin(vals), cend(vals));
         CATCH_REQUIRE(std::fabs(to_degrees(angle_diff(t, ts))) < 0.1f);
      };

      for(size_t sz = 10000; sz < 10001; ++sz)
         for(auto t = -361.0f; t <= 361.0f; t += 1.0f)
            test_it(to_radians(t), sz);

      for(size_t sz = 10000; sz < 10001; ++sz)
         for(auto t = -361.0f; t <= 361.0f; t += 1.0f) test_short(t, sz);
   }

   CATCH_SECTION("math-round")
   {
      const float val = float(100.0 * M_PI);

      if(false) {
         INFO(format("M_PI = {}", val));
         for(int i = -9; i < 10; ++i) {
            cout << format("{} --> {:f}", i, round(val, i)) << endl;
         }
      }
   }
}

} // namespace perceive
