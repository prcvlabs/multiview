
#include "online-stereo-calibration.hpp"

#include "perceive/calibration/find-F.hpp"
#include "perceive/calibration/plane-set/cps-operations.hpp"
#include "perceive/cost-functions/features-2d/features-2d.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/geometry/projective/polynomial-model.hpp"
#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/canny.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/cv-helpers.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/graphics/tiny-string.hpp"
#include "perceive/io/fp-io.hpp"
#include "perceive/io/perceive-assets.hpp"
#include "perceive/utils/cli-utils.hpp"
#include "perceive/utils/eigen-helpers.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/tick-tock.hpp"

// --------------------------------------------------------------------- Summary
// (*) Use SURF/SIFT/Harris to create corners in one or more images
//     - Output images
// (*) Select a matching function. (e.g., SIFT-to-SIFT distance)
// (*) Map (distorted) corner features to ideal points
// (*) Run rounds of RANSAC to find 'E'
// (*) Output images
// (*) Write json file
// -----------------------------------------------------------------------------

namespace perceive::calibration
{
// ------------------------------------------------------------------ CornerPair
//
struct CornerPair
{
   Vector2 u0, u1; // undistorted
   Point2 p0, p1;  // The corners...
   int ind0 = 0;
   int ind1 = 0;
   real score; // in [0..1], with lower better

   string to_string() const noexcept
   {
      return format(R"V0G0N(
CornerPair
   Indices:   [{}, {}]
   Distorted: {{}, {}}   {{}, {}}
   Ideal:     {{}, {}}   {{}, {}}
   Score:      {}
)V0G0N",
                    ind0,
                    ind1,
                    p0.x,
                    p0.y,
                    p1.x,
                    p1.y,
                    u0.x,
                    u0.y,
                    u1.x,
                    u1.y,
                    score);
   }
};

inline string str(const CornerPair& cp) { return cp.to_string(); }

inline string str(const Vector3r& x) { return str(to_vec3(x)); }

// ------------------------------------------------------------- to-homgen-vec3r

inline Vector3r to_homgen_vec3r(const Vector2& x)
{
   return Vector3r(x.x, x.y, 1.0);
}

// -------------------------------------------------------------- print cps info

static void print_cps_info(const vector<CornerPair>& cps,
                           const BinocularCameraInfo& bcam_info,
                           const cv::Mat in_im[2],
                           const string& outdir)
{
   array<ARGBImage, 2> im;
   for(size_t i = 0; i < 2; ++i) {
      cv::Mat grey;
      if(in_im[i].channels() == 1)
         grey = in_im[i];
      else
         cv::cvtColor(in_im[i], grey, cv::COLOR_BGR2GRAY);
      im[i] = cv_to_argb(grey);
   }

   // Output the epipolar error, etc.
   for(auto i = 0u; i < cps.size(); ++i) {
      const auto& cp = cps[i];
      auto X = bcam_info.solve3d_from_distorted(to_vec2(cp.p0), to_vec2(cp.p1));
      cout << format("#{:3d}, |X| = |{}| = {}, from {}, {}",
                     i,
                     str(X),
                     X.norm(),
                     str(cp.p0),
                     str(cp.p1))
           << endl;
   }

   // Drawing the result
   for(auto i = 0u; i < cps.size(); ++i) {
      const auto& cp = cps[i];
      draw_cross(im[0], cp.p0, k_yellow, 5);
      draw_cross(im[1], cp.p1, k_yellow, 5);
      const auto offset = Point2(2, 2);
      render_string(im[0], format("{}", i), cp.p0 + offset, k_cyan, k_black);
      render_string(im[1], format("{}", i), cp.p1 + offset, k_cyan, k_black);
   }

   for(size_t i = 0; i < 2; ++i)
      im[i].save(format("{}/_{}_cps-info.png", outdir, i));
}

// ---------------------------------------------------- test plane ray intersect

static void test_plane_ray_intersect(const vector<CornerPair>& cps,
                                     const BinocularCameraInfo& bcam_info,
                                     const EuclideanTransform& extrinsic,
                                     const cv::Mat in_im[2],
                                     const string& outdir)
{
   LOG_ERR(format("HERE"));

   // const auto w = in_im[0].cols;
   // const auto h = in_im[0].rows;
   // Matrix3r K   = Matrix3r::Identity();
   // K(0, 0) = K(1, 1) = 400.0;
   // K(0, 2)           = 0.5 * w;
   // K(1, 2)           = 0.5 * h;

   // BinocularCamera bcam;
   // bcam.init(bcam_info, w, h, K, w, h, false);

   // PlaneOpsCalculator pcalc;
   // pcalc.init(bcam_info, extrinsic);

   // const auto p3 = Plane{0.0, 0.0, 1.0, 0.0}; //

   // for(auto i = 0u; i < cps.size(); ++i) {
   //    const auto& cp = cps[i];
   //    auto X = bcam_info.solve3d_from_distorted(to_vec2(cp.p0),
   //    to_vec2(cp.p1)); auto Y = extrinsic.apply(X); auto Z0 =
   //    bcam.plane_ray_intersect(CAM0, extrinsic, p3, to_vec2(cp.p0)); auto Z1
   //    = bcam.plane_ray_intersect(CAM1, extrinsic, p3, to_vec2(cp.p1));

   //    // Let's project back
   //    auto z00
   //        = bcam.backproject_to_distorted(CAM0, extrinsic.inverse_apply(Z0));
   //    auto z01
   //        = bcam.backproject_to_distorted(CAM0, extrinsic.inverse_apply(Z1));
   //    auto z10
   //        = bcam.backproject_to_distorted(CAM1, extrinsic.inverse_apply(Z0));
   //    auto z11
   //        = bcam.backproject_to_distorted(CAM1, extrinsic.inverse_apply(Z1));

   //    auto W0 = pcalc.plane_ray_intersect(CAM0, p3, to_vec2(cp.p0));
   //    auto W1 = pcalc.plane_ray_intersect(CAM1, p3, to_vec2(cp.p1));

   //    auto w00 = pcalc.project_to_distorted(CAM0, Z0);
   //    auto w01 = pcalc.project_to_distorted(CAM0, Z1);
   //    auto w10 = pcalc.project_to_distorted(CAM1, Z0);
   //    auto w11 = pcalc.project_to_distorted(CAM1, Z1);

   //    cout << format(
   //        "Z0: |{} - {}| = {}\n", str(Z0), str(W0), (Z0 - W0).norm());
   //    cout << format(
   //        "Z1: |{} - {}| = {}\n", str(Z1), str(W1), (Z1 - W1).norm());

   //    cout << format(
   //        "w00: |{} - {}| = {}\n", str(w00), str(z00), (w00 -
   //        z00).norm());
   //    cout << format(
   //        "w01: |{} - {}| = {}\n", str(w01), str(z01), (w01 -
   //        z01).norm());
   //    cout << format(
   //        "w10: |{} - {}| = {}\n", str(w10), str(z10), (w10 -
   //        z10).norm());
   //    cout << format(
   //        "w11: |{} - {}| = {}\n", str(w11), str(z11), (w11 -
   //        z11).norm());

   //    cout << format("#{:3d}, Y = {}, Z0 = {}, Z1 = {}, dist = {},
   //    from {},
   //    {}, "
   //                   "cam0 = {}, {}, cam1 = {}, {}",
   //                   i,
   //                   str(Y),
   //                   str(Z0),
   //                   str(Z1),
   //                   X.norm(),
   //                   str(cp.p0),
   //                   str(cp.p1),
   //                   str(z00),
   //                   str(z01),
   //                   str(z10),
   //                   str(z11))
   //         << endl
   //         << endl;

   //    if(i == 5) {
   //       cv::Mat mapx, mapy, dst0, dst1;

   //       INFO("HERE");
   //       pcalc.make_mapxy(true, p3, w, h, mapx, mapy);
   //       cv::remap(in_im[0], dst1, mapx, mapy, cv::INTER_LINEAR);
   //       pcalc.make_mapxy(false, p3, w, h, mapx, mapy);
   //       cv::remap(in_im[1], dst0, mapx, mapy, cv::INTER_LINEAR);
   //       cv::imwrite("/tmp/www-im0.png", in_im[0]);
   //       cv::imwrite("/tmp/www-ds0.png", dst0);
   //       cv::imwrite("/tmp/www-im1.png", in_im[1]);
   //       cv::imwrite("/tmp/www-ds1.png", dst1);

   //       FATAL("kBAM!");
   //    }
   // }
}

// -------------------------------------------------------- scharr-cost-function

// Dot product between the two fields, at the two points, using a circular
// region
static real scharr_patch_match(const Point2 p0,
                               const Field& scharr0,
                               const Point2 p1,
                               const Field& scharr1,
                               int radius)
{
   real score  = 0.0;
   int counter = 0;
   for(auto dy = -radius; dy <= radius; ++dy) {
      for(auto dx = -radius; dx <= radius; ++dx) {
         if(square(dx) + square(dy) > square(radius)) continue;
         const Point2 o0 = p0 + Point2(dx, dy);
         const Point2 o1 = p1 + Point2(dx, dy);
         if(!scharr0.in_bounds(o0)) return std::numeric_limits<real>::max();
         if(!scharr1.in_bounds(o1)) return std::numeric_limits<real>::max();
         const auto& u        = scharr0(o0);
         const auto& v        = scharr1(o1);
         const auto cos_theta = 0.5 + (0.5 * dot(u, v) / (u.norm() * v.norm()));
         if(std::isnan(cos_theta)) continue;
         score += cos_theta;
         counter++;
      }
   }
   return 1.0 - (score / real(counter));
}

// ----------------------------------------------------- test-scharr-patch-match
//
template<typename S, typename H>
static void test_scharr_patch_match(const OnlineStereoParams& params,
                                    const cv::Mat grey[2],
                                    const S& scharr,
                                    const vector<CornerPair>& cps,
                                    const H& harris)
{
   ARGBImage argb;
   const int patch_sz = params.match_aperture;
   auto test_match    = [&](int ind0) {
      vector<CornerPair> matches;
      std::copy_if(begin(cps),
                   end(cps),
                   std::back_inserter(matches),
                   [&](auto& cp) { return cp.ind0 == ind0; });

      cv_to_argb(grey[1], argb);

      for(const auto& cp : matches) {
         const Point2 p0 = cp.p0;
         const Point2 p1 = cp.p1;
         const auto s    = cp.score;

         // k for outside circle
         const auto border_k
             = vector3_to_kolour(hsv_to_rgb(Vector3(0.0, 1.0 - s, 1.0)));

         // Visualize the match
         for(auto dy = -patch_sz; dy <= patch_sz; ++dy) {
            for(auto dx = -patch_sz; dx <= patch_sz; ++dx) {
               const Point2 o0 = p0 + Point2(dx, dy);
               const Point2 o1 = p1 + Point2(dx, dy);
               if(!scharr[0].in_bounds(o0)) continue;
               if(!scharr[1].in_bounds(o1)) continue;
               if(square(dx) + square(dy) >= square(patch_sz)
                  and square(dx) + square(dy) <= square(patch_sz + 2)) {
                  argb(o1) = border_k;
               } else if(square(dx) + square(dy) < square(patch_sz)) {
                  const auto& u = scharr[0](o0);
                  const auto& v = scharr[1](o1);
                  const auto cos_theta
                      = 0.5 + (0.5 * dot(u, v) / (u.norm() * v.norm()));
                  const auto k = vector3_to_kolour(
                      hsv_to_rgb(Vector3(200.0, 1.0 - cos_theta, 1.0)));
                  argb(o1) = k;
               }
            }
         }

         argb.save(format("{}/d_1_corner={}.png", params.outdir, ind0));
      }
   };

   test_match(63);
   test_match(38);
} // namespace perceive::calibration

// ----------------------------------------------------------- make-corner-pairs
//
template<typename P, typename T>
static CornerPair make_corner_pair(const P& M,
                                   const int ind0,
                                   const int ind1,
                                   const T& all_corners,
                                   const real score)
{
   CornerPair cp;
   Vector2 x = all_corners[0][size_t(ind0)];
   Vector2 y = all_corners[1][size_t(ind1)];
   cp.u0     = M[0].undistort(x);
   cp.u1     = M[1].undistort(y);
   cp.p0     = to_pt2(x);
   cp.p1     = to_pt2(y);
   cp.ind0   = ind0;
   cp.ind1   = ind1;
   cp.score  = score;
   return cp;
}

template<typename P, typename S, typename T>
static vector<CornerPair> make_corner_pairs(const OnlineStereoParams& params,
                                            const P& M,
                                            const S& scharr,
                                            const T& all_corners)
{
   bool feedback = false;
   auto process  = [&](vector<CornerPair>& cps,
                      size_t ind0,
                      size_t ind1,
                      bool always_add) {
      Vector2 x = all_corners[0][ind0];
      Vector2 y = all_corners[1][ind1];
      if(feedback)
         cout << format("make corners {}, {}", str(x), str(y)) << endl;

      if(!always_add and fabs(x.x - y.x) > params.distorted_x_threshold) return;
      if(!always_add and fabs(x.y - y.y) > params.distorted_y_threshold) return;

      const real score = scharr_patch_match(
          to_pt2(x), scharr[0], to_pt2(y), scharr[1], params.match_aperture);
      if(feedback)
         cout << format(
             "  score = {}  (thes - {})\n", score, params.match_thres);
      if(always_add or score < params.match_thres)
         cps.push_back(
             make_corner_pair(M, int(ind0), int(ind1), all_corners, score));
   };

   vector<CornerPair> cps;
   if(params.in_corners.size()) {
      for(size_t i = 0; i < params.in_corners.size(); ++i)
         process(cps, i, i, true);

      std::stringstream ss("");
      for(const auto& cp : cps)
         ss << format("{} {} {} {}", cp.p0.x, cp.p0.y, cp.p1.x, cp.p1.y)
            << endl;
      file_put_contents("/tmp/points.text", ss.str());

   } else {
      cps.reserve(all_corners[0].size() * all_corners[1].size());
      for(auto ind0 = 0u; ind0 < all_corners[0].size(); ++ind0)
         for(auto ind1 = 0u; ind1 < all_corners[1].size(); ++ind1)
            process(cps, ind0, ind1, false);
   }

   if(false) {
      WARN(format("printing interesting corner-pairs:"));
      for(auto i = 0u; i < cps.size(); ++i) {
         if(cps[i].ind0 == 63 || cps[i].ind0 == 38)
            cout << format("#{:4d}: {}", i, str(cps[i])) << endl;
      }
   }

   return cps;
}

static void make_test_hypothesis(const vector<CornerPair>& cps,
                                 std::vector<Vector3r>& pts0,
                                 std::vector<Vector3r>& pts1)
{
   pts0.clear();
   pts1.clear();
   for(const auto& cp : cps) {
      pts0.push_back(to_homgen_vec3r(cp.u0));
      pts1.push_back(to_homgen_vec3r(cp.u1));
   }
}

static Matrix3r condition_E_(const Matrix3r& E)
{
   Matrix3r U, D, V;
   Vector3r ss;
   svd_UDV(E, U, ss, V);
   cout << "E   = " << endl << E << endl << endl;
   cout << "U   = " << endl << U << endl << endl;
   cout << "D   = " << endl << ss.transpose() << endl << endl;
   cout << "V   = " << endl << V << endl << endl;

   D = Matrix3r::Zero();
   for(auto i = 0; i < 2; ++i) D(i, i) = ss(i);
   Matrix3r UDV = U * D * V.transpose();

   cout << "UDV = " << endl << UDV << endl << endl;
   cout << "\u0394   = " << endl << (E - UDV) << endl << endl;

   D       = Matrix3r::Zero();
   D(0, 0) = ss(0);
   D(1, 1) = ss(1);
   // D(1, 1) = 0.5 * (ss(0) + ss(1));

   Matrix3r U_V = U * D * V.transpose();
   cout << "U_V = " << endl << U_V << endl << endl;
   cout << "\u0394   = " << endl << (E - U_V) << endl << endl;

   return U_V;
}

static void test_E_cps(const vector<CornerPair>& cps)
{
   std::vector<Vector3r> pts0, pts1;
   make_test_hypothesis(cps, pts0, pts1);
   Matrix3r E0 = longuet_higgins(pts0, pts1);
   Matrix3r E1 = estimate_F(pts0, pts1);
   Quaternion q;
   Vector3 t;
   estimate_Rt_from_E(E1, pts0, pts1, q, t, true, true);
   Matrix3r E2 = make_essential_matrix(q, t);

   INFO("longuet-higgins");
   Matrix3r E = E2;
   for(auto i = 0u; i < cps.size(); ++i) {
      auto dist = xFl_lFx(E, cps[i].u0, cps[i].u1);
      cout << format("#{:2d},  {}  {}  {}",
                     i,
                     400.0 * dist,
                     str(cps[i].p0),
                     str(cps[i].p1))
           << endl;
   }

   FATAL("kBAM!");
}

// -------------------------------------------------------------------
static std::pair<Quaternion, Vector3>
estimate_qt(const OnlineStereoParams& in_params,
            const cv::Mat grey[2],
            const vector<CornerPair>& cps)
{
   const auto N = cps.size();
   std::vector<Vector3r> pts0(N);
   std::vector<Vector3r> pts1(N);
   for(auto i = 0u; i < N; ++i) {
      pts0[i] = to_vec3r(homgen_R2_to_P2(cps[i].u0));
      pts1[i] = to_vec3r(homgen_R2_to_P2(cps[i].u1));
   }

   Matrix3r E0 = estimate_F(pts0, pts1);
   Quaternion q;
   Vector3 t;
   estimate_Rt_from_E(E0, pts1, pts0, q, t, false, true);

   return std::make_pair(q, t);
}

// ------------------------------------------------------------------- RANSAC qt
std::pair<Quaternion, Vector3> RANSAC_qt(const OnlineStereoParams& in_params,
                                         const cv::Mat grey[2],
                                         const vector<CornerPair>& cps)
{
   vector<int> indices(cps.size());
   std::iota(begin(indices), end(indices), 0);

   // Random number generator
   std::random_device rd;
   std::mt19937 pseudo_rand{rd()};

   // (*) Select a random subset of the original data
   auto random_hypothesis = [&]() -> auto
   {
      constexpr size_t N = 8;
      array<int, N> rand_indices;
      std::sample(
          begin(indices), end(indices), begin(rand_indices), N, pseudo_rand);

      std::vector<Vector3r> pts0(N);
      std::vector<Vector3r> pts1(N);
      for(size_t i = 0; i < N; ++i) {
         pts0[i] = to_vec3r(homgen_R2_to_P2(cps[size_t(rand_indices[i])].u0));
         pts1[i] = to_vec3r(homgen_R2_to_P2(cps[size_t(rand_indices[i])].u1));
      }

      Quaternion q;
      Vector3 t;
      Matrix3r E0 = estimate_F(pts0, pts1);
      estimate_Rt_from_E(E0, pts0, pts1, q, t, true, false);
      return std::make_pair(q, t);
   };

   // (*) Count the inlier set
   auto find_inlier_set = [&](Quaternion q, Vector3 t, int& counter) {
      counter = 0;
      vector<bool> inliers(cps.size());
      Matrix3r E = make_essential_matrix(q, t);
      for(auto i = 0u; i < cps.size(); ++i) {
         const auto dist      = 400.0 * xFl_lFx(E, cps[i].u0, cps[i].u1);
         const auto is_inlier = dist < in_params.ransac_threshold;
         inliers[i]           = is_inlier;
         if(is_inlier) counter++;
      }
      return inliers;
   };

   // (*) Refine the hypothesis
   auto refine_hypothesis
       = [&](const size_t N, const std::vector<bool>& inliers) {
            vector<Vector3r> pts0, pts1;
            pts0.resize(N);
            pts1.resize(N);
            size_t pos = 0;
            for(auto i = 0u; i < cps.size(); ++i) {
               if(!inliers[i]) continue;
               Expects(pos < N);
               pts0[pos] = to_vec3r(homgen_R2_to_P2(cps[i].u0));
               pts1[pos] = to_vec3r(homgen_R2_to_P2(cps[i].u1));
               ++pos;
            }
            Expects(pos == N);
            Quaternion q;
            Vector3 t;
            if(pts0.size() >= 8) {
               Matrix3r E0 = estimate_F(pts0, pts1);
               estimate_Rt_from_E(E0, pts0, pts1, q, t, true, false);
            }
            return std::make_pair(q, t);
         };

   // (*) This is the RANSAC algorithm
   int best_score = 0;
   Quaternion best_q;
   Vector3 best_t;
   vector<bool> best_inliers;
   for(auto i = 0; i < 1000; ++i) {
      int n_inliers      = 0;
      const auto [q, t]  = random_hypothesis();
      const auto inliers = find_inlier_set(q, t, n_inliers);
      if(n_inliers > best_score and n_inliers >= 3) {
         best_inliers = inliers;
         best_score   = n_inliers;
         std::tie(best_q, best_t)
             = refine_hypothesis(size_t(n_inliers), inliers);
         cout << format("#{:5d} :: best = {} / {}%\n",
                        i,
                        n_inliers,
                        100.0 * real(n_inliers) / real(inliers.size()));
      }
   }

   // (*) Draw the result
   {
      // what's the mest match?
      Matrix3r E = make_essential_matrix(best_q, best_t);

      // Get only the inliers
      auto ii = std::max_element(begin(cps), end(cps), [&](auto& a, auto& b) {
         auto a_in = best_inliers[size_t(&a - &cps[0])];
         auto b_in = best_inliers[size_t(&b - &cps[0])];
         if(a_in and !b_in) return true;
         if(b_in and !a_in) return true;
         return xFl_lFx(E, a.u0, a.u1) > xFl_lFx(E, b.u0, b.u1);
      });

      ARGBImage argb    = hcat(cv_to_argb(grey[0]), cv_to_argb(grey[1]));
      const auto offset = Point2(grey[0].cols, 0);

      int counter = 0;
      for(auto i = 0u; i < cps.size(); ++i) {
         if(!best_inliers[i]) continue;
         if(++counter % 10 != 0) continue;
         const auto p0 = cps[i].p0;
         const auto p1 = cps[i].p1;
         plot_line_AA(p0, p1 + offset, [&](int x, int y, float alpha) {
            if(argb.in_bounds(x, y))
               set(argb, x, y, blend(argb(x, y), k_yellow, 1.0f - alpha));
         });
         draw_cross(argb, p0, k_red, 5);
         draw_cross(argb, p1 + offset, k_red, 5);
      }
      argb.save(format("{}/z_matches.png", in_params.outdir));
   }

   auto calc_err = [&]() {
      Matrix3r E   = make_essential_matrix(best_q, best_t);
      auto err     = 0.0;
      auto counter = 0;
      for(auto i = 0u; i < cps.size(); ++i) {
         if(best_inliers[i]) {
            err += xFl_lFx(E, cps[i].u0, cps[i].u1);
            ++counter;
         }
      }
      return 400.0 * err / real(counter);
   };

   // if(fix_result) {
   //    Matrix3r E = make_essential_matrix(best_q, best_t);
   //    for(auto i = 0u; i < cps.size(); ++i) {
   //       const auto& cp = cps[i];
   //       auto dist      = xFl_lFx(E, cp.u0, cp.u1);
   //       cout << format("#{:2d},  {}  {}  {}",
   //                      i,
   //                      400.0 * dist,
   //                      str(cp.p0),
   //                      str(cp.p1))
   //            << endl;
   //    }
   // }

   INFO(format("RANSAC err = {}", calc_err()));

   return std::make_pair(best_q, best_t);
}

// -------------------------------------------------------------- epipolar-curve

static void make_epipolar_curve(const Vector2& x,
                                const DistortionModel& M0,
                                const DistortionModel& M1,
                                const real min_dist,
                                const real max_dist,
                                const Quaternion& q,
                                const Vector3& t,
                                const real baseline,
                                Vector2& a,
                                Vector2& b,
                                Vector2& c)
{
   const bool feedback = false;
   a = b = c = x;

   // First, must back-project 'X'
   const auto Xr = homgen_R2_to_P2(M0.undistort(x)).normalized();
   assert(fabs(Xr.norm() - 1.0) < 1e-6);

   const auto C0 = Vector3(0.0, 0.0, 0.0);
   const auto C1 = -baseline * q.inverse_rotate(t.normalized());

   if(feedback) {
      INFO(format("make-epipolar-curve: {}", str(x)));
      cout << format("C0 = {}", str(C0)) << endl;
      cout << format("C1 = {}", str(C1)) << endl;
      cout << format("Xr = {}", str(Xr)) << endl;
   }

   auto project_cam1 = [&](const Vector3& X) -> Vector2 {
      if(feedback) {
         cout << format("X  = {}", str(X)) << endl;
         cout << format("X-1= {}", str(X - C1)) << endl;
         cout << format("X1 = {}", str(q.rotate(X - C1))) << endl;
         cout << format("x1 = {}", str(homgen_P2_to_R2(q.rotate(X - C1))))
              << endl;
         cout << format("d1 = {}",
                        str(M1.distort(homgen_P2_to_R2(q.rotate(X - C1)))))
              << endl
              << endl;
      }
      auto Xr = homgen_P2_to_R2(q.rotate(X - C1));
      return M1.distort(Xr);
   };

   a = project_cam1(C0 + min_dist * Xr);
   c = project_cam1(C0 + max_dist * Xr);

   Vector2 a1 = project_cam1(C0 + (min_dist + 1e-5) * Xr);
   Vector2 c1 = project_cam1(C0 + (max_dist - 1e-5) * Xr);

   // 'b' is the intersection of the line [a-a1] and [c-c1]
   b = homgen_P2_to_R2(cross(to_homgen_line(a, a1), to_homgen_line(c, c1)));

   if(feedback) {
      cout << format("A  = {}", str(a)) << endl;
      cout << format("B  = {}", str(b)) << endl;
      cout << format("C  = {}", str(c)) << endl;
      cout << "." << endl;
      cout << endl;
   }
}

// ------------------------------------------------------------------- refine-qt
std::pair<Quaternion, Vector3> refine_qt(const OnlineStereoParams& params,
                                         const vector<CornerPair>& cps,
                                         const BinocularCameraInfo& bcam_info,
                                         const cv::Mat grey[2],
                                         const Field scharr[2])
{
   const auto outdir   = params.outdir;
   const auto baseline = 0.16;
   const auto mn_d     = 1.5;
   const auto mx_d     = 7.0;

   print_cps_info(cps, bcam_info, grey, outdir);

   FATAL("kBAM!");

   // How does this look???
   std::for_each(cbegin(cps), cend(cps), [&](const auto& cp) {
      const auto d0 = to_vec2(cp.p0);
      const auto d1 = to_vec2(cp.p1);
      const auto u0 = bcam_info.M[0].undistort(d0);
      const auto u1 = bcam_info.M[1].undistort(d1);
      Matrix3r R    = quaternion_to_rot3x3(bcam_info.q);
      const auto t  = bcam_info.baseline * bcam_info.t.normalised();
      auto X        = bcam_info.solve3d_from_distorted(d0, d1);
      auto Y        = triangulate(u0, u1, R, t);
      auto e        = bcam_info.reproj_err_from_distorted(d0, d1);

      cout << format(" + {} {} ==> {} {}.  X = |{}| = {}, Y = "
                     "|{}| err = {}",
                     str(cp.p0),
                     str(cp.p1),
                     str(bcam_info.project(CAM0, X)),
                     str(bcam_info.project(CAM1, X)),
                     str(X),
                     X.norm(),
                     str(Y),
                     e)
           << endl;
   });

   FATAL("kBAM!");

   const auto& M0 = bcam_info.M[0];
   const auto& M1 = bcam_info.M[1];
   auto q         = bcam_info.q;
   auto t         = bcam_info.t;

   // (*) Run Canny on grey
   // We have the scharr lookup...
   // Using epipolar lines, find the four top matches for plane-plane-ray-ray
   // Do N rounds of (parallel) belief propagation

   auto do_canny = [](const cv::Mat& grey) {
      GreyImage g = cv_to_grey(grey);
      GreyImage out;
      out.resize(g.width, g.height);
      canny(g.data(), g.width, g.height, out.data());
      return out;
   };

   GreyImage edges[2];
   for(auto i = 0; i < 2; ++i) {
      edges[i] = do_canny(grey[i]);
      edges[i].save(format("{}/q_canny_{}.png", outdir, i));
   }

   Field fedges[2];
   for(auto i = 0; i < 2; ++i) {
      auto& f = fedges[i];
      f.resize(edges[i].width, edges[i].height);
      f.fill(Vector2(0.0, 0.0));
      for(auto y = 0u; y < edges[i].height; ++y)
         for(auto x = 0u; x < edges[i].width; ++x)
            if(edges[i](x, y) > 0) f(x, y) = scharr[i](x, y);
      field_to_colour(f).save(format("{}/r_canny_{}.png", outdir, i));
   }

   { // (*) Let's look at those epipolar curves
      ARGBImage argb     = field_to_colour(fedges[1]);
      auto draw_epipolar = [&](Vector2 p0) {
         Vector2 A, B, C;
         make_epipolar_curve(p0, M0, M1, mn_d, mx_d, q, t, baseline, A, B, C);
         plot_quad_bezier_AA(A, B, C, [&](int x, int y, float a) {
            if(argb.in_bounds(x, y))
               set(argb, x, y, blend(argb(x, y), k_white, 1.0f - a));
         });
      };

      auto tcps = cps;
      for(auto cp : tcps) {
         draw_epipolar(to_vec2(cp.p0));
         draw_cross(argb, cp.p1, k_white, 5);
      }

      argb.save(format("{}/t_epipolar_1.png", outdir));
   }

   constexpr int M = 4; // keep top 4 scores
   using node_t    = array<float, M>;
   using NodeImage = ImageContainerT<node_t>;
   NodeImage three_d;
   three_d.resize(grey[0].cols, grey[0].rows);

   return std::make_pair(q, t);
}

// ----------------------------------------------- run-online-stereo-calibration

bool run_online_stereo_calibration(const OnlineStereoParams& in_params)
{
   // ---- Parameters ----
   const auto camera_id             = in_params.camera_id;
   const auto cam0_fname            = in_params.cam0_fname;
   const auto cam1_fname            = in_params.cam1_fname;
   const auto image_fname           = in_params.image_fname;
   const auto outdir                = in_params.outdir;
   const auto match_aperture        = in_params.match_aperture;
   const auto match_thres           = in_params.match_thres;
   const auto distorted_y_threshold = in_params.distorted_y_threshold;
   const auto extrinsic             = in_params.extrinsic;

   // ---- Load the image ----
   cv::Mat im01 = cv::imread(image_fname);
   cv::Mat in_im[2];
   hsplit(im01, in_im[0], in_im[1]);
   cv::Mat grey[2];
   for(auto i = 0; i < 2; ++i) {
      cv::cvtColor(in_im[i], grey[i], cv::COLOR_BGR2GRAY);
      cv::imwrite(format("{}/a_{}_grey.png", outdir, i), grey[i]);
   }
   const auto w = grey[0].cols;
   const auto h = grey[0].rows;
   INFO(format("loaded '{}' to produce {}x{} image pair", image_fname, w, h));

   // ---- Load camera models ----
   BinocularCameraInfo bcam_info;
   fetch(bcam_info.M[0], cam0_fname);
   fetch(bcam_info.M[1], cam1_fname);
   for(size_t i = 0; i < 2; ++i) {
      bcam_info.M[i].set_working_format(unsigned(grey[0].cols),
                                        unsigned(grey[0].rows));
   }

   // ---- Run Harris/MinEigen/Scharr ----
   ImageFeatures2d f2d[2];
   ImageFeatures2d::Params params;
   const auto is_cancelled = []() { return false; };

   vector<Vector2> harris[2], min_eigen[2], all_corners[2];
   Field scharr[2];

   for(auto i = 0; i < 2; ++i) {
      run_scharr(f2d[i], grey[i], params, is_cancelled);
      scharr[i] = f2d[i].scharr;

      if(in_params.in_corners.size() > 0) {
         transform(begin(in_params.in_corners),
                   end(in_params.in_corners),
                   back_inserter(all_corners[i]),
                   [&](auto pq) { return (i == 0) ? pq.first : pq.second; });
      } else {
         if(in_params.use_harris) {
            params.use_harris = true;
            run_harris(f2d[i], grey[i], params, is_cancelled);
            harris[i] = f2d[i].corners;
            f2d[i].corners.clear();
         }

         if(in_params.use_min_eigen) {
            params.use_harris = false;
            run_harris(f2d[i], grey[i], params, is_cancelled);
            min_eigen[i] = f2d[i].corners;
            f2d[i].corners.clear();
         }

         all_corners[i].reserve(harris[i].size() + min_eigen[i].size());
         all_corners[i].insert(
             end(all_corners[i]), begin(harris[i]), end(harris[i]));
         all_corners[i].insert(
             end(all_corners[i]), begin(min_eigen[i]), end(min_eigen[i]));
      }
   }

   for(auto i = 0; i < 2; ++i) {
      ARGBImage argb;
      cv_to_argb(grey[i], argb);
      int counter = 0;
      for(const auto x : min_eigen[i]) draw_cross(argb, to_pt2(x), k_yellow, 5);
      for(const auto x : harris[i]) {
         draw_cross(argb, to_pt2(x), k_red, 5);
         render_string(
             argb, format("{}", counter++), to_pt2(x), k_cyan, k_black);
      }
      argb.save(format("{}/b_{}_corners.png", outdir, i));
      field_to_argb(scharr[i]).save(format("{}/c_{}_scharr.png", outdir, i));
   }

   const bool fix_result = (in_params.in_corners.size() > 0);

   // ---- Create potentional inliers ----
   const auto cps
       = make_corner_pairs(in_params, bcam_info.M, scharr, all_corners);
   INFO(format("cps.size() = {}", cps.size()));

   // ---- Look at ----
   // C1:          [0.16428, 0.00310, -0.00917]
   // rot:         3.868235 deg about [0.36436, -0.91404, -0.17822]
   // baseline:    0.164561
   bcam_info.q = axis_angle_to_quaternion(
       Vector4(0.36436, -0.91404, -0.17822, to_radians(3.868235)));
   bcam_info.t
       = -bcam_info.q.rotate(Vector3(0.16428, 0.00310, -0.00917)).normalised();
   bcam_info.baseline = 0.164561;

   print_cps_info(cps, bcam_info, in_im, outdir);

   // ---- Test Scharr Patch Match ----
   // test_scharr_patch_match(in_params, grey, scharr, cps, harris);

   // ---- Test bcam::plane-ray-intersect
   test_plane_ray_intersect(cps, bcam_info, extrinsic, in_im, outdir);

   FATAL("kBAM!");

   // ---- RANSAC the best F ----
   Quaternion q;
   Vector3 t;
   if(fix_result) {
      std::tie(q, t) = estimate_qt(in_params, grey, cps);
   } else {
      std::tie(q, t) = RANSAC_qt(in_params, grey, cps);
   }
   INFO(format("best q = {}, t = {}", str(q), str(t)));

   // ---- Canny Point Cloud ----
   bcam_info.q        = q;
   bcam_info.t        = t;
   bcam_info.baseline = 0.16;
   std::tie(q, t)     = refine_qt(in_params, cps, bcam_info, grey, scharr);

   return true;
}
} // namespace perceive::calibration
