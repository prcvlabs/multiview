
#include "phase-position-helpers.hpp"

#include "find-homography.hpp"
#include "phase-position.hpp"

#include "perceive/calibration/aruco-cube.hpp"
#include "perceive/geometry/fitting-planes.hpp"
#include "perceive/geometry/projective/back-project-kite.hpp"
#include "perceive/geometry/projective/caching-undistort-inverse.hpp"
#include "perceive/geometry/rotation.hpp"
#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/cv-helpers.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/graphics/tiny-string.hpp"
#include "perceive/optimization/golden-section-search.hpp"
#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"
#include "perceive/utils/create-cv-remap.hpp"
#include "perceive/utils/eigen-helpers.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/opencv-helpers.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace perceive::calibration
{
// -----------------------------------------------------------------------------
// -------------------------------------------------------------- calc detect Ws
//
static vector<vector<array<Vector3, 4>>>
calc_detect_Ws(const ArucoCube& ac,
               const vector<ArucoCube::FaceDetection>& detects)
{
   vector<vector<array<Vector3, 4>>> detect_Ws(detects.size());
   {
      for(size_t i = 0; i < detects.size(); ++i) {
         const auto& detect    = detects[i];
         const auto& face_spec = ac.measures[size_t(detect.f_ind)];
         auto& Ws              = detect_Ws[i];
         Ws.resize(detect.quads.size());
         Expects(Ws.size() == detect.marker_ids.size());
         for(size_t j = 0; j < Ws.size(); ++j) {
            const int marker_id = detect.marker_ids[j];
            const int m_pos     = face_spec.marker_pos(marker_id);
            Expects(m_pos >= 0 and m_pos < 4);
            Ws[j] = face_spec.marker_3d_quad(m_pos);
         }
      }
   }
   return detect_Ws;
}

// -----------------------------------------------------------------------------
// ---------------------------------------------------- Calc Regularization Term

real calc_regularization_term(const ArucoCube& ac,
                              const ArucoCube::FaceDetection& detect,
                              const EuclideanTransform& et0,
                              const ArucoFaceProjectionInfo& pinfo,
                              const Matrix3r& K) noexcept
{
   const bool feedback = false;
   // Calculate the regularization term...
   auto err = 0.0;

   const auto& face_spec = ac.measures[size_t(detect.f_ind)];

   if(feedback) INFO("Regularization Feedback");

   for(auto i = 0u; i < 4; ++i) {
      const auto& X = pinfo.quad3d[i];
      const auto x  = homgen_P2_to_R2(to_vec3(K * to_vec3r(X)));

      const auto Y = et0.apply(face_spec.Xs[i]);
      const auto y = homgen_P2_to_R2(to_vec3(K * to_vec3r(Y)));

      if(feedback) {
         cout << format(
             "   x[{}] = |{} - {}| = {}", i, str(y), str(x), (x - y).norm())
              << endl;
      }

      const auto d = (x - y).norm();
      if(d > 50.0) err += square(d - 50.0);
   }

   if(feedback) { cout << endl << endl; }

   return 0.25 * err;
}

// -----------------------------------------------------------------------------
// ---------------------------------------------------------------- Estimate et0

EuclideanTransform estimate_et0(const ArucoCube& ac,
                                const vector<ArucoCube::FaceDetection>& detects,
                                const Matrix3r& K,
                                ARGBImage* argb_ptr) noexcept
{
   const Matrix3r K_inv = K.inverse();
   const Vector3 O{0.0, 0.0, 0.0};

   vector<Vector3> A, B;
   vector<Vector2> p2;

   // What is the error on this?
   for(const auto& d : detects) {
      const auto N          = d.quad3ds.size();
      const auto& face_spec = ac.measures[size_t(d.f_ind)];
      for(auto i = 0u; i < N; ++i) {
         const auto marker_id = d.marker_ids[i];
         const auto m_pos     = face_spec.marker_pos(marker_id);
         Expects(m_pos >= 0 and m_pos < 4);
         const auto w3  = face_spec.marker_3d_quad(m_pos);
         const auto& q2 = d.quads[i];
         const auto& q3 = d.quad3ds[i];

         A.insert(end(A), cbegin(w3), cend(w3));
         B.insert(end(B), cbegin(q3), cend(q3));
         p2.insert(end(p2), cbegin(q2), cend(q2));
      }
   }

   const EuclideanTransform et = transform_between(A, B);

   if(argb_ptr) {
      ARGBImage& argb = *argb_ptr;
      for(auto i = 0u; i < A.size(); ++i) {
         const auto& W   = A[i];        // world coord
         const auto& X   = B[i];        // Detection
         const auto Y    = et.apply(W); // world => CAM
         const Vector2 x = homgen_P2_to_R2(to_vec3(K * to_vec3r(X)));
         const Vector2 y = homgen_P2_to_R2(to_vec3(K * to_vec3r(Y)));

         draw_square(argb, to_pt2(p2[i]), k_blue, 4);
         bresenham(x, y, [&](int x, int y) {
            if(argb.in_bounds(x, y)) argb(x, y) = k_yellow;
         });
         draw_cross(argb, to_pt2(x), k_blue, 4);
         draw_cross(argb, to_pt2(y), k_red, 4);
      }
   }

   return et;
}

// --------------------------------------------------------- refine-pointwise-et
// refine-pointwise-et Use a pointwise cost-function to refine the `et` position
// from a set of `detects`
static std::pair<EuclideanTransform, bool>
refine_pointwise_et(const ArucoCube& ac,
                    const vector<ArucoCube::FaceDetection>& detects,
                    const CachingUndistortInverse& cu,
                    const string_view cam_pos_name,
                    const EuclideanTransform& in_et,
                    const bool feedback) noexcept
{
   EuclideanTransform et = in_et;
   const Vector3 O{0.0, 0.0, 0.0};
   const bool super_feedback = false;
   const int n_params        = 6;

   const std::function<real(const EuclideanTransform& et)> fn_et
       = ac.pointwise_error_fun(cu, detects);

   auto pack   = [&](real* X) { pack_et_6df(et, X); };
   auto unpack = [&](const real* X) { return et = unpack_et_6df(X); };

   vector<real> start(n_params);
   vector<real> step(n_params);
   vector<real> best_params(n_params);

   int counter     = 0;
   real best_score = std::numeric_limits<real>::max();
   auto fn         = [&](const real* X) -> real {
      const auto ret = fn_et(unpack(X));
      if(ret < best_score) {
         best_score = ret;
         std::copy(X, X + n_params, begin(best_params));
         if(super_feedback) {
            cout << format("   #{:8d} :: {:10.7f} :: {{}}",
                           counter,
                           best_score,
                           implode(X, X + n_params, ", "))
                 << endl;
         }
      }
      counter++;
      return ret;
   };

   auto do_refine = [&](bool use_nelder_mead) {
      vector<real> xmin((size_t(n_params)));
      real ynewlo   = dNAN;
      real ystartlo = dNAN;
      real reqmin   = 1e-7;
      real diffstep = 0.1;
      int kcount    = 10000000; // max interations
      int icount = 0, numres = 0, ifault = 0;
      const char* method = nullptr;

      if(!use_nelder_mead) {
         method = "levenberg-marquardt";
         levenberg_marquardt(fn,
                             n_params,
                             &start[0],
                             &xmin[0],
                             reqmin,
                             diffstep,
                             10,
                             kcount,
                             icount,
                             ifault);
         ynewlo = fn(&xmin[0]);

      } else {
         method = "nelder-mead";

         nelder_mead(fn,
                     n_params,
                     &start[0],
                     &xmin[0],
                     ynewlo,
                     reqmin,
                     &step[0],
                     10,
                     10 * kcount,
                     icount,
                     numres,
                     ifault);
      }
   };

   pack(&start[0]);
   const real ystartlo = fn(&start[0]);
   auto now            = tick();
   for(auto i = 0; i < 10; ++i) {
      do_refine((i % 2) == 0);
      std::copy(cbegin(best_params), cend(best_params), begin(start));
      for(size_t i = 0; i < 3; ++i) { // perturb translation
         start[i] += 0.001 * (2.0 * uniform() - 1.0);
         step[i] = 0.01; // 1cm
      }
      for(size_t i = 3; i < n_params; ++i) { // pertube rotation
         start[i] += 0.001 * (uniform() * one_degree() - 0.5 * one_degree());
         step[i] = one_degree();
      }
   }
   const real ynewlo    = fn(&best_params[0]);
   const real elapsed_s = tock(now);

   unpack(&best_params[0]);

   const bool success = ynewlo < 20.0; // TODO, should be in degrees

   if(!success or feedback) { // Print feedback
      std::stringstream ss{""};
      ss << endl;
      ss << format("Feedback refining planes on cam {}", cam_pos_name) << endl;
      ss << format("    elapsed time:         {}s", elapsed_s) << endl;
      ss << format("    initial-score:        {}", ystartlo) << endl;
      ss << format("    final-score:          {}", ynewlo) << endl;
      ss << format("    success:              {}", str(success)) << endl;

      // And output to screen
      sync_write([&]() { cout << indent(ss.str(), 4); });
   }

   return std::make_pair(et, success);
}

// -----------------------------------------------------------------------------
// --------------------------------------------------------- refine pointwise et

// Get the initial EuclideanTransform from the detection: World => Cam
bool refine_pointwise_et(const ArucoCube& ac,
                         const vector<ArucoCube::FaceDetection>& detects,
                         const CachingUndistortInverse& cu,
                         const string_view cam_id,
                         EuclideanTransform& inout_et,
                         ARGBImage* argb_ptr,
                         const bool feedback) noexcept
{
   const auto [et, success]
       = refine_pointwise_et(ac, detects, cu, cam_id, inout_et, feedback);
   if(argb_ptr) { ac.render_pointwise(*argb_ptr, detects, cu, et, {}); }
   if(success) { inout_et = et; }
   return success;
}

// Get the initial EuclideanTransform from the detection: World => Cam
bool refine_pointwise_et(const ArucoCube& ac,
                         const vector<ArucoCube::FaceDetection>& detects,
                         const Matrix3r& K,
                         const int cam_id,
                         EuclideanTransform& inout_et,
                         ARGBImage* argb_ptr,
                         const bool feedback) noexcept
{
   return refine_pointwise_et(ac,
                              detects,
                              CachingUndistortInverse(K),
                              format("{}", cam_id),
                              inout_et,
                              argb_ptr,
                              feedback);
}

// -----------------------------------------------------------------------------
// --------------------------------------- calc aruco face coords and homography

ArucoFaceProjectionInfo
calc_aruco_face_homography(const EuclideanTransform& et,
                           const ArucoCube& ac,
                           const ArucoCube::face_e f_ind,
                           const array<cv::Mat, 6>& face_ims)
{
   return ac.calc_face_projection_info(
       et, f_ind, face_ims[0].cols, face_ims[0].rows);
}

vector<ArucoFaceProjectionInfo> calc_aruco_face_coords_and_homography(
    const EuclideanTransform& et,
    const ArucoCube& ac,
    const vector<ArucoCube::FaceDetection>& detects,
    const array<cv::Mat, 6>& face_ims)
{
   vector<ArucoFaceProjectionInfo> out(detects.size());

   auto calc_face_proj_info = [&](size_t ind) {
      const auto f_ind = detects[ind].f_ind;
      out[ind]         = calc_aruco_face_homography(et, ac, f_ind, face_ims);
   };
   for(size_t i = 0; i < detects.size(); ++i) calc_face_proj_info(i);

   return out;
}

// -----------------------------------------------------------------------------
// ---------------------------------------------------------- project aruco face

// static void project_aruco_face_worker(
//     const ArucoFaceProjectionInfo& pinfo,
//     const Matrix3r& K,
//     const Matrix3r& K_inv,
//     std::function<void(int x,
//                        int y,            // output image x, y
//                        const Vector3& X, // 3D world coord
//                        const Point2& p   // face-image coords
//                        )> g) noexcept
// {
//    if(pinfo.f_ind == ArucoCube::BOTTOM) return;

//    // Project the quad3d to the image
//    array<Vector2, 4> quad;
//    std::transform(
//        cbegin(pinfo.quad3d), cend(pinfo.quad3d), begin(quad), [&](auto& X) {
//           return homgen_P2_to_R2(to_vec3(K * to_vec3r(X)));
//        });

//    // Get the AABB for the 2d quad
//    AABB aabb = AABB::minmax();
//    for(const auto& x : quad) aabb.union_point(x);

//    // n 'integer' AABB that we can iterate across: the region of interest
//    AABBi roi;
//    roi.left   = std::floor(aabb.left);
//    roi.right  = std::ceil(aabb.right);
//    roi.top    = std::floor(aabb.top);
//    roi.bottom = std::ceil(aabb.bottom);

//    // Render every pixel in ROI
//    const Vector3 C{0.0, 0.0, 0.0};
//    for(auto y = roi.top; y <= roi.bottom; ++y) {
//       for(auto x = roi.left; x <= roi.right; ++x) {
//          Vector3 ray = to_vec3(K_inv * Vector3r(x, y, 1.0)).normalised();
//          Vector3 X   = plane_ray_intersection(pinfo.p3, C, C + ray);
//          Point2 ix   = to_pt2(homgen_P2_to_R2(to_vec3(pinfo.H *
//          to_vec3r(X)))); g(x, y, X, ix);
//       }
//    }
// }

// static void project_aruco_face_worker(
//     const ArucoFaceProjectionInfo& pinfo,
//     const Matrix3r& K,
//     const Matrix3r& K_inv,
//     std::function<void(int x, int y, int ix, int iy)> g) noexcept
// {
//    project_aruco_face_worker(
//        pinfo, K, K_inv, [&g](int x, int y, const Vector3& X, const Point2& p)
//        {
//           g(x, y, p.x, p.y);
//        });
//    return;
// }

static void project_aruco_face_worker(
    const ArucoFaceProjectionInfo& pinfo,
    const CachingUndistortInverse& cu,
    std::function<void(int x,
                       int y,            // output image x, y
                       const Vector3& X, // 3D world coord
                       const Point2& p   // face-image coords
                       )> g) noexcept
{
   if(pinfo.f_ind == ArucoCube::BOTTOM) return;

   // Project the quad3d to the image
   array<Vector2, 4> quad;
   std::transform(
       cbegin(pinfo.quad3d), cend(pinfo.quad3d), begin(quad), [&](auto& X) {
          return cu.distort(homgen_P2_to_R2(X));
          // return homgen_P2_to_R2(to_vec3(K * to_vec3r(X)));
       });

   // Get the AABB for the 2d quad
   AABB aabb = AABB::minmax();
   for(const auto& x : quad) aabb.union_point(x);

   // n 'integer' AABB that we can iterate across: the region of interest
   AABBi roi;
   roi.left   = int(std::floor(aabb.left));
   roi.right  = int(std::ceil(aabb.right));
   roi.top    = int(std::floor(aabb.top));
   roi.bottom = int(std::ceil(aabb.bottom));

   // Render every pixel in ROI
   const Vector3 C{0.0, 0.0, 0.0};
   for(auto y = roi.top; y <= roi.bottom; ++y) {
      for(auto x = roi.left; x <= roi.right; ++x) {
         Vector3 ray
             = homgen_R2_to_P2(cu.undistort(Vector2(x, y))).normalised();
         // to_vec3(K_inv * Vector3r(x, y, 1.0)).normalised();
         Vector3 X = plane_ray_intersection(pinfo.p3, C, C + ray);
         Point2 ix = to_pt2(homgen_P2_to_R2(to_vec3(pinfo.H * to_vec3r(X))));
         g(x, y, X, ix);
      }
   }
}

static void project_aruco_face_worker(
    const ArucoFaceProjectionInfo& pinfo,
    const CachingUndistortInverse& cu,
    std::function<void(int x, int y, int ix, int iy)> g) noexcept
{
   project_aruco_face_worker(
       pinfo, cu, [&g](int x, int y, const Vector3& X, const Point2& p) {
          g(x, y, p.x, p.y);
       });
}

void project_aruco_face(
    const ArucoFaceProjectionInfo& pinfo,
    const Matrix3r& K,
    const Matrix3r& K_inv,
    const LABImage& face_im,
    std::function<void(int x, int y, const LAB lab)> f) noexcept
{
   project_aruco_face(pinfo, CachingUndistortInverse(K), face_im, f);
}

void project_aruco_face(
    const ArucoFaceProjectionInfo& pinfo,
    const Matrix3r& K,
    const Matrix3r& K_inv,
    const cv::Mat& face_im,
    std::function<void(int x, int y, uint32_t k)> f) noexcept
{
   project_aruco_face(pinfo, CachingUndistortInverse(K), face_im, f);
}

void project_aruco_face(const ArucoFaceProjectionInfo& pinfo,
                        const CachingUndistortInverse& cu,
                        const LABImage& face_im,
                        std::function<void(int x, int y, LAB lab)> f) noexcept
{
   project_aruco_face_worker(pinfo, cu, [&](int x, int y, int ix, int iy) {
      if(in_bounds(face_im, ix, iy)) f(x, y, face_im(ix, iy));
   });
}

// -----------------------------------------------------------------------------
// ----------------------------------------------------------- render aruco cube

void render_aruco_cube(ARGBImage& argb,
                       const vector<ArucoFaceProjectionInfo>& pinfos,
                       const Matrix3r& K,
                       const Matrix3r& K_inv,
                       const vector<ArucoCube::FaceDetection>& detects,
                       const array<cv::Mat, 6>& face_ims) noexcept
{
   auto project = [&](const Vector3& X) -> Point2 {
      return to_pt2(homgen_P2_to_R2(to_vec3(K * to_vec3r(X))));
   };

   auto draw_side = [&](const Vector3& A, const Vector3& B, uint32_t kolour) {
      bresenham(project(A), project(B), [&](int x, int y) {
         if(argb.in_bounds(x, y)) argb(x, y) = kolour;
      });
   };

   // Render the face image onto the drawing
   for(auto k = 0u; k < pinfos.size(); ++k) {
      const auto& pinfo = pinfos[k];
      project_aruco_face(pinfos[k],
                         K,
                         K_inv,
                         face_ims[size_t(detects[k].f_ind)],
                         [&](int x, int y, uint32_t k) {
                            if(argb.in_bounds(x, y))
                               argb(x, y) = blend(argb(x, y), k, 0.5);
                         });
   }

   // Draw a frame around each image
   for(auto k = 0u; k < pinfos.size(); ++k) {
      const auto& pinfo = pinfos[k];
      const auto& A     = pinfo.quad3d;
      const Point2 off  = Point2(-1, -3);
      for(auto i = 0u; i < A.size(); ++i) {
         draw_side(A[i], A[(i + 1) % A.size()], pinfo.kolour);
         render_string(argb,
                       format("{:c}", 'A' + i),
                       project(A[i]) + off,
                       k_yellow,
                       k_black);
      }
   }
}

// -----------------------------------------------------------------------------
// ------------------------------------------------------------- refine-et-dense

void refine_et_dense(const ArucoCube& ac,
                     const vector<ArucoCube::FaceDetection>& detects,
                     const array<cv::Mat, 6>& face_ims,
                     const Matrix3r& K,
                     const int cam_id,
                     EuclideanTransform& inout_et,
                     ARGBImage& argb,
                     const bool feedback) noexcept
{
   Expects(detects.size() > 1);

   EuclideanTransform& et       = inout_et;
   const EuclideanTransform et0 = inout_et;
   const Matrix3r K_inv         = K.inverse();
   const Vector3 O{0.0, 0.0, 0.0};
   const bool super_feedback = false;
   const int n_params        = 6;

   array<LABImage, 6> face_labs;
   std::transform(cbegin(face_ims),
                  cend(face_ims),
                  begin(face_labs),
                  [&](const auto& im) { return cv_to_LAB_im(im); });

   LABImage lab_im = argb_to_LAB_im(argb);

   // World coords for 3d quads for detected markers
   const vector<vector<array<Vector3, 4>>> detect_Ws
       = calc_detect_Ws(ac, detects);

   auto pack = [&](real* X) {
      Vector3 saa = quaternion_to_saa(et.rotation);
      for(auto i = 0; i < 3; ++i) *X++ = saa[i];
      for(auto i = 0; i < 3; ++i) *X++ = et.translation[i];
   };

   auto unpack = [&](const real* X) {
      Vector3 saa;
      for(auto i = 0; i < 3; ++i) saa[i] = *X++;
      for(auto i = 0; i < 3; ++i) et.translation[i] = *X++;
      et.rotation = saa_to_quaternion(saa);
      et.scale    = 1.0;
   };

   auto score_quad = [&](const auto& pinfo,
                         const ArucoCube::FaceDetection& detect) {
      auto err     = 0.0;
      auto counter = 0;

      const auto& face_lab = face_labs[size_t(detect.f_ind)];

      project_aruco_face(
          pinfo, K, K_inv, face_lab, [&](int x, int y, const LAB k) {
             if(!lab_im.in_bounds(x, y)) {
                err += 10.0;
             } else {
                const auto s = cie1976_normalized_distance(k, lab_im(x, y));
                Expects(s >= 0.0f and s <= 1.0f);
                err += real(s);
             }
             counter++;
          });

      const auto proj_err = (counter == 0) ? 10000.0 : (err / real(counter));

      const auto reg_term = calc_regularization_term(ac, detect, et0, pinfo, K);

      return proj_err + reg_term;
   };

   int counter     = 0;
   real best_score = std::numeric_limits<real>::max();
   vector<real> best_params(n_params);
   auto fn = [&](const real* X) -> real {
      unpack(X);
      const auto pinfos
          = calc_aruco_face_coords_and_homography(et, ac, detects, face_ims);

      auto err = 0.0;

      for(auto i = 0u; i < detects.size(); ++i)
         err += score_quad(pinfos[i], detects[i]);

      auto ret = err / real(detects.size());
      if(ret < best_score) {
         best_score = ret;
         std::copy(X, X + n_params, begin(best_params));
         if(super_feedback) {
            cout << format("   #{:8d} :: {:10.7f} :: {{}}",
                           counter,
                           best_score,
                           implode(X, X + n_params, ", "))
                 << endl;
         }
      }
      counter++;

      return ret;
   };

   vector<real> start(n_params);
   auto do_refine = [&](bool use_nelder_mead) {
      vector<real> xmin(n_params);
      real ynewlo   = dNAN;
      real ystartlo = dNAN;
      real reqmin   = 1e-7;
      real diffstep = 0.1;
      int kcount    = 10000000; // max interations
      int icount = 0, numres = 0, ifault = 0;
      const char* method = nullptr;

      if(!use_nelder_mead) {
         method = "levenberg-marquardt";
         levenberg_marquardt(fn,
                             n_params,
                             &start[0],
                             &xmin[0],
                             reqmin,
                             diffstep,
                             10,
                             kcount,
                             icount,
                             ifault);
         ynewlo = fn(&xmin[0]);

      } else {
         method = "nelder-mead";

         vector<real> step(n_params);
         auto X = &step[0];
         for(auto i = 0; i < 3; ++i) *X++ = one_degree(); //
         for(auto i = 0; i < 3; ++i) *X++ = 0.01;         // 1cm

         nelder_mead(fn,
                     n_params,
                     &start[0],
                     &xmin[0],
                     ynewlo,
                     reqmin,
                     &step[0],
                     10,
                     10 * kcount,
                     icount,
                     numres,
                     ifault);
      }
   };

   pack(&start[0]);
   const real ystartlo = fn(&start[0]);
   auto now            = tick();
   for(auto i = 0; i < 3; ++i) {
      do_refine((i % 2) == 0);
      std::copy(cbegin(best_params), cend(best_params), begin(start));
      for(size_t i = 0; i < 3; ++i) // pertube rotation
         start[i] += 0.1 * (uniform() * one_degree() - 0.5 * one_degree());
      for(size_t i = 3; i < 6; ++i) // perturb translation
         start[i] += 0.001 * (2.0 * uniform() - 1.0);
   }
   const real ynewlo    = fn(&best_params[0]);
   const real elapsed_s = tock(now);

   unpack(&best_params[0]);

   if(feedback) { // Print feedback
      std::stringstream ss{""};
      ss << endl;
      ss << format("Feedback refining position of cam {}", cam_id) << endl;
      ss << format("    elapsed time:         {}s", elapsed_s) << endl;
      ss << format("    initial-score:        {}", ystartlo) << endl;
      ss << format("    final-score:          {}", ynewlo) << endl;
      ss << et.inverse().to_string();
      // ss << format("    t, q = {}, {}", str(et.translation),
      // str(et.rotation))
      //    << endl;
      ss << endl;

      // And output to screen
      sync_write([&]() { cout << indent(ss.str(), 4); });
   }

   {
      ARGBImage& out = argb;

      { // Draw each visible face
         const auto pinfos
             = calc_aruco_face_coords_and_homography(et, ac, detects, face_ims);

         for(auto i = 0u; i < detects.size(); ++i) {
            const auto& detect  = detects[i];
            const auto& M       = ac.measures[size_t(detect.f_ind)];
            const auto& pinfo   = pinfos[i];
            const auto& face_im = face_ims[size_t(detect.f_ind)];

            project_aruco_face(
                pinfo, K, K_inv, face_im, [&](int x, int y, uint32_t k) {
                   if(out.in_bounds(x, y)) out(x, y) = blend(k, out(x, y), 0.5);
                });
         }
      }
   }

   inout_et = et;
}

// -----------------------------------------------------------------------------
// -------------------------------------------------------------- refine bcam qt
// Use dense methods to estimate 'q' and 't' for bcam
real refine_bcam_qt(const PhasePositionOptDetails& opt,
                    const bool do_refine_bcam, // as opposed to et0
                    const bool in_opt_K,
                    BinocularCameraInfo& inout_bcam_info,
                    EuclideanTransform& out_et0,
                    const unsigned et_result_index,
                    const string& outdir,
                    const bool feedback) noexcept
{
   const size_t N = opt.imgs.size();
   Expects(N > 0);
   Expects(et_result_index < N);
   // if(!do_refine_bcam) Expects(N == 1);

   const auto ac   = opt.ac;
   Matrix3r K0     = opt.K;
   Matrix3r K0_inv = opt.K.inverse();
   Matrix3r K1     = opt.K;
   Matrix3r K1_inv = opt.K.inverse();
   Matrix3r K0a    = Matrix3r::Identity();
   Matrix3r K1a    = Matrix3r::Identity();

   // const Matrix3r K     = opt.K;
   // const Matrix3r K_inv = K.inverse();

   const bool opt_K             = in_opt_K and do_refine_bcam;
   const size_t n_params        = opt_K ? (6 + 6 + N * 6) : 6;
   const bool super_feedback    = true;
   BinocularCameraInfo bcam_opt = inout_bcam_info;

   EuclideanTransform et0          = opt.imgs[et_result_index].ets[0];
   EuclideanTransform et1          = opt.imgs[et_result_index].ets[1];
   const EuclideanTransform in_e01 = et0.inverse() * et1;
   EuclideanTransform e01          = in_e01;
   e01.translation = opt.p.baseline_estimate * e01.translation.normalised();

   CachingUndistortInverse cu0, cu1;
   cu0.init(K0 * K0a);
   cu1.init(K1 * K1a);

   vector<array<ARGBImage, 2>> argbs(N);
   for(size_t k = 0u; k < N; ++k)
      for(size_t i = 0; i < 2; ++i)
         argbs[k][i] = cv_to_argb(opt.imgs[k].undistorted[i]);

   vector<array<LABImage, 2>> labs(N);
   for(size_t k = 0u; k < N; ++k)
      for(size_t i = 0; i < 2; ++i) labs[k][i] = argb_to_LAB_im(argbs[k][i]);

   vector<LABImage> lab_face_ims(opt.face_ims.size());
   std::transform(cbegin(opt.face_ims),
                  cend(opt.face_ims),
                  begin(lab_face_ims),
                  [](const auto& cvim) { return cv_to_LAB_im(cvim); });

   vector<EuclideanTransform> et0s(N);
   for(size_t k = 0; k < N; ++k) et0s[k] = opt.imgs[k].ets[0];

   // To regularize the optimization, we need to find the initial 2d coords
   // for each aruco-cube detection

   auto clamped_baseline = [&](real in) -> real {
      const real delta = 0.010; // +/- 1.0cm
      const auto est   = opt.p.baseline_estimate;
      return std::clamp(in, est - delta, est + delta);
   };

   auto transfer_pixels =
       [&](const ARGBImage& in_argb,
           const auto& detects,
           const EuclideanTransform& et_cam,
           const EuclideanTransform& et_alt,
           const auto& cu_cam,
           const auto& cu_alt,
           std::function<void(const Vector2& a, const Vector2& b)> callback) {
          //
          array<Vector2, 4> xs;

          for(size_t i = 0; i < detects.size(); ++i) {
             const auto f_ind = detects[i].f_ind; // just need the face index
             const auto& spec = ac.measures[size_t(detects[i].f_ind)];
             const auto p3    = et_cam.apply_to_plane(spec.p3);

             // Project the cube face to the image
             for(size_t j = 0; j < 4; ++j)
                xs[j]
                    = cu_cam.distort(homgen_P2_to_R2(et_cam.apply(spec.Xs[j])));

             // Find the bounding box
             AABB aabb = AABB::minmax();
             for(size_t j = 0; j < 4; ++j) aabb.union_point(xs[j]);

             AABBi bounds;
             bounds.left   = int(std::ceil(aabb.left)); // Round inwards
             bounds.top    = int(std::ceil(aabb.top));
             bounds.right  = int(std::floor(aabb.right));
             bounds.bottom = int(std::floor(aabb.bottom));

             // Iterate over each pixel in the bounding box
             for(auto y = bounds.top; y <= bounds.bottom; ++y) {
                for(auto x = bounds.left; x <= bounds.right; ++x) {
                   const auto a = Vector2(x, y);
                   if(!point_in_polygon(a, cbegin(xs), cend(xs))) continue;

                   // Transfer the point to the other image
                   const auto b = transfer_point_between_images(
                       a, p3, et_cam, et_alt, cu_cam, cu_alt);

                   callback(a, b);
                }
             }
          }
       };

   auto render_argb = [&](const ARGBImage& argb_cam,
                          const ARGBImage& argb_alt,
                          const auto& detects,
                          const EuclideanTransform& et_cam,
                          const EuclideanTransform& et_alt,
                          const auto& cu_cam,
                          const auto& cu_alt) {
      ARGBImage out = argb_cam;
      transfer_pixels(out,
                      detects,
                      et_cam,
                      et_alt,
                      cu_cam,
                      cu_alt,
                      [&](const Vector2& a, const Vector2& b) {
                         if(!out.in_bounds(a)) return;
                         if(!out.in_bounds(b)) return;
                         const auto s = cie2000_score(argb_cam(a), argb_alt(b));
                         out(a)       = hsv_to_rgb_uint32(0.0, 0.0, 1.0 - s);
                      });
      return out;
   };

   auto pack = [&](real* X) {
      auto X0 = X;
      if(do_refine_bcam) {
         const auto saa = quaternion_to_saa(e01.rotation);
         const auto taa = cartesian_to_spherical(e01.translation);
         Expects(taa.is_finite());
         Expects(std::fabs(taa.z) > 1e-6);
         for(auto i = 0; i < 3; ++i) *X++ = saa[i];
         for(auto i = 0; i < 3; ++i) *X++ = taa[i];
         if(opt_K and n_params > 6) {
            *X++ = K0a(0, 0);
            *X++ = K0a(0, 2);
            *X++ = K0a(1, 2);
            *X++ = K1a(0, 0);
            *X++ = K1a(0, 2);
            *X++ = K1a(1, 2);
            for(auto k = 0u; k < N; ++k) {
               const auto saa = quaternion_to_saa(et0s[k].rotation);
               const auto taa = cartesian_to_spherical(et0s[k].translation);
               for(auto i = 0; i < 3; ++i) *X++ = saa[i];
               for(auto i = 0; i < 3; ++i) *X++ = taa[i];
            }
         }
      } else {
         const auto saa = quaternion_to_saa(et0.rotation);
         const auto taa = cartesian_to_spherical(et0.translation);
         for(auto i = 0; i < 3; ++i) *X++ = saa[i];
         for(auto i = 0; i < 3; ++i) *X++ = taa[i];
      }
      for(size_t i = 0; i < n_params; ++i) Expects(std::isfinite(X0[i]));
      Expects(X0[5] > 1e-6);
      Expects(size_t(X - X0) == n_params);
   };

   auto unpack = [&](const real* X) {
      for(size_t i = 0; i < n_params; ++i) Expects(std::isfinite(X[i]));
      Vector3 saa, taa;
      if(do_refine_bcam) {
         for(auto i = 0; i < 3; ++i) saa[i] = *X++;
         for(auto i = 0; i < 3; ++i) taa[i] = *X++;
         taa.z           = clamped_baseline(taa.z);
         e01.rotation    = saa_to_quaternion(saa);
         e01.translation = spherical_to_cartesian(taa);
         bcam_opt.set_from_et(e01.inverse());
         if(opt_K and n_params > 6) {
            K0a(0, 0) = K0a(1, 1) = *X++;
            K0a(0, 2)             = *X++;
            K0a(1, 2)             = *X++;
            K1a(0, 0) = K1a(1, 1) = *X++;
            K1a(0, 2)             = *X++;
            K1a(1, 2)             = *X++;
            K0                    = opt.K * K0a;
            K0_inv                = K0.inverse();
            K1                    = opt.K * K1a;
            K1_inv                = K1.inverse();
            cu0.init(K0);
            cu1.init(K1);
            for(auto k = 0u; k < N; ++k) {
               for(auto i = 0; i < 3; ++i) saa[i] = *X++;
               for(auto i = 0; i < 3; ++i) taa[i] = *X++;
               et0s[k].rotation    = saa_to_quaternion(saa);
               et0s[k].translation = spherical_to_cartesian(taa);
            }
         }
      } else {
         for(auto i = 0; i < 3; ++i) saa[i] = *X++;
         for(auto i = 0; i < 3; ++i) taa[i] = *X++;
         Expects(saa.is_finite());
         Expects(taa.is_finite());
         et0.rotation    = saa_to_quaternion(saa);
         et0.translation = spherical_to_cartesian(taa);
         et1             = bcam_opt.make_et1(et0.inverse()).inverse();
         Expects(et0.translation.is_finite());
         Expects(et1.translation.is_finite());
      }
   };

   auto score_quad = [&](const LABImage& lab,
                         const auto& pinfo,
                         const Matrix3r& K,
                         const Matrix3r& K_inv,
                         const LABImage& face_lab) {
      auto err     = 0.0;
      auto counter = 0;
      project_aruco_face(
          pinfo, K, K_inv, face_lab, [&](int x, int y, const LAB k) {
             if(!lab.in_bounds(x, y)) {
                err += 10.0;
             } else {
                const auto s = cie1976_normalized_distance(k, lab(x, y));
                Expects(s >= 0.0f and s <= 1.0f);
                err += real(s);
             }
             counter++;
          });

      return (counter == 0) ? 10000.0 : (err / real(counter));
   };

   int counter     = 0;
   auto fn_timer   = tick();
   real best_score = std::numeric_limits<real>::max();
   vector<real> best_params((size_t(n_params)));
   ParallelJobSet pjobs;
   vector<real> rets(N);

   auto print_update = [&]() {
      cout << format("    #{:8d} :: {:10.7f} :: ", counter, best_score);
      if(do_refine_bcam) {
         cout << format("|{}| = {}, rot = {}",
                        str(bcam_opt.C1()),
                        bcam_opt.baseline,
                        bcam_opt.q.to_readable_str());
      } else {
         cout << str(et0.inverse());
      }
      cout << format(" :: {}s", tock(fn_timer)) << endl;
   };

   auto fn = [&](const real* X) -> real {
      unpack(X);

      auto process_k = [&](unsigned k) {
         const auto& im0 = argbs[k][0];
         const auto& im1 = argbs[k][1];

         const auto& lab0 = labs[k][0];
         const auto& lab1 = labs[k][1];

         auto err           = 0.0;
         auto pixel_counter = 0;

         const auto& detects0 = opt.imgs[k].detects[0];
         const auto& detects1 = opt.imgs[k].detects[1];
         const auto e0        = do_refine_bcam ? et0s[k] : et0;
         const auto e1        = bcam_opt.make_et1(e0.inverse()).inverse();

         auto f0 = [&](const Vector2& a, const Vector2& b) {
            if(!lab0.in_bounds(a) || !lab1.in_bounds(b)) {
               err += 100.0;
            } else {
               err += real(cie1976_normalized_distance(lab0(a), lab1(b)));
            }
            ++pixel_counter;
         };

         auto f1 = [&](const Vector2& a, const Vector2& b) {
            if(!lab1.in_bounds(a) || !lab0.in_bounds(b)) {
               err += 100.0;
            } else {
               err += real(cie1976_normalized_distance(lab1(a), lab0(b)));
            }
            ++pixel_counter;
         };

         auto transfer_err = [&]() {
            err = 0.0;
            transfer_pixels(im0, detects0, e0, e1, cu0, cu1, f0);
            transfer_pixels(im1, detects1, e1, e0, cu1, cu0, f1);
            return err / real(pixel_counter);
         };

         auto e_err = [&](const auto& e,
                          const auto& detects,
                          const auto& lab,
                          const int ind) {
            auto err          = 0.0;
            const auto pinfos = calc_aruco_face_coords_and_homography(
                e, ac, detects, opt.face_ims);

            for(size_t i = 0; i < detects.size(); ++i) {
               const auto proj_err
                   = score_quad(lab,
                                pinfos[i],
                                K0,
                                K0_inv,
                                lab_face_ims[size_t(detects[i].f_ind)]);

               const auto reg_term
                   = calc_regularization_term(ac,
                                              detects[i],
                                              opt.imgs[k].et0s[size_t(ind)],
                                              pinfos[i],
                                              K0);

               err += (proj_err + reg_term);
            }

            return err / real(detects.size());
         };

         const auto e0_err = e_err(e0, detects0, lab0, 0);
         const auto e1_err = e_err(e1, detects1, lab1, 1);

         rets[k] = 0.5 * (transfer_err() + 0.5 * (e0_err + e1_err));
      };

      const unsigned k0 = do_refine_bcam ? 0u : et_result_index;
      const unsigned k1 = do_refine_bcam ? unsigned(N) : k0 + 1;

      unsigned job_counter = 0u;
      for(auto k = k0; k < k1; ++k) {
         pjobs.schedule([process_k, k]() { process_k(k); });
         job_counter++;
      }
      pjobs.execute();

      auto ret = 0.0;
      for(auto k = k0; k < k1; ++k) ret += rets[k];
      ret /= real(job_counter);

      if(ret < best_score) {
         best_score = ret;
         std::copy(X, X + n_params, begin(best_params));
         if(super_feedback) { print_update(); }
      }

      counter++;

      return ret;
   };

   vector<real> start(n_params);
   auto do_refine = [&](bool use_nelder_mead) {
      vector<real> xmin(n_params);
      real ynewlo   = dNAN;
      real ystartlo = dNAN;
      real reqmin   = 1e-3;
      real diffstep = 0.01;
      int kcount    = 100; // max interations
      int icount = 0, numres = 0, ifault = 0;
      const char* method = nullptr;

      // do_refine_bcam ? (n_params - 1) : n_params;
      const auto in_n_params = unsigned(n_params);

      if(!use_nelder_mead) {
         method = "levenberg-marquardt";
         levenberg_marquardt(fn,
                             in_n_params,
                             &start[0],
                             &xmin[0],
                             reqmin,
                             diffstep,
                             3,
                             kcount,
                             icount,
                             ifault);
         ynewlo = fn(&xmin[0]);

      } else {
         method = "nelder-mead";

         vector<real> step(n_params);
         auto X = &step[0];
         for(auto i = 0; i < 5; ++i) *X++ = 0.5 * one_degree(); //
         for(auto i = 5; i < 6; ++i) *X++ = 0.01;               // 1cm
         if(opt_K and n_params > 6) {
            for(size_t i = 6; i < n_params; ++i) *X++ = 0.5 * one_degree();
         }

         nelder_mead(fn,
                     in_n_params,
                     &start[0],
                     &xmin[0],
                     ynewlo,
                     reqmin,
                     &step[0],
                     10,
                     10 * kcount,
                     icount,
                     numres,
                     ifault);
      }
   };

   pack(&start[0]);
   const real ystartlo = fn(&start[0]);

   if(do_refine_bcam) {
      for(auto k = 0u; k < N; ++k) {
         const auto& detects0 = opt.imgs[k].detects[0];
         const auto& detects1 = opt.imgs[k].detects[1];
         const auto& im0      = argbs[k][0];
         const auto& im1      = argbs[k][1];
         et0                  = opt.imgs[k].ets[0];
         et1                  = bcam_opt.make_et1(et0.inverse()).inverse();
         render_argb(im0, im1, detects0, et0, et1, cu0, cu1)
             .save(format("{}/www_qt_{}-cam{}_INITIAL.png", outdir, k, 0));
         render_argb(im1, im0, detects1, et1, et0, cu1, cu0)
             .save(format("{}/www_qt_{}-cam{}_INITIAL.png", outdir, k, 1));
      }
   } else {
      const auto k         = et_result_index;
      const auto& detects0 = opt.imgs[k].detects[0];
      const auto& detects1 = opt.imgs[k].detects[1];
      const auto& im0      = argbs[k][0];
      const auto& im1      = argbs[k][1];
      render_argb(im0, im1, detects0, et0, et1, cu0, cu1)
          .save(format("{}/yyy_et0_{}-cam{}_INITIAL.png", outdir, k, 0));
      render_argb(im1, im0, detects1, et1, et0, cu1, cu0)
          .save(format("{}/yyy_et0_{}-cam{}_INITIAL.png", outdir, k, 1));
   }

   auto now = tick();
   for(auto j = 0; j < (do_refine_bcam ? 10 : 1); ++j) { //
      const bool use_nelder_mead = (j % 2) == 0;         // TODO was (true)
      const auto method_s
          = (use_nelder_mead ? "nelder-mead" : "levenberg-marquardt");
      cout << format("    --------- [{}] {}", j, method_s) << endl;
      if(j > 0) {
         fn(&start[0]);
         print_update();
      }
      do_refine(use_nelder_mead);
      std::copy(cbegin(best_params), cend(best_params), begin(start));
      for(size_t i = 0; i < 5; ++i) // pertube rotation
         start[i] += 0.5 * (uniform() * 2.0 - 1.0) * one_degree();
      for(size_t i = 5; i < 6; ++i) // perturb translation
         start[i] += 0.005 * (2.0 * uniform() - 1.0);
      if(opt_K and n_params > 6) {
         for(size_t i = 6; i < n_params; ++i)
            start[i] = 0.5 * (uniform() * 2.0 - 1.0) * one_degree();
      }
      if(false && do_refine_bcam) { // adjust baseline again
         unpack(&start[0]);
         e01.translation
             = opt.p.baseline_estimate * e01.translation.normalised();
         pack(&start[0]);
      }
   }
   const real ynewlo    = fn(&best_params[0]);
   const real elapsed_s = tock(now);

   unpack(&best_params[0]);

   if(do_refine_bcam) {
      const int sw = 640; // (640*2)x(480*N)
      const int sh = 480;
      cv::Mat composite
          = cv::Mat{sh * int(N), sw * 2, CV_8UC3, cv::Scalar(255, 255, 255)};
      Expects(sw * 2 == composite.cols);
      Expects(sh * int(N) == composite.rows);

      auto calc_cv_rect
          = [&](int n, int j) { return cv::Rect(j * sw, n * sh, sw, sh); };

      for(auto k = 0u; k < N; ++k) {
         const auto& detects0 = opt.imgs[k].detects[0];
         const auto& detects1 = opt.imgs[k].detects[1];
         const auto& im0      = argbs[k][0];
         const auto& im1      = argbs[k][1];
         et0                  = opt.imgs[k].ets[0];
         et1                  = bcam_opt.make_et1(et0.inverse()).inverse();
         const auto o0 = render_argb(im0, im1, detects0, et0, et1, cu0, cu1);
         const auto o1 = render_argb(im1, im0, detects1, et1, et0, cu1, cu0);
         o0.save(format("{}/xxx_qt_{}-cam{}_FINAL.png", outdir, k, 0));
         o1.save(format("{}/xxx_qt_{}-cam{}_FINAL.png", outdir, k, 1));

         cv::Mat a0 = argb_to_cv(o0);
         cv::Mat a1 = argb_to_cv(o1);
         cv::Mat im_small;
         cv::resize(a0, im_small, cv::Size(sw, sh));
         im_small.copyTo(composite(calc_cv_rect(int(k), 0)));
         cv::resize(a1, im_small, cv::Size(sw, sh));
         im_small.copyTo(composite(calc_cv_rect(int(k), 1)));
      }

      cv::imwrite(format("{}/zzz_composite_FINAL.png", outdir), composite);

   } else {
      const auto k         = et_result_index;
      const auto& detects0 = opt.imgs[k].detects[0];
      const auto& detects1 = opt.imgs[k].detects[1];
      const auto& im0      = argbs[k][0];
      const auto& im1      = argbs[k][1];
      render_argb(im0, im1, detects0, et0, et1, cu0, cu1)
          .save(format("{}/zzz_et0_{}-cam{}_FINAL.png", outdir, k, 0));
      render_argb(im1, im0, detects1, et1, et0, cu1, cu0)
          .save(format("{}/zzz_et0_{}-cam{}_FINAL.png", outdir, k, 1));
   }

   if(feedback) { // Print feedback
      std::stringstream ss{""};
      ss << endl;
      if(do_refine_bcam) {
         ss << format("Feedback refining stereo parameters") << endl;
      } else {
         ss << format("Feedback refining et0 (binocular)") << endl;
      }
      ss << format("    elapsed time:         {}s", elapsed_s) << endl;
      ss << format("    initial-score:        {}", ystartlo) << endl;
      ss << format("    final-score:          {}", ynewlo) << endl;
      ss << "---" << endl;
      if(do_refine_bcam) {
         ss << format("baseline  = {}", str(bcam_opt.baseline)) << endl;
         ss << format("q         = {}", str(bcam_opt.q.to_readable_str()))
            << endl;
         ss << format("t         = {}", str(bcam_opt.t)) << endl;
         ss << format("C1        = {}", str(bcam_opt.C1())) << endl;
      } else {
         ss << format("et0^-1    = {}", et0.inverse().to_string()) << endl;
         ss << format("et1^-1    = {}", et1.inverse().to_string()) << endl;
      }

      // And output to screen
      sync_write([&]() { cout << indent(ss.str(), 4); });
   }

   out_et0                  = et0;
   inout_bcam_info.q        = bcam_opt.q;
   inout_bcam_info.t        = bcam_opt.t;
   inout_bcam_info.baseline = bcam_opt.baseline;

   return ynewlo;
}

ARGBImage render_aruco_cube_helicopter(const Vector3& cam_pos,
                                       const unsigned w,
                                       const unsigned h,
                                       const real hfov,
                                       const ArucoCube& ac,
                                       const array<cv::Mat, 6>& face_ims,
                                       const real axes_len) noexcept
{
   constexpr int k_n_faces = ArucoCube::k_n_faces;

   const Vector3 at     = ac.center();
   const Quaternion rot = look_at(cam_pos, at, Vector3(0, 0, -1));
   const auto et        = EuclideanTransform{cam_pos, rot}.inverse();

   auto calc_K = [&]() {
      Matrix3r K   = Matrix3r::Identity();
      const auto f = 0.5 * h / tan(0.5 * hfov);
      K(0, 0)      = f;
      K(1, 1)      = f;
      K(0, 2)      = 0.5 * w;
      K(1, 2)      = 0.5 * h;
      return K;
   };
   const Matrix3r K     = calc_K();
   const Matrix3r K_inv = K.inverse();

   ARGBImage argb;
   argb.resize(w, h);
   argb.fill(k_light_slate_gray);

   // 1 -- 2
   // |    |
   // 0 -- 3
   // inline bool quad_ray_intersection(const array<Vector3, 4> X,
   //                                   const Vector3& u, // ray point
   //                                   const Vector3& v) // ray point

   // Get he planes for each quad
   auto point_occluded = [&](int ind, const Vector3& X) -> bool {
      const real X_dist = (X - cam_pos).norm();

      // We need to find all intersecting quads
      for(auto i = 0; i < k_n_faces; ++i) {
         if(i == int(ArucoCube::BOTTOM)) continue;
         if(i == ind) continue;

         const auto& p3 = ac.measures[size_t(i)].p3;
         Expects(p3.is_finite());

         if(quad_ray_intersection(ac.measures[size_t(i)].Xs, cam_pos, X)) {
            const auto Y    = plane_ray_intersection(p3, cam_pos, X);
            const auto dist = (Y - cam_pos).norm();

            if(false) {
               auto feedback = (ind == unsigned(ArucoCube::BACK));
               if(feedback) {
                  cout << format("C = {}, X = {}, |X-C| = {}, Y = {}, "
                                 "|Y-C| = {}, {}",
                                 str(cam_pos),
                                 str(X),
                                 X_dist,
                                 str(Y),
                                 dist,
                                 str(bool(dist < X_dist)))
                       << endl;
               }
            }

            if(dist < X_dist) return true;
         }
      }
      return false;
   };

   { // Draw the XYZ axes
      auto project = [&](const Vector3& W) {
         const auto X_cam = et.apply(W);
         const auto x     = homgen_P2_to_R2(to_vec3(K * to_vec3r(X_cam)));
         return x;
      };

      auto draw_it = [&](const Vector3& A, const Vector3& B, uint32_t k) {
         plot_line_AA(project(A), project(B), [&](int x, int y, float a) {
            set(argb, x, y, k);
         });
      };

      if(axes_len > 0.0) {
         draw_it(Vector3(0, 0, 0), Vector3(axes_len, 0, 0), k_red);
         draw_it(Vector3(0, 0, 0), Vector3(0, axes_len, 0), k_green);
         draw_it(Vector3(0, 0, 0), Vector3(0, 0, axes_len), k_blue);
      }
   }

   // Draw the faces
   for(auto i = 0; i < k_n_faces; ++i) {
      const auto f_ind = ArucoCube::face_e(i);
      if(f_ind == ArucoCube::BOTTOM) continue;
      const auto pinfo = calc_aruco_face_homography(et, ac, f_ind, face_ims);

      const auto C   = cam_pos;
      const auto& im = face_ims[size_t(i)];
      project_aruco_face_worker(
          pinfo,
          CachingUndistortInverse(K),
          [&](int x, int y, const Vector3& cam_X, const Point2& p) {
             if(!in_bounds(im, p)) return;
             if(!argb.in_bounds(x, y)) return;
             const auto W = et.inverse_apply(cam_X);
             if(!point_occluded(i, W))
                argb(x, y) = vec3b_to_rgb(im.at<cv::Vec3b>(p.y, p.x));
          });
   }
   return argb;
}

} // namespace perceive::calibration
