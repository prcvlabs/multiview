
#include <algorithm>
#include <iterator>

#define CATCH_CONFIG_PREFIX_ALL

#include "perceive/calibration/plane-set/cps-opts.hpp"
#include "perceive/contrib/catch.hpp"
#include "perceive/geometry/fitting-planes.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/geometry/projective/distorted-camera.hpp"
#include "perceive/geometry/rotation.hpp"
#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/optimization/golden-section-search.hpp"
#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/string-utils.hpp"

static const bool is_testing = false;

namespace perceive
{
using namespace perceive::calibration;

static real k_regularize(const Vector3& C, const Vector3& C0) noexcept
{
   return 0.05 * ((C - C0).norm() + 30 * std::fabs(C.z - C0.z));
}

// static Quaternion look_at(const Vector3& C, const Vecto

static std::pair<Quaternion, Quaternion>
make_q01(const BinocularCameraInfo& bcam_info,
         const Vector3& C, // camera center
         const Vector2& x,
         const Vector2& y,
         const Vector3& X,
         const Vector3& N) noexcept
{
   const auto ray_x = homgen_R2_to_P2(bcam_info.M[0].undistort(x)).normalised();
   const auto ray_y = homgen_R2_to_P2(bcam_info.M[0].undistort(y)).normalised();
   const auto [q0, q1] = calc_rotation(C, ray_x, ray_y, X, N);
   return std::make_pair(q0.inverse(), q1.inverse());
}

struct OptConstraint
{
   virtual ~OptConstraint() {}

   virtual real
   evaluate(const BinocularCameraInfo& bcam_info,
            const EuclideanTransform& et0,
            const array<CachingUndistortInverse, 2> cus) const noexcept = 0;

   virtual string
   to_string(const BinocularCameraInfo& bcam_info,
             const EuclideanTransform& et0,
             const array<CachingUndistortInverse, 2> cus) const noexcept = 0;
};

struct LineConstraint : public OptConstraint
{
   Plane U, V; // A 3d line is the intersection of these two planes
   Vector2 u0; // Left and right camera

   LineConstraint(const Plane& U_, const Plane& V_, const Vector2& u0_)
       : U(U_)
       , V(V_)
       , u0(u0_)
   {}
   virtual ~LineConstraint() {}

   real
   evaluate(const BinocularCameraInfo& bcam_info,
            const EuclideanTransform& et0,
            const array<CachingUndistortInverse, 2> cus) const noexcept override
   {
      const auto [C, q] = make_camera_extrinsics(et0);
      const Vector3 X   = plane_ray_intersect(C, q, cus[0], U, u0);
      const Vector3 Y   = V.image(X);
      const Vector2 v0  = project_to_distorted(C, q, cus[0], Y);
      return (u0 - v0).norm();
   }

   string to_string(
       const BinocularCameraInfo& bcam_info,
       const EuclideanTransform& et0,
       const array<CachingUndistortInverse, 2> cus) const noexcept override
   {
      const auto [C, q] = make_camera_extrinsics(et0);
      const Vector3 X   = plane_ray_intersect(C, q, cus[0], U, u0);
      const Vector3 Y   = V.image(X);
      const Vector2 v0  = project_to_distorted(C, q, cus[0], Y);
      return format("    [X = {:s}] [Y = {:s}], |{:s} - {:s}| = {}",
                    str(X),
                    str(Y),
                    str(u0),
                    str(v0),
                    (u0 - v0).norm());
   }
};

struct BinoConstraint : public OptConstraint
{
   Vector3 U;    // A 3d line is the intersection of these two planes
   Vector2 u[2]; // Left and right camera

   BinoConstraint(const Vector3& U_, const Vector2& u0, const Vector2& u1)
       : U(U_)
   {
      u[0] = u0;
      u[1] = u1;
   }
   virtual ~BinoConstraint() {}

   Vector2 pp(const EuclideanTransform& et,
              const CachingUndistortInverse& cu) const noexcept
   {
      return cu.distort(homgen_P2_to_R2(et.inverse_apply(U).normalised()));
   };

   real
   evaluate(const BinocularCameraInfo& bcam_info,
            const EuclideanTransform& et0,
            const array<CachingUndistortInverse, 2> cus) const noexcept override
   {
      const auto et1 = bcam_info.make_et1(et0);
      Vector2 v[2];
      v[0]            = pp(et0, cus[0]);
      v[1]            = pp(et1, cus[1]);
      const auto err0 = (v[0] - u[0]).norm();
      const auto err1 = (v[1] - u[1]).norm();
      return 0.5 * (err0 + err1);
   }

   string to_string(
       const BinocularCameraInfo& bcam_info,
       const EuclideanTransform& et0,
       const array<CachingUndistortInverse, 2> cus) const noexcept override
   {
      const auto et1 = bcam_info.make_et1(et0);
      Vector2 v[2];
      v[0]            = pp(et0, cus[0]);
      v[1]            = pp(et1, cus[1]);
      const auto err0 = (v[0] - u[0]).norm();
      const auto err1 = (v[1] - u[1]).norm();
      return format("    U = {:s}, |{:s} - {:s}| = {}, |{:s} - {:s}| = {}",
                    str(U),
                    str(u[0]),
                    str(v[0]),
                    err0,
                    str(u[1]),
                    str(v[1]),
                    err1);
   }
};

static std::pair<EuclideanTransform, real>
optimize(const int w,
         const int h,
         const BinocularCameraInfo& bcam_info,
         const Vector3& C0, // camera center
         const Vector3& X_world,
         const Vector2& x_cam,
         const Vector3& Yr_world,
         const Vector2& y_cam,
         const vector<unique_ptr<OptConstraint>>& constraints,
         const bool use_nelder_mead = true,
         const bool feedback        = true) noexcept
{
   const int n_params = 3;
   Vector3 C          = C0;
   EuclideanTransform et_opt, eta, etb;
   Quaternion q0, q1;

   array<CachingUndistortInverse, 2> cus;
   for(size_t i = 0; i < 2; ++i) {
      cus[i].init(bcam_info.M[i]);
      cus[i].set_working_format(w, h);
   }

   auto pack = [&](real* X) {
      for(auto i = 0; i < 3; ++i) *X++ = C[i];
   };

   auto unpack = [&](const real* X) {
      for(auto i = 0; i < 3; ++i) C[i] = *X++;
   };

   auto eval = [&](const EuclideanTransform& et) -> real {
      if(constraints.size() == 0) return 0.0;
      auto err = 0.0;
      for(const auto& oc : constraints) err += oc->evaluate(bcam_info, et, cus);
      return err / real(constraints.size());
   };

   auto counter  = 0;
   auto best_err = std::numeric_limits<real>::max();
   auto fn       = [&](const real* X) {
      unpack(X);

      std::tie(q0, q1)
          = make_q01(bcam_info, C, x_cam, y_cam, X_world, Yr_world);
      eta              = dcam_Cq_to_euclidean_transform(C, q0);
      etb              = dcam_Cq_to_euclidean_transform(C, q1);
      const auto err_a = eval(eta);
      const auto err_b = eval(etb);
      const auto err   = std::min(err_a, err_b);

      if(err < best_err) {
         best_err = err;
         et_opt   = (err_a < err_b) ? eta : etb;
         if(feedback) {
            cout << format("#{:4d}, err={:10.8f}", counter, best_err);
            for(auto& oc : constraints)
               cout << format(",  {:10.8f}",
                              oc->evaluate(bcam_info, et_opt, cus));
            cout << endl;
            for(auto& oc : constraints)
               cout << "    " << oc->to_string(bcam_info, et_opt, cus) << endl;
            cout << endl;
         }

         ++counter;
      }
      return err;
   };

   vector<real> start(n_params);
   vector<real> xmin(n_params);
   real ynewlo   = dNAN;
   real ystartlo = dNAN;
   real reqmin   = 1e-7;
   real diffstep = 0.1;

   int kcount = 1000; // max interations
   int icount = 0, numres = 0, ifault = 0;
   const char* method = nullptr;

   pack(&start[0]);
   ystartlo = fn(&start[0]);

   if(!use_nelder_mead) {
      method = "levenberg-marquardt";
      levenberg_marquardt(fn,
                          n_params,
                          &start[0],
                          &xmin[0],
                          reqmin,
                          diffstep,
                          5,
                          kcount,
                          icount,
                          ifault);
      ynewlo = fn(&xmin[0]);
   } else {
      method = "nelder-mead";

      vector<real> step(n_params);
      for(size_t i = 0; i < n_params; ++i) step[i] = 0.2; // 20cm
      nelder_mead(fn,
                  n_params,
                  &start[0],
                  &xmin[0],
                  ynewlo,
                  reqmin,
                  &step[0],
                  10,
                  20 * kcount,
                  icount,
                  numres,
                  ifault);
   }

   ynewlo = best_err;

   if(feedback) {
      INFO(format("Feedback positioning {:s} ({} params)",
                  bcam_info.camera_id,
                  n_params));
      cout << format("   method:               {:s}", method) << endl;
      cout << format("   fault-code:           {}", ifault) << endl;
      auto msg = (use_nelder_mead) ? nelder_mead_fault_str(ifault)
                                   : levenberg_marquardt_fault_str(ifault);
      cout << format("   fault-string:         {:s}", msg) << endl;
      cout << endl;
      cout << format("   initial-score:        {}", ystartlo) << endl;
      cout << format("   final-score:          {}", ynewlo) << endl;
      cout << endl;

      const auto [C, q] = make_camera_extrinsics(et_opt);
      cout << format("    {:s}, C={:s}, q={:s}",
                     bcam_info.camera_id,
                     str(C),
                     q.to_readable_str())
           << endl;

      cout << "." << endl << endl;
   }

   return std::make_pair(et_opt, ynewlo);
}

using Line2D = array<Vector2, 2>;
struct ImageLines
{
   // Line [x, u] and [x, v] are perpendicular
   // [x, u] is on the two planes [X and U]
   // [x, v] is on the two planes [X and V]
   Vector2 x = Vector2::nan();
   Vector2 u = Vector2::nan();
   Vector2 v = Vector2::nan();

   string to_string() const noexcept
   {
      return format("x = {:s}, u = {:s}, v= {:s}", str(x), str(u), str(v));
   }
};

struct LineData
{
   Plane X = Plane::nan();
   Plane U = Plane::nan();
   Plane V = Plane::nan();
   array<ImageLines, 2> ll; // left image and right image
   real error(const array<DistortedCamera, 2>& dcam) const noexcept;

   string to_string() const noexcept
   {
      return format(
          "X = {:s}, U = {:s}, V = {:s}\nll[0] = {:s}\nll[1] = {:s}\n",
          str(X),
          str(U),
          str(V),
          ll[0].to_string(),
          ll[1].to_string());
   }
};

struct RunData
{
   string camera_id;
   Vector2 working_format;
   EuclideanTransform et0;
   Vector3 C0 = Vector3::nan();
   vector<LineData> dat;

   real error(const array<DistortedCamera, 2>& dcam) const noexcept;
   Quaternion est_rotation(const CachingUndistortInverse& cu,
                           const Vector3& C) const noexcept;
};

static real dcam_dist(const DistortedCamera& dcam,
                      const Vector2& x,
                      const Plane& A,
                      const Plane& B)
{
   Vector3 a = plane_ray_intersect(dcam, A, x);
   Vector3 b = plane_ray_intersect(dcam, B, x);
   // INFO(format("|{:s} - {:s}| = {}", str(a), str(b), (a - b).norm()));
   return (a - b).norm();
}

real LineData::error(const array<DistortedCamera, 2>& dcam) const noexcept
{
   const auto& dat = *this;
   auto proc_line  = [&](int i, auto& u, auto& x, auto& U, auto& X) {
      real lerr    = 0.0;
      int lcounter = 0;
      bresenham(u, x, [&](int x, int y) {
         lerr += dcam_dist(dcam[size_t(i)], Vector2(x, y), U, X);
         lcounter++;
      });
      return lerr / real(lcounter);
   };

   auto err    = 0.0;
   int counter = 0;

   for(size_t i = 0; i < 2; ++i) {
      err += dcam_dist(dcam[i], dat.ll[i].x, dat.X, dat.U);
      if(dat.V.is_finite()) {
         err += dcam_dist(dcam[i], dat.ll[i].x, dat.U, dat.V);
         err += dcam_dist(dcam[i], dat.ll[i].x, dat.X, dat.V);
      }

      if(false) {
         if(dat.V.is_finite()) {
            err += 3.0 * proc_line(i, dat.ll[i].u, dat.ll[i].x, dat.U, dat.X);
            err += 3.0 * proc_line(i, dat.ll[i].v, dat.ll[i].x, dat.V, dat.X);
         } else {
            err += 1.0 * proc_line(i, dat.ll[i].u, dat.ll[i].x, dat.U, dat.X);
         }
      } else {
         if(dat.V.is_finite()) {
            err += 3.0 * dcam_dist(dcam[i], dat.ll[i].u, dat.X, dat.U);
            err += 3.0 * dcam_dist(dcam[i], dat.ll[i].v, dat.X, dat.V);
         } else {
            err += 1.0 * dcam_dist(dcam[i], dat.ll[i].u, dat.X, dat.U);
         }
      }

      counter += (dat.V.is_finite()) ? 9 : 3;
   }

   if(!std::isfinite(err)) { cout << dat.to_string() << endl; }
   Expects(std::isfinite(err));

   return err / real(counter);
} // namespace perceive

real RunData::error(const array<DistortedCamera, 2>& dcam) const noexcept
{
   real err = 0.0;
   for(const auto& ld : dat) err += ld.error(dcam);
   err = (dat.size() == 0) ? 0.0 : (err / real(dat.size()));

   if(C0.is_finite()) err += k_regularize(dcam[0].C, C0);

   return err;
}

Quaternion RunData::est_rotation(const CachingUndistortInverse& cu,
                                 const Vector3& C) const noexcept
{
   auto line_normal
       = [](const Plane& X, const Plane& Y) { return cross(X.xyz(), Y.xyz()); };

   for(const auto& ll : dat) {
      if(!ll.V.is_finite()) continue;

      const auto x = homgen_R2_to_P2(cu.undistort(ll.ll[0].x));
      const auto u = homgen_R2_to_P2(cu.undistort(ll.ll[0].u));
      const auto v = homgen_R2_to_P2(cu.undistort(ll.ll[0].v));
      const auto X = intersection_of_3_planes(ll.X, ll.U, ll.V);

      const auto un = line_normal(ll.U, ll.X);
      const auto vn = line_normal(ll.V, ll.X);

      const auto [u0, u1] = calc_rotation(C, x, u, X, un);
      const auto [v0, v1] = calc_rotation(C, x, v, X, vn);

      // Select the rotations that are most similar
      const auto Z        = Vector3(0, 0, 1);
      const real u0_score = dot(u0.rotate(Z), v0.rotate(Z)) - 1.0;
      const real u1_score = dot(u1.rotate(Z), v0.rotate(Z)) - 1.0;

      // Sanity checks

      return (u0_score < u1_score) ? u0 : u1;
   }

   return Quaternion{};
}

static real run_opt(const RunData& dat,
                    const BinocularCameraInfo& bcam_info,
                    array<DistortedCamera, 2>& dcam,
                    EuclideanTransform& inout_et,
                    const bool use_nelder_mead,
                    const bool feedback)
{
   EuclideanTransform et0 = inout_et;

   auto pack = [&](real* X) {
      for(auto i = 0; i < 3; ++i) *X++ = et0.translation[i];
   };

   auto unpack = [&](const real* X) {
      for(auto i = 0; i < 3; ++i) et0.translation[i] = *X++;
      et0.rotation   = dat.est_rotation(dcam[0].cu, et0.translation);
      const auto et1 = bcam_info.make_et1(et0);
      std::tie(dcam[0].C, dcam[0].q) = make_camera_extrinsics(et0);
      std::tie(dcam[1].C, dcam[1].q) = make_camera_extrinsics(et1);
   };

   const auto n_params = 3;
   vector<real> xbest(n_params);
   auto counter  = 0;
   auto best_err = std::numeric_limits<real>::max();
   auto fn       = [&](const real* X) {
      unpack(X);
      const auto err = dat.error(dcam);
      if(err < best_err) {
         best_err = err;
         pack(&xbest[0]);
         if(feedback)
            cout << format("#{:4d}, err={:10.8f}", counter, best_err) << endl;
         ++counter;
      }
      return err;
   };

   vector<real> start(n_params);
   vector<real> xmin(n_params);
   real ynewlo   = dNAN;
   real ystartlo = dNAN;
   real reqmin   = 1e-7;
   real diffstep = 0.1;

   int kcount = 1000; // max interations
   int icount = 0, numres = 0, ifault = 0;
   const char* method = nullptr;

   pack(&start[0]);
   ystartlo = fn(&start[0]);

   if(!use_nelder_mead) {
      method = "levenberg-marquardt";
      levenberg_marquardt(fn,
                          n_params,
                          &start[0],
                          &xmin[0],
                          reqmin,
                          diffstep,
                          5,
                          kcount,
                          icount,
                          ifault);
      ynewlo = fn(&xmin[0]);
   } else {
      method = "nelder-mead";

      vector<real> step(n_params);
      for(size_t i = 0; i < n_params; ++i) step[i] = one_degree();
      nelder_mead(fn,
                  n_params,
                  &start[0],
                  &xmin[0],
                  ynewlo,
                  reqmin,
                  &step[0],
                  10,
                  20 * kcount,
                  icount,
                  numres,
                  ifault);
   }

   ynewlo   = fn(&xbest[0]);
   inout_et = et0;

   if(feedback) {
      INFO(format(
          "Feedback positioning {:s} ({} params)", dat.camera_id, n_params));
      cout << format("   method:               {:s}", method) << endl;
      cout << format("   fault-code:           {}", ifault) << endl;
      auto msg = (use_nelder_mead) ? nelder_mead_fault_str(ifault)
                                   : levenberg_marquardt_fault_str(ifault);
      cout << format("   fault-string:         {:s}", msg) << endl;
      cout << endl;
      cout << format("   initial-score:        {}", ystartlo) << endl;
      cout << format("   final-score:          {}", ynewlo) << endl;
      cout << endl;

      const auto [C, q] = make_camera_extrinsics(et0);
      cout << format("    {:s}, C={:s}, q={:s}",
                     dat.camera_id,
                     str(C),
                     q.to_readable_str())
           << endl;

      cout << "." << endl << endl;
   }

   return ynewlo;
}

static void run_it(const SceneDescription& scene_desc, const RunData& dat)
{
   const auto cam_ind = scene_desc.find_camera(dat.camera_id);
   INFO(format("{:s} ind = {}", dat.camera_id, cam_ind));
   auto bcam_info = scene_desc.bcam_infos[size_t(cam_ind)];
   for(size_t i = 0; i < 2; ++i)
      bcam_info.M[i].set_working_format(unsigned(dat.working_format.x),
                                        unsigned(dat.working_format.y));
   // const auto& et0 = scene_desc.cam_transforms[cam_ind];
   const auto& et0 = dat.et0;

   array<DistortedCamera, 2> dcam;
   std::tie(dcam[0], dcam[1])
       = make_distorted_camera_pair(bcam_info,
                                    et0,
                                    unsigned(dat.working_format.x),
                                    unsigned(dat.working_format.y));

   //
   EuclideanTransform et_opt = et0;
   real best_err             = std::numeric_limits<real>::max();
   const auto max_itr        = 10000;
   for(auto i = 0; i < false && max_itr; ++i) {
      const bool last            = i == 0 or i + 1 == max_itr;
      const bool use_nm          = false; // i % 2 == 0;
      EuclideanTransform et_opt0 = et_opt;
      if(i > 0) {
         auto& et = et_opt0;
         for(auto j = 0; j < 3; ++j)
            et.translation[j] = 40.0 * (uniform() - 0.5);
         Vector3 saa
             = Vector3{uniform() * M_PI, uniform() * M_PI, uniform() * M_PI};
         et.rotation = saa_to_quaternion(saa);
      }

      const auto err = run_opt(dat, bcam_info, dcam, et_opt0, use_nm, false);

      if(err < best_err) {
         best_err         = err;
         et_opt           = et_opt0;
         const auto err_c = err - k_regularize(dcam[0].C, dat.C0);
         cout << format("iteration #{:4d}: {}, {}", i, best_err, err_c) << endl;
      }
   }

   for(auto i = 0; i < 100; ++i) {
      const bool last   = (i + 1 == 100);
      const bool use_nm = (i % 2 == 0);
      const auto err    = run_opt(dat, bcam_info, dcam, et_opt, use_nm, last);
      const auto err_c  = err - k_regularize(dcam[0].C, dat.C0);
      cout << format("iteration #{:4d}: {}, {}", i, err, err_c) << endl;
   }

   for(const auto& ld : dat.dat) {
      for(size_t i = 0; i < 2; ++i) {
         Vector3 x0 = plane_ray_intersect(dcam[i], ld.X, ld.ll[i].x);
         Vector3 x1 = plane_ray_intersect(dcam[i], ld.U, ld.ll[i].x);
         Vector3 x2 = plane_ray_intersect(dcam[i], ld.V, ld.ll[i].x);

         cout << format(
             "x0 = {:s}, x1 = {:s}, x2 = {:s}", str(x0), str(x1), str(x2))
              << endl;
      }
   }
   // if(false) {
   //    Vector3 X = bcam_info.solve3d_from_distorted(dat.ll[0].x, dat.ll[1].x);
   //    cout << format("X-cam = {:s}", str(X)) << endl;
   //    cout << format("X-wrl = {:s}", str(et_opt.apply(X))) << endl;
   //    cout << format("X-wrl = {:s}", str(et_opt.inverse().apply(X))) << endl;
   // }

   cout << et_opt.to_json_str() << endl;
   // cout << et_opt.inverse().to_json_str() << endl;
}

// -----------------------------------------------------------------------------

CATCH_TEST_CASE("PlanePhase", "[plane_phase]")
{
   auto perceive_data_dir = [&]() {
      if(is_testing) FATAL(format("FAIL"));
      return ""s;
   };

   const string manifest_filename1
       = format("{:s}/computer-vision/test-data/museum_videos/manifest.json",
                perceive_data_dir());
   const string manifest_filename
       = format("{:s}/computer-vision/test-data/museum_videos/D2/manifest.json",
                perceive_data_dir());

   shared_ptr<SceneDescription> scene_desc_;
   auto get_scene_desc = [&]() {
      if(scene_desc_ == nullptr) {
         const Json::Value manifest_data
             = parse_json(file_get_contents(manifest_filename));
         scene_desc_ = make_shared<SceneDescription>();
         SceneDescription::InitOptions opts;
         scene_desc_->init_from_json(manifest_data, opts);
      }
      return scene_desc_;
   };

   const real cam_z0 = 4.28;
   const auto Z_axis = Vector3(0, 0, 1);

   auto make_at = [&](real x, real y) {
      const real m_p_pix = 0.042109;
      return Vector3((x - 860.0) * m_p_pix, (470 - y) * m_p_pix, 0.0);
   };

   // The planes
   const auto p3floor     = Plane(0, 0, 1, 0);
   const auto p3baseboard = Plane(0, 0, 1, 0); //
   const auto p3A         = Plane(1, 0, 0, -9.55883);
   const auto p3B         = Plane(0, 1, 0, -14.06453);
   const auto p3C         = Plane(0, 1, 0, -13.85398);
   const auto p3D         = Plane(0, 1, 0, -10.73789);
   const auto p3E         = Plane(0, 1, 0, -9.81148);
   const auto p3F         = Plane(0, 1, 0, -5.76898);
   const auto p3G         = Plane(1, 0, 0, -3.07398);
   const auto p3H         = Plane(0, 1, 0, -5.55844); //
   const auto p3I         = Plane(1, 0, 0, -13.096);
   const auto p3J         = Plane(0, 1, 0, -0.8843);
   const auto p3K         = Plane(0, 1, 0, 4.505703);
   const auto p3L         = Plane(1, 0, 0, 1.810703);
   const auto p3M         = Plane(1, 0, 0, -5.55844);
   const auto p3N         = Plane(1, 0, 0, 0.757969);
   const auto p3O         = Plane(1, 0, 0, 8.75875);
   const auto p3P         = Plane(0, 1, 0, -16.42265625);
   const auto p3Q         = Plane(1, 0, 0, 9.685156);
   const auto p3R         = Plane(0, 1, 0, -19.7493);
   const auto p3S         = Plane(1, 0, 0, 22.06531);
   const auto p3T         = Plane(0, 1, 0, -10.7379);
   const auto p3U         = Plane(1, 0, 0, 0.0);
   const auto p3V         = Plane(1, 0, 0, -3.70563);
   const auto p3W         = Plane(1, 0, 0, -9.39039);     //
   const auto p3X         = Plane(1, 0, 0, -0.294765625); //
   const auto p3Y         = Plane(1, 0, 0, -4.5478125);
   const auto p3Z         = Plane(0, 1, 0, -16.42265625);
   const auto p31         = Plane(1, 0, 0, 17.93859375);
   const auto p32         = Plane(0, 1, 0, -17.054296875);
   const auto p33         = Plane(1, 0, 0, 12.464375);
   const auto p34         = Plane(0, 1, 0, -14.022421875);
   const auto p35         = Plane(0, 1, 0, -13.138125);
   const auto p36         = Plane(1, 0, 0, 13.39078125);

   // Quaternion q = look_at(
   //     Vector3{2.0, 0.0, 2.0}, Vector3{2.0, 2.0, 2.0}, Vector3{0.0,
   //     0.0, 1.0});
   // cout << format("q.rotate => {:s}", str(q.rotate(Vector3(0, 0, 1)))) <<
   // endl; FATAL("kBAM!");

   //
   // ------------------------------------------------ plane-phase
   //
   CATCH_SECTION("plane-phase_1022")
   {
      if(!is_testing) return;

      auto scene_desc = get_scene_desc();
      RunData dat;
      dat.camera_id      = "C0001022_v2"s;
      dat.working_format = Vector2(896, 672);
      const Vector3 C    = Vector3(4.362411, 18.252205, cam_z0);
      const Quaternion q = look_at(C, make_at(983, 139), Z_axis);
      dat.et0            = dcam_Cq_to_euclidean_transform(C, q);
      dat.C0             = C;

      auto make_l1 = [&]() {
         auto ld = LineData{};
         ld.X    = p3baseboard;
         ld.U    = p3A;
         ld.V    = p3D;
         if(false) {
            ld.ll[0].x = Vector2(334, 190); // From the floor
            ld.ll[0].u = Vector2(117, 413);
            ld.ll[0].v = Vector2(552, 211);
            ld.ll[1].x = Vector2(315, 176);
            ld.ll[1].u = Vector2(101, 382);
            ld.ll[1].v = Vector2(573, 208);
         } else {
            ld.ll[0].x = Vector2(328, 180); // From the baseboard
            ld.ll[0].u = Vector2(79, 430);
            ld.ll[0].v = Vector2(602, 207);
            ld.ll[1].x = Vector2(310, 166);
            ld.ll[1].u = Vector2(45, 414);
            ld.ll[1].v = Vector2(583, 200);
         }
         return ld;
      };

      auto make_l2 = [&]() {
         auto ld = LineData{};
         ld.X    = p3baseboard;
         ld.U    = p3B;
         if(false) { // From the floor
            ld.ll[0].x = Vector2(883, 409);
            ld.ll[0].u = Vector2(588, 359);
            ld.ll[1].x = Vector2(884, 410);
            ld.ll[1].u = Vector2(565, 352);
         } else { // From the baseboard
            ld.ll[0].x = Vector2(632, 353);
            ld.ll[0].u = Vector2(862, 391);
            ld.ll[1].x = Vector2(573, 340);
            ld.ll[1].u = Vector2(858, 392);
         }
         return ld;
      };

      dat.dat.push_back(make_l1());
      dat.dat.push_back(make_l2());

      run_it(*scene_desc, dat);

      //
      CATCH_REQUIRE(true);
   }

   CATCH_SECTION("plane-phase_1023")
   {
      if(!is_testing) return;

      auto scene_desc = get_scene_desc();

      RunData dat;
      dat.camera_id      = "C0001023_v1"s;
      dat.working_format = Vector2(896, 672);
      const Vector3 C    = Vector3(2.273, 13.6418, cam_z0);
      const Quaternion q = Quaternion(0, 0, 0, 1);
      dat.et0            = dcam_Cq_to_euclidean_transform(C, q);
      dat.C0             = C;

      auto make_l1 = [&]() {
         auto ld    = LineData{};
         ld.X       = p3baseboard;
         ld.U       = p3A;
         ld.V       = p3D;
         ld.ll[0].x = Vector2(225, 220);
         ld.ll[0].u = Vector2(5, 295);
         ld.ll[0].v = Vector2(416, 373);
         ld.ll[1].x = Vector2(231, 244);
         ld.ll[1].u = Vector2(6, 318);
         ld.ll[1].v = Vector2(417, 398);
         return ld;
      };

      auto make_l2 = [&]() {
         auto ld    = LineData{};
         ld.X       = p3baseboard;
         ld.U       = p3C;
         ld.ll[0].x = Vector2(105, 549);
         ld.ll[0].u = Vector2(150, 668);
         ld.ll[1].x = Vector2(101, 568);
         ld.ll[1].u = Vector2(134, 668);
         return ld;
      };

      auto make_l3 = [&]() {
         auto ld    = LineData{};
         ld.X       = p3baseboard;
         ld.U       = p3F;
         ld.ll[0].x = Vector2(616, 211);
         ld.ll[0].u = Vector2(892, 312);
         ld.ll[1].x = Vector2(604, 229);
         ld.ll[1].u = Vector2(889, 333);
         return ld;
      };

      auto make_l4 = [&]() {
         auto ld    = LineData{};
         ld.X       = p3baseboard;
         ld.U       = p3G;
         ld.V       = p3D;
         ld.ll[0].x = Vector2(500, 441);
         ld.ll[0].u = Vector2(557, 395);
         ld.ll[0].v = Vector2(397, 357);
         ld.ll[1].x = Vector2(498, 465);
         ld.ll[1].u = Vector2(559, 419);
         ld.ll[1].v = Vector2(378, 366);
         return ld;
      };

      dat.dat.push_back(make_l1());
      // dat.dat.push_back(make_l2());
      // dat.dat.push_back(make_l3());
      dat.dat.push_back(make_l4());

      run_it(*scene_desc, dat);

      //
      CATCH_REQUIRE(true);
   }

   CATCH_SECTION("plane-phase_1026")
   {
      if(!is_testing) return;

      auto scene_desc = get_scene_desc();

      const int w = 896;
      const int h = 672;

      RunData dat;
      dat.camera_id      = "C0001026_v3"s;
      dat.working_format = Vector2(w, h);
      const Vector3 C    = Vector3(12.96969, 5.937422, cam_z0);
      const Quaternion q = Quaternion(0, 0, 0, 1);
      dat.et0            = dcam_Cq_to_euclidean_transform(C, q);
      dat.C0             = C;

      auto make_l1 = [&]() {
         auto ld    = LineData{};
         ld.X       = p3baseboard;
         ld.U       = p3G;
         ld.V       = p3E;
         ld.ll[0].x = Vector2(674, 97);
         ld.ll[0].u = Vector2(689, 47);
         ld.ll[0].v = Vector2(879, 365);
         ld.ll[1].x = Vector2(1523 - w, 100);
         ld.ll[1].u = Vector2(1538 - w, 54);
         ld.ll[1].v = Vector2(1756 - w, 450);
         return ld;
      };

      auto make_l2 = [&]() {
         auto ld    = LineData{};
         ld.X       = p3baseboard;
         ld.U       = p3E;
         ld.ll[0].x = Vector2(674, 97);
         ld.ll[0].u = Vector2(879, 365);
         ld.ll[1].x = Vector2(1523 - w, 100);
         ld.ll[1].u = Vector2(1756 - w, 450);
         return ld;
      };

      auto make_l3 = [&]() {
         auto ld    = LineData{};
         ld.X       = p3F;
         ld.U       = p3W;
         ld.ll[0].x = Vector2(464, 350);
         ld.ll[0].u = Vector2(463, 35);
         ld.ll[1].x = Vector2(1304 - w, 342);
         ld.ll[1].u = Vector2(1303 - w, 116);
         return ld;
      };

      auto make_l4 = [&]() {
         auto ld    = LineData{};
         ld.X       = p3H;
         ld.U       = p3W;
         ld.ll[0].x = Vector2(440, 351);
         ld.ll[0].u = Vector2(423, 26);
         ld.ll[1].x = Vector2(1281 - w, 342);
         ld.ll[1].u = Vector2(1262 - w, 24);
         return ld;
      };

      // dat.dat.push_back(make_l1());
      dat.dat.push_back(make_l2());
      dat.dat.push_back(make_l3());
      dat.dat.push_back(make_l4());

      run_it(*scene_desc, dat);
      CATCH_REQUIRE(true);
   }

   CATCH_SECTION("plane-phase_1028")
   {
      if(!is_testing) return;

      auto scene_desc = get_scene_desc();

      const int w = 896;
      const int h = 672;

      RunData dat;
      dat.camera_id      = "C0001028_v1"s;
      dat.working_format = Vector2(w, h);
      const Vector3 C    = Vector3(4.80046875, 0.92640625, cam_z0);
      const Quaternion q = Quaternion(0, 0, 0, 1);
      dat.et0            = dcam_Cq_to_euclidean_transform(C, q);
      dat.C0             = C;

      auto make_l1 = [&]() {
         auto ld = LineData{};
         ld.X    = p3baseboard;
         ld.U    = p3H;
         ld.V    = p3W;
         if(true) {
            ld.ll[0].x = Vector2(630, 302);
            ld.ll[0].u = Vector2(39, 318);
            ld.ll[0].v = Vector2(684, 146);
            ld.ll[1].x = Vector2(1531 - w, 299);
            ld.ll[1].u = Vector2(937 - w, 310);
            ld.ll[1].v = Vector2(1580 - w, 145);
         } else {
            ld.ll[0].x = Vector2(630, 302);
            ld.ll[0].u = Vector2(39, 318);
            ld.ll[0].v = Vector2(684, 146);
            ld.ll[1].x = Vector2(1531 - w, 299);
            ld.ll[1].u = Vector2(937 - w, 310);
            ld.ll[1].v = Vector2(1580 - w, 145);
         }
         return ld;
      };

      auto make_l2 = [&]() {
         auto ld = LineData{};
         ld.X    = p3baseboard;
         ld.U    = p3E;
         ld.V    = p3I;
         if(false) {
            ld.ll[0].x = Vector2(765, 141);
            ld.ll[0].u = Vector2(689, 143);
            ld.ll[0].v = Vector2(876, 241);
            ld.ll[1].x = Vector2(1670 - w, 138);
            ld.ll[1].u = Vector2(1586 - w, 140);
            ld.ll[1].v = Vector2(1772 - w, 235);
         } else {
            ld.ll[0].x = Vector2(765, 141);
            ld.ll[0].u = Vector2(689, 143);
            ld.ll[0].v = Vector2(793, 166);
            ld.ll[1].x = Vector2(1670 - w, 138);
            ld.ll[1].u = Vector2(1586 - w, 140);
            ld.ll[1].v = Vector2(1687 - w, 155);
         }
         return ld;
      };

      // dat.dat.push_back(make_l1());
      dat.dat.push_back(make_l2());

      run_it(*scene_desc, dat);
      CATCH_REQUIRE(true);
   }

   CATCH_SECTION("pp_1021") // -------------------------------------------- 1021
   {
      if(!is_testing) return;

      const auto camera_id = "C0001021_v1"s;
      BinocularCameraInfo bcam_info;
      fetch(bcam_info, camera_id);

      const int w = 896;
      const int h = 672;

      for(auto& m : bcam_info.M) m.set_working_format(w, h);

      const auto C0 = Vector3(-3.916171875, 13.6434375, 4.222038);
      const auto X  = intersection_of_3_planes(p3P, p3X, p3floor);
      const auto x  = Vector2(553, 249); // Floor
      const auto Yr = Vector3(0, 0, 1);  // Up
      const auto y  = Vector2(594, 50);  // Up

      // Constraints
      vector<unique_ptr<OptConstraint>> ocs;
      ocs.push_back(make_unique<LineConstraint>(p3B, p3N, Vector2(796, 189)));
      ocs.push_back(
          make_unique<LineConstraint>(p3N, p3floor, Vector2(748, 418)));
      ocs.push_back(
          make_unique<LineConstraint>(p3N, p3floor, Vector2(391, 211)));
      ocs.push_back(make_unique<LineConstraint>(p3R, p3N, Vector2(354, 178)));
      ocs.push_back(
          make_unique<LineConstraint>(p3R, p3floor, Vector2(100, 296)));

      ocs.push_back(make_unique<BinoConstraint>(
          intersection_of_3_planes(p3P, p3X, p3floor),
          Vector2(553, 249),
          Vector2(1444 - w, 255)));
      ocs.push_back(make_unique<BinoConstraint>(
          intersection_of_3_planes(p3B, p3N, p3floor),
          Vector2(685, 383),
          Vector2(1567 - w, 384)));

      const auto [et_opt, ynewlo]
          = optimize(w, h, bcam_info, C0, X, x, Yr, y, ocs);

      cout << et_opt.to_json_str() << endl;
   }

   CATCH_SECTION("pp_1022") // -------------------------------------------- 1022
   {
      if(!is_testing) return;

      const auto camera_id = "C0001022_v2"s;
      BinocularCameraInfo bcam_info;
      fetch(bcam_info, camera_id);

      const int w = 896;
      const int h = 672;

      for(auto& m : bcam_info.M) m.set_working_format(w, h);

      const auto C0 = Vector3(5, 19, 4.222038);
      const auto X  = intersection_of_3_planes(p3Y, p3B, p3floor);
      const auto x  = Vector2(726, 415); // Floor
      const auto Yr = Vector3(0, 0, 1);  // Up
      const auto y  = Vector2(813, 269); // Up

      // Constraints
      vector<unique_ptr<OptConstraint>> ocs;
      ocs.push_back(
          make_unique<LineConstraint>(p3A, p3floor, Vector2(283, 330)));
      ocs.push_back(
          make_unique<LineConstraint>(p3A, p3floor, Vector2(464, 210)));
      ocs.push_back(
          make_unique<LineConstraint>(p3D, p3floor, Vector2(761, 254)));
      ocs.push_back(make_unique<LineConstraint>(p3A, p3D, Vector2(506, 170)));
      ocs.push_back(
          make_unique<LineConstraint>(p3B, p3floor, Vector2(828, 468)));
      ocs.push_back(make_unique<LineConstraint>(p3B, p3Y, Vector2(787, 311)));

      ocs.push_back(make_unique<BinoConstraint>(
          intersection_of_3_planes(p3Y, p3B, p3floor),
          Vector2(726, 415),
          Vector2(1593 - w, 409)));
      ocs.push_back(make_unique<BinoConstraint>(
          intersection_of_3_planes(p3A, p3D, p3floor),
          Vector2(507, 181),
          Vector2(1386 - w, 172)));

      const auto [et_opt, ynewlo]
          = optimize(w, h, bcam_info, C0, X, x, Yr, y, ocs);

      cout << et_opt.to_json_str() << endl;
   }

   CATCH_SECTION("pp_1023") // -------------------------------------------- 1023
   {
      if(!is_testing) return;

      const auto camera_id = "C0001023_v1"s;
      BinocularCameraInfo bcam_info;
      fetch(bcam_info, camera_id);

      const int w = 896;
      const int h = 672;

      for(auto& m : bcam_info.M) m.set_working_format(w, h);

      const auto C0 = Vector3(1.462102, 13.050914, 3.948683);
      const auto X  = intersection_of_3_planes(p3D, p3G, p3floor);
      const auto x  = Vector2(500, 454); // Floor|D|G
      const auto Yr = Vector3(0, 0, 1);  // Up
      const auto y  = Vector2(529, 220); // Up

      // Constraints
      vector<unique_ptr<OptConstraint>> ocs;
      ocs.push_back(make_unique<LineConstraint>(p3D, p3G, Vector2(522, 282)));
      ocs.push_back(
          make_unique<LineConstraint>(p3G, p3floor, Vector2(528, 431)));
      ocs.push_back(
          make_unique<LineConstraint>(p3C, p3floor, Vector2(124, 587)));
      ocs.push_back(
          make_unique<LineConstraint>(p3A, p3floor, Vector2(197, 243)));
      ocs.push_back(make_unique<LineConstraint>(p3A, p3D, Vector2(226, 222)));
      // ocs.push_back(
      //     make_unique<LineConstraint>(p3G, p3E, Vector2(586, 284)));
      // ocs.push_back(
      //     make_unique<LineConstraint>(p3F, p3floor, Vector2(726, 252)));

      ocs.push_back(make_unique<BinoConstraint>(
          intersection_of_3_planes(p3G, p3D, p3floor),
          Vector2(500, 454),
          Vector2(1393 - w, 478)));
      // ocs.push_back(make_unique<BinoConstraint>(
      //     intersection_of_3_planes(p3E, p3G, p3floor),
      //     Vector2(560, 405),
      //     Vector2(1454 - w, 431)));
      // ocs.push_back(make_unique<BinoConstraint>(
      //     intersection_of_3_planes(p3A, p3D, p3floor),
      //     Vector2(228, 233),
      //     Vector2(1131 - w, 256)));

      const auto [et_opt, ynewlo]
          = optimize(w, h, bcam_info, C0, X, x, Yr, y, ocs);

      cout << et_opt.to_json_str() << endl;
   }

   CATCH_SECTION("pp_1024") // -------------------------------------------- 1024
   {
      if(!is_testing) return;

      const auto camera_id = "C0001024_v2"s;
      BinocularCameraInfo bcam_info;
      fetch(bcam_info, camera_id);

      const int w = 896;
      const int h = 672;

      for(auto& m : bcam_info.M) m.set_working_format(w, h);

      const auto C0 = Vector3(-10.737890625, 18.822890625, 4.222038);
      const auto X  = intersection_of_3_planes(p32, p36, p3floor);
      const auto x  = Vector2(303, 587); // Floor
      const auto Yr = Vector3(0, 0, 1);  // Up
      const auto y  = Vector2(191, 259); // Up

      // Constraints
      vector<unique_ptr<OptConstraint>> ocs;
      ocs.push_back(make_unique<LineConstraint>(p31, p34, Vector2(291, 162)));
      // ocs.push_back(
      //     make_unique<LineConstraint>(p31, p3floor, Vector2(493, 333)));
      // ocs.push_back(
      //     make_unique<LineConstraint>(p3R, p3floor, Vector2(641, 473)));

      ocs.push_back(make_unique<BinoConstraint>( //
          intersection_of_3_planes(p31, p34, p3floor),
          Vector2(361, 300),
          Vector2(1245 - w, 284)));
      ocs.push_back(make_unique<BinoConstraint>( //
          intersection_of_3_planes(p32, p36, p3floor),
          Vector2(303, 587),
          Vector2(1216 - w, 571)));

      const auto [et_opt, ynewlo]
          = optimize(w, h, bcam_info, C0, X, x, Yr, y, ocs);

      cout << et_opt.to_json_str() << endl;
   }

   CATCH_SECTION("pp_1025") // -------------------------------------------- 1025
   {
      if(!is_testing) return;

      const auto camera_id = "C0001025_v1"s;
      BinocularCameraInfo bcam_info;
      fetch(bcam_info, camera_id);

      const int w = 896;
      const int h = 672;

      for(auto& m : bcam_info.M) m.set_working_format(w, h);

      const auto C0 = Vector3(-5.726875, 14.106640625, 4.222038);
      const auto X  = intersection_of_3_planes(p3Z, p3Q, p3floor);
      const auto x  = Vector2(210, 395); // Floor
      const auto Yr = Vector3(0, 0, 1);  // Up
      const auto y  = Vector2(111, 205); // Up

      // Constraints
      vector<unique_ptr<OptConstraint>> ocs;
      ocs.push_back(make_unique<LineConstraint>(p3O, p3R, Vector2(383, 239)));
      ocs.push_back(
          make_unique<LineConstraint>(p3O, p3floor, Vector2(386, 248)));
      ocs.push_back(
          make_unique<LineConstraint>(p3R, p3floor, Vector2(386, 248)));
      ocs.push_back(
          make_unique<LineConstraint>(p3R, p3floor, Vector2(639, 281)));
      ocs.push_back(
          make_unique<LineConstraint>(p3O, p3floor, Vector2(325, 347)));
      ocs.push_back(
          make_unique<LineConstraint>(p3Z, p3floor, Vector2(238, 401)));

      ocs.push_back(make_unique<BinoConstraint>( //
          intersection_of_3_planes(p3R, p3O, p3floor),
          Vector2(386, 248),
          Vector2(1257 - w, 267)));
      ocs.push_back(make_unique<BinoConstraint>(
          intersection_of_3_planes(p3Z, p3Q, p3floor),
          Vector2(210, 395),
          Vector2(1074 - w, 422)));

      const auto [et_opt, ynewlo]
          = optimize(w, h, bcam_info, C0, X, x, Yr, y, ocs);

      cout << et_opt.to_json_str() << endl;
   }

   CATCH_SECTION("pp_1027") // -------------------------------------------- 1027
   {
      if(!is_testing) return;

      const auto camera_id = "C0001027_v1"s;
      BinocularCameraInfo bcam_info;
      fetch(bcam_info, camera_id);

      const int w = 896;
      const int h = 672;

      for(auto& m : bcam_info.M) m.set_working_format(w, h);

      const auto C0 = Vector3(4.187025, 7.171535, 2.599380);
      const auto X  = intersection_of_3_planes(p3E, p3G, p3floor);
      const auto x  = Vector2(249, 306); // Floor|E|G
      const auto Yr = Vector3(0, 0, 1);  // Up
      const auto y  = Vector2(164, 98);  // Up

      // Constraints
      vector<unique_ptr<OptConstraint>> ocs;
      ocs.push_back(make_unique<LineConstraint>(p3E, p3G, Vector2(184, 146)));
      ocs.push_back(
          make_unique<LineConstraint>(p3E, p3floor, Vector2(670, 270)));
      ocs.push_back(
          make_unique<LineConstraint>(p3E, p3floor, Vector2(413, 291)));
      ocs.push_back(
          make_unique<LineConstraint>(p3C, p3floor, Vector2(158, 107)));
      ocs.push_back(
          make_unique<LineConstraint>(p3X, p3floor, Vector2(36, 161)));

      ocs.push_back(make_unique<BinoConstraint>(
          intersection_of_3_planes(p3E, p3G, p3floor),
          Vector2(249, 306),
          Vector2(1124 - w, 270)));
      ocs.push_back(make_unique<BinoConstraint>(
          intersection_of_3_planes(p3X, p3C, p3floor),
          Vector2(68, 107),
          Vector2(952 - w, 58)));

      const auto [et_opt, ynewlo]
          = optimize(w, h, bcam_info, C0, X, x, Yr, y, ocs);

      cout << et_opt.to_json_str() << endl;
   }

   CATCH_SECTION("pp_1026") // -------------------------------------------- 1026
   {
      if(!is_testing) return;

      //
      // auto scene_desc = get_scene_desc();

      const auto camera_id = "C0001026_v3"s;
      BinocularCameraInfo bcam_info;
      fetch(bcam_info, camera_id);

      const int w = 896;
      const int h = 672;

      for(auto& m : bcam_info.M) m.set_working_format(w, h);

      const auto C0 = Vector3(12.453906, 5.937422, 3.329993);
      const auto X  = Vector3(9.34828125, 5.5584375, 0.0); // Floor|W|H
      const auto x  = Vector2(440, 363);                   // Floor|W|H
      const auto Yr = Vector3(0, 0, 1);                    // Up
      const auto y  = Vector2(423, 43);                    // Up

      // Constraints
      vector<unique_ptr<OptConstraint>> ocs;
      ocs.push_back(
          make_unique<LineConstraint>(p3E, p3floor, Vector2(670, 110)));
      ocs.push_back(
          make_unique<LineConstraint>(p3J, p3floor, Vector2(194, 145)));
      ocs.push_back(make_unique<BinoConstraint>(
          intersection_of_3_planes(p3H, p3W, p3floor),
          Vector2(440, 363),
          Vector2(1281 - w, 355)));
      ocs.push_back(make_unique<BinoConstraint>(
          intersection_of_3_planes(p3F, p3W, p3floor),
          Vector2(463, 363),
          Vector2(1304 - w, 355)));
      ocs.push_back(make_unique<BinoConstraint>(
          intersection_of_3_planes(p3E, p3G, p3floor),
          Vector2(670, 110),
          Vector2(1518 - w, 113)));
      ocs.push_back(make_unique<BinoConstraint>(
          intersection_of_3_planes(p3J, p3V, p3floor),
          Vector2(194, 145),
          Vector2(1046 - w, 128)));

      const auto [et_opt, ynewlo]
          = optimize(w, h, bcam_info, C0, X, x, Yr, y, ocs);

      cout << et_opt.to_json_str() << endl;
   }

   CATCH_SECTION("pp_1028") // -------------------------------------------- 1028
   {
      if(!is_testing) return;

      const auto camera_id = "C0001028_v1"s;
      BinocularCameraInfo bcam_info;
      fetch(bcam_info, camera_id);

      const int w = 896;
      const int h = 672;

      for(auto& m : bcam_info.M) m.set_working_format(w, h);

      const auto C0 = Vector3(7.400000, 1.500000, 4.029993);
      const auto X  = Vector3(9.34828125, 5.5584375, 0.0); // Floor|W|H
      const auto x  = Vector2(627, 314);                   // Floor|W|H
      const auto Yr = Vector3(0, 0, 1);                    // Up
      const auto y  = Vector2(677, 164);                   // Up

      // Constraints
      vector<unique_ptr<OptConstraint>> ocs;
      ocs.push_back(make_unique<LineConstraint>(p3W, p3H, Vector2(636, 285)));
      ocs.push_back(make_unique<LineConstraint>(p3W, p3H, Vector2(676, 167)));
      ocs.push_back(
          make_unique<LineConstraint>(p3H, p3floor, Vector2(32, 331)));
      ocs.push_back(
          make_unique<LineConstraint>(p3H, p3floor, Vector2(275, 326)));
      ocs.push_back(make_unique<LineConstraint>(p3E, p3I, Vector2(772, 125)));
      ocs.push_back(
          make_unique<LineConstraint>(p3I, p3floor, Vector2(779, 172)));

      ocs.push_back(make_unique<BinoConstraint>(
          intersection_of_3_planes(p3H, p3W, p3floor),
          Vector2(626, 314),
          Vector2(1528 - w, 310)));
      ocs.push_back(make_unique<BinoConstraint>(
          intersection_of_3_planes(p3I, p3E, p3floor),
          Vector2(760, 154),
          Vector2(1665 - w, 150)));

      const auto [et_opt, ynewlo]
          = optimize(w, h, bcam_info, C0, X, x, Yr, y, ocs);

      cout << et_opt.to_json_str() << endl;
   }

   CATCH_SECTION("pp_1029") // -------------------------------------------- 1029
   {
      if(!is_testing) return;

      const auto camera_id = "C0001029_v2"s;
      BinocularCameraInfo bcam_info;
      fetch(bcam_info, camera_id);

      const int w = 896;
      const int h = 672;

      for(auto& m : bcam_info.M) m.set_working_format(w, h);

      const auto C0 = Vector3(-27.286875, 15.49625, 4.000000);
      const auto X  = intersection_of_3_planes(p3S, p35, p3floor);
      const auto x  = Vector2(615, 458); //
      const auto Yr = Vector3(0, 0, 1);  // Up
      const auto y  = Vector2(688, 109); // Up

      // Constraints
      vector<unique_ptr<OptConstraint>> ocs;
      ocs.push_back(make_unique<LineConstraint>(p3S, p35, Vector2(645, 308)));
      ocs.push_back(
          make_unique<LineConstraint>(p3S, p3floor, Vector2(172, 483)));
      ocs.push_back(
          make_unique<LineConstraint>(p3T, p3floor, Vector2(823, 465)));

      ocs.push_back(make_unique<BinoConstraint>(
          intersection_of_3_planes(p3S, p35, p3floor),
          Vector2(615, 458),
          Vector2(1435 - w, 465)));

      const auto [et_opt, ynewlo]
          = optimize(w, h, bcam_info, C0, X, x, Yr, y, ocs);

      cout << et_opt.to_json_str() << endl;
   }

   CATCH_SECTION("pp_1030") // -------------------------------------------- 1030
   {
      if(!is_testing) return;
      const auto camera_id = "C0001030_v2"s;
      BinocularCameraInfo bcam_info;
      fetch(bcam_info, camera_id);

      const int w = 896;
      const int h = 672;

      for(auto& m : bcam_info.M) m.set_working_format(w, h);

      const auto C0 = Vector3(2.450000, 5.350000, 4.000000);
      const auto X  = intersection_of_3_planes(p3V, p3J, p3floor);
      const auto x  = Vector2(498, 360); // Floor|V|J
      const auto Yr = Vector3(0, 0, 1);  // Up
      const auto y  = Vector2(537, 32);  // Up

      // Constraints
      vector<unique_ptr<OptConstraint>> ocs;
      ocs.push_back(make_unique<LineConstraint>(p3V, p3J, Vector2(517, 194)));
      ocs.push_back(
          make_unique<LineConstraint>(p3V, p3floor, Vector2(509, 338)));
      ocs.push_back(
          make_unique<LineConstraint>(p3U, p3floor, Vector2(851, 438)));
      ocs.push_back(
          make_unique<LineConstraint>(p3J, p3floor, Vector2(114, 245)));

      ocs.push_back(make_unique<BinoConstraint>(
          intersection_of_3_planes(p3V, p3J, p3floor),
          Vector2(498, 360),
          Vector2(1361 - w, 339)));
      ocs.push_back(make_unique<BinoConstraint>(
          intersection_of_3_planes(p3U, p3J, p3floor),
          Vector2(856, 466),
          Vector2(1695 - w, 450)));

      const auto [et_opt, ynewlo]
          = optimize(w, h, bcam_info, C0, X, x, Yr, y, ocs);

      cout << et_opt.to_json_str() << endl;
   }
}

} // namespace perceive
