
#include "helpers.hpp"

#include "manifest-data.hpp"
#include "print-helper.hpp"

#include "perceive/calibration/n-way-camera.hpp"
#include "perceive/geometry/projective/distorted-camera.hpp"
#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/tiny-string.hpp"
#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"
#include "perceive/scene/scene-description-info.hpp"
#include "perceive/utils/file-system.hpp"

namespace perceive::calibration::position_scene_cameras
{
// ------------------------------------------------------------------ make-scene
//
static SceneDescriptionInfo
make_scene(const string_view scene_id,
           const ManifestData& mdata,
           const vector<EuclideanTransform>& ets) noexcept(false)
{
   vector<string> bcam_names;
   bcam_names.reserve(size_t(mdata.n_cameras()));
   std::transform(cbegin(mdata.cam_infos),
                  cend(mdata.cam_infos),
                  std::back_inserter(bcam_names),
                  std::mem_fn(&CamInfo::camera_id));

   SceneDescriptionInfo scene;

   scene.store_id        = 0;
   scene.scene_key       = scene_id;
   scene.bcam_keys       = std::move(bcam_names);
   scene.bcam_transforms = ets;

   return scene;
}

// -------------------------------------------------------------- calc_detect_Ws
//
using Quad3D = array<Vector3, 4>;

static Quad3D apply_et(const EuclideanTransform& cube_et, const Quad3D& quad)
{
   Quad3D o;
   for(size_t i = 0; i < 4; ++i) o[i] = cube_et.apply(quad[i]);
   return o;
}

static vector<vector<Quad3D>>
calc_detect_Ws(const ArucoCube& ac,
               const vector<ArucoCube::FaceDetection>& detects,
               const EuclideanTransform& cube_et)
{
   auto face_detection_to_quads = [&](const ArucoCube::FaceDetection& detect) {
      Expects(detect.quads.size() == detect.marker_ids.size());
      const auto& face_spec = ac.measures.at(size_t(detect.f_ind));
      vector<Quad3D> Ws;
      Ws.reserve(detect.quads.size());
      std::transform(cbegin(detect.marker_ids),
                     cend(detect.marker_ids),
                     std::back_inserter(Ws),
                     [&](const int marker_id) {
                        const int m_pos = face_spec.marker_pos(marker_id);
                        Expects(m_pos >= 0 and m_pos < 4);
                        return apply_et(cube_et,
                                        face_spec.marker_3d_quad(m_pos));
                     });
      return Ws;
   };

   vector<vector<Quad3D>> detect_Ws;
   detect_Ws.reserve(detects.size());
   std::transform(cbegin(detects),
                  cend(detects),
                  std::back_inserter(detect_Ws),
                  face_detection_to_quads);

   return detect_Ws;
}

// ------------------------------------------------------------ render-pointwise
//
cv::Mat render_pointwise(const ArucoCube& ac,
                         const cv::Mat& image,
                         const CachingUndistortInverse& cu,
                         const vector<ArucoCube::FaceDetection>& detects,
                         const EuclideanTransform& cam_et,
                         const EuclideanTransform& cube_et) noexcept
{
   ARGBImage out = cv_to_argb(image);
   Expects(out.size() > 0);

   const auto dcam = make_distorted_camera(cam_et, cu, out.width, out.height);

   auto project
       = [&](const Vector3& X) { return project_to_distorted(dcam, X); };

   auto render_quad = [&](const Quad3D& Xs, uint32_t k) {
      array<Vector2, 4> Q;
      std::transform(cbegin(Xs), cend(Xs), begin(Q), project);
      render_quad_2d(out, Q, k);
   };

   auto project_render_line
       = [&](const Vector3& A, const Vector3& B, const uint32_t k) {
            const auto a = project(A);
            const auto b = project(B);
            plot_line_AA(a, b, out.bounds(), [&](int x, int y, float a) {
               if(out.in_bounds(x, y)) out(x, y) = blend(k, out(x, y), a);
            });
         };

   plot_axis(out, dcam);

   { // Draw each visible face
      for(const auto& detect : detects) {
         const auto& M = ac.measures[size_t(detect.f_ind)];
         const auto k  = M.kolour;

         // The bounding quad
         const auto Q = apply_et(cube_et.inverse(), M.Xs);
         render_quad(Q, M.kolour);

         if(detect.f_ind == ArucoCube::TOP) {
            for(size_t i = 0; i < 4; ++i) {
               const auto x = project(Q[i]);
               render_string(out, format("{}", i), to_pt2(x), k_red, k_black);
            }
         }
      }
   }

   return argb_to_cv(out);
}

// ---------------------------------------------------------------- render-dense
//
cv::Mat render_dense(const ArucoCube& ac,
                     const cv::Mat& image,
                     const CachingUndistortInverse& cu,
                     const vector<ArucoCube::FaceDetection>& detects,
                     const array<cv::Mat, 6>& face_ims,
                     const EuclideanTransform& cam_et, // Cam->World
                     const EuclideanTransform& cube_et // Cube->World
                     ) noexcept
{
   ARGBImage out = cv_to_argb(image);
   ac.render_dense(
       out, detects, face_ims, cu, cam_et.inverse(), cube_et.inverse());
   return argb_to_cv(out);
}

// ---------------------------------------------------------------- do-opt-n-way
//
struct NWayResult
{
   vector<EuclideanTransform> cam_ets;
   vector<EuclideanTransform> cube_ets;
   real error = dNAN;
};

static NWayResult do_opt_n_way(const ManifestData& mdata,
                               const vector<EuclideanTransform>& in_cam_ets,
                               const vector<EuclideanTransform>& in_cube_ets,
                               const string_view outdir,
                               const bool feedback) noexcept(false) // bad_alloc
{
   const auto now = tick();

   Expects(int(in_cam_ets.size()) == mdata.n_cameras());
   Expects(int(in_cube_ets.size()) == mdata.n_positions());

   const ArucoCube& ac = mdata.ac;

   // We need a cost-function for every cdat
   using cost_fun_type = std::function<real(const EuclideanTransform& et)>;
   auto get_cost_funs  = [&]() {
      vector<cost_fun_type> cost_funs;
      cost_funs.reserve(mdata.data.size());
      std::transform(cbegin(mdata.data),
                     cend(mdata.data),
                     std::back_inserter(cost_funs),
                     [&](const auto& cdat) {
                        const auto& cinfo = mdata.cam_infos.at(cdat.cam_index);
                        return ac.pointwise_error_fun(cinfo.cu, cdat.detects);
                     });
      return cost_funs;
   };
   const auto cost_funs = get_cost_funs(); // one for every mdata.data

   auto calc_err = [&](const auto& cdat,
                       const cost_fun_type& fn,
                       const auto& cam_ets,
                       const auto& cube_ets) -> real {
      const auto& cam_et  = cam_ets.at(cdat.cam_index);
      const auto& cube_et = cube_ets.at(cdat.position_index);
      return fn((cam_et * cube_et).inverse());
   };

   //
   vector<EuclideanTransform> cam_ets  = in_cam_ets;
   vector<EuclideanTransform> cube_ets = in_cube_ets;

   auto pack = [&](real* X) {
      auto pack_one_et = [&](const auto& et) {
         pack_et_6df(et, X);
         X += 6;
      };
      ranges::for_each(cam_ets, pack_one_et);
      ranges::for_each(cube_ets, pack_one_et);
   };

   auto unpack = [&](const real* X) {
      auto unpack_one_et = [&](auto& et) {
         et = unpack_et_6df(X);
         X += 6;
      };
      ranges::for_each(cam_ets, unpack_one_et);
      ranges::for_each(cube_ets, unpack_one_et);
   };

   auto init_step = [&](real* X) {
      auto init_one = [&](const auto& et) {
         for(auto i = 0; i < 3; ++i) *X++ = 0.01;
         for(auto i = 3; i < 6; ++i) *X++ = one_degree();
      };
      ranges::for_each(cam_ets, init_one);
      ranges::for_each(cube_ets, init_one);
   };

   const int n_params = 6 * (mdata.n_cameras() + mdata.n_positions());
   vector<real> start((size_t(n_params)));
   vector<real> step((size_t(n_params)));
   vector<real> best_params((size_t(n_params)));
   init_step(&step[0]);
   pack(&start[0]);

   int counter     = 0;
   real best_score = std::numeric_limits<real>::max();
   auto last_now   = now;
   auto fn         = [&](const real* X) -> real {
      unpack(X);

      real err          = 0.0;
      const real sz_inv = 1.0 / real(cost_funs.size());
      for(const auto&& [cdat, fn] : views::zip(mdata.data, cost_funs))
         err += calc_err(cdat, fn, cam_ets, cube_ets) * sz_inv;

      if(feedback && tock(last_now) > 1.0) {
         last_now = tick();
         cout << format("    #{:5d}  :: {:10.7f}", counter, best_score) << endl;
      }

      const auto ret = err;
      if(ret < best_score) {
         best_score = ret;
         std::copy(X, X + n_params, begin(best_params));
         if(feedback && false) {
            cout << format("   #{:8d} :: {:10.7f}", counter, best_score)
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
      if(!use_nelder_mead) {
         levenberg_marquardt(fn,
                             unsigned(n_params),
                             &start[0],
                             &xmin[0],
                             reqmin,
                             diffstep,
                             10,
                             kcount,
                             icount,
                             ifault);
      } else {
         nelder_mead(fn,
                     unsigned(n_params),
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

   const real ystartlo = fn(&start[0]);
   do_refine(true);
   unpack(&best_params[0]);

   const real elapsed_s = tock(now);
   const real ynewlo    = fn(&best_params[0]);

   if(feedback) { // Print feedback
      auto feedback_string = [&]() {
         std::stringstream ss{""};
         ss << "\n"
            << format("Feedback global cam-cube positioning\n")
            << format("    n-params:             {}\n", n_params)
            << format("    elapsed time:         {}s\n", elapsed_s)
            << format("    initial-score:        {}\n", ystartlo)
            << format("    final-score:          {}\n", ynewlo);
         return ss.str();
      };
      // And output to screen
      sync_write([&]() { cout << indent(feedback_string(), 4); });
   }

   NWayResult ret;
   ret.cam_ets  = std::move(cam_ets);
   ret.cube_ets = std::move(cube_ets);
   ret.error    = ynewlo;
   return ret;
}

// -------------------------------------------------------------- optimize-n-way
//
std::pair<vector<EuclideanTransform>, bool>
optimize_n_way(const string_view scene_id,
               const ManifestData& mdata,
               const string_view outdir)
{
   // Estimate ets that are consistent across the entire scene
   auto get_init_cam_ets = [&]() {
      vector<EuclideanTransform> ets;
      ets.reserve(size_t(mdata.n_cameras()));
      std::transform(cbegin(mdata.cam_infos),
                     cend(mdata.cam_infos),
                     std::back_inserter(ets),
                     std::mem_fn(&CamInfo::init_et));
      return ets;
   };

   auto get_init_cube_ets = [&]() {
      vector<EuclideanTransform> ets;
      ets.reserve(size_t(mdata.n_positions()));
      std::transform(cbegin(mdata.positions),
                     cend(mdata.positions),
                     std::back_inserter(ets),
                     std::mem_fn(&PositionInfo::init_et));
      return ets;
   };

   auto render_all_pointwise = [&](const string_view prefix,
                                   const auto& cam_ets,
                                   const auto& cube_ets) {
      for(const auto& cdat : mdata.data) {
         cv_to_argb(render_pointwise(mdata.ac,
                                     cdat.undistorted,
                                     CachingUndistortInverse(mdata.K),
                                     cdat.detects,
                                     cam_ets.at(cdat.cam_index),
                                     cube_ets.at(cdat.position_index)))
             .save(format("{}/{}_{}.png", outdir, prefix, cdat.name));
      }
   };

   auto render_all_dense = [&](const string_view prefix,
                               const auto& cam_ets,
                               const auto& cube_ets) {
      for(const auto& cdat : mdata.data) {
         ARGBImage argb = cv_to_argb(cdat.undistorted);
         mdata.ac.render_dense(argb,
                               cdat.detects,
                               mdata.face_ims,
                               CachingUndistortInverse(mdata.K),
                               cam_ets.at(cdat.cam_index),
                               cube_ets.at(cdat.position_index));
         argb.save(format("{}/{}_{}.png", outdir, prefix, cdat.name));
      }
   };

   const auto init_cam_ets  = get_init_cam_ets();
   const auto init_cube_ets = get_init_cube_ets();

   render_all_pointwise("010_pointwise", init_cam_ets, init_cube_ets);
   render_all_dense("011_dense", init_cam_ets, init_cube_ets);

   print(g_info, g_default, "performing global optimization");

   const auto ret
       = do_opt_n_way(mdata, init_cam_ets, init_cube_ets, outdir, true);

   print(g_info, g_default, "done");

   render_all_pointwise("020_pointwise-final", ret.cam_ets, ret.cube_ets);
   render_all_dense("021_dense-final", ret.cam_ets, ret.cube_ets);

   // create the scene
   const auto scene_info = make_scene(scene_id, mdata, ret.cam_ets);
   { // save
      const auto fname = format("{}/{}.json", outdir, scene_info.scene_key);

      // Write the output
      string out_s;
      write(scene_info, out_s);
      file_put_contents(fname, out_s);

      print(g_victory,
            g_default,
            format("saving scene-info object to {}", fname));
   }

   return std::make_pair(ret.cam_ets, true);
}

} // namespace perceive::calibration::position_scene_cameras
