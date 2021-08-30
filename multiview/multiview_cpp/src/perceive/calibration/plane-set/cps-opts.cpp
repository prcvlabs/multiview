
#include "cps-opts.hpp"

#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/geometry/projective/distorted-camera.hpp"
#include "perceive/graphics.hpp"
#include "perceive/optimization/golden-section-search.hpp"
#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"
#include "perceive/utils/create-cv-remap.hpp"

#define This PhasePlaneOptData

namespace perceive::calibration
{
// --------------------------------------------- make-bcam-p3-cam-transfer-mapxy

void make_bcam_p3_cam_transfer_mapxy(
    const bool left_to_right,
    const Plane& p3,
    const int w,
    const int h,
    const array<CachingUndistortInverse, 2>& cu,
    const bool use_cu,     // Perform distort/undistort operations
    const Vector3 C[2],    // Two camera centers
    const Quaternion q[2], // Two rotations, from world to CAM0/1
    const BinaryImage& mask,
    ParallelJobSet& pjobs,
    cv::Mat& mapx,
    cv::Mat& mapy)
{
   static const Matrix3r H = Matrix3r::Identity();
   static const Matrix3r K = Matrix3r::Identity();

   const int src_cam = left_to_right ? CAM1 : CAM0;
   const int dst_cam = left_to_right ? CAM0 : CAM1;

   auto plane_ray_intersect = [&](int cam_ind, const Vector2& D) {
      auto U = use_cu ? cu[size_t(cam_ind)].undistort(D) : D;
      return ::perceive::plane_ray_intersect(C[cam_ind], q[cam_ind], p3, U);
   };

   auto project_to_undistorted = [&](int cam_ind, const Vector3& X) -> Vector2 {
      return ::perceive::project_to_undistorted(C[cam_ind], q[cam_ind], X);
   };

   auto f = [&](const Vector2& x) -> Vector2 {
      if(mask.in_bounds(x) && !mask(x)) return Vector2(-1, -1);

      // let's not bother with
      auto X = plane_ray_intersect(src_cam, x);
      auto U = project_to_undistorted(dst_cam, X);
      if(!use_cu) return U;
      Vector2 O{-1.0, -1.0};
      if(cu[size_t(dst_cam)].in_bounds(U)) O = cu[size_t(dst_cam)].distort(U);
      // else
      //    O = cu[dst_cam].M().distort(U);

      if(false && (x == Vector2(566, 522))) {
         INFO(format("x = {}", str(x)));
         INFO(format("X = {}", str(X)));
         INFO(format("U = {}", str(U)));
         INFO(format("O = {}", str(O)));
         INFO(format("cu= {}", str(cu[size_t(dst_cam)].working_format())));
         INFO("");
      }
      return O;
   };

   create_cv_remap_threaded(
       unsigned(w), unsigned(h), H, f, K, mapx, mapy, pjobs, false);
}

// ----------------------------------------------------------- image-match-score
//
real image_match_score(const cv::Mat& ref,
                       const cv::Mat& src,
                       const BinaryImage& mask,
                       GreyImage* g_ptr)
{
   Expects(ref.cols == src.cols);
   Expects(ref.rows == src.rows);
   Expects(mask.width == unsigned(ref.cols));
   Expects(mask.height == unsigned(ref.rows));
   const auto w = ref.cols;
   const auto h = ref.rows;

   if(g_ptr) {
      g_ptr->resize(w, h);
      g_ptr->fill(127);
   }

   auto ret = 0.0;

   const auto black = cv::Vec3b(0, 0, 0);

   int counter = 0;
   for(auto y = 0; y < h; ++y) {
      auto src_ptr = src.ptr<cv::Vec3b>(y);
      auto ref_ptr = ref.ptr<cv::Vec3b>(y);
      for(auto x = 0; x < w; ++x) {
         if(!mask(x, y)) continue;

         auto& X = ref_ptr[x];
         auto& Y = src_ptr[x];

         if(Y == black) continue;

         // skip neutral areas
         auto match = cie2000_score(X, Y);
         ret += match;

         if(g_ptr) (*g_ptr)(x, y) = uint8_t(255.0 * (1.0 - match));

         ++counter;
      }
   }

   return (counter == 0) ? 0.0 : ret / real(counter);
}

// -------------------------------------------------------------- make-slic-mask

BinaryImage make_slic_mask(const IntImage& labels,
                           const std::vector<int>& selected) noexcept
{
   BinaryImage out;
   const auto w = labels.width;
   const auto h = labels.height;
   out.resize(w, h);
   out.fill(false);

   // Empty selected?
   if(selected.size() == 0) { return out; }

   // Make a lookup for selected
   const int max_val = *std::max_element(begin(selected), end(selected));
   vector<bool> lookup((size_t(max_val + 1)));
   std::fill(begin(lookup), end(lookup), false);
   for(const auto ind : selected) lookup[size_t(ind)] = true;

   for(auto y = 0u; y < h; ++y) {
      const auto row_ptr = labels.row_ptr(y);
      for(auto x = 0u; x < w; ++x) {
         auto label = row_ptr[x];
         if(label <= max_val && lookup[size_t(label)] == true) out(x, y) = true;
      }
   }

   return out;
}

// ----------------------------------------------------------- phase-plane error

// struct WorkingData
// {
//    ParallelJobSet pjobs;
//    array<cv::Mat, 2> mapx, mapy;
//    array<cv::Mat, 2> dst;
// };

// static real
// phase_plane_error(const BinocularCameraInfo& bcam_info,
//                   const EuclideanTransform& bcam_et,
//                   const array<CachingUndistortInverse, 2>& cu,
//                   const int w,
//                   const int h,
//                   const array<BinaryImage, 2> masks, // make_slic_mask
//                   const array<cv::Mat, 2> cv_im,     // SLIC input image
//                   const Plane& p3,                        // plane of
//                   interest WorkingData& working_data) noexcept
// {
//    Vector3 C[2];
//    Quaternion q[2];

//    const auto et0 = bcam_et;
//    const auto et1 = bcam_info.make_et1(et0);

//    std::tie(C[0], q[0]) = make_camera_extrinsics(et0);
//    std::tie(C[1], q[1]) = make_camera_extrinsics(et1);

//    for(auto i = 0; i < 2; ++i) {
//       if(cu[i].working_format() != Vector2(w, h))
//          FATAL(format("working format = {}, but w = {}, h = {}",
//                       str(cu[i].working_format()),
//                       w,
//                       h));
//    }

//    auto& pjobs = working_data.pjobs;
//    auto& mapx  = working_data.mapx;
//    auto& mapy  = working_data.mapy;
//    auto& dst   = working_data.dst;

//    make_bcam_p3_cam_transfer_mapxy(
//        true, p3, w, h, cu, true, C, q, masks[1], pjobs, mapx[0], mapy[0]);
//    make_bcam_p3_cam_transfer_mapxy(
//        false, p3, w, h, cu, true, C, q, masks[0], pjobs, mapx[1], mapy[1]);

//    cv::remap(cv_im[0], dst[1], mapx[0], mapy[0], cv::INTER_LINEAR);
//    cv::remap(cv_im[1], dst[0], mapx[1], mapy[1], cv::INTER_LINEAR);

//    const auto err0 = image_match_score(cv_im[0], dst[0], masks[0], nullptr);
//    const auto err1 = image_match_score(cv_im[1], dst[1], masks[1], nullptr);
//    const auto err  = 0.5 * (err0 + err1);

//    return err;
// }

// ------------------------------------------------------ make-phase-plane-image

ARGBImage make_phase_plane_image(
    const BinocularCameraInfo& bcam_info,
    const EuclideanTransform& bcam_et,
    const array<CachingUndistortInverse, 2>& cu,
    const vector<const SlicData*>& slic,     // sensor images to convolve
    const Plane& p3,                         // plane of interest
    const vector<std::vector<int>>& selected // slic indices, one per sensor
    ) noexcept
{
   ARGBImage out;
   ParallelJobSet pjobs;
   array<cv::Mat, 2> mapx, mapy;
   array<cv::Mat, 2> cv_im, dst;
   array<GreyImage, 2> gs;
   array<BinaryImage, 2> masks; // based on selected images
   Vector3 C[2];
   Quaternion q[2];
   const string outdir = "/tmp"s;

   for(size_t i = 0; i < 2; ++i) cv_im[i] = argb_to_cv(slic[i]->input_image);

   for(size_t i = 0; i < 2; ++i)
      masks[i] = make_slic_mask(slic[i]->labels, selected[i]);

   const auto et0 = bcam_et;
   const auto et1 = bcam_info.make_et1(et0);

   std::tie(C[0], q[0]) = make_camera_extrinsics(et0);
   std::tie(C[1], q[1]) = make_camera_extrinsics(et1);

   const int w = int(slic[0]->input_image.width);
   const int h = int(slic[0]->input_image.height);
   for(size_t i = 0; i < 2; ++i) {
      if(cu[i].working_format() != Vector2(w, h))
         FATAL(format("working format = {}, but w = {}, h = {}",
                      str(cu[i].working_format()),
                      w,
                      h));
   }

   make_bcam_p3_cam_transfer_mapxy(
       true, p3, w, h, cu, true, C, q, masks[1], pjobs, mapx[0], mapy[0]);
   make_bcam_p3_cam_transfer_mapxy(
       false, p3, w, h, cu, true, C, q, masks[0], pjobs, mapx[1], mapy[1]);

   cv::remap(cv_im[0], dst[1], mapx[0], mapy[0], cv::INTER_LINEAR);
   cv::remap(cv_im[1], dst[0], mapx[1], mapy[1], cv::INTER_LINEAR);

   const auto err0 = image_match_score(cv_im[0], dst[0], masks[0], &gs[0]);
   const auto err1 = image_match_score(cv_im[1], dst[1], masks[1], &gs[1]);
   const auto err  = 0.5 * (err0 + err1);

   INFO(format("err = {}", err));

   GreyImage gg = hcat(gs[0], gs[1]);
   out          = grey_to_argb(gg);

   if(false) {
      INFO(format("p3 = {}", str(p3)));
      for(auto i = 0; i < 2; ++i) {
         INFO(format("C = {}, q = {}", str(C[i]), q[i].to_readable_str()));
      }
      for(size_t i = 0; i < 2; ++i)
         gs[i].save(format("{}/zzz{}.png", outdir, i));
      for(size_t i = 0; i < 2; ++i) {
         cv::imwrite(format("{}/xxx{}.png", outdir, i), cv_im[i]);
         cv::imwrite(format("{}/yyy{}.png", outdir, i), dst[i]);
      }
      for(size_t i = 0; i < 2; ++i)
         binary_im_to_grey(masks[i]).save(format("{}/grey{}.png", outdir, i));

      if(false) {
         for(size_t j = 0; j < 2; ++j) {
            cout << format("SELECTED[{}]", j) << endl;
            cout << format("   [{}]",
                           implode(begin(selected[j]), end(selected[j]), ", "))
                 << endl;
            cout << endl;
         }
      }
   }

   return out;
}

// ----------------------------------------------------- PhasePlaneOptData::init

void This::init(const SceneDescription& scene_desc,
                const PhasePlaneData& data) noexcept
{
   // -- (*) -- Setup parameters
   const int n_cams = std::accumulate(
       cbegin(data.s_infos),
       cend(data.s_infos),
       0,
       [&](int val, const auto& x) { return val + (x.do_optimize ? 1 : 0); });

   const int n_p3s = std::accumulate(
       cbegin(data.p3s), cend(data.p3s), 0, [&](int val, const auto& x) {
          return val + (x.do_optimize ? 1 : 0);
       });

   auto make_bcam_lookup = [&]() -> std::unordered_map<string, int> {
      std::unordered_map<string, int> out;
      for(size_t i = 0; i < scene_desc.bcam_infos.size(); ++i)
         out[scene_desc.bcam_infos[i].camera_id] = int(i);
      return out;
   };
   const auto bcam_lookup = make_bcam_lookup();

   scene_key = scene_desc.scene_info.scene_key;

   vector<int> p3_lookup; // such that p3s[i] == 'data.p3s[p3_lookup[i]]'

   { // planes
      p3s.reserve(size_t(n_p3s));
      p3_lookup.reserve(size_t(n_p3s));
      for(size_t i = 0; i < data.p3s.size(); ++i) {
         const auto& pinfo = data.p3s[i];
         if(pinfo.do_optimize) {
            p3_lookup.push_back(int(i));
            p3s.push_back(pinfo);
         }
      }
   }

   { // cams
      const size_t sz_n_cams = size_t(n_cams);

      et.resize(sz_n_cams);
      bcam_infos.resize(sz_n_cams);
      cus.resize(sz_n_cams);
      stills.resize(sz_n_cams);
      argb_stills.resize(sz_n_cams);
      slic_data.resize(sz_n_cams);
      cv_ims.resize(sz_n_cams);
      cv_dsts.resize(sz_n_cams);
      mapxs.resize(sz_n_cams);
      mapys.resize(sz_n_cams);

      size_t cam_ind = 0;
      for(const auto& s_info : data.s_infos) {
         if(!s_info.do_optimize) continue;
         Expects(cam_ind < sz_n_cams);
         auto ii = bcam_lookup.find(s_info.camera_key);
         if(ii == cend(bcam_lookup))
            FATAL(format("precondition failed: failed to find camera '{}' in "
                         "scene '{}'",
                         s_info.camera_key,
                         scene_desc.scene_info.scene_key));
         Expects(unsigned(ii->second) < scene_desc.cam_transforms.size());
         Expects(unsigned(ii->second) < scene_desc.bcam_infos.size());
         Expects(scene_desc.n_sensors_for(ii->second) == 2);
         array<int, 2> sensor_inds;
         for(size_t j = 0; j < 2; ++j)
            sensor_inds[j] = scene_desc.sensor_lookup(ii->second, int(j));

         et[cam_ind]         = scene_desc.cam_transforms[size_t(ii->second)];
         bcam_infos[cam_ind] = &scene_desc.bcam_infos[size_t(ii->second)];

         hsplit(scene_desc.LAB_still(ii->second),
                stills[cam_ind][0],
                stills[cam_ind][1]);
         hsplit(scene_desc.argb_still(ii->second),
                argb_stills[cam_ind][0],
                argb_stills[cam_ind][1]);

         // Expects(unsigned(ii->second) < slic_datas.size());
         for(size_t j = 0; j < 2; ++j) {
            // INFO(format(
            //     "slic-data {}, {}", s_info.slic_size,
            //     s_info.slic_compactness));
            // argb_stills[cam_ind][j].save(
            //     format("/tmp/aaa_{}x{}.png", cam_ind, j));
            slic_data[cam_ind][j].init(argb_stills[cam_ind][j],
                                       s_info.slic_size,
                                       s_info.slic_compactness,
                                       []() { return false; });
         }

         for(size_t j = 0; j < 2; ++j)
            cv_ims[cam_ind][j] = argb_to_cv(argb_stills[cam_ind][j]);

         for(size_t j = 0; j < 2; ++j) {
            const auto sensor_ind = sensor_inds[j];
            cus[cam_ind][j].init(
                scene_desc.bcam_infos[size_t(ii->second)].M[j]);
            cus[cam_ind][j].set_working_format(stills[cam_ind][j].width,
                                               stills[cam_ind][j].height);
         }

         cam_ind++;
      }
      Expects(cam_ind == sz_n_cams);
   }

   { // Masks
      const size_t sz_n_cams = size_t(n_cams);
      const size_t sz_n_p3s  = size_t(n_p3s);
      masks.resize(sz_n_p3s);
      mask_counters.resize(sz_n_p3s);
      for(size_t i = 0; i < sz_n_p3s; ++i) {
         const auto& p3 = p3s[i];
         masks[i].resize(sz_n_cams);
         mask_counters[i].resize(sz_n_cams);
         for(size_t j = 0; j < sz_n_cams; ++j) {
            // what is the actual cam index?
            auto ii = bcam_lookup.find(bcam_infos[j]->camera_id);
            Expects(ii != cend(bcam_lookup));

            for(size_t k = 0; k < 2; ++k) {
               auto sensor_id = scene_desc.sensor_lookup(ii->second, int(k));

               std::vector<int> selected;
               if(unsigned(sensor_id) < p3.selected.size())
                  selected.insert(end(selected),
                                  begin(p3.selected[size_t(sensor_id)]),
                                  end(p3.selected[size_t(sensor_id)]));

               masks[i][j][k]
                   = make_slic_mask(slic_data[j][k].labels, selected);

               mask_counters[i][j][k] = 0;
               for(auto y = 0u; y < masks[i][j][k].height; ++y)
                  for(auto x = 0u; x < masks[i][j][k].width; ++x)
                     if(masks[i][j][k](x, y)) mask_counters[i][j][k]++;

               // binary_im_to_grey(masks[i][j][k])
               //     .save(format("/tmp/grey_{}x{}x{}.png", i, j, k));
            }
         }
      }
   }

   { // Combine all the masks together
      const size_t sz_n_cams = size_t(n_cams);
      const size_t sz_n_p3s  = size_t(n_p3s);
      all_masks.resize(sz_n_cams);
      for(size_t i = 0; i < sz_n_cams; ++i) {
         for(size_t k = 0; k < 2; ++k) {
            IntImage& im = all_masks[i][k];
            const int w  = int(argb_stills[i][k].width);
            const int h  = int(argb_stills[i][k].height);

            //
            im.resize(w, h);
            im.fill(-1);

            for(auto y = 0; y < h; ++y) {
               for(auto x = 0; x < w; ++x) {
                  for(size_t p = 0; p < sz_n_p3s; ++p) {
                     const auto& mask = masks[p][i][k];
                     Expects(mask.in_bounds(x, y));
                     if(mask(x, y)) im(x, y) = int32_t(p);
                  }
               }
            }
         }
      }
   }

   { // Sensor Lookup
      for(auto i = 0u; i < bcam_infos.size(); ++i) {
         const auto& bcam_info = *bcam_infos[i];
         for(auto j = 0; j < bcam_info.n_sensors(); ++j)
            this->sensor_lookup[bcam_info.M[size_t(j)].sensor_id()]
                = Point2(int(i), int(j));
      }
   }

   { // ray-plane-plane-info
      // vector<int> p3_lookup; // such that p3s[i] == 'data.p3s[p3_lookup[i]]'

      auto get_sensor = [&](const string& sensor_key) -> Point2 {
         auto ii = sensor_lookup.find(sensor_key);
         return ii == cend(sensor_lookup) ? Point2(-1, -1) : ii->second;
      };

      auto get_p3 = [&](const int ind) -> int {
         if(unsigned(ind) >= data.p3s.size()) return -1;
         auto ii = std::find_if(cbegin(p3_lookup),
                                cend(p3_lookup),
                                [&](auto x) { return x == ind; });
         if(ii == cend(p3_lookup)) return -1;
         return int(std::distance(cbegin(p3_lookup), ii));
      };

      for(const auto& rp : data.rp_infos) {
         const auto xy   = get_sensor(rp.sensor_key);
         const auto ind0 = get_p3(rp.inds[0]);
         const auto ind1 = get_p3(rp.inds[1]);
         const auto ind2 = get_p3(rp.inds[2]);

         if(ind0 < 0 or ind1 < 0 or xy.x < 0 or xy.y < 0) continue;
         if(rp.is_point() and ind2 < 0) continue;

         // Sanity checks
         Expects(unsigned(xy.x) < bcam_infos.size());
         Expects(unsigned(xy.y)
                 < unsigned(bcam_infos[size_t(xy.x)]->n_sensors()));
         Expects(bcam_infos[size_t(xy.x)]->M[size_t(xy.y)].sensor_id()
                 == rp.sensor_key);
         Expects(unsigned(rp.inds[0]) < data.p3s.size());
         Expects(unsigned(rp.inds[1]) < data.p3s.size());
         if(rp.is_point()) Expects(unsigned(rp.inds[2]) < data.p3s.size());
         Expects(unsigned(ind0) < p3s.size());
         Expects(unsigned(ind1) < p3s.size());
         if(rp.is_point()) Expects(unsigned(ind2) < p3s.size());
         const auto& dp0 = data.p3s[size_t(rp.inds[0])];
         const auto& dp1 = data.p3s[size_t(rp.inds[1])];
         const auto& tp0 = p3s[size_t(ind0)];
         const auto& tp1 = p3s[size_t(ind1)];
         Expects(dp0.name == tp0.name);
         Expects(dp1.name == tp1.name);
         if(rp.is_point())
            Expects(data.p3s[size_t(rp.inds[2])].name
                    == p3s[size_t(ind2)].name);

         // Okay, we're good
         rpp_bcam_sensor.push_back(xy);
         rpps.push_back(rp);
         rpps.back().inds[0] = ind0;
         rpps.back().inds[1] = ind1;
         rpps.back().inds[2] = (rp.is_point()) ? ind2 : -1;
      }
   }
}

// ----------------------------------------------------------------------- error

real This::error(const string& outdir, const bool feedback) noexcept
{
   real final_error = 0.0;
   int err_counter  = 0;
   if(n_cams() <= 0) return final_error;

   Vector3 C[2];
   Quaternion q[2];
   array<GreyImage, 2> gs;
   array<GreyImage*, 2> g_ptr;
   std::fill(begin(g_ptr), end(g_ptr), nullptr);
   if(feedback)
      for(size_t j = 0; j < 2; ++j) g_ptr[j] = &gs[j];

   const int w = int(argb_stills[0][0].width);
   const int h = int(argb_stills[0][0].height);
   // const real n_cams_inv = 1.0 / real(n_cams());
   // const real n_p3s_inv = 1.0 / real(n_cams());

   vector<Plane> p3vs(p3s.size());
   for(size_t i = 0; i < p3vs.size(); ++i) p3vs[i] = p3s[i].p3();

   Expects(rpps.size() == rpp_bcam_sensor.size());
   for(auto rpp_ind = 0u; rpp_ind < rpps.size(); ++rpp_ind) {
      const auto& rp     = rpps[rpp_ind];
      const auto cam_ind = rpp_bcam_sensor[rpp_ind].x;
      const auto sn      = rpp_bcam_sensor[rpp_ind].y;
      const auto& cu     = cus[size_t(cam_ind)];

      auto get_et = [&]() {
         Expects(sn >= 0 and sn < 2);
         const auto& et0 = et[size_t(cam_ind)];
         return sn == 0 ? et0 : bcam_infos[size_t(cam_ind)]->make_et1(et0);
      };
      const auto& et         = get_et();
      std::tie(C[sn], q[sn]) = make_camera_extrinsics(et);

      auto rp_reproj_err = 0.0;
      if(rp.is_point()) {
         const auto& U = p3vs[size_t(rp.inds[0])];
         const auto& V = p3vs[size_t(rp.inds[1])];
         const auto& T = p3vs[size_t(rp.inds[2])];
         const auto W  = intersection_of_3_planes(U, V, T); // world point
         const auto X  = et.inverse_apply(W);
         const auto x  = cu[size_t(sn)].distort(homgen_P2_to_R2(X));
         rp_reproj_err = 2.0 * (rp.x - x).norm();
         err_counter += 2;

         if(false && rp.inds[0] == 12 && rp.inds[1] == 13 && rp.inds[2] == 14) {
            cout << format("*** REPORT *** {}", rp.sensor_key) << endl;
            cout << format("W {} -> X {}", str(W), str(X)) << endl;
            cout << format("|x - x0| = |{} - {}| = {}",
                           str(x),
                           str(rp.x),
                           (x - rp.x).norm())
                 << endl;
            cout << endl;
         }
      } else {
         const auto& U    = p3vs[size_t(rp.inds[0])];
         const auto& V    = p3vs[size_t(rp.inds[1])];
         const Vector2& u = rp.x;
         const Vector3 X
             = plane_ray_intersect(C[sn], q[sn], cu[size_t(sn)], U, u);
         const Vector3 Y = V.image(X);
         const Vector2 v
             = project_to_distorted(C[sn], q[sn], cu[size_t(sn)], Y);
         rp_reproj_err = (u - v).norm();

         err_counter += 1;
      }

      final_error += rp_reproj_err;
   }

   for(auto cam = 0; cam < n_cams(); ++cam) {
      const auto& et0      = et[size_t(cam)];
      const auto et1       = bcam_infos[size_t(cam)]->make_et1(et0);
      std::tie(C[0], q[0]) = make_camera_extrinsics(et0);
      std::tie(C[1], q[1]) = make_camera_extrinsics(et1);

      const auto& cu    = cus[size_t(cam)];
      auto& mapx        = mapxs[size_t(cam)];
      auto& mapy        = mapys[size_t(cam)];
      auto& dst         = cv_dsts[size_t(cam)];
      const auto& cv_im = cv_ims[size_t(cam)];

      for(auto p3_ind = 0; p3_ind < n_p3s(); ++p3_ind) {
         const auto& mask         = masks[size_t(p3_ind)][size_t(cam)];
         const auto& mask_counter = mask_counters[size_t(p3_ind)][size_t(cam)];
         if(mask_counter[0] == 0 or mask_counter[1] == 0) continue;

         const auto& p3 = p3vs[size_t(p3_ind)];
         make_bcam_p3_cam_transfer_mapxy(
             true, p3, w, h, cu, true, C, q, mask[1], pjobs, mapx[0], mapy[0]);
         make_bcam_p3_cam_transfer_mapxy(
             false, p3, w, h, cu, true, C, q, mask[0], pjobs, mapx[1], mapy[1]);

         cv::remap(cv_im[0], dst[1], mapx[0], mapy[0], cv::INTER_LINEAR);
         cv::remap(cv_im[1], dst[0], mapx[1], mapy[1], cv::INTER_LINEAR);

         const real err0
             = image_match_score(cv_im[0], dst[0], mask[0], g_ptr[0]);
         const real err1
             = image_match_score(cv_im[1], dst[1], mask[1], g_ptr[1]);
         const real err = 0.5 * (err0 + err1);
         if(err > 0.0) {
            final_error += image_match_score_weight * err;
            err_counter++;
         }

         // final_error += 0.5 * (err0 + err1) * n_cams_inv;

         if(feedback) {
            const string& camera_id = bcam_infos[size_t(cam)]->camera_id;
            const string& plane_id  = p3s[size_t(p3_ind)].name;

            GreyImage g0 = hcat(gs[0], gs[1]);
            ARGBImage f0 = grey_to_colour(g0);
            if(false) {
               GreyImage m0 = hcat(binary_im_to_grey(mask[0]),
                                   binary_im_to_grey(mask[1]));

               // ARGBImage a0 = hcat(cv_to_argb(cv_im[0]),
               // cv_to_argb(cv_im[1]));
               ARGBImage a1 = hcat(cv_to_argb(dst[0]), cv_to_argb(dst[1]));

               ARGBImage gm = grey_to_argb(vcat(g0, m0));
               // ARGBImage fm = vcat(a0, a1);

               ARGBImage f0 = vcat(gm, a1);
            }

            f0.save(format(
                "{}/zz-score-pe_{}-{}.png", outdir, camera_id, plane_id));

            // for(auto j = 0; j < 2; ++j) {
            //    auto make_fname = [&](const char* prefix) {
            //       return format("{}/{}-pe_{}_{}-{}.png",
            //                     outdir,
            //                     prefix,
            //                     camera_id,
            //                     plane_id,
            //                     (j == 0 ? "l" : "r"));
            //    };

            //    g_ptr[j]->save(make_fname("zz-score"));
            //    // binary_im_to_grey(mask[j]).save(make_fname("zz-mask"));
            //    // cv::imwrite(make_fname("zz-cv_im"), cv_im[j]);
            //    // cv::imwrite(make_fname("zz-dst"), dst[j]);
            // }
         }
      }
   }

   return (err_counter == 0) ? 0.0 : (final_error / real(err_counter));
}

cv::Mat This::make_ref_disparity_map(const int cam_no,
                                     const string& outdir) noexcept
{
   Expects(unsigned(cam_no) < bcam_infos.size());

   const int w0          = int(argb_stills[size_t(cam_no)][0].width);
   const int h0          = int(argb_stills[size_t(cam_no)][0].height);
   const auto& bcam_info = *bcam_infos[size_t(cam_no)];
   const auto& cam_id    = bcam_info.camera_id;
   const auto& all_mask  = all_masks[size_t(cam_no)];

   // INFO(format("make {}", bcam_info.camera_id));

   const auto& et0 = et[size_t(cam_no)];

   BinocularCamera bcam;
   const auto w     = 800;
   const auto h     = 600;
   const Matrix3r K = Matrix3r::Identity();
   cv::Mat mapx[2];
   cv::Mat mapy[2];
   cv::Mat undistorted[2];

   DistortedCamera cam0, cam1;
   std::tie(cam0, cam1)
       = make_distorted_camera_pair(bcam_info, et0, unsigned(w0), unsigned(h0));
   bcam.init(bcam_info, unsigned(w0), unsigned(h0), K, w, h, true);
   bcam.set_working_format(unsigned(w0), unsigned(h0));

   bcam.get_mapxy(mapx, mapy);

   const bool debug_on = false;
   const Vector2 D0    = Vector2(200, 364);
   if(debug_on) {
      const auto ray0  = bcam.to_ray(CAM0, D0);
      const Vector2 R0 = bcam.project_to_rectified(CAM0, ray0);
      WARN(format("debug lines below"));
      // cout << format("cam= {}", bcam.
      cout << format("D0 = {}", str(D0)) << endl;
      cout << format("r0 = {}", str(ray0)) << endl;
      cout << format("R0 = {}", str(R0)) << endl;
      cout << format("n-p3s = {}/{}", n_p3s(), p3s.size()) << endl;

      for(auto p3_ind = 0u; p3_ind < p3s.size(); ++p3_ind) {
         const auto& mask = masks[p3_ind][size_t(cam_no)][0];
         binary_im_to_grey(mask).save(
             format("/tmp/mask_{}.png", p3s[p3_ind].name));
      }

      int_image_to_argb(all_mask[0])
          .save(format("/tmp/mask_{}_argb-l.png", cam_id));
      int_image_to_argb(all_mask[1])
          .save(format("/tmp/mask_{}_argb-r.png", cam_id));
   }

   // Create disparity map
   cv::Mat disp(h, w, CV_32FC1);

   auto process_row = [&](int y) {
      float* row = disp.ptr<float>(y);

      for(auto x = 0; x < w; ++x) {
         row[x] = std::numeric_limits<float>::quiet_NaN();

         // Rectified to distorted. (checked.)
         const auto D = to_vec2(
             Vector2f(mapx[0].at<float>(y, x), mapy[0].at<float>(y, x)));

         const bool it_is = debug_on and ((D - D0).norm() <= 1.1);

         // What plane are we talking about?
         auto find_p3 = [&](const Vector2& D) {
            if(!all_mask[0].in_bounds(D)) return Plane::nan();
            const auto p3_ind = all_mask[0](D);
            const auto p3
                = (p3_ind == -1) ? Plane::nan() : p3s[size_t(p3_ind)].p3();
            return p3;
         };
         const Plane p3 = find_p3(D);
         if(!p3.is_finite()) continue;

         // 3D point, world coordinates
         const auto W = plane_ray_intersect(cam0, p3, D);

         // 3D point, CAM0 coordinates
         const auto X = et0.inverse_apply(W);

         auto f = [&](real dx) -> real {
            return (X - bcam.solve3d(x, x - dx, y)).quadrance();
         };

         // Disparity
         // We want x1 such that X == bcam.solve3d(x, x1, y);
         const auto dx = golden_section_search(f, 0.001, 200.0, 1e-6);

         if(it_is) {
            // How did we go?
            const auto Y = bcam.solve3d(x, x - dx, y);
            INFO(format("|{} - {}| = {}", str(X), str(Y), (X - Y).norm()));

            cout << format("cam  = {}", bcam_info.camera_id) << endl;
            cout << format("et0  = {}", et0.to_json_str()) << endl;
            cout << format("[dx] = {}", dx) << endl;
            cout << format("[d]  = {}x{}", w0, h0) << endl;
            cout << format("[r]  = {}x{}", w, h) << endl;
            cout << format("R    = ({}, {})", x, y) << endl;
            cout << format("D    = {}", str(D)) << endl;
            cout << format("p3ind= {}", all_mask[0](x, y)) << endl;
            cout << format("p3   = {}", str(p3)) << endl;
            cout << format("W    = {}", str(W)) << endl;
            cout << format("X    = {}", str(X)) << endl;
            cout << format("p3.X = {}", p3.side(W)) << endl;
            cout << format("P(D) = {}", str(bcam.project(CAM0, X))) << endl;
            cout << format("P(R) = {}", str(bcam.project_to_rectified(CAM0, X)))
                 << endl;
            FATAL("kBAM!");
         }

         Expects((D - bcam.project(CAM0, X)).norm() < 1e-3);
         Expects((Vector2(x, y) - bcam.project_to_rectified(CAM0, X)).norm()
                 < 1e-3);

         row[x] = float(dx);
      }
   };

   for(auto y = 0; y < h; ++y)
      pjobs.schedule([&process_row, y]() { process_row(y); });
   pjobs.execute();

   auto make_disp_image = [&](const cv::Mat& disp) {
      cv::Mat disp_image = cv::Mat(h, w, CV_8UC1);
      double minVal;
      double maxVal;
      minMaxLoc(disp_image, &minVal, &maxVal);
      minVal = 0.0;
      maxVal = 100.0;
      disp.convertTo(disp_image, CV_8UC1, 255.0 / (maxVal - minVal));
      return disp_image;
   };
   cv::imwrite(format("{}/v_{}_disp-map.png", outdir, bcam_info.camera_id),
               make_disp_image(disp));

   FloatImage fim;
   cv_to_float_im(disp, fim);
   fim.save(format("{}/w_disp-float-image.data", outdir));

   string key = format("{}_{}",
                       strip_suffix(scene_key, '_'),
                       strip_suffix(bcam_info.camera_id, '_'));
   store(fim, key);
   INFO(format("stored reference disparity at: {}", key));

   return disp;
}

// ------------------------------------------------------------ optimize-cameras

real optimize_cameras(const SceneDescription& scene_desc,
                      const PhasePlaneData& phase_plane_data,
                      const vector<array<SlicData*, 2>>& slic_datas,
                      const bool use_nelder_mead,
                      vector<EuclideanTransform>& et_out,
                      vector<string>& cam_ids_out,
                      const string& outdir,
                      const bool feedback) noexcept
{
   PhasePlaneOptData data;
   data.init(scene_desc, phase_plane_data);
   if(et_out.size() > 0) {
      if(et_out.size() != data.et.size())
         FATAL(format("Expected {} euclidean-transforms, but got {}!",
                      data.n_cams(),
                      et_out.size()));
      data.et = et_out;
   }

   if(feedback) {
      // -- (*) -- Some feedback
      INFO("optimize cameras:");
      cout << format("   Cameras ({})", data.n_cams()) << endl;
      for(const auto& bcam_info_ptr : data.bcam_infos)
         cout << format("      {}", bcam_info_ptr->camera_id) << endl;
      cout << format("   Planes  ({})", data.n_p3s()) << endl;
      for(const auto& pinfo : data.p3s)
         cout << format("      {} = {}", pinfo.name, str(pinfo.p3())) << endl;

      auto err = data.error(outdir, true); // output files
      cout << format("   Initial error = {}", err) << endl;
      // exit(0);
   }

   const size_t n_params = 6 * size_t(data.n_cams());
   const auto err        = data.error("/tmp", true);

   auto pack = [&](real* X) {
      Vector3 saa;
      for(const auto& et : data.et) {
         saa = quaternion_to_saa(et.rotation);
         for(auto i = 0; i < 3; ++i) *X++ = et.translation[i];
         for(auto i = 0; i < 3; ++i) *X++ = saa[i];
      }
   };

   auto unpack = [&](const real* X) {
      Vector3 saa;
      for(auto& et : data.et) {
         for(auto i = 0; i < 3; ++i) et.translation[i] = *X++;
         for(auto i = 0; i < 3; ++i) saa[i] = *X++;
         et.rotation = saa_to_quaternion(saa);
      }
   };

   vector<real> xbest(n_params);
   auto counter  = 0;
   auto best_err = std::numeric_limits<real>::max();
   auto fn       = [&](const real* X) {
      unpack(X);
      const auto err = data.error();
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
                          unsigned(n_params),
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
      auto simplex_factor = 0.05;
      for(size_t i = 0; i < n_params; ++i) step[i] = one_degree();
      nelder_mead(fn,
                  unsigned(n_params),
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

   unpack(&xbest[0]);
   et_out = data.et;
   cam_ids_out.clear();
   for(const auto bcam_ptr : data.bcam_infos)
      cam_ids_out.push_back(bcam_ptr->camera_id);
   Expects(et_out.size() == cam_ids_out.size());

   if(feedback) {
      ynewlo = data.error(outdir, true); // output files

      INFO(format("Feedback for optimizing {} cameras ({} params)",
                  data.n_cams(),
                  n_params));
      cout << format("   iterations:           {}", icount) << endl;
      cout << format("   fault-code:           {}", ifault) << endl;
      auto msg = (use_nelder_mead) ? nelder_mead_fault_str(ifault)
                                   : levenberg_marquardt_fault_str(ifault);
      cout << format("   fault-string:         {}", msg) << endl;
      cout << endl;
      cout << format("   initial-score:        {}", ystartlo) << endl;
      cout << format("   final-score:          {}", ynewlo) << endl;
      cout << endl;

      for(size_t i = 0; i < size_t(data.n_cams()); ++i) {
         const auto et0    = data.et[i];
         const auto [C, q] = make_camera_extrinsics(et0);
         cout << format("    {}, C={}, q={}",
                        data.bcam_infos[i]->camera_id,
                        str(C),
                        q.to_readable_str())
              << endl;
      }

      cout << format("cost-function images saved to: {}", outdir) << endl;

      cout << "." << endl << endl;
   }

   return ynewlo;
}

real optimize_p3s(const SceneDescription& scene_desc,
                  const PhasePlaneData& phase_plane_data,
                  const vector<array<SlicData*, 2>>& slic_datas,
                  const bool use_nelder_mead, // true is a good bet
                  vector<EuclideanTransform>& et_out,
                  vector<string>& cam_ids_out,
                  PhasePlaneData& opt_out,
                  const string& outdir,
                  const bool feedback) noexcept
{
   PhasePlaneOptData data;
   data.init(scene_desc, phase_plane_data);
   if(et_out.size() > 0) {
      if(et_out.size() != data.et.size())
         FATAL(format("Expected {} euclidean-transforms, but got {}!",
                      data.n_cams(),
                      et_out.size()));
      data.et = et_out;
   }

   if(feedback) {
      // -- (*) -- Some feedback
      INFO("optimize p3s:");
      cout << format("   Cameras ({})", data.n_cams()) << endl;
      for(const auto& bcam_info_ptr : data.bcam_infos)
         cout << format("      {}", bcam_info_ptr->camera_id) << endl;
      cout << format("   Planes  ({})", data.n_p3s()) << endl;
      for(const auto& pinfo : data.p3s)
         cout << format("      {} = {}", pinfo.name, str(pinfo.p3())) << endl;
      cout << format("   Points ({})", data.rpps.size()) << endl;
      for(const auto& rp : data.rpps)
         cout << format("      [{}, {}, {}, {}, {}]\n",
                        rp.sensor_key,
                        rp.inds[0],
                        rp.inds[1],
                        rp.x.x,
                        rp.x.y);

      auto err = data.error(outdir, true); // output files
      cout << format("   Initial error = {}", err) << endl;
   }

   Expects(data.n_p3s() > 0);
   const auto n_params = size_t(data.n_p3s() - 1);
   const auto err      = data.error("/tmp", true);

   auto pack = [&](real* X) {
      for(int i = 1; i < data.n_p3s(); ++i) *X++ = data.p3s[size_t(i)].d;
   };

   auto unpack = [&](const real* X) {
      for(int i = 1; i < data.n_p3s(); ++i) data.p3s[size_t(i)].d = *X++;
   };

   const char* method = use_nelder_mead ? "nelder-mead" : "levenberg-marquardt";
   vector<real> xbest(n_params);
   auto counter  = 0;
   auto best_err = std::numeric_limits<real>::max();
   auto fn       = [&](const real* X) {
      unpack(X);
      const auto err = data.error();
      if(err < best_err) {
         best_err = err;
         pack(&xbest[0]);
         if(feedback) {
            cout << "-------------------------------------------------- "
                 << method << endl;
            cout << format("#{:4d}, err={:10.8f}", counter, best_err) << endl;
            cout << "   Planes:" << endl;
            for(size_t i = 0; i < size_t(data.n_p3s()); ++i) {
               cout << format(
                   "      {}: {}", data.p3s[i].name, str(data.p3s[i].p3()))
                    << endl;
            }
            cout << "\n";
         }
      }
      ++counter;
      return err;
   };

   vector<real> start(n_params);
   vector<real> xmin(n_params);
   real ynewlo   = dNAN;
   real ystartlo = dNAN;
   real reqmin   = 1e-5;
   real diffstep = 0.1;

   int kcount = 40; // max interations
   int icount = 0, numres = 0, ifault = 0;

   pack(&start[0]);
   ystartlo = fn(&start[0]);

   if(!use_nelder_mead) {
      levenberg_marquardt(fn,
                          unsigned(n_params),
                          &start[0],
                          &xmin[0],
                          reqmin * 10.0,
                          diffstep,
                          2,
                          kcount,
                          icount,
                          ifault);
      ynewlo = fn(&xmin[0]);
   } else {
      vector<real> step(n_params);
      auto simplex_factor = 0.05;
      for(size_t i = 0; i < n_params; ++i) step[i] = one_degree();
      nelder_mead(fn,
                  unsigned(n_params),
                  &start[0],
                  &xmin[0],
                  ynewlo,
                  reqmin,
                  &step[0],
                  10,
                  40 * kcount,
                  icount,
                  numres,
                  ifault);
   }

   unpack(&xbest[0]);
   data.error(outdir, true);
   et_out = data.et;
   cam_ids_out.clear();
   for(const auto bcam_ptr : data.bcam_infos)
      cam_ids_out.push_back(bcam_ptr->camera_id);
   Expects(et_out.size() == cam_ids_out.size());

   Expects(opt_out.p3s.size() == data.p3s.size());
   for(auto i = 0u; i < opt_out.p3s.size(); ++i) {
      opt_out.p3s[i].d = data.p3s[i].d;
   }

   if(feedback) {
      ynewlo = data.error(outdir, true); // output files

      INFO(format("Feedback for optimizing {} cameras ({} params)",
                  data.n_cams(),
                  n_params));
      cout << format("   method:               {}", method) << endl;
      cout << format("   iterations:           {}", icount) << endl;
      cout << format("   fault-code:           {}", ifault) << endl;
      auto msg = (use_nelder_mead) ? nelder_mead_fault_str(ifault)
                                   : levenberg_marquardt_fault_str(ifault);
      cout << format("   fault-string:         {}", msg) << endl;
      cout << endl;
      cout << format("   initial-score:        {}", ystartlo) << endl;
      cout << format("   final-score:          {}", ynewlo) << endl;
      cout << endl;

      for(size_t i = 0; i < size_t(data.n_p3s()); ++i) {
         cout << format("    {}, {}", data.p3s[i].name, str(data.p3s[i].p3()))
              << endl;
      }

      cout << format("cost-function images saved to: {}", outdir) << endl;

      cout << "." << endl << endl;
   }

   return ynewlo;
}

real optimize_all(const SceneDescription& scene_desc,
                  const PhasePlaneData& phase_plane_data,
                  const vector<array<SlicData*, 2>>& slic_datas,
                  const bool use_nelder_mead, // true is a good bet
                  vector<EuclideanTransform>& et_out,
                  vector<string>& cam_ids_out,
                  PhasePlaneData& opt_out,
                  const string& outdir,
                  const bool feedback) noexcept
{
   const bool fix_relative = false;
   static EuclideanTransform e01{
       Vector3{0.442274, 2.813195, 0.011674},
       Quaternion{0.004430, 0.003533, 0.103784, 0.994584},
       1.0};

   PhasePlaneOptData data;
   data.init(scene_desc, phase_plane_data);
   if(et_out.size() > 0) {
      if(et_out.size() != data.et.size())
         FATAL(format("Expected {} euclidean-transforms, but got {}!",
                      data.n_cams(),
                      et_out.size()));
      data.et = et_out;
   }

   if(fix_relative) Expects(data.n_cams() == 2);

   if(feedback) {
      // -- (*) -- Some feedback
      INFO("optimize all:");
      cout << format("   Cameras ({})", data.n_cams()) << endl;
      for(const auto& bcam_info_ptr : data.bcam_infos)
         cout << format("      {}", bcam_info_ptr->camera_id) << endl;
      cout << format("   Planes  ({})", data.n_p3s()) << endl;
      for(const auto& pinfo : data.p3s)
         cout << format("      {} = {}", pinfo.name, str(pinfo.p3())) << endl;
      cout << format("   Points ({})", data.rpps.size()) << endl;
      for(const auto& rp : data.rpps)
         cout << format("      [{}, {}, {}, {}, {}]\n",
                        rp.sensor_key,
                        rp.inds[0],
                        rp.inds[1],
                        rp.x.x,
                        rp.x.y);

      auto err = data.error(outdir, true); // output files
      cout << format("   Initial error = {}", err) << endl;
      // exit(0);
   }

   const auto n_params
       = size_t(6 * (fix_relative ? 1 : data.n_cams()) + data.n_p3s() - 1);
   const auto err = data.error("/tmp", true);

   auto pack = [&](real* X) {
      Vector3 saa;
      for(const auto& et : data.et) {
         saa = quaternion_to_saa(et.rotation);
         for(auto i = 0; i < 3; ++i) *X++ = et.translation[i];
         for(auto i = 0; i < 3; ++i) *X++ = saa[i];
         if(fix_relative) break; // only need the first et
      }
      for(int i = 1; i < data.n_p3s(); ++i) *X++ = data.p3s[size_t(i)].d;
   };

   auto unpack = [&](const real* X) {
      Vector3 saa;
      for(auto& et : data.et) {
         for(auto i = 0; i < 3; ++i) et.translation[i] = *X++;
         for(auto i = 0; i < 3; ++i) saa[i] = *X++;
         et.rotation = saa_to_quaternion(saa);
         if(fix_relative) {
            data.et[1] = et * e01;
            break;
         }
      }
      for(int i = 1; i < data.n_p3s(); ++i) data.p3s[size_t(i)].d = *X++;
   };

   const char* method = use_nelder_mead ? "nelder-mead" : "levenberg-marquardt";
   vector<real> xbest(n_params);
   auto counter  = 0;
   auto best_err = std::numeric_limits<real>::max();
   auto fn       = [&](const real* X) {
      unpack(X);
      const auto err = data.error();
      if(err < best_err) {
         best_err = err;
         pack(&xbest[0]);
         if(feedback) {
            cout << "-------------------------------------------------- "
                 << method << endl;
            cout << format("#{:4d}, err={:10.8f}", counter, best_err) << endl;
            cout << format("   Cameras: [");
            for(auto i = 0u; i < data.et.size(); ++i) {
               if(i > 0) cout << ", ";
               cout << data.bcam_infos[i]->camera_id;
            }
            cout << "]" << endl;

            for(auto i = 0u; i < data.et.size(); ++i) {
               if(i > 0) cout << ",\n";
               cout << indent(data.et[i].to_json_str(), 7);
            }
            if(data.et.size() > 0) cout << endl;

            cout << "   Planes:" << endl;
            for(size_t i = 0; i < size_t(data.n_p3s()); ++i) {
               cout << format(
                   "      {}: {}", data.p3s[i].name, str(data.p3s[i].p3()))
                    << endl;
            }
            cout << "\n";
         }
      }
      ++counter;
      return err;
   };

   vector<real> start(n_params);
   vector<real> xmin(n_params);
   real ynewlo   = dNAN;
   real ystartlo = dNAN;
   real reqmin   = 1e-5;
   real diffstep = 0.1;

   int kcount = 40; // max interations
   int icount = 0, numres = 0, ifault = 0;

   pack(&start[0]);
   ystartlo = fn(&start[0]);

   if(!use_nelder_mead) {
      levenberg_marquardt(fn,
                          unsigned(n_params),
                          &start[0],
                          &xmin[0],
                          reqmin * 10.0,
                          diffstep,
                          2,
                          kcount,
                          icount,
                          ifault);
      ynewlo = fn(&xmin[0]);
   } else {
      vector<real> step(n_params);
      auto simplex_factor = 0.05;
      for(size_t i = 0; i < n_params; ++i) step[i] = one_degree();
      nelder_mead(fn,
                  unsigned(n_params),
                  &start[0],
                  &xmin[0],
                  ynewlo,
                  reqmin,
                  &step[0],
                  10,
                  40 * kcount,
                  icount,
                  numres,
                  ifault);
   }

   unpack(&xbest[0]);
   data.error(outdir, true);
   et_out = data.et;
   cam_ids_out.clear();
   for(const auto bcam_ptr : data.bcam_infos)
      cam_ids_out.push_back(bcam_ptr->camera_id);
   Expects(et_out.size() == cam_ids_out.size());

   Expects(opt_out.p3s.size() == data.p3s.size());
   for(auto i = 0u; i < opt_out.p3s.size(); ++i) {
      opt_out.p3s[i].d = data.p3s[i].d;
   }

   if(feedback) {
      ynewlo = data.error(outdir, true); // output files

      INFO(format("Feedback for optimizing {} cameras ({} params)",
                  data.n_cams(),
                  n_params));
      cout << format("   method:               {}", method) << endl;
      cout << format("   iterations:           {}", icount) << endl;
      cout << format("   fault-code:           {}", ifault) << endl;
      auto msg = (use_nelder_mead) ? nelder_mead_fault_str(ifault)
                                   : levenberg_marquardt_fault_str(ifault);
      cout << format("   fault-string:         {}", msg) << endl;
      cout << endl;
      cout << format("   initial-score:        {}", ystartlo) << endl;
      cout << format("   final-score:          {}", ynewlo) << endl;
      cout << endl;

      for(size_t i = 0; i < size_t(data.n_cams()); ++i) {
         const auto et0    = data.et[i];
         const auto [C, q] = make_camera_extrinsics(et0);
         cout << format("    {}, C={}, q={}",
                        data.bcam_infos[i]->camera_id,
                        str(C),
                        q.to_readable_str())
              << endl;
      }

      for(size_t i = 0; i < size_t(data.n_p3s()); ++i) {
         cout << format("    {}, {}", data.p3s[i].name, str(data.p3s[i].p3()))
              << endl;
      }

      cout << format("cost-function images saved to: {}", outdir) << endl;

      cout << "." << endl << endl;
   }

   return ynewlo;
}

} // namespace perceive::calibration
