
#include "camera-extrinsic.hpp"
#include "stdinc.hpp"

#include "perceive/geometry.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/geometry/projective/triangulation.hpp"
#include "perceive/io/perceive-assets.hpp"
#include "perceive/scene/aruco-result-info.hpp"
#include "perceive/scene/scene-description-info.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/math.hpp"

#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"

namespace perceive
{
// -------------------------------------------------- Estimate camera extrinsics

EuclideanTransform estimate_camera_extrinsics(
    const DistortionModel& M, // make sure working format is set
    const Matrix3r& K,
    const bool use_M,
    const Vector3& C0,
    const vector<Vector3>& W, // world coords
    const vector<Vector2>& P, // image coords
    real& reproj_error,
    const bool feedback,
    const bool super_feedback)
{
   Expects(P.size() == W.size());
   Expects(P.size() > 0);
   const bool use_nelder_mead = true;
   reproj_error               = 0.0;

   // Turn pixels 'P' into rays 'R'
   vector<Vector3> R(P.size());
   if(use_M) {
      std::transform(
          cbegin(P), cend(P), begin(R), [&](Vector2 D) { return M.to_ray(D); });
   } else {
      Matrix3r K_inv = K.inverse();
      std::transform(cbegin(P), cend(P), begin(R), [&](Vector2 D) {
         return to_vec3(K_inv * Vector3r(D.x, D.y, 1.0)).normalised();
      });
   }

   auto calc_Q = [&](const Vector3& C) {
      vector<Quaternion> qs;
      for(auto i = 0u; i < P.size(); ++i) {
         for(auto j = i + 1; j < P.size(); ++j) {
            auto wi = (W[i] - C).normalised();
            auto wj = (W[j] - C).normalised();
            auto q  = calc_rotation(R[i], wi, R[j], wj);
            qs.push_back(q);
         }
      }
      return qs[0]; // TODO, revisit this: why doesn't it work?
      return average_rotations(qs);
   };

   // ---- Find the camera center: 'C' such that all the angles are good

   Vector3 C           = C0;
   const auto n_params = 3;
   auto pack           = [&](real* X) {
      for(auto i = 0; i < 3; ++i) *X++ = C(i);
   };
   auto unpack = [&](const real* X) {
      for(auto i = 0; i < 3; ++i) C(i) = *X++;
   };

   auto show_errs_for_each_point = false;
   auto counter                  = 0;
   auto best_err                 = std::numeric_limits<real>::max();
   auto fn                       = [&](const real* X) {
      unpack(X);
      auto q   = calc_Q(C);
      auto err = 0.0;

      for(auto i = 0u; i < W.size(); ++i) {
         auto R    = (W[i] - C);
         auto E    = q.inverse_rotate(R);
         auto U    = E / E.z; // normalized coord
         auto D    = M.distort(U);
         auto err1 = (D - P[i]).norm();
         if(show_errs_for_each_point) {
            cout << format(
                " + {:2d} : p={} W={}, e={}", i, str(P[i]), str(W[i]), err1)
                 << endl;
         }
         err += err1;
      }
      err /= real(W.size());

      if(super_feedback and feedback) {
         if(err < best_err) {
            best_err = err;
            cout << format("#{:4d}, err = {}", counter, to_degrees(err))
                 << endl;
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
   int kcount    = 100000; // max interations
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
                          10,
                          kcount,
                          icount,
                          ifault);
      ynewlo = fn(&xmin[0]);

   } else {
      method = "nelder-mead";

      vector<real> step(n_params);
      auto X = &step[0];
      for(auto i = 0; i < 3; ++i) *X++ = 0.01; // 1cm

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

   unpack(&xmin[0]);

   if(feedback) {
      cout << format("Feedback for finding C for sensor {}", M.sensor_id())
           << endl;
      cout << format("   iterations:           {}", icount) << endl;
      cout << format("   fault-code:           {}", ifault) << endl;
      auto msg = (use_nelder_mead) ? nelder_mead_fault_str(ifault)
                                   : levenberg_marquardt_fault_str(ifault);
      cout << format("   fault-string:         {}", msg) << endl;
      cout << endl;
      cout << format("   initial-score:        {}", ystartlo) << endl;
      cout << format("   final-score:          {}", ynewlo) << endl;
      cout << endl;
      cout << format("C0 = {}", str(C0)) << endl;
      cout << format("C  = {}", str(C)) << endl;
      cout << endl;
   }

   show_errs_for_each_point = feedback;
   fn(&xmin[0]);

   // ---- Finally, calculate the rotation
   Quaternion q = calc_Q(C);
   auto et      = EuclideanTransform(C, q);

   if(feedback) {
      cout << format("et = {}", str(et)) << endl;
      ;
      cout << endl;
   }

   reproj_error = ynewlo;

   return et;
}

EuclideanTransform estimate_camera_extrinsics(
    const DistortionModel& M, // make sure working format is set
    const Vector3& C0,
    const vector<Vector3>& W, // world coords
    const vector<Vector2>& P, // image coords
    real& reproj_error,
    const bool feedback,
    const bool super_feedback)
{
   return estimate_camera_extrinsics(M,
                                     Matrix3r::Identity(),
                                     true,
                                     C0,
                                     W,
                                     P,
                                     reproj_error,
                                     feedback,
                                     super_feedback);
}

// -------------------------------------- Estimate (searh for) camera extrinsics

EuclideanTransform estimate_camera_extrinsics(
    const DistortionModel& M, // make sure working format is set
    const Vector3& min_xyz,   // 3D volume in which to find camera
    const Vector3& max_xyz,
    const vector<Vector3>& W, // world coords
    const vector<Vector2>& P, // image coords
    real& reproj_error,
    const bool feedback,
    const bool super_feedback)
{
   for(auto i = 0; i < 3; ++i) Expects(max_xyz(i) > min_xyz(i));

   Vector3 range          = max_xyz - min_xyz;
   const real granularity = 3.0;

   real best_score = std::numeric_limits<real>::max();
   EuclideanTransform best_et;

   auto try_C = [&](const Vector3& C) {
      real score = dNAN;
      auto et    = estimate_camera_extrinsics(
          M, C, W, P, score, super_feedback, false);
      if(std::isfinite(score) and score < best_score) {
         best_score = score;
         best_et    = et;
         if(feedback) {
            cout << format("reproj = {}, {}", best_score, str(best_et)) << endl;
         }
      }
   };

   const auto e = 1e-9;
   auto max_x   = min_xyz.x + ceil(range.y / granularity) * granularity + e;
   auto max_y   = min_xyz.y + ceil(range.x / granularity) * granularity + e;
   auto max_z   = min_xyz.z + ceil(range.z / granularity) * granularity + e;

   for(auto z = min_xyz.z; z <= max_z; z += granularity)
      for(auto y = min_xyz.y; y <= max_y; y += granularity)
         for(auto x = min_xyz.x; x <= max_x; x += granularity)
            try_C(Vector3(x, y, z));

   reproj_error = best_score;
   return best_et;
}

// ---------------------------------------------------------- Estimate Extrinsic

ArucoResultInfo estimate_extrinsic(const EstimateCamExtrinsicInfo& calib_info)
{
   const string scene_key = calib_info.scene_key;
   const auto& est_info   = calib_info.sensors;
   const auto ref_ind     = calib_info.ref_ind;
   const bool feedback    = calib_info.feedback;

   Expects(ref_ind < est_info.size());

   vector<EuclideanTransform> ets(est_info.size());

   auto process_i = [&](unsigned i) {
      const auto& info = est_info[i];
      const auto& C    = info.estimated_center;
      auto error       = 0.0;

      Matrix3r K = Matrix3r::Identity();
      bool use_M = info.is_regular_sensor;
      DistortionModel M;
      fetch(M, info.sensor_id);
      M.set_working_format(unsigned(info.image_format.x),
                           unsigned(info.image_format.y));
      INFO(format("working format set to: {}", str(info.image_format)));
      const auto& W = info.world_coords;
      const auto& P = info.image_coords;

      Vector3 xyz0(0, 0, 0);
      Vector3 xyz1(12, 12, 3);

      if(feedback) cout << string(70, '-') << " " << info.sensor_id << endl;

      if(C.is_finite()) {
         ets[i] = estimate_camera_extrinsics(
             M, K, use_M, C, W, P, error, feedback, false);
      } else {
         FATAL(format("C was not finite: {}", str(C)));
         ets[i]
             = estimate_camera_extrinsics(M, xyz0, xyz1, W, P, error, feedback);
      }
   };

   ParallelJobSet jobs;
   for(auto i = 0u; i < est_info.size(); ++i)
      jobs.schedule([i, process_i]() { process_i(i); });
   jobs.execute_non_parallel();

   EuclideanTransform W = ets[ref_ind];

   vector<string> ses(est_info.size());
   std::transform(cbegin(est_info),
                  cend(est_info),
                  begin(ses),
                  [](const auto& info) { return info.sensor_id; });

   ArucoResultInfo ainfo;
   ainfo.scene_key        = calib_info.scene_key;
   ainfo.reference_sensor = ses[ref_ind];
   ainfo.global_transform = W;
   ainfo.sensor_keys.insert(end(ainfo.sensor_keys), cbegin(ses), cend(ses));
   ainfo.aruco_transforms.resize(ets.size());
   std::transform(cbegin(ets),
                  cend(ets),
                  begin(ainfo.aruco_transforms),
                  [&](const auto& et) { return et / W; });
   return ainfo;
}

// ----------------------------------------------------- MEGA Estimate Extrinsic

ArucoResultInfo mega_estimate_extrinsic(const EstimateCamExtrinsicInfo& cinfo)
{
   const auto& calib_info = cinfo;
   const string scene_key = calib_info.scene_key;
   const auto& est_info   = calib_info.sensors;
   const auto ref_ind     = calib_info.ref_ind;
   const bool feedback    = calib_info.feedback;

   Expects(ref_ind < est_info.size());

   vector<EuclideanTransform> ets(est_info.size());
   vector<DistortionModel> M(est_info.size());

   // Load all the models
   for(auto i = 0u; i < M.size(); ++i) {
      const auto& info = est_info[i];
      fetch(M[i], info.sensor_id);
      M[i].set_working_format(unsigned(info.image_format.x),
                              unsigned(info.image_format.y));
   }

   auto fn = [&](const real* X) {
      // unpack(X);
      for(auto i = 0u; i < M.size(); ++i) {
         const auto& info = est_info[i];
         const auto& W    = info.world_coords;
         const auto& P    = info.image_coords;
      }
   };

   // auto process_i = [&] (unsigned i) {
   //     const auto& info = est_info[i];
   //     const auto& C = info.estimated_center;
   //     auto error = 0.0;

   //     Matrix3r K = Matrix3r::Identity();
   //     bool use_M = info.is_regular_sensor;
   //     DistortionModel M;
   //     fetch(M, info.sensor_id);
   //     M.set_working_format(info.image_format.x, info.image_format.y);
   //     INFO(format("working format set to: {}", str(info.image_format)));
   //     const auto& W = info.world_coords;
   //     const auto& P = info.image_coords;

   //     Vector3 xyz0(0, 0, 0);
   //     Vector3 xyz1(12, 12, 3);

   //     if(feedback)
   //         cout << string(70, '-') << " " << info.sensor_id << endl;

   //     if(C.is_finite()) {
   //         ets[i] = estimate_camera_extrinsics(M, K, use_M,
   //                                             C, W, P, error, feedback,
   //                                             false);
   //     } else {
   //         FATAL(format("C was not finite: {}", str(C)));
   //         ets[i] = estimate_camera_extrinsics(M, xyz0, xyz1, W, P,
   //                                             error, feedback);
   //     }
   // };

   // // ParallelJobSet jobs;
   // // for(auto i = 0u; i < est_info.size(); ++i)
   //     jobs.schedule([i, process_i] () { process_i(i); });
   // jobs.execute_non_parallel();

   // EuclideanTransform W = ets[ref_ind];

   // vector<string> ses(est_info.size());
   // std::transform(cbegin(est_info), cend(est_info), begin(ses),
   //                [] (const auto& info) { return info.sensor_id; });

   ArucoResultInfo ainfo;
   // ainfo.scene_key = calib_info.scene_key;
   // ainfo.reference_sensor = ses[ref_ind];
   // ainfo.global_transform = W;
   // ainfo.sensor_keys.insert(end(ainfo.sensor_keys), cbegin(ses), cend(ses));
   // ainfo.aruco_transforms.resize(ets.size());
   // std::transform(cbegin(ets), cend(ets), begin(ainfo.aruco_transforms),
   //                [&] (const auto& et) { return et / W; });
   return ainfo;
}

// ------------------------------------------------------------------- Load/Save

void load(EstimateCamExtrinsicInfo& data, const string& fname) noexcept(false)
{
   read(data, file_get_contents(fname));
}
void save(const EstimateCamExtrinsicInfo& data,
          const string& fname) noexcept(false)
{
   string s;
   write(data, s);
   file_put_contents(fname, s);
}

void read(EstimateCamExtrinsicInfo& data, const std::string& in) noexcept(false)
{
   read(data, str_to_json(in));
}
void write(const EstimateCamExtrinsicInfo& data,
           std::string& out) noexcept(false)
{
   auto g = [&](const auto& wc) -> string {
      return format("\"{}\": {}", json_encode(wc.name), json_encode(wc.pos));
   };

   auto h = [&](const auto& p) -> string {
      return format("\"{}\": {}", json_encode(p.name), json_encode(p.p3));
   };

   auto f = [&](const auto& sinfo) {
      const auto& C = sinfo.estimated_center;
      return format(
          R"V0G0N(
   {{
      "sensor-id": {},
      "estimated-center": [{}, {}, {}],
      "image-format": [{}, {}],
      "coord-name":   [{}],
      "world-coords": [{}],
      "image-coords": [{}]
   }}{})V0G0N",
          json_encode(sinfo.sensor_id),
          C.x,
          C.y,
          C.z,
          sinfo.image_format.x,
          sinfo.image_format.y,
          implode(cbegin(sinfo.coord_names),
                  cend(sinfo.coord_names),
                  ", ",
                  [&](const auto& s) { return json_encode(s); }),
          implode(cbegin(sinfo.world_coords),
                  cend(sinfo.world_coords),
                  ", ",
                  [&](const auto& s) { return json_encode(s); }),
          implode(cbegin(sinfo.image_coords),
                  cend(sinfo.image_coords),
                  ", ",
                  [&](const auto& s) { return json_encode(s); }),
          "");
   };

   out = format(R"V0G0N(
{{
   "type": "EstimateCamExtrinsicInfo",
   "scene-key": {},
   "ref-index": {},
   "feedback": {},
   "global-world-coords": {{{}}},
   "planes": {{{}}},
   "sensor-info": [{}]
}}
{})V0G0N",
                json_encode(data.scene_key),
                data.ref_ind,
                (data.feedback ? "true" : "false"),
                implode(cbegin(data.coords), cend(data.coords), ", ", g),
                implode(cbegin(data.planes), cend(data.planes), ", ", h),
                implode(cbegin(data.sensors), cend(data.sensors), ", ", f),
                "");
}

void read(EstimateCamExtrinsicInfo& data,
          const Json::Value& root) noexcept(false)
{
   EstimateCamExtrinsicInfo o;
   json_load(get_key(root, "scene-key"), o.scene_key);
   json_load(get_key(root, "ref-index"), o.ref_ind);
   if(has_key(root, "feedback"))
      json_load(get_key(root, "feedback"), o.feedback);

   if(has_key(root, "planes")) {
      auto arr = get_key(root, "planes");
      if(!arr.isObject())
         throw std::runtime_error("'planes' is not an object!");
      const auto keys = arr.getMemberNames();
      o.planes.resize(keys.size());

      for(auto i = 0u; i < keys.size(); ++i) {
         auto& plane = o.planes[i];
         plane.name  = keys[i];
         json_load(arr[keys[i]], plane.p3);
      }
   }

   if(has_key(root, "global-world-coords")) {
      auto arr = get_key(root, "global-world-coords");
      if(!arr.isObject())
         throw std::runtime_error("'global-world-coords' is not an object!");

      const auto keys = arr.getMemberNames();

      o.coords.resize(keys.size());

      for(auto i = 0u; i < keys.size(); ++i) {
         auto& wc = o.coords[i];
         wc.name  = keys[i];
         json_load(arr[keys[i]], wc.pos);

         // Does 'wc.pos' sit on any plane?
         for(auto j = 0u; j < o.planes.size(); ++j)
            if(o.planes[j].p3.distance(wc.pos) < 1e-9)
               wc.plane_ids.push_back(int(j));
      }
   }

   auto arr = get_key(root, "sensor-info");
   if(!arr.isArray())
      throw std::runtime_error("'sensor-info' is not an array!");

   o.sensors.resize(arr.size());
   for(auto i = 0u; i < arr.size(); ++i) {
      const auto& node = arr[i];
      auto& info       = o.sensors[i];

      json_load(get_key(node, "sensor-id"), info.sensor_id);
      json_load(get_key(node, "estimated-center"), info.estimated_center);
      json_load(get_key(node, "image-format"), info.image_format);
      json_load(get_key(node, "image-coords"), info.image_coords);

      if(has_key(node, "image-file")) {
         json_load(get_key(node, "image-file"), info.image_filename);
      }

      auto wc_node = get_key(node, "world-coords");
      if(!wc_node.isArray())
         throw std::runtime_error("'world-coords' is not an array!");
      info.world_coords.resize(wc_node.size());
      for(auto i = 0u; i < info.world_coords.size(); ++i) {
         auto wc = wc_node[i];
         if(wc.isString()) {
            if(info.coord_names.size() != info.world_coords.size())
               info.coord_names.resize(info.world_coords.size());
            json_load(wc, info.coord_names[i]);
            info.world_coords[i] = o.lookup(info.coord_names[i]);
         } else {
            json_load(wc, info.world_coords[i]);
         }
      }

      const auto& facecam_node = node["face-cam"];
      if(facecam_node.isBool())
         info.is_regular_sensor = facecam_node.asBool();
      else
         info.is_regular_sensor = true;

      if(!info.is_regular_sensor) json_load(get_key(node, "intrinsic"), info.K);
   }

   data = o;
}

// ------------------------------------------------------ Run Estimate Extrinsic

void run_estimate_extrinsic(const string& input_file,
                            const string& output_file,
                            const bool mega_opt,
                            const bool feedback)
{
   EstimateCamExtrinsicInfo data;
   load(data, input_file);

   if(false) {
      string out;
      write(data, out);
      WARN(format("Loaded '{}'", input_file));
      cout << out << endl << endl;
      FATAL("kbam!");
   }

   SceneDescriptionInfo s_info;
   fetch(s_info, data.scene_key);

   bool has_error    = false;
   auto check_sensor = [&](const string& sensor_id) {
      auto p = [&](const auto& sinfo) -> bool {
         return sinfo.sensor_id == sensor_id;
      };
      auto ii = std::find_if(cbegin(data.sensors), cend(data.sensors), p);

      if(ii == cend(data.sensors)) {
         has_error = true;
         LOG_ERR(format("Scene '{}' has a camera with sensor '{}'; however, "
                        "this sensor was not present in input data.",
                        data.scene_key,
                        sensor_id));
      }
   };

   // Make sure all the sensors in s-info are present in data
   auto counter = 0;
   for(const auto& bcam_key : s_info.bcam_keys) {
      BinocularCameraInfo bcam_info;
      fetch(bcam_info, bcam_key);
      check_sensor(bcam_info.M[0].sensor_id());
      check_sensor(bcam_info.M[1].sensor_id());
      counter += 2;
   }

   auto n_dat_sensors = 0;
   for(const auto& sensor : data.sensors)
      if(sensor.is_regular_sensor) n_dat_sensors++;

   if(n_dat_sensors != counter) {
      has_error = true;
      LOG_ERR(format("Expected {} sensors, but input file has {}",
                     counter,
                     n_dat_sensors));
   }

   data.feedback = feedback;

   if(mega_opt) {
      auto ainfo = mega_estimate_extrinsic(data);
      FATAL(format("save step not done"));
      // save(ainfo, output_file);
   } else {
      auto ainfo = estimate_extrinsic(data);
      save(ainfo, output_file);
   }

   if(has_error) {
      cout << format("Aborting...") << endl;
      exit(1);
   }
}

} // namespace perceive
