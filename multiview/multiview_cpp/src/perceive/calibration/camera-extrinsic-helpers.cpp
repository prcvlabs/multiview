
#include "camera-extrinsic-helpers.hpp"
#include "perceive/geometry.hpp"
#include "perceive/utils/eigen-helpers.hpp"

#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"

namespace perceive
{
// --------------------------------------------------------- for output purposes

static const auto g_info = "\x1b[37m\u261b\x1b[0m"s;

static void
print2(const string& glyph, const string& msg, const bool print_newline = true)
{
   cout << format("   {} {}\x1b[0m", glyph, msg);
   if(print_newline)
      cout << endl;
   else
      cout.flush();
}

// ----------------------------------------------------- estimate-et-LLS-one-cam

// Estimate et for a single camera, LLS
EuclideanTransform estimate_et_LLS_one_cam(const vector<Vector3>& Ws,
                                           const vector<Vector2>& Us)
{
   Expects(Ws.size() == Us.size());
   Expects(Us.size() >= 6);

   // Set up equation AX = b, were 'X' is a column vector for [R|t]
   const auto equations_per_point = 2;
   const auto N                   = Us.size() * equations_per_point;

   // [r11 r12 r13 t1 r21 r22 r23 t2 r31 r32 r33 t3]
   MatrixXr A = MatrixXr::Zero(long(N), 12);

   for(auto i = 0u; i < Us.size(); ++i) {
      const auto& X = Ws[i];
      const auto U0 = homgen_R2_to_P2(Us[i]).normalised();

      const auto offset = i * equations_per_point;

      {
         const auto row = offset + 0;
         A(row, 4)      = -U0(2) * X(0); // -u_2 v_3
         A(row, 5)      = -U0(2) * X(1);
         A(row, 6)      = -U0(2) * X(2);
         A(row, 7)      = -U0(2) * 1.0;
         A(row, 8)      = U0(1) * X(0); //  u_3 v_2
         A(row, 9)      = U0(1) * X(1);
         A(row, 10)     = U0(1) * X(2);
         A(row, 11)     = U0(1) * 1.0;
      }

      {
         const auto row = offset + 1;
         A(row, 0)      = U0(2) * X(0); //  u_1 v_3
         A(row, 1)      = U0(2) * X(1);
         A(row, 2)      = U0(2) * X(2);
         A(row, 3)      = U0(2) * 1.0;
         A(row, 8)      = -U0(0) * X(0); // -u_3 v_1
         A(row, 9)      = -U0(0) * X(1);
         A(row, 10)     = -U0(0) * X(2);
         A(row, 11)     = -U0(0) * 1.0;
      }
   }

   // right null-space of A
   MatrixXr U, V;
   VectorXr D, X;
   svd_UDV(A, U, D, V);
   auto err = svd_thin(A, X);

   // How well conditioned was that?
   if(true) {
      cout << "V = " << V << endl << endl;
      cout << "X = " << X << endl;
      cout << format("err = {}", err) << endl;
      cout << "D = " << D << endl;
   }

   Matrix3r R_out = Matrix3r::Zero();
   Vector3 t_out;
   R_out(0, 0) = V(0, 11);
   R_out(0, 1) = V(1, 11);
   R_out(0, 2) = V(2, 11);
   t_out(0)    = V(3, 11);
   R_out(1, 0) = V(4, 11);
   R_out(1, 1) = V(5, 11);
   R_out(1, 2) = V(6, 11);
   t_out(1)    = V(7, 11);
   R_out(2, 0) = V(8, 11);
   R_out(2, 1) = V(9, 11);
   R_out(2, 2) = V(10, 11);
   t_out(2)    = V(11, 11);

   // Vector3 axis = to_vec3(R_out * Vector3r(0, 1, 0));
   // Quaternion rot180q(axis.x, axis.y, axis.z, to_radians(180.0));
   // Matrix3r rot180 = quaternion_to_rot3x3(rot180q);
   // Matrix3r R_tmp = rot180 * R_out;
   // R_out = R_tmp;

   // t_out = -t_out;
   R_out = -R_out;
   t_out = -t_out;

   EuclideanTransform et_out;
   et_out.rotation    = rot3x3_to_quaternion(R_out);
   et_out.translation = t_out; // -to_vec3(R_out.inverse() * to_vec3r(t_out));
   et_out.scale       = 1.0;

   cout << ">>" << endl;
   for(auto i = 0u; i < Ws.size(); ++i) {
      const auto X   = Ws[i];
      const auto u0  = (to_vec3(R_out * to_vec3r(X)) + t_out).normalised();
      const auto u1  = (homgen_R2_to_P2(Us[i])).normalised();
      const auto u2  = (et_out.rotation.apply(X) + t_out).normalised();
      const auto deg = to_degrees(acos(dot(u1, u2)));
      cout << format("u,u = <{}, {}, {}> = {}", str(u0), str(u1), str(u2), deg)
           << endl;
   }
   cout << "------------" << endl;

   return et_out;
}

// --------------------------------------------------------- estimate-et-one-cam

EuclideanTransform
estimate_et_one_cam(const DistortionModel& M, // make sure working format is set
                    const Vector3& C0,
                    const vector<Vector3>& W, // world coords
                    const vector<Vector2>& P, // image coords (undistorted)
                    real& reproj_error,
                    const bool feedback,
                    const bool super_feedback)
{
   Expects(P.size() == W.size());
   Expects(P.size() > 0);
   const auto N               = W.size();
   const bool use_nelder_mead = true;
   reproj_error               = 0.0;

   // Turn pixels 'P' into rays 'R'
   vector<Vector3> R(N);
   std::transform(
       cbegin(P), cend(P), begin(R), [&](Vector2 D) { return M.to_ray(D); });

   auto calc_Q = [&](const Vector3& C) {
      vector<Quaternion> qs;
      for(auto i = 0u; i < N; ++i) {
         for(auto j = i + 1; j < N; ++j) {
            auto wi = (W[i] - C).normalised();
            auto wj = (W[j] - C).normalised();
            auto q  = calc_rotation(R[i], wi, R[j], wj);
            qs.push_back(q);
         }
      }
      return qs[0];
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

// ----------------------------------------------------------- refine-et-one-cam

static real refine_et_one_cam(const vector<Vector3>& Ws,
                              const vector<Vector2>& Us,
                              EuclideanTransform& et0,
                              const bool use_nelder_mead = false)
{
   EuclideanTransform et = et0;

   auto pack = [&](real* X) {
      Vector3 saa = quaternion_to_saa(et.rotation);
      for(auto i = 0; i < 3; ++i) *X++ = et.translation[i];
      for(auto i = 0; i < 3; ++i) *X++ = saa[i];
   };

   auto unpack = [&](const real* X) {
      Vector3 saa(0, 0, 0);
      for(auto i = 0; i < 3; ++i) et.translation[i] = *X++;
      for(auto i = 0; i < 3; ++i) saa[i] = *X++;
      et.rotation = saa_to_quaternion(saa);
   };

   auto fn = [&](const real* X) -> real {
      unpack(X);
      auto err = real(0.0);
      for(auto i = 0u; i < Ws.size(); ++i) {
         const auto U   = Us[i];
         const auto W   = Ws[i];
         const auto ray = (et.rotation.apply(W) + et.translation).normalised();
         const auto Y   = homgen_P2_to_R2(ray);
         err += (ray.z < 0.0 ? 100.0 : 1.0) * (U - Y).norm();
      }
      return err / real(Ws.size());
   };

   const auto n_params = 6;
   const bool feedback = false;
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
      for(auto i = 0; i < 3; ++i) *X++ = 0.01;
      for(auto i = 3; i < 6; ++i) *X++ = to_degrees(1.0);
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
      cout << format("opt [R|t], one cam") << endl;
      cout << format("   iterations:           {}", icount) << endl;
      cout << format("   fault-code:           {}", ifault) << endl;
      auto msg = (use_nelder_mead) ? nelder_mead_fault_str(ifault)
                                   : levenberg_marquardt_fault_str(ifault);
      cout << format("   fault-string:         {}", msg) << endl;
      cout << endl;
      cout << format("   initial-score:        {}", ystartlo * 400.0) << endl;
      cout << format("   final-score:          {}", ynewlo * 400.0) << endl;
      cout << endl;
      cout << format("   et0 = {}", str(et0)) << endl;
      cout << format("   et  = {}", str(et)) << endl;
      cout << endl;
   }

   if(ynewlo < ystartlo) et0 = et;

   return ynewlo;
}

// --------------------------------------------------------- estimate-et-one-cam

EuclideanTransform optimize_et_one_cam(const EuclideanTransform& et0,
                                       const vector<Vector3>& Ws,
                                       const vector<Vector2>& Us)
{
   EuclideanTransform et_opt;
   et_opt = et0;

   auto print_result = [&](const int attempt_no,
                           const EuclideanTransform& et_opt,
                           const real ynewlo) {
      const auto e_inv = et_opt.inverse();
      const auto aa    = quaternion_to_axis_angle(e_inv.rotation);

      auto msg = format("#{:4d}, t = {}, rot = {}, \u03b8 = {} degrees"
                        ", err = {}",
                        attempt_no,
                        str(e_inv.translation),
                        str(aa.xyz()),
                        to_degrees(aa[3]),
                        ynewlo * 400.0);

      print2(g_info, msg);
   };

   // print_result(0, et0, NAN);
   auto yoptlo = refine_et_one_cam(Ws, Us, et_opt);
   print_result(0, et_opt, yoptlo);

   const int n_attempts = 1000;
   const real min_theta = to_radians(45.0), max_theta = to_radians(90.0);
   const real min_t = 0.50, max_t = 5.0;

   for(auto attempt_no = 0; attempt_no < n_attempts; ++attempt_no) {
      real perc = (n_attempts - attempt_no) / real(n_attempts - 1); // [0..1]
      real theta_diff = perc * (max_theta - min_theta) + min_theta;
      real t_diff     = perc * (max_t - min_t) + min_t;

      // const real theta_diff = to_radians(45.0);
      // const real t_diff = 1.0;

      // Randomly perturb
      EuclideanTransform e = et_opt;
      for(auto i = 0; i < 3; ++i)
         e.translation[i] += t_diff * 2.0 * (0.5 - uniform());
      auto saa = quaternion_to_saa(e.rotation);
      for(auto i = 0; i < 3; ++i)
         saa[i] += theta_diff * 2.0 * (0.5 - uniform());
      e.rotation = saa_to_quaternion(saa);

      auto ynewlo = refine_et_one_cam(Ws, Us, e);
      if(ynewlo < yoptlo) {
         yoptlo = ynewlo;
         et_opt = e;
         print_result(attempt_no, et_opt, ynewlo);
      }
   }

   return et_opt;
}

} // namespace perceive
