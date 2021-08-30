
#include "stereo-cam-pos-opt.hpp"

#include "camera-extrinsic-helpers.hpp"

#include "perceive/foundation.hpp"
#include "perceive/geometry.hpp"
#include "perceive/graphics.hpp"

#include "perceive/io/perceive-assets.hpp"

#include "perceive/utils/file-system.hpp"
#include "perceive/utils/math.hpp"

#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/sprite.hpp"

#include "perceive/geometry.hpp"
#include "perceive/geometry/projective/back-project-kite.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/geometry/projective/triangulation.hpp"
#include "perceive/scene/aruco-result-info.hpp"
#include "perceive/scene/scene-description.hpp"
#include "perceive/scene/scene-manifest.hpp"

#include "perceive/calibration/camera-extrinsic.hpp"
#include "perceive/calibration/find-F.hpp"

#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"

#ifdef USING_OPENGL
#include "gl/gl-inc.hpp"
#include <GL/gl.h>
#endif

namespace perceive::calibration
{
using namespace std::string_literals;

// --------------------------------------------------------- for output purposes

inline void
print(const string& glyph, const string& msg, const bool print_newline = true)
{
   cout << format(" {} {}\x1b[0m", glyph, msg);
   if(print_newline)
      cout << endl;
   else
      cout.flush();
}

inline void
print2(const string& glyph, const string& msg, const bool print_newline = true)
{
   cout << format("   {} {}\x1b[0m", glyph, msg);
   if(print_newline)
      cout << endl;
   else
      cout.flush();
}

static const auto g_info         = "\x1b[37m\u261b\x1b[0m"s;
static const auto g_skull        = "\x1b[91m\u2620\x1b[0m"s;
static const auto g_radioactive  = "\x1b[91m\u2622\x1b[0m"s;
static const auto g_dotted_sq    = "\x1b[96m\u2b1a\x1b[0m"s;
static const auto g_bullet       = "\x1b[0m\u2738\x1b[0m"s;
static const auto g_bullet_big   = "\x1b[36m\u2739\x1b[0m"s;
static const auto g_cross_arrows = "\x1b[96m\u2928\x1b[0m"s;
static const auto g_waves        = "\x1b[96m\u29da\x1b[0m"s;
static const auto g_wave         = "\x1b[96m\u223c\x1b[0m"s;
static const auto g_wedge        = "\x1b[0m\u2023\x1b[0m"s;
static const auto g_cross        = "\x1b[96m\u2613\x1b[0m"s;
static const auto g_victory      = "\x1b[40m\x1b[97m\u270c\x1b[0m"s;
static const auto g_coffee       = "\x1b[40m\x1b[97m\u26fe\x1b[0m"s;
static const auto g_tick         = "\x1b[40m\x1b[92m\u2714\x1b[0m"s;

static const auto g_default    = "\x1b[0m"s;
static const auto g_red        = "\x1b[31m"s;
static const auto g_error      = "\x1b[4m\x1b[91m"s;
static const auto g_light_gray = "\x1b[37m"s;
static const auto g_light_red  = "\x1b[91m"s;
static const auto g_white      = "\x1b[97m"s;

// ----------------------------------------------- LLS method to get 'R' and 't'

// static EuclideanTransform get_et(const vector<Vector3>& Ws,
//                                      const vector<Vector2>& U0s,
//                                      const vector<Vector2>& U1s,
//                                      const EuclideanTransform& et_cam0_1,
//                                      const BinocularCamera& bcam)
// {
//     if(U0s.size() < 5)
//         FATAL(format("Require at least 6 corresponding pairs to do this!"));
//     Expects(U0s.size() == U1s.size());
//     Expects(Ws.size() == U0s.size());

//     {

//         EuclideanTransform et0;
//         Vector3 t0_ = Vector3(-3.54873, 3.43016, -2.02586);
//         et0.rotation = Quaternion(Vector4(0.97186, -0.21565, 0.09476,
//                                           to_radians(110.738)));
//         et0.translation = -et0.rotation.inverse_apply(t0_);

//         cout << format("et  = {}", str(et)) << endl;
//         cout << format("et0 = {}", str(et0.inverse())) << endl;

//         const auto R0 = quaternion_to_rot3x3(et0.rotation);
//         const auto R1 = quaternion_to_rot3x3(et .rotation);
//         const auto t0 = -et0.rotation.rotate(et0.translation);
//         const auto t1 = et.translation;
//             //et .rotation.rotate(et.translation);

//         cout << format("t0 = {}", str(t0)) << endl;

//         for(auto i = 0u; i < Ws.size(); ++i) {
//             const auto X = Ws[i];
//             const auto U0 = (to_vec3(R0 * to_vec3r(X)) + t0).normalised();
//             const auto U1 = (to_vec3(R1 * to_vec3r(X)) + t1).normalised();

//             cout << format("U0 = {}", str(U0)) << endl;
//             cout << format("U2 = {}",
//             str(homgen_R2_to_P2(U0s[i]).normalised()))
//                  << endl;
//             cout << format("U1 = {}", str(U1)) << endl;

//             cout << endl;
//         }

//         FATAL("kBAM!");
//     }

//     const auto e01i = et_cam0_1.inverse();

//     const Matrix3r R0 = Matrix3r::Identity();
//     const Vector3  t0 = Vector3(0, 0, 0);

//     const Matrix3r R_ = quaternion_to_rot3x3(e01i.rotation);
//     const Vector3  t_ = e01i.translation;
//     const Vector3  C_ = to_vec3(R_.inverse() * to_vec3r(t_));

//     // Set up equation AX = b, were 'X' is a column vector for [R|t]
//     const auto equations_per_point = 6;
//     const auto N = U0s.size() * equations_per_point;
//     MatrixXr b = MatrixXr::Zero(N, 1);

//     // [r11 r12 r13 t1 r21 r22 r23 t2 r31 r32 r33 t3]
//     MatrixXr A = MatrixXr::Zero(N, 12);

//     for(auto i = 0u; i < U0s.size(); ++i) {
//         const auto& X = Ws[i];

//         // const auto U0 = homgen_R2_to_P2(U0s[i]).normalised();
//         // const auto U1 = homgen_R2_to_P2(U1s[i]).normalised();

//         Matrix3r R1 = R_ * R0;
//         Vector3  t1 = to_vec3(R_ * to_vec3r(t0)) + t_;
//         const auto U0 = (to_vec3(R0 * to_vec3r(X)) + t0).normalised();
//         const auto U1 = (to_vec3(R1 * to_vec3r(X)) + t1).normalised();

//         const auto V0 = to_vec3(R_.inverse() * to_vec3r(U0)).normalised();
//         const auto V1 = to_vec3(R_.inverse() * to_vec3r(U1)).normalised();
//         const auto offset = i * equations_per_point;

//         {
//             const auto row = offset + 0;
//             A(row, 4) = -U0(2) * X(0); // -u_2 v_3
//             A(row, 5) = -U0(2) * X(1);
//             A(row, 6) = -U0(2) * X(2);
//             A(row, 7) = -U0(2) * 1.0;
//             A(row, 8) =  U0(1) * X(0); //  u_3 v_2
//             A(row, 9) =  U0(1) * X(1);
//             A(row,10) =  U0(1) * X(2);
//             A(row,11) =  U0(1) * 1.0;
//             b(row) = 0.0;
//         }

//         {
//             const auto row = offset + 1;
//             A(row, 0) =  U0(2) * X(0); //  u_1 v_3
//             A(row, 1) =  U0(2) * X(1);
//             A(row, 2) =  U0(2) * X(2);
//             A(row, 3) =  U0(2) * 1.0;
//             A(row, 8) = -U0(0) * X(0); // -u_3 v_1
//             A(row, 9) = -U0(0) * X(1);
//             A(row,10) = -U0(0) * X(2);
//             A(row,11) = -U0(0) * 1.0;
//             b(row) = 0.0;
//         }

//         {
//             const auto row = offset + 2;
//             A(row, 0) = -U0(1) * X(0); // -u_1 v_2
//             A(row, 1) = -U0(1) * X(1);
//             A(row, 2) = -U0(1) * X(2);
//             A(row, 3) = -U0(1) * 1.0;
//             A(row, 4) =  U0(0) * X(0); //  u_2 v_1
//             A(row, 5) =  U0(0) * X(1);
//             A(row, 6) =  U0(0) * X(2);
//             A(row, 7) =  U0(0) * 1.0;
//             b(row) = 0.0;
//         }

//         {
//             const auto row = offset + 3;
//             A(row, 4) = -V1(2) * X(0); // -u_2 v_3
//             A(row, 5) = -V1(2) * X(1);
//             A(row, 6) = -V1(2) * X(2);
//             A(row, 7) = -V1(2) * 1.0;
//             A(row, 8) =  V1(1) * X(0); //  u_3 v_2
//             A(row, 9) =  V1(1) * X(1);
//             A(row,10) =  V1(1) * X(2);
//             A(row,11) =  V1(1) * 1.0;
//             b(row) = C_(1) * V1(2) - C_(2) * V1(1);
//         }

//         {
//             const auto row = offset + 4;
//             A(row, 0) =  V1(2) * X(0); //  u_1 v_3
//             A(row, 1) =  V1(2) * X(1);
//             A(row, 2) =  V1(2) * X(2);
//             A(row, 3) =  V1(2) * 1.0;
//             A(row, 8) = -V1(0) * X(0); // -u_3 v_1
//             A(row, 9) = -V1(0) * X(1);
//             A(row,10) = -V1(0) * X(2);
//             A(row,11) = -V1(0) * 1.0;
//             b(row) = C_(2) * V1(0) - C_(0) * V1(2);
//         }

//         {
//             const auto row = offset + 5;
//             A(row, 0) = -V1(1) * X(0); // -u_1 v_2
//             A(row, 1) = -V1(1) * X(1);
//             A(row, 2) = -V1(1) * X(2);
//             A(row, 3) = -V1(1) * 1.0;
//             A(row, 4) =  V1(0) * X(0); //  u_2 v_1
//             A(row, 5) =  V1(0) * X(1);
//             A(row, 6) =  V1(0) * X(2);
//             A(row, 7) =  V1(0) * 1.0;
//             b(row) = C_(0) * V1(1) - C_(1) * V1(0);
//         }
//     }

//     MatrixXr At = A.transpose();
//     MatrixXr AtA = At * A;
//     MatrixXr AtA_inv = AtA.inverse();
//     MatrixXr AtA_inv_At = AtA_inv * At;

//     MatrixXr X = AtA_inv_At * b;

//     Matrix3r R_out = Matrix3r::Zero();
//     Vector3 t_out;
//     R_out(0, 0) =  X(0);
//     R_out(0, 1) =  X(1);
//     R_out(0, 2) =  X(2);
//     t_out(0)    =  X(3);
//     R_out(1, 0) =  X(4);
//     R_out(1, 1) =  X(5);
//     R_out(1, 2) =  X(6);
//     t_out(1)    =  X(7);
//     R_out(2, 0) =  X(8);
//     R_out(2, 1) =  X(9);
//     R_out(2, 2) = X(10);
//     t_out(2)    = X(11);

//     EuclideanTransform et_out;
//     et_out.translation = t_out;
//     et_out.rotation = rot3x3_to_quaternion(R_out);
//     et_out.scale = 1.0;

//     if(true) {

//         // Get the undistorted points
//         EuclideanTransform et0, et1;
//         et0.translation = Vector3(-3.54873, 3.43016, -2.02586);
//         et0.rotation = Quaternion(Vector4(0.97186, -0.21565, 0.09476,
//                                           to_radians(110.738)));

//         et1 = et0 * e01i;

//         cout << "et0 = " << str(et0) << endl;
//         cout << "et1 = " << str(et1) << endl;

//         auto test_X = [&] (Vector3 X) {

//             const auto Y = et0.apply(X);

//             const auto& M0 = bcam.model(CAM0);
//             const auto& M1 = bcam.model(CAM1);
//             const auto D0 = bcam.project(CAM0, Y);
//             const auto D1 = bcam.project(CAM1, Y);
//             const auto U0 = M0.undistort(D0); // Normalized coords
//             const auto U1 = M1.undistort(D1); //

//             const auto e01 = et_cam0_1;
//             const auto e01i = et_cam0_1.inverse();

//             Matrix3r R1 = R_ * R0;
//             Vector3  t1 = to_vec3(R_ * to_vec3r(t0)) + t_;

//             const auto u0 = homgen_P2_to_R2(to_vec3(R0 * to_vec3r(X)) + t0);
//             const auto u1 = homgen_P2_to_R2(to_vec3(R1 * to_vec3r(X)) + t1);

//             // For the RHS points, let's try premultiplying by R_^{-1}
//             const auto v1 = (to_vec3(R0 * to_vec3r(X)) + t0 +
//             C_).normalised(); const auto V1 =
//             to_vec3(R_.inverse()*to_vec3r(homgen_R2_to_P2(U1)))
//             .normalised();

//             cout << format("X    = {}", str(X)) << endl;
//             cout << format("Y    = {}", str(Y)) << endl;
//             cout << format("D0,1 = {} :: {}", str(D0), str(D1)) << endl;
//             cout << format("U0,1 = {} :: {}", str(U0), str(U1)) << endl;
//             cout << format("u0,1 = {} :: {}", str(u0), str(u1)) << endl;
//             cout << "------------------------" << endl;
//             cout << format("v1,1 = |{} - {}| = {}",
//                            str(v1), str(V1), (v1 - V1).norm()) << endl;
//             cout << endl;
//         };

//         test_X(Vector3(5.48200, 3.22900, 0.71700));
//         test_X(Vector3(5.94400, 3.82200, 0.71700));
//         test_X(Vector3(4.48600, 4.47900, 0.71700));
//         test_X(Vector3(7.76500, 3.28400, 1.20600));

//         MatrixXr Y = MatrixXr::Zero(12, 1);

//         Y(0)  = R0(0, 0);
//         Y(1)  = R0(0, 1);
//         Y(2)  = R0(0, 2);
//         Y(3)  = t0(0);
//         Y(4)  = R0(1, 0);
//         Y(5)  = R0(1, 1);
//         Y(6)  = R0(1, 2);
//         Y(7)  = t0(1);
//         Y(8)  = R0(2, 0);
//         Y(9)  = R0(2, 1);
//         Y(10) = R0(2, 2);
//         Y(11) = t0(2);

//         MatrixXr Z = A * Y - b;
//         cout << "Z = " << endl << Z << endl << endl;

//         cout << format("z\tX\tY\tdelta") << endl;
//         for(auto i = 0; i < 12; ++i) {
//             cout << format("{:4.3f}\t{}\t{}\t{}",
//                            Z(i), X(i), Y(i), fabs(X(i) - Y(i)))
//                  << endl;
//         }

//         FATAL("We were here");
//     }

//     return et_out.inverse();
// }

// ------------------------------------------------------------------- Get fname

static string get_fname(const string& f)
{
   if(is_regular_file(f)) return f;
   throw std::runtime_error(format("failed to find file '{}'", f));
}

// ----------------------------------------------------------- Find Sensor Index

static int find_sensor_index(const EstimateCamExtrinsicInfo& ex_info,
                             const string& id)
{
   auto ii = std::find_if(cbegin(ex_info.sensors),
                          cend(ex_info.sensors),
                          [&](const auto& s) { return id == s.sensor_id; });
   if(ii == cend(ex_info.sensors)) return -1;
   return int(std::distance(cbegin(ex_info.sensors), ii));
}

// ------------------------------------------------------------------ Draw Image

static void
draw_one_cam_result(const DistortionModel& M, // make sure working format is set
                    const vector<Vector3>& Ws,
                    const EuclideanTransform& et_opt,
                    const vector<string>& names,
                    const vector<Vector2>& Ds,
                    const EstimateCamExtrinsicInfo::SensorInfo& info,
                    const string& filename)
{
   ARGBImage im;
   if(!info.image_filename.empty()) {
      im = ARGBImage::load(get_fname(info.image_filename));
   } else {
      WARN(format("no image specified, so not drawing results"));
      return;
   }

   for(auto i = 0u; i < Ws.size(); ++i) {
      const auto& d    = Ds[i];
      const auto label = names[i];

      const auto& W = Ws[i];
      const auto U  = homgen_P2_to_R2(et_opt.apply(W));
      const auto D  = M.distort(U);

      const auto o = Point2(2, -7);

      bresenham(d, D, [&](int x, int y) { set(im, x, y, k_yellow); });

      draw_cross(im, to_pt2(d), k_red, 5);
      draw_cross(im, to_pt2(D), k_blue, 5);
      render_string(im, label, to_pt2(d) + o, k_yellow, k_black);
   }

   im.save(filename);
}

// -------------------------------------------------------- draw position result

static void
draw_position_result(const string& prefix,
                     const vector<Vector3>& Ws,
                     vector<array<Vector2, 2>>& raw_pairs,
                     const vector<string>& names,
                     const BinocularCamera& bcam,
                     const EuclideanTransform& et_opt,
                     const EstimateCamExtrinsicInfo::SensorInfo& info0,
                     const EstimateCamExtrinsicInfo::SensorInfo& info1)
{
   ARGBImage im0, im1;
   if(!info0.image_filename.empty() and !info1.image_filename.empty()) {
      im0 = ARGBImage::load(get_fname(info0.image_filename));
      im1 = ARGBImage::load(get_fname(info1.image_filename));
   } else {
      WARN(format("no image specified, so not drawing results"));
      return;
   }

   for(auto i = 0u; i < Ws.size(); ++i) {
      const auto& x0 = raw_pairs[i][0];
      const auto& x1 = raw_pairs[i][1];

      const auto label = names[i];

      const auto Y  = et_opt.apply(Ws[i]);
      const auto y0 = bcam.project(CAM0, Y);
      const auto y1 = bcam.project(CAM1, Y);

      const auto o = Point2(2, -7);

      bresenham(x0, y0, [&](int x, int y) { set(im0, x, y, k_yellow); });
      bresenham(x1, y1, [&](int x, int y) { set(im1, x, y, k_yellow); });

      draw_cross(im0, to_pt2(x0), k_red, 5);
      draw_cross(im0, to_pt2(y0), k_blue, 5);
      render_string(im0, label, to_pt2(x0) + o, k_yellow, k_black);

      draw_cross(im1, to_pt2(x1), k_red, 5);
      draw_cross(im1, to_pt2(y1), k_blue, 5);
      render_string(im1, label, to_pt2(x1) + o, k_yellow, k_black);
   }

   im0.save(format(
       "/tmp/stereo-cam-opt_{}_{}", prefix, basename(info0.image_filename)));
   im1.save(format(
       "/tmp/stereo-cam-opt_{}_{}", prefix, basename(info1.image_filename)));
}

// ----------------------------------------------------------------

static void refine_e(const vector<array<Vector2, 2>>& correspondences,
                     Quaternion& q0,
                     Vector3& t0)
{
   auto pack = [&](real* X) {
      Vector3 s   = cartesian_to_spherical(t0);
      Vector3 saa = quaternion_to_saa(q0);
      for(auto i = 0; i < 2; ++i) *X++ = s[i];
      for(auto i = 0; i < 3; ++i) *X++ = saa[i];
   };

   Vector3 t, saa;
   auto unpack = [&](const real* X) {
      Vector3 s(0, 0, 0);
      for(auto i = 0; i < 2; ++i) s[i] = *X++;
      for(auto i = 0; i < 3; ++i) saa[i] = *X++;
      t = spherical_to_cartesian(s.x, s.y, 1.0);
   };

   auto fn = [&](const real* X) -> real {
      unpack(X);
      const Matrix3r Tx = make_skew_symmetric(t);
      const Matrix3r R  = saa_to_rot3x3(saa);
      const Matrix3r E  = Tx * R;
      auto err          = 0.0;
      for(const auto& rl : correspondences)
         err += calibration::xFl_lFx(E, rl[0], rl[1]);
      return err / real(correspondences.size());
   };

   const auto n_params = 5;

   const bool feedback        = true;
   const bool use_nelder_mead = false;
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
      cout << format("opt R,t") << endl;
      cout << format("   iterations:           {}", icount) << endl;
      cout << format("   fault-code:           {}", ifault) << endl;
      auto msg = (use_nelder_mead) ? nelder_mead_fault_str(ifault)
                                   : levenberg_marquardt_fault_str(ifault);
      cout << format("   fault-string:         {}", msg) << endl;
      cout << endl;
      cout << format("   initial-score:        {}", ystartlo) << endl;
      cout << format("   final-score:          {}", ynewlo) << endl;
      cout << endl;
      cout << format("   t = {}", str(t)) << endl;
      cout << format("   R = {}", saa_to_quaternion(saa).to_readable_str())
           << endl;
      cout << endl;
   }

   t0 = t;
   q0 = saa_to_quaternion(saa);
}

// --------------------------------------------------------------- Cost Function

static real reproj_et(const BinocularCamera& bcam,
                      const vector<Vector3>& Ws,
                      const vector<array<Vector2, 2>> raw_pairs,
                      const EuclideanTransform& et_opt)
{
   auto err = 0.0;
   for(auto i = 0u; i < Ws.size(); ++i) {
      const auto Y   = et_opt.apply(Ws[i]); // world to cam
      const auto& x0 = raw_pairs[i][0];
      const auto& x1 = raw_pairs[i][1];
      const auto y0  = bcam.project(CAM0, Y);
      const auto y1  = bcam.project(CAM1, Y);
      err += 0.5 * ((x0 - y0).norm() + (x1 - y1).norm());
   }
   return err / real(Ws.size());
}

// --------------------------------------------------------------- Cost Function

static real planar_reproj_et(const BinocularCamera& bcam,
                             const vector<Vector3>& Ws,
                             const vector<Plane>& P3s,
                             const vector<array<Vector2, 2>> raw_pairs,
                             const EuclideanTransform& et)
{
   real err = 0.0;

   const auto C0  = et.inverse_apply(bcam.C(CAM0));
   const auto C1  = et.inverse_apply(bcam.C(CAM1));
   const auto& M0 = bcam.model(CAM0);
   const auto& M1 = bcam.model(CAM1);

   // for(unsigned i{0}; i < Ws.size(); ++i) {
   //    const auto& W  = Ws[i];           // The world point
   //    const auto& d0 = raw_pairs[i][0]; // left distorted
   //    // Rays for individual camera's points of view
   //    const auto c_ray0 = homgen_R2_to_P2(M0.undistort(CAM0,
   //    d0)).normalised(); const auto c_p0   = bcam.project(CAM0, ray0);

   //    // Rotate into world coordinates
   //    const auto ray0 = et.rotation.apply(c_ray0);
   //    const auto p0   = bcam.project(CAM0, ray0);
   // }

   return err / real(Ws.size());
}

static real planar_reproj_et_(const BinocularCamera& bcam,
                              const vector<Vector3>& Ws,
                              const vector<Plane>& P3s,
                              const vector<array<Vector2, 2>> raw_pairs,
                              const EuclideanTransform& et)
{
   using Matrix4r = Eigen::Matrix<real, 4, 4>;

   const auto C0 = bcam.C(0);
   const auto C1 = bcam.C(1);

   const auto& C = et.translation;
   const auto& q = et.rotation; // world to cam

   const Matrix4r T       = make_transform_matrix(et);
   const Matrix4r T_inv_t = T.inverse().transpose();

   const auto N = P3s.size();
   auto err     = 0.0;
   for(auto i = 0u; i < N; ++i) {
      // Image points (distorted)
      const auto& d0 = raw_pairs[i][0];
      const auto& d1 = raw_pairs[i][1];

      const auto u0 = bcam.model(CAM0).undistort(d0);
      const auto u1 = bcam.model(CAM1).undistort(d1);

      const auto A = bcam.solve3d_from_undistorted(u0, u1);
      const auto B = et.inverse_apply(A);

      const auto W_ = et.apply(Ws[i]);
      // const auto x0
      //     = homgen_P2_to_R2(bcam.to_ray(CAM0, bcam.project(CAM0, W_)));
      // const auto x1
      //     = homgen_P2_to_R2(bcam.to_ray(CAM1, bcam.project(CAM1, W_)));

      // World point
      const auto& W   = Ws[i];
      const auto& p3w = P3s[i]; // world

      // Camera points
      const auto p3c = to_vec4(T_inv_t * to_vec4r(p3w));

      // Rays, in camera coords, generated from undistorted image points
      const auto r0 = homgen_R2_to_P2(u0).normalised();

      // Project 'Z' into the right camera
      const auto y0 = bcam.project(CAM0, W_);
      const auto y1 = bcam.project(CAM1, W_);

      // const auto yu0 = homgen_R2_to_P2(bcam.model(CAM0).undistort(y0));

      // Ray intersection with plane in camera coords
      const auto Z  = plane_ray_intersection(p3c, Vector3(0, 0, 0), r0);
      const auto Z_ = et.inverse_apply(Z);

      const auto z1 = bcam.project(CAM1, Z);

      const auto dz_err = (d1 - z1).norm();
      err += dz_err;

      cout << "----------------------------------" << endl;
      cout << format("#{:2d}, p3-dist={:5.3f}, "
                     "C=[{:5.3f}, {:5.3f}, {:5.3f}], "
                     "W=[{:5.3f}, {:5.3f}, {:5.3f}], dz-err = {}",
                     i,
                     p3w.point_plane_distance(W),
                     C.x,
                     C.y,
                     C.z,
                     W.x,
                     W.y,
                     W.z,
                     dz_err)
           << endl;
      cout << format(
          "Y = {}, dist = {}", str(et.apply(Ws[i])), p3c.side(et.apply(Ws[i])))
           << endl;
      cout << format("Z = {}", str(Z)) << endl;
      cout << format("Z'= {}", str(Z_)) << endl;
      cout << format("A = {}", str(A)) << endl;
      cout << format("B = {}", str(B)) << endl;

      cout << format("d0 = {}", str(d0)) << endl;
      cout << format("d1 = {}", str(d1)) << endl;
      cout << format("u0 = {}", str(u0)) << endl;
      cout << format("u1 = {}", str(u1)) << endl;
      cout << format("y0 = {}", str(y0)) << endl;
      cout << format("y1 = {}", str(y1)) << endl;
      cout << format("z1 = {}", str(z1)) << endl;
      cout << endl;
   }

   if(true) {
      cout << format("et = {}", str(et)) << endl;
      cout << format("cam-C = {}", str(et.translation)) << endl;
      cout << format("p-ray = {}", str(et.rotation.rotate(Vector3(0, 0, 1))))
           << endl;

      FATAL("kBAM!");
   }

   return err / real(N);
}

// ----------------------------------------------- Apply non-linear optimization

static real apply_nonlinear_opt(BinocularCamera& bcam,
                                const vector<Vector3>& Ws,
                                const vector<Plane>& P3s, // one per Ws
                                const vector<array<Vector2, 2>> raw_pairs,
                                const EuclideanTransform& etC0,
                                const bool use_planar_cost_function,
                                EuclideanTransform& et_opt)
{
   Expects(use_planar_cost_function == false);

   const auto n_params = 7;
   et_opt              = etC0;
   auto fn             = [&](const real* X) -> real {
      et_opt.unpack(X);
      bcam.set_baseline(et_opt.scale);
      et_opt.scale = 1.0;

      return use_planar_cost_function
                             ? planar_reproj_et(bcam, Ws, P3s, raw_pairs, et_opt)
                             : reproj_et(bcam, Ws, raw_pairs, et_opt);
   };

   const bool feedback        = false;
   const bool use_nelder_mead = false;
   vector<real> start(n_params);
   vector<real> xmin(n_params);
   real ynewlo   = dNAN;
   real ystartlo = dNAN;
   real reqmin   = 1e-7;
   real diffstep = 0.1;
   int kcount    = 100000; // max interations
   int icount = 0, numres = 0, ifault = 0;
   const char* method = nullptr;

   et_opt.pack(&start[0]);
   start[6] = bcam.baseline();
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

   et_opt.unpack(&xmin[0]);
   et_opt.scale = 1.0;

   if(feedback) {
      cout << format("et_opt    {}", str(et_opt)) << endl;
      cout << format("et_opt^-1 {}", str(et_opt.inverse())) << endl;
      cout << format("   iterations:           {}", icount) << endl;
      cout << format("   fault-code:           {}", ifault) << endl;
      auto msg = (use_nelder_mead) ? nelder_mead_fault_str(ifault)
                                   : levenberg_marquardt_fault_str(ifault);
      cout << format("   fault-string:         {}", msg) << endl;
      cout << endl;
      cout << format("   initial-score:        {}", ystartlo) << endl;
      cout << format("   final-score:          {}", ynewlo) << endl;
      cout << format("   final baseline:       {}", bcam.baseline()) << endl;
      cout << endl;
   }

   return ynewlo;
}

// ----------------------------------------------------------- Optimize Baseline

// static real optimize_bcam_baseline(BinocularCameraInfo& bcam_info)
// {
//    reproj_error = reproj_et(bcam, Ws, raw_pairs, etC0);
// }

// ---------------------------------------------------------------- Process Bcam

static EuclideanTransform process_bcam(const EstimateCamExtrinsicInfo& ex_info,
                                       const SceneDescriptionInfo& scene_info,
                                       BinocularCameraInfo& bcam_info,
                                       const bool do_refine_e)
{
   BinocularCamera bcam;
   Matrix3r K      = Matrix3r::Identity();
   const auto hfov = to_radians(80.0);
   const auto f    = 400.0;
   const auto w    = 800.0;
   const auto h    = 2.0 * f * tan(0.5 * hfov);
   K(0, 0) = K(1, 1)          = f;
   K(0, 2)                    = 0.5 * w;
   K(1, 2)                    = 0.5 * h;
   const unsigned distorted_w = unsigned(ex_info.sensors[0].image_format.x);
   const unsigned distorted_h = unsigned(ex_info.sensors[0].image_format.y);

   const bool use_planar_cost_function = false;

   bcam.init(
       bcam_info, distorted_w, distorted_h, K, unsigned(w), unsigned(h), false);
   bcam.set_working_format(distorted_w, distorted_h);

   const auto C0 = bcam_info.C0();
   const auto C1 = bcam_info.C1();

   // (*) Find corresponding points in ex-info
   // (*) Solve corresponding points to 3D
   // (*) Find the rigid-body translation/rotation to place camera C0
   // (*) Check, check, and check

   const auto& sensor0 = bcam_info.M[0].sensor_id();
   const auto& sensor1 = bcam_info.M[1].sensor_id();
   const auto ind0     = find_sensor_index(ex_info, sensor0);
   const auto ind1     = find_sensor_index(ex_info, sensor1);

   if(ind0 == -1 or ind1 == -1) {
      if(ind0 == -1)
         cout << format("Failed to find sensor '{}' in annotated data", sensor0)
              << endl;
      if(ind1 == -1)
         cout << format("Failed to find sensor '{}' in annotated data", sensor1)
              << endl;
      throw std::runtime_error("failed to find sensor");
   }

   const auto& info0        = ex_info.sensors[size_t(ind0)];
   const auto& info1        = ex_info.sensors[size_t(ind1)];
   const auto& coord_names0 = info0.coord_names;
   const auto& coord_names1 = info1.coord_names;

   // Get corresponding pairs common between sensors
   vector<Point2> corresp_inds;
   for(auto i = 0u; i < coord_names0.size(); ++i) {
      const auto& label0 = coord_names0[i];
      auto jj = std::find(cbegin(coord_names1), cend(coord_names1), label0);
      if(jj == cend(coord_names1)) continue;
      auto j             = std::distance(cbegin(coord_names1), jj);
      const auto& label1 = coord_names1[size_t(j)];
      corresp_inds.emplace_back(int(i), int(j));
   }

   // Get the set of corresponding points
   vector<Vector3> Ws;
   vector<Plane> P3s;
   vector<string> P3_names;
   vector<string> names;
   vector<array<Vector2, 2>> raw_pairs;
   vector<array<Vector2, 2>> correspondences;
   for(const auto& rl_inds : corresp_inds) {
      const auto& n0 = info0.coord_names[size_t(rl_inds.x)];
      const auto& n1
          = info1.coord_names[size_t(rl_inds.y)]; // should have n0 == n1
      if(n0 != n1) FATAL(format("n0 = {}, n1 = {}", n0, n1));

      const auto& W0 = info0.world_coords[size_t(rl_inds.x)];
      const auto& W1
          = info1.world_coords[size_t(rl_inds.y)]; // should have W0 == W1
      if((W0 - W1).norm() > 1e-9)
         FATAL(format("W0 = {}, but W1 = {}", str(W0), str(W1)));

      // Which plane does 'W0' sit on?
      if(use_planar_cost_function || true) {
         auto ii
             = std::find_if(cbegin(ex_info.planes),
                            cend(ex_info.planes),
                            [&](const auto& p3info) {
                               return p3info.p3.point_plane_distance(W0) < 1e-9;
                            });
         if(ii == cend(ex_info.planes)) {
            LOG_ERR(
                format("failed to find a plane for 3D point W = {}", str(W0)));
            cout << format(" -> printing planes: ") << endl;
            for(const auto& p3info : ex_info.planes) {
               cout << format("    plane '{}': {}, distance = {}",
                              p3info.name,
                              str(p3info.p3),
                              p3info.p3.point_plane_distance(W0))
                    << endl;
            }
         } else {
            P3s.push_back(ii->p3);
            P3_names.push_back(ii->name);
         }
      }

      const auto x0 = info0.image_coords[size_t(rl_inds.x)];
      const auto x1 = info1.image_coords[size_t(rl_inds.y)];
      const auto u0 = bcam.model(0).undistort(x0);
      const auto u1 = bcam.model(1).undistort(x1);

      Ws.push_back(W0);
      names.push_back(n0);
      raw_pairs.push_back(array<Vector2, 2>{{x0, x1}});
      correspondences.push_back(array<Vector2, 2>{{u0, u1}});
   }

   // Refine rotation and translation if we mean to
   if(do_refine_e) refine_e(correspondences, bcam_info.q, bcam_info.t);

   // Make 'E'
   const Matrix3r Tx = make_skew_symmetric(bcam_info.t.normalised());
   const Matrix3r R  = quaternion_to_rot3x3(bcam_info.q);
   const Matrix3r E  = Tx * R;

   // Resolve initial positions of 'X'
   vector<Vector3> Xs;
   print2(g_bullet, format("Printing EPIPOLAR errors"));
   cout << format("   If the binocular (and distortion) calibration are\n"
                  "   correct, then these errors will be in the ballpark\n"
                  "   of the error of the input points. That is, the mouse\n"
                  "   error from clicking on a point in the point-picking\n"
                  "   GUI. About 5 pixels or less.\n");
   for(auto i = 0u; i < raw_pairs.size(); ++i) {
      const auto& x0 = raw_pairs[i][0];
      const auto& x1 = raw_pairs[i][1];
      const auto& u0 = correspondences[i][0];
      const auto& u1 = correspondences[i][1];

      // Rotate u0 and u1 into rays in camera space
      const auto r0 = homgen_R2_to_P2(u0).normalised();
      const auto r1
          = bcam_info.q.inverse_rotate(homgen_R2_to_P2(u1).normalised());

      // Intersect
      const auto X = intersect_rays_2(C0, C0 + r0, C1, C1 + r1);

      auto e_err = calibration::xFl_lFx(E, u0, u1);

      const bool is_bad = 400.0 * e_err > 5.0;

      if(use_planar_cost_function) {
         cout << format("   {}={}, on {}, p0=[{},{}], p1=[{},{}], "
                        "epipolar-err = {}",
                        coord_names0[i],
                        str(Ws[i]),
                        P3_names[i],
                        x0.x,
                        x0.y,
                        x1.x,
                        x1.y,
                        400.0 * e_err);
      } else {
         cout << format("   {}={}, p0=[{},{}], p1=[{},{}], "
                        "epipolar-err = {}",
                        coord_names0[i],
                        str(Ws[i]),
                        x0.x,
                        x0.y,
                        x1.x,
                        x1.y,
                        400.0 * e_err);
      }

      if(is_bad) { cout << " - \x1b[31mWARNING\x1b[0m"; }

      cout << endl;

      Xs.emplace_back(X);
   }

   // Find the rotation and translation between Ws[0] and Xs[1]
   if(false) {
      auto et0 = transform_between(Ws, Xs);
      cout << format("et0 = {}", str(et0)) << endl;
      draw_position_result(
          "et0", Ws, raw_pairs, names, bcam, et0, info0, info1);
   }

   // ----------------------------------------------------------------- Get etC

   EuclideanTransform etC0, etC1;

   if(true) {
      // Get the distorted points
      vector<Vector2> xs0(raw_pairs.size());
      vector<Vector2> xs1(raw_pairs.size());
      std::transform(cbegin(raw_pairs),
                     cend(raw_pairs),
                     begin(xs0),
                     [&](const auto& pq) { return pq[0]; });
      std::transform(cbegin(raw_pairs),
                     cend(raw_pairs),
                     begin(xs1),
                     [&](const auto& pq) { return pq[1]; });

      Vector3 C0 = ex_info.sensors[size_t(ind0)].estimated_center;
      if(!C0.is_finite()) C0 = Vector3(5.0, 5.0, 2.5);
      Vector3 C1 = ex_info.sensors[size_t(ind1)].estimated_center;
      if(!C1.is_finite()) C1 = Vector3(5.0, 5.0, 2.5);

      // Get the undistorted (normalized) points
      const auto& M0 = bcam_info.M[0];
      const auto& M1 = bcam_info.M[1];
      vector<Vector2> U0s(raw_pairs.size());
      vector<Vector2> U1s(raw_pairs.size());
      std::transform(cbegin(xs0), cend(xs0), begin(U0s), [&](const auto& D) {
         return M0.undistort(D);
      });
      std::transform(cbegin(xs1), cend(xs1), begin(U1s), [&](const auto& D) {
         return M1.undistort(D);
      });

      auto reproj_error = 0.0;
      print2(g_bullet, format("initial global [R|t] for {}", sensor0));
      auto etC0_ = estimate_et_one_cam(M0, C0, Ws, xs0, reproj_error, false);
      print2(g_bullet, format("initial global [R|t] for {}", sensor1));
      auto etC1_ = estimate_et_one_cam(M1, C1, Ws, xs1, reproj_error, false);

      print2(g_bullet, format("refining global [R|t] from {}", sensor0));
      etC0           = optimize_et_one_cam(etC0_, Ws, U0s);
      auto filename0 = format("/tmp/one-cam-opt_{}.png", sensor0);
      draw_one_cam_result(M0, Ws, etC0, names, xs0, info0, filename0);
      print2(g_bullet,
             format("Saving per-sensor optimization image to: {}", filename0));

      print2(g_bullet, format("refining global [R|t] from {}", sensor1));
      etC1           = optimize_et_one_cam(etC1_, Ws, U1s);
      auto filename1 = format("/tmp/one-cam-opt_{}.png", sensor1);
      draw_one_cam_result(M1, Ws, etC1, names, xs1, info1, filename1);
      print2(g_bullet,
             format("Saving per-sensor optimization image to: {}", filename1));

      const real baseline
          = (etC1.apply(Vector3(0, 0, 0)) - etC0.apply(Vector3(0, 0, 0)))
                .norm();
      if(false) {
         print2(g_bullet, format("setting baseline to {}", baseline));
         bcam_info.baseline = baseline;
         bcam.init(bcam_info,
                   distorted_w,
                   distorted_h,
                   K,
                   unsigned(w),
                   unsigned(h),
                   false);
         bcam.set_working_format(distorted_w, distorted_h);
      }

      reproj_error = reproj_et(bcam, Ws, raw_pairs, etC0);

      if(false) {
         // etC0 = get_et(Ws, U0s, U1s, bcam_info.euclid_transform(), bcam);
         // auto reproj_error = reproj_et(bcam, Ws, raw_pairs, etC0);
         // Vector3 C0 = ex_info.sensors[ind0].estimated_center;
         // if(!C0.is_finite())
         //     C0 = Vector3(5.0, 5.0, 3.0);

         // real reproj_error = 0.0;
         // etC0 = estimate_camera_extrinsics(M0, C0, Ws, xs0,
         //                                   reproj_error, false)
         //     .inverse();
      }

      print2(g_info,
             format("initial euclidean transform inv is {}", str(etC0)));

      print2(g_info,
             format("initial position at {}, reproj-err = {}",
                    str(etC0.inverse().translation),
                    reproj_error));

      // draw_one_cam_result(M0, Ws, etC0, names, xs0, info0);
      // draw_one_cam_result(M1, Ws, etC1, names, xs1, info1);

   } else {
      // C1001
      // [4.47378, 1.81039, 2.15950], rot = [0.95917, -0.24053, 0.14878], θ =
      // 246.180546 degrees, err = 21.668381
      etC0.translation = Vector3{4.48233, 1.86241, 2.17744};
      Vector4 aa0{0.95917, -0.24053, 0.14878, to_radians(246.180546)};
      etC0.rotation = axis_angle_to_quaternion(aa0);
      etC0          = etC0.inverse();

      // C1011
      // [4.44284, 7.21389, 2.38577], rot = [-0.26967, 0.78095, -0.56338], θ =
      // 193.819011 degrees, err = 4.246124

      etC0.translation = Vector3{0.35617, 0.64795, -0.69734};
      aa0 = Vector4{-0.21224, 0.10876, -0.97115, to_radians(272.822410)};
      etC0.rotation = axis_angle_to_quaternion(aa0);
      etC0          = etC0.inverse();

      // C1001
      // [4.24273, 2.10294, 2.10792], rot = [0.94998, -0.25704, 0.17740], θ =
      // 244.846396 degrees, err = 1.090707
      etC0.translation = Vector3{4.24273, 2.10294, 2.10792};
      aa0 = Vector4{0.94998, -0.25704, 0.17740, to_radians(244.846396)};
      etC0.rotation = axis_angle_to_quaternion(aa0);
      etC0          = etC0.inverse();

      // Get the distorted points
      vector<Vector2> xs0(raw_pairs.size());
      std::transform(cbegin(raw_pairs),
                     cend(raw_pairs),
                     begin(xs0),
                     [&](const auto& pq) { return pq[0]; });
      const auto& M0 = bcam_info.M[0];
      // draw_one_cam_result(M0, Ws, etC0, names, xs0, info0);
   }

   draw_position_result("etC0", Ws, raw_pairs, names, bcam, etC0, info0, info1);

   if(false) {
      auto show_it = [&](const string& name, const EuclideanTransform& etC) {
         cout << std::string(40, '-') << " " << name << endl;

         for(auto i = 0u; i < Ws.size(); ++i) {
            const auto& x0 = raw_pairs[i][0];
            const auto& x1 = raw_pairs[i][1];
            const auto& u0 = correspondences[i][0];
            const auto& u1 = correspondences[i][1];
            const auto W   = Ws[i];
            const auto X   = Xs[i];

            const auto Y  = etC.apply(W);
            const auto y0 = bcam.project(CAM0, Y);
            const auto y1 = bcam.project(CAM1, Y);

            cout << format("{} l:|{} - {}| = {}, r:|{} - {}| = {}",
                           name,
                           str(x0),
                           str(y0),
                           (x0 - y0).norm(),
                           str(x1),
                           str(y1),
                           (x1 - y1).norm())
                 << endl;

            cout << endl;
         }
      };

      show_it("etC0", etC0);
   }

   // The back-projected kite should be a rectangle if the
   // four points are a planar rectangle. Tests the distortion calibration.
   if(false) {
      for(size_t j = 0; j < 2; ++j) {
         back_project_kite(homgen_R2_to_P2(correspondences[0][j]),
                           homgen_R2_to_P2(correspondences[1][j]),
                           homgen_R2_to_P2(correspondences[2][j]),
                           homgen_R2_to_P2(correspondences[3][j]),
                           1.0);
      }
   }

   // ------------------------------------------- Apply non-linear optimization

   auto print_result
       = [&](const int counter, const EuclideanTransform& et_opt, real ynewlo) {
            const auto e_inv = et_opt.inverse();
            const auto aa    = quaternion_to_axis_angle(e_inv.rotation);

            auto msg = format("#{:3d}, t = {}, rot = {}, "
                              "\u03b8 = {} degrees, err = {}",
                              counter,
                              str(e_inv.translation),
                              str(aa.xyz()),
                              to_degrees(aa[3]),
                              ynewlo);

            print2(g_info, msg);
         };

   //  WARN(format("ZZZZ etC0 = {}", str(etC0)));

   EuclideanTransform et_opt;
   auto ynewlo = apply_nonlinear_opt(
       bcam, Ws, P3s, raw_pairs, etC0, use_planar_cost_function, et_opt);
   print_result(0, et_opt, ynewlo);

   for(auto attempt_no = 0; attempt_no < 10; ++attempt_no) {
      EuclideanTransform et_attempt;
      EuclideanTransform e = et_opt;
      for(auto i = 0; i < 3; ++i) e.translation[i] += (0.5 - uniform());

      auto saa = quaternion_to_saa(e.rotation);
      for(auto i = 0; i < 3; ++i)
         saa[i] += to_radians(25.0 * (0.5 - uniform()));
      e.rotation = saa_to_quaternion(saa);

      auto lo = apply_nonlinear_opt(
          bcam, Ws, P3s, raw_pairs, e, use_planar_cost_function, et_attempt);
      if(lo < ynewlo) {
         ynewlo = lo;
         et_opt = et_attempt;
         print_result(attempt_no + 1, et_attempt, lo);
      }
   }

   // WARN(format("et-opt = {}", str(et_opt)));

   draw_position_result(
       "et-opt", Ws, raw_pairs, names, bcam, et_opt, info0, info1);

   { // Print out the 3D locations of all the found points
      for(auto i = 0u; i < Ws.size(); ++i) {
         const auto label = names[i];
         const auto Y     = et_opt.apply(Ws[i]);
         cout << format(" >> {}, |{}| = {}", label, Y.norm(), str(Y)) << endl;
      }
   }

   // cout << format("RESULT: {}", bcam_info.camera_id) << endl;
   // cout << json_encode(et_opt.inverse()) << endl;

   if(scene_info.cad_model_key != "") {
      Expects(bcam_info.n_sensors() == 2);

      const auto et0 = et_opt.inverse();
      const auto et1 = bcam_info.make_et1(et0);

      const auto fbo_w    = 800;
      const auto fbo_h    = 600;
      const auto fbo_hfov = to_radians(80.0);

      auto sprite_ptr = make_shared<Sprite>();
      fetch(*sprite_ptr, scene_info.cad_model_key);
      sprite_ptr->apply_transform(scene_info.cad_model_transform);

      for(auto i = 0; i < bcam_info.n_sensors(); ++i) {
         const auto& info = (i == 0) ? info0 : info1;

         if(info.image_filename.empty()) continue;

         auto im = ARGBImage::load(get_fname(info.image_filename));

         auto fname = format("/tmp/zzz-blended_{}_{}_{}.png",
                             bcam_info.camera_id,
                             i,
                             bcam_info.M[size_t(i)].sensor_id());

#ifndef USING_OPENGL
         INFO(format("no OpenGl, so not saving blended image"));
#else
         print2(g_info, format("blended image saved to {}", fname));

         auto argb = blend_im_with_CAD(im,
                                       bcam_info.M[size_t(i)],
                                       (i == 0 ? et0 : et1),
                                       *sprite_ptr,
                                       fbo_w,
                                       fbo_h,
                                       fbo_hfov);
         argb.save(fname);
#endif
      }
   }

   return et_opt.inverse();
}

// ------------------------------------------------------ run stereo cam pos opt

bool run_stereo_cam_pos_opt(const string& manifest_file,
                            const bool update_E,
                            const string& out_file)
{
   if(!is_regular_file(manifest_file)) {
      cout << format("Failed to find manifest file '{}'", manifest_file)
           << endl;
      return false;
   }

   if(out_file == manifest_file) {
      cout << format("Output file cannot be input file!") << endl;
      return false;
   }

   // ---- Process command line

   EstimateCamExtrinsicInfo ex_info;
   try {
      load(ex_info, manifest_file);
      print(g_bullet_big, format("loaded file '{}'", basename(manifest_file)));
   } catch(std::runtime_error& e) {
      cout << format(
          "error loading ex-cam-info file '{}': {}", manifest_file, e.what())
           << endl;
      return false;
   }

   // -- Load scene

   SceneDescriptionInfo scene_info;
   try {
      fetch(scene_info, ex_info.scene_key);
      print(g_bullet_big, format("loaded scene '{}'", ex_info.scene_key));
   } catch(std::runtime_error& e) {
      cout << format(
          "error loading scene-info '{}': {}", ex_info.scene_key, e.what())
           << endl;
      return false;
   }

   // -- Process each bcam

   std::vector<EuclideanTransform> ets;
   for(const auto& bcam_key : scene_info.bcam_keys) {
      cout << format("   \x1b[97m{}\x1b[0m {}", string(50, '-'), bcam_key)
           << endl;

      BinocularCameraInfo bcam_info;
      try {
         fetch(bcam_info, bcam_key);
      } catch(std::runtime_error& e) {
         cout << format("failed to load bcam '{}': {}", bcam_key, e.what())
              << endl;
         continue;
      }

      auto et = process_bcam(ex_info, scene_info, bcam_info, update_E);
      ets.push_back(et);
   }

   // -- Produce output

   if(out_file != "") {
      scene_info.bcam_transforms = ets;
      save(scene_info, out_file);
      print(g_tick, format("ouput saved to '{}'", out_file));
   }

   print(g_victory, format("DONE!!!"));

   return true;
}

} // namespace perceive::calibration
