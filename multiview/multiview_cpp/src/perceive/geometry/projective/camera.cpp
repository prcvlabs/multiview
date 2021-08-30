
#include "camera.hpp"
#include "stdinc.hpp"

#include "perceive/io/json-io.hpp"
#include "perceive/utils/eigen-helpers.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/opencv-helpers.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define This Camera

namespace perceive
{
// ---------------------------------------------------------------- Construction

This::This()
{
   // std::fill(distort_params_.begin(), distort_params_.end(), 0.0);
   calc_derived();
}

// --------------------------------------------------------------------- File IO

void This::save(std::string& filename) const
{
   FILE* fp = fopen(filename.c_str(), "w");
   if(fp == nullptr) {
      throw std::runtime_error(
          format("Failed to open file for writing: '{}'", filename));
   }

   std::stringstream ss("");
   ss << json_save(*this) << std::endl;
   fprintf(fp, "%s", ss.str().c_str());

   fclose(fp);
}

Camera This::load(std::string& filename)
{
   std::string file_contents = file_get_contents(filename);
   const Json::Value root    = parse_json(file_contents);

   // std::ifstream fin;
   // Json::Reader reader(Json::Features::strictMode());
   // Json::Value root;
   std::string err = "";
   Camera cam;

   try {
      // fin.open(filename);
      // reader.parse(fin, root, false);
      json_load(root, cam);
   } catch(std::ifstream::failure& e) {
      // File I/O error
      err = format("File I/O error reading file '{}'", filename);
   } catch(std::runtime_error& e) {
      err = format("Error reading JSON file '{}': {}", filename, e.what());
   } catch(std::exception& e) {
      // JSON parse error
      err = format("Parse error reading JSON file '{}'", filename);
   }

   // try {
   //    fin.close();
   // } catch(...) {}

   if(err != "") throw std::runtime_error(err);

   return cam;
}

// ---------------------------------------------------------------- Calc Derived

void This::calc_derived()
{
   // Intrinsic
   K_(0, 0) = fx_;
   K_(1, 1) = fy_;
   K_(2, 2) = real(1.0);
   K_(0, 2) = ppt_.x;
   K_(1, 2) = ppt_.y;
   K_(0, 1) = K_(1, 0) = K_(2, 0) = K_(2, 1) = real(0.0);

   K_inv_  = K_.inverse();
   hfov_   = 2.0 * atan(0.5 * w_ / fx_);
   vfov_   = 2.0 * atan(0.5 * h_ / fy_);
   f_      = 0.5 * (fx_ + fy_);
   fx_inv_ = real(1.0) / fx_;
   fy_inv_ = real(1.0) / fy_;

   // Rotation
   R_                   = quaternion_to_rot3x3(q_);
   R_inv_               = R_.inverse();
   t_                   = -to_vec3(R_ * to_vec3r(C_));
   P_.block(0, 0, 3, 3) = K_ * R_;
   P_.block(0, 3, 3, 1) = K_ * to_vec3r(t_);
   KR_inv_              = R_inv_ * K_inv_;

   // The imaging plane
   {
      Vector3 n = to_vec3(R_inv_ * Vector3r(0, 0, 1));
      C_plane_  = Plane(n, -dot(C_, n));
   }

   // Will force recalculationg of mapx_ and mapy_ on demand
   dirty_ = true;
}

// ------------------------------------------------------------------- Is Finite

bool This::is_finite() const
{
   return w_ > 0 && h_ > 0 && std::isfinite(fx_) && fx_ > real(0.0)
          && std::isfinite(fy_) && fy_ > real(0.0) && ppt_.is_finite()
          && C_.is_finite() && q_.is_finite() && ::perceive::is_finite(K_)
          && ::perceive::is_finite(K_inv_) && std::isfinite(hfov_)
          && hfov_ > real(0.0) && std::isfinite(vfov_) && vfov_ > real(0.0)
          && std::isfinite(f_) && f_ > real(0.0) && std::isfinite(fx_inv_)
          && fx_inv_ > real(0.0) && std::isfinite(fy_inv_)
          && fy_inv_ > real(0.0) && ::perceive::is_finite(R_)
          && ::perceive::is_finite(R_inv_) && ::perceive::is_finite(KR_inv_)
          && t_.is_finite() && ::perceive::is_finite(P_);
}

// ------------------------------------------------------------------ Set format

void This::set_format(unsigned w, unsigned h)
{
   real scale_w = real(w) / this->w();
   real scale_h = real(h) / this->h();
   auto new_ppt = Vector2(ppt().x * scale_w, ppt().y * scale_h);
   set_intrinsic(w, h, fx() * scale_w, fy() * scale_h, new_ppt);
}

// --------------------------------------------------------------- Set Intrinsic

void This::set_intrinsic(unsigned w,
                         unsigned h,
                         real fx,
                         real fy,
                         const Vector2& ppt)
{
   w_   = w;
   h_   = h;
   fx_  = fx;
   fy_  = fy;
   ppt_ = ppt;
   calc_derived();
}

// --------------------------------------------------------------- Set Extrinsic

void This::set_extrinsic(const Vector3& C, const Quaternion& q)
{
   C_ = C;
   q_ = q;
   calc_derived();
}

void This::set_extrinsic(const Matrix3r& R, const Vector3& t)
{
   Matrix3r R_inv = R.inverse();
   Vector3r CC    = -R_inv * to_vec3r(t);
   C_             = to_vec3(CC);
   q_             = rot3x3_to_quaternion(R);
   calc_derived();
}

// ------------------------------------------------------------------- Normalize

Vector2 This::normalize(const Vector2& x) const // Image => normalized coords
{
   return Vector2((x.x - ppt_.x) * fx_inv_, (x.y - ppt_.y) * fy_inv_);
}

Vector2 This::unnormalize(const Vector2& x) const // Normalized => image coords
{
   return Vector2(x.x * fx_ + ppt_.x, x.y * fy_ + ppt_.y);
}

// --------------------------------------------------------------------- Project

Vector2 This::project(const Vector3& X) const // project (R3->R2)
{
   Vector2 x;
   x.x    = P_(0, 0) * X(0) + P_(0, 1) * X(1) + P_(0, 2) * X(2) + P_(0, 3);
   x.y    = P_(1, 0) * X(0) + P_(1, 1) * X(1) + P_(1, 2) * X(2) + P_(1, 3);
   real z = P_(2, 0) * X(0) + P_(2, 1) * X(1) + P_(2, 2) * X(2) + P_(2, 3);
   return x * (real(1.0) / z);
}

Vector3 This::project(const Vector4& X) const // project (P3->P2)
{
   Vector3 x;
   x.x = P_(0, 0) * X(0) + P_(0, 1) * X(1) + P_(0, 2) * X(2) + P_(0, 3) * X(3);
   x.y = P_(1, 0) * X(0) + P_(1, 1) * X(1) + P_(1, 2) * X(2) + P_(1, 3) * X(3);
   x.z = P_(2, 0) * X(0) + P_(2, 1) * X(1) + P_(2, 2) * X(2) + P_(2, 3) * X(3);
   return x.normalise_point();
}

// ---------------------------------------------------------------- Back-project

Vector3 This::back_project(const Vector2& X) const
{
   return to_vec3(KR_inv_ * Vector3r(X.x, X.y, 1.0)).normalised();
}

Vector3 This::back_project(const Vector3& X) const
{
   return to_vec3(KR_inv_ * to_vec3r(X)).normalised();
}

Vector3r This::back_project(const Vector3r& X) const
{
   Vector3r ray = KR_inv_ * X;
   ray /= ray.norm();
   return ray;
}

// ---------------------------------------------------------- Reprojection Error

real This::reproj_error_sq(const Vector2& x, const Vector3& X) const
{
   return quadrance(x - project(X));
}

real This::reproj_error(const Vector2& x, const Vector3& X) const
{
   return norm(x - project(X));
}

// -------------------------------------------------------------------- In Front

bool This::in_front(const Vector3& X) const
{
   return dot(C_plane_.xyz(), X) + C_plane_.w > 0.0;
}

// ------------------------------------------------------------------- To String

std::string This::to_string() const
{
   std::stringstream ss("");

   ss << format(R"V0G0N(
Camera

   w/h                      = [{}, {}]
   ppt                      = {}
   f/fx/fy                  = [{}, {}, {}]
   hfov/vfov (degrees)      = [{}, {}]
   camera centre            = {}
   rot-axis/theta (degrees) = {}, {}
   distortion params        = [{}]

   K = -----------------------------------
       | {:9.3f}, {:9.3f}, {:9.3f} |
       | {:9.3f}, {:9.3f}, {:9.3f} |
       | {:9.3f}, {:9.3f}, {:9.3f} |
       -----------------------------------

   Rt= -----------------------------------------------
       | {:9.3f}, {:9.3f}, {:9.3f} | {:9.3f} |
       | {:9.3f}, {:9.3f}, {:9.3f} | {:9.3f} |
       | {:9.3f}, {:9.3f}, {:9.3f} | {:9.3f} |
       -----------------------------------------------

   P = -----------------------------------------------
       | {:9.3f}, {:9.3f}, {:9.3f} | {:9.3f} |
       | {:9.3f}, {:9.3f}, {:9.3f} | {:9.3f} |
       | {:9.3f}, {:9.3f}, {:9.3f} | {:9.3f} |
       -----------------------------------------------


)V0G0N",
                w(),
                h(),
                ppt().to_string(),
                f(),
                fx(),
                fy(),
                to_degrees(hfov()),
                to_degrees(vfov()),
                C().to_string(),
                q().axis().to_string(),
                to_degrees(q().theta()),
                // implode(distort_params().begin(),
                //         distort_params().end(),", "),

                K_(0, 0),
                K_(0, 1),
                K_(0, 2),
                K_(1, 0),
                K_(1, 1),
                K_(1, 2),
                K_(2, 0),
                K_(2, 1),
                K_(2, 2),

                R_(0, 0),
                R_(0, 1),
                R_(0, 2),
                t_(0),
                R_(1, 0),
                R_(1, 1),
                R_(1, 2),
                t_(1),
                R_(2, 0),
                R_(2, 1),
                R_(2, 2),
                t_(2),

                P_(0, 0),
                P_(0, 1),
                P_(0, 2),
                P_(0, 3),
                P_(1, 0),
                P_(1, 1),
                P_(1, 2),
                P_(1, 3),
                P_(2, 0),
                P_(2, 1),
                P_(2, 2),
                P_(2, 3));

   return ss.str();
}

// -------------------------------------------------- homography between cameras

Matrix3r homography_between(const Matrix34r& Po, const Matrix34r& Pn)
{
   Matrix3r Ro = Po.block(0, 0, 3, 3);
   Matrix3r Rn = Pn.block(0, 0, 3, 3);
   Matrix3r H  = Rn * Ro.inverse();
   H /= std::cbrt(H.determinant());
   return H;
}

// ------------------------------------------------------ Find Rectified Cameras

void find_rectified_cameras(const Matrix3r& K1,
                            const Matrix3r& K2,
                            const Matrix3r& R,
                            const Vector3r& C1d,
                            const Vector3r& C2d,
                            unsigned w,
                            unsigned h,     // output size
                            Matrix34r& Pn1, // New camera matrix (cam 1)
                            Matrix34r& Pn2, // New camera matrix (cam 2)
                            Matrix3r& Kn,   // ouput K matrix (shared)
                            Matrix3r& H1_out,
                            Matrix3r& H2_out)
{
   Matrix3r R_inv  = R.inverse();
   Vector3 C1      = to_vec3(C1d);
   Vector3 C2      = to_vec3(C2d);
   double baseline = (C1 - C2).norm();

   Vector3 z1 = to_vec3(Vector3r(R.block(2, 0, 1, 3).transpose()));
   Vector3 z2 = to_vec3(Vector3r(R.block(2, 0, 1, 3).transpose()));

   // The XYZ co-ordinates of the new camera
   Vector3 X = (C2 - C1).normalised();
   Vector3 Y = cross(0.5 * (z1 + z2), X).normalised();
   Vector3 Z = cross(X, Y).normalised();

   Matrix3r Rn, Rn_inv;
   Rn << X(0), X(1), X(2), Y(0), Y(1), Y(2), Z(0), Z(1), Z(2);
   Rn_inv = Rn.inverse();

   if(Rn.determinant() < 0)
      FATAL(format("R1.determinant() = {}", Rn.determinant()));

   // Output has not been premultiplied by K
   Pn1.block(0, 0, 3, 3) = Rn;
   Pn1.block(0, 3, 3, 1) = Vector3r::Zero();
   Pn2.block(0, 0, 3, 3) = Rn;
   Pn2.block(0, 3, 3, 1) = Vector3r(-baseline, 0, 0);

   // Transformation matrices between two cameras
   Matrix3r H1 = Rn * K1.inverse();
   Matrix3r H2 = Rn * R_inv * K2.inverse();

   // Lets transform the corners of the target format:
   auto transform = [](const Matrix3r& H, double x, double y) {
      Vector3r a = H * Vector3r(x, y, 1.0);
      return Vector2(a(0) / a(2), a(1) / a(2));
   };

   // Lets find the bounding box for our new camera...
   array<Vector2, 4> format_rect = {{Vector2{0.0, 0.0},
                                     Vector2{real(w), 0.0},
                                     Vector2{real(w), real(h)},
                                     Vector2{0.0, real(h)}}};
   AABB aabb                     = AABB::minmax();
   for(const auto& p : format_rect) {
      aabb.union_point(transform(H1, p.x, p.y));
      aabb.union_point(transform(H2, p.x, p.y));
   }

   // Now find the translation and scaling that makes meets the
   // output size requirements...
   double scale = std::min(w / aabb.w(), h / aabb.h());
   Vector2 ppt  = 0.5 * Vector2(w, h) - aabb.centre() * scale;

   double f  = scale;
   double px = ppt.x; // 0.5 * double(w);
   double py = ppt.y; // 0.5 * double(h);
   Kn << f, 0, px, 0, f, py, 0, 0, 1;

   H1_out = Kn * H1;
   H2_out = Kn * H2;
}

void json_load(const Json::Value& node, Camera& cam)
{
   // Camera::DistortVec distort;
   Vector2 format;
   Vector2 ppt;
   Vector3 centre;
   Quaternion quaternion;
   real fx, fy;

   // loadT(get_key(node, "distortion"), &distort[0], distort.size());
   json_load(get_key(node, "format"), format);
   json_load(get_key(node, "ppt"), ppt);
   json_load(get_key(node, "centre"), centre);
   fx = load_numeric(get_key(node, "fx"));
   fy = load_numeric(get_key(node, "fy"));
   json_load(get_key(node, "quaternion"), quaternion);

   format = format.round();

   // cam.distort_params() = distort;
   cam.set_intrinsic(unsigned(format.x), unsigned(format.y), fx, fy, ppt);
   cam.set_extrinsic(centre, quaternion);
}

// ----------------------------------------------------------------- Save Camera

Json::Value json_save(const Camera& cam)
{
   Json::Value node(Json::objectValue);

   node["format"]     = json_save(Vector2(cam.w(), cam.h()));
   node["ppt"]        = json_save(cam.ppt());
   node["fx"]         = cam.fx();
   node["fy"]         = cam.fy();
   node["centre"]     = json_save(cam.C());
   node["quaternion"] = json_save(cam.q());

   return node;
}

} // namespace perceive
