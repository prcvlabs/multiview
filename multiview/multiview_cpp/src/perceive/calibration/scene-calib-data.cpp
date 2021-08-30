
#include "scene-calib-data.hpp"

namespace perceive::calibration
{
// --------------------------------------------------------------- LabelledPoint
//
string LabelledPoint::to_string() const noexcept
{
   Json::StyledWriter writer;
   return writer.write(to_json());
}

string str(const LabelledPoint& o) noexcept { return o.to_string(); }

Json::Value LabelledPoint::to_json() const noexcept
{
   Json::Value x;
   x["label"]     = json_save(label);
   x["camera_no"] = json_save(camera_no);
   x["x"]         = json_save(this->x);
   x["on_floor"]  = json_save(on_floor);
   x["is_origin"] = json_save(is_origin);
   return x;
}

bool LabelledPoint::read(const Json::Value& x) noexcept
{
   LabelledPoint o;
   try {
      const auto op = "reading LabelledLine";
      o.label       = json_load_key<int>(x, "label", op);
      o.camera_no   = json_load_key<int>(x, "camera_no", op);
      o.x           = json_load_key<Vector2f>(x, "x", op);
      o.on_floor    = json_load_key<bool>(x, "on_floor", op);
      o.is_origin   = json_load_key<bool>(x, "is_origin", op);
   } catch(std::exception& e) {
      LOG_ERR(format("{}", e.what()));
      return false;
   }
   *this = o;
   return true;
}

// ---------------------------------------------------------------- LabelledLine
//
string LabelledLine::to_string() const noexcept
{
   Json::StyledWriter writer;
   return writer.write(to_json());
}

string str(const LabelledLine& o) noexcept { return o.to_string(); }

Json::Value LabelledLine::to_json() const noexcept
{
   Json::Value x;
   x["label"]     = json_save(label);
   x["camera_no"] = json_save(camera_no);
   x["x0"]        = json_save(x0);
   x["x1"]        = json_save(x1);
   x["axis"]      = json_save(int(axis));
   x["on_floor"]  = json_save(on_floor);
   return x;
}

bool LabelledLine::read(const Json::Value& x) noexcept
{
   LabelledLine o;
   try {
      const auto op = "reading LabelledLine";
      o.label       = json_load_key<int>(x, "label", op);
      o.camera_no   = json_load_key<int>(x, "camera_no", op);
      o.x0          = json_load_key<Vector2f>(x, "x0", op);
      o.x1          = json_load_key<Vector2f>(x, "x1", op);
      o.axis = static_cast<decltype(o.axis)>(json_load_key<int>(x, "axis", op));
      o.on_floor = json_load_key<bool>(x, "on_floor", op);

      if(o.axis >= NONE && o.axis <= Z_AXIS) {
         // all good
      } else {
         throw std::runtime_error(
             format("read invalid {}, while {}", o.axis, op));
      }
   } catch(std::exception& e) {
      LOG_ERR(format("{}", e.what()));
      return false;
   }
   *this = o;
   return true;
}

static constexpr std::array<Vector3f, 4> k_axes_v_ = {Vector3f{0.0, 0.0, 0.0},
                                                      Vector3f{1.0, 0.0, 0.0},
                                                      Vector3f{0.0, 1.0, 0.0},
                                                      Vector3f{0.0, 0.0, 1.0}};

static constexpr std::array<const char*, 4> k_axes_s_ = {"?", "X", "Y", "Z"};

const string_view str(LabelledLine::Axis axis) noexcept
{
   const size_t ind = size_t(axis);
   return (ind < k_axes_s_.size()) ? k_axes_s_[ind] : k_axes_s_[0];
}

const Vector3f& axis_to_vec3f(LabelledLine::Axis axis) noexcept
{
   const size_t ind = size_t(axis);
   return (ind < k_axes_v_.size()) ? k_axes_v_[ind] : k_axes_v_[0];
}

// ------------------------------------------------------------- CalibrationInfo
//
string CalibrationInfo::to_string() const noexcept
{
   Json::StyledWriter writer;
   return writer.write(to_json());
}

string str(const CalibrationInfo& o) noexcept { return o.to_string(); }

Json::Value CalibrationInfo::to_json() const noexcept
{
   Json::Value x;
   x["points"] = json_save_t(
       cbegin(points), cend(points), [](const auto& o) { return o.to_json(); });
   x["lines"] = json_save_t(
       cbegin(lines), cend(lines), [](const auto& o) { return o.to_json(); });
   return x;
}

bool CalibrationInfo::read(const Json::Value& x) noexcept
{
   CalibrationInfo o;
   try {
      json_load_t<LabelledPoint>(
          x["points"], o.points, [](const auto& n, auto& p) { p.read(n); });
      json_load_t<LabelledLine>(
          x["lines"], o.lines, [](const auto& n, auto& l) { l.read(n); });
   } catch(std::exception& e) {
      LOG_ERR(format("failed to load calibraiton info: {}", e.what()));
      return false;
   }
   *this = o;
   return true;
}

string CalibrationInfo::validate_calibration(
    const int n_cameras,
    const vector<AABBi>& bounds) const noexcept
{
   std::stringstream ss{""};

   Expects(size_t(n_cameras) == bounds.size());

   std::unordered_map<int, LabelledPoint> all_points;
   std::unordered_map<int, LabelledLine> all_lines;

   { // At most 1 origin per camera
      auto count_origins = [&](const int camera_no) {
         int counter = 0;
         for(const auto& o : points)
            if(o.is_origin && o.camera_no == camera_no) ++counter;
         return counter;
      };

      for(auto camera_no = 0; camera_no < n_cameras; ++camera_no)
         if(count_origins(camera_no) > 1)
            ss << format("multiview origins for camera {}\n", camera_no);
   }

   { // Labels must be unique per camera view
      auto check_label_uniqueness = [&](const auto& vec) {
         std::unordered_set<int> ids;
         ids.reserve(vec.size());
         for(auto cam_no = 0; cam_no < n_cameras; ++cam_no) {
            ids.clear();
            for(const auto& o : vec) {
               if(o.camera_no == cam_no && o.label >= 0) {
                  if(ids.count(o.label) == 0) {
                     ids.insert(o.label);
                  } else {
                     ss << format("duplicate label = {} found in camera {}\n",
                                  o.label,
                                  o.camera_no);
                  }
               }
            }
         }
      };

      check_label_uniqueness(points);
      check_label_uniqueness(lines);
   }

   { // Labels for all points and lines
      auto check_label = [&](const auto& o) {
         if(o.label < 0) { ss << format("no label for {}\n", o.to_string()); }
         if(o.camera_no < 0 || o.camera_no >= n_cameras) {
            ss << format("invalid camera number for {}\n", o.to_string());
         }
      };

      auto check_point = [&](const auto& o) {
         check_label(o);
         if(size_t(o.camera_no) < bounds.size()) {
            if(!bounds[size_t(o.camera_no)].contains(to_pt2(o.x))) {
               ss << format("point out outside image for {}\n", o.to_string());
            }
         }
         if(o.is_origin && !o.on_floor) {
            ss << format("origin points must be on the floor {}\n",
                         o.to_string());
         }

         if(o.label >= 0) {
            auto ii = all_points.find(o.label);
            if(ii == cend(all_points)) {
               all_points[o.label] = o;
            } else {
               if(ii->second.on_floor != o.on_floor)
                  ss << format("on-floor mismatch:\n{}\n{}\n",
                               ii->second.to_string(),
                               o.to_string());
            }
         }
      };

      auto check_line = [&](const auto& o) {
         check_label(o);
         if(size_t(o.camera_no) < bounds.size()) {
            if(!bounds[size_t(o.camera_no)].contains(to_pt2(o.x0))
               || !bounds[size_t(o.camera_no)].contains(to_pt2(o.x1))) {
               ss << format("line-point out outside image for {}\n",
                            o.to_string());
            }
         }
         if(int(o.axis) <= (LabelledLine::NONE)
            || int(o.axis) > int(LabelledLine::Z_AXIS)) {
            ss << format("line has no axis set: {}\n", o.to_string());
         }

         if(o.label >= 0) {
            auto ii = all_lines.find(o.label);
            if(ii == cend(all_lines)) {
               all_lines[o.label] = o;
            } else {
               if(ii->second.axis != o.axis)
                  ss << format("line-axis mismatch:\n{}\n{}\n",
                               ii->second.to_string(),
                               o.to_string());
               if(ii->second.on_floor != o.on_floor)
                  ss << format("on-floor mismatch:\n{}\n{}\n",
                               ii->second.to_string(),
                               o.to_string());
            }
         }
      };

      for(const auto& o : points) check_point(o);
      for(const auto& o : lines) check_line(o);
   }

   return ss.str();
}

// ----------------------------------------------------------- point-point error
//
Vector3f epipolar_line(const DistortedCamera& dcam0,
                       const DistortedCamera& dcam1,
                       const Vector3f& n0) noexcept
{
   // Image (dcam0.C) --> (dcam0.C + n0) in dcam1
   const auto u = project_to_normalized(dcam1, dcam0.C);
   const auto v = project_to_normalized(
       dcam1, dcam0.C + dcam0.q.inverse_apply(to_vec3(n0.normalised())));
   return to_vec3f(to_homgen_line(u, v));
}

static float
point_point_error(const DistortedCamera& dcam0,
                  const DistortedCamera& dcam1,
                  const Vector3f& n0, // normalized coordinate, n0.z == 1.0f
                  const Vector3f& n1) noexcept
{
   // Get the epipolar line in 1 with respect to the other
   // Then calculate the normalized coordinate line-distance error
   const auto ll1 = epipolar_line(dcam0, dcam1, n0);
   const auto ll0 = epipolar_line(dcam1, dcam0, n1);
   return 0.5f * (dot(ll0, n0) + dot(ll1, n1));
}

// ------------------------------------------------------------- line-line error
//
Plane4f projective_line_to_plane(const DistortedCamera& dcam,
                                 const Vector3f& n0,
                                 const Vector3f& n1) noexcept
{
   return to_vec4f(
       plane_from_3_points(dcam.C,
                           dcam.C + dcam.q.inverse_apply(to_vec3(n0)),
                           dcam.C + dcam.q.inverse_apply(to_vec3(n1))));
}

Vector3f recover_line_center(const DistortedCamera& dcam,
                             const Vector3f& n0,
                             const Vector3f& n1,
                             const Plane4f& p3) noexcept
{
   const Vector3f m = to_vec3f(dcam.q.inverse_apply(to_vec3(0.5f * (n0 + n1))));
   const Vector3f C = to_vec3f(dcam.C);
   const Vector3f X = plane_ray_intersection(p3, C, C + m);
   return X;
}

Vector3f image_line(const DistortedCamera& dcam,
                    const Vector3f& axis,
                    const Vector3f& n0,
                    const Vector3f& n1,
                    const Plane4f& p3) noexcept
{
   const auto X  = to_vec3(recover_line_center(dcam, n0, n1, p3));
   const auto x0 = project_to_normalized(dcam, X);
   const auto x1 = project_to_normalized(dcam, X + to_vec3(axis));
   return to_vec3f(to_homgen_line(x0, x1));
}

float line_line_error(const DistortedCamera& dcam0,
                      const DistortedCamera& dcam1,
                      LabelledLine::Axis axis,
                      const Vector3f& a0,
                      const Vector3f& a1,
                      const Vector3f& b0,
                      const Vector3f& b1) noexcept
{
   const auto& axis_vec = axis_to_vec3f(axis);
   Expects(axis_vec.quadrance() > 0.0f);
   const auto ll0 = image_line(
       dcam0, axis_vec, a0, a1, projective_line_to_plane(dcam1, b0, b1));
   const auto ll1 = image_line(
       dcam1, axis_vec, b0, b1, projective_line_to_plane(dcam0, a0, a1));

   return 0.25f * (dot(ll0, a0) + dot(ll0, a1) + dot(ll1, b0) + dot(ll1, b1));
}

// --------------------------------------------------------------- bundle-adjust
//
vector<DistortedCamera>
bundle_adjust(const vector<DistortedCamera>& dcams,
              const CalibrationInfo& calib_info) noexcept
{
   return dcams;
}

} // namespace perceive::calibration
