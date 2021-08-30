
#pragma once

#include "perceive/io/struct-meta.hpp"
#include "perceive/scene/pose-annotation.hpp"

namespace perceive
{
// A single point on a track
struct TrackPoint
{
   float x = 0.0f, y = 0.0f;    // The xy location of the track
   int t                = 0;    // the frame-number of the track
   float gaze_direction = fNAN; // theta, radians
   int p3d_index = -1; // Index into tracklet-exec.p3ds, if it still exists
   PoseAnnotation pose       = PoseAnnotation::NONE;
   vector<float> pose_scores = {}; // used in smoothing, but not exported

   TrackPoint() = default;
   TrackPoint(float x_,
              float y_,
              int t_,
              float gaze           = 0.0f,
              int p3d_ind_         = -1,
              PoseAnnotation pose_ = PoseAnnotation::NONE) noexcept
       : x(x_)
       , y(y_)
       , t(t_)
       , gaze_direction(gaze)
       , p3d_index(p3d_ind_)
       , pose(pose_)
   {}
   TrackPoint(const TrackPoint&) = default;
   TrackPoint(TrackPoint&&)      = default;
   ~TrackPoint()                 = default;
   TrackPoint& operator=(const TrackPoint&) = default;
   TrackPoint& operator=(TrackPoint&&) = default;

   size_t memory_usage() const noexcept { return sizeof(TrackPoint); }

   Vector2f xy() const noexcept { return Vector2f(x, y); }
   void set_xy(Vector2f o) noexcept
   {
      x = o.x;
      y = o.y;
   }

   Point2 rounded_xy() const noexcept { return to_pt2(xy().round()); }

   Vector2 forward_vector() const noexcept
   {
      return !std::isfinite(gaze_direction)
                 ? Vector2::nan()
                 : Vector2(cos(real(gaze_direction)),
                           sin(real(gaze_direction)));
   }

   void set_pose_scores(const vector<float>& scores) noexcept
   {
      pose_scores = scores;
      if(pose_scores.size() != size_t(n_pose_annotations())) {
         // reduce size, or expand
         pose_scores.resize(n_pose_annotations(), 0.0f);
      }

      auto bb = cbegin(pose_scores);
      auto ii = std::max_element(bb, cend(pose_scores));
      pose    = (ii == cend(pose_scores))
                    ? PoseAnnotation::NONE
                    : to_pose_annotation(int(std::distance(bb, ii)));
   }

   bool operator==(const TrackPoint& o) const noexcept
   {
      return float_is_same(x, o.x) && float_is_same(y, o.y) && t == o.t
             && float_is_same(gaze_direction, o.gaze_direction);
   }
   bool operator!=(const TrackPoint& o) const noexcept { return !(*this == o); }
   bool operator<(const TrackPoint& o) const noexcept
   {
      return (t != o.t)               ? (t < o.t)
             : !float_is_same(x, o.x) ? (x < o.x)
                                      : (y < o.y);
   }

   bool operator<=(const TrackPoint& o) const noexcept { return !(o < *this); }
   bool operator>(const TrackPoint& o) const noexcept { return o < *this; }
   bool operator>=(const TrackPoint& o) const noexcept { return !(*this < o); }

   string to_string() const noexcept;
   string to_json_string() const noexcept;
   Json::Value to_json() const noexcept;
   bool load_json(const Json::Value& x) noexcept(false);
   void read_with_defaults(const Json::Value& o,
                           const TrackPoint* defaults = nullptr) noexcept;

   friend string str(const TrackPoint& o) { return o.to_string(); }
};

inline float quadrance(const TrackPoint& A, const TrackPoint& B) noexcept
{
   return square(A.x - B.x) + square(A.y - B.y) + float(square(A.t - B.t));
}

META_READ_WRITE_LOAD_SAVE(TrackPoint)

} // namespace perceive

namespace std
{
// Point4
template<> struct hash<perceive::TrackPoint>
{
   size_t operator()(const perceive::TrackPoint& v) const
   {
      return perceive::Vector3(
                 perceive::real(v.x), perceive::real(v.y), perceive::real(v.t))
          .hash();
      // return v.xyt().hash();
   }
};

} // namespace std
