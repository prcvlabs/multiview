
#pragma once

#include "p2d-address.hpp"

#include "perceive/cost-functions/fowlkes/fowlkes.hpp"
#include "perceive/geometry/skeleton/skeleton-2d.hpp"
#include "perceive/io/struct-meta.hpp"

namespace perceive::tracks
{
// A node in the tracks computation
class Node
{
 private:
   vector<P2dAddress> p2ds_   = {};             // associated p2ds
   Vector2f xy_               = {-1.0f, -1.0f}; // in the histogram
   int t_                     = -1;             // frame number (t == time)
   float gaze_                = fNAN;
   bool interp_               = false;
   float prob_fp_             = fNAN; // probability false-positive
   vector<float> pose_scores_ = {};

   // `still-score` separate to `prob-fp`, because the still image
   // is accumulated as the video file is read. This means that
   // the scores are going to be useless towards the beginning of the
   // video. The TrackerResults.process(...) function can account for
   // this, because it knows how much of the video has been computed
   // when assessing the still-score.
   float still_score_ = fNAN;

   // There's a suite of associated scores

 public:
   Node()                = default;
   Node(const Node&)     = default;
   Node(Node&&) noexcept = default;
   ~Node()               = default;
   Node& operator=(const Node&) = default;
   Node& operator=(Node&&) noexcept = default;

   // Getters
   const auto& p2ds() const noexcept { return p2ds_; }
   const Vector2f& xy() const noexcept { return xy_; }
   int t() const noexcept { return t_; }
   float gaze() const noexcept { return gaze_; }
   bool is_interp() const noexcept { return interp_; }
   float prob_fp() const noexcept { return prob_fp_; }
   const auto& pose_scores() const noexcept { return pose_scores_; }
   float still_score() const noexcept { return still_score_; }

   // Setters
   void set_p2ds(vector<P2dAddress>&& val)
   {
      Expects(val.size() < 1000000); // should be reasonable
      p2ds_ = std::move(val);
   }
   void set_xy(const Vector2f& x) noexcept { xy_ = x; }
   void set_t(int t) noexcept { t_ = t; }
   void set_gaze(float gaze) noexcept { gaze_ = gaze; }
   void set_interp(bool val) noexcept { interp_ = val; }
   void set_prob_fp(float val) noexcept { prob_fp_ = val; }
   void set_pose_scores(const vector<float>& scores) noexcept
   {
      pose_scores_ = scores;
   }
   void set_still_score(float val) noexcept { still_score_ = val; }

   size_t memory_usage() const noexcept;

   string to_string() const noexcept;
   friend inline string str(const Node& o) { return o.to_string(); }
};

} // namespace perceive::tracks
