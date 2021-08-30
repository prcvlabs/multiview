
#pragma once

#include "perceive/graphics.hpp"

namespace perceive
{
class BBSegment
{
 public:
   struct Params
   {
      bool operator==(const Params&) const;
      bool operator!=(const Params& o) const { return !(*this == o); }

      unsigned object_threshold{1600};
      real max_object_radius{0.40};
      real min_object_radius{0.35};
      real object_skirt{0.45};
      unsigned max_regions{400};
      real nms_region_overlap{0.50};
      real region_overlap{0.0};
      bool run_choose_best{true};
      // TODO timeout
      std::string to_string() const;
   };

   struct ObjectHypothesis
   {
      int label{0};
      Point2 X; // center
      float score{0.0f};
      AABBi aabb;

      ObjectHypothesis() = default;
      ObjectHypothesis(Point2 X_, float score_, AABBi aabb_);
   };

   static vector<ObjectHypothesis>
   bb_segment(const FloatImage& hist, const real hist_side, const Params& p);

   static void set_labels(vector<ObjectHypothesis>& current,
                          const vector<ObjectHypothesis>& previous);

   static void test_bb_segment(const FloatImage& hist, const real hist_side,
                               const Params& p, const string& outdir);

   static ARGBImage render(const vector<ObjectHypothesis>& objects,
                           const vector<ObjectHypothesis>& previous,
                           const FloatImage& hist, const float cap = 50.0f);
};

} // namespace perceive
