
#include "merge-aliased-op.hpp"

#define This MergePose3DExec

namespace perceive
{
// ------------------------------------------------------------------- meta-data
//
const vector<MemberMetaData>& This::Params::meta_data() const noexcept
{
   auto make_meta = [&]() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(This::Params, BOOL, do_merge, true));
      m.push_back(MAKE_META(This::Params, COMPATIBLE_OBJECT, p3d_params, true));
      m.push_back(MAKE_META(This::Params, REAL, iu_threshold, true));
      return m;
   };
   static vector<MemberMetaData> meta = make_meta();
   return meta;
}

// ------------------------------------------------------------------- calculate
//
vector<Pose3D> This::calculate(const SceneDescription& scene_desc,
                               const LocalizationData& ldat,
                               const Params& params,
                               const Tracklet::Params& tparams) noexcept
{
   return {};

#ifdef DO_NOT_COMPILE

   if(!params.do_merge) return ldat.p3ds;

   using SensorPose = Pose3D::SensorPose;

   // [1] If there's strong overlap from two or more sensors,
   //     where we minimize the reprojection error, and maximize
   //     histogram overlap.

   const vector<Pose3D>& in = ldat.p3ds;
   Expects(std::all_of(
       cbegin(in), cend(in), [](auto& o) { return o.size() == 1; }));

   vector<bool> marks(in.size());
   std::fill(begin(marks), end(marks), false);
   auto is_marked = [&](int ind) { return marks[ind]; };
   auto mark      = [&](int ind) { marks[ind] = true; };

   vector<const SensorPose*> sps;
   auto dist = [&](const Pose3D& a, const Pose3D& b) -> real {
      sps.clear();
      sps.resize(a.size() + b.size());
      for(const auto& sp : a.poses()) sps.push_back(&sp);
      for(const auto& sp : b.poses()) sps.push_back(&sp);
      return sensor_pose_score(scene_desc, sps);
   };

   vector<Pose3D> out;
   out.reserve(in.size());

   for(index i = 0; i < in.size(); ++i) {
      if(is_marked(i)) continue; // we've already nabbed this input
      mark(i); // we're not reusing this. (This line for clarity only)
      out.push_back(in[i]);
      Pose3D& p3d = out.back();

      for(index j = i + 1; j < in.size(); ++j) {
         // Can we combine the jth Pose3D with p3d?
         Pose3D o = Pose3D::combine(&scene_desc, params.p3d_params, p3d, in[j]);

         if(false) {
            INFO(format("combine [{}] => {}",
                        implode(cbegin(o.poses()),
                                cend(o.poses()),
                                ", ",
                                [&](const auto& sp) {
                                   return format(
                                       "{{}, {}}", sp.sensor_no, sp.detect_no);
                                }),
                        o.iu()));
         }

         if(o.iu() > params.iu_threshold) {
            p3d = std::move(o);
            mark(j);
         }
      }
   }

   return out;
#endif
}

} // namespace perceive
