
#include "slic-3d.hpp"

#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/utils/threads.hpp"

#define This Slic3d

namespace perceive
{
// ------------------------------------------------------------------- meta data

const vector<MemberMetaData>& This::Params::meta_data() const noexcept
{
#define ThisParams Slic3d::Params
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, REAL, selected_threshold, true));
      return m;
   };
#undef ThisParams
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
}

// bool This::Params::operator==(const Params& o) const noexcept
// {
//    //
//    return true && (selected_threshold == o.selected_threshold);
// }

// std::string This::Params::to_string() const
// {
//    return format(R"V0G0N(
// Slic3d Parameters:
//    select-threshold    {}
// {})V0G0N",
//                  selected_threshold,
//                  "");
// }

// --------------------------------------------------------------- score-slic-p3
// 'next' returns TRUE until there are no more pixels
// @return average score for plane
real score_slic_p3(const DistortedCamera& cam0,
                   const DistortedCamera& cam1,
                   const LABImage& lab0,
                   const LABImage& lab1,
                   const vector<Point2>& inliers,
                   const Plane& p3) noexcept
{
   int counter = 0;
   real score  = 0.0;

   for(const auto& x : inliers) {
      auto y = to_pt2(transfer_distorted(cam0, cam1, p3, Vector2(x.x, x.y)));
      if(lab0.in_bounds(x) and lab1.in_bounds(y)) {
         const float val = cie1976_normalized_distance(lab0(x), lab1(y));
         score += real(val);
         counter++;
      }
   }

   return (counter == 0) ? 0.0 : (score / real(counter));
}

// ------------------------------------------------------------- set-p3s-to-slic
// @return A plane (could be NAN plane) for each
//         superpixel in `f2d0`
Slic3d::PlaneResult set_p3s_to_slic(const Slic3d::Params& params,
                                    const unsigned cam_num,
                                    const DistortedCamera& in_cam0,
                                    const DistortedCamera& in_cam1,
                                    const ImageFeatures2d& in_f2d0,
                                    const ImageFeatures2d& in_f2d1,
                                    const bool left_is_ref,
                                    const std::vector<Plane> p3s) noexcept
{
   ParallelJobSet pjobs;
   Slic3d::PlaneResult out;

   const auto& cam0 = left_is_ref ? in_cam0 : in_cam1;
   const auto& cam1 = left_is_ref ? in_cam1 : in_cam0;
   const auto& f2d0 = left_is_ref ? in_f2d0 : in_f2d1;
   const auto& f2d1 = left_is_ref ? in_f2d1 : in_f2d0;

   const auto N = f2d0.slic_info.size();

   out.plane_ind.resize(N);
   out.p3s.resize(N);
   out.scores.resize(N);
   out.selected.resize(N);
   fill(begin(out.plane_ind), end(out.plane_ind), -1);
   fill(begin(out.p3s), end(out.p3s), Plane::nan());
   fill(begin(out.scores), end(out.scores), 0.0);
   fill(begin(out.selected), end(out.selected), false);

   const auto test_lbl = 1000000;

   auto process_spixel = [&](unsigned label) {
      Expects(label < N);
      const auto& info = f2d0.slic_info[label];
      auto best_score  = std::numeric_limits<real>::max();
      auto best_ind    = -1;

      for(auto ind = 0u; ind < p3s.size(); ++ind) {
         const auto& p3 = p3s[ind];
         auto score     = uniform();
         // auto score     = score_slic_p3(
         //     cam0, cam1, f2d0.slic_im_LAB, f2d1.slic_im_LAB, info.inliers,
         //     p3);

         if(int(label) == int(test_lbl) and cam_num == 1) {
            LOG_ERR(format("label = {}, c={}, p3[{}] = {}, score = {}",
                           label,
                           str(info.center),
                           ind,
                           str(p3),
                           score));
         }

         if(score < best_score) {
            best_score = score;
            best_ind   = int(ind);
         }
      }

      out.plane_ind[label] = best_ind;
      out.p3s[label] = (best_ind >= 0) ? p3s[size_t(best_ind)] : Plane::nan();
      out.scores[label]   = best_score;
      out.selected[label] = best_score < params.selected_threshold;

      if(int(label) == int(test_lbl) and cam_num == 1) {
         WARN(format("out-plane = {}", best_ind));
      }

      if(false) {
         INFO(format("selected[{}] = {}, ({} < {})",
                     label,
                     str(out.selected[label]),
                     best_score,
                     params.selected_threshold));
      }
   };

   pjobs.reserve(N);
   for(auto label = 0u; label < N; ++label)
      pjobs.schedule([&process_spixel, label]() { process_spixel(label); });
   pjobs.execute();

   return out;
}

} // namespace perceive
