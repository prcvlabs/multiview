
#pragma once

#include "params.hpp"

namespace perceive
{
//
class HyperposeWrapper final
{
 private:
   struct Pimpl;
   unique_ptr<Pimpl> pimpl_;

 public:
   HyperposeWrapper();
   HyperposeWrapper(const HyperposeWrapper&) = delete;
   HyperposeWrapper(HyperposeWrapper&&)      = default;
   ~HyperposeWrapper();
   HyperposeWrapper& operator=(const HyperposeWrapper&) = delete;
   HyperposeWrapper& operator=(HyperposeWrapper&&) = default;

   vector<vector<shared_ptr<const Skeleton2D>>>
   run(const SceneDescription* scene_desc_ptr,
       const int frame_no,
       const int n_images,
       std::function<const cv::Mat*(int i)> get_image,
       const pose_skeleton::HyperposeParams& p,
       const string_view outdir,
       ParallelJobSet& pjobs,
       std::function<bool()> is_cancelled) noexcept;
};

void clear_hyperpose_registry() noexcept;

} // namespace perceive
