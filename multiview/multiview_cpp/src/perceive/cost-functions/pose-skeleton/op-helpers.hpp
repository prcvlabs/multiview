
#pragma once

#include "params.hpp"

namespace perceive
{
class OpenposeWrapper final
{
 private:
   struct Pimpl;
   unique_ptr<Pimpl> pimpl_;

 public:
   OpenposeWrapper();
   OpenposeWrapper(const OpenposeWrapper&) = delete;
   OpenposeWrapper(OpenposeWrapper&&)      = default;
   ~OpenposeWrapper();
   OpenposeWrapper& operator=(const OpenposeWrapper&) = delete;
   OpenposeWrapper& operator=(OpenposeWrapper&&) = default;

   vector<vector<shared_ptr<const Skeleton2D>>>
   run(const SceneDescription* scene_desc_ptr,
       const int frame_no,
       const int n_images,
       std::function<const cv::Mat*(int i)> get_image,
       const pose_skeleton::OpenposeParams& p,
       const string_view outdir,
       ParallelJobSet& pjobs,
       std::function<bool()> is_cancelled) noexcept;
};

} // namespace perceive
