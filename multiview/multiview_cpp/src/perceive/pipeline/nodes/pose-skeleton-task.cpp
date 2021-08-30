
#include "pose-skeleton-task.hpp"

#include "perceive/cost-functions/pose-skeleton/pose-skeleton-exec.hpp"
#include "perceive/pipeline/detail/helpers.hpp"

namespace perceive::pipeline::pose_skeleton_init
{
// ------------------------------------------------------------------- meta data

const vector<MemberMetaData>& Params::meta_data() const noexcept
{
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(Params, COMPATIBLE_OBJECT, op_params, true));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
}

// ----------------------------------------------------------------------- Pimpl

struct Task::Pimpl
{
   PoseSkeletonExec op_exec; // For executing openpoase
};

// ------------------------------------------------------------------------ Task
//
Task::Task()
    : pimpl_(make_unique<Pimpl>())
{}
Task::~Task() = default;

// --------------------------------------------------------------------- Execute
//
shared_ptr<const Result>
Task::execute(const RunData& data,
              const Params& params,
              std::function<bool()> is_cancelled) const noexcept
{
   if(is_cancelled()) return nullptr;

   auto ret{make_shared<Result>()};

   ret->p = params;

   ret->copy_images_result
       = data.match_result<copy_sensor_images::Result>("copy_sensor_images");
   if(!ret->copy_images_result) {
      LOG_ERR("Cant find dependency: copy_sensor_images");
      return nullptr;
   }

   const auto& scene_desc{ret->scene_desc()};
   const auto& sensor_images = ret->copy_images_result->images.sensor_images;
   const int frame_no        = ret->copy_images_result->images.frame_no;

   if(is_cancelled()) return nullptr;

   const auto now        = tick();
   ret->op               = pimpl_->op_exec.run(scene_desc,
                                 frame_no,
                                 sensor_images,
                                 params.op_params,
                                 params.out_dir,
                                 is_cancelled);
   const auto elapsed_ms = tock(now);

   if(is_cancelled()) return nullptr;

   if(params.feedback) print_timing(format("openpose: {}s", elapsed_ms));

   if(is_cancelled()) { return nullptr; }

   return ret;
}
} // namespace perceive::pipeline::pose_skeleton_init
