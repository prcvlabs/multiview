
#include "stdinc.hpp"

#include "pose-skeleton-exec.hpp"

#include "hyperpose-helpers.hpp"
#include "op-helpers.hpp"

#include "perceive/geometry/skeleton/2d-helpers.hpp"

#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/cv-helpers.hpp"
#include "perceive/graphics/tiny-string.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/utils/base64.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/opencv-helpers.hpp"

#define This PoseSkeletonExec

namespace perceive
{
using namespace pose_skeleton;

// ----------------------------------------------------------------------- Pimpl

struct This::Pimpl
{
 private:
   ParallelJobSet pjobs_;

 public:
   ParallelJobSet& pjobs() noexcept { return pjobs_; }

   OpenposeWrapper op_wrapper;
   HyperposeWrapper hp_wrapper;
};

// -------------------------------------------------------- This::Results::Pimpl
//
struct This::Result::Pimpl
{
   vector<bool> sensor_included;
   vector<vector<shared_ptr<const Skeleton2D>>> poses_;
   real time_    = 0;
   int frame_no_ = -1;

   Pimpl(int n_sensors, int frame_no)
   {
      frame_no_ = frame_no;
      sensor_included.resize(size_t(n_sensors), false);
      poses_.resize(size_t(n_sensors));
   }
};

This::Result::Result(int n_sensors, int frame_no)
    : pimpl_(make_unique<Pimpl>(n_sensors, frame_no))
{}
This::Result::~Result()              = default;
This::Result::Result(This::Result&&) = default;
This::Result& This::Result::operator=(This::Result&&) = default;

int This::Result::frame_no() const noexcept { return pimpl_->frame_no_; }

unsigned This::Result::size() const noexcept
{
   return unsigned(pimpl_->poses_.size());
}

bool This::Result::valid_index(int ind) const noexcept
{
   return unsigned(ind) < size();
}

vector<unsigned> This::Result::sensors() const noexcept
{
   std::vector<unsigned> ret;
   for(auto i = 0u; i < size(); ++i)
      if(valid_index(int(i))) { ret.push_back(i); }
   return ret;
}

const vector<shared_ptr<const Skeleton2D>>&
This::Result::pose(int ind) const noexcept
{
   if(!valid_index(ind)) { INFO(format("Invalid sensor {}", ind)); }
   Expects(valid_index(ind));
   return pimpl_->poses_.at(size_t(ind));
}

// ----------------------------------------------------- Construction/Assignment
//
This::This()
    : pimpl_(make_unique<Pimpl>())
{}

This::~This() = default;

// ------------------------------------------------------------------ This::run_
//
This::Result This::run_(const SceneDescription* scene_desc_ptr,
                        const int frame_no,
                        const vector<cv::Mat>& images,
                        const Params& p,
                        const string_view outdir,
                        std::function<bool()> is_cancelled) noexcept
{
   const bool is_trace_mode = multiview_trace_mode();
   const unsigned n_sensors = unsigned(images.size());
   Expects((scene_desc_ptr == nullptr)
           || (n_sensors <= unsigned(scene_desc_ptr->n_sensors())));

   This::Result ret = This::Result(int(n_sensors), frame_no);

   if(images.size() == 0 || is_cancelled() || multiview_openpose_disabled())
      return ret;

   auto get_image_ptr = [&](int sensor_no) -> const cv::Mat* {
      if(unsigned(sensor_no) >= n_sensors) return nullptr;
      Expects(size_t(sensor_no) < images.size());
      if(scene_desc_ptr != nullptr) {
         const auto num = scene_desc_ptr->bcam_lookup(sensor_no).y;
         return (num != 0) ? nullptr : &images.at(size_t(sensor_no));
      }
      return &images.at(size_t(sensor_no));
   };

   const auto now = tick();

   bool use_hyperpose
       = (p.use_hyperpose && k_has_hyperpose) || (!k_has_openpose);

   if(use_hyperpose) {
      ret.pimpl_->poses_ = pimpl_->hp_wrapper.run(scene_desc_ptr,
                                                  frame_no,
                                                  int(n_sensors),
                                                  get_image_ptr,
                                                  p.hp_params,
                                                  outdir,
                                                  pimpl_->pjobs(),
                                                  is_cancelled);
   } else {
      ret.pimpl_->poses_ = pimpl_->op_wrapper.run(scene_desc_ptr,
                                                  frame_no,
                                                  int(n_sensors),
                                                  get_image_ptr,
                                                  p.op_params,
                                                  outdir,
                                                  pimpl_->pjobs(),
                                                  is_cancelled);
   }

   ret.pimpl_->sensor_included.resize(n_sensors);
   for(auto i = 0u; i < n_sensors; ++i)
      ret.pimpl_->sensor_included.at(i) = get_image_ptr(int(i)) != nullptr;
   ret.pimpl_->frame_no_ = frame_no;
   ret.pimpl_->time_     = tock(now);

   if(false && is_trace_mode) {
      for(auto i = 0u; i < ret.size(); ++i) {
         const auto& poses = ret.pose(int(i));
         for(auto j = 0u; j < poses.size(); ++j) {
            const auto fname = format("/tmp/pose_{}_{}.json", i, j);
            file_put_contents(fname, poses[j]->to_json_str());
            INFO(format("saved {}, (memory-usage = {} kbytes)",
                        fname,
                        real(poses[j]->memory_usage()) / 1024.0));
         }
      }
   }

   return ret;
}

// ------------------------------------------------------------------- This::run
//

PoseSkeletonExec::Result This::run(const SceneDescription& scene_desc,
                                   const int frame_no,
                                   const vector<cv::Mat>& images,
                                   const Params& p,
                                   const string_view outdir,
                                   std::function<bool()> is_cancelled) noexcept
{
   return this->run_(&scene_desc, frame_no, images, p, outdir, is_cancelled);
}

PoseSkeletonExec::Result
This::run(const cv::Mat& im, const Params& p, const string_view outdir) noexcept
{
   vector<cv::Mat> images(1);
   images.at(0) = im;
   return this->run_(nullptr, 0, images, p, outdir, []() { return false; });
}

} // namespace perceive
