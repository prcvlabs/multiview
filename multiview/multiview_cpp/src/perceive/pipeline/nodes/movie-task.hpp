
#pragma once

#include "stdinc.hpp"

#include "floor_hist.hpp"
#include "load_scene_description.hpp"
#include "localization-task.hpp"

#include "perceive/cost-functions/movie-stats/movie-stats-file.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/cli-args.hpp"
#include "perceive/pipeline/detail/task-timings.hpp"
#include "perceive/pipeline/pipeline-task.hpp"

namespace perceive::pipeline
{
class MovieResults;
class FrameResults;
} // namespace perceive::pipeline

namespace perceive::pipeline::movie
{
void frame_results_deleter(FrameResults* ptr);

struct Params
{
   CUSTOM_NEW_DELETE(Params)

   Params();
   Params(const Params&);
   Params(Params&&) = default;
   ~Params();
   Params& operator=(const Params&);
   Params& operator=(Params&&) = default;

   bool operator==(const Params& o) const noexcept;
   bool operator!=(const Params& o) const noexcept { return !(*this == o); }
   string to_string(int indent = 0) const noexcept;
   string to_json_string(int indent = 0) const noexcept;
   Json::Value to_json() const noexcept;
   void read(const Json::Value&) noexcept(false);
   void read_with_defaults(const Json::Value&, const Params* defaults) noexcept;

   unique_ptr<FrameResults, void (*)(FrameResults*)> frame_results;
   shared_ptr<const LocalizationDataEnvelope> ldata_env_ptr;
   bool generate_stats          = false;
   bool load_stats              = false;
   unsigned max_frames_to_cache = 80;
   int start_frame              = 0;
   int end_frame                = 0;

   bool feedback  = false;
   string out_dir = "/tmp"s;
};

class Task;

struct Result
{
 private:
   struct Pimpl;
   unique_ptr<Pimpl> pimpl_;
   friend class Task;

 public:
   CUSTOM_NEW_DELETE(Result)

   Result(FrameResults&, const Params& params);
   Result(const Result&) = delete;
   Result(Result&&)      = default;
   ~Result();
   Result& operator=(const Result&) = delete;
   Result& operator=(Result&&) = default;

   unsigned full_movie_n_frames() const noexcept;
   unsigned start_frame() const noexcept; // [start-end] is INCLUSIVE
   int end_frame() const noexcept;        // i.e., -1 if n-frames is 0
   unsigned n_frames() const noexcept; // the n-frames specified on command line
   const SceneDescription& scene_desc() const noexcept;

   unsigned n_cached_frames() const noexcept;
   unsigned max_frames_to_cache() const noexcept;

   // Lazily calculates per-frame results
   shared_ptr<const floor_hist::Result>
   floor_hist_result(unsigned frame_no) const noexcept;

   const LocalizationData::Params& loc_params() const noexcept;

   shared_ptr<const LocalizationData>
   localization_result(unsigned frame_no, const string_view message = "") const
       noexcept;

   // Returns nullptr if the result isn't already calculated
   shared_ptr<const LocalizationData>
   try_get_localization_result(unsigned frame_no) const noexcept;

   real localization_result_timing(unsigned frame_no,
                                   const string_view message = "") const
       noexcept;
   const LABImage* get_sensor_lab(unsigned frame_no,
                                  unsigned sensor_no,
                                  const string_view message) const noexcept;
   shared_ptr<const LABImage>
   get_sensor_lab_shared_ptr(unsigned frame_no,
                             unsigned sensor_no,
                             const string_view message) const noexcept;

   // Returns nullptr if the result isn't already calculated
   shared_ptr<const LABImage>
   try_get_sensor_lab_shared_ptr(unsigned frame_no, unsigned sensor_no) const
       noexcept;

   shared_ptr<const PointCloud>
   get_camera_pt_cloud_ptr(unsigned frame_no,
                           unsigned camera_no,
                           const string_view message) const noexcept;

   const TaskTimings& task_timings() const noexcept;

   vector<shared_ptr<const LABImage>>
   frame_labs(int frame_no, const string_view message) const;

   // Save 'accumulated' stats in whatever form they are
   bool save_movie_stats(const string_view out_filename) const noexcept;
   bool gen_and_save_movie_stats(const int start_frame,
                                 const int n_frames,
                                 const string_view out_filename) const noexcept;

   size_t memory_usage() const noexcept;
};

class Task : public PipelineTask<Params, Result>
{
 public:
   CUSTOM_NEW_DELETE(Task)

 protected:
   shared_ptr<const Result> execute(const RunData& data,
                                    const Params& params,
                                    std::function<bool()> is_cancelled) const
       noexcept override;
};

} // namespace perceive::pipeline::movie

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::movie::Params)
} // namespace perceive
