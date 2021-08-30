
#pragma once

#include "stdinc.hpp"

#include "floor_hist.hpp"
#include "load_scene_description.hpp"
#include "localization-task.hpp"
#include "movie-task.hpp"

#include "perceive/cost-functions/tracklets/tracklet-exec.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/io/struct-meta.hpp"
#include "perceive/pipeline/detail/task-timings.hpp"
#include "perceive/pipeline/pipeline-task.hpp"

namespace perceive::pipeline
{
class MovieResults;
class FrameResults;
} // namespace perceive::pipeline

namespace perceive::pipeline::tracklet
{
struct Params final : public MetaCompatible
{
   CUSTOM_NEW_DELETE(Params)

   virtual ~Params() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   // unsigned max_frames = 8; // look at this many frames at a time
   Tracklet::Params tracklet_params;

   unsigned max_tracklets_to_store = 1;

   bool feedback{false};
   string out_dir{"/tmp"s};
};

class Task;

struct Result final
{
 private:
   struct Pimpl;
   unique_ptr<Pimpl> pimpl_;

 public:
   CUSTOM_NEW_DELETE(Result)

   Result(const Params& p, shared_ptr<const movie::Result>);
   Result(const Result&)     = delete;
   Result(Result&&) noexcept = default;
   ~Result();

   Result& operator=(const Result&) = delete;
   Result& operator=(Result&&) = default;

   const SceneDescription& scene_desc() const noexcept;
   const Params& params() const noexcept;
   const movie::Result& movie_ret() const noexcept;

   int n_frames() const noexcept;
   int n_tracklet_objects() const noexcept;
   int max_frames_per_tracklet() const noexcept;

   // Lazy evaluation
   // Returns nullptr if frame-no is out of range
   // If 'params().max_frames' == N, then
   // passing [0..N) returns the same Tracklet object
   shared_ptr<const Tracklet> tracklets(int frame_no,
                                        std::function<bool()> is_cancelled
                                        = nullptr) const noexcept(false);
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

} // namespace perceive::pipeline::tracklet

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::tracklet::Params)
} // namespace perceive
