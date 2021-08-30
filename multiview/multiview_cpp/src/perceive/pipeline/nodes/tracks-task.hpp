
#pragma once

#include "stdinc.hpp"

#include "load_scene_description.hpp"
#include "localization-task.hpp"
#include "movie-task.hpp"
#include "tracklet-task.hpp"

#include "perceive/cost-functions/tracks/node-sequence.hpp"
#include "perceive/cost-functions/tracks/ops.hpp"
#include "perceive/cost-functions/tracks/tracks-exec.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/io/struct-meta.hpp"
#include "perceive/pipeline/detail/task-timings.hpp"
#include "perceive/pipeline/pipeline-task.hpp"
#include "perceive/pipeline/tracker/tracker-results.hpp"

namespace perceive::pipeline
{
class MovieResults;
class FrameResults;
} // namespace perceive::pipeline

namespace perceive::pipeline::tracks
{
struct Params final : public MetaCompatible
{
   CUSTOM_NEW_DELETE(Params)

   virtual ~Params() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   Tracks::Params tracks_params;

   int start_frame{0};
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

   Result(const Params& p, shared_ptr<const tracklet::Result>);
   Result(const Result&)     = delete;
   Result(Result&&) noexcept = default;
   ~Result();

   Result& operator=(const Result&) = delete;
   Result& operator=(Result&&) = default;

   const SceneDescription& scene_desc() const noexcept;
   const Params& params() const noexcept;
   const movie::Result& movie_ret() const noexcept;
   const tracklet::Result& tracklets() const noexcept;

   // Calculated "on the fly", and not useful for anything other
   // than debugging.
   int frame_no_to_track_object_idx(int frame_no) const noexcept;

   // Get tracks by frame number (contigous frames can return the same track)
   int n_frames() const noexcept;
   int n_tracks_objects() const noexcept;

   const Tracks* tracks(int frame_no) const noexcept;
   const Tracks* tracks_ind(int idx) const noexcept;

   TrackerResult get_tracker_result() const noexcept;

   vector<calibration::TrainingDataPoint> get_training_data() const noexcept;

   struct CompDataEnvelope
   {
      // The tracklets hold pointers that `comp_data` uses
      shared_ptr<const Tracklet> prev_tracklet                    = nullptr;
      shared_ptr<const Tracklet> this_tracklet                    = nullptr;
      std::unique_ptr<perceive::tracks::ComputationData> data_ptr = nullptr;
   };

   CompDataEnvelope calc_comp_data(int idx) const noexcept;
};

class Task final : public PipelineTask<Params, Result>
{
 public:
   CUSTOM_NEW_DELETE(Task)

 protected:
   shared_ptr<const Result> execute(const RunData& data,
                                    const Params& params,
                                    std::function<bool()> is_cancelled) const
       noexcept override;
};

} // namespace perceive::pipeline::tracks

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::tracks::Params)
} // namespace perceive
