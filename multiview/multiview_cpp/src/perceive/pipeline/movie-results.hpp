
#pragma once

#include "perceive/scene/scene-description.hpp"

#include "cli-args.hpp"
#include "detail/task-timings.hpp"
#include "frame-results.hpp"
#include "nodes/nodes.hpp"
#include "perceive/cost-functions/classifier/classifier.hpp"
#include "perceive/cost-functions/tracklets/tracks-index.hpp"
#include "perceive/pipeline/tracker/tracker-results.hpp"
#include "pipeline-input.hpp"
#include "pipeline-output.hpp"

namespace perceive::pipeline
{
class MovieResults
{
 public:
   // ------------------------------------------------------------------- Params
   //
   struct Params
   {
      CliArgs config;
      shared_ptr<const SceneDescription> scene_desc;
      PipelineInput params;
      PostProcessParams post_process_params;

      // Sorry for the long name, but don't be confused by this
      // parameter! We're loading in 'pipeline-output' (from a previous
      // pipeline run) with a view to producing track output
      FowlkesResult visualization_preloaded_data; //
      TracksIndex visualization_index;

      //
      FowlkesResult ground_truth;

      // Initialize parameters from the passed config file
      // Loading the manifest takes time... so it's optional to do it
      // now or later...
      // If 'params.frame_params' is Json::nullValue, then it's updated
      bool init(const CliArgs& config, const bool do_load_manifest) noexcept;
      bool load_manifest() noexcept;
   };

   CUSTOM_NEW_DELETE(MovieResults)

 private:
   Params params_;
   void init_tasks_() noexcept;

 public:
   // -------------------------------------------------------------------- Tasks
   // 'frame_results' is here for convenience... for setting
   // parameters and looking at results. Call 'update_movie_task_params'
   // to transfer 'frame_results' parameters to 'movie_task'

   unique_ptr<FrameResults> frame_results;
   movie::Task movie_task;
   tracklet::Task tracklet_task;
   tracks::Task tracks_task;

   // ------------------------------------------------------------- Construction
   //
   MovieResults(const Params& p) noexcept(false);
   MovieResults(const MovieResults&) = delete;
   MovieResults(MovieResults&&)      = delete;
   ~MovieResults()                   = default;
   MovieResults& operator=(const MovieResults&) = delete;
   MovieResults& operator=(MovieResults&&) = delete;

   // ---------------------------------------------------------------- Utilities
   //
   void force_update_movie_task_params() noexcept;
   void test_and_update_movie_task_params(bool feedback) noexcept;
   void regenerate_movie_stats(std::function<bool()> is_cancelled,
                               std::function<void()> on_done_thunk) noexcept;

   void output_dot_graph(const string_view outdir) noexcept;
   void set_feedback(bool feedback) noexcept;
   void set_output_dir(const string_view out_dir);

   // ---------------------------------------------------------------- Params IO
   //
   Params params() const noexcept { return params_; }
   bool set_params(const Params&) noexcept;
   Json::Value task_params() const noexcept;
};

} // namespace perceive::pipeline
