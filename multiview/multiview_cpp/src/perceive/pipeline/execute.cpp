
#include "execute.hpp"

#include "movie-results.hpp"
#include "pipeline-output.hpp"
#include "test-output.hpp"

#include "perceive/calibration/training-data/make-training-data-video.hpp"
#include "perceive/cost-functions/classifier/classifier-registry.hpp"
#include "perceive/cost-functions/fowlkes/fowlkes.hpp"
#include "perceive/cost-functions/pose-skeleton/hyperpose-helpers.hpp"
#include "perceive/movie/debug-movie.hpp"
#include "perceive/utils/file-system.hpp"
#include "tracker/run-tracklets-algorithm-1.hpp"

namespace perceive::pipeline
{
using namespace perceive::movie;

bool run_pipeline(const CliArgs& config) noexcept
{
   const auto now     = tick();
   real movie_seconds = 0.0;

   TRACE("run pipeline");
   MovieResults::Params in_params;

   { // (*) ---- Print config
      INFO(format("Printing config:"));
      cout << config.to_string() << endl;
   }

   { // (*) ---- Parallel load any classifier
      schedule([&]() {
         if(!config.pose_classifier.empty()) {
            get_classifier(config.pose_classifier);
         }
      });
   }

   { // (*) ---- Initialize params, including loading the scene
      if(!in_params.init(config, false)) return false;
   }

   { // (*) ---- Load the scene
      if(!in_params.load_manifest()) return false;
      Expects(in_params.scene_desc);
      INFO(format("Loaded Scene: {}\n\n", in_params.scene_desc->to_string()));
   }

   const auto scene_desc = in_params.scene_desc;

   { // (*) ---- Export video if we're creating a testcase
      if(config.setup_testcase) {
         TRACE(format("setup testcase: export movie and manifest"));
         export_movie_and_manifest(*scene_desc,
                                   config.outdir,
                                   scene_desc->scene_fps,
                                   config.start_frame_no,
                                   config.n_frames_to_process);
         INFO(format("Movie files exported to {}", config.outdir));
         exit(0);
      }
   }

   // (*) ---- Create 'MovieResults'
   unique_ptr<MovieResults> movie;
   try {
      TRACE(format("creating MovieResults object"));
      movie = make_unique<MovieResults>(in_params);
   } catch(std::exception& e) {
      LOG_ERR(format("exception creating MovieResults: {}", e.what()));
      return false;
   }
   const auto p = movie->params();

   { // (*) ---- Save the parameters that we've loaded
      Json::Value o = in_params.params.to_json();
      Expects(has_key(o, "config") == false);
      o["config"]      = config.to_json();
      o["task_params"] = movie->task_params();
      const auto fname = format("{}/params.json", config.outdir);
      TRACE(format("saving parameters to: {}", fname));
      file_put_contents(fname, str(o));
   }

   // (*) ---- Output 'dot-graph'
   movie->frame_results->output_dot_graph(p.config.outdir);

   // (*) ---- Get the movie-task results
   // NOTE: this will generate (or load) stats according to parameters
   // NOTE: this will screw up task-timings, if they're being collected
   TRACE(format("getting task result"));
   auto movie_ret = movie->movie_task.get_result_synchronized();

   if(movie_ret->n_frames() < 2) {
      LOG_ERR(format("at least 2 frames required to run the pipeline, "
                     "start-frame = {}, end-frame = {}, n-frames = {}",
                     movie_ret->start_frame(),
                     movie_ret->end_frame(),
                     movie_ret->n_frames()));
      return false;
   }

   { // (*) ---- Output colour floor histogram, if required
      if(p.config.colour_floor_hist) {
         TRACE(format("writing floor histogram"));
         movie_ret->floor_hist_result(movie_ret->start_frame());
      }
   }

   // (*) ---- Save Tracklet Data (if we need to)
   if(p.config.save_localization_data) {
      TRACE(format("saving localization data"));

      if(p.config.generate_timings) {
         WARN(format("saving localization data will interfer with timings"));
      }

      const auto& outdir = p.config.outdir;
      auto fname         = format("{}/localization.data", outdir);

      save_localization_data(
          fname,
          movie_ret->start_frame(),
          movie_ret->n_frames(),
          scene_desc->frame_duration(),
          [&](unsigned t) { return movie_ret->localization_result(t).get(); },
          p.config.verbosity >= 1);

      INFO(format("saved localization data to '{}'", fname));
   }

   // (*) ---- Run the tracker
   unique_ptr<TestOutput> testcase_result;
   if(p.config.generate_tracks || p.config.export_training_data) {
      TRACE(format("generating tracks"));

      if(p.config.verbosity >= 1)
         INFO(format("running the 'tracklets' tracker"));

      FowlkesResult track_results0 = run_tracklets_algorithm_1(p, movie.get());
      const FowlkesResult* track_results_ptr = &track_results0;

      // Save the output
      TRACE(format("creating pipeline output data"));
      PipelineOutput outdata;
      outdata.config        = p.config;
      outdata.params        = movie->params().params.to_json();
      outdata.track_results = track_results_ptr->to_json();
      save(outdata, p.config.pipeline_output_filename);

      if(p.config.run_pipeline_testcase) {
         TRACE(format("running pipeline testcase"));
         Expects(track_results_ptr != nullptr);
         testcase_result = make_unique<TestOutput>(
             compare_tracker_output(p.ground_truth,
                                    *track_results_ptr,
                                    lookup_person_radius(outdata.params)));
         if(!testcase_result->test_passes) LOG_ERR(format("testcase failed"));
      }

      // (*) ---- Generate Movies
      const auto movie_tick = tick();
      DebugMovieParams params{config};
      if(p.config.debug_video_output) {
         if(p.config.verbosity >= 1) INFO("Making Debug Movie");
         make_debug_movie(outdata,
                          *track_results_ptr,
                          testcase_result.get(),
                          movie.get(),
                          params);
      } else if(p.config.presentation_video_output) {
         LOG_ERR(format(
             "PRESENTATION video no implemented yet, outputing debug video"));
         make_debug_movie(outdata,
                          *track_results_ptr,
                          testcase_result.get(),
                          movie.get(),
                          params);
      }
      movie_seconds = tock(movie_tick);
   }

   // (*) ---- Render the timings to the output directory
   if(p.config.generate_timings) {
      const auto fname = format("{}/timings.png", p.config.outdir);
      TRACE(format("saving timings to: {}", fname));
      render_timings(movie_ret->task_timings()).save(fname);
   }

   // (*) ---- Record an error if the testcase failed
   bool success = true;
   if(p.config.run_pipeline_testcase && (testcase_result != nullptr)) {
      INFO("testcase results:");
      cout << str(*testcase_result) << endl;
      if(!testcase_result->test_passes) success = false;
   }

   // (*) ---- Create training data video
   if(p.config.export_training_data) {
      finish_training_data_export(format("{}/training-data", config.outdir),
                                  scene_desc->scene_fps,
                                  p.config.export_training_data_prefix);
   }

   { // (*) ---- Trailing information
      const auto total_seconds = tock(now);
      const auto exec_seconds  = total_seconds - movie_seconds;
      INFO(format(R"V0G0N(pipeline execution information:

   success:                  {}

   n-frames:                 {}
   number of cameras:        {}

   pipeline-seconds:         {}s
   debug-movie-seconds:      {}s

   total-seconds:            {}s
   seconds per (frame/cam):  {}s

)V0G0N",
                  str(success),
                  scene_desc->n_frames(),
                  scene_desc->n_cameras(),
                  exec_seconds,
                  movie_seconds,
                  total_seconds,
                  exec_seconds
                      / (scene_desc->n_frames() * scene_desc->n_cameras())));
   }

   INFO(format("pipeline-execute done."));

   return success;
}

} // namespace perceive::pipeline
