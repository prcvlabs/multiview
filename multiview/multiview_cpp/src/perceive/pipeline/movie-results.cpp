
#include "movie-results.hpp"

#include "perceive/geometry/skeleton/2d-helpers.hpp"
#include "perceive/utils/file-system.hpp"

#define This MovieResults

namespace perceive::pipeline
{
// -------------------------------------------------------- Generate Task-Params
//
static Json::Value create_task_params(const FrameResults& frame_results,
                                      const tracklet::Task& tracklet_task,
                                      const tracks::Task& tracks_task,
                                      const PostProcessParams& post_process)
{
   auto out               = Json::Value{Json::objectValue};
   out["frame_params"]    = frame_results.params_to_json();
   out["tracklet_params"] = tracklet_task.params().to_json();
   out["tracks_params"]   = tracks_task.params().to_json();
   out["post_process"]    = post_process.to_json();
   return out;
}

// -------------------------------------------------------------- traverse nodes
//
template<typename ResultType, typename Visitor>
static void traverse_nodes(ResultType& r, const Visitor& v)
{
   v(r.movie_task);
   v(r.tracklet_task);
   v(r.tracks_task);
}

template<typename ResultType, typename Visitor>
static void ctraverse_nodes(const ResultType& r, const Visitor& v)
{
   traverse_nodes(*const_cast<ResultType*>(&r), v);
}

// --------------------------------------------------------------
//
static void update_movie_stats_params(const CliArgs& config,
                                      movie_stats::Params& msp)
{
   msp.no_stats      = config.no_stats;
   msp.save_filename = config.stats_filename;
   msp.in_stats_fnames.resize(config.n_in_stats_fnames());
   for(auto i = 0u; i < msp.in_stats_fnames.size(); ++i)
      msp.in_stats_fnames[i] = config.in_stats_fname(i);
}

// ---------------------------------------------------------------------- Params
//
bool This::Params::init(const CliArgs& config,
                        const bool do_load_manifest) noexcept
{
   TRACE(format("init movie results"));
   Params p;

   // (*) ---- Copy config parameters
   p.config = config;

   // (*) ---- Load manifest...
   if(do_load_manifest) {
      if(!p.load_manifest()) return false;
   }

   // (*) ---- Load parameters
   if(config.has_pipeline_params()) {
      try {
         PipelineInput defaults;
         p.params.read_with_defaults(config.pipeline_params, &defaults);
      } catch(std::exception& e) {
         LOG_ERR(format("exception loading parameters: {}", e.what()));
         return false;
      }
   }

   // (*) ---- Load Tracks
   if(!config.tracks_filename.empty()) {
      try {
         PipelineOutput pout_dat;
         load(pout_dat, config.tracks_filename);
         read(p.visualization_preloaded_data, pout_dat.track_results);
         p.visualization_index.init(p.visualization_preloaded_data.tracks);
         INFO(format("loading pipeline-output (for visualization): '{}'",
                     config.tracks_filename));
      } catch(std::exception& e) {
         LOG_ERR(format("exception loading pipeline-output (for "
                        "visualization): '{}': {}",
                        config.tracks_filename,
                        e.what()));
         return false;
      }
   }

   // (*) ---- Load GroundTruth, if available
   if(!config.tracks_filename.empty()) {
      PipelineOutput ground_truth;
      try {
         INFO(format("load ground truth: {}", config.tracks_filename));
         load(ground_truth, config.tracks_filename);
         p.ground_truth = ground_truth.tracks_to_fowlkes();
         // FATAL(format("frame-duration = {}",
      } catch(std::exception& e) {
         LOG_ERR(format("failed to load ground-truth file '{}': {}",
                        config.tracks_filename,
                        e.what()));
         return false;
      }
   }

   // (*) ---- Update "*this"
   *this = p;

   return true;
}

bool This::Params::load_manifest() noexcept
{
   TRACE(format("load manifest"));
   const auto now = tick();

   if(!config.has_manifest_params()) {
      LOG_ERR(format("no manifest parameters specified!"));
      return false;
   }

   try {
      auto scene_desc = make_shared<SceneDescription>();
      SceneDescription::InitOptions opts;
      opts.target_fps           = config.target_fps;
      opts.optional_search_path = config.manifest_opt_search_path;
      opts.on_the_fly_still     = config.on_the_fly_still;
      opts.no_stereo            = config.no_stereo;
      opts.verbose              = true; // config.verbosity >= 1;
      scene_desc->init_from_json(config.manifest_params, opts);
      scene_desc->seek_frame_(config.start_frame_no - 1);
      this->scene_desc = scene_desc;
   } catch(std::exception& e) {
      LOG_ERR(format("exception loading manifest: {}", e.what()));
      return false;
   }

   if(params.task_params.isNull()) {
      FrameResults frame_results(this->scene_desc, false);
      tracklet::Task tracklet_task;
      tracks::Task tracks_task;
      PostProcessParams post_process_params;

      params.task_params = create_task_params(
          frame_results, tracklet_task, tracks_task, post_process_params);
   }

   if(config.verbosity >= 1) {
      INFO(format("manifest loaded: {}s", tock(now)));
   }

   return true;
}

// --------------------------------------------------------------- Movie Results
//
This::This(const Params& p) noexcept(false)
{
   if(!set_params(p))
      throw std::runtime_error(format("failed to initialize movie-results"));
   Expects(frame_results != nullptr);
   init_tasks_();
}

// ---------------------------------------------------- Update Movie Task Params
//
void This::init_tasks_() noexcept
{
   // -- Name the tasks --
   movie_task.set_taskname("movie");
   tracklet_task.set_taskname("tracklets");
   tracks_task.set_taskname("tracks");

   // -- Wire the tasks --
   tracklet_task.subscribe(&movie_task);
   tracks_task.subscribe(&tracklet_task);
}

// ---------------------------------------------------- Update Movie Task Params
//
void This::force_update_movie_task_params() noexcept
{
   auto calc_start_end_frame = [&](const LocalizationDataEnvelope* ldat_env) {
      Expects(params_.scene_desc);

      const int total_frames = params_.scene_desc->n_frames();
      const int start_frame  = params_.config.start_frame_no;
      int n_frames           = std::max(0, total_frames - start_frame);
      if(params_.config.n_frames_to_process >= 0) {
         n_frames = std::min(n_frames, params_.config.n_frames_to_process);
      }
      const int end_frame = start_frame + n_frames - 1; // ie., INCLUSIVE

      return std::make_pair(start_frame, end_frame);
   };

   auto p                               = movie_task.params();
   p.feedback                           = params_.config.verbosity > 0;
   p.out_dir                            = params_.config.outdir;
   p.generate_stats                     = params_.config.generate_stats;
   p.load_stats                         = params_.config.is_loading_stats();
   std::tie(p.start_frame, p.end_frame) = calc_start_end_frame(nullptr);
   p.ldata_env_ptr                      = params_.config.ldata_env_ptr;

   p.max_frames_to_cache
       = tracklet_task.params().tracklet_params.max_frames_per_tracklet;

   if(!p.frame_results)
      p.frame_results = unique_ptr<FrameResults, void (*)(FrameResults*)>(
          new FrameResults(*frame_results), movie::frame_results_deleter);
   else
      p.frame_results->operator=(*frame_results);

   p.frame_results->set_output_dir(p.out_dir);
   p.frame_results->set_feedback(params_.config.verbosity > 1);

   {
      auto msp = p.frame_results->movie_stats.params();
      update_movie_stats_params(params_.config, msp);
      p.frame_results->movie_stats.set_params(msp);
   }

   {
      auto hsp                        = p.frame_results->floor_hist.params();
      hsp.hist_params.color_histogram = params_.config.colour_floor_hist;
      p.frame_results->floor_hist.set_params(hsp);
   }

   movie_task.set_params(p);
}

void This::test_and_update_movie_task_params(bool feedback) noexcept
{
   auto p = movie_task.params();
   if(!p.frame_results) {
      if(feedback) FATAL(format("CREATING movie-params.frameresults"));
      force_update_movie_task_params();
      return;
   }

   { // It is UNFORTUNATE that we don't want to compare some parameters...
      // HACK HACK HACK HACK HACK (You got a better idea?)
      p.frame_results->copy_sensor_images.set_params(
          frame_results->copy_sensor_images.params());
   }
   if(p.frame_results->operator!=(*frame_results)) {
      if(feedback) INFO(format("UPDATING movie-params.frameresults"));
      force_update_movie_task_params();
   } else {
      // if(feedback) INFO(format("<.> movie-param.frameresults _same_"));
   }
}

// ------------------------------------------------------ regenerate movie stats
//
void This::regenerate_movie_stats(std::function<bool()> is_cancelled,
                                  std::function<void()> done_thunk) noexcept
{
   schedule([o = this, p0 = params(), is_cancelled, done_thunk]() {
      using namespace std::chrono_literals;
      Params p                = p0;
      p.config.generate_stats = true;
      MovieResults e(p);
      std::atomic<bool> is_done{false};
      schedule([&]() {
         e.movie_task.result([&](auto x) {
            is_done = true;
            if(x != nullptr) {
               if(o->frame_results) o->frame_results->movie_stats.cancel();
               o->movie_task.cancel();
            }
         });
      });
      while(!is_done) {
         try_one_scheduled_job();
         const bool cancelled = is_cancelled();
         if(cancelled) e.movie_task.cancel();
         std::this_thread::yield();
      }
      if(done_thunk) done_thunk();
   });
}

// -----------------------------------------------------------------------------
//
void This::output_dot_graph(const string_view outdir) noexcept
{
   frame_results->output_dot_graph(string(outdir));
}

void This::set_feedback(bool feedback) noexcept
{
   frame_results->set_feedback(feedback);
}

void This::set_output_dir(const string_view out_dir)
{
   frame_results->set_output_dir(string(out_dir));
   traverse_nodes(*this, [out_dir](auto& x) {
      auto p    = x.params();
      p.out_dir = out_dir;
      x.set_params(p);
   });
}

// ------------------------------------------------------------------ set params
//
bool This::set_params(const Params& p) noexcept
{
   bool success               = true;
   params_                    = p;
   const auto& task_params    = p.params.task_params;
   const bool has_task_params = (task_params.type() == Json::objectValue);

   frame_results
       = make_unique<FrameResults>(p.scene_desc, p.config.verbosity > 0);

   // Load parameters into frame-results
   if(has_task_params) {
      success = has_key(task_params, "frame_params")
                && frame_results->read_params(task_params["frame_params"],
                                              "frame_params"s);
   }

   frame_results->set_feedback(p.config.verbosity > 1);

   // { // Localization parameters
   //    auto lp = frame_results->localization.params();
   //    lp.localization_params.export_training_data_prefix
   //        = p.config.export_training_data_prefix;
   //    frame_results->localization.set_params(lp);
   // }

   { // Make sure we have the correct movie-stats filename
      auto msp = frame_results->movie_stats.params();
      update_movie_stats_params(p.config, msp);
      frame_results->movie_stats.set_params(msp);
   }

   { // Setup TrackletTask params
      auto tp    = tracklet_task.params();
      tp.out_dir = p.config.outdir;
      if(has_task_params && success) {
         success = has_key(task_params, "tracklet_params");
         try {
            if(success)
               tp.tracklet_params
                   .read_with_defaults<decltype(tp.tracklet_params)>(
                       task_params["tracklet_params"], true, "tracklet_params");
         } catch(std::exception& e) {
            WARN(format("exception whilst reading tracklet-params: {}",
                        e.what()));
            success = false;
         }
      }
      tracklet_task.set_params(tp);
   }

   { // Setup TracksTask params
      auto tp        = tracks_task.params();
      tp.start_frame = p.config.start_frame_no;
      tp.out_dir     = p.config.outdir;

      if(has_task_params && success) {
         success = has_key(task_params, "tracks_params");
         try {
            if(success)
               tp.tracks_params.read_with_defaults<decltype(tp.tracks_params)>(
                   task_params["tracks_params"], true, "tracks_params");
         } catch(std::exception& e) {
            WARN(
                format("exception whilst reading tracks-params: {}", e.what()));
            success = false;
         }
      }

      tp.tracks_params.export_training_data_prefix
          = p.config.export_training_data_prefix;
      tp.tracks_params.ground_truth = p.ground_truth;

      { // Load classifiers
         auto init_classifier_s = [&](const string& key) -> string {
            if(!key.empty()) {
               try {
                  Classifier classifier;
                  fetch(classifier, key);
                  return classifier.write();
               } catch(std::exception& e) {
                  FATAL(format("failed to load classifier with key, '{}': {}",
                               key,
                               e.what()));
               }
            }
            return ""s;
         };

         tp.tracks_params.pose_classifier = p.config.pose_classifier;
      }

      tracks_task.set_params(tp);
   }

   { // PostProcessParams
      if(has_task_params && success) {
         success = has_key(task_params, "post_process");
         try {
            if(success) {
               params_.post_process_params
                   .read_with_defaults<decltype(params_.post_process_params)>(
                       task_params["post_process"], true, "post_process");
            }
         } catch(std::exception& e) {
            WARN(
                format("exception whilst reading tracks-params: {}", e.what()));
            success = false;
         }
      }
   }

   force_update_movie_task_params();

   return success;
}

// ----------------------------------------------------------------- task-params
//
Json::Value This::task_params() const noexcept
{
   Expects(frame_results != nullptr);
   return create_task_params(
       *frame_results, tracklet_task, tracks_task, params_.post_process_params);
}

} // namespace perceive::pipeline
