
#include "tracks-task.hpp"

#include "perceive/utils/spin-lock.hpp"

namespace perceive::pipeline::tracks
{
static Vector2 get_top_left(const pipeline::movie::Result& movie_ret)
{
   Expects(movie_ret.n_frames() > 0);
   return movie_ret.localization_result(movie_ret.start_frame())
       ->bounds.top_left();
}

static real get_hist_sz(const pipeline::movie::Result& movie_ret)
{
   Expects(movie_ret.n_frames() > 0);
   return movie_ret.localization_result(movie_ret.start_frame())->hist_sz;
}

struct TrackCalculator
{
 private:
   // State variables
   shared_ptr<const pipeline::tracklet::Result> tracklet_ret_ = nullptr;
   const int movie_start_frame                                = 0;
   const int movie_n_frames                                   = 0;
   const int movie_end_frame                                  = 0;
   const real frame_duration                                  = 0.0;
   const Vector2 top_left                                     = {};
   const real hist_sz                                         = 0.0;
   const int frames_per_tracklet_                             = 0;
   const int start_idx                                        = -1;
   const int end_idx                                          = -1;
   const LocalizationData::Params loc_params                  = {};
   const Tracks::Params tracks_params                         = {};
   const string outdir                                        = "/tmp"s;
   const bool feedback                                        = false;

   enum CalcState : int8_t { EMPTY, CALCULATING, DONE };

   struct Job
   {
      shared_ptr<const Tracklet> tracklet = nullptr;
      int idx                             = -1; // the `idx` for the tracklet
      CalcState state                     = EMPTY;
      vector<std::function<void(Job&)>> completions
          = {}; // padlocked when called
   };

   static constexpr std::size_t k_job_cache_size = 4; // we cache up to 4 jobs
   std::deque<Job> jobs;
   vector<Tracks> tracks;
   vector<CalcState> marks; // which tracks are calculated
   mutable SpinLock padlock;

   std::size_t n_jobs() const
   {
      lock_guard<decltype(padlock)> lock(padlock);
      return jobs.size();
   }

 public:
   TrackCalculator(const Tracks::Params& tracks_p_,
                   shared_ptr<const tracklet::Result> tracklets_ret_,
                   const string_view in_outdir,
                   const bool feedback_)
       : tracklet_ret_(tracklets_ret_)
       , movie_start_frame(int(movie_ret().start_frame()))
       , movie_n_frames(int(movie_ret().n_frames()))
       , movie_end_frame(movie_ret().end_frame())
       , frame_duration(scene_desc().frame_duration())
       , top_left(get_top_left(movie_ret()))
       , hist_sz(get_hist_sz(movie_ret()))
       , frames_per_tracklet_(tracklets_ret_->max_frames_per_tracklet())
       , start_idx(movie_start_frame / frames_per_tracklet_)
       , end_idx(movie_end_frame / frames_per_tracklet_)
       , loc_params(movie_ret().loc_params())
       , tracks_params(tracks_p_)
       , outdir(in_outdir)
       , feedback(feedback_)
   {
      Expects(movie_n_frames > 0);
      Expects(start_idx >= 0);
      Expects(start_idx <= end_idx);

      tracks.resize(size_t(end_idx - start_idx + 1));
      marks.resize(tracks.size(), EMPTY); //
   }

   const tracklet::Result& tracklets() const noexcept { return *tracklet_ret_; }

   const SceneDescription& scene_desc() const noexcept
   {
      return tracklet_ret_->scene_desc();
   }

   const Tracklet::Params& tracklet_params() const noexcept
   {
      return tracklet_ret_->params().tracklet_params;
   }

   const movie::Result& movie_ret() const noexcept
   {
      return tracklet_ret_->movie_ret();
   }

   int n_frames() const noexcept { return movie_n_frames; }

   int n_tracks_objects() const noexcept { return int(tracks.size()); }

   int frames_per_tracklet() const noexcept { return frames_per_tracklet_; }

   int frame_no_to_track_object_idx(int frame_no) const noexcept
   {
      if(frames_per_tracklet_ <= 0) return -1;
      return frame_no / int(frames_per_tracklet_);
   }

   // ----------------------------------------------- Report
   void report(const int idx) const
   {
      const int start_frame       = idx * frames_per_tracklet_;
      const int end_frame         = start_frame + frames_per_tracklet_ - 1;
      const size_t movie_usage    = movie_ret().memory_usage();
      const size_t tracklet_usage = 0; // TODO, fix this
      const size_t tracks_usage   = vector_memory_usage(
          tracks, [](const Tracks& tts) { return tts.memory_usage(); });

      sync_write([&]() {
         cout << format(
             "{}{}MEMORY USAGE track [{}..{}], movie={}, tracklet={}, "
             "tracks={}{}",
             ANSI_COLOUR_BLUE_BG,
             ANSI_COLOUR_WHITE,
             start_frame,
             end_frame,
             mem_usage_str(movie_usage),
             mem_usage_str(tracklet_usage),
             mem_usage_str(tracks_usage),
             ANSI_COLOUR_RESET)
              << endl;
      });
   }

   // --------------------------------- Calculate All Tracks

   TrackerResult get_tracker_result()
   {
      // Get all the tracks
      vector<const Tracks*> tracks;
      {
         tracks.reserve(size_t(end_idx - start_idx + 1));
         for(auto idx = start_idx; idx <= end_idx; ++idx) {
            const auto now = tick();

            const Tracks* track = get_track(idx);
            tracks.push_back(track);
            Expects(track);

            INFO(format(
                "Track frames {:04d}-{:04d}, amortized time {:4.3f}s per frame",
                track->start_frame,
                track->start_frame + track->n_frames,
                tock(now) / real(track->n_frames)));
         }
      }

      { // Sanity
         if(tracks.size() == 0) {
            LOG_ERR(format("attempt to get tracks result, because apparently "
                           "there's no frames!"));
            return {};
         }
      }

      // Create the result
      TrackerResult ret;
      ret.start_frame    = movie_start_frame;
      ret.n_frames       = movie_n_frames;
      ret.hist_w         = tracks[0]->w;
      ret.hist_h         = tracks[0]->h;
      ret.top_left       = top_left;
      ret.hist_sz        = hist_sz;
      ret.frame_duration = frame_duration;

      // How many tracks could we be looking at?
      auto calc_max_id = [&]() {
         int max_id = -1;
         for(auto& ptr : tracks)
            for(const auto& tt : ptr->seqs)
               if(tt.id() > max_id) max_id = tt.id();
         return max_id;
      };

      { // Combine all the node-sequences, merging as necessary
         ret.seqs.resize(size_t(calc_max_id() + 1));
         vector<bool> marks(ret.seqs.size(), false);
         for(const auto& track : tracks) {
            for(const auto& seq : track->seqs) {
               Expects(size_t(seq.id()) < ret.seqs.size());
               if(!marks[size_t(seq.id())]) {
                  ret.seqs[size_t(seq.id())] = seq;  // copy
                  marks[size_t(seq.id())]    = true; // so we append next time
               } else {
                  ret.seqs[size_t(seq.id())].append(seq); // append
               }
            }
         }
      }

      { // Remove all empty tracks
         auto ii = std::stable_partition(
             begin(ret.seqs), end(ret.seqs), [&](const auto& seq) {
                return seq.size() > 0;
             });
         ret.seqs.erase(ii, end(ret.seqs));
      }

      return ret;
   }

   // --------------------------------- get track
   const Tracks* get_track(const int idx)
   {
      if(idx < start_idx || idx > end_idx) return nullptr; // out of bounds
      const Tracks* out = get_if_done(idx);
      if(out == nullptr) { get_track_worker(idx); }
      return &tracks[size_t(idx - start_idx)];
   }

   const Tracks* get_if_done(const int idx)
   {
      Expects(idx >= start_idx && idx <= end_idx);
      bool keep_waiting = true;
      while(keep_waiting) {
         {
            lock_guard<decltype(padlock)> lock(padlock);
            switch(marks[size_t(idx - start_idx)]) {
            case EMPTY:
               marks[size_t(idx - start_idx)]
                   = CALCULATING; // force others to wait
               keep_waiting = false;
               break;
            case CALCULATING: Expects(keep_waiting); break; // keep waiting
            case DONE:
               keep_waiting = false;                    // yes this is redundent
               return &tracks[size_t(idx - start_idx)]; // Done
            }
         }

         if(keep_waiting) // try to schedule other work
            if(!try_one_scheduled_job()) std::this_thread::yield();
      }
      return nullptr;
   }

   Result::CompDataEnvelope get_track_worker(const int idx)
   {
      Result::CompDataEnvelope comp_dat;

      if(idx < start_idx && idx > end_idx) return comp_dat;

      // There's a problem with multiple threads reading mp4 files
      // (from scene-desc)
      static std::mutex track_padlock;
      lock_guard<std::mutex> lock(track_padlock);

      bool do_save = false;
      {
         lock_guard<decltype(padlock)> lock(padlock);
         do_save = (marks[size_t(idx - start_idx)] == CALCULATING);
      }

      ///////////////////////////////////////////////////
      // NOTE: Once CALCULATING, we /must/ get to DONE //
      ///////////////////////////////////////////////////

      const int prev_idx = idx - 1; // may be out of range

      // We actually need the previous track
      if(prev_idx >= start_idx && get_track_state(prev_idx) != DONE) {
         // We have to calculate all tracks before `idx`
         for(auto i = start_idx; i < idx; ++i) get_track_worker(i);
         Expects(get_track_state(prev_idx) == DONE);
      }

      Tracks* prev_track = (idx == start_idx)
                               ? nullptr
                               : &tracks[size_t(prev_idx - start_idx)];

      std::tie(comp_dat.prev_tracklet, comp_dat.this_tracklet)
          = wait_for_this_tracklet(idx);
      // const auto [prev_tracklet, this_tracklet] =
      // wait_for_this_tracklet(idx);
      if(feedback) report(idx);

      try {
         comp_dat.data_ptr = perceive::tracks::calc_tracks_comp_data(
             scene_desc(),
             loc_params,
             tracklet_params(),
             tracks_params,
             frame_duration,
             frames_per_tracklet(),
             comp_dat.this_tracklet.get(),
             comp_dat.prev_tracklet.get(),
             prev_track,
             []() { return false; },
             outdir,
             feedback);

         if(do_save)
            tracks.at(size_t(idx - start_idx)) = comp_dat.data_ptr->execute();
      } catch(std::exception& e) {
         LOG_ERR(format("uncaught exception in `calc_tracks`: {}", e.what()));
         assert(false);
         Expects(false);
      }

      if(do_save) {
         lock_guard<decltype(padlock)> lock(padlock);
         Expects(marks[size_t(idx - start_idx)] == CALCULATING);
         marks[size_t(idx - start_idx)] = DONE;
      }

      return comp_dat;
   }

 private:
   CalcState get_track_state(const int idx)
   {
      Expects(idx >= start_idx && idx <= end_idx);
      lock_guard<decltype(padlock)> lock(padlock);
      return marks[size_t(idx - start_idx)];
   }

   // Must return `{prev_tracklet, this_tracklet}`.
   // May need to schedule the jobs, once-and-only-once,
   // And cannot be overrun
   std::pair<shared_ptr<const Tracklet>, shared_ptr<const Tracklet>>
   wait_for_this_tracklet(const int idx)
   {
      Expects(idx >= start_idx && idx <= end_idx);

      auto poke_job = [this](int idx,
                             shared_ptr<const Tracklet>& ptr,
                             bool wait, // increment n-pending-jobs
                             std::atomic<int>& n_pending_jobs) {
         if(idx < start_idx || idx > end_idx) {
            return; // nothing to schedule: index out of range
         }

         // Returns TRUE if scheduling is required
         auto lookup_job = [&](const int idx) -> bool {
            Expects(idx >= 0);
            lock_guard<decltype(padlock)> lock(padlock);

            auto ii = std::find_if(
                begin(jobs), end(jobs), [&](auto& o) { return o.idx == idx; });

            if(ii == end(jobs)) {
               // Before we push, make sure that we're not going over size
               if(jobs.size() >= k_job_cache_size) { // pop DONE jobs
                  std::stable_partition(
                      begin(jobs), end(jobs), [&](const auto& o) {
                         return o.state == DONE;
                      });
                  while(jobs.size() >= k_job_cache_size
                        && jobs.front().state == DONE)
                     jobs.pop_front();
               }

               jobs.emplace_back(); // schedule a new job...
               Job& job  = jobs.back();
               job.idx   = idx;
               job.state = CALCULATING;

               return true; // schedule the job
            }

            auto& job = *ii;
            if(job.state == DONE) {
               Expects(job.tracklet != nullptr);
               ptr = job.tracklet;
            } else if(wait) { // add a completion handler
               n_pending_jobs.fetch_add(1, std::memory_order_relaxed);
               job.completions.push_back([&n_pending_jobs, &ptr](Job& job) {
                  // Padlock is locked when completion is called
                  Expects(job.tracklet != nullptr);
                  Expects(job.state == DONE);
                  ptr = job.tracklet;
                  n_pending_jobs.fetch_sub(1, std::memory_order_release);
               });
            }

            return false; // nothing to schedule
         };

         const auto schedule_job = lookup_job(idx);

         if(schedule_job) {
            if(wait) n_pending_jobs.fetch_add(1, std::memory_order_relaxed);
            const int frame_no = idx * frames_per_tracklet_;
            schedule([idx, &ptr, &n_pending_jobs, frame_no, wait, this]() {
               shared_ptr<const Tracklet> ret_ptr = nullptr;
               try {
                  ret_ptr = tracklet_ret_->tracklets(frame_no,
                                                     []() { return false; });
               } catch(std::exception& e) {
                  FATAL(format("unhandled exception calculting tracklets from "
                               "job #{} (frame-no = {}.): {}",
                               idx,
                               frame_no,
                               e.what()));
               }

               {
                  lock_guard<decltype(padlock)> lock(padlock);
                  auto ii = std::find_if(begin(jobs), end(jobs), [&](auto& o) {
                     return o.idx == idx;
                  });
                  Expects(ii != cend(jobs));
                  Expects(ii->tracklet == nullptr);
                  Expects(ret_ptr != nullptr);
                  Expects(ii->state == CALCULATING);
                  ii->tracklet = ret_ptr;
                  ii->state    = DONE;
                  for(const auto& f : ii->completions)
                     f(*ii);               // run completions
                  ii->completions.clear(); // and free resources
               }
               if(wait) {
                  ptr = ret_ptr;
                  n_pending_jobs.fetch_sub(1, std::memory_order_release);
               }
            });
         }

         auto wait_pending_is_done = [&]() {
            while(n_pending_jobs.load(std::memory_order_acquire) != 0)
               if(!try_one_scheduled_job()) std::this_thread::yield();
         };

         // there seems to be an issue with out-of-order reading of video
         wait_pending_is_done();
      };

      std::atomic<int> n_pending_jobs{0}; // number of scheduled jobs

      shared_ptr<const Tracklet> prev_tracklet = nullptr;
      shared_ptr<const Tracklet> this_tracklet = nullptr;
      shared_ptr<const Tracklet> next_tracklet = nullptr;

      poke_job(idx - 1, prev_tracklet, true, n_pending_jobs);  // wait
      poke_job(idx + 0, this_tracklet, true, n_pending_jobs);  // wait
      poke_job(idx + 1, next_tracklet, false, n_pending_jobs); // don't wait

      return {prev_tracklet, this_tracklet}; // And we're done
   }
};

// ---------------------------------------------------------------------- Params
//
const vector<MemberMetaData>& Params::meta_data() const noexcept
{
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(Params, COMPATIBLE_OBJECT, tracks_params, true));
      m.push_back(MAKE_META(Params, INT, start_frame, true));
      m.push_back(MAKE_META(Params, BOOL, feedback, false));
      m.push_back(MAKE_META(Params, STRING, out_dir, false));
      return m;
   };

   static vector<MemberMetaData> meta = make_meta();
   return meta;
}

// --------------------------------------------------------------- Result::Pimpl
//
struct Result::Pimpl
{
   Result* parent = nullptr;
   Params p;
   mutable TrackCalculator track_calc; // thread-safe

   Pimpl(Result* in_parent,
         const Params& p_,
         shared_ptr<const tracklet::Result> tracklets_ret_)
       : parent(in_parent)
       , p(p_)
       , track_calc(p_.tracks_params, tracklets_ret_, p.out_dir, p.feedback)
   {}

   const SceneDescription& scene_desc() const noexcept
   {
      return track_calc.scene_desc();
   }

   const tracklet::Result& tracklets() const noexcept
   {
      return track_calc.tracklets();
   }

   int frame_no_to_track_object_idx(int frame_no) const noexcept
   {
      return track_calc.frame_no_to_track_object_idx(frame_no);
   }

   int n_frames() const noexcept { return track_calc.n_frames(); }

   int n_tracks_objects() const noexcept
   {
      return track_calc.n_tracks_objects();
   }

   const Tracks* tracks(int frame_no) const noexcept
   {
      return tracks_ind(frame_no_to_track_object_idx(frame_no));
   }

   const Tracks* tracks_ind(int idx) const noexcept
   {
      return track_calc.get_track(idx);
   }

   TrackerResult get_tracker_result() const noexcept
   {
      return track_calc.get_tracker_result();
   }

   Result::CompDataEnvelope calc_comp_data(int idx) const noexcept
   {
      return track_calc.get_track_worker(idx);
   }
};

Result::Result(const Params& p, shared_ptr<const tracklet::Result> ret)
    : pimpl_(make_unique<Pimpl>(this, p, ret))
{}
Result::~Result() = default;

const SceneDescription& Result::scene_desc() const noexcept
{
   return pimpl_->scene_desc();
}

const Params& Result::params() const noexcept { return pimpl_->p; }

const movie::Result& Result::movie_ret() const noexcept
{
   return tracklets().movie_ret();
}

const tracklet::Result& Result::tracklets() const noexcept
{
   return pimpl_->tracklets();
}

int Result::frame_no_to_track_object_idx(int frame_no) const noexcept
{
   return pimpl_->frame_no_to_track_object_idx(frame_no);
}

int Result::n_frames() const noexcept { return pimpl_->n_frames(); }

int Result::n_tracks_objects() const noexcept
{
   return pimpl_->n_tracks_objects();
}

const Tracks* Result::tracks(int frame_no) const noexcept
{
   return pimpl_->tracks(frame_no);
}

const Tracks* Result::tracks_ind(int idx) const noexcept
{
   return pimpl_->tracks_ind(idx);
}

TrackerResult Result::get_tracker_result() const noexcept
{
   return pimpl_->get_tracker_result();
}

vector<calibration::TrainingDataPoint>
Result::get_training_data() const noexcept
{
   auto count_n_training_data_points = [&]() {
      int counter = 0;
      for(auto i = 0; i < n_tracks_objects(); ++i) {
         const auto tracks_ptr = tracks_ind(i);
         Expects(tracks_ptr != nullptr);
         counter += tracks_ptr->training_data.size();
      }
      return counter;
   };

   vector<calibration::TrainingDataPoint> out;
   out.reserve(size_t(count_n_training_data_points()));
   for(auto i = 0; i < n_tracks_objects(); ++i) {
      const auto tracks_ptr = tracks_ind(i);
      Expects(tracks_ptr != nullptr);
      out.insert(end(out),
                 cbegin(tracks_ptr->training_data),
                 cend(tracks_ptr->training_data));
   }

   Expects(out.size() == size_t(count_n_training_data_points()));
   return out;
}

Result::CompDataEnvelope Result::calc_comp_data(int idx) const noexcept
{
   return pimpl_->calc_comp_data(idx);
}

// --------------------------------------------------------------------- execute
//
shared_ptr<const Result>
Task::execute(const RunData& data,
              const Params& params,
              std::function<bool()> is_cancelled) const noexcept
{
   if(is_cancelled()) return nullptr;

   auto tracklets_ret = data.match_result<tracklet::Result>("tracklets");
   if(!tracklets_ret) {
      LOG_ERR("Can't find dependency: 'tracklets'");
      return nullptr;
   }

   if(tracklets_ret->n_frames() == 0) {
      LOG_ERR("Movie requires at least 1 frame");
      return nullptr;
   }

   if(tracklets_ret->params().tracklet_params.max_frames_per_tracklet == 0) {
      LOG_ERR("'tracklets-params::tracklet_params.max_frames_per_tracklet' "
              "cannot be zero!");
      return nullptr;
   }

   return perceive::make_shared<Result>(params, tracklets_ret);
}

} // namespace perceive::pipeline::tracks
