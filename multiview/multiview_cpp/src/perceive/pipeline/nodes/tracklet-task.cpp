
#include "tracklet-task.hpp"

#include "perceive/movie/debug-movie.hpp"
#include "perceive/pipeline/frame-results.hpp"

namespace perceive::pipeline::tracklet
{
/**
 * @param x      Variable to update
 * @param arg    Value to add to `x`
 * @param order  The memory synchronization ordering for the read-modify-write
 *               operation if the comparison succeeds. All values are permitted.
 * @return       The value immediately preceding the effects of this function.
 */
template<typename T>
T fetch_add(std::atomic<T>& x, T arg, std::memory_order order)
{
   T current = x.load(std::memory_order_relaxed);
   while(!x.compare_exchange_weak(
       current, current + arg, order, std::memory_order_relaxed))
      ; // Loop body empty, see docs for `compare_exchange_weak`
   return current;
}

// ------------------------------------------------------------------- meta-data
//
const vector<MemberMetaData>& Params::meta_data() const noexcept
{
   auto make_meta = []() {
      vector<MemberMetaData> m;
      // m.push_back(MAKE_META(Params, UNSIGNED, max_frames, true));
      m.push_back(MAKE_META(Params, COMPATIBLE_OBJECT, tracklet_params, true));
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
   shared_ptr<const movie::Result> movie_ret;

   int full_movie_n_frames = -1;
   int movie_n_frames      = -1;
   int movie_start_frame   = -1;
   int movie_end_frame     = -1;
   int frame_sz            = -1;
   int min_idx             = -1;
   int max_idx             = -1; // inclusive

   struct TrackletEnvelope
   {
      int idx                             = -1;
      shared_ptr<const Tracklet> tracklet = nullptr;
   };

   mutable std::mutex padlock_;
   mutable std::deque<TrackletEnvelope> tracklets_;

   Pimpl(Result* in_parent,
         const Params& p_,
         shared_ptr<const movie::Result> movie_ret_)
       : parent(in_parent)
       , p(p_)
       , movie_ret(movie_ret_)
   {
      full_movie_n_frames = int(movie_ret->full_movie_n_frames());
      movie_start_frame   = int(movie_ret->start_frame());
      movie_n_frames      = int(movie_ret->n_frames());
      movie_end_frame     = movie_start_frame + movie_n_frames - 1;

      frame_sz = int(p.tracklet_params.max_frames_per_tracklet);
      min_idx  = movie_start_frame / frame_sz;
      max_idx  = movie_end_frame / frame_sz;
   }

   shared_ptr<const Tracklet>
   get_tracklet(int idx, std::function<bool()> is_cancelled) const
       noexcept(false)
   {
      auto get_ptr = [&](int idx) {
         {
            lock_guard lock(padlock_);
            for(auto& ii : tracklets_)
               if(ii.idx == idx) return ii.tracklet; // could be null

            // Okay, if we got here, then we need to start a job
            TrackletEnvelope env;
            env.idx      = idx;
            env.tracklet = nullptr;
            tracklets_.push_back(env);
         }

         return init_tracklet(idx, is_cancelled);
      };

      shared_ptr<const Tracklet> ret = get_ptr(idx);
      while(ret == nullptr && !is_cancelled()) { // wait for job completion
         if(!try_one_scheduled_job()) std::this_thread::yield();
         ret = get_ptr(idx);
      }

      return ret;
   }

   shared_ptr<const Tracklet>
   init_tracklet(int idx, std::function<bool()> is_cancelled) const
       noexcept(false)
   {
      shared_ptr<Tracklet> ret_ptr;

      const char* cmsg      = "tracklets";
      const string_view msg = cmsg;

      const int start_frame = idx * frame_sz;
      const int end_frame
          = std::min<int>(start_frame + frame_sz - 1, full_movie_n_frames - 1);
      const int n_to_process = end_frame - start_frame + 1;
      Expects(n_to_process >= 0);

      const auto& scene_desc = movie_ret->scene_desc();

      const auto now                       = tick();
      std::atomic<real> loc_result_seconds = 0.0;
      {
         // We hold the localization-data pointers here, to prevent
         // memory from being released while it's still being used.
         std::deque<shared_ptr<const LocalizationData>> hold_loc_ptrs;
         SpinLock padlock;

         auto get_loc_ptr
             = [&](unsigned t) -> shared_ptr<const LocalizationData> {
            auto loc           = movie_ret->localization_result(t, "");
            const real seconds = movie_ret->localization_result_timing(t, "");
            fetch_add(loc_result_seconds, seconds, std::memory_order_relaxed);
            {
               lock_guard lock(padlock);
               hold_loc_ptrs.push_back(loc);
            }
            Expects(loc->invariant_check());
            return loc;
         };

         auto get_lab_ptr =
             [&](unsigned t, unsigned sensor_no) -> shared_ptr<const LABImage> {
            return movie_ret->get_sensor_lab_shared_ptr(t, sensor_no, "");
         };

         auto get_pt_cloud_ptr
             = [&](unsigned t,
                   unsigned camera_no) -> shared_ptr<const PointCloud> {
            return movie_ret->get_camera_pt_cloud_ptr(t, camera_no, "");
         };

         ret_ptr = make_shared<Tracklet>(Tracklet::execute(scene_desc,
                                                           p.tracklet_params,
                                                           start_frame,
                                                           n_to_process,
                                                           frame_sz,
                                                           p.feedback,
                                                           get_loc_ptr,
                                                           get_lab_ptr,
                                                           get_pt_cloud_ptr,
                                                           is_cancelled));

         if(!is_cancelled()) {
            lock_guard lock(padlock_);
            bool done = false;
            for(auto& ii : tracklets_) {
               if(ii.idx == idx) {
                  ii.tracklet = std::move(ret_ptr);
                  done        = true;
                  break;
               }
            }

            if(!done) {
               TrackletEnvelope env;
               env.idx      = idx;
               env.tracklet = std::move(ret_ptr);
               tracklets_.push_back(env);
            }

            if(tracklets_.size() > p.max_tracklets_to_store) {
               tracklets_.pop_front();
            }
         }
      }

      if(p.feedback || multiview_trace_mode()) {
         auto to_ms = [](const real seconds) {
            return int(std::round(1000.0 * seconds));
         };
         const auto loc_ms = to_ms(loc_result_seconds);

         const auto msg0
             = format("localization took {: 5d}ms for {} frames... amortized "
                      "{: 5d}ms per frame",
                      loc_ms,
                      n_to_process,
                      to_ms((loc_result_seconds / real(n_to_process))));
         const auto s1   = tock(now);
         const auto msg1 = format(
             "tracklets    took {: 5d}ms for {} frames... amortized {: 5d}ms "
             "per frame",
             to_ms(s1),
             n_to_process,
             to_ms(s1 / real(n_to_process)));

         if(p.feedback) {
            INFO(msg0);
            INFO(msg1);
         }
      }

      return ret_ptr;
   }

   shared_ptr<const Tracklet>
   tracklets(int frame_no, std::function<bool()> is_cancelled) const
       noexcept(false)
   {
      if(frame_sz == 0) { return nullptr; }
      const int idx = frame_no / int(frame_sz);
      if(idx < min_idx or idx > max_idx) {
         TRACE(format("idx {} not in [{}..{}]", idx, min_idx, max_idx));
         return nullptr;
      }
      auto ret = get_tracklet(idx, is_cancelled);
      Expects(ret != nullptr || is_cancelled());
      return ret;
   }
};

Result::Result(const Params& p, shared_ptr<const movie::Result> ret)
    : pimpl_(make_unique<Pimpl>(this, p, ret))
{}
Result::~Result() = default;

const SceneDescription& Result::scene_desc() const noexcept
{
   Expects(pimpl_->movie_ret);
   return movie_ret().scene_desc();
}

const Params& Result::params() const noexcept { return pimpl_->p; }

const movie::Result& Result::movie_ret() const noexcept
{
   return *pimpl_->movie_ret;
}

int Result::n_frames() const noexcept
{
   Expects(pimpl_->movie_ret);
   return int(movie_ret().full_movie_n_frames());
}

int Result::n_tracklet_objects() const noexcept
{
   return int(pimpl_->tracklets_.size());
}

shared_ptr<const Tracklet>
Result::tracklets(int frame_no, std::function<bool()> is_cancelled) const
    noexcept(false)
{
   if(!is_cancelled) is_cancelled = []() { return false; };
   return pimpl_->tracklets(frame_no, is_cancelled);
}

int Result::max_frames_per_tracklet() const noexcept
{
   return int(pimpl_->p.tracklet_params.max_frames_per_tracklet);
}

// --------------------------------------------------------------------- execute
//
shared_ptr<const Result>
Task::execute(const RunData& data,
              const Params& params,
              std::function<bool()> is_cancelled) const noexcept
{
   if(is_cancelled()) return nullptr;

   if(params.tracklet_params.max_frames_per_tracklet == 0) {
      LOG_ERR("'tracklet_params.max_frames_per_tracklet' cannot be zero!");
      return nullptr;
   }

   auto movie_ret = data.match_result<movie::Result>("movie");
   if(!movie_ret) {
      LOG_ERR("Can't find dependency: 'movie-task'");
      return nullptr;
   }

   if(movie_ret->n_frames() == 0) {
      LOG_ERR("Movie requires at least 1 frame");
      return nullptr;
   }

   return ::perceive::make_shared<Result>(params, movie_ret);
}

} // namespace perceive::pipeline::tracklet
