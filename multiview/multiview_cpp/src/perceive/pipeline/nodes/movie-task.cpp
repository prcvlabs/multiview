
#include "movie-task.hpp"

#include "perceive/io/fp-io.hpp"
#include "perceive/pipeline/detail/task-timings.hpp"
#include "perceive/pipeline/frame-results.hpp"
#include "perceive/pipeline/movie-results.hpp"
#include "perceive/utils/file-system.hpp"

namespace perceive::pipeline::movie
{
// ---------------------------------------------------------------------- Params
//
void frame_results_deleter(FrameResults* ptr)
{
   if(ptr) delete ptr;
}

Params::Params()
    : frame_results(nullptr, frame_results_deleter)
{}

Params::Params(const Params& o)
    : frame_results(nullptr, frame_results_deleter)
{
   if(o.frame_results != nullptr)
      frame_results = unique_ptr<FrameResults, void (*)(FrameResults*)>(
          new FrameResults(*o.frame_results), frame_results_deleter);
}

Params::~Params() = default;

Params& Params::operator=(const Params& o)
{
   if(this != &o) {
      //
      if(!o.frame_results) {
         frame_results.reset();
      } else {
         if(!frame_results)
            frame_results = unique_ptr<FrameResults, void (*)(FrameResults*)>(
                new FrameResults(*o.frame_results), frame_results_deleter);
         else
            *frame_results = *o.frame_results;
      }

      generate_stats = o.generate_stats;
      load_stats     = o.load_stats;
      start_frame    = o.start_frame;
      end_frame      = o.end_frame;
      feedback       = o.feedback;
      out_dir        = o.out_dir;
      ldata_env_ptr  = o.ldata_env_ptr;
   }
   return *this;
}

bool Params::operator==(const Params& o) const noexcept
{
   auto check_frame_results = [&]() {
      if(frame_results == nullptr and o.frame_results == nullptr) return true;
      if(frame_results == nullptr or o.frame_results == nullptr) return false;
      return *frame_results == *o.frame_results;
   };

#define TEST(x) x == o.x
   return true && check_frame_results() && TEST(start_frame) && TEST(end_frame);
#undef TEST
}

string Params::to_string(int indent) const noexcept
{
   return to_json_string(indent);
}

string Params::to_json_string(int spaces) const noexcept
{
   std::stringstream ss{""};
   ss << to_json() << endl;
   return indent(ss.str(), spaces);
}

Json::Value Params::to_json() const noexcept
{
   auto node = Json::Value{Json::objectValue};
   if(frame_results)
      node["frame_results"s] = frame_results->params_to_json();
   else
      WARN(format(
          "result will be invalid, because 'frame_results' was not yet set"));
   return node;
}

void Params::read(const Json::Value& node) noexcept(false)
{
   if(!frame_results)
      throw std::runtime_error(
          "must set 'frame-results' before calling 'read'");
   if(node.type() != Json::objectValue)
      throw std::runtime_error(
          "expected an Json::objectValue when reading pipeline::movie::Params");
   if(!node.isMember("frame_results"s))
      throw std::runtime_error("failed to find 'frame_results' member when "
                               "reading pipeline::movie::Params");
   if(!frame_results->read_params(node["frames_results"s], "frames_results"s))
      throw std::runtime_error("failed to read 'frame_results' parameters when "
                               "reading pipeline::movie::Params");
}

void Params::read_with_defaults(const Json::Value& o,
                                const Params* defaults) noexcept
{
   *this = *defaults;
   try {
      read(o);
   } catch(std::exception& e) {}
}

// --------------------------------------------------------------- Result::Pimpl
//
struct Result::Pimpl
{
   Params params;
   unique_ptr<FrameResults> frame_results_;
   shared_ptr<const SceneDescription> scene_desc;
   shared_ptr<const LocalizationDataEnvelope> ldata_env_ptr;
   unsigned start_frame{0};
   unsigned full_movie_n_frames{0};
   unique_ptr<TaskTimings> timings;
   LocalizationData::Params localization_params;
   mutable SpinLock padlock;

   MovieStatsFile::Accumulator stats_accumulator;

   vector<bool> floor_hist_accumulated;
   size_t frame_cache_memory_usage   = 0;
   size_t ldata_env_ptr_memory_usage = 0;

   struct FrameData
   {
      int frame_no                               = 0;
      real timing                                = 0.0;
      shared_ptr<const LocalizationData> loc_ptr = nullptr;
      vector<shared_ptr<const LABImage>> labs;
      vector<shared_ptr<const PointCloud>> point_clouds;

      bool is_valid() const noexcept
      {
         return (loc_ptr != nullptr) && (labs.size() > 0)
                && (point_clouds.size() > 0);
      }

      size_t memory_usage() const noexcept
      {
         size_t ret = sizeof(decltype(*this));
         if(loc_ptr != nullptr) ret += loc_ptr->memory_usage();
         ret += vector_memory_usage(labs, [](const auto& o) {
            return (o == nullptr) ? 0 : o->memory_usage();
         });
         ret += vector_memory_usage(point_clouds, [](const auto& o) {
            return (o == nullptr) ? 0 : o->memory_usage();
         });
         return ret;
      }
   };

   std::deque<FrameData> frame_cache;

   // -- Constructor --
   Pimpl(FrameResults& fr, const Params& in_params)
   {
      params         = in_params;
      timings        = make_unique<TaskTimings>();
      frame_results_ = make_unique<FrameResults>(fr);
      ldata_env_ptr  = in_params.ldata_env_ptr;

      timings->update_tick();
      scene_desc
          = frame_results_->load_scene_description.get_result_synchronized()
                ->scene_desc;
      full_movie_n_frames = unsigned(scene_desc->n_frames());
      floor_hist_accumulated.resize(full_movie_n_frames);
      std::fill(
          begin(floor_hist_accumulated), end(floor_hist_accumulated), false);

      if(ldata_env_ptr != nullptr)
         ldata_env_ptr_memory_usage = ldata_env_ptr->memory_usage();

      localization_params
          = frame_results_->localization.params().localization_params;
   }

   FrameResults& frame_results() noexcept { return *frame_results_; }

   unsigned n_cached_frames() const noexcept
   {
      lock_guard lock(padlock);
      return unsigned(frame_cache.size());
   }

   // -- Get-Frame --
   const FrameData* try_get_frame(int frame_no) noexcept
   {
      { // Look in cache
         lock_guard lock(padlock);
         for(auto& fd : frame_cache)
            if(fd.frame_no == frame_no) return &fd;
      }
      return nullptr;
   }

   const FrameData* get_frame(int frame_no, const string_view message) noexcept
   {
      {
         auto ret = try_get_frame(frame_no);
         if(ret) return ret;
      }

      // Okay, attempt to create the cached object
      FrameData frame = (ldata_env_ptr == nullptr)
                            ? run_frame(unsigned(frame_no), message)
                            : load_ldata_env_frame(unsigned(frame_no));
      if(frame.is_valid()) {
         lock_guard lock(padlock);
         frame_cache_memory_usage += frame.memory_usage();
         frame_cache.emplace_back(std::move(frame));

         // Observe frame size limits
         if(frame_cache.size() > params.max_frames_to_cache) {
            const auto mem = frame_cache.front().memory_usage();
            Expects(mem <= frame_cache_memory_usage);
            frame_cache_memory_usage -= mem;
            frame_cache.pop_front();
         }
         return &frame_cache.back();
      }

      // We failed =(
      return nullptr;
   }

   void
   load_lab_images_pt_clouds(unsigned frame_no,
                             vector<shared_ptr<const LABImage>>& labs,
                             vector<shared_ptr<const PointCloud>>& pt_clouds)
   {
      const int n_sensors = scene_desc->n_sensors();
      const int n_cameras = scene_desc->n_cameras();
      labs.resize(size_t(n_sensors));
      pt_clouds.resize(size_t(n_cameras));

      Expects(size_t(n_sensors) == frame_results_->convert_to_lab.size());
      Expects(size_t(n_cameras) == frame_results_->disp_map_update.size());

      if(multiview_debug_locks())
         INFO(format("MOVLAB run frame({})", frame_no));
      {
         lock_guard lock(padlock);
         frame_results_->set_frame(int(frame_no));

         std::transform(
             cbegin(frame_results_->disp_map_update),
             cend(frame_results_->disp_map_update),
             begin(pt_clouds),
             [](const auto& task_ptr) {
                auto ret = task_ptr->get_result_synchronized()->point_cloud;
                Expects(ret != nullptr);
                return ret;
             });

         std::transform(cbegin(frame_results_->convert_to_lab),
                        cend(frame_results_->convert_to_lab),
                        begin(labs),
                        [](const auto& task_ptr) {
                           auto ret
                               = task_ptr->get_result_synchronized()->lab_image;
                           Expects(ret != nullptr);
                           return ret;
                        });

         if(multiview_debug_locks())
            INFO(format("MOVLAB LOCK frame({})", frame_no));
      }
      if(multiview_debug_locks())
         INFO(format("MOVLAB LOCK frame({})", frame_no));
   }

   FrameData
   make_frame_data_(unsigned frame_no,
                    vector<shared_ptr<const LABImage>>&& labs,
                    vector<shared_ptr<const PointCloud>>&& point_clouds,
                    shared_ptr<const LocalizationData> loc_ptr)
   {
      Expects(frame_no < full_movie_n_frames);
      FrameData dat;
      dat.frame_no     = int(frame_no);
      dat.labs         = std::move(labs);
      dat.point_clouds = std::move(point_clouds);
      dat.loc_ptr      = loc_ptr;
      if(dat.labs.size() == 0 || dat.loc_ptr == nullptr) {
         dat.labs.clear();
         dat.loc_ptr = nullptr; // an invald frame
      }
      return dat;
   }

   FrameData load_ldata_env_frame(unsigned frame_no)
   {
      Expects(ldata_env_ptr != nullptr);
      if(frame_no < ldata_env_ptr->start_frame) return {};
      const unsigned index = frame_no - ldata_env_ptr->start_frame;
      if(index >= ldata_env_ptr->loc_data.size()) return {};
      vector<shared_ptr<const LABImage>> labs;
      vector<shared_ptr<const PointCloud>> pt_clouds;
      load_lab_images_pt_clouds(frame_no, labs, pt_clouds);
      auto loc_ptr
          = make_shared<LocalizationData>(ldata_env_ptr->loc_data[index]);
      return make_frame_data_(
          frame_no, std::move(labs), std::move(pt_clouds), loc_ptr);
   }

   // -- Run Frame --
   FrameData run_frame(unsigned frame_no, string_view message)
   {
      Expects(ldata_env_ptr == nullptr);

      if(frame_no >= full_movie_n_frames) return {};

      // -- Set the timing callback handler
      if(frame_no == 0) {
         frame_results_->set_task_finished_callback(nullptr);
      } else {
         frame_results_->set_task_finished_callback(
             [this](const TaskNode* task, real seconds) {
                if(!begins_with(task->taskname(), "bcam_init"s)
                   and !begins_with(task->taskname(), "get_xy_mapping"s))
                   this->timings->update_timing(task->taskname(), seconds);
             });
      }

      auto now = tick();

      const int n_sensors = scene_desc->n_sensors();
      const int n_cameras = scene_desc->n_cameras();

      shared_ptr<const LocalizationData> loc_ptr = nullptr;
      vector<shared_ptr<const LABImage>> labs((size_t(n_sensors)));
      vector<shared_ptr<const PointCloud>> point_clouds((size_t(n_cameras)));

      if(multiview_debug_locks())
         INFO(format("MOVRET run frame({})", frame_no));
      {
         lock_guard lock(padlock);
         if(multiview_debug_locks())
            INFO(format("MOVRET LOCK frame({})", frame_no));

         timings->update_tick();
         frame_results_->set_frame(int(frame_no));

         // Copy out the localization result
         auto lret = frame_results_->localization.get_result_synchronized();
         Expects(lret);
         const auto& fhist_ret = lret->floor_hist_result;

         loc_ptr = make_shared<LocalizationData>(lret->data);
         for(auto i = 0; i < n_sensors; ++i)
            labs[size_t(i)] = fhist_ret->lab_result[size_t(i)]->lab_image;
         for(auto i = 0; i < n_cameras; ++i)
            point_clouds[size_t(i)]
                = fhist_ret->disp_result[size_t(i)]->point_cloud;

         { // Accumulate the result
            const auto& fret = lret->floor_hist_result;
            Expects(fret);
            accumulate(int(frame_no), fret->hist);
         }

         const auto s = tock(now);
         if(!message.empty())
            cout << format(
                "{} - frame {:04d} - {}s", message.data(), frame_no, s)
                 << endl;
      }

      if(multiview_debug_locks())
         INFO(format("MOVRET RELEASE frame({})", frame_no));

      auto fd = make_frame_data_(
          frame_no, std::move(labs), std::move(point_clouds), loc_ptr);
      fd.timing = tock(now);
      return fd;
   }

   shared_ptr<const floor_hist::Result> calc_floor_hist(unsigned frame_no)
   {
      lock_guard lock(padlock);
      frame_results_->set_frame(int(frame_no));
      return frame_results_->floor_hist.get_result_synchronized();
   }

   void accumulate(int frame_no, const FloorHistogram& fhist)
   {
      Expects(unsigned(frame_no) < floor_hist_accumulated.size());
      if(!floor_hist_accumulated[size_t(frame_no)]) {
         floor_hist_accumulated[size_t(frame_no)] = true;
         stats_accumulator.incremental_update(fhist);
      }
   }

   void accumulate_movie_stats(const int start_frame, const int n_frames)
   {
      const int end_frame = start_frame + n_frames - 1;
      for(int i = start_frame; i <= end_frame; ++i) {
         Expects(unsigned(i) < floor_hist_accumulated.size());
         if(!floor_hist_accumulated[size_t(i)]) {
            auto now = tick();
            timings->update_tick();
            frame_results_->set_frame(i);
            auto fret = frame_results_->floor_hist.get_result_synchronized();
            Expects(fret);
            accumulate(i, fret->hist);
            cout << format(
                "generating movie stats, {}/{}, {}s", i, end_frame, tock(now))
                 << endl;
         }
      }
   }

   shared_ptr<const LocalizationData> localization(unsigned frame_no,
                                                   const string_view message)
   {
      auto dat = get_frame(int(frame_no), message);
      return (dat == nullptr) ? nullptr : dat->loc_ptr;
   }

   shared_ptr<const LocalizationData> try_get_localization(unsigned frame_no)
   {
      auto dat = try_get_frame(int(frame_no));
      return (dat == nullptr) ? nullptr : dat->loc_ptr;
   }

   real localization_timing(unsigned frame_no, const string_view message)
   {
      auto dat = get_frame(int(frame_no), message);
      return (dat == nullptr) ? 0.0 : dat->timing;
   }

   const LABImage*
   sensor_lab(unsigned frame_no, unsigned sensor_no, const string_view message)
   {
      auto dat = get_frame(int(frame_no), message);
      if(dat == nullptr) return nullptr;
      Expects(sensor_no < dat->labs.size());
      return dat->labs[sensor_no].get();
   }

   shared_ptr<const LABImage> sensor_lab_ptr(unsigned frame_no,
                                             unsigned sensor_no,
                                             const string_view message)
   {
      auto dat = get_frame(int(frame_no), message);
      if(dat == nullptr) return nullptr;
      Expects(sensor_no < dat->labs.size());
      return dat->labs[sensor_no];
   }

   shared_ptr<const LABImage> try_get_sensor_lab_ptr(unsigned frame_no,
                                                     unsigned sensor_no)
   {
      auto dat = try_get_frame(int(frame_no));
      if(dat == nullptr) return nullptr;
      Expects(sensor_no < dat->labs.size());
      return dat->labs[sensor_no];
   }

   shared_ptr<const PointCloud> cam_pt_cloud_ptr(unsigned frame_no,
                                                 unsigned camera_no,
                                                 const string_view message)
   {
      auto dat = get_frame(int(frame_no), message);
      if(dat == nullptr) return nullptr;
      Expects(camera_no < dat->point_clouds.size());
      return dat->point_clouds[camera_no];
   }

   size_t memory_usage() const noexcept
   {
      size_t ret = sizeof(decltype(*this));
      ret += (ldata_env_ptr == nullptr) ? size_t(0)
                                        : ldata_env_ptr->memory_usage();
      ret += stats_accumulator.memory_usage();
      ret += frame_cache_memory_usage;
      return ret;
   }

}; // namespace perceive::pipeline::movie

Result::Result(FrameResults& fr, const Params& params)
    : pimpl_(make_unique<Pimpl>(fr, params))
{
   //
}

Result::~Result() = default;

unsigned Result::full_movie_n_frames() const noexcept
{
   return pimpl_->full_movie_n_frames;
}

unsigned Result::start_frame() const noexcept
{
   return unsigned(pimpl_->params.start_frame);
}

unsigned Result::n_frames() const noexcept
{
   return unsigned(pimpl_->params.end_frame) - start_frame() + 1;
}

int Result::end_frame() const noexcept
{
   return int(start_frame() + n_frames()) - 1; // -1 of n-frames is 0
}

const SceneDescription& Result::scene_desc() const noexcept
{
   return *pimpl_->scene_desc;
}

unsigned Result::n_cached_frames() const noexcept
{
   return pimpl_->n_cached_frames();
}

unsigned Result::max_frames_to_cache() const noexcept
{
   return pimpl_->params.max_frames_to_cache;
}

shared_ptr<const LocalizationData>
Result::localization_result(unsigned frame_no,
                            const string_view message) const noexcept
{
   return pimpl_->localization(frame_no, message);
}

shared_ptr<const LocalizationData>
Result::try_get_localization_result(unsigned frame_no) const noexcept
{
   return pimpl_->try_get_localization(frame_no);
}

real Result::localization_result_timing(
    unsigned frame_no,
    const string_view message) const noexcept
{
   return pimpl_->localization_timing(frame_no, message);
}

const LABImage* Result::get_sensor_lab(unsigned frame_no,
                                       unsigned sensor_no,
                                       const string_view message) const noexcept
{
   return pimpl_->sensor_lab(frame_no, sensor_no, message);
}

shared_ptr<const LABImage>
Result::get_sensor_lab_shared_ptr(unsigned frame_no,
                                  unsigned sensor_no,
                                  const string_view message) const noexcept
{
   return pimpl_->sensor_lab_ptr(frame_no, sensor_no, message);
}

shared_ptr<const LABImage>
Result::try_get_sensor_lab_shared_ptr(unsigned frame_no,
                                      unsigned sensor_no) const noexcept
{
   return pimpl_->try_get_sensor_lab_ptr(frame_no, sensor_no);
}

shared_ptr<const PointCloud>
Result::get_camera_pt_cloud_ptr(unsigned frame_no,
                                unsigned camera_no,
                                const string_view message) const noexcept
{
   return pimpl_->cam_pt_cloud_ptr(frame_no, camera_no, message);
}

shared_ptr<const floor_hist::Result>
Result::floor_hist_result(unsigned frame_no) const noexcept
{
   return (frame_no < full_movie_n_frames()) ? pimpl_->calc_floor_hist(frame_no)
                                             : nullptr;
}

const TaskTimings& Result::task_timings() const noexcept
{
   return *pimpl_->timings;
}

// Save 'accumulated' stats in whatever form they are
bool Result::save_movie_stats(const string_view out_filename) const noexcept
{
   bool success = false;
   auto& P      = *pimpl_;
   MovieStatsFile stats;
   stats.initialize(scene_desc().scene_info, P.stats_accumulator);
   try {
      save(stats, string(out_filename));
      INFO(format("saving stats to {}", out_filename));
      success = true;
   } catch(std::exception& e) {
      LOG_ERR(format("exception saving stats file: {}", e.what()));
   }
   return success;
}

bool Result::gen_and_save_movie_stats(
    const int start_frame,
    const int n_frames,
    const string_view out_filename) const noexcept
{
   pimpl_->accumulate_movie_stats(start_frame, n_frames);
   return save_movie_stats(out_filename);
}

const LocalizationData::Params& Result::loc_params() const noexcept
{
   return pimpl_->localization_params;
}

size_t Result::memory_usage() const noexcept { return pimpl_->memory_usage(); }

vector<shared_ptr<const LABImage>>
Result::frame_labs(int frame_no, const string_view message) const
{
   auto dat = pimpl_->get_frame(frame_no, message);
   if(dat == nullptr) return {};
   return dat->labs;
}

// --------------------------------------------------------------------- execute
//
shared_ptr<const Result>
Task::execute(const RunData& data,
              const Params& params,
              std::function<bool()> is_cancelled) const noexcept
{
   if(is_cancelled()) return nullptr;

   if(params.frame_results == nullptr) {
      LOG_ERR(format("must set 'frame-results' before calling execute"));
      return nullptr;
   }

   const auto scene_desc
       = params.frame_results->load_scene_description.get_result_synchronized()
             ->scene_desc;

   auto movie_dat = make_shared<Result>(*params.frame_results, params);

   // ---- Generate stats
   if(params.generate_stats and !params.load_stats) {
      if(movie_dat->pimpl_->ldata_env_ptr != nullptr) {
         FATAL("logic error: shouldn't be generating stats when loading "
               "localization data");
      }
      movie_dat->pimpl_->frame_results_->set_feedback(false);
      const auto filename
          = params.frame_results->movie_stats.params().save_filename;
      const int n_frames = params.end_frame - params.start_frame + 1;
      movie_dat->gen_and_save_movie_stats(
          params.start_frame, n_frames, filename);
      movie_dat->pimpl_->frame_results_->set_feedback(params.feedback);
   }

   return movie_dat;
}

} // namespace perceive::pipeline::movie
