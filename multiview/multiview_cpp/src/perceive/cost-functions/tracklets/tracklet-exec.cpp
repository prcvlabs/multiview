
#include "tracklet-exec.hpp"

#include "tracklet-ops.hpp"

#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/cost-functions/tracklets/merge-aliased-op.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/tiny-string.hpp"
#include "perceive/io/fp-io.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/movie/movie-utils.hpp"
#include "perceive/optimization/hungarian-algorithm.hpp"

#define This Tracklet

template<typename T, class less_eq = std::less_equal<T>>
inline constexpr bool inclusive_between_(const T low_bound,
                                         const T value,
                                         const T high_bound,
                                         less_eq leq = std::less_equal<T>{})
{
   return leq(value, high_bound) && leq(low_bound, value);
}

namespace perceive
{
// ------------------------------------------------------------------- meta-data
//
const vector<MemberMetaData>& This::Params::meta_data() const noexcept
{
   auto make_meta = [&]() {
      vector<MemberMetaData> m;
      m.push_back(
          MAKE_META(This::Params, UNSIGNED, max_frames_per_tracklet, true));
      m.push_back(
          MAKE_META(This::Params, UNSIGNED, frame_t_interp_window, true));
      m.push_back(MAKE_META(This::Params, FLOAT, speed_median, true));
      m.push_back(MAKE_META(This::Params, FLOAT, speed_stddev, true));
      m.push_back(MAKE_META(This::Params, FLOAT, speed_cap, true));
      m.push_back(MAKE_META(This::Params, FLOAT, noise_factor, true));
      m.push_back(
          MAKE_META(This::Params, FLOAT, projective_distance_threshold, true));
      return m;
   };
   static vector<MemberMetaData> meta = make_meta();
   return meta;
}

// ------------------------------------------------------------------ operator==
//
bool This::FrameData::operator==(const FrameData& o) const noexcept
{
   auto p2ds_eq = [](const auto& p2ds0, const auto& p2ds1) -> bool {
      if(p2ds0.size() != p2ds1.size()) return false;
      for(auto i = 0u; i < p2ds0.size(); ++i)
         if(*p2ds0[i] != *p2ds1[i]) return false;
      return true;
   };

   auto test_ptr = [](const auto& A, const auto& B) {
      if(A == nullptr && B == nullptr) return true;
      if(A != nullptr && B != nullptr) return *A == *B;
      return false;
   };

#define TEST(x) (x == o.x)
   return test_ptr(loc_ptr, o.loc_ptr)
          && p2ds_eq(p2d_info_ptrs, o.p2d_info_ptrs);
#undef TEST
}

bool This::FrameData::operator!=(const FrameData& o) const noexcept
{
   return !(*this == o);
}

bool This::operator==(const Tracklet& o) const noexcept
{
#define TEST(x) (x == o.x)
   return TEST(p) and TEST(start_frame) and TEST(n_frames) and TEST(frames);
#undef TEST
}

bool This::operator!=(const Tracklet& o) const noexcept
{
   return !(*this == o);
}

// ------------------------------------------------- Read/write to/from a stream
//
static string k_tracklet_magic_number = "tracklet-magic_v2"s;

void This::read(FILE* fp) noexcept(false) { FATAL("Not implemented"); }

void This::write(FILE* fp) const noexcept { FATAL("Not implemented"); }

// ------------------------------------------------------------------- to-string
//
string This::to_string() const noexcept
{
   return format(R"V0G0N(
{{
   "type":                   "TRACKLET",
   "start-frame":             {},
   "n-frames":                {},
   "max-frames-per-tracklet": {},
   "params": {}
}}
{})V0G0N",
                 start_frame,
                 n_frames,
                 p.max_frames_per_tracklet,
                 indent(p.to_string(), 3),
                 "");
}

// ---------------------------------------------------------------- memory-usage

size_t This::memory_usage() const noexcept
{
   return sizeof(This)
          + vector_memory_usage(frames, [&](const auto& frame_dat) {
               return frame_dat.memory_usage();
            });
}

size_t This::FrameData::memory_usage() const noexcept
{
   return sizeof(This) + (loc_ptr ? loc_ptr->memory_usage() : size_t(0))
          + p2d_info_ptrs.capacity() * sizeof(void*)
          + vector_memory_usage(sensor_labs,
                                [&](const auto& lab_ptr) {
                                   return (lab_ptr == nullptr)
                                              ? 0
                                              : lab_ptr->memory_usage();
                                })
          + vector_memory_usage(cam_pt_clouds, [&](const auto& ptr) {
               return (ptr == nullptr) ? 0 : ptr->memory_usage();
            });
}

// ----------------------------------------------------------- execute-hungarian
//
static Tracklet execute_op_hungarian(
    const SceneDescription& scene_desc,
    const This::Params& p,
    const int frame0,
    const int n_frames,
    const int max_frames_per_tracklet,
    const bool feedback,
    std::function<const LocalizationData*(int frameno)> in_get_loc,
    std::function<bool()> is_cancelled) noexcept
{
   FATAL(format("NOT IMPLEMENTED"));
   return {};
} // namespace perceive

// -----------------------------------------------------------
//
static Tracklet tracker_v3_process(
    const SceneDescription& scene_desc,
    const Tracklet::Params& params,
    const int start_frame,
    const int n_frames,
    const int max_frames_per_tracklet,
    const bool feedback,
    std::function<shared_ptr<const LocalizationData>(int)> get_localization,
    std::function<shared_ptr<const LABImage>(int frameno, int sensor_no)>
        in_get_lab,
    std::function<shared_ptr<const PointCloud>(int frameno, int camera_num)>
        get_pt_cloud,
    std::function<bool()> is_cancelled) noexcept
{
   Expects(n_frames > 0);
   Expects(n_frames <= max_frames_per_tracklet);
   const int end_frame = start_frame + n_frames - 1;

   const int n_cameras = scene_desc.n_cameras();
   const int n_sensors = scene_desc.n_sensors();

   // Cache the LABs
   vector<vector<shared_ptr<const LABImage>>> labs(
       (size_t(n_frames))); // let's cache these
   for(size_t i = 0; i < size_t(n_frames); ++i) {
      labs[i].resize(size_t(n_sensors));
      for(size_t j = 0; j < size_t(n_sensors); ++j) {
         labs[i][j] = in_get_lab(int(i + size_t(start_frame)), int(j));
         Expects(labs[i][j] != nullptr);
      }
   }

   auto get_lab = [&](int frame_no, int sensor_no) -> const LABImage* {
      Expects(sensor_no < n_sensors);
      Expects(frame_no >= start_frame);
      Expects(frame_no <= end_frame);
      return labs[size_t(frame_no - start_frame)][size_t(sensor_no)].get();
   };

   auto make_frame_data = [&](const int frameno) {
      const auto loc_ptr = get_localization(frameno);
      Expects(loc_ptr != nullptr);
      Expects(loc_ptr->invariant_check());
      Tracklet::FrameData fd;
      fd.loc_ptr = loc_ptr;
      // fd.loc_ptr->p2ds.clear();
      // fd.loc_ptr->p2ds.shrink_to_fit(); // free memory here

      {
         size_t sz = 0;
         for(const auto& sensor_p2ds : loc_ptr->p2ds) sz += sensor_p2ds.size();

         fd.p2d_info_ptrs.reserve(sz);
         for(const auto& sensor_p2ds : loc_ptr->p2ds)
            for(const auto& o : sensor_p2ds) fd.p2d_info_ptrs.push_back(&o);
         Expects(sz == fd.p2d_info_ptrs.size());
      }

      Expects(frameno >= start_frame && frameno <= end_frame);
      fd.sensor_labs = labs[size_t(frameno - start_frame)];

      fd.cam_pt_clouds.resize(size_t(n_cameras));
      for(auto i = 0; i < n_cameras; ++i) {
         fd.cam_pt_clouds[size_t(i)] = get_pt_cloud(frameno, i);
         // Note, fd.cam_pt_clouds[i] could be nullptr!
      }

      return fd;
   };

   Tracklet ret = {};
   ParallelJobSet pjobs;

   { // Setup the initial frame data
      ret.frames.resize(size_t(n_frames));
      auto process_i = [&](int i) {
         ret.frames[size_t(i)] = make_frame_data(start_frame + i);
      };
      for(auto t = start_frame; t <= end_frame; ++t)
         pjobs.schedule([i = t - start_frame, &process_i]() { process_i(i); });
      pjobs.execute();
   }

   ret.start_frame = start_frame;
   ret.n_frames    = n_frames;
   ret.p           = params;

   return ret;
}

// --------------------------------------------------------------------- execute
//
Tracklet This::execute(
    const SceneDescription& scene_desc,
    const Params& p,
    const int frame0,
    const int n_frames,
    const int max_frames_per_tracklet,
    const bool feedback,
    std::function<shared_ptr<const LocalizationData>(int)> get_localization,
    std::function<shared_ptr<const LABImage>(int frameno, int sensor_no)>
        get_lab,
    std::function<shared_ptr<const PointCloud>(int frameno, int camera_num)>
        get_pt_cloud,
    std::function<bool()> is_cancelled) noexcept(false)
{
   Expects(n_frames > 0);

   const auto now = tick();

   if(feedback) {
      TRACE(format("tracklet-exec: [{}..{}], with max-frames-per-tracklet={}",
                   frame0,
                   frame0 + n_frames,
                   max_frames_per_tracklet));
   }

   auto ret = tracker_v3_process(scene_desc,
                                 p,
                                 frame0,
                                 n_frames,
                                 max_frames_per_tracklet,
                                 feedback,
                                 get_localization,
                                 get_lab,
                                 get_pt_cloud,
                                 is_cancelled);

   const auto execute_s = tock(now);

   { // Paranoid
      int counter = 0;
      for(const auto& fdat : ret.frames) {
         const int frame_no = frame0 + counter++;

         for(const auto& sensor_p2ds : fdat.loc_ptr->p2ds) {
            for(const auto& pinfo : sensor_p2ds) {
               Expects(pinfo.p2d_ptr);
               Expects(pinfo.p2d_ptr->is_interpolation() == false);
            }
         }
         for(const auto pinfo_ptr : fdat.p2d_info_ptrs) {
            Expects(pinfo_ptr);
            Expects(pinfo_ptr->p2d_ptr);
            Expects(pinfo_ptr->p2d_ptr->is_interpolation() == false);
         }

         if(false) { //
            Expects(fdat.sensor_labs.size() > 0);
            const auto argb = LAB_im_to_argb(*fdat.sensor_labs.at(0));
            argb.save(format("/tmp/zzz-{:04d}_0.png", frame_no));
         }
      }
   }

   return ret;
}

} // namespace perceive
