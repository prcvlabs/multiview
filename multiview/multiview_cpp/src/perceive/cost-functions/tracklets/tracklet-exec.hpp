
#pragma once

#include "perceive/cost-functions/fowlkes/fowlkes.hpp"
#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/cost-functions/pose-skeleton/pose-3d.hpp"
#include "perceive/geometry/skeleton/interpolate.hpp"
#include "perceive/geometry/skeleton/p2d-affinity.hpp"
#include "perceive/geometry/skeleton/skeleton-2d.hpp"
#include "perceive/io/struct-meta.hpp"

namespace perceive
{
struct LocalizationData;

class Tracklet
{
 public:
   struct Params final : public MetaCompatible
   {
      virtual ~Params() {}
      const vector<MemberMetaData>& meta_data() const noexcept override;

      // If `2` and frame0.t = 20, then we join
      // edges from frame #20 to frames [21..23]
      unsigned frame_t_interp_window   = 4; // sliding window over time
      unsigned max_frames_per_tracklet = 10;
      float speed_median               = 1.4f; // 2.0 meters per second
      float speed_stddev               = 1.0f; // 2.0 meters per second
      float speed_cap    = 3.5f; // 3.0 m/s is the fastest we consider
      float noise_factor = 0.4f; // expected noise in floor X positions
      float projective_distance_threshold = 55.0f; // per frame
   };

   Params p;
   int start_frame = 0;
   int n_frames    = 0;

   struct FrameData
   {
      shared_ptr<const LocalizationData> loc_ptr;
      vector<const Skeleton2DInfo*> p2d_info_ptrs; // p2ds are in `loc`

      vector<shared_ptr<const LABImage>> sensor_labs;     // full-image labs
      vector<shared_ptr<const PointCloud>> cam_pt_clouds; //

      bool operator==(const FrameData&) const noexcept;
      bool operator!=(const FrameData&) const noexcept;
      size_t memory_usage() const noexcept;
   };

   vector<FrameData> frames; // size is [1..max_frames_per_tracklet].

   // Read/write to/from a stream
   void read(FILE* fp) noexcept(false);
   void write(FILE* fp) const noexcept;
   string to_string() const noexcept;

   bool operator==(const Tracklet& o) const noexcept;
   bool operator!=(const Tracklet& o) const noexcept;

   friend string str(const Tracklet& tt) noexcept { return tt.to_string(); }

   size_t memory_usage() const noexcept;

   static Tracklet execute(
       const SceneDescription& scene_desc,
       const Params& p,
       const int frame0,
       const int n_frames,
       const int max_frames_per_tracklet,
       const bool feedback,
       std::function<shared_ptr<const LocalizationData>(int frameno)> get_loc,
       std::function<shared_ptr<const LABImage>(int frameno, int sensor_no)>
           get_lab,
       std::function<shared_ptr<const PointCloud>(int frameno, int camera_num)>
           get_pt_cloud,
       std::function<bool()> is_cancelled) noexcept(false);
};

} // namespace perceive
