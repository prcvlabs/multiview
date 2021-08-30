
#pragma once

#include "perceive/cost-functions/floor-histogram.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/scene/scene-description-info.hpp"
//#include "perceive/pipeline/params.hpp"

namespace perceive
{
struct MovieStatsFile
{
   // pipeline::Params params;         // used to create file
   SceneDescriptionInfo scene_info; // ibid
   AABB bounds;                     // bounds on histogram
                                    // (for converting cells to R^3)
   real hist_sz{0.0};
   unsigned N{0}; // number of frames

   // Store statistics on histogram cells
   struct Stat
   {
      float average{0.0};
      float stddev{0.0};
   };

   using HistStat = ImageContainerT<Stat>;
   HistStat hist_stats; // orthographic view of floor

   unsigned w() const { return hist_stats.width; }
   unsigned h() const { return hist_stats.height; }

   bool initialize(const SceneDescriptionInfo& scene_info,
                   const unsigned n_frames,
                   std::function<const FloorHistogram*(unsigned)> get_hist);

   struct Accumulator
   {
      AABB bounds;
      real hist_sz{0.0};
      unsigned N{0};
      real K{0.0};
      ImageContainerT<Vector2> sum_ss; // {sum, sum-of-squares}
      void incremental_update(const FloorHistogram& fhist);

      size_t memory_usage() const noexcept;
   };
   bool initialize(const SceneDescriptionInfo& scene_info,
                   const Accumulator& accumulator);

   // -- Export --
   void export_image(const string& outdir) const;
};

MovieStatsFile combine(const MovieStatsFile& a,
                       const MovieStatsFile& b) noexcept;

void load(MovieStatsFile& data, const string& fname) noexcept(false);
void save(const MovieStatsFile& data, const std::string& fname) noexcept(false);

} // namespace perceive
