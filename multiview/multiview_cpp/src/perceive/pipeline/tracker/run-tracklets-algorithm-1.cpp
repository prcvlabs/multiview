
#include "run-tracklets-algorithm-1.hpp"

#include <deque>

#include "perceive/cost-functions/fowlkes/fowlkes.hpp"
#include "perceive/cost-functions/tracklets/tracklet-exec.hpp"
#include "perceive/cost-functions/tracks/tracks-exec.hpp"
#include "perceive/io/fp-io.hpp"
#include "perceive/pipeline/movie-results.hpp"
#include "perceive/pipeline/nodes/movie-task.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/spin-lock.hpp"

namespace perceive
{
FowlkesResult run_tracklets_algorithm_1(const pipeline::MovieResults::Params& p,
                                        pipeline::MovieResults* movie) noexcept
{
   TRACE(format("tracker algorithm 1"));

   auto tracks_ret = movie->tracks_task.get_result_synchronized();

   if(p.config.export_training_data) {
      const auto fname = format("{}/training-data.json", p.config.outdir);
      INFO(format("exporting training data..."));
      const auto data = tracks_ret->get_training_data();

      const auto json_val
          = json_save_t(cbegin(data), cend(data), [&](const auto& tp) {
               return tp.to_json();
            });

      Json::StyledWriter writer;
      file_put_contents(fname, writer.write(json_val));
      INFO(format("training data exported to '{}'", fname));
   }

   // Get the complete set of tracker results
   const auto result0 = tracks_ret->get_tracker_result();

   const auto now = tick();

   // Remove ghost tracks here, and filter short tracks
   const auto result1 = post_process(result0, p.post_process_params);

   // Smooth tracks
   const auto fowlkes_result
       = smooth_tracks(p.params.fowlkes_params, result1.to_fowlkes_result());

   if(p.config.verbosity > 0)
      INFO(format("finalizing track data: {}s", tock(now)));

   return fowlkes_result;
}

} // namespace perceive
