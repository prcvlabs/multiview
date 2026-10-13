
#include "shard-range.hpp"

#include "perceive/cost-functions/localization/localization-data.hpp"

namespace perceive::pipeline
{
// ------------------------------------------------------- shard-alignment-error
//
std::optional<string>
shard_alignment_error(const int start,
                      const int n,
                      const int total,
                      const int k,
                      const LocalizationDataEnvelope* env) noexcept
{
   const int end = start + n; // exclusive

   if(k <= 0)
      return format("max-frames-per-tracklet must be positive, but it is {}",
                    k);

   if(n < 2)
      return format("a shard needs at least 2 frames, but [{}, {}) has {}",
                    start,
                    end,
                    n);

   if(start < 0 or end > total)
      return format("shard [{}, {}) is outside the movie's frames [0, {})",
                    start,
                    end,
                    total);

   if(start % k != 0)
      return format("shard start frame {} is not a multiple of "
                    "max-frames-per-tracklet = {}",
                    start,
                    k);

   if(end % k != 0 and end != total)
      return format("shard end frame {} (exclusive) is neither a multiple of "
                    "max-frames-per-tracklet = {} nor the movie's end {}",
                    end,
                    k,
                    total);

   if(env != nullptr) {
      const int env_start = int(env->start_frame);
      const int env_end   = env_start + int(env->loc_data.size());
      if(start < env_start or end > env_end)
         return format("shard [{}, {}) is not inside the frames [{}, {}) of "
                       "the loaded localization data",
                       start,
                       end,
                       env_start,
                       env_end);
   }

   return std::nullopt;
}

} // namespace perceive::pipeline
