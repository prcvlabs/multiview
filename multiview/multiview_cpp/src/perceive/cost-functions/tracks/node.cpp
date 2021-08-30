
#include "node.hpp"

#define This Node

namespace perceive::tracks
{
size_t This::memory_usage() const noexcept
{
   return sizeof(This) + sizeof(P2dAddress) * p2ds_.capacity();
}

string This::to_string() const noexcept
{
   return format("[{}, {}, {}, {}, {}, [{}]]",
                 xy().x,
                 xy().y,
                 t(),
                 gaze(),
                 prob_fp(),
                 implode(cbegin(pose_scores()), cend(pose_scores()), ", "));
}

} // namespace perceive::tracks
