
#include "tracks-index.hpp"

#define This TracksIndex

namespace perceive
{
// ------------------------------------------------------------------------ init
//
void This::init(const vector<Track>& tracks) noexcept
{
   // Get the maximum 't' value from a path of any track
   int max_t = -1;
   for(const auto& tt : tracks)
      for(const auto& tp : tt.path)
         if(max_t < tp.t) max_t = tp.t;

   data_.resize(size_t(max_t + 1));

   for(auto i = 0u; i < tracks.size(); ++i) {
      const auto& tt = tracks[i];
      for(auto j = 0u; j < tt.path.size(); ++j) {
         const int t = tt.path[j].t;
         Expects(t >= 0 and t <= max_t);
         Expects(unsigned(t) < data_.size());
         FrameItem item;
         item.track_index = int(i);
         item.tp_index    = int(j);
         data_[size_t(t)].push_back(item);
      }
   }
}

// ------------------------------------------------------------------------- get
//
std::pair<const Track*, const TrackPoint*>
This::get(const vector<Track>& tracks,
          const FrameItem& frame_item) const noexcept
{
   // (safely) load the track
   if(unsigned(frame_item.track_index) >= tracks.size())
      return {nullptr, nullptr};
   const auto tt_ptr = &tracks[size_t(frame_item.track_index)];

   // (safely) load the track-point
   if(unsigned(frame_item.tp_index) >= tt_ptr->path.size())
      return {nullptr, nullptr};
   const auto tp_ptr = &tt_ptr->path[size_t(frame_item.tp_index)];

   // We're done
   return {tt_ptr, tp_ptr};
}

} // namespace perceive
