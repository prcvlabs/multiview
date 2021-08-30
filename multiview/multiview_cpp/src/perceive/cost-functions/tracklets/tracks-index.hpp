
#pragma once

#include "track.hpp"

namespace perceive
{
class TracksIndex
{
 public:
   struct FrameItem
   {
      int track_index = -1;
      int tp_index    = -1; // track-point index within track
   };

   using FrameData = vector<FrameItem>;

 private:
   vector<FrameData> data_;

 public:
   void init(const vector<Track>& tracks) noexcept;

   // Returns nullptr if the frame-item is out of bounds
   std::pair<const Track*, const TrackPoint*>
   get(const vector<Track>& tracks, const FrameItem& frame_item) const noexcept;

   bool has_frame(int t) const noexcept { return unsigned(t) < data_.size(); }

   const vector<FrameData>& data() const noexcept { return data_; }
   const FrameData& frame_data(int t) const noexcept
   {
      Expects(size_t(t) < data_.size());
      return data_[size_t(t)];
   }
};

} // namespace perceive
