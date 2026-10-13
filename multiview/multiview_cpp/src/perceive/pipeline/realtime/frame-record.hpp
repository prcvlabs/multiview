
#pragma once

#include "perceive/graphics/image-container.hpp"
#include "perceive/utils/timestamp.hpp"

#include <chrono>

namespace perceive
{
struct LocalizationData;
class PointCloud;
} // namespace perceive

namespace perceive::pipeline::realtime
{
// ------------------------------------------------------------------ DropReason
// Why a scene frame never reached the tracker with a result of its own.
//
enum class DropReason : int {
   NONE = 0,
   NO_FREE_SLOT,   // the dispatcher took a newer frame
   TRACKER_BEHIND, // no room downstream of the slots
   SOURCE_BEHIND,  // the decoders had not finished when the frame was due
   DECODE_ERROR,   // a camera failed to decode the frame
   DAG_ERROR       // the per-frame pipeline returned no result
};

constexpr int k_n_drop_reasons = 6;

// ------------------------------------------------------------- str(DropReason)
//
inline const char* str(DropReason reason) noexcept
{
   switch(reason) {
   case DropReason::NONE: return "none";
   case DropReason::NO_FREE_SLOT: return "no_free_slot";
   case DropReason::TRACKER_BEHIND: return "tracker_behind";
   case DropReason::SOURCE_BEHIND: return "source_behind";
   case DropReason::DECODE_ERROR: return "decode_error";
   case DropReason::DAG_ERROR: return "dag_error";
   }
   return "none";
}

// -------------------------------------------------------------- to-drop-reason
// `DropReason::NONE` for an unknown name.
//
inline DropReason to_drop_reason(const string_view name) noexcept
{
   for(int i = 0; i < k_n_drop_reasons; ++i)
      if(name == str(DropReason(i))) return DropReason(i);
   return DropReason::NONE;
}

// --------------------------------------------------------------- FrameTimeline
// When a frame passed each stage. `timestamp` is the capture time of the scene
// frame; the time points are on the steady clock of this process.
//
struct FrameTimeline
{
   using time_point = std::chrono::steady_clock::time_point;

   int frame_no = -1;
   Timestamp timestamp;   // add_seconds(timestamp0, frame_no / fps)
   time_point t_due;      // when the paced clock released the frame
   time_point t_arrival;  // when the source assembled the frame
   time_point t_dispatch; // when a slot took the frame
   time_point t_dag_done; // when the slot's result was ready
   time_point t_provisional;
   time_point t_tracked;
   time_point t_revised;
   time_point t_final;
};

// ----------------------------------------------------------------- FrameRecord
// What the tracker needs from one scene frame: the same payload that the batch
// driver caches per frame. A dropped frame carries only its timeline and the
// reason.
//
struct FrameRecord
{
   FrameTimeline timeline;
   DropReason drop_reason                     = DropReason::NONE;
   shared_ptr<const LocalizationData> loc_ptr = nullptr;
   vector<shared_ptr<const LABImage>> labs;           // one per sensor
   vector<shared_ptr<const PointCloud>> point_clouds; // one per camera

   int frame_no() const noexcept { return timeline.frame_no; }
   bool is_dropped() const noexcept { return drop_reason != DropReason::NONE; }

   static FrameRecord make_dropped(const FrameTimeline& timeline,
                                   DropReason reason) noexcept
   {
      FrameRecord o;
      o.timeline    = timeline;
      o.drop_reason = reason;
      return o;
   }
};

} // namespace perceive::pipeline::realtime
