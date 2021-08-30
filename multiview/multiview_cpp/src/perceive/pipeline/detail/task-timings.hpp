
#pragma once

#include "perceive/graphics/image-container.hpp"

namespace perceive
{
class TaskTimings
{
 public:
   struct TimingInfo
   {
      std::string name;
      unsigned count{0};      // to convert sums into averages
      real sum_start{0.0};    // seconds, starting offset
      real sum_duration{0.0}; // seconds

      real average_start() const noexcept { return sum_start / real(count); }
      real average_duration() const noexcept
      {
         return sum_duration / real(count);
      }
   };

 private:
   mutable std::mutex padlock_;
   std::unordered_map<string, TimingInfo> timings_;
   decltype(tick()) tick_;

 public:
   CUSTOM_NEW_DELETE(TaskTimings)

   // Returns timings, THREAD_SAFE
   std::vector<TimingInfo> timings() const noexcept;

   // Update the timing for a given task.
   // Start-time is automatically calculated as offset from 'tick_'
   // THREAD_SAFE
   void update_timing(const string& name, const real duration) noexcept;

   // Reset 'tick_'... call at the start of each frame's execution
   // THREAD_SAFE
   void update_tick() noexcept;

   // THREAD_SAFE
   void clear() noexcept;
};

ARGBImage render_timings(const TaskTimings&) noexcept;

} // namespace perceive
