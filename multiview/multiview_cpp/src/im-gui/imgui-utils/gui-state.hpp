
#pragma once

#include "event.hpp"

namespace perceive::gui
{
struct GuiState
{
 private:
   clock_type::time_point last_tic_ = clock_type::now();
   std::deque<Event> pending_events_;

 public:
   /// Clear any stored events
   void clear_events() noexcept;

   /// Insert events, for playback
   void insert_events(const vector<Event>& events) noexcept;

   /// Appended to the event queue
   void post_event(Event event) noexcept;

   /// Retrieve an event... either:
   /// [1] A `pending_event_`, or if none,
   /// [2] Poll the underlying system for an event, or if none,
   /// [3] A `TimeElapsed` event
   Event next_event(SDL_Window* window);
};

} // namespace perceive::gui
