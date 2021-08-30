
#include "gui-state.hpp"

#define This GuiState

namespace perceive::gui
{
static auto steady_delta_tic(clock_type::time_point last_tic) noexcept
{
   const auto now     = clock_type::now();
   const auto elapsed = (now - last_tic);
   last_tic           = now;
   return elapsed;
}

void This::clear_events() noexcept { pending_events_.clear(); }

void This::insert_events(const vector<Event>& events) noexcept
{
   pending_events_.insert(end(pending_events_), cbegin(events), cend(events));
}

/// Appended to the event queue, but executed before SDL events
void This::post_event(Event event) noexcept
{
   pending_events_.push_back(event);
}

Event This::next_event(SDL_Window* window)
{
   if(!pending_events_.empty()) {
      // We have a stored event... so return this
      const auto event = pending_events_.front();
      pending_events_.pop_front();
      update_window_state_from_event(window, event);
      return event;
   } else if(SDL_Event sdl_event; SDL_PollEvent(&sdl_event)) {
      // Okay, we have an SDL event... push it
      return make_event(sdl_event);
   }

   // Create a timer event
   //   return TimeElapsed{steady_delta_tic(last_tic_)};
   return std::monostate{};
}

} // namespace perceive::gui

#undef This
