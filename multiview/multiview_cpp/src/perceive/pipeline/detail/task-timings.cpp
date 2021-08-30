
#include "task-timings.hpp"

#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/tiny-string.hpp"
#include "perceive/utils/threads.hpp"

#define This TaskTimings

namespace perceive
{
// ----------------------------------------------------------------- get timings
// Returns timings, THREAD_SAFE
std::vector<This::TimingInfo> This::timings() const noexcept
{
   std::vector<This::TimingInfo> ret;
   ret.reserve(100);
   {
      lock_guard<decltype(padlock_)> lock(padlock_);
      for(auto& ii : timings_) ret.push_back(ii.second);
   }
   return ret;
}

// --------------------------------------------------------------- update timing
// Update the timing for a given task.
// Start-time is automatically calculated as offset from 'tick_'
// THREAD_SAFE
void This::update_timing(const string& name, const real duration) noexcept
{
   lock_guard<decltype(padlock_)> lock(padlock_);
   auto whence = tock(tick_) - duration;

   auto ii = timings_.find(name);
   if(ii == timings_.end()) {
      timings_[name]  = TimingInfo{};
      ii              = timings_.find(name);
      ii->second.name = name;
      Expects(ii != timings_.end());
   }

   auto& info = ii->second;
   info.count++;
   info.sum_start += whence;
   info.sum_duration += duration;
}

// ----------------------------------------------------------------- update tick
// Reset 'tick_'... call at the start of each frame's execution
// THREAD_SAFE
void This::update_tick() noexcept
{
   lock_guard<decltype(padlock_)> lock(padlock_);
   tick_ = tick();
}

// ----------------------------------------------------------------------- clear
// THREAD_SAFE
void This::clear() noexcept
{
   lock_guard<decltype(padlock_)> lock(padlock_);
   timings_.clear();
}

// -------------------------------------------------------------- render timings

ARGBImage render_timings(const TaskTimings& in) noexcept
{
   std::vector<TaskTimings::TimingInfo> data = in.timings();
   std::sort(begin(data), end(data), [&](auto& a, auto& b) {
      return a.average_start() < b.average_start();
   });

   // How high is our font??
   auto get_tiny_font_height = []() {
      int tiny_w = 0, tiny_h = 0;
      render_tiny_dimensions("The rain in Spain...", tiny_w, tiny_h);
      return tiny_h;
   };

   const int N     = int(data.size());
   const int row_h = get_tiny_font_height() + 1; // plus1 for faint rule
   const int h     = N * row_h + 1;              // That's the fence post
   const int w     = 800;                        // just some number
   const uint32_t faint_rule_k = k_light_salmon;

   ARGBImage out(w, h);
   out.fill(k_white); // background colour

   // Fill in the background
   for(auto y = 0; y < h; ++y) {
      for(auto x = 0; x < w; ++x) {
         const int row   = y / row_h;
         const bool dark = (row / 3) % 2 == 1;
         out(x, y)       = dark ? k_light_gray : k_white;
      }
   }

   // Lets draw the faint rules, on every third row
   for(auto y = 0; y < h; y += 3 * row_h)
      for(auto x = 0; x < w; ++x)
         out(x, y) = (x % 2 == 0) ? faint_rule_k : k_white;

   // Render the task names
   int text_x = 0;
   for(auto i = 0; i < N; ++i) {
      render_string(out,
                    data[size_t(i)].name,
                    Point2(3, row_h * i + 2),
                    k_dark_slate_gray,
                    k_white);
      const auto dims = render_tiny_dimensions(data[size_t(i)].name.c_str());
      if(dims.x > text_x) text_x = dims.x; // right most pixel
   }

   // Gives the horizontal range where we render bars
   const int pixel_x0    = text_x + 3;       // 3 for the left margin
   const int pixel_range = w - pixel_x0 - 3; // 3 for the right margin

   // What is the right-most time?
   real max_end = 0.0;
   for(size_t i = 0; i < size_t(N); ++i) {
      const real end = data[i].average_start() + data[i].average_duration();
      if(end > max_end) max_end = end;
   }

   // Convert a time-point to an x-location
   auto s_to_x = [&](real seconds) -> int {
      return int(std::round(pixel_x0 + pixel_range * (seconds / max_end)));
   };

   // Draw time vertically
   for(auto t = 0.0; t <= max_end; t += 0.1) {
      const auto x = s_to_x(t);
      if(out.in_bounds(x, 0)) {
         const uint32_t k = (t == std::round(t)) ? k_red : k_gray;
         for(auto y = 0; y < h; y += 2) out(x, y) = k;
      }
   }

   // Render the duration bars
   for(size_t i = 0; i < size_t(N); ++i) {
      const auto x0 = s_to_x(data[i].average_start());
      const auto x1
          = s_to_x(data[i].average_start() + data[i].average_duration());
      fill_rect(out,
                Point2(x0, int(i) * int(row_h) + 1),
                k_orange_red,
                x1 - x0,
                row_h - 1);
   }

   return out;
}

} // namespace perceive
