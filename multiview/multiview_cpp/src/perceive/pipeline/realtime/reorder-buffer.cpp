
#include "reorder-buffer.hpp"

namespace perceive::pipeline::realtime
{
// ---------------------------------------------------------------- make-dropped
//
static FrameRecord make_dropped(int frame_no, DropReason reason) noexcept
{
   FrameTimeline timeline;
   timeline.frame_no = frame_no;
   return FrameRecord::make_dropped(timeline, reason);
}

// ---------------------------------------------------------------- Construction
//
ReorderBuffer::ReorderBuffer(int first_frame_no,
                             std::chrono::nanoseconds missing_timeout)
    : next_frame_no_(first_frame_no)
    , missing_timeout_(missing_timeout)
{}

// --------------------------------------------------------------- next-frame-no
//
int ReorderBuffer::next_frame_no() const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   return next_frame_no_;
}

// ------------------------------------------------------------------- n-records
//
size_t ReorderBuffer::n_records() const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   return records_.size();
}

// ---------------------------------------------------------------------- n-late
//
uint64_t ReorderBuffer::n_late() const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   return n_late_;
}

// ------------------------------------------------------------------- n-expired
//
uint64_t ReorderBuffer::n_expired() const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   return n_expired_;
}

// ------------------------------------------------------------------- find-drop
// The caller holds `padlock_`
//
const ReorderBuffer::DropRange*
ReorderBuffer::find_drop_(int frame_no) const noexcept
{
   auto ii = drops_.upper_bound(frame_no);
   if(ii == cbegin(drops_)) return nullptr;
   --ii;
   return (frame_no <= ii->second.to) ? &ii->second : nullptr;
}

// ------------------------------------------------------------------- has-later
// The caller holds `padlock_`. True if anything after `frame_no` is in.
//
bool ReorderBuffer::has_later_(int frame_no) const noexcept
{
   if(!records_.empty() and crbegin(records_)->first > frame_no) return true;
   return std::any_of(cbegin(drops_), cend(drops_), [frame_no](const auto& o) {
      return o.second.to > frame_no;
   });
}

// -------------------------------------------------------------- update-blocked
// The caller holds `padlock_`. Starts the missing-frame clock when the next
// frame is missing and holds up later ones.
//
void ReorderBuffer::update_blocked_(time_point now) noexcept
{
   const bool is_in = records_.count(next_frame_no_) > 0
                      or find_drop_(next_frame_no_) != nullptr;
   const bool is_blocked = !is_in and has_later_(next_frame_no_);
   if(is_blocked and !is_blocked_) blocked_since_ = now;
   is_blocked_ = is_blocked;
}

// -------------------------------------------------------------------- pop-next
// The caller holds `padlock_`, and the next frame is in.
//
FrameRecord ReorderBuffer::pop_next_() noexcept
{
   const int frame_no = next_frame_no_++;
   is_blocked_        = false;

   FrameRecord record;
   auto ii = records_.find(frame_no);
   if(ii != end(records_)) {
      record = std::move(ii->second);
      records_.erase(ii);
   } else {
      const auto drop_ptr = find_drop_(frame_no);
      record              = make_dropped(frame_no,
                            (drop_ptr == nullptr) ? DropReason::DAG_ERROR
                                                               : drop_ptr->reason);
   }

   // Ranges that are all behind us
   for(auto jj = begin(drops_); jj != end(drops_);)
      jj = (jj->second.to < next_frame_no_) ? drops_.erase(jj) : std::next(jj);

   return record;
}

// ------------------------------------------------------------------------ push
//
bool ReorderBuffer::push(FrameRecord&& record, time_point now) noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   const int frame_no = record.frame_no();
   if(frame_no < next_frame_no_ or records_.count(frame_no) > 0) {
      ++n_late_;
      return false;
   }
   records_.emplace(frame_no, std::move(record));
   update_blocked_(now);
   return true;
}

// ----------------------------------------------------------------- insert-drop
// The caller holds `padlock_`, and no range has a frame in [from, to].
//
void ReorderBuffer::insert_drop_(int from, int to, DropReason reason) noexcept
{
   // Coalesce with a range that ends just before, for the same reason
   auto ii = drops_.upper_bound(from);
   if(ii != cbegin(drops_)) {
      auto& prev = std::prev(ii)->second;
      if(prev.reason == reason and prev.to + 1 == from) {
         prev.to = to;
         return;
      }
   }

   DropRange range;
   range.to     = to;
   range.reason = reason;
   drops_[from] = range;
}

// ------------------------------------------------------------------- push-drop
// Frames already in a range keep its reason; the rest of [from, to] go into
// new ranges, so that ranges never overlap.
//
void ReorderBuffer::push_drop(int from,
                              int to,
                              DropReason reason,
                              time_point now) noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   from = std::max(from, next_frame_no_);
   if(to < from) return;

   while(true) {
      const auto drop_ptr = find_drop_(from);
      int last            = to; // [from, last] is all in a range, or in none
      if(drop_ptr != nullptr) {
         last = std::min(to, drop_ptr->to);
      } else {
         const auto next = drops_.upper_bound(from);
         if(next != cend(drops_)) last = std::min(to, next->first - 1);
         insert_drop_(from, last, reason);
      }
      if(last == to) break;
      from = last + 1;
   }
   update_blocked_(now);
}

// ------------------------------------------------------------------- pop-ready
//
std::optional<FrameRecord> ReorderBuffer::pop_ready(time_point now) noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   update_blocked_(now);

   const bool is_in = records_.count(next_frame_no_) > 0
                      or find_drop_(next_frame_no_) != nullptr;
   if(is_in) return pop_next_();

   const bool has_timeout = missing_timeout_.count() > 0;
   if(is_blocked_ and has_timeout
      and now - blocked_since_ >= missing_timeout_) {
      LOG_ERR(format("frame {} never came back from the pipeline; dropping it",
                     next_frame_no_));
      ++n_expired_;
      return pop_next_();
   }

   return std::nullopt;
}

// ----------------------------------------------------------------------- flush
//
vector<FrameRecord> ReorderBuffer::flush() noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   vector<FrameRecord> out;
   while(has_later_(next_frame_no_ - 1)) out.push_back(pop_next_());
   return out;
}

} // namespace perceive::pipeline::realtime
