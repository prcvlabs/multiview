
#pragma once

#include <chrono>
#include <condition_variable>

namespace perceive::pipeline::realtime
{
// -------------------------------------------------------------------- SlotPool
// `n_slots` slots, each busy with at most one piece of work at a time.
// `dispatch` waits for an idle slot, marks it busy, and calls a start function
// on the calling thread, which must arrange for `release` to be called exactly
// once when the slot's work is done (on any thread). A busy slot is never
// taken back. `drain` waits until every slot is idle; the destructor drains.
//
// Every wait is a plain condition variable: a thread that waits here never
// runs a job of the global thread pool. Do not call `dispatch` or `drain` from
// a pool thread whose job the slots' work waits for.
//
template<typename Slot> class SlotPool
{
 public:
   using start_function = std::function<void(Slot& slot, unsigned index)>;

   // `dispatch` without a timeout
   static constexpr std::chrono::milliseconds k_wait_forever{-1};

 private:
   mutable std::mutex padlock_;
   std::condition_variable slot_released_;
   vector<unique_ptr<Slot>> slots_;
   vector<char> is_busy_;
   unsigned n_busy_       = 0;
   unsigned max_busy_     = 0;
   uint64_t n_dispatched_ = 0;
   uint64_t n_released_   = 0;

   int acquire_(std::chrono::milliseconds timeout) noexcept;

 public:
   explicit SlotPool(vector<unique_ptr<Slot>> slots);
   SlotPool(const SlotPool&) = delete;
   SlotPool(SlotPool&&)      = delete;
   ~SlotPool() { drain(); }
   SlotPool& operator=(const SlotPool&) = delete;
   SlotPool& operator=(SlotPool&&)      = delete;

   unsigned n_slots() const noexcept { return unsigned(slots_.size()); }
   unsigned n_busy() const noexcept;
   unsigned max_busy() const noexcept; // most slots busy at once, lifetime
   uint64_t n_dispatched() const noexcept;
   uint64_t n_released() const noexcept;
   bool is_busy(unsigned index) const noexcept;
   Slot& slot(unsigned index) noexcept { return *slots_[index]; }
   const Slot& slot(unsigned index) const noexcept { return *slots_[index]; }

   // Waits up to `timeout` for an idle slot (0 does not wait). False if there
   // is none, or if `start` throws, in which case the slot is idle again.
   bool dispatch(std::chrono::milliseconds timeout,
                 const start_function& start) noexcept;

   // The slot's work is done. Called once per successful `dispatch`.
   void release(unsigned index) noexcept;

   // Blocks until every slot is idle.
   void drain() noexcept;
};

// ---------------------------------------------------------- set-params-forcing
// Sets the params of a pipeline task, and cancels it when no registered member
// changed (`set_params` cancels only on a change). A slot that is given the
// same frame number again therefore recomputes, instead of handing back the
// previous frame's memoised result.
//
template<typename Task, typename Params>
void set_params_forcing(Task& task, const Params& params) noexcept
{
   if(!task.set_params(params)) task.cancel();
}

// ---------------------------------------------------------------- Construction
//
template<typename Slot>
SlotPool<Slot>::SlotPool(vector<unique_ptr<Slot>> slots)
    : slots_(std::move(slots))
{
   Expects(slots_.size() > 0);
   for(const auto& slot : slots_) Expects(slot != nullptr);
   is_busy_.resize(slots_.size(), 0);
}

// ---------------------------------------------------------------------- n-busy
//
template<typename Slot> unsigned SlotPool<Slot>::n_busy() const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   return n_busy_;
}

// -------------------------------------------------------------------- max-busy
//
template<typename Slot> unsigned SlotPool<Slot>::max_busy() const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   return max_busy_;
}

// ---------------------------------------------------------------- n-dispatched
//
template<typename Slot> uint64_t SlotPool<Slot>::n_dispatched() const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   return n_dispatched_;
}

// ------------------------------------------------------------------ n-released
//
template<typename Slot> uint64_t SlotPool<Slot>::n_released() const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   return n_released_;
}

// --------------------------------------------------------------------- is-busy
//
template<typename Slot>
bool SlotPool<Slot>::is_busy(unsigned index) const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   return index < is_busy_.size() and is_busy_[index] != 0;
}

// --------------------------------------------------------------------- acquire
// The index of a slot that is now busy, or -1 after `timeout`.
//
template<typename Slot>
int SlotPool<Slot>::acquire_(std::chrono::milliseconds timeout) noexcept
{
   std::unique_lock<std::mutex> lock(padlock_);
   auto has_idle = [this]() { return n_busy_ < slots_.size(); };
   if(timeout == k_wait_forever)
      slot_released_.wait(lock, has_idle);
   else if(timeout.count() > 0)
      slot_released_.wait_for(lock, timeout, has_idle);
   if(!has_idle()) return -1;

   const auto ii = std::find(begin(is_busy_), end(is_busy_), 0);
   Expects(ii != end(is_busy_));
   *ii = 1;
   ++n_busy_;
   ++n_dispatched_;
   max_busy_ = std::max(max_busy_, n_busy_);
   return int(std::distance(begin(is_busy_), ii));
}

// -------------------------------------------------------------------- dispatch
//
template<typename Slot>
bool SlotPool<Slot>::dispatch(std::chrono::milliseconds timeout,
                              const start_function& start) noexcept
{
   const int index = acquire_(timeout);
   if(index < 0) return false;
   try {
      start(*slots_[size_t(index)], unsigned(index));
   } catch(std::exception& e) {
      LOG_ERR(format("exception starting slot {}: {}", index, e.what()));
      release(unsigned(index));
      return false;
   }
   return true;
}

// --------------------------------------------------------------------- release
//
template<typename Slot> void SlotPool<Slot>::release(unsigned index) noexcept
{
   {
      std::lock_guard<std::mutex> lock(padlock_);
      if(index >= is_busy_.size() or is_busy_[index] == 0) {
         LOG_ERR(format("slot {} released, but it was not busy", index));
         return;
      }
      is_busy_[index] = 0;
      --n_busy_;
      ++n_released_;
   }
   slot_released_.notify_all();
}

// ----------------------------------------------------------------------- drain
//
template<typename Slot> void SlotPool<Slot>::drain() noexcept
{
   std::unique_lock<std::mutex> lock(padlock_);
   slot_released_.wait(lock, [this]() { return n_busy_ == 0; });
}

} // namespace perceive::pipeline::realtime
