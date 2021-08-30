
#pragma once

#include <atomic>
#include <thread>

#include <time.h>

namespace perceive
{
/**
 * @ingroup threads
 * @brief Lock free light-weight "`mutex`" with small memory footprint. Useful
 * in situations where locks are held for a small amount of time.
 */
class SpinLock
{
 private:
   std::atomic<bool> padlock_{false};

   static inline void nano_sleep_(unsigned nanos) noexcept
   {
      timespec ts;
      ts.tv_sec  = 0;
      ts.tv_nsec = nanos;
      nanosleep(&ts, nullptr);
   }

 public:
   /**
    * @brief Obtain a lock, blocking the thread if necessary.
    * The `SpinLock` automatically backs off if busy waiting does not succeed
    * quickly. Will deadlock if the `SpinLock` is already locked.
    */
   void lock() noexcept
   {
      for(uint64_t attempts = 0; !try_lock(); ++attempts) {
         if(attempts < 4)
            ; // just try again
         else if(attempts < 64)
            std::this_thread::yield(); // back off
         else
            nano_sleep_(2000); // back off more (2 microseconds)
      }
   }

   /**
    * @brief Optimisically acquire a lock, returning `true` if successful.
    *        Does not block.
    */
   bool try_lock() noexcept
   {
      // The lock succeeds _only_ if `padlock_` goes from FALSE to TRUE
      bool success = !std::atomic_exchange_explicit(
          &padlock_, true, std::memory_order_acquire);
      return success;
   }

   /**
    * @brief Unlocks. Non-blocking.
    */
   void unlock() noexcept
   {
      std::atomic_store_explicit(&padlock_, false, std::memory_order_release);
   }

#ifndef NDEBUG
   /**
    * @brief Returns `true` if locked. May return spurious results. Not a safe
    *        way to test before attempting to obtain a lock. (Attempting to do
    *        this indicates a flaw in the program design.) This function is
    *        useful for `assert()` statements.
    */
   bool is_locked() const noexcept // For assert only!
   {
      return std::atomic_load_explicit(&padlock_, std::memory_order_relaxed);
   }
#endif
};

} // namespace perceive
