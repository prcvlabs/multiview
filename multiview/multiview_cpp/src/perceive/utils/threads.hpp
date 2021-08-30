
#pragma once

#include <atomic>
#include <functional>
#include <thread>
#include <vector>

namespace perceive
{
unsigned hardware_concurrency();
void schedule(std::function<void()> f);
bool try_one_scheduled_job();
void dispose_schedular();

class ParallelJobSet
{
 private:
   std::vector<std::function<void()>> Fs;

 public:
   ParallelJobSet()                      = default;
   ParallelJobSet(const ParallelJobSet&) = delete;
   ParallelJobSet(ParallelJobSet&&)      = default;
   ~ParallelJobSet()                     = default;
   ParallelJobSet& operator=(const ParallelJobSet&) = delete;
   ParallelJobSet& operator=(ParallelJobSet&&) = default;

   void reserve(size_t sz); // ensure capacity for jobs

   void schedule(std::function<void()>);
   void execute_condition_var(); // Uses condition variables to wait
   void execute();               // Re-uses the current thread during execute
   void execute_non_parallel();  // Doesn't use threads
};

// ------------------------------------------------------------------ lock-guard
// A schedular aware lock-guard
template<class mutex_type> class lock_guard
{
 private:
   mutex_type& mutex_;

 public:
   explicit lock_guard(mutex_type& mutex)
       : mutex_(mutex)
   {
      while(!mutex_.try_lock()) {
         if(!try_one_scheduled_job()) std::this_thread::yield();
      }
   }

   ~lock_guard() { mutex_.unlock(); }

   lock_guard(lock_guard const&) = delete;
   lock_guard& operator=(lock_guard const&) = delete;
};

// ---------------------------------------------------- non-automatic-lock-guard
// A schedular aware lock-guard
template<typename mutex_type> class non_automatic_lock_guard
{
 private:
   mutex_type& padlock_;
   bool did_lock_;

 public:
   explicit non_automatic_lock_guard(mutex_type& padlock,
                                     bool lock_automatically = false)
       : padlock_(padlock)
       , did_lock_(false)
   {
      if(lock_automatically) perform_lock();
   }

   ~non_automatic_lock_guard()
   {
      if(did_lock_) padlock_.unlock();
   }

   non_automatic_lock_guard(non_automatic_lock_guard const&) = delete;
   non_automatic_lock_guard& operator=(non_automatic_lock_guard const&)
       = delete;

   void perform_lock()
   {
      assert(!did_lock_); // Can only be called once
      did_lock_ = true;
      while(!padlock_.try_lock())
         if(!try_one_scheduled_job()) std::this_thread::yield();
   }
};

} // namespace perceive
