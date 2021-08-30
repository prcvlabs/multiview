
#include "perceive/foundation.hpp"
#include "threads.hpp"

// #include <boost/context/all.hpp>

#include <thread>
#include <vector>

#include <condition_variable>
#include <deque>
#include <mutex>

#ifdef PERCEIVE_USE_APPLE_LIBDISPATCH
#include <dispatch/dispatch.h>
#endif

namespace perceive
{
unsigned hardware_concurrency() { return std::thread::hardware_concurrency(); }

// ---------------------------------------------------------- Notification Queue

class NotificationQueue final
{
 private:
   std::deque<std::function<void()>> q_;
   bool done_{false};
   std::mutex padlock_;
   std::condition_variable ready_;

   typedef std::unique_lock<decltype(padlock_)> lock_t;

 public:
   NotificationQueue()  = default;
   ~NotificationQueue() = default;

   // No copy
   NotificationQueue(const NotificationQueue&) = delete;
   void operator=(const NotificationQueue&) = delete;

   // No move
   NotificationQueue(NotificationQueue&&) = delete;
   void operator=(NotificationQueue&&) = delete;

   // Get front of queue if it exists. Blocking waits if it
   // doesn't exist. If 'done_' == true, will return false,
   // indiciating that nothing was returned.
   bool pop(std::function<void()>& x)
   {
      lock_t lock{padlock_};
      while(q_.empty() && !done_) ready_.wait(lock);
      if(q_.empty()) // if we are done
         return false;
      x = move(q_.front());
      q_.pop_front();
      return true;
   }

   template<typename F> void push(F&& f)
   {
      {
         lock_t lock{padlock_};
         q_.emplace_back(std::forward<F>(f));
      }
      ready_.notify_one();
   }

   bool try_pop(std::function<void()>& x)
   {
      lock_t lock{padlock_, std::try_to_lock};
      if(!lock || q_.empty()) return false;
      x = move(q_.front());
      q_.pop_front();
      return true;
   }

   template<typename F> bool try_push(F&& f)
   {
      {
         lock_t lock{padlock_, std::try_to_lock};
         if(!lock) return false;
         q_.emplace_back(std::forward<F>(f));
      }
      ready_.notify_one();
      return true;
   }

   void done()
   {
      {
         lock_t lock{padlock_};
         done_ = true;
      }
      ready_.notify_all();
   }
};

// ----------------------------------------------------- Portable Task Schedular

class PortableTaskSchedular
{
 private:
   const unsigned count_{};
   std::vector<std::thread> threads_;
   unique_ptr<NotificationQueue[]> q_;
   std::atomic<unsigned> index_{0};
   std::atomic<bool> ready_{true};

   bool try_run_one_(unsigned i)
   {
      std::function<void()> f;

      // Use (fast) try-pop to attempt to grab a function
      for(unsigned n = 0; n != count_; ++n)
         if(q_[(i + n) % count_].try_pop(f)) break;

      if(f) {
         try {
            if(ready_) f();
         } catch(...) {
            fprintf(stderr, "ERROR: uncaught exception in task schedular\n");
         }
         return true;
      }

      return false;
   }

   void run_(unsigned i)
   {
      while(true) {
         if(!try_run_one_(i)) {
            std::function<void()> f;

            // If we failed, then try a locking-pop
            if(!q_[i].pop(f))
               break; // Locking-pop only fails if queue has done signal

            Expects(f);

            // Execute function
            try {
               f();
            } catch(...) {
               fprintf(stderr, "ERROR: uncaught exception in task schedular\n");
            }
         }
      }
   }

 public:
   PortableTaskSchedular(unsigned count
                         = std::max(hardware_concurrency() * 2u, 8u))
       : count_(count)
       , q_(new NotificationQueue[count_])
   {
      if(q_.get() == nullptr) throw std::bad_alloc();
      // Boot up the threads
      threads_.reserve(count_);
      for(unsigned n = 0; n != count_; ++n)
         threads_.emplace_back([this, n] { this->run_(n); });
   }

   ~PortableTaskSchedular() { dispose(); }

   void dispose()
   {
      ready_ = false;
      for(unsigned ind = 0; ind < count_; ++ind)
         q_[ind].done();                // signal each queue to quit
      for(auto& e : threads_) e.join(); // join the threads
   }

   // F must be a thunk (std::function<void ()>)
   template<typename F> void schedule(F&& f)
   {
      if(!ready_) return;

      // Use (fast) try-push to get the thunk on a given thread.
      //
      // std::forward<F> is not an error because try_push itself
      // will not forward unless it consumes the value.
      unsigned i = index_++;
      for(unsigned n = 0; n != count_; ++n)
         if(q_[(i + n) % count_].try_push(std::forward<F>(f))) return;

      // Tiny chance that try-push never succeeded, so use locking-push
      q_[i % count_].push(std::forward<F>(f));
   }

   bool try_run_one()
   {
      static std::atomic<unsigned> counter{0};
      return try_run_one_(counter++ % count_);
   }
};

// ------------------------------------------------------- LibDispatch Schedular

#ifdef PERCEIVE_USE_APPLE_LIBDISPATCH
class LibDispatchTaskSchedular
{
 public:
   // F must be a thunk (std::function<void ()>)
   void schedule(std::function<void()> f)
   {
      using f_t = typename std::decay<decltype(f)>::type;

      auto Q = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0);
      dispatch_async_f(Q, new f_t(std::move(f)), [](void* f_) {
         auto f = static_cast<f_t*>(f_);
         (*f)();
         delete f;
      });
   }

   bool try_run_one()
   {
      FATAL("Not supported");
      return false;
   }
};
#endif

// -------------------------------------------------------------------- Schedule

#ifdef PERCEIVE_USE_APPLE_LIBDISPATCH
static inline detail::LibDispatchTaskSchedular& global_schedular()
{
   static detail::LibDispatchTaskSchedular instance;
   return instance;
}
#else
static inline PortableTaskSchedular& global_schedular()
{
   static PortableTaskSchedular instance;
   return instance;
}
#endif

void schedule(std::function<void()> f)
{
   global_schedular().schedule(std::move(f));
}

bool try_one_scheduled_job() { return global_schedular().try_run_one(); }

void dispose_schedular() { global_schedular().dispose(); }

// --------------------------------------------------------------- Parallel Jobs

void ParallelJobSet::reserve(size_t sz)
{
   if(Fs.size() < sz) Fs.reserve(sz);
}

template<typename T> static void parallel_schedule(const T& Fs)
{
   const unsigned N = unsigned(Fs.size());
   std::atomic<unsigned> counter{0};
   std::mutex padlock;
   std::condition_variable condition;

   std::unique_lock<decltype(padlock)> lock(padlock);

   for(auto& func : Fs)
      schedule([&]() {
         try {
            func();
         } catch(...) {
            LOG_ERR(format("uncaught exception"));
         }

         if(++counter >= N) condition.notify_one();
      });

   condition.wait(lock, [&]() { return counter == N; });
}

void ParallelJobSet::schedule(std::function<void()> f)
{
   Fs.emplace_back(std::move(f));
}

void ParallelJobSet::execute_non_parallel()
{
   for(auto& f : Fs) f();
   Fs.clear();
}

void ParallelJobSet::execute_condition_var()
{
   parallel_schedule(Fs);
   Fs.clear();
}

void ParallelJobSet::execute()
{
#ifdef PERCEIVE_USE_APPLE_LIBDISPATCH
   execute_condition_var();
   return;
#endif

   auto f = [&]() {
      const unsigned N = unsigned(Fs.size());
      std::atomic<unsigned> counter{0};

      auto run_job = [&](unsigned i) {
         try {
            Fs[i]();
         } catch(std::exception& e) {
            LOG_ERR(format("uncaught exception: {}", e.what()));
         } catch(...) {
            LOG_ERR(format("uncaught exception"));
         }
         ++counter;
      };

      for(unsigned i = 0; i < N; ++i) {
         ::perceive::schedule([&run_job, i]() { run_job(i); });
      }

      while(true) {
         global_schedular().try_run_one();
         if(counter >= N)
            break;
         else
            std::this_thread::yield();
      }
   };

   f();
   Fs.clear();
}

} // namespace perceive
