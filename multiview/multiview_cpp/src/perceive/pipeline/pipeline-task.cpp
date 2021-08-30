
#include "pipeline-task.hpp"

#include "perceive/utils/string-utils.hpp"
#include "perceive/utils/threads.hpp"
#include "perceive/utils/tick-tock.hpp"

#define This TaskNode

namespace perceive
{
//
bool This::debug_flag_ = false;

// ---------------------------------------------------------------- str(RunData)

static string str(const This::RunData& run_data)
{
   std::stringstream ss{""s};
   ss << format("RunData: {}", run_data.taskname()) << endl;
   int counter = 0;
   for(const auto& dat : run_data.data) {
      ss << format("   [{}] {}, p = {:p}",
                   counter++,
                   dat.first,
                   dat.second.get())
         << endl;
   }
   return ss.str();
}

// ------------------------------------------------------- subscribe/unsubscribe

void This::subscribe(TaskNode* parent) noexcept
{
   dag::add_parent(this, parent); // this task depends on 'parent'
}

void This::unsubscribe(TaskNode* parent) noexcept
{
   dag::remove_parent(this, parent);
}

// ----------------------------------------------------------- find/match parent

TaskNode* This::find_parent(string_view l) noexcept
{
   auto ii = find_if(begin(parents()), end(parents()), [&](TaskNode* x) {
      return x->taskname() == l;
   });
   return (ii == end(parents())) ? nullptr : *ii;
}

TaskNode* This::find_parent(std::function<bool(const TaskNode*)> f) const
    noexcept
{
   auto ii = find_if(begin(parents()), end(parents()), f);
   return (ii == end(parents())) ? nullptr : *ii;
}

TaskNode* This::match_parent(string_view l) noexcept
{
   return find_parent(
       [l](const TaskNode* x) { return begins_with(x->graph_label(), l); });
}

// ------------------------------------------------------- Increment Run Counter

unsigned This::increment_run_counter() noexcept
{
   return ++run_counter_; // cancels everything with a smaller run-counter
}

// ---------------------------------------------------------------------- Cancel

void This::cancel() noexcept
{
   increment_run_counter();
   status_.store(state::canceled, std::memory_order_release);
   for(auto x : children()) x->cancel();
}

// -------------------------------------------------------------- try-get-result

shared_ptr<const void> This::try_get_result() noexcept
{
   shared_ptr<const void> ret{nullptr};
   unsigned run_counter = run_counter_.load(std::memory_order_acquire);
   {
      lock_guard<decltype(padlock_)> lock(padlock_);
      if(result_ and result_run_counter_ == run_counter) {
         ret = result_; // we're done here
      }
   }

   return ret;
}

// ----------------------------------------------------------------- calc-result
// calculate-and-set-result Expects(only one thread in here at a time)
void This::calc_result(callback_type callback) noexcept
{
   if(debug_flag_) INFO(format("calc_result({})", this->taskname()));

   unsigned run_counter = run_counter_.load(std::memory_order_acquire);

   {
      decltype(result_) callback_ret{nullptr};
      {
         lock_guard<decltype(padlock_)> lock(padlock_);
         if(result_ and result_run_counter_ == run_counter) {
            if(callback) callback_ret = result_;
         } else {
            if(callback) {
               waiters_.emplace_back(run_counter, std::move(callback));
            }

            auto status = status_.load(std::memory_order_acquire);
            if(status == state::idle or status == state::canceled) {
               status_.store(state::running, std::memory_order_release);
            } else {
               return; // already running
            }
         }
      }

      if(callback_ret) {
         callback(callback_ret);
         return;
      }
   }

   if(multiview_trace_pipeline_mode())
      INFO(format("Task '{}' starting", this->taskname()));

   auto run_data = make_shared<RunData>();
   Expects(run_data != nullptr);

   // So we know when things get cancelled
   run_data->is_cancelled = [this, run_counter]() {
      return run_counter != run_counter_.load(std::memory_order_acquire);
   };

   // So we know what run this was, when saving the result
   run_data->run_counter = run_counter;
   run_data->task        = this;

   { // Try-get parent results
      auto& data = run_data->data;
      data.resize(parents().size());
      std::transform(
          begin(parents()), end(parents()), begin(data), [&](TaskNode* x) {
             return make_pair(x->taskname(), x->try_get_result());
          });
   }

   int job_counter = 0; // The number of missing parent results
   for(const auto& data : run_data->data)
      if(!data.second) job_counter++;
   run_data->job_counter = job_counter;

   if(job_counter == 0) { // go! go! go! (all parent results are there)
      calc_part2(*run_data);
      return;
   }

   // This handler is called everytime a parent result comes good.
   run_data->ready_handler
       = [run_data](unsigned index, shared_ptr<const void> ret) {
            auto& data = run_data->data;
            auto x     = run_data->task;
            Expects(index < data.size());
            Expects(data[index].second == nullptr);
            data[index].second = ret;
            int counter        = --(run_data->job_counter);
            if(counter == 0)
               schedule([x, run_data]() { x->calc_part2(*run_data); });
            Ensures(counter >= 0);
         };

   // Enqueue waiting functions for each parent result.
   int enqueued_counter = 0;
   for(auto i = 0u; i < run_data->data.size(); ++i) {
      auto& data = run_data->data[i];
      if(!data.second) {
         ++enqueued_counter;
         auto* x = run_data->task->parents()[i]; // the parent task

         {
            lock_guard<std::mutex> lock(x->padlock_);
            x->waiters_.emplace_back(x->run_counter_,
                                     [i, run_data](shared_ptr<const void> ret) {
                                        run_data->ready_handler(i, ret);
                                     });
         }

         auto f = [x]() { x->calc_result(callback_type{}); };
         schedule(std::move(f));
      }
   }

   Expects(job_counter == enqueued_counter);
}

// ------------------------------------------------------------------ calc-part2
//
void This::calc_part2(RunData& run_data) noexcept
{
   if(debug_flag_) INFO(format("calc_part2({})", this->taskname()));

   shared_ptr<const void> ret = nullptr;

   // The 'ready_hander' has a pointer to `run_data` inside of it.
   // Clearing the ready-handler means that `run_data` will be
   // destroyed at the correct time.
   run_data.ready_handler = [](unsigned, shared_ptr<const void>) {};

   const bool has_all_parents
       = std::all_of(cbegin(run_data.data),
                     cend(run_data.data),
                     [](const auto& x) { return x.second != nullptr; });

   try {
      if(!run_data.is_cancelled()) {
         if(has_all_parents) {
            const auto now     = tick();
            ret                = calculate_(run_data);
            const auto seconds = tock(now);
            if(timing_callback_) timing_callback_(this, seconds);
            if(multiview_trace_pipeline_mode())
               INFO(format("Task '{}' completed in {}s",
                           this->taskname(),
                           seconds));
         } else {
            LOG_ERR(format("Task '{}' cannot start because it is missing "
                           "required inputs: ",
                           this->taskname()));
            for(const auto& data : run_data.data)
               if(!data.second) cout << format("   {}\n", data.first);
            cout << endl;
         }
      }
   } catch(...) {
      FATAL("uncaught exception in calculate() violates preconditions");
   }

   bool do_callback   = false;
   bool was_cancelled = false;
   decltype(waiters_) waiters;
   waiters.reserve(100);
   {
      lock_guard<decltype(padlock_)> lock(padlock_);
      if(has_all_parents and !run_data.is_cancelled()) {
         result_             = ret;
         result_run_counter_ = run_data.run_counter;
         do_callback         = true;

      } else {
         ret = nullptr; // it was cancelled
      }

      { // Update status, and get ready to execute waiters
         status_.store(state::idle, std::memory_order_release);
         auto ii = std::partition(
             begin(waiters_), end(waiters_), [&](const auto& x) -> bool {
                return x.first > run_data.run_counter;
             });

         for(auto jj = ii; jj != end(waiters_); ++jj) {
            Expects(jj->first <= run_data.run_counter);
            waiters.emplace_back(std::move(*jj));
         }

         waiters_.erase(ii, end(waiters_));
      }
   }

   for(auto& f : waiters) {
      Expects(f.first <= run_data.run_counter);
      Expects(f.second);
      f.second(f.first == run_data.run_counter ? ret : nullptr);
   }

   if(debug_flag_) INFO(format("DONE calc_part2({})", this->taskname()));
}

} // namespace perceive
