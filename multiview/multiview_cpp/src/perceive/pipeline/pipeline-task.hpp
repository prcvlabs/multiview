
#pragma once

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>

#include "perceive/utils/dag-node.hpp"
#include "perceive/utils/timestamp.hpp"

namespace perceive
{
class TaskNode;

/**
 * TaskNode
 *
 * Concrete class implement "dag-node.hpp" concept.
 *
 * Invariants:
 * + Object lifetime is managed outside of DAG. So
 *   never copy or move objects. In `multiview`, we
 *   keep all tasknotes in a single "frame-results"
 *   object, and link them together there.
 * + Results are always given as "shared_ptr<const void>",
 *   and cast to "shared_ptr<const T>" in the PipelineTask
 *   below. Const means _never_ modify a results object once
 *   generated.
 *
 *   ALSO
 *
 *   -- And this is IMPORTANT --
 *
 *   If a result object (of type T) needs to refer to another
 *   "parent" result object (of type U), then the result object
 *   must store a shared_ptr<const U> inside type T.
 *
 *   That is, all results must refer to other results through a
 *   chain of const shared pointers. This is guaranteed to be
 *   threadsafe (so long as types are thread-compatible), and
 *   memory is seamlessly discarded _after_ the result is no
 *   longer needed.
 *
 *   This means that result shared_ptr<const T> must be shared
 *   by value, and the object using the result keeps the value.
 *
 *   FOR EXAMPLE: refer to "superpixel_example.hpp", and note
 *   that the Result object keeps a COPY of
 *
 *           shared_ptr<const split_image::Result>
 *
 *   This copy exists so that the 'make_label_image' method can
 *   render the superpixel boundaries on the split-image. The
 *   copy of the shared_ptr is made in superpixels::execute(...).
 *   No further consideration of memory management or thread-saftey
 *   is necessary.
 */

class TaskNode
{
 public:
   using callback_type = std::function<void(shared_ptr<const void>)>;

   CUSTOM_NEW_DELETE(TaskNode)

   struct RunData
   {
    public:
      string outdir = "/tmp"s;
      bool feedback = true;
      vector<std::pair<string, shared_ptr<const void>>> data;
      std::function<bool()> is_cancelled;

    private:
      friend class TaskNode;
      std::function<void(unsigned index, shared_ptr<const void>)> ready_handler;
      std::atomic<int> job_counter = 0;
      unsigned run_counter         = 0;
      TaskNode* task               = nullptr;

    public:
      RunData()               = default;
      RunData(const RunData&) = delete;
      RunData(RunData&&)      = delete;
      ~RunData()              = default;
      RunData& operator=(const RunData&) = delete;
      RunData& operator=(RunData&&) = delete;

      const string& taskname() const noexcept { return task->taskname(); }

      template<typename T>
      shared_ptr<const T> find_result(const string_view name) const
      {
         auto f  = [&](auto& s) { return s == name; };
         auto ii = find_if(cbegin(data), cend(data), f);
         return ii == cend(data)
                    ? nullptr
                    : std::static_pointer_cast<const T>(ii->second);
      }

      template<typename T>
      shared_ptr<const T> match_result(const string_view name) const
          noexcept(false)
      {
         auto f  = [&](auto& s) { return begins_with(s.first, name); };
         auto ii = find_if(cbegin(data), cend(data), f);
         return ii == cend(data)
                    ? nullptr
                    : std::static_pointer_cast<const T>(ii->second);
      }
   };

 private:
   std::mutex padlock_;
   std::string taskname_;
   std::vector<TaskNode*> parents_;         // memory is not owned
   std::vector<TaskNode*> children_;        // memory is not owned
   std::atomic<unsigned> run_counter_ = 0;  // for cancelling
   unsigned result_run_counter_       = 0;  // run-counter for this result
   shared_ptr<const void> result_{nullptr}; // memory owned here
   std::function<void(const TaskNode* task, real seconds)> timing_callback_;

   unsigned increment_run_counter() noexcept;

   enum state : int { idle, running, canceled };
   std::atomic<state> status_ = state::idle;

   std::vector<std::pair<unsigned, callback_type>>
       waiters_; // callbacks are once-off

   void calc_part2(RunData& run_data) noexcept;

 protected:
   virtual shared_ptr<const void> calculate_(const RunData& run_data) noexcept
       = 0;
   virtual void shallow_copy_params_(const TaskNode* o) noexcept = 0;

   static bool debug_flag_;

 public:
   TaskNode(std::string taskname = ""s)
       : taskname_(
             taskname.empty()
                 ? format("TASK({:p})", reinterpret_cast<const void*>(this))
                 : taskname)
   {}
   TaskNode(const TaskNode&) = delete;            // no copy
   TaskNode(TaskNode&&)      = delete;            // no move
   TaskNode& operator=(const TaskNode&) = delete; // no copy
   TaskNode& operator=(TaskNode&&) = delete;      // no move

   virtual ~TaskNode() = default;

   // Getters/Setters
   const std::string& graph_label() const noexcept { return taskname_; }
   const std::string& taskname() const noexcept { return taskname_; }
   void set_taskname(string_view l) noexcept { taskname_ = l; }
   void set_timing_callback(
       std::function<void(const TaskNode* task, real seconds)> f) noexcept
   {
      timing_callback_ = std::move(f);
   }

   // DAG stuff
   std::vector<TaskNode*>& parents() noexcept { return parents_; }
   std::vector<TaskNode*>& children() noexcept { return children_; }
   const std::vector<TaskNode*>& parents() const noexcept { return parents_; }
   const std::vector<TaskNode*>& children() const noexcept { return children_; }

   // Subscribe/unsubscribe to a parent. i.e., make dependent on...
   void subscribe(TaskNode* parent) noexcept;
   void unsubscribe(TaskNode* parent) noexcept;

   // Finding parent nodes
   TaskNode* find_parent(string_view l) noexcept;
   TaskNode* find_parent(std::function<bool(const TaskNode*)> f) const noexcept;
   TaskNode* match_parent(string_view l) noexcept; // parent's name starts-with

   // Actions
   void cancel() noexcept; // cancel computation (propagates to descendents)
   void calc_result(callback_type callback) noexcept; // may block/calculate
   shared_ptr<const void> try_get_result() noexcept;  // nullptr if stale

   // Manually set the result... USE WITH CAUTION
   // void manually_set_result(shared_ptr<const void>) noexcept;

   void shallow_copy_params_result(const TaskNode& o) noexcept
   {
      if(&o != this) {
         Expects(taskname_ == o.taskname_);
         {
            lock_guard<decltype(padlock_)> lock(padlock_);
            increment_run_counter(); // WARNING, does not invalidate descendents
            result_ = o.result_;
         }
         shallow_copy_params_(&o);
      }
   }

   virtual bool params_equal(const TaskNode& o) const noexcept = 0;
};

// ---------------------------------------------------------------- PipelineTask
//
template<typename P, typename T> class PipelineTask : public TaskNode
{
 private:
   std::mutex padlock_;
   P params_;

 public:
   CUSTOM_NEW_DELETE(PipelineTask)

   PipelineTask(string taskname = ""s)
       : TaskNode(taskname)
   {}
   virtual ~PipelineTask() = default;

   // Setters
   bool set_params(const P& params) noexcept // May cancel a running computation
   {
      lock_guard<decltype(padlock_)> lock(padlock_);
      const bool has_changed = !(params_ == params);
      params_ = params; // parameters don't have to be '==', e.g., 'feedback'
      if(has_changed) cancel();
      return has_changed;
   }

   const P& params() const noexcept { return params_; }

   // Just static casts the result. May be executed on a
   // different thread to where it started.
   void result(std::function<void(shared_ptr<const T>)> f = {}) noexcept
   {
      calc_result([f](shared_ptr<const void> ret) {
         if(f) f(std::static_pointer_cast<const T>(ret));
      });
   }

   shared_ptr<const T> try_get_casted_result() noexcept
   {
      return std::static_pointer_cast<const T>(try_get_result());
   }

   shared_ptr<const T> get_result_synchronized() noexcept
   {
      std::promise<shared_ptr<const T>> promise;
      auto future = promise.get_future();
      result([&promise](auto ret) { promise.set_value(ret); });
      return future.get();
   }

   shared_ptr<const void> calculate_(const RunData& run_data) noexcept override
   {
      if(run_data.is_cancelled()) return nullptr;

      P params;
      {
         lock_guard<decltype(padlock_)> lock(padlock_);
         params = params_;
      }

      if(run_data.is_cancelled()) return nullptr;

      auto ret = execute(run_data, params, run_data.is_cancelled);
      if(!run_data.is_cancelled() and !ret)
         WARN(format("{}.execute(...) return nullptr, even though it "
                     "wasn't cancelled",
                     taskname()));

      return ret;
   }

   bool params_equal(const TaskNode& o) const noexcept override
   {
      if(&o == this) return true;
      Expects(taskname() == o.taskname());
      return params_ == static_cast<const PipelineTask<P, T>*>(&o)->params();
   }

 protected:
   //
   void shallow_copy_params_(const TaskNode* o) noexcept override
   {
      if(o != this) {
         lock_guard<decltype(padlock_)> lock(padlock_);
         params_ = static_cast<const PipelineTask<P, T>*>(o)->params();
      }
   }

   // The passed function returns TRUE if the operation should be abandoned.
   // (i.e., cancelled.) An abandoned execution returns a nullptr.
   // MUST BE REENTRANT and NON-BLOCKING
   virtual shared_ptr<const T> execute(const RunData& data,
                                       const P& params,
                                       std::function<bool()> is_cancelled) const
       noexcept
       = 0;
};

} // namespace perceive
