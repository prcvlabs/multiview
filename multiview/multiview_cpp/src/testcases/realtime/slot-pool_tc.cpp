
#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/pipeline/pipeline-task.hpp"
#include "perceive/pipeline/realtime/bounded-queue.hpp"
#include "perceive/pipeline/realtime/slot-pool.hpp"
#include "perceive/utils/threads.hpp"

#include <atomic>
#include <condition_variable>
#include <thread>

namespace perceive::pipeline::realtime
{
using namespace std::chrono_literals;

namespace slot_test
{
   // Live `Result` objects, to check that only the slots' own results remain
   static std::atomic<int> n_live_results = 0;

   // Holds `Work` until opened, when a test needs slots to stay busy
   struct Gate
   {
      std::mutex padlock;
      std::condition_variable opened;
      bool is_open = true;

      void close()
      {
         std::lock_guard<std::mutex> lock(padlock);
         is_open = false;
      }

      void open()
      {
         {
            std::lock_guard<std::mutex> lock(padlock);
            is_open = true;
         }
         opened.notify_all();
      }

      void wait()
      {
         std::unique_lock<std::mutex> lock(padlock);
         opened.wait(lock, [this]() { return is_open; });
      }
   };

   static Gate gate;

   struct Params
   {
      unsigned frame_num = 0;
      bool operator==(const Params& o) const noexcept
      {
         return frame_num == o.frame_num;
      }
   };

   struct Result
   {
      int frame_no = -1;
      int value    = 0;

      Result() { ++n_live_results; }
      Result(const Result&) = delete;
      ~Result() { --n_live_results; }
      Result& operator=(const Result&) = delete;
   };

   // ------------------------------------------------------------------- Source
   // The slot's input for frame `frame_num`, from a provider, as
   // `copy_sensor_images` does
   class Source final : public PipelineTask<Params, Result>
   {
    public:
      std::function<int(int frame_num)> provider;

    protected:
      shared_ptr<const Result>
      execute(const RunData& data,
              const Params& params,
              std::function<bool()> is_cancelled) const noexcept override
      {
         auto ret      = make_shared<Result>();
         ret->frame_no = int(params.frame_num);
         ret->value    = provider(int(params.frame_num));
         return ret;
      }
   };

   // --------------------------------------------------------------------- Work
   //
   class Work final : public PipelineTask<Params, Result>
   {
    protected:
      shared_ptr<const Result>
      execute(const RunData& data,
              const Params& params,
              std::function<bool()> is_cancelled) const noexcept override
      {
         const auto src = data.match_result<Result>("source");
         if(src == nullptr) return nullptr;
         gate.wait();
         std::this_thread::sleep_for(5ms);
         auto ret      = make_shared<Result>();
         ret->frame_no = src->frame_no;
         ret->value    = 2 * src->value;
         return ret;
      }
   };

   // --------------------------------------------------------------------- Join
   //
   class Join final : public PipelineTask<Params, Result>
   {
    protected:
      shared_ptr<const Result>
      execute(const RunData& data,
              const Params& params,
              std::function<bool()> is_cancelled) const noexcept override
      {
         const auto src  = data.match_result<Result>("source");
         const auto work = data.match_result<Result>("work");
         if(src == nullptr or work == nullptr) return nullptr;
         auto ret      = make_shared<Result>();
         ret->frame_no = src->frame_no;
         ret->value    = work->value;
         return ret;
      }
   };

   // --------------------------------------------------------------------- Slot
   //
   struct Slot
   {
      Source source;
      Work work;
      Join join;

      std::mutex padlock;
      int frame_no = -1;
      int input    = 0;

      Slot()
      {
         source.set_taskname("source");
         work.set_taskname("work");
         join.set_taskname("join");
         work.subscribe(&source);
         join.subscribe(&source);
         join.subscribe(&work);
         source.provider = [this](int frame_num) {
            std::lock_guard<std::mutex> lock(padlock);
            return (frame_num == frame_no) ? input : -1;
         };
      }
   };

   struct Output
   {
      int frame_no = -1;
      int value    = 0;
   };

   // ----------------------------------------------------------------- dispatch
   // As `FrameSlotPool::dispatch`: pin the input, then set the frame and ask
   // for the result in a pool job; the callback hands the result over, then
   // releases the slot.
   static bool dispatch(SlotPool<Slot>& pool,
                        int frame_no,
                        int input,
                        std::chrono::milliseconds timeout,
                        std::function<void(Output)> on_done)
   {
      return pool.dispatch(timeout, [&](Slot& slot, unsigned index) {
         {
            std::lock_guard<std::mutex> lock(slot.padlock);
            slot.frame_no = frame_no;
            slot.input    = input;
         }
         SlotPool<Slot>* pool_ptr = &pool;
         Slot* slot_ptr           = &slot;
         schedule([pool_ptr, slot_ptr, index, frame_no, on_done]() {
            Params p;
            p.frame_num = unsigned(frame_no);
            set_params_forcing(slot_ptr->source, p);
            slot_ptr->join.result(
                [pool_ptr, index, on_done](shared_ptr<const Result> ret) {
                   Output out;
                   if(ret != nullptr) {
                      out.frame_no = ret->frame_no;
                      out.value    = ret->value;
                   }
                   on_done(out);
                   pool_ptr->release(index);
                });
         });
      });
   }

   static unique_ptr<SlotPool<Slot>> make_pool(unsigned n_slots)
   {
      vector<unique_ptr<Slot>> slots;
      for(unsigned i = 0; i < n_slots; ++i)
         slots.push_back(make_unique<Slot>());
      return make_unique<SlotPool<Slot>>(std::move(slots));
   }

   static bool wait_for_live_results(int n)
   {
      for(int i = 0; i < 200 and n_live_results != n; ++i)
         std::this_thread::sleep_for(10ms);
      return n_live_results == n;
   }

} // namespace slot_test

// ------------------------------------------------------------------- TEST_CASE
//
CATCH_TEST_CASE("SlotPool", "[realtime_slots]")
{
   using namespace slot_test;
   using Pool = SlotPool<Slot>;

   CATCH_SECTION("two-slots-twenty-frames")
   {
      std::mutex padlock;
      vector<Output> outputs;
      auto on_done = [&](Output out) {
         std::lock_guard<std::mutex> lock(padlock);
         outputs.push_back(out);
      };

      const int n_before = n_live_results;
      {
         auto pool = make_pool(2);
         for(int k = 0; k < 20; ++k)
            CATCH_REQUIRE(
                dispatch(*pool, k, 10 * k, Pool::k_wait_forever, on_done));
         pool->drain();
         CATCH_REQUIRE(pool->n_busy() == 0);
         CATCH_REQUIRE(pool->max_busy() == 2);
         CATCH_REQUIRE(pool->n_dispatched() == 20);
         CATCH_REQUIRE(pool->n_released() == 20);
         bury(std::move(pool));
      }
      // The memoised result of each node of the two buried slots, no more
      CATCH_REQUIRE(wait_for_live_results(n_before + 2 * 3));

      std::sort(begin(outputs), end(outputs), [](auto& a, auto& b) {
         return a.frame_no < b.frame_no;
      });
      CATCH_REQUIRE(outputs.size() == 20);
      for(int k = 0; k < 20; ++k) {
         CATCH_REQUIRE(outputs[size_t(k)].frame_no == k);
         CATCH_REQUIRE(outputs[size_t(k)].value == 20 * k);
      }
   }

   CATCH_SECTION("busy-slots-are-not-dispatched")
   {
      std::atomic<int> n_done = 0;
      auto on_done            = [&](Output) { ++n_done; };

      auto pool = make_pool(2);
      gate.close();
      CATCH_REQUIRE(dispatch(*pool, 0, 0, 0ms, on_done));
      CATCH_REQUIRE(dispatch(*pool, 1, 1, 0ms, on_done));
      CATCH_REQUIRE(pool->n_busy() == 2);
      CATCH_REQUIRE(!dispatch(*pool, 2, 2, 0ms, on_done));
      CATCH_REQUIRE(!dispatch(*pool, 2, 2, 20ms, on_done));
      CATCH_REQUIRE(pool->is_busy(0));
      CATCH_REQUIRE(pool->is_busy(1));
      gate.open();
      pool->drain();
      CATCH_REQUIRE(n_done == 2);
      CATCH_REQUIRE(dispatch(*pool, 2, 2, 0ms, on_done));
      pool->drain();
      CATCH_REQUIRE(n_done == 3);
      bury(std::move(pool));
   }

   CATCH_SECTION("same-frame-number-twice")
   {
      std::mutex padlock;
      vector<Output> outputs;
      auto on_done = [&](Output out) {
         std::lock_guard<std::mutex> lock(padlock);
         outputs.push_back(out);
      };

      auto pool = make_pool(1);
      CATCH_REQUIRE(dispatch(*pool, 5, 1, Pool::k_wait_forever, on_done));
      CATCH_REQUIRE(dispatch(*pool, 5, 7, Pool::k_wait_forever, on_done));
      pool->drain();
      CATCH_REQUIRE(outputs.size() == 2);
      CATCH_REQUIRE(outputs[0].value == 2);
      CATCH_REQUIRE(outputs[1].value == 14); // not the memoised 2
      bury(std::move(pool));
   }

   CATCH_SECTION("admission-keeps-room-for-every-busy-slot")
   {
      // A stalled consumer: the dispatcher admits a frame only while the
      // queue has room for it and for every frame still in a slot, so no
      // completion ever finds the queue full.
      constexpr unsigned n_slots = 2;
      BoundedQueue<int> loc_q(n_slots + 8);
      std::atomic<int> n_failed = 0;
      auto on_done              = [&](Output out) {
         if(!loc_q.try_push(out.frame_no)) ++n_failed;
      };

      auto pool      = make_pool(n_slots);
      int n_admitted = 0;
      for(int k = 0; k < 100; ++k) {
         if(!loc_q.has_room(pool->n_busy() + 1)) break;
         CATCH_REQUIRE(dispatch(*pool, k, k, Pool::k_wait_forever, on_done));
         ++n_admitted;
      }
      pool->drain();
      CATCH_REQUIRE(n_admitted == int(loc_q.capacity()));
      CATCH_REQUIRE(n_failed == 0);
      CATCH_REQUIRE(loc_q.size() == loc_q.capacity());
      vector<int> frames; // in completion order
      while(auto k = loc_q.try_pop()) frames.push_back(*k);
      std::sort(begin(frames), end(frames));
      CATCH_REQUIRE(int(frames.size()) == n_admitted);
      for(int k = 0; k < n_admitted; ++k) CATCH_REQUIRE(frames[size_t(k)] == k);
      bury(std::move(pool));
   }
}

} // namespace perceive::pipeline::realtime
