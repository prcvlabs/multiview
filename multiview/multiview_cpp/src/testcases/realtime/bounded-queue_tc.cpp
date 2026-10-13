
#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/pipeline/realtime/bounded-queue.hpp"

#include <atomic>
#include <thread>

namespace perceive::pipeline::realtime
{
using namespace std::chrono_literals;

// ------------------------------------------------------------------- TEST_CASE
//
CATCH_TEST_CASE("BoundedQueue", "[realtime_queue]")
{
   CATCH_SECTION("fifo")
   {
      BoundedQueue<int> q(4);
      for(int i = 0; i < 4; ++i) CATCH_REQUIRE(q.try_push(i));
      CATCH_REQUIRE(!q.try_push(4)); // full
      CATCH_REQUIRE(q.size() == 4);
      CATCH_REQUIRE(!q.has_room(1));
      for(int i = 0; i < 4; ++i) CATCH_REQUIRE(q.pop() == i);
      CATCH_REQUIRE(q.try_pop() == std::nullopt);
      CATCH_REQUIRE(q.pop_for(1ms) == std::nullopt);
      CATCH_REQUIRE(q.n_pushed() == 4);
      CATCH_REQUIRE(q.n_dropped() == 0);
   }

   CATCH_SECTION("push-blocks-at-capacity")
   {
      BoundedQueue<int> q(2);
      CATCH_REQUIRE(q.push(0));
      CATCH_REQUIRE(q.push(1));
      std::atomic<bool> pushed = false;
      std::thread producer([&]() {
         q.push(2); // blocks until the consumer pops
         pushed = true;
      });
      std::this_thread::sleep_for(50ms);
      CATCH_REQUIRE(!pushed);
      CATCH_REQUIRE(q.pop() == 0);
      producer.join();
      CATCH_REQUIRE(pushed);
      CATCH_REQUIRE(q.pop() == 1);
      CATCH_REQUIRE(q.pop() == 2);
   }

   CATCH_SECTION("drop-oldest")
   {
      BoundedQueue<int> q(3, true);
      for(int i = 0; i < 10; ++i) CATCH_REQUIRE(q.push(i));
      CATCH_REQUIRE(q.try_push(10));
      CATCH_REQUIRE(q.size() == 3);
      CATCH_REQUIRE(q.n_dropped() == 8);
      CATCH_REQUIRE(q.n_pushed() == 11);
      CATCH_REQUIRE(q.pop() == 8);
      CATCH_REQUIRE(q.pop() == 9);
      CATCH_REQUIRE(q.pop() == 10);
   }

   CATCH_SECTION("close-wakes-everyone")
   {
      BoundedQueue<int> empty_q(1);
      BoundedQueue<int> full_q(1);
      CATCH_REQUIRE(full_q.push(0));

      std::atomic<int> n_woken  = 0;
      std::optional<int> popped = 42;
      bool pushed               = true;
      bool has_room             = true;
      std::thread a([&]() {
         popped = empty_q.pop();
         ++n_woken;
      });
      std::thread b([&]() {
         pushed = full_q.push(1);
         ++n_woken;
      });
      std::thread c([&]() {
         has_room = full_q.wait_for_room(1);
         ++n_woken;
      });
      std::this_thread::sleep_for(50ms);
      CATCH_REQUIRE(n_woken == 0);
      empty_q.close();
      full_q.close();
      a.join();
      b.join();
      c.join();
      CATCH_REQUIRE(n_woken == 3);
      CATCH_REQUIRE(popped == std::nullopt);
      CATCH_REQUIRE(!pushed);
      CATCH_REQUIRE(!has_room);

      // A closed queue refuses new elements and drains the old ones
      CATCH_REQUIRE(!full_q.try_push(2));
      CATCH_REQUIRE(full_q.pop() == 0);
      CATCH_REQUIRE(full_q.pop() == std::nullopt);
   }

   CATCH_SECTION("wait-for-room")
   {
      BoundedQueue<int> q(4);
      for(int i = 0; i < 3; ++i) CATCH_REQUIRE(q.push(i));
      CATCH_REQUIRE(q.wait_for_room(1)); // does not block
      std::atomic<bool> has_room = false;
      std::thread t([&]() { has_room = q.wait_for_room(3); });
      std::this_thread::sleep_for(20ms);
      CATCH_REQUIRE(!has_room);
      q.pop();
      q.pop();
      t.join();
      CATCH_REQUIRE(has_room);
   }

   CATCH_SECTION("many-producers-one-consumer")
   {
      constexpr int k_n_producers = 4;
      constexpr int k_n_items     = 2500;
      BoundedQueue<int> q(8);
      vector<std::thread> producers;
      for(int p = 0; p < k_n_producers; ++p)
         producers.emplace_back([&q, p]() {
            for(int i = 0; i < k_n_items; ++i) q.push(p * k_n_items + i);
         });

      vector<int> seen(k_n_producers * k_n_items, 0);
      vector<int> last(k_n_producers, -1);
      bool in_order = true;
      for(size_t n = 0; n < seen.size(); ++n) {
         const auto x = q.pop();
         CATCH_REQUIRE(x.has_value());
         seen[size_t(*x)] += 1;
         const int p = *x / k_n_items;
         if(*x <= last[size_t(p)]) in_order = false;
         last[size_t(p)] = *x;
      }
      for(auto& t : producers) t.join();

      CATCH_REQUIRE(in_order); // each producer's items arrive in order
      CATCH_REQUIRE(
          std::all_of(cbegin(seen), cend(seen), [](int n) { return n == 1; }));
      CATCH_REQUIRE(q.size() == 0);
      CATCH_REQUIRE(q.n_dropped() == 0);
   }
}

} // namespace perceive::pipeline::realtime
