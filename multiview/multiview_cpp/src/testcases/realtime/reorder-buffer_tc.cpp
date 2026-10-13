
#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/pipeline/realtime/reorder-buffer.hpp"

namespace perceive::pipeline::realtime
{
using namespace std::chrono_literals;

namespace reorder_test
{
   static FrameRecord make_record(int frame_no)
   {
      FrameRecord record;
      record.timeline.frame_no = frame_no;
      return record;
   }

   // Every record that is ready at `now`
   static vector<FrameRecord> pop_all(ReorderBuffer& buffer,
                                      ReorderBuffer::time_point now)
   {
      vector<FrameRecord> out;
      while(auto record = buffer.pop_ready(now)) out.push_back(*record);
      return out;
   }

   static vector<int> frame_nos(const vector<FrameRecord>& records)
   {
      vector<int> out;
      for(const auto& record : records) out.push_back(record.frame_no());
      return out;
   }
} // namespace reorder_test

// ------------------------------------------------------------------- TEST_CASE
//
CATCH_TEST_CASE("ReorderBuffer", "[realtime_reorder]")
{
   using namespace reorder_test;
   const auto t0 = ReorderBuffer::time_point{} + 1000s;

   CATCH_SECTION("out-of-order-completions")
   {
      ReorderBuffer buffer(0);
      vector<FrameRecord> out;
      for(int k : {3, 1, 0, 2}) {
         CATCH_REQUIRE(buffer.push(make_record(k), t0));
         for(auto& record : pop_all(buffer, t0)) out.push_back(record);
      }
      CATCH_REQUIRE(frame_nos(out) == vector<int>{0, 1, 2, 3});
      for(const auto& record : out) CATCH_REQUIRE(!record.is_dropped());
      CATCH_REQUIRE(buffer.next_frame_no() == 4);
      CATCH_REQUIRE(buffer.n_records() == 0);
   }

   CATCH_SECTION("drop-ranges-fill-gaps")
   {
      ReorderBuffer buffer(4);
      CATCH_REQUIRE(buffer.push(make_record(7), t0));
      CATCH_REQUIRE(pop_all(buffer, t0).empty());
      buffer.push_drop(4, 6, DropReason::NO_FREE_SLOT, t0);
      const auto out = pop_all(buffer, t0);
      CATCH_REQUIRE(frame_nos(out) == vector<int>{4, 5, 6, 7});
      for(int i = 0; i < 3; ++i) {
         CATCH_REQUIRE(out[size_t(i)].is_dropped());
         CATCH_REQUIRE(out[size_t(i)].drop_reason == DropReason::NO_FREE_SLOT);
      }
      CATCH_REQUIRE(!out[3].is_dropped());

      // Adjacent ranges of different reasons keep their reasons
      buffer.push_drop(8, 8, DropReason::SOURCE_BEHIND, t0);
      buffer.push_drop(9, 10, DropReason::TRACKER_BEHIND, t0);
      buffer.push_drop(11, 11, DropReason::TRACKER_BEHIND, t0);
      const auto out2 = pop_all(buffer, t0);
      CATCH_REQUIRE(frame_nos(out2) == vector<int>{8, 9, 10, 11});
      CATCH_REQUIRE(out2[0].drop_reason == DropReason::SOURCE_BEHIND);
      CATCH_REQUIRE(out2[1].drop_reason == DropReason::TRACKER_BEHIND);
      CATCH_REQUIRE(out2[3].drop_reason == DropReason::TRACKER_BEHIND);

      // A drop of frames already returned changes nothing
      buffer.push_drop(0, 5, DropReason::NO_FREE_SLOT, t0);
      CATCH_REQUIRE(pop_all(buffer, t0).empty());
      CATCH_REQUIRE(buffer.next_frame_no() == 12);
   }

   CATCH_SECTION("missing-timeout")
   {
      ReorderBuffer buffer(0, 5s);
      CATCH_REQUIRE(buffer.push(make_record(1), t0));
      CATCH_REQUIRE(buffer.push(make_record(2), t0 + 1s));
      CATCH_REQUIRE(pop_all(buffer, t0 + 4999ms).empty());

      const auto out = pop_all(buffer, t0 + 5s);
      CATCH_REQUIRE(frame_nos(out) == vector<int>{0, 1, 2});
      CATCH_REQUIRE(out[0].drop_reason == DropReason::DAG_ERROR);
      CATCH_REQUIRE(buffer.n_expired() == 1);

      // The lost frame turns up after all: discarded, never returned twice
      CATCH_REQUIRE(!buffer.push(make_record(0), t0 + 6s));
      CATCH_REQUIRE(!buffer.push(make_record(2), t0 + 6s));
      CATCH_REQUIRE(buffer.n_late() == 2);
      CATCH_REQUIRE(pop_all(buffer, t0 + 6s).empty());

      // The clock starts when the missing frame holds up a later one
      CATCH_REQUIRE(buffer.push(make_record(4), t0 + 100s));
      CATCH_REQUIRE(pop_all(buffer, t0 + 104s).empty());
      CATCH_REQUIRE(frame_nos(pop_all(buffer, t0 + 105s)) == vector<int>{3, 4});
   }

   CATCH_SECTION("no-timeout-waits-forever")
   {
      ReorderBuffer buffer(0);
      CATCH_REQUIRE(buffer.push(make_record(1), t0));
      CATCH_REQUIRE(pop_all(buffer, t0 + 1000000s).empty());
      CATCH_REQUIRE(buffer.n_expired() == 0);
      CATCH_REQUIRE(buffer.push(make_record(0), t0 + 1000000s));
      CATCH_REQUIRE(frame_nos(pop_all(buffer, t0 + 1000000s))
                    == vector<int>{0, 1});
   }

   CATCH_SECTION("flush")
   {
      ReorderBuffer buffer(10);
      for(int k : {12, 10, 11}) CATCH_REQUIRE(buffer.push(make_record(k), t0));
      const auto out = buffer.flush();
      CATCH_REQUIRE(frame_nos(out) == vector<int>{10, 11, 12});
      CATCH_REQUIRE(buffer.flush().empty());

      // Holes before the last frame received come out as DAG_ERROR drops
      ReorderBuffer buffer2(0);
      CATCH_REQUIRE(buffer2.push(make_record(2), t0));
      buffer2.push_drop(4, 5, DropReason::NO_FREE_SLOT, t0);
      const auto out2 = buffer2.flush();
      CATCH_REQUIRE(frame_nos(out2) == vector<int>{0, 1, 2, 3, 4, 5});
      CATCH_REQUIRE(out2[0].drop_reason == DropReason::DAG_ERROR);
      CATCH_REQUIRE(out2[1].drop_reason == DropReason::DAG_ERROR);
      CATCH_REQUIRE(!out2[2].is_dropped());
      CATCH_REQUIRE(out2[3].drop_reason == DropReason::DAG_ERROR);
      CATCH_REQUIRE(out2[4].drop_reason == DropReason::NO_FREE_SLOT);
      CATCH_REQUIRE(out2[5].drop_reason == DropReason::NO_FREE_SLOT);
   }
}

} // namespace perceive::pipeline::realtime
