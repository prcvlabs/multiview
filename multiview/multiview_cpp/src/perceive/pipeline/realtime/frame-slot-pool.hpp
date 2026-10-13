
#pragma once

#include "frame-record.hpp"
#include "paced-frame-source.hpp"

#include <chrono>

namespace perceive
{
class TaskTimings;
}

namespace perceive::pipeline
{
class FrameResults;
}

namespace perceive::pipeline::realtime
{
// --------------------------------------------------------------- FrameSlotPool
// `n_slots` copies of the per-frame pipeline (`FrameResults`), each computing
// at most one frame at a time, so that frames overlap. A slot takes its images
// from the frame it was given (`copy_sensor_images`'s images provider) rather
// than from the scene's videos.
//
// `dispatch` marks an idle slot busy on the calling thread, and schedules a
// pool job that sets the slot's frame and asks for its localization result.
// When the result is ready (on a pool thread), the slot builds the frame's
// `FrameRecord` (or a `DAG_ERROR` drop when there is no result), hands it to
// `on_done`, and only then becomes idle. A busy slot is never cancelled.
//
class FrameSlotPool
{
 public:
   // Called on a pool thread; must not block, and must not throw
   using on_done_type = std::function<void(FrameRecord&& record)>;

 private:
   struct Pimpl;
   unique_ptr<Pimpl> pimpl_;

 public:
   // Copies `master` (parameters only) `n_slots` times. Main thread.
   FrameSlotPool(const FrameResults& master,
                 unsigned n_slots,
                 on_done_type on_done);
   FrameSlotPool(const FrameSlotPool&) = delete;
   FrameSlotPool(FrameSlotPool&&)      = delete;
   ~FrameSlotPool(); // drain()
   FrameSlotPool& operator=(const FrameSlotPool&) = delete;
   FrameSlotPool& operator=(FrameSlotPool&&)      = delete;

   unsigned n_slots() const noexcept;
   unsigned n_busy() const noexcept;
   unsigned max_busy() const noexcept;
   uint64_t n_dispatched() const noexcept;

   // Per-node timings of slot `index`, without `bcam_init` and
   // `get_xy_mapping`, which run once per slot.
   const TaskTimings& slot_timings(unsigned index) const noexcept;

   // Main thread, before the first `dispatch`: computes every slot's
   // `bcam_init` and `get_xy_mapping` results, which depend on the scene only
   // and survive every change of frame. False if any of them failed.
   bool warm_up() noexcept;

   // Waits up to `timeout` for an idle slot (0: no wait;
   // `SlotPool<>::k_wait_forever`). False if no slot became idle.
   bool dispatch(shared_ptr<const SourceFrame> frame,
                 const FrameTimeline& timeline,
                 std::chrono::milliseconds timeout) noexcept;

   // Blocks until every slot is idle, i.e., every dispatched frame has been
   // handed to `on_done`.
   void drain() noexcept;
};

} // namespace perceive::pipeline::realtime
