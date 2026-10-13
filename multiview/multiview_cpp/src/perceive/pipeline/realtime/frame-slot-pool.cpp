
#include "frame-slot-pool.hpp"

#include "slot-pool.hpp"

#include "perceive/pipeline/detail/task-timings.hpp"
#include "perceive/pipeline/frame-results.hpp"

#include <thread>

namespace perceive::pipeline::realtime
{
using std::chrono::steady_clock;

// ------------------------------------------------------------------------ Slot
//
struct FrameSlot
{
   unique_ptr<FrameResults> results;
   unique_ptr<TaskTimings> timings;

   // The frame being computed, and its timeline
   std::mutex padlock;
   shared_ptr<const SourceFrame> frame = nullptr;
   FrameTimeline timeline;

   // ------------------------------------------------------------ pinned-images
   // The images of the frame being computed, if it is `frame_num`
   shared_ptr<const SceneDescription::FrameImages>
   pinned_images(int frame_num) noexcept
   {
      std::lock_guard<std::mutex> lock(padlock);
      if(frame == nullptr or frame->frame_no != frame_num) return nullptr;
      return shared_ptr<const SceneDescription::FrameImages>(frame,
                                                             &frame->images);
   }
};

// ----------------------------------------------------------------------- Pimpl
//
struct FrameSlotPool::Pimpl
{
   on_done_type on_done;
   unique_ptr<SlotPool<FrameSlot>> pool; // last: drained before the rest goes

   // ----------------------------------------------------------------- run-slot
   // A pool job: start computing the slot's frame.
   void run_slot(unsigned index) noexcept
   {
      auto& slot   = pool->slot(index);
      int frame_no = -1;
      {
         std::lock_guard<std::mutex> lock(slot.padlock);
         frame_no = slot.frame->frame_no;
      }

      auto& results = *slot.results;
      auto p        = results.copy_sensor_images.params();
      p.frame_num   = unsigned(frame_no);
      slot.timings->update_tick();
      set_params_forcing(results.copy_sensor_images, p);
      results.localization.result(
          [this, index](shared_ptr<const localization::Result> ret) {
             finish_slot(index, std::move(ret));
          });
   }

   // -------------------------------------------------------------- make-record
   //
   static FrameRecord make_record(const FrameTimeline& timeline,
                                  const localization::Result* ret) noexcept
   {
      if(ret == nullptr or ret->floor_hist_result == nullptr)
         return FrameRecord::make_dropped(timeline, DropReason::DAG_ERROR);

      try {
         const auto& fhist_ret = *ret->floor_hist_result;
         FrameRecord record;
         record.timeline = timeline;
         record.loc_ptr  = make_shared<LocalizationData>(ret->data);
         record.labs.resize(fhist_ret.lab_result.size());
         for(size_t i = 0; i < fhist_ret.lab_result.size(); ++i)
            if(fhist_ret.lab_result[i] != nullptr)
               record.labs[i] = fhist_ret.lab_result[i]->lab_image;
         record.point_clouds.resize(fhist_ret.disp_result.size());
         for(size_t i = 0; i < fhist_ret.disp_result.size(); ++i)
            if(fhist_ret.disp_result[i] != nullptr)
               record.point_clouds[i] = fhist_ret.disp_result[i]->point_cloud;
         return record;
      } catch(std::exception& e) {
         LOG_ERR(format("exception copying the result of frame {}: {}",
                        timeline.frame_no,
                        e.what()));
      }
      return FrameRecord::make_dropped(timeline, DropReason::DAG_ERROR);
   }

   // -------------------------------------------------------------- finish-slot
   // The slot's localization callback: on a pool thread, or on the thread of
   // `run_slot` when the result was memoised.
   void finish_slot(unsigned index,
                    shared_ptr<const localization::Result> ret) noexcept
   {
      auto& slot = pool->slot(index);
      FrameTimeline timeline;
      {
         std::lock_guard<std::mutex> lock(slot.padlock);
         timeline   = slot.timeline;
         slot.frame = nullptr; // the slot's results keep what they need
      }
      timeline.t_dag_done = steady_clock::now();

      auto record = make_record(timeline, ret.get());
      try {
         on_done(std::move(record));
      } catch(std::exception& e) {
         LOG_ERR(format("exception handing over frame {}: {}",
                        timeline.frame_no,
                        e.what()));
      }
      pool->release(index);
   }
};

// ---------------------------------------------------------------- Construction
//
FrameSlotPool::FrameSlotPool(const FrameResults& master,
                             unsigned n_slots,
                             on_done_type on_done)
    : pimpl_(make_unique<Pimpl>())
{
   Expects(n_slots > 0);
   Expects(on_done);
   auto& P   = *pimpl_;
   P.on_done = std::move(on_done);

   vector<unique_ptr<FrameSlot>> slots;
   for(unsigned i = 0; i < n_slots; ++i) {
      auto slot     = make_unique<FrameSlot>();
      slot->results = make_unique<FrameResults>(master);
      slot->timings = make_unique<TaskTimings>();
      slot->results->set_feedback(false);

      FrameSlot* slot_ptr = slot.get();
      slot->results->copy_sensor_images.set_images_provider(
          [slot_ptr](int frame_num) {
             return slot_ptr->pinned_images(frame_num);
          });

      TaskTimings* timings_ptr = slot->timings.get();
      slot->results->set_task_finished_callback(
          [timings_ptr](const TaskNode* task, real seconds) {
             if(!begins_with(task->taskname(), "bcam_init"s)
                and !begins_with(task->taskname(), "get_xy_mapping"s))
                timings_ptr->update_timing(task->taskname(), seconds);
          });

      slots.push_back(std::move(slot));
   }

   P.pool = make_unique<SlotPool<FrameSlot>>(std::move(slots));
}

// ----------------------------------------------------------------- Destruction
// Draining guarantees that every localization callback has run. Jobs that
// `calc_result` scheduled for a parent whose result already existed can still
// be queued at that point; they return within microseconds, so wait a moment
// before the nodes go.
//
FrameSlotPool::~FrameSlotPool()
{
   drain();
   std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

// --------------------------------------------------------------------- n-slots
//
unsigned FrameSlotPool::n_slots() const noexcept
{
   return pimpl_->pool->n_slots();
}

// ---------------------------------------------------------------------- n-busy
//
unsigned FrameSlotPool::n_busy() const noexcept
{
   return pimpl_->pool->n_busy();
}

// -------------------------------------------------------------------- max-busy
//
unsigned FrameSlotPool::max_busy() const noexcept
{
   return pimpl_->pool->max_busy();
}

// ---------------------------------------------------------------- n-dispatched
//
uint64_t FrameSlotPool::n_dispatched() const noexcept
{
   return pimpl_->pool->n_dispatched();
}

// ---------------------------------------------------------------- slot-timings
//
const TaskTimings& FrameSlotPool::slot_timings(unsigned index) const noexcept
{
   Expects(index < n_slots());
   return *pimpl_->pool->slot(index).timings;
}

// --------------------------------------------------------------------- warm-up
//
bool FrameSlotPool::warm_up() noexcept
{
   auto& P = *pimpl_;

   std::mutex padlock;
   std::condition_variable all_done;
   int n_waiting = 0;
   bool success  = true;

   auto on_result = [&](shared_ptr<const void> ret) {
      {
         std::lock_guard<std::mutex> lock(padlock);
         if(ret == nullptr) success = false;
         --n_waiting;
      }
      all_done.notify_all();
   };

   auto warm_node = [&](TaskNode& node) {
      {
         std::lock_guard<std::mutex> lock(padlock);
         ++n_waiting;
      }
      node.calc_result(on_result);
   };

   for(unsigned i = 0; i < n_slots(); ++i) {
      auto& results = *P.pool->slot(i).results;
      for(auto& node : results.bcam_init) warm_node(*node);
      for(auto& node : results.get_xy_mapping) warm_node(*node);
   }

   std::unique_lock<std::mutex> lock(padlock);
   all_done.wait(lock, [&]() { return n_waiting == 0; });
   return success;
}

// -------------------------------------------------------------------- dispatch
//
bool FrameSlotPool::dispatch(shared_ptr<const SourceFrame> frame,
                             const FrameTimeline& timeline,
                             std::chrono::milliseconds timeout) noexcept
{
   if(frame == nullptr or frame->is_dropped()) {
      LOG_ERR(format("cannot dispatch a frame without images"));
      return false;
   }

   Pimpl* P = pimpl_.get();
   return P->pool->dispatch(timeout, [&](FrameSlot& slot, unsigned index) {
      {
         std::lock_guard<std::mutex> lock(slot.padlock);
         slot.frame               = frame;
         slot.timeline            = timeline;
         slot.timeline.t_dispatch = steady_clock::now();
      }
      schedule([P, index]() { P->run_slot(index); });
   });
}

// ----------------------------------------------------------------------- drain
//
void FrameSlotPool::drain() noexcept { pimpl_->pool->drain(); }

} // namespace perceive::pipeline::realtime
