
#include <algorithm>
#include <future>

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"
#include "perceive/pipeline/pipeline-task.hpp"

static const bool feedback = false;

namespace perceive::pipeline
{
namespace value
{
   static std::atomic<int> count = 0;

   struct Params
   {
      int value;
      bool operator==(const Params& o) const noexcept
      {
         return o.value == value;
      }
      bool operator!=(const Params& o) const noexcept { return !(o == *this); }
   };

   struct Result
   {
      int value;
      Result()
      {
         if(feedback) INFO("Creating value::Result");
         ++count;
      }
      ~Result()
      {
         if(feedback) INFO("Destroying value::Result");
         --count;
      }
   };

   class Task : public PipelineTask<Params, Result>
   {
    protected:
      shared_ptr<const Result>
      execute(const RunData& data,
              const Params& params,
              std::function<bool()> is_cancelled) const noexcept override
      {
         auto ret   = make_shared<Result>();
         ret->value = params.value;
         return ret;
      }
   };
} // namespace value

namespace multiplier
{
   static std::atomic<int> count = 0;

   struct Params
   {
      int left{0};
      int right{0};
   };

   struct Result
   {
      shared_ptr<const value::Result> left;
      shared_ptr<const value::Result> right;
      int result;
      Result()
      {
         if(feedback) INFO("Creating multiplier::Result");
         ++count;
      }
      ~Result()
      {
         if(feedback) INFO("Destroying multiplier::Result");
         --count;
      }
   };

   class Task : public PipelineTask<Params, Result>
   {
    protected:
      shared_ptr<const Result>
      execute(const RunData& data,
              const Params& params,
              std::function<bool()> is_cancelled) const noexcept override
      {
         auto ret{make_shared<Result>()};
         ret->left   = data.match_result<value::Result>("left");
         ret->right  = data.match_result<value::Result>("right");
         ret->result = ret->left->value * ret->right->value;
         return ret;
      }
   };
} // namespace multiplier

CATCH_TEST_CASE("PipelineMemoryLeak", "[pipeline_memory_leak]")
{
   CATCH_SECTION("pipeline-memory-leak-1")
   {
      // if(feedback) INFO("Begin");

      // struct Results
      // {
      //    value::Task left;
      //    value::Task right;
      //    multiplier::Task multiplier;
      // } pipeline;

      // pipeline.left.set_taskname("left");
      // pipeline.right.set_taskname("right");
      // pipeline.multiplier.subscribe(&pipeline.left);
      // pipeline.multiplier.subscribe(&pipeline.right);

      // for(int i{0}; i < 5; ++i) {
      //    value::Params lp;
      //    lp.value = i;
      //    pipeline.left.set_params(lp);

      //    value::Params rp;
      //    rp.value = i + 1;
      //    pipeline.right.set_params(rp);

      //    std::promise<shared_ptr<const multiplier::Result>>
      //    result_promise; auto result_future = result_promise.get_future();

      //    pipeline.multiplier.result(
      //        [&](auto task_result) { result_promise.set_value(task_result);
      //        });

      //    auto result = result_future.get();
      //    if(feedback)
      //       INFO(format("Got result: {} * {} = {}",
      //                   lp.value,
      //                   rp.value,
      //                   result->result));
      // }

      // if(feedback) INFO(format("live value results: {}", value::count));
      // if(feedback)
      //    INFO(format("live multiplier results: {}", multiplier::count));

      // CATCH_REQUIRE(value::count == 2);
      // CATCH_REQUIRE(multiplier::count == 1);

      // if(feedback) INFO("Done!");
   }
}
} // namespace perceive::pipeline
