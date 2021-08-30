
#include "exit-helpers.hpp"

namespace perceive
{
static SpinLock padlock_;
static vector<std::function<void()>> thunks_;

void register_exit_function(std::function<void()> f)
{
   lock_guard lock(padlock_);
   thunks_.push_back(std::move(f));
}

void run_exit_functions()
{
   vector<std::function<void()>> thunks;
   {
      lock_guard lock(padlock_);
      std::swap(thunks, thunks_);
   }

   for(auto&& f : thunks) {
      try {
         f();
      } catch(std::exception& e) {
         LOG_ERR(format("uncaught exception in exit handler: {}", e.what()));
      } catch(...) {
         FATAL(format("uncaught exception, of unknown type, in exit handler."));
      }
   }
}

} // namespace perceive
