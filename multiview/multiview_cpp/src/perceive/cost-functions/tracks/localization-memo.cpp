
#include "localization-memo.hpp"

#define This LocalizationMemo

namespace perceive::detail
{
void This::init(const int start_t,
                const int max_n_frames,
                std::function<const LocalizationData*(int t)> get_loc) noexcept
{
   start_t_ = start_t;
   memo_.clear();
   memo_.reserve(size_t(max_n_frames));
   for(auto i = 0; i < max_n_frames; ++i) {
      const auto t   = start_t + i;
      const auto ptr = get_loc(t);
      if(ptr)
         memo_.push_back(ptr);
      else
         break;
   }
}

// Return nullptr if 't' not available
const LocalizationData* This::get(const int t) noexcept
{
   const int ind = t - start_t_;
   if(unsigned(ind) >= memo_.size()) {
      FATAL(
          format("fn = {} - {} = {}, sz = {}", t, start_t_, ind, memo_.size()));
   }
   return memo_[size_t(ind)];
}

} // namespace perceive::detail
