
#pragma once

#include "perceive/cost-functions/localization/localization-data.hpp"

namespace perceive::detail
{
// Memo as in "Memoization"
struct LocalizationMemo
{
 private:
   int start_t_ = 0;
   vector<const LocalizationData*> memo_;

 public:
   CUSTOM_NEW_DELETE(LocalizationMemo)

   void init(const int start_t,
             const int max_frames,
             std::function<const LocalizationData*(int t)>) noexcept;

   // Return nullptr if 't' not available
   const LocalizationData* get(const int t) noexcept;

   int size() const noexcept { return int(memo_.size()); }

   const LocalizationData* front() const noexcept { return memo_.front(); }
   const LocalizationData* back() const noexcept { return memo_.back(); }

   vector<const LocalizationData*>::const_iterator cbegin() const noexcept
   {
      return memo_.cbegin();
   }
   vector<const LocalizationData*>::const_iterator cend() const noexcept
   {
      return memo_.cend();
   }
};

} // namespace perceive::detail
