
#include "human-heights.hpp"

namespace perceive::human
{
static constexpr real h_stddev = 6.0 * 0.0254;

real height_z_score(real height) noexcept
{
   if(height > max_height()) return 100.0;
   if(height < min_height()) return -100.0;
   const auto z = 0.5 * (height - k_average_height) / h_stddev;
   return z;
}

real prob_height_between(real low, real high) noexcept
{
   if(!std::isfinite(low) || !std::isfinite(high)) return 0.0;
   if(high < low) std::swap(low, high);

   const real z_high = height_z_score(high);
   const real z_low  = height_z_score(low);

   Expects(z_high >= z_low);

   const real phi_high = phi_function(z_high);
   const real phi_low  = phi_function(z_low);

   const real out = phi_high - phi_low;

   Expects(out >= 0.0);
   Expects(out <= 1.0);

   return out;
}

real height_z_score(int gender, int age, real height) noexcept
{
   return (height - k_average_height) / h_stddev;
}

real prob_height_between(int gender, int age, real low, real high) noexcept
{
   return phi_function(height_z_score(gender, age, high))
          - phi_function(height_z_score(gender, age, low));
}

} // namespace perceive::human
