
#pragma once

namespace perceive
{
// ------------------------------------------------------- center-and-scale-data
// minus the average, and divide by stddev
// Returns the centre and stddev
template<typename InputIt>
auto center_and_scale_data(InputIt start, InputIt finish)
    -> std::pair<typename std::iterator_traits<InputIt>::value_type, double>;

// Rotates data so that the principle axis straight up (Z)
// and the 2nd major axis aligns with the X axis/
// Returns the original Principle, and 2nd axis.
std::pair<Vector3f, Vector3f> rectify_3d_vec(vector<Vector3f>& data);

//
//
//
//
//

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// --------------------------------------------------------------- implementions
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
//
//
//
//
//

// ------------------------------------------------------- center-and-scale-data
//
template<typename InputIt>
auto center_and_scale_data(InputIt start, InputIt finish)
    -> std::pair<typename std::iterator_traits<InputIt>::value_type, double>
{
   using T = typename std::iterator_traits<InputIt>::value_type;

   const auto plus              = std::plus<T>{};
   const auto minus             = std::minus<T>{};
   const auto additive_identity = T{};

   const auto N = std::distance(start, finish);
   Expects(N > 2);

   auto calc_average = [&]() {
      size_t counter = 0;
      auto sum       = additive_identity;
      for(auto ii = start; ii != finish; ++ii) {
         if(is_finite(*ii)) {
            sum = plus(sum, *ii);
            ++counter;
         }
      }
      return sum / double(counter);
   };
   const auto average = calc_average();

   auto calc_stddev = [&]() -> double {
      size_t counter = 0;
      auto sum_sq    = 0.0;
      for(auto ii = start; ii != finish; ++ii) {
         if(is_finite(*ii)) {
            sum_sq += square(distance(*ii, average));
            ++counter;
         }
      }
      return std::sqrt(sum_sq / double(counter));
   };
   const auto stddev = calc_stddev();

   // center and scale
   for(auto ii = start; ii != finish; ++ii) *ii = minus(*ii, average) / stddev;

   return {average, stddev};
}

// -------------------------------------------------------

} // namespace perceive
