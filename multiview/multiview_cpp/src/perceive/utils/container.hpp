
#pragma once

#include <algorithm>
#include <numeric>

#include "cpp-lib-compatibility.hpp"

namespace perceive
{
// ------------------------------------------------------------------ overloaded
/// std::visit(overloaded {
///       [](auto arg) { std::cout << arg << ' '; },
///       [](double arg) { std::cout << std::fixed << arg << ' '; },
///       [](const std::string& arg) { std::cout << std::quoted(arg) << ' '; },
///    },
///    v);
// template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

// ------------------------------------------------------------------- pair-hash
//
struct pair_hash
{
   template<typename T1, typename T2>
   std::size_t operator()(const std::pair<T1, T2>& ij) const
   {
      return std::hash<T1>()(ij.first) ^ std::hash<T2>()(ij.second);
   }
};

// ----------------------------------------------------------- remove duplicates
//
template<typename Container> void remove_duplicates(Container& c) noexcept
{
   std::sort(c.begin(), c.end());
   auto new_end = std::unique(c.begin(), c.end());
   c.resize(size_t(std::distance(c.begin(), new_end)));
}

template<typename Container>
Container remove_duplicates_copy(const Container& in) noexcept
{
   auto c = in;
   std::sort(c.begin(), c.end());
   auto new_end = std::unique(c.begin(), c.end());
   c.resize(size_t(std::distance(c.begin(), new_end)));
   return c;
}

// --------------------------------------------------------------------- flatten
//
// deque<list<int>> foo =  {{1, 2, 3}, {2, 3}, {}};
// vector<int> bar = flatten(foo); // bar == { 1, 2, 3, 2, 3 }
//
template<typename Container> auto flatten(const Container& c) noexcept
{
   const auto N
       = std::accumulate(cbegin(c), cend(c), 0, [&](size_t n, const auto& x) {
            return n + size_t(std::distance(cbegin(x), cend(x)));
         });
   using FlatType = typename std::decay<typename Container::value_type>::type;
   using T        = typename std::decay<typename FlatType::value_type>::type;
   std::vector<T> o;
   o.reserve(size_t(N));
   for(const auto& x : c) o.insert(end(o), cbegin(x), cend(x));
   return o;
}

// -------------------------------------------------------------- fold container
//
template<typename C, typename T, typename BinaryOperation>
T fold_container(const C& container, T zero, BinaryOperation f)
{
   return std::accumulate(cbegin(container), cend(container), zero, f);
}

// ----------------------------------------------------- make-generator-function
//
// vector<int> v = { 0, 1, 2, 3, 4 };
// auto next = make_generator_function(cbegin(v), cend(v));
// for(auto ptr = next(); ptr != nullptr; ptr = next()) {
//    std::cout << *ptr << std::endl;
// }
//
template<typename InputIterator>
std::function<typename InputIterator::pointer()> inline make_generator_function(
    InputIterator begin,
    InputIterator end)
{
   using pointer = typename InputIterator::pointer;
   return [begin, end]() mutable {
      if(begin == end) return pointer{nullptr};
      return std::addressof(*begin++);
   };
}

// ----------------------------------------------------------------- for_each2/t
// Declarative iteration over two containers
template<class InputIt1, class InputIt2, class BinaryOperation>
inline void
for_each2(InputIt1 first1, InputIt1 last1, InputIt2 first2, BinaryOperation op)
{
   while(first1 != last1) op(*first1++, *first2++);
}

template<class A, class B, class BinaryOperation>
inline void for_each_t(const A& a, const B& b, BinaryOperation op)
{
   for_each2(cbegin(a), cend(a), cbegin(b), op);
}

// ---------------------------------------------------------------- find-last-if
// Find the last element in range where f(*ii) is true
template<typename InputIt, typename F>
InputIt find_last_if(InputIt begin, InputIt end, F f)
{
   InputIt ret = end;
   for(auto ii = begin; ii != end; ++ii)
      if(f(*ii)) ret = ii;
   return ret;
}

// ------------------------------------------------------------ smallest element
//
namespace detail
{
   template<typename InputItr, typename Comp, typename F>
   InputItr smallest_elem_(InputItr first, InputItr last, Comp comp, F dist)
   {
      if(first == last) return last; // empty range
      auto ii   = first;
      auto best = ii++;
      if(ii == last) return best; // range of size 1
      auto best_distance = dist(*best);

      while(ii != last) {
         const auto distance = dist(*ii);
         if(comp(distance, best_distance)) {
            best_distance = distance;
            best          = ii;
         }
         ++ii;
      }
      return best;
   }

   // template
   // struct identity
   // {
   //    template<typename U>
   //    constexpr auto operator()(U&& v) const noexcept
   //       -> decltype(std::forward<U>(v))
   //    {
   //       return std::forward<U>(v);
   //    }
   // };
} // namespace detail

template<typename InputItr,
         typename Comp = std::less<>,
         typename F    = ranges::identity>
InputItr
smallest_elem(InputItr first, InputItr last, Comp comp = Comp(), F dist = F())
{
   return detail::smallest_elem_(first, last, comp, dist);
}

namespace rng
{
   template<typename Container,
            typename Comp = std::less<>,
            typename F    = ranges::identity>
   auto smallest_elem(const Container& c, Comp comp = Comp(), F dist = F()) ->
       typename Container::const_iterator
   {
      return detail::smallest_elem_(cbegin(c), cend(c), comp, dist);
   }
} // namespace rng

// ---------------------------------------------------------------- memory-usage

template<typename T, typename F>
size_t vector_memory_usage(const T& vec, F f) noexcept
{
   return sizeof(typename T::value_type) * (vec.capacity() - vec.size())
          + std::accumulate(
              cbegin(vec),
              cend(vec),
              size_t(0),
              [&f](size_t sz, const auto& element) { return sz = f(element); });
}

} // namespace perceive
