
#pragma once

#include <cstddef>
#include <type_traits>

namespace perceive
{
template<typename T, typename C>
bool is_valid_index(T idx, const C& container) noexcept
{
   static_assert(std::is_integral<T>::value, "index must be an integral type");
   return size_t(idx) < container.size();
}

struct index
{
   using ssize_t
       = std::common_type_t<std::ptrdiff_t, std::make_signed_t<std::size_t>>;

   ssize_t val = 0;

   // Construction
   index()             = default;
   index(const index&) = default;
   index(index&&)      = default;
   ~index()            = default;

   index(int x)
       : val{x}
   {}

   template<typename T, std::enable_if_t<std::is_integral_v<T>, int> = 0>
   index& operator=(T x) noexcept
   {
      val = static_cast<ssize_t>(x);
      return *this;
   }

   index& operator=(const index&) = default;
   index& operator=(index&&) = default;

   operator ssize_t() noexcept { return val; }

   friend std::ostream& operator<<(std::ostream& os, index o)
   {
      os << o.val;
      return os;
   }

   // Squashes warning for comparing signed and unsigned
   template<typename T, std::enable_if_t<std::is_integral_v<T>, int> = 0>
   bool operator==(T x) const noexcept
   {
      return val == static_cast<ssize_t>(x);
   }
   template<typename T, std::enable_if_t<std::is_integral_v<T>, int> = 0>
   bool operator!=(T x) const noexcept
   {
      return val != static_cast<ssize_t>(x);
   }
   template<typename T, std::enable_if_t<std::is_integral_v<T>, int> = 0>
   bool operator<(T x) const noexcept
   {
      return val < static_cast<ssize_t>(x);
   }
   template<typename T, std::enable_if_t<std::is_integral_v<T>, int> = 0>
   bool operator<=(T x) const noexcept
   {
      return val <= static_cast<ssize_t>(x);
   }
   template<typename T, std::enable_if_t<std::is_integral_v<T>, int> = 0>
   bool operator>(T x) const noexcept
   {
      return val > static_cast<ssize_t>(x);
   }
   template<typename T, std::enable_if_t<std::is_integral_v<T>, int> = 0>
   bool operator>=(T x) const noexcept
   {
      return val >= static_cast<ssize_t>(x);
   }

   index operator++(int) noexcept
   {
      index o = *this;
      ++o;
      return o;
   }

   index& operator++() noexcept
   {
      ++val;
      return *this;
   }

   index operator--(int) noexcept
   {
      index o = *this;
      --o;
      return o;
   }

   index& operator--() noexcept
   {
      --val;
      return *this;
   }

   template<typename T, std::enable_if_t<std::is_integral_v<T>, int> = 0>
   index& operator+=(T x) noexcept
   {
      val += static_cast<ssize_t>(x);
      return *this;
   }

   template<typename T, std::enable_if_t<std::is_integral_v<T>, int> = 0>
   index& operator-=(T x) noexcept
   {
      val -= static_cast<ssize_t>(x);
      return *this;
   }

   template<typename T, std::enable_if_t<std::is_integral_v<T>, int> = 0>
   index& operator*=(T x) noexcept
   {
      val *= static_cast<ssize_t>(x);
      return *this;
   }

   template<typename T, std::enable_if_t<std::is_integral_v<T>, int> = 0>
   index& operator/=(T x) noexcept
   {
      val /= static_cast<ssize_t>(x);
      return *this;
   }

   template<typename T, std::enable_if_t<std::is_integral_v<T>, int> = 0>
   index operator+(T x) const noexcept
   {
      index o;
      o += x;
      return o;
   }

   template<typename T, std::enable_if_t<std::is_integral_v<T>, int> = 0>
   index operator-(T x) const noexcept
   {
      index o;
      o -= x;
      return o;
   }

   template<typename T, std::enable_if_t<std::is_integral_v<T>, int> = 0>
   index operator/(T x) const noexcept
   {
      index o;
      o *= x;
      return o;
   }

   template<typename T, std::enable_if_t<std::is_integral_v<T>, int> = 0>
   index operator*(T x) const noexcept
   {
      index o;
      o /= x;
      return o;
   }
};

} // namespace perceive
