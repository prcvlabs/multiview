
#pragma once

// -------------------------------------------------------------------- identity
//
#if __cplusplus <= 201703L
namespace std
{
template<class T = void> struct identity
{
   constexpr T&& operator()(T&& t) const noexcept { return forward<T>(t); }
};

template<> struct identity<void>
{
   template<typename T> constexpr T&& operator()(T&& t) const noexcept
   {
      return forward<T>(t);
   }
};
} // namespace std
#endif
