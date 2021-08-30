
#pragma once

#include <type_traits>

template<typename T> constexpr bool is_nothrow_move_assign()
{
   return std::is_nothrow_move_constructible<T>::value
          and std::is_nothrow_assignable<T, T&&>::value;
}
