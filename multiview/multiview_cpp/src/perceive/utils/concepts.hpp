
#pragma once

#include <concepts>
#include <type_traits>

namespace perceive
{
template<class T> concept arithmetic = std::is_arithmetic_v<T>;
} // namespace perceive
