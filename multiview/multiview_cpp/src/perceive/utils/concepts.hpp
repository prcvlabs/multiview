#pragma once

#include <concepts>
#include <type_traits>

// The standard concepts (std::integral, std::same_as, ...) come from
// <concepts>. This project used to polyfill them for older standard
// libraries; only the non-standard 'arithmetic' concept remains.
namespace perceive
{
template<class T> concept arithmetic = std::is_arithmetic_v<T>;
} // namespace perceive
