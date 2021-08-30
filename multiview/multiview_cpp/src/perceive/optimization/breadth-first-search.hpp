
#pragma once

#include <functional>
#include <vector>

namespace perceive
{
std::vector<unsigned> breadth_first_search(
    const unsigned source,
    const unsigned sink,
    const std::size_t n_vertices,
    std::function<void(const unsigned u, std::function<void(const unsigned)>)>
        for_each_neighbour) noexcept;

std::vector<unsigned> breadth_first_search(
    const unsigned source,
    std::function<bool(const unsigned u)> is_sink,
    const std::size_t n_vertices,
    std::function<void(const unsigned u, std::function<void(const unsigned)>)>
        for_each_neighbour) noexcept;
} // namespace perceive
