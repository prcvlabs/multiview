
#pragma once

#include <string>
#include <string_view>

namespace perceive
{
const std::string_view hostname() noexcept(false); // std::alloc

// Returns 0 if there's a system error
size_t usable_ram() noexcept;

} // namespace perceive
