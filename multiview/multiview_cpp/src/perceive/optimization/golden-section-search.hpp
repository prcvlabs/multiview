
#pragma once

#include <functional>

namespace perceive
{
// f(x) should be smaller when it's better.
// @returns The minimal 'x'
double golden_section_search(std::function<double(double)> f,
                             double in_a,
                             double in_b,
                             double tolerance);

} // namespace perceive
