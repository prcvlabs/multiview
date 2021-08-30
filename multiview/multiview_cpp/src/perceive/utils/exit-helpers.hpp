
#pragma once

namespace perceive
{
/// These functions are called before `std::atexit` functions
void register_exit_function(std::function<void()> f);

/// It's up to the implementer to run this function at program exit. Sorry!
void run_exit_functions(); // also clears them out
} // namespace perceive
