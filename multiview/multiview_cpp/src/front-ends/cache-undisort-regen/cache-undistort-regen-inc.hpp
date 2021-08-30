
#pragma once

namespace perceive::cache_undistort_regen
{
inline string brief() noexcept
{
   return "front end for regenerating cache-undistort inverses.";
}

int run_main(int argc, char** argv);
} // namespace perceive::cache_undistort_regen
