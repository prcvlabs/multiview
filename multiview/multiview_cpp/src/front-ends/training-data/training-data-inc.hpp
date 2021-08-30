
#pragma once

namespace perceive::training_data
{
inline string brief() noexcept
{
   return "imports training-data JSON, and prints out a flat array.";
}

int run_main(int argc, char** argv);
} // namespace perceive::training_data
