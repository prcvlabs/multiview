
#pragma once

namespace perceive::classifier
{
inline string brief() noexcept
{
   return "trains classifiers using either an rtree or svm.";
}

int run_main(int argc, char** argv);
} // namespace perceive::classifier
