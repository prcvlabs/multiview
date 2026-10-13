
#pragma once

#include <string>

namespace perceive::test
{
// The directory that holds the fetched testcases (build/fetch-testcases.sh)
// and the cloud references: $MULTIVIEW_TESTCASE_DATA if it is set, otherwise
// $HOME/multiview-data, otherwise "".
std::string testcase_data_dir() noexcept;
} // namespace perceive::test
