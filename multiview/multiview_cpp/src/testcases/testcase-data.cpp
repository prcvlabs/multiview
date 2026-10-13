
#include "testcase-data.hpp"

namespace perceive::test
{
// ----------------------------------------------------------- testcase data dir
//
std::string testcase_data_dir() noexcept
{
   const char* data_dir = getenv("MULTIVIEW_TESTCASE_DATA");
   if(data_dir != nullptr) return string(data_dir);
   const char* home_dir = getenv("HOME");
   if(home_dir != nullptr) return format("{}/multiview-data", home_dir);
   return ""s;
}
} // namespace perceive::test
