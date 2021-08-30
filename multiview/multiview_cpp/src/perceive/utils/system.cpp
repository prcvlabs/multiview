
#include <unistd.h>

#include <string>
#include <vector>

#include "system.hpp"

namespace perceive
{
const std::string_view hostname() noexcept(false)
{
   auto get_hostname = [&]() -> std::string {
      std::vector<char> buf(2048);
      Expects(gethostname(&buf[0], buf.size()) == 0);
      return std::string(&buf[0]);
   };
   static std::string hostname_ = get_hostname();
   return hostname_;
}

size_t usable_ram() noexcept
{
   const auto page_size       = sysconf(_SC_PAGESIZE);
   const auto available_pages = sysconf(_SC_PHYS_PAGES);
   if(page_size < 0 || available_pages < 0) {
      WARN(format("sysconf failed to get usable RAM"));
      return 0;
   }
   return size_t(page_size) * size_t(available_pages);
}

// std::string exec(const string_view cmd)
// {
//    std::array<char, 128> buffer;
//    std::stringstream ss{""};

//    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.data(), "r"),
//                                                  pclose);
//    if(!pipe) { throw std::runtime_error("popen() failed!"); }

//    while(true) {
//       const auto sz = fread(&buffer[0], 1, buffer.size(), pipe.get());
//       if(sz == 0) break;
//       ss.write(&buffer[0], sz);
//    }

//    return ss.str();
// }

} // namespace perceive
