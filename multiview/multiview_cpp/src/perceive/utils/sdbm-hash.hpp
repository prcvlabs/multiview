
#pragma once

#include <cstdint>

namespace perceive
{
inline size_t sdbm_hash(const void* data, unsigned sz)
{
   auto ptr                = reinterpret_cast<const std::uint8_t*>(data);
   size_t h                = 0;
   const std::uint8_t* end = ptr + sz;
   while(ptr != end) h = *ptr++ + (h << 6) + (h << 16) - h;
   return h;
}

} // namespace perceive
