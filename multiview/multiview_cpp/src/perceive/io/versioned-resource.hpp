
#pragma once

namespace perceive
{
struct VersionedResource
{
   string name;
   string digest; // md5 digest extracted from source name (if any)
   int version = -1;

   static VersionedResource make(const string_view) noexcept(false);

   string to_string() const noexcept(false);
   friend string str(const VersionedResource&) noexcept(false);
};

}; // namespace perceive
