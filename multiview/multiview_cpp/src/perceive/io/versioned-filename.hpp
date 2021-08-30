
#pragma once

#include "perceive/utils/timestamp.hpp"

namespace perceive
{
// The file is made up of the following parts:
//
// ... /directory/stem_date_md5.extension

class VersionedFilename
{
 private:
   string directory_; // could be relative or absolute
   string stem_part_; //
   string date_part_; // A timestamp, or empty
   string md5_part_;  // A hex-md5 code, or empty
   string extension_; // with the '.', so like ".png"

 public:
   VersionedFilename(const string_view fname) noexcept { init(fname); }
   VersionedFilename(const VersionedFilename&) = default;
   VersionedFilename(VersionedFilename&&)      = default;
   ~VersionedFilename()                        = default;
   VersionedFilename& operator=(const VersionedFilename&) = default;
   VersionedFilename& operator=(VersionedFilename&&) = default;

   // Can be a multiview-data file, non-multiview-data file, or s3 url
   void init(const string_view fname) noexcept;

   void set_directory(const string_view) noexcept;
   void set_stem(const string_view) noexcept(false);
   void set_extension(const string_view) noexcept(false);

   void set_date(const Timestamp) noexcept;
   void set_date(int year, int month, int day) noexcept(false);
   void clear_date() noexcept;

   // @{ Getters
   const string_view directory() const noexcept { return directory_; }
   const string_view extension() const noexcept { return extension_; }
   const string_view stem_part() const noexcept { return stem_part_; }
   const string_view md5_part() const noexcept { return md5_part_; }
   const string_view date_part() const noexcept { return date_part_; }

   string basename() const noexcept; // basename without extension
   string filename() const noexcept; // The full filename. If
   // @}

   string to_string() const noexcept;
   friend string str(const VersionedFilename&) noexcept;
};

} // namespace perceive
