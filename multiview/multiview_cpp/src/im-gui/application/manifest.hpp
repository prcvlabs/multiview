
#pragma once

namespace perceive::gui
{
struct Config;

struct Manifest
{
   bool has_error          = false;
   string err_message      = ""s;
   string mp4_fname        = ""s;
   string annotation_fname = ""s;

   string to_string() const noexcept;

   friend inline string str(const Manifest& o) noexcept
   {
      return o.to_string();
   }
};

Manifest make_manifest(const Config& config) noexcept;

} // namespace perceive::gui
