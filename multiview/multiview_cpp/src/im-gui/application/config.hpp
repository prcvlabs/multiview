
#pragma once

#include "manifest.hpp"

namespace perceive::gui
{
struct Config
{
   bool show_help = false; // The help-switch was passed
   bool has_error = false; // Failed to parse

   bool print_files     = false;
   bool allow_overwrite = false;

   string outdir = "/tmp"s;

   string annotation_dir = ""s;

   Manifest manifest = {};

   string gui_ini_fname = ""s;
};

Config parse_command_line(int argc, char** argv) noexcept;
void show_help(string_view argv0) noexcept;

} // namespace perceive::gui
