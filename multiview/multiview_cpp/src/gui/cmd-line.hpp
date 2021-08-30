
#pragma once

#include "stdinc.hpp"

#include "perceive/pipeline/cli-args.hpp"

namespace perceive
{
namespace gui
{
   struct AppConfig
   {
      int cpanel_width{370};
      int cpanel_form_label_min_width{100};

      ::perceive::pipeline::CliArgs config;
   };

   void show_help(string argv0);
   AppConfig parse_command_line(int argc, char** argv);
} // namespace gui
} // namespace perceive
