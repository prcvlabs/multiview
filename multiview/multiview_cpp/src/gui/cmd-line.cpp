
#include "cmd-line.hpp"

#include "perceive/utils/cli-utils.hpp"
#include "perceive/utils/file-system.hpp"

namespace perceive::gui
{
// ------------------------------------------------------------------- show-help

void show_help(string argv0) { return ::perceive::pipeline::show_help(argv0); }

// ---------------------------------------------------------- parse-command-line

AppConfig parse_command_line(int argc, char** argv)
{
   AppConfig config;
   config.config = ::perceive::pipeline::parse_command_line(argc, argv);
   return config;
}

} // namespace perceive::gui
