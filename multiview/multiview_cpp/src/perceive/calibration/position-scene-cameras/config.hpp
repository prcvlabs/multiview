
#pragma once

namespace perceive::calibration::position_scene_cameras
{
// ---------------------------------------------------------------------- config

struct Config
{
   bool show_help        = false;
   bool has_error        = false;
   string outdir         = "/tmp"s;
   string scene_id       = ""s;
   string manifest_fname = ""s;
   string cube_key       = ""s;
   string cube_filename  = ""s;
   bool use_fast_distort = false;
};

void show_help(string argv0);

Config parse_command_line(int argc, char** argv) noexcept;

} // namespace perceive::calibration::position_scene_cameras
