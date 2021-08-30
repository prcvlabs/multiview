
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>

#ifdef BENCHMARK
#include <benchmark/benchmark.h>
#endif

#include "perceive/foundation.hpp"

#include "blur-faces/blur-faces-inc.hpp"
#include "cache-undisort-regen/cache-undistort-regen-inc.hpp"
#include "classifier/classifier-inc.hpp"
#include "distortion-calib/distortion-calib-inc.hpp"
#include "dump-default-params/dump-default-params-inc.hpp"
#include "phase-camera-extrinsic/phase-camera-extrinsic-inc.hpp"
#include "phase-stereo-calib/phase-stereo-calib-inc.hpp"
#include "pipeline/pipeline-inc.hpp"
#include "plane-set-calib/plane-set-calib-inc.hpp"
#include "position-scene-cameras/position-scene-cameras-inc.hpp"
#include "render-floor-map/render-floor-map-inc.hpp"
#include "run-scraps/run-scraps-inc.hpp"
#include "sprite-main/sprite-main-inc.hpp"
#include "training-data/training-data-inc.hpp"
#include "undistort/undistort-inc.hpp"

using namespace perceive;
using namespace std::string_literals;

// ------------------------------------------------------------------------ Runs

static auto make_runs()
{
   std::unordered_map<std::string, std::function<int(int, char**)>> r;
   std::unordered_map<std::string, std::function<std::string()>> b;

#ifdef BENCHMARK
#define REGISTER(z)                 \
   r[#z] = perceive::z ::benchmark; \
   b[#z] = perceive::z ::brief;
#else
#define REGISTER(z)                   \
   {                                  \
      r[#z] = perceive::z ::run_main; \
      b[#z] = perceive::z ::brief;    \
   }
#endif

#ifdef BENCHMARK
   // -- register benchmarks here --
   // REGISTER(graphcut);
#else
   // -- register -- "main" functions
   REGISTER(distortion_calib);
   REGISTER(plane_set_calib);
   REGISTER(sprite_main);
   REGISTER(run_scraps);
   REGISTER(pipeline);
   REGISTER(undistort);
   REGISTER(position_scene_cameras);
   REGISTER(cache_undistort_regen);
   REGISTER(phase_stereo_calib);
   REGISTER(phase_camera_extrinsic);
   REGISTER(render_floor_map);
   REGISTER(dump_default_params);
   REGISTER(blur_faces);
   REGISTER(training_data);
   REGISTER(classifier);
#endif

#undef REGISTER

   return make_pair(r, b);
}

// ------------------------------------------------------------------- show-help

static void show_help(const char* arg0)
{
   std::unordered_map<std::string, std::function<int(int, char**)>> runs;
   std::unordered_map<std::string, std::function<std::string()>> briefs;

   std::tie(runs, briefs) = make_runs();

   std::vector<std::string> names;
   for(const auto& ii : runs) names.push_back(ii.first);
   std::sort(names.begin(), names.end());

   auto f = [&](const string& s) {
      auto ii        = briefs.find(s);
      std::string bb = ""s;
      if(ii == cend(briefs)) {
         WARN(format("failed to find brief of '{:s}'", s));
      } else {
         bb = ii->second();
      }
      const int sz = 25 - int(s.size());
      std::string spaces(size_t(sz), ' ');

      return format("{:s}{:s}    {:s}", s, spaces, bb);
   };

   cout << format(R"V0G0N(

   Usage: {:s} [-h] [runs...]

      Run can be one of:

      {:s}

)V0G0N",
                  basename(arg0),
                  implode(names.begin(), names.end(), "\n      ", f));
}

// ------------------------------------------------------------------------ main

int main(int argc, char** argv)
{
   int ret = 0;
   {
      bool do_help = false;
      std::vector<std::string> to_run;

      if(argc < 2) {
         cout << "Type -h for help" << endl;
         return EXIT_FAILURE;
      }

      const std::string arg = argv[1];
      if(arg == "--help"s || arg == "-h") {
         show_help(argv[0]);
         return EXIT_SUCCESS;
      }

      // The benchmarks/test-cases
      auto [runs, briefs] = make_runs();

      if(runs.find(arg) == runs.end()) {
         WARN(perceive::format("Failed to find run '{:s}'", arg));
         return EXIT_FAILURE;
      }

      perceive::set_cuda_device_reset();

      // Init environment variables
      perceive::load_environment_variables();

      // Now "shift" argv[0] to argv[1]
      ret = runs.find(arg)->second(argc - 1, &argv[1]);
   }

   pp_atexit_mem_usage();

   return ret;
}
