
#include "stdinc.hpp"

#include "realtime-inc.hpp"

#include "perceive/pipeline/cli-args.hpp"
#include "perceive/pipeline/realtime/realtime-pipeline.hpp"

namespace perceive::realtime
{
// ----------------------------------------------------------------------- brief

string brief() noexcept
{
   return "The pipeline on frames as they arrive, to NDJSON.";
}

// -------------------------------------------------------------------- run main
// Takes every `pipeline` flag, and the real-time flags. Human-readable output
// goes to stderr, so that stdout can carry the NDJSON stream (`--out -`).
//
int run_main(int argc, char** argv)
{
   namespace rt = perceive::pipeline::realtime;

   auto* const cout_buffer = std::cout.rdbuf(std::cerr.rdbuf());
   auto restore_cout       = [cout_buffer]() { std::cout.rdbuf(cout_buffer); };

   rt::RealtimeParams rp;
   const pipeline::CliArgs config = pipeline::parse_command_line(
       argc, argv, [&rp](int argc, char** argv, int& i) {
          return rt::parse_realtime_arg(rp, argc, argv, i);
       });

   if(config.show_help) {
      restore_cout();
      pipeline::show_help(argv[0]);
      cout << rt::realtime_args_help() << endl;
      return EXIT_SUCCESS;
   }

   const string errors = rp.validate();
   if(!errors.empty()) LOG_ERR(format("realtime: {}", errors));

   if(config.has_error or !errors.empty()) {
      LOG_ERR(format("aborting due to previous errors..."));
      restore_cout();
      return EXIT_FAILURE;
   }

   INFO(perceive::format("Multiview Configuration:"));
   std::cout << perceive::environment_info() << std::endl;

   const bool success = rt::run_realtime(config, rp);
   run_exit_functions();

   restore_cout();
   return success ? EXIT_SUCCESS : EXIT_FAILURE;
}
} // namespace perceive::realtime
