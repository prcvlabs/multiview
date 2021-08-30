
#include "stdinc.hpp"

#include "perceive/pipeline/cli-args.hpp"
#include "perceive/pipeline/execute.hpp"

namespace perceive::pipeline
{
// ----------------------------------------------------------------------- brief

string brief() noexcept { return "The multiview pipeline!"; }

// -------------------------------------------------------------------- run main
//
int run_main(int argc, char** argv)
{
   CliArgs config = parse_command_line(argc, argv);
   if(config.show_help) {
      show_help(argv[0]);
      return EXIT_SUCCESS;
   }

   if(config.has_error) {
      LOG_ERR(format("aborting due to previous errors..."));
      return EXIT_FAILURE;
   }

   // Output the configuration info
   INFO(perceive::format("Multiview Configuration:"));
   std::cout << perceive::environment_info() << std::endl;

   const bool success = run_pipeline(config);
   run_exit_functions();

   return success ? EXIT_SUCCESS : EXIT_FAILURE;
}
} // namespace perceive::pipeline
