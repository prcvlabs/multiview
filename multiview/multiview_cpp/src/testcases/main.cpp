
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>

#include "stdinc.hpp"

// ---- Compile in catch
#define CATCH_CONFIG_PREFIX_ALL
#define CATCH_CONFIG_RUNNER
#include "perceive/contrib/catch.hpp"

// ------------------------------------------------------------------------ main

int main(int argc, char** argv)
{
   Catch::Session session; // There must be exactly one instance

   // Let Catch (using Clara) parse the command line
   auto return_code = session.applyCommandLine(argc, argv);
   if(return_code != EXIT_SUCCESS) return return_code; // Command line error

   perceive::load_environment_variables();

   INFO(perceive::format("Multiview Configuration:"));
   std::cout << perceive::environment_info() << std::endl;

   return session.run();
}
