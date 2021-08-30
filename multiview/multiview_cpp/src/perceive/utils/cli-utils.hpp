
#pragma once

#include "perceive/foundation.hpp"

/**
 * EXAMPLE
 */

// struct Config
// {
//     bool show_help{false};
//     string filename{""};
// };

// int run_main(int argc, char * * argv)
// {
//     Config config;
//     auto has_error = false;

//     for(int i = 1; i < argc; ++i) {
//         string arg = argv[i];
//         try {

//             if(arg == "-h" || arg == "--help")
//                 { config.show_help = true;  continue; }
//             if(arg == "-f")
//                 { config.filename = cli::safe_arg_str(argc, argv, i); }

//         } catch(std::runtime_error& e) {
//             cout << format("Error on command-line: {}", e.what()) << endl;
//             has_error = true;
//         }
//     }

//     if(has_error) {
//         cout << format("aborting...") << endl;
//         return EXIT_FAILURE;
//     }

//     // ---

//     return EXIT_SUCCESS;
// }

namespace perceive::cli
{
inline string safe_arg_str(int argc, char** argv, int& i) noexcept(false)
{
   auto arg = argv[i];
   ++i;
   if(i >= argc) {
      auto msg = format("expected string after argument '{}'", arg);
      throw std::runtime_error(msg);
   }
   return string(argv[i]);
}

inline int safe_arg_int(int argc, char** argv, int& i) noexcept(false)
{
   auto arg = argv[i];
   ++i;
   auto badness = (i >= argc);
   auto ret     = 0;

   if(!badness) {
      char* end = nullptr;
      ret       = int(strtol(argv[i], &end, 10));
      if(*end != '\0') badness = true;
   }

   if(badness) {
      auto msg = format("expected integer after argument '{}'", arg);
      throw std::runtime_error(msg);
   }

   return ret;
}

inline real safe_arg_real(int argc, char** argv, int& i) noexcept(false)
{
   auto arg = argv[i];
   ++i;
   auto badness = (i >= argc);
   auto ret     = 0.0;

   if(!badness) {
      char* end = nullptr;
      ret       = strtod(argv[i], &end);
      if(*end != '\0') badness = true;
   }

   if(badness) {
      auto msg = format("expected numeric after argument '{}'", arg);
      throw std::runtime_error(msg);
   }

   return ret;
}

} // namespace perceive::cli
