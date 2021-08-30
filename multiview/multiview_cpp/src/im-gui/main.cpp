
#include "stdinc.hpp"

#include "application/annotation-app.hpp"

int main(int argc, char** argv)
{
   using namespace perceive;

   load_environment_variables();

   const int ret = gui::AnnotationApp::run_app(argc, argv);

   pp_atexit_mem_usage();
   run_exit_functions();

   return ret;
}
