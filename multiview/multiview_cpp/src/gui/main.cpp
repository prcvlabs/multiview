
#include "stdinc.hpp"

#include "app-state.hh"
#include "cmd-line.hpp"
#include "interface/main-window.hh"
#include "qt-helpers.hpp"

#include <QApplication>
#include <QIcon>
// #include <QPushButton>

int main(int argc, char** argv)
{
   using namespace perceive;

   int ret = 0;
   {
      gui::AppConfig config = gui::parse_command_line(argc, argv);
      if(config.config.has_error) return EXIT_FAILURE;

      if(config.config.show_help) {
         gui::show_help(argv[0]);
         return EXIT_SUCCESS;
      }

      // Load environment variables
      set_cuda_device_reset();
      load_environment_variables();

      if(!app_state()->initialize(config)) return EXIT_FAILURE;

      // Output the configuration info
      INFO(format("Multiview Configuration:"));
      std::cout << environment_info() << std::endl;

      QApplication app(argc, argv);
      app.setWindowIcon(QIcon(":/icons/logo_web.png"));
      app.setStyleSheet(read_asset_file(":/gui.css").c_str());

      MainWindow main_window{};
      app_state()->set_main_window(&main_window);
      app_state()->open_manifest_file(config.config.manifest_params);
      main_window.show();

      ret = app.exec();

      AppState::dispose_instance();
   }

   pp_atexit_mem_usage();
   run_exit_functions();

   return ret;
}
