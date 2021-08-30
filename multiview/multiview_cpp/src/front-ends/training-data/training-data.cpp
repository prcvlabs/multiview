
#include "stdinc.hpp"

#include <opencv2/ml.hpp>

#include "training-data-inc.hpp"

#include "perceive/calibration/training-data/cv-test-1.hpp"
#include "perceive/calibration/training-data/train-svm.hpp"
#include "perceive/calibration/training-data/training-data-point.hpp"
#include "perceive/calibration/training-data/training-data-summary.hpp"
#include "perceive/cost-functions/classifier/classifier.hpp"
#include "perceive/utils/cli-utils.hpp"
#include "perceive/utils/file-system.hpp"

namespace perceive::training_data
{
using calibration::TrainingDataPoint;
using calibration::TrainingDataSummary;

// ---------------------------------------------------------------------- config

struct Config
{
   string_view exec;
   bool has_error        = false;
   bool show_help        = false;
   bool export_csv       = true;
   bool print_summary    = false;
   vector<string> fnames = {};
   string csv_fname      = ""s;
};

// ------------------------------------------------------------------- show-help

void show_help(const string_view argv0)
{
   cout << format(R"V0G0N(

   Usage: {} [OPTIONS...] <filename>*

      --summary                     Print a summary of loaded training data.
      --export-csv <filename>       Export csv format.

)V0G0N",
                  basename(argv0));
}

// ------------------------------------------------------------- parse-arguments

Config parse_arguments(int argc, char** argv)
{
   Expects(argc > 0);
   Config config;

   config.exec = argv[0];
   config.fnames.reserve(size_t(argc - 1));

   for(int i = 1; i < argc; ++i) {
      string_view arg{argv[i]};
      try {
         if((arg == "-h") || (arg == "--help"))
            config.show_help = true;
         else if(arg == "--summary")
            config.print_summary = true;
         else if(arg == "--export-csv")
            config.csv_fname = cli::safe_arg_str(argc, argv, i);
         else
            config.fnames.push_back(string(arg.data(), arg.size()));
      } catch(std::exception& e) {
         LOG_ERR(format("{}", e.what()));
         config.has_error = true;
      }
   }

   if(config.show_help) return config;

   for(const auto& fname : config.fnames) {
      if(!is_regular_file(fname.data())) {
         config.has_error = true;
         LOG_ERR(format("failed to find file '{}'", fname));
      }
   }

   if(!config.csv_fname.empty()) {
      config.export_csv = true;
      if(is_regular_file(config.csv_fname)) {
         config.has_error = true;
         LOG_ERR(
             format("cowardly refusing to overwrite '{}' white exporting data",
                    config.csv_fname));
      }
   }

   return config;
}

// ------------------------------------------------------------------- read-file

struct FileData
{
   string fname;
   TrainingDataSummary summary;
   vector<TrainingDataPoint> data;
};

FileData read_file(const string_view fname)
{
   vector<TrainingDataPoint> tdps;
   tdps.reserve(10000);
   try {
      const auto raw = file_get_contents(fname);
      const auto arr = parse_json(raw);
      if(!arr.isArray()) throw std::runtime_error("expected to load an array");

      const int N = int(arr.size());
      for(int i = 0; i < N; ++i) {
         TrainingDataPoint training_point;
         read(training_point, arr[i]);
         tdps.push_back(std::move(training_point));
      }
   } catch(std::exception& e) {
      FATAL(format("exception reading file '{}': {}", fname, e.what()));
   }
   tdps.shrink_to_fit();

   auto get_label = [](const string_view fname) {
      const auto s = basename(dirname(fname));
      return s.empty() ? basename(fname, true) : s;
   };

   FileData fdata;
   fdata.fname   = fname;
   fdata.summary = init_training_data_summary(get_label(fname), tdps);
   fdata.data    = std::move(tdps);
   return fdata;
}

vector<FileData> load_training_points(const vector<string>& fnames)
{
   vector<FileData> out;
   out.reserve(fnames.size());
   std::transform(cbegin(fnames),
                  cend(fnames),
                  std::back_inserter(out),
                  [&](const auto& fname) { return read_file(fname); });
   return out;
}

// ------------------------------------------------------------------- flat-data
// @param nan_val: Need to replace NANs
vector<TrainingDataPoint> flatten_data(const vector<FileData>& fdatas)
{
   auto calc_size = [&]() {
      size_t sz = 0;
      for(const auto& fdata : fdatas) sz += fdata.data.size();
      return sz;
   };

   vector<TrainingDataPoint> out;
   out.reserve(calc_size());
   for(const auto& fdata : fdatas)
      out.insert(end(out), cbegin(fdata.data), cend(fdata.data));

   return out;
}

// -------------------------------------------------------------------- run-main

int run_main(int argc, char** argv)
{
   const auto config = parse_arguments(argc, argv);
   if(config.show_help) {
      show_help(argv[0]);
      return EXIT_SUCCESS;
   }

   if(config.has_error) {
      cout << "Aborting due to previous errors." << endl;
      return EXIT_SUCCESS;
   }

   const auto fdatas = load_training_points(config.fnames);

   if(config.print_summary) {
      cout << calibration::training_data_summary_table(
          int(fdatas.size()),
          [&fdatas](int ind) -> const TrainingDataSummary& {
             return fdatas.at(size_t(ind)).summary;
          })
           << endl;
   }

   if(config.export_csv) {
      // Flatten the data
      const auto flat_data = flatten_data(fdatas);
      const auto raw_data  = encode_csv(flat_data, false);
      try {
         file_put_contents(config.csv_fname, raw_data);
      } catch(std::exception& e) {
         FATAL(format("I/O error saving file '{}'", config.csv_fname));
      }
      INFO(format("training data saved to '{}'", config.csv_fname));
   }

   return EXIT_SUCCESS;
}

} // namespace perceive::training_data
