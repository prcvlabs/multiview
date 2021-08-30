
#include "stdinc.hpp"

#include <opencv2/ml.hpp>

#include "classifier-inc.hpp"

#include "perceive/calibration/training-data/cv-test-1.hpp"
#include "perceive/calibration/training-data/train-svm.hpp"
#include "perceive/calibration/training-data/training-data-point.hpp"
#include "perceive/calibration/training-data/training-data-summary.hpp"
#include "perceive/cost-functions/classifier/classifier.hpp"
#include "perceive/utils/cli-utils.hpp"
#include "perceive/utils/file-system.hpp"

namespace perceive
{
using calibration::TrainingDataPoint;
using calibration::TrainingDataSummary;

// ------------------------------------------------------------------ train-data

struct TrainData
{
   cv::Mat samples;
   cv::Mat response;
};

// ---------------------------------------------------------------- load-cv-data

std::tuple<vector<string>, cv::Mat, vector<float>>
load_cv_data(const string_view fname,
             const bool impute_missing_values,
             const bool has_header_line) noexcept(false)
{
   const string raw = file_get_contents(fname);
   auto out
       = calibration::decode_csv(raw, impute_missing_values, has_header_line);

   const cv::Mat& data = std::get<1>(out);
   if(data.cols == 0) {
      WARN(format("csv data is empty."));
   } else {
      Expects(data.type() == CV_32F);
      Expects(data.cols > 1);
      std::array<int32_t, n_pose_annotations()> counts;
      std::fill(begin(counts), end(counts), 0);
      for(auto i = 0; i < data.rows; ++i) {
         int32_t idx = int32_t(std::round(data.at<float>(i, 1)));
         if(idx >= 0 && idx < n_pose_annotations()) {
            counts[size_t(idx)] += 1;
         } else {
            WARN(format("found a pose index out of range! Expects a number in "
                        "[0..{}), but got {}.",
                        n_pose_annotations(),
                        idx));
         }
      }
      INFO(format("Loaded {} rows of data:", data.rows));
      for(auto i = 0; i < n_pose_annotations(); ++i)
         cout << format("   {:10s}   {}\n",
                        str(to_pose_annotation(i)),
                        counts.at(size_t(i)));
      cout << endl;
   }

   return out;
}

} // namespace perceive

namespace perceive::classifier
{
// ---------------------------------------------------------------------- config

struct Config
{
   bool has_error           = false;
   bool show_help           = false;
   string csv_fname         = ""s;
   bool has_header_line     = true;
   bool impute_set          = false;
   bool impute_missing      = true;
   bool reexport            = false;
   string out_fname         = ""s;
   bool allow_overwrite     = false;
   PredictType predict      = PredictType::POSE;
   ClassifierType type      = ClassifierType::NONE;
   int max_trees            = 1000;
   real validation_percent  = 0.15;
   bool show_columns        = false;
   vector<string> col_names = {};
   bool column_search       = false;

   string to_string() const noexcept;
   friend string str(const Config& o) noexcept { return o.to_string(); }
   friend std::ostream& operator<<(std::ostream& os, const Config& o) noexcept;
};

string Config::to_string() const noexcept
{
   std::stringstream ss{""};
   ss << *this;
   return ss.str();
}

std::ostream& operator<<(std::ostream& os, const Config& o) noexcept
{
   os << format(R"V0G0N(
Classifier-Config
   csv-fname:       '{}'
   has-header-line:  {}
   impute-missing:   {}
   out-fname:       '{}'
   reexport:         {}
   allow-overwrite:  {}
   predict:          {}
   classifier-type:  {}
   max-trees:        {}
   validation-set:   {:4.2f}
   show-columns:     {}
   col-names:       [{}]
   column-search:    {}
)V0G0N",
                o.csv_fname,
                str(o.has_header_line),
                str(o.impute_missing),
                o.out_fname,
                str(o.reexport),
                str(o.allow_overwrite),
                str(o.predict),
                str(o.type),
                o.max_trees,
                o.validation_percent,
                str(o.show_columns),
                rng::implode(o.col_names, ", "),
                str(o.column_search));
   return os;
}

// ------------------------------------------------------------------- show-help

static void show_help(const string_view argv0)
{
   Config defaults;
   cout << format(R"V0G0N(

   Usage: {} [OPTIONS...]

      --csv-file <filename>         CSV data.
      --allow-missing-values        Allow missing values in training data.
      --no-header-line              CSV file has no header line.

      -o <filename>                 Output filename.
      -y                            Allow overwrite of output filename.

      --reexport                    Reexport the training data in a format
                                    suitable for libsvm.

      --tpfp                        Classify true-positive/false-positive
      --pose                        Classify pose.

      --rtree                       Train a random forest.
      --svm                         Train an SVM.
      --validation <number>         Percentage [0..1] for validation set.
                                    Default is {:4.2f}.

      --max-trees <integer>         Maximum number of trees in an rtree.
                                    Default is {}.

      --show-columns                Prints columns in loaded CSV file.
     [--col <name>]+                Use the specified column name.
                                    All columns are used if none are specified.
      --col-search                  Attempt to discover the best columns.

)V0G0N",
                  basename(argv0),
                  defaults.validation_percent,
                  defaults.max_trees);
}

// ------------------------------------------------------------- parse-arguments

static Config parse_arguments(int argc, char** argv)
{
   Expects(argc > 0);
   Config config;

   if(argc == 1) config.show_help = true;

   for(int i = 1; i < argc; ++i) {
      string_view arg{argv[i]};
      try {
         if((arg == "-h") || (arg == "--help"))
            config.show_help = true;
         else if(arg == "--csv-file")
            config.csv_fname = cli::safe_arg_str(argc, argv, i);
         else if(arg == "--allow-missing-values")
            config.impute_missing = !(config.impute_set = true);
         else if(arg == "--impute-missing-values")
            config.impute_missing = config.impute_set = true;
         else if(arg == "--no-header-line")
            config.has_header_line = false;
         else if(arg == "--reexport")
            config.reexport = true;
         else if(arg == "-o")
            config.out_fname = cli::safe_arg_str(argc, argv, i);
         else if(arg == "-y")
            config.allow_overwrite = true;
         else if(arg == "--rtree")
            config.type = ClassifierType::RTREE;
         else if(arg == "--svm")
            config.type = ClassifierType::SVM;
         else if(arg == "--tpfp")
            config.predict = PredictType::TPFP;
         else if(arg == "--pose")
            config.predict = PredictType::POSE;
         else if(arg == "--max-trees")
            config.max_trees = cli::safe_arg_int(argc, argv, i);
         else if(arg == "--validation")
            config.validation_percent = cli::safe_arg_real(argc, argv, i);
         else if(arg == "--show-columns")
            config.show_columns = true;
         else if(arg == "--col")
            config.col_names.push_back(cli::safe_arg_str(argc, argv, i));
         else if(arg == "--col-search")
            config.column_search = true;
      } catch(std::exception& e) {
         LOG_ERR(format("{}", e.what()));
         config.has_error = true;
      }
   }

   if(config.show_help) return config;

   if(!config.impute_set && config.type == ClassifierType::SVM)
      config.impute_missing = false;

   if(config.csv_fname.empty()) {
      config.has_error = true;
      LOG_ERR(format("Must specify a csv file!"));
   } else if(!is_regular_file(config.csv_fname)) {
      config.has_error = true;
      LOG_ERR(format("Could not find csv file '{}'", config.csv_fname));
   }

   if(config.out_fname.empty()) {
      config.has_error = true;
      LOG_ERR(format("Must specify an output file!"));
   } else if(!config.allow_overwrite && is_regular_file(config.out_fname)) {
      config.has_error = true;
      LOG_ERR(format("Cowardly refusing to overwrite output file '{}'",
                     config.out_fname));
   }

   if(!(config.validation_percent >= 0.0 && config.validation_percent <= 1.0)) {
      config.has_error = true;
      LOG_ERR(format("Validation percent must be in range [0..1]. Got: {}",
                     config.validation_percent));
   }

   return config;
}

// --------------------------------------------------------------- get-label-col

static int get_label_col(const PredictType predict) noexcept
{
   switch(predict) {
   case PredictType::NONE: Expects(false); break;
   case PredictType::TPFP: return 2;
   case PredictType::POSE: return 1;
   }
   Expects(false);
   return -1;
}

// ---------------------------------------------------------------- convert-cols

static vector<int> convert_cols(const vector<string>& headers,
                                const vector<string>& selected) noexcept(false)
{
   vector<int> cols;
   cols.reserve(selected.size());

   std::transform(cbegin(selected),
                  cend(selected),
                  std::back_inserter(cols),
                  [&](const string& s) -> int {
                     auto ii = std::find(cbegin(headers), cend(headers), s);
                     if(ii == cend(headers))
                        throw std::runtime_error(
                            format("failed to find header '{}'", s));
                     return int(std::distance(cbegin(headers), ii));
                  });

   { // check for duplicates
      const size_t size0 = cols.size();
      remove_duplicates(cols);
      if(cols.size() != size0)
         throw std::runtime_error(format("duplicate column detected"));
   }

   return cols;
}

// ------------------------------------------------------------ train classifier

TrainClassifierResult train_one(const ClassifierType type,
                                const PredictType predict,
                                const vector<string>& headers,
                                const vector<float>& medians,
                                const cv::Mat& data,
                                const vector<string>& selected,
                                const int max_trees,
                                const real validation_set_size,
                                const bool feedback) // [0..1]
{
   Expects(predict != PredictType::NONE);
   const int label_col = get_label_col(predict);
   const auto cols     = convert_cols(headers, selected);
   const auto [samples, labels]
       = calibration::prepare_training_data(data, cols, label_col);
   const auto seed = make_random_seed();
   return train_classifier(type,
                           predict,
                           medians,
                           samples,
                           labels,
                           max_trees,
                           validation_set_size,
                           seed,
                           feedback);
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

   INFO(format("Configuration:"));
   cout << config << '\n';

   const auto [headers, data, medians] = load_cv_data(
       config.csv_fname, config.impute_missing, config.has_header_line);

   // if(config.show_columns) {
   //    INFO(format("Columns:"));
   //    cout << indent(implode(cbegin(headers), cend(headers), "\n"), 3) <<
   //    endl;
   // }

   if(config.reexport) { // Output svm-formated data
      const auto svm_fname = config.out_fname;

      auto map_annotation = [&](int32_t val) {
         if(config.predict == PredictType::TPFP) return val;
         switch(to_pose_annotation(val)) {
         case PoseAnnotation::NONE: return -1;
         case PoseAnnotation::STAND: return 0;
         case PoseAnnotation::WALK: return 0;
         case PoseAnnotation::SIT: return 1;
         case PoseAnnotation::LAY: return 2;
         case PoseAnnotation::PHONE: return 3;
         case PoseAnnotation::OTHER: return 3;
         }
         return -1;
      };

      std::stringstream ss{""};
      const int label_col = get_label_col(config.predict);
      // [0] pose-detection-label (==> NAN)
      // [1] POSE
      // [2] TPFP
      // [3-19] the data
      const int col0 = 3;
      for(int row = 0; row < data.rows; ++row) {
         int label = map_annotation(int32_t(data.at<float>(row, label_col)));
         if(label < 0) continue;
         ss << label;
         for(int col = col0; col < data.cols; ++col) {
            const auto val = (col == col0) ? fNAN : data.at<float>(row, col);
            if(std::isfinite(val)) ss << ' ' << (col - col0 + 1) << ':' << val;
         }
         ss << '\n';
      }

      file_put_contents(svm_fname, ss.str());
      cout << format(R"V0G0N(

   Svm training data exported to {}.

   To train, run,

      /path/to/svm-train-gpu -c <number> -g <number> -b 1 {}

   A parameter search can be done using

      /path/to/grid.py {}

)V0G0N",
                     svm_fname,
                     svm_fname,
                     svm_fname)
           << endl;
      return EXIT_SUCCESS;
   }

   if(config.predict == PredictType::NONE
      || config.type == ClassifierType::NONE) {
      INFO(format("Exiting."));
      return EXIT_SUCCESS;
   }

   // Which columns are we using??
   vector<string> col_names = config.col_names;
   if(col_names.size() == 0)
      col_names = vector<string>(begin(headers) + 2, end(headers));

   const auto one_result = train_one(config.type,
                                     config.predict,
                                     headers,
                                     medians,
                                     data,
                                     col_names,
                                     config.max_trees,
                                     config.validation_percent,
                                     true);

   one_result.classifier->save(config.out_fname);

   return EXIT_SUCCESS;
}

} // namespace perceive::classifier
