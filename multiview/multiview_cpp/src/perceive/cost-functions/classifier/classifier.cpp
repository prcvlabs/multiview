
#include "classifier.hpp"

#include <opencv2/ml.hpp>

#include "svm.h"

#include "perceive/calibration/training-data/training-data-point.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/opencv-helpers.hpp"

#define This Classifier

namespace perceive
{
static VectorXr decode_eigen_vector(const string_view sv)
{
   vector<string> Xs = explode(sv, ",");
   VectorXr X(Xs.size());
   for(size_t i = 0; i < Xs.size(); ++i)
      X(long(i)) = strtod(Xs.at(i).c_str(), nullptr);
   return X;
}

static vector<int32_t> decode_int32_vector(const string_view sv)
{
   vector<string> Xs = explode(sv, ",");
   vector<int32_t> X(Xs.size());
   for(size_t i = 0; i < Xs.size(); ++i) X.at(i) = std::atoi(Xs.at(i).c_str());
   return X;
}

static string cv_mat_32f_row_to_str(const cv::Mat& mat, const int row)
{
   std::stringstream ss{""};
   for(int col = 0; col < mat.cols; ++col) {
      if(col > 0) ss << ", ";
      ss << mat.at<float>(row, col);
   }
   return ss.str();
}

// ----------------------------------------------------------------------- enums
//
const char* str(const PredictType o) noexcept
{
   switch(o) {
#define CASE(x) \
   case PredictType::x: return #x
      CASE(NONE);
      CASE(TPFP);
      CASE(POSE);
#undef CASE
   }
   Expects(false);
   return "<unknown>";
}

const char* str(const ClassifierType o) noexcept
{
   switch(o) {
#define CASE(x) \
   case ClassifierType::x: return #x
      CASE(NONE);
      CASE(SVM);
      CASE(RTREE);
#undef CASE
   }
   Expects(false);
   return "<unknown>";
}

// ----------------------------------------------------------------------- enums
//
enum class SVM_TYPE : int { C_SVC = 0, NU_SVC, ONE_CLASS, EPSILON_SVR, NU_SVR };
enum class KERNEL_TYPE : int { LINEAR = 0, POLY, RBF, SIGMOID, PRECOMPUTED };

const char* str(SVM_TYPE) noexcept;
SVM_TYPE int_to_svm_type(int) noexcept;
SVM_TYPE str_to_svm_type(const string_view) noexcept;

const char* str(KERNEL_TYPE) noexcept;
KERNEL_TYPE int_to_kernel_type(int) noexcept;
KERNEL_TYPE str_to_kernel_type(const string_view) noexcept;

// ----------------------------------------------------------------------- enums
//
const char* str(SVM_TYPE o) noexcept
{
   switch(o) {
#define CASE(x) \
   case SVM_TYPE::x: return #x
      CASE(C_SVC);
      CASE(NU_SVC);
      CASE(ONE_CLASS);
      CASE(EPSILON_SVR);
      CASE(NU_SVR);
#undef CASE
   }
   FATAL("logic error");
   return "<unknown>";
}

SVM_TYPE int_to_svm_type(int o) noexcept
{
   switch(SVM_TYPE(o)) {
#define CASE(x) \
   case SVM_TYPE::x: return SVM_TYPE::x
      CASE(C_SVC);
      CASE(NU_SVC);
      CASE(ONE_CLASS);
      CASE(EPSILON_SVR);
      CASE(NU_SVR);
#undef CASE
   }
   FATAL("logic error");
   return SVM_TYPE::C_SVC;
}

SVM_TYPE str_to_svm_type(const string_view o) noexcept
{
#define CASE(x) \
   if(o == #x) return SVM_TYPE::x
   CASE(C_SVC);
   CASE(NU_SVC);
   CASE(ONE_CLASS);
   CASE(EPSILON_SVR);
   CASE(NU_SVR);
#undef CASE
   FATAL("logic error");
   return SVM_TYPE::C_SVC;
}

const char* str(KERNEL_TYPE o) noexcept

{
   switch(o) {
#define CASE(x) \
   case KERNEL_TYPE::x: return #x
      CASE(LINEAR);
      CASE(POLY);
      CASE(RBF);
      CASE(SIGMOID);
      CASE(PRECOMPUTED);
#undef CASE
   }
   FATAL("logic error");
   return "<unknown>";
}

KERNEL_TYPE int_to_kernel_type(int o) noexcept
{
   switch(KERNEL_TYPE(o)) {
#define CASE(x) \
   case KERNEL_TYPE::x: return KERNEL_TYPE::x
      CASE(LINEAR);
      CASE(POLY);
      CASE(RBF);
      CASE(SIGMOID);
      CASE(PRECOMPUTED);
#undef CASE
   }
   FATAL("logic error");
   return KERNEL_TYPE::LINEAR;
}

KERNEL_TYPE str_to_kernel_type(const string_view o) noexcept
{
#define CASE(x) \
   if(o == #x) return KERNEL_TYPE::x;
   CASE(LINEAR);
   CASE(POLY);
   CASE(RBF);
   CASE(SIGMOID);
   CASE(PRECOMPUTED);
#undef CASE
   FATAL("logic error");
   return KERNEL_TYPE::LINEAR;
}

// ------------------------------------------------------------------- svm-model

struct SvmModel
{
   svm_model* svm = nullptr;
   VectorXr interp_data;

   bool is_init = false;

   vector<int32_t> tr_classes = {};

   int n_tr_classes() const noexcept
   {
      return tr_classes.size() == 0 ? 0 : int(tr_classes.size());
      // (1 + tr_classes.back());
   }

   int translate_class(int val) const noexcept
   {
      Expects(val >= 0 && val < int(tr_classes.size()));
      const int out = tr_classes.at(size_t(val));
      if(false) {
         INFO(format("val = {}, out = {}, tr-classes = [{}], n_tr = {}",
                     val,
                     out,
                     rng::implode(tr_classes, ", "),
                     n_tr_classes()));
      }
      Expects(out >= 0 && out < n_tr_classes());
      return out;
   }

   ~SvmModel()
   {
      if(svm) svm_free_and_destroy_model(&svm);
   }

   string to_string() const noexcept
   {
      if(!svm) return "Classifier: <nullptr>"s;

      vector<int> svs((size_t(n_classes())));
      for(size_t i = 0; i < svs.size(); ++i)
         svs[i] = n_support_vectors_for_class(int(i));

      return format(R"V0G0N(
Classifier
   svm-type:        {}
   n-classes:       {}
   n-support-vecs:  {} 
   sv-per-class:   [{}]
   is-prob-model:   {}
)V0G0N",
                    str(svm_type()),
                    n_classes(),
                    n_sv(),
                    implode(cbegin(svs), cend(svs), ", "),
                    str(is_probability_model()));
   }

   // ------------------------------------------------------------------ getters

   SVM_TYPE svm_type() const noexcept
   {
      return int_to_svm_type((!svm) ? 0 : svm_get_svm_type(svm));
   }

   int n_classes() const noexcept { return !svm ? 0 : svm_get_nr_class(svm); }

   vector<int> labels() const noexcept
   {
      vector<int> ret;
      if(svm) {
         ret.resize(size_t(n_classes()));
         svm_get_labels(svm, &ret[0]);
      }
      return ret;
   }

   vector<int> sv_indices() const noexcept
   {
      vector<int> ret;
      if(svm) {
         ret.resize(size_t(n_sv()));
         svm_get_sv_indices(svm, &ret[0]);
      }
      return ret;
   }

   int n_sv() const noexcept { return !svm ? 0 : svm_get_nr_sv(svm); }

   int n_support_vectors_for_class(int k) const noexcept
   {
      if(!svm) return 0;
      Expects(k >= 0 && k < n_classes());
      return svm->nSV[k];
   }

   real nr_svr_probability() const noexcept
   {
      return !svm ? 0.0 : svm_get_svr_probability(svm);
   }

   bool is_probability_model() const noexcept
   {
      return !svm ? 0 : svm_check_probability_model(svm);
   }

   MatrixXr sv_coef() const noexcept
   {
      MatrixXr X;
      if(svm) {
         X.resize(n_classes() - 1, n_sv());
         for(auto r = 0; r < X.rows(); ++r)
            for(auto c = 0; c < X.cols(); ++c) X(r, c) = svm->sv_coef[r][c];
      }
      return X;
   }

   MatrixXr support_vectors() const noexcept
   {
      MatrixXr Xs;
      if(svm) {
         const int l   = n_sv();
         int max_index = 0;
         for(int i = 0; i < l; ++i) {
            const svm_node* sv = svm->SV[i];
            while(sv->index != -1) {
               Expects(sv->index > 0);
               if(max_index < sv->index) max_index = sv->index;
               ++sv;
            }
         }

         Xs.resize(max_index - 1, l);
         Xs.fill(dNAN);
         for(int i = 0; i < l; ++i) {
            const svm_node* sv = svm->SV[i];
            while(sv->index != -1) {
               Xs(sv->index - 1, l) = sv->value;
               ++sv;
            }
         }
      }
      return Xs;
   }

   // ------------------------------------------------------------------ predict

   static vector<svm_node> vecXr_to_vec_svm_node(const VectorXr& data)
   {
      vector<svm_node> nodes;
      nodes.reserve(size_t(data.rows() + 1));

      auto push_node = [&nodes](int index, real val) {
         nodes.push_back({});
         nodes.back().index = index;
         nodes.back().value = val;
      };

      for(auto row = 0; row < data.rows(); ++row)
         if(std::isfinite(data(row))) push_node(row + 1, data(row));
      push_node(-1, 0.0);
      return nodes;
   }

   real predict(const VectorXr& data,
                const bool fill_missing_values) const noexcept
   {
      static thread_local vector<real> probs;
      return predict(data, fill_missing_values, probs);
   }

   real predict(const VectorXr& data,
                const bool fill_missing_values,
                vector<real>& out_probs) const noexcept
   {
      static thread_local vector<real> probs;
      if(!svm) NAN;
      const auto nodes = vecXr_to_vec_svm_node(data);
      probs.resize(size_t(n_classes()));
      const auto pred = svm_predict_probability(svm, &nodes[0], &probs[0]);

      const auto out = real(translate_class(int(std::round(pred))));

      out_probs.resize(size_t(n_tr_classes()));
      std::fill(begin(out_probs), end(out_probs), 0);
      for(int i = 0; i < int(probs.size()); ++i)
         out_probs.at(size_t(translate_class(i))) = probs.at(size_t(i));

      if(false) {
         INFO("SVM");
         cout << to_string() << endl;
         cout << format("out =  {} ==> {}", pred, out) << endl;
         cout << format("inp = [{}]", implode(cbegin(probs), cend(probs), ", "))
              << endl;
         cout << format("prp = [{}]",
                        implode(cbegin(out_probs), cend(out_probs), ", "))
              << endl
              << endl;
         // FATAL("kBAM!");
      }

      return out;
   }

   // ---------------------------------------------------------------------- I/O

   bool read(const string_view data) noexcept
   {
      bool success = false;
      string tmpd;
      FILE* fp = nullptr;

      try {
         tmpd                    = make_temp_directory("/tmp/read-svm.XXXXXX");
         const auto fname_tar_gz = format("{}/file.tar.gz", tmpd);
         file_put_contents(fname_tar_gz, data);

         const auto cmd = format(
             "cd {} ; cat {} | gzip -dc | tar -xf -", tmpd, fname_tar_gz);
         const auto ret_code = std::system(cmd.c_str());
         if(ret_code != 0)
            throw std::runtime_error(
                format("logic error running command: {}", cmd));

         const auto fname_model = format("{}/svm.model", tmpd);
         if(!is_regular_file(fname_model))
            throw std::runtime_error(
                format("failed to find file `{}` in tar.gz archieve",
                       basename(fname_model)));

         tr_classes = decode_int32_vector(
             file_get_contents(format("{}/classes.text", tmpd)));

         string raw   = file_get_contents(fname_model);
         fp           = fmemopen(&raw[0], raw.size(), "rb");
         auto new_svm = svm_load_model(fname_model.c_str());
         if(new_svm) {
            if(svm) svm_free_and_destroy_model(&svm);
            svm     = new_svm;
            is_init = true;
         }

         success = true;
      } catch(std::exception& e) {
         success = false;
      }

      if(fp != nullptr) fclose(fp);

      try {
         if(!tmpd.empty()) remove_all(tmpd);
      } catch(std::exception& e) {
         success = false;
      }

      return success;
   }

   bool write(string& data) const noexcept
   {
      string tmpd = ""s;
      bool ret    = false;

      FATAL(format("not implemented"));

      if(!svm) { return false; }

      try {
         tmpd            = make_temp_directory("/tmp/save-classifier");
         const auto tmpf = format("{}/file.json", tmpd);
         ret             = svm_save_model(tmpf.c_str(), svm);
         data            = file_get_contents(tmpf);
      } catch(std::exception& e) {
         LOG_ERR(format("exception writing classifier: {}", e.what()));
         ret = false;
      }

      try {
         remove_all(tmpd);
      } catch(std::exception& e) {
         LOG_ERR(format(
             "exception cleaning up directory '{}': {}", tmpd, e.what()));
         ret = false;
      }

      return ret;
   }
};

// ------------------------------------------------------------------ RTreeModel

struct RTreeModel
{
   cv::Ptr<cv::ml::RTrees> model = nullptr;
   VectorXr interp_data;

   vector<int32_t> classes = {};

   int n_classes() const noexcept
   {
      return classes.size() == 0 ? 0 : (1 + classes.back());
   }

   int translate_class(int val) const noexcept
   {
      Expects(val >= 0 && val < int(classes.size()));
      const int out = classes.at(size_t(val));
      Expects(out >= 0 && out < n_classes());
      return out;
   }

   bool is_init() const noexcept { return model && !model->empty(); }

   string to_string() const noexcept
   {
      return format(R"V0G0N(
RTree Classifier
)V0G0N");
   }

   // ------------------------------------------------------------------ predict
   cv::Mat make_samples(const VectorXr& data,
                        const bool fill_missing_values) const
   {
      cv::Mat samples = cv::Mat(1, int(data.cols()), CV_32F);
      for(auto c = 0; c < samples.cols; ++c) {
         const bool use_interp = fill_missing_values && !std::isfinite(data(c))
                                 && c < interp_data.cols();
         samples.at<float>(0, c) = float(use_interp ? interp_data(c) : data(c));
      }

      return samples;
   }

   real predict(const VectorXr& data,
                const bool fill_missing_values) const noexcept
   {
      static thread_local vector<real> probs;
      return predict(data, fill_missing_values, probs);
   }

   real predict(const VectorXr& data,
                const bool fill_missing_values,
                vector<real>& out_probs) const noexcept
   {
      if(model.get() == nullptr) return dNAN;
      cv::Mat results;
      results.setTo(0.0);
      cv::Mat samples = make_samples(data, fill_missing_values);

      model->getVotes(samples, results, 0);

      Expects(results.type() == CV_32S);
      Expects(results.rows == 2);
      Expects(results.cols == int(classes.size()));
      out_probs.resize(size_t(n_classes()), 0);
      real sum       = 0.0;
      int max_count  = 0;
      int best_class = 0;
      for(auto i = 0; i < results.cols; ++i) {
         const int count                          = results.at<int32_t>(1, i);
         out_probs.at(size_t(translate_class(i))) = real(count);
         if(count > max_count) {
            max_count  = count;
            best_class = translate_class(i);
         }
         sum += real(count);
      }

      for(auto& x : out_probs) x /= sum;

      return best_class;
   }

   // ---------------------------------------------------------------------- I/O

   string write(bool compress_it_big = false) const noexcept
   {
      if(!model) return ""s;

      string tmpd = ""s;
      string out  = ""s;

      try {
         vector<real> medians((size_t(interp_data.rows())));
         for(int i = 0; i < interp_data.rows(); ++i)
            medians.at(size_t(i)) = interp_data(i);

         tmpd = make_temp_directory("/tmp/save-rtree.XXXXXX");

         const auto fname_ml      = format("{}/file.ml", tmpd);
         const auto fname_medians = format("{}/medians.text", tmpd);
         const auto fname_classes = format("{}/classes.text", tmpd);
         const auto fname_tar_gz  = format("{}/file.tar.gz", tmpd);

         model->save(fname_ml);
         file_put_contents(fname_medians,
                           implode(cbegin(medians), cend(medians), ","));
         file_put_contents(fname_classes,
                           implode(cbegin(classes), cend(classes), ","));

         // Now compress with gzip
         const auto ret_code
             = std::system(format("cd {} ; tar -c file.ml medians.text "
                                  "classes.text | gzip -c{} > {}",
                                  tmpd,
                                  (compress_it_big ? 9 : 1),
                                  fname_tar_gz)
                               .c_str());

         out = file_get_contents(fname_tar_gz);
      } catch(std::exception& e) {
         LOG_ERR(format("exception writing model: {}", e.what()));
         out = ""s;
      }

      try {
         if(!tmpd.empty()) remove_all(tmpd);
      } catch(std::exception& e) {
         LOG_ERR(format("failed to remove temp-directory: '{}'", tmpd));
      }

      return out;
   }

   bool read(const string_view data, string& err_message) noexcept
   {
      bool success = false;
      string tmpd;

      try {
         tmpd = make_temp_directory("/tmp/read-rtree.XXXXXX");
         const auto fname_tar_gz = format("{}/file.tar.gz", tmpd);
         file_put_contents(fname_tar_gz, data);

         const auto cmd = format(
             "cd {} ; cat {} | gzip -dc | tar -xf -", tmpd, fname_tar_gz);
         const auto ret_code = std::system(cmd.c_str());
         if(ret_code != 0)
            throw std::runtime_error(
                format("logic error running command: {}", cmd));

         model       = cv::ml::RTrees::load(format("{}/file.ml", tmpd));
         interp_data = decode_eigen_vector(
             file_get_contents(format("{}/medians.text", tmpd)));
         classes = decode_int32_vector(
             file_get_contents(format("{}/classes.text", tmpd)));

         success = true;
      } catch(std::exception& e) {
         err_message += format("exception reading model: {}", e.what());
         success = false;
      }

      try {
         if(!tmpd.empty()) remove_all(tmpd);
      } catch(std::exception& e) {
         err_message += format("failed to remove temp-directory: '{}'", tmpd);
         success = false;
      }

      return success;
   }
};

// ----------------------------------------------------------------------- Pimpl

struct This::Pimpl
{
   Pimpl()             = default;
   Pimpl(const Pimpl&) = delete;
   Pimpl(Pimpl&&)      = delete;
   ~Pimpl()            = default;
   Pimpl& operator=(const Pimpl&) = delete;
   Pimpl& operator=(Pimpl&&) = delete;

   SvmModel svm;
   RTreeModel rtree;
   bool is_svm = false;

   string to_string() const noexcept
   {
      return is_svm ? svm.to_string() : rtree.to_string();
   }

   bool is_init() const noexcept
   {
      return is_svm ? svm.is_init : rtree.is_init();
   }

   real predict(const VectorXr& data,
                const bool fill_missing_values) const noexcept
   {
      return is_svm ? svm.predict(data, fill_missing_values)
                    : rtree.predict(data, fill_missing_values);
   }

   real predict(const VectorXr& data,
                const bool fill_missing_values,
                vector<real>& out_probs) const noexcept
   {
      return is_svm ? svm.predict(data, fill_missing_values, out_probs)
                    : rtree.predict(data, fill_missing_values, out_probs);
   }

   bool read(const string_view raw) noexcept
   {
      string err_message = ""s;
      { // try to read as an rtree
         if(rtree.read(raw, err_message)) {
            is_svm = false;
            return true;
         }
      }

      is_svm         = true;
      const bool ret = svm.read(raw);

      if(!ret) {
         LOG_ERR(format("also failed to read model as rtree: {}", err_message));
      }

      return ret;
   }

   string write() const noexcept
   {
      if(is_svm) {
         string data;
         svm.write(data);
         return data;
      }
      return rtree.write();
   }
};

// ---------------------------------------------------------------- Construction

This::This()
    : pimpl_(new Pimpl)
{}

This::~This() = default;

bool This::is_init() const noexcept { return pimpl_->is_init(); }

// ------------------------------------------------------------------- to-string

string str(const Classifier& o) noexcept { return o.to_string(); }

string This::to_string() const noexcept { return pimpl_->to_string(); }

// ------------------------------------------------------------------- load/save

bool This::load(const string_view fname) noexcept
{
   try {
      const auto raw = file_get_contents(fname);
      return read(raw);
   } catch(std::exception& e) {
      WARN(format("Classifier::load(\"{}\") failed: {}", fname, e.what()));
   }
   return false;
}

bool This::save(const string_view fname) const noexcept
{
   try {
      file_put_contents(fname, this->write());
      return true;
   } catch(std::exception& e) {
      WARN(format("Classifier::save(\"{}\") failed: {}", fname, e.what()));
   }
   return false;
}

bool This::read(const string_view raw) noexcept { return pimpl_->read(raw); }

string This::write() const noexcept { return pimpl_->write(); }

// --------------------------------------------------------------------- predict

real This::predict(const VectorXr& data,
                   const bool fill_missing_values) const noexcept
{
   return pimpl_->predict(data, fill_missing_values);
}

real This::predict(const VectorXr& data,
                   const bool fill_missing_values,
                   vector<real>& out_probs) const noexcept
{
   return pimpl_->predict(data, fill_missing_values, out_probs);
}

// ------------------------------------------------------------ score-classifier

real score_classifier(const Classifier* ptr,
                      const cv::Mat& samples,
                      const cv::Mat& labels) noexcept
{
   Expects(ptr != nullptr);
   Expects(samples.rows == labels.rows);
   Expects(labels.cols == 1);
   Expects(samples.type() == CV_32F);
   Expects(labels.type() == CV_32S);
   Expects(samples.rows > 0);

   int success_counts = 0;
   for(int row = 0; row < samples.rows; ++row) {
      const int32_t val
          = int32_t(ptr->predict(row_to_vecXr(samples, row), true));
      const int32_t gt = labels.at<int32_t>(row);
      if(gt == val) success_counts += 1;
   }

   return real(success_counts) / real(samples.rows);
}

// ----------------------------------------------------------------- train-rtree

unique_ptr<Classifier> train_rtree(const cv::Mat& samples,
                                   const cv::Mat& labels,
                                   const vector<float>& medians,
                                   const int max_trees) noexcept
{
   VectorXr medians_Xr(medians.size());
   for(size_t i = 0; i < medians.size(); ++i)
      medians_Xr(long(i)) = real(medians.at(i));

   INFO(format("training the classifier..."));
   auto rtree_ptr = cv::ml::RTrees::create();
   rtree_ptr->setMaxDepth(20);
   rtree_ptr->setMinSampleCount(10);
   rtree_ptr->setMaxCategories(15);
   rtree_ptr->setCalculateVarImportance(true);
   rtree_ptr->setActiveVarCount(4);
   rtree_ptr->setTermCriteria(cv::TermCriteria(
       cv::TermCriteria::COUNT + cv::TermCriteria::EPS, max_trees, 1e-9));

   rtree_ptr->train(samples, cv::ml::ROW_SAMPLE, labels);

   INFO(format("Number of trees: {}", rtree_ptr->getRoots().size()));

   // Print variable importance
   cv::Mat var_importance = rtree_ptr->getVarImportance();
   if(!var_importance.empty()) {
      float rt_imp_sum = float(sum(var_importance)[0]);
      cout << format("Var#   important (in %), rt-imp-sum = {}\n", rt_imp_sum);
      const int n = int(var_importance.total());
      for(int i = 0; i < n; i++)
         cout << format("{:2d}   {:4.1f}\n",
                        i,
                        100.0f * var_importance.at<float>(i) / rt_imp_sum);
   }

   // Get the mapping between classifier output, and class
   auto get_class_mapping = [&]() {
      std::vector<int32_t> vec;
      vec.reserve(size_t(labels.rows));
      for(auto i = 0; i < labels.rows; ++i)
         vec.push_back(labels.at<int32_t>(i));
      remove_duplicates(vec);
      std::sort(begin(vec), end(vec));
      return vec;
   };
   const auto class_mapping = get_class_mapping();

   string tmpd                = ""s;
   unique_ptr<Classifier> ptr = nullptr;
   try {
      tmpd = make_temp_directory("/tmp/train-rtree.XXXXXX"); // scaling...
      const auto fname_ml = format("{}/rtree-model", tmpd);

      RTreeModel rmodel;
      rmodel.model       = rtree_ptr;
      rmodel.interp_data = medians_Xr;
      rmodel.classes     = class_mapping;
      file_put_contents(fname_ml, rmodel.write(true));

      ptr = make_unique<Classifier>();
      ptr->load(fname_ml);
   } catch(std::exception& e) {
      LOG_ERR(format("exception writing rtree: {}", e.what()));
   }

   try {
      remove_all(tmpd);
   } catch(std::exception& e) {
      LOG_ERR(
          format("exception cleaning up directory '{}': {}", tmpd, e.what()));
   }

   return ptr;
}

// ------------------------------------------------------------------- train-svm

unique_ptr<Classifier> train_svm(const cv::Mat& samples,
                                 const cv::Mat& labels,
                                 const vector<float>& medians) noexcept
{
   Expects(samples.rows == labels.rows);
   Expects(labels.cols == 1);

   const auto& svm_exec = svm_train_gpu_exec();

   auto encode_it = [&]() -> string {
      std::stringstream ss{""};
      for(int r = 0; r < samples.rows; ++r) {
         ss << format("{}", labels.at<int32_t>(r, 0));
         for(int c = 0; c < samples.cols; ++c) {
            const float x = samples.at<float>(r, c);
            if(std::isfinite(x)) ss << format(" {}:{}", c, x);
         }
         ss << '\n';
      }
      return ss.str();
   };

   try {
      // Save the SVM data
      const auto fname = "/tmp/trainig-data.text";
      file_put_contents(fname, encode_it());

      INFO(format("SVM testing data saved to {}", fname));
      cout << format(R"V0G0N(
   To complete training, you need to:

     (1) create an SVM model with `{}` executable, `tools/grid.py`.
     (2) save the model file as `svm.model`
     (3) create a file `classes.text` with the class labels:

            cat /tmp/training-dat.text | awk '{{ print $1 }}' | sort | uniq | tr '\n' ',' | sed 's/,$//' > classes.text
  
     (4) place it in a tar.gz archive as follows:

            tar -c svm.model classes.text | gzip -c9 > pose-svm_vX.ml.gz

         the `pose-svm_vX` is the name of the model. The file extention
         must be `ml.gz`
     (5) copy the model onto s3 using:
 
            aws s3 cp pose-svm_vX.ml.gz s3://perceive-multiview/calibration/classifiers/

     (6) you can now use the model in multiview-cli by passing the arguments:

            multiview-cli ... --pose-classifier pose-svm_vX ...

)V0G0N",
                     basename(svm_exec));
      FATAL("kBAM!");
   } catch(std::exception& e) {
      LOG_ERR(format("exception writing classifier: {}", e.what()));
   }

   return nullptr;
}

// ------------------------------------------------------------ train-classifier

TrainClassifierResult train_classifier(const ClassifierType type,
                                       const PredictType predict,
                                       const vector<float>& medians,
                                       const cv::Mat& full_samples,
                                       const cv::Mat& full_labels,
                                       const int max_trees,
                                       const real validation_set_size, // [0..1]
                                       const size_t random_seed,
                                       const bool feedback)
{
   Expects(max_trees > 0);
   Expects(validation_set_size >= 0.0 && validation_set_size <= 1.0);
   Expects(type != ClassifierType::NONE);
   Expects(predict != PredictType::NONE);

   // -- Make the training-validation set
   const auto now0 = tick();
   const auto data = calibration::make_training_validation_set(
       full_samples, full_labels, validation_set_size, random_seed);
   const auto s0 = tock(now0);

   if(feedback) {
      INFO(format("training {} {} classifier.", str(predict), str(type)));

      cout << format(
          "   full-data:     [{}x{}]\n", full_samples.rows, full_samples.cols);
      cout << format("   validation:     {:4.2f}\n", validation_set_size);
      cout << format("   training-set:  [{}x{}]\n",
                     data.training_samples.rows,
                     data.training_samples.cols);
      cout << format("   validatn-set:  [{}x{}]\n",
                     data.validation_samples.rows,
                     data.validation_samples.cols);
   }

   TrainClassifierResult ret;
   ret.type    = type;
   ret.predict = predict;
   ret.classifier
       = type == ClassifierType::SVM
             ? train_svm(data.training_samples, data.training_labels, medians)
             : train_rtree(data.training_samples,
                           data.training_labels,
                           medians,
                           max_trees);
   Expects(ret.classifier != nullptr);

   const auto now_predict = tick();
   ret.score              = score_classifier(
       ret.classifier.get(), data.validation_samples, data.validation_labels);
   const auto predict_t
       = tock(now_predict) / real(data.validation_samples.rows);

   if(feedback) {
      cout << format("validation score: {:5.2f}%", 100.0 * ret.score) << endl;
      cout << format("classifier->predict(...) = {:7.3f}ms", predict_t * 1000.0)
           << endl;
   }

   return ret;
}

} // namespace perceive
