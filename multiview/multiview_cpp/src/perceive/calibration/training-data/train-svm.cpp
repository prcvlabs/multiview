
#include "train-svm.hpp"

#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>

#include "perceive/utils/file-system.hpp"

namespace perceive
{
using calibration::TrainingDataPoint;

cvSVM create_cvSVM() { return cv::ml::SVM::create(); }

const char* svm_type_to_str(int type)
{
   switch(type) {
#define CASE(x) \
   case cv::ml::SVM::x: return #x
      CASE(C_SVC);
      CASE(NU_SVC);
      CASE(ONE_CLASS);
      CASE(EPS_SVR);
      CASE(NU_SVR);
#undef CASE
   }
   return "<unknown svm-type>";
}

const char* kernel_type_to_str(int type)
{
   switch(type) {
#define CASE(x) \
   case cv::ml::SVM::x: return #x
      CASE(CUSTOM);
      CASE(LINEAR);
      CASE(POLY);
      CASE(RBF);
      CASE(SIGMOID);
      CASE(CHI2);
      CASE(INTER);
#undef CASE
   }
   return "<unknown kernel-type>";
}

string str(const cvSVM& svm) noexcept
{
   if(svm.get() == nullptr) return "<null svm>"s;

   return format(R"V0G0N(
SVM:
   type:           {}
   kernel-type:    {}
   empty:          {}
   var-count:      {}
   is-classifier:  {}
   is-trained:     {}
   nu:             {}
   p:              {}
   C:              {}
   #-support-vecs: {}
)V0G0N",
                 svm_type_to_str(svm->getType()),
                 kernel_type_to_str(svm->getKernelType()),
                 str(svm->empty()),
                 svm->getVarCount(),
                 str(svm->isClassifier()),
                 str(svm->isTrained()),
                 svm->getNu(),
                 svm->getP(),
                 svm->getC(),
                 svm->getSupportVectors().rows);
}

// ---------------------------------------------------------------- Input/Output
//
bool load(const string_view fname, cvSVM& svm) noexcept
{
   return read(file_get_contents(fname), svm);
}

bool save(const cvSVM& svm, const string_view fname) noexcept
{
   try {
      file_put_contents(fname, write(svm));
      return true;
   } catch(std::exception& e) {
      LOG_ERR(format("failed to write svm to file '{}': {}", fname, e.what()));
   }
   return false;
}

bool read(const string_view raw, cvSVM& svm) noexcept
{
   try {
      const string data{raw.data(), raw.size()};
      auto fs = cv::FileStorage{data,
                                cv::FileStorage::READ |       //
                                    cv::FileStorage::MEMORY | //
                                    cv::FileStorage::FORMAT_JSON};
      svm->read(fs.root());
      return true;
   } catch(std::exception& e) {
      LOG_ERR(format("failed to read cv::SVM: {}", e.what()));
   }
   return false;
}

string write(const cvSVM& svm) noexcept
{
   auto fs = cv::FileStorage{};
   fs.open("svm.json"s,
           cv::FileStorage::WRITE |      //
               cv::FileStorage::MEMORY | //
               cv::FileStorage::FORMAT_JSON);
   svm->write(fs);
   return fs.releaseAndGetString();
}

// -----------------------------------------------------------
//
static std::tuple<cv::Mat, cv::Mat, cv::Mat>
transform_data_(const vector<TrainingDataPoint>& data, const real nan_val)
{
   Expects(data.size() > 0);
   const int rows = int(data.size());
   const int cols = int(data.front().flatten().size());

   auto samples = cv::Mat(rows, cols, CV_32F);
   auto labels0 = cv::Mat(rows, 1, CV_32SC1);
   auto labels1 = cv::Mat(rows, 1, CV_32SC1);

   auto get_val = [nan_val](const auto& X, int col) {
      const auto val = X.at(size_t(col));
      return std::isfinite(val) ? val : nan_val;
   };

   for(auto row = 0; row < rows; ++row) {
      const auto X = data.at(size_t(row)).flatten();
      Expects(X.size() == size_t(cols));
      for(auto col = 0; col < cols; ++col)
         samples.at<float>(row, col) = float(get_val(X, col));
      labels0.at<int32_t>(row, 0) = data.at(size_t(row)).is_false_positive;
      labels1.at<int32_t>(row, 0) = int(data.at(size_t(row)).pose);
   }

   return {samples, labels0, labels1};
}

std::pair<cv::Mat, cv::Mat>
transform_tp_fp_data(const vector<TrainingDataPoint>& data, const real nan_val)
{
   auto [sample, l0, l1] = transform_data_(data, nan_val);
   return {sample, l0};
}

std::pair<cv::Mat, cv::Mat>
transform_pose_data(const vector<TrainingDataPoint>& data, const real nan_val)
{
   auto [sample, l0, l1] = transform_data_(data, nan_val);
   return {sample, l1};
}

// ----------------------------------------------------------- train-tp-fp-model
//
cvSVM train_model(const cv::Mat& samples, // samples predict labels
                  const cv::Mat& labels,
                  const int svm_type,
                  const int kernel_type,
                  const real svm_nu, // parameter for svm-type = nu_SVC
                  const real svm_C,  // parameter for svm-type = C_SVC
                  const int max_iter,
                  const double eps,
                  const bool feedback) // error
{
   if(true) {
      INFO(format("[{}x{}], [{}x{}], nu = {}, C = {}",
                  samples.rows,
                  samples.cols,
                  labels.rows,
                  labels.cols,
                  svm_nu,
                  svm_C));
   }
   Expects(samples.rows > 3);
   Expects(samples.rows == labels.rows);

   const auto now = tick();

   auto svm = create_cvSVM();

   svm->setType(svm_type);
   svm->setKernel(kernel_type);
   svm->setNu(svm_nu);
   svm->setC(svm_C);
   svm->setTermCriteria(cv::TermCriteria(
       cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, max_iter, eps));

   if(feedback || true)
      INFO(format("training, samples = [{}x{}], labels = [{}x{}]",
                  samples.rows,
                  samples.cols,
                  labels.rows,
                  labels.cols));
   svm->train(samples, cv::ml::ROW_SAMPLE, labels);

   if(feedback) {
      INFO(format("training finished, {}s", tock(now)));
      cout << str(svm) << endl;
   }

   return svm;
}

} // namespace perceive
