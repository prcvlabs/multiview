
#pragma once

#include "training-data-point.hpp"

#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>

namespace perceive
{
using cvSVM = cv::Ptr<cv::ml::SVM>;

cvSVM create_cvSVM();

const char* svm_type_to_str(int type);
const char* kernel_type_to_str(int type);
string str(const cvSVM& svm) noexcept;

// @{ file I/O
bool load(const string_view fname, cvSVM&) noexcept;
bool save(const cvSVM&, const string_view fname) noexcept;
bool read(const string_view raw, cvSVM&) noexcept;
string write(const cvSVM&) noexcept; // std::bad_alloc
// @}

// Convert training-data to samples->labels matrices (for training)
std::pair<cv::Mat, cv::Mat>
transform_tp_fp_data(const vector<calibration::TrainingDataPoint>& data,
                     const real nan_val);

// Convert training-data to samples->labels matrices (for training)
std::pair<cv::Mat, cv::Mat>
transform_pose_data(const vector<calibration::TrainingDataPoint>& data,
                    const real nan_val);

// SVM type:
// { C_SVC      n>=2, non-linear seperable, penalty 'C'
//   NU_SVC     n-classes, non-lienar, nu in [0..1]
//   ONE_CLASS  one-class, estimates distribution
//   EPS_SVR    eta regression, penalty 'C', 'p'
//   NU_SVR     As above, but 'nu' instead of 'C'
// }
//
// Kernel types:
// { cv::ml::LINEAR, POLY, RBF, SIGMOID, CHI2, INTER }
//
// Parameters:
// { cv::ml::C, GAMMA, P, NU, COEF, DEGREE }
cvSVM train_model(const cv::Mat& samples, // samples predict labels
                  const cv::Mat& labels,
                  const int svm_type,
                  const int kernel_type,
                  const real nu, // parameter for svm-type = nu_SVC
                  const real C,  // parameter for svm-type = C_SVC
                  const int max_iter,
                  const double eps, // stop error
                  const bool feedback);

} // namespace perceive
