
#pragma once

#include <opencv2/core.hpp>

namespace perceive
{
// ----------------------------------------------------------------------- enums

enum class PredictType : int { NONE, TPFP, POSE };
enum class ClassifierType : int { NONE, SVM, RTREE };

const char* str(const PredictType o) noexcept;
const char* str(const ClassifierType o) noexcept;

// ------------------------------------------------------------------ Classifier
//
class Classifier
{
 private:
   struct Pimpl;
   unique_ptr<Pimpl> pimpl_; // ARGH!!!! Must by copyable to use in the pipeline

 public:
   Classifier();
   Classifier(const Classifier&) = delete;
   Classifier(Classifier&&)      = default;
   ~Classifier();
   Classifier& operator=(const Classifier&) = delete;
   Classifier& operator=(Classifier&&) = default;

   bool is_init() const noexcept;
   bool is_svm() const noexcept;
   bool is_rtree() const noexcept;

   real predict(const VectorXr& data, const bool fill_missing_values) const
       noexcept;
   real predict(const VectorXr& data,
                const bool fill_missing_values,
                vector<real>& out_probs) const noexcept;

   string to_string() const noexcept;
   friend string str(const Classifier&) noexcept;

   bool load(const string_view fname) noexcept;
   bool save(const string_view fname) const noexcept;
   bool read(const string_view raw) noexcept;
   string write() const noexcept; // std::bad_alloc
};

real score_classifier(const Classifier* ptr,
                      const cv::Mat& samples,
                      const cv::Mat& labels) noexcept;

unique_ptr<Classifier> train_rtree(const cv::Mat& samples,
                                   const cv::Mat& labels) noexcept;

struct TrainClassifierResult
{
   ClassifierType type               = ClassifierType::NONE;
   PredictType predict               = PredictType::NONE;
   unique_ptr<Classifier> classifier = nullptr;
   real score                        = 0.0;
};
TrainClassifierResult train_classifier(const ClassifierType type,
                                       const PredictType predict,
                                       const vector<float>& medians,
                                       const cv::Mat& samples,
                                       const cv::Mat& labels,
                                       const int max_trees,
                                       const real validation_set_size, // [0..1]
                                       const size_t random_seed,
                                       const bool feedback);

} // namespace perceive
