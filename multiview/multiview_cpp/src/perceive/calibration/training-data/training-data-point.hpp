
#pragma once

#include "perceive/geometry/skeleton/skeleton-2d-info.hpp"
#include "perceive/io/struct-meta.hpp"
#include "perceive/scene/pose-annotation.hpp"

namespace perceive
{
struct SceneDescription;
struct Skeleton2DInfo;
} // namespace perceive

namespace perceive::calibration
{
struct TrainingDataPoint
{
   static constexpr int k_n_keypoints = skeleton::k_n_keypoints;
   static constexpr int k_n_bones     = skeleton::Bone::k_n_bones;
   static constexpr int k_n_joints    = skeleton::Joint::k_n_joints;

   int frame_no         = -1;
   float speed          = 0.0f; //
   float bg_delta_score = 0.0f;
   float z_theta        = 0.0f;     // Angle between `up` and Z_Axis (radians)
   array<float, k_n_joints> joints; // joint-thetas
   PoseAnnotation pose    = PoseAnnotation::NONE;
   bool is_false_positive = false; //
   bool is_init           = false;

   bool operator==(const TrainingDataPoint& o) const noexcept;
   bool operator!=(const TrainingDataPoint& o) const noexcept;

   string to_string() const noexcept;
   string to_json_string() const noexcept;
   Json::Value to_json() const noexcept;
   bool load_json(const Json::Value& x) noexcept(false);
   void read_with_defaults(const Json::Value& o,
                           const TrainingDataPoint* defaults
                           = nullptr) noexcept;

   // [labels | flatten]
   static vector<string> headers() noexcept; // std::bad_alloc

   // Training only, so doesn't include "pose" and "is-false-positive"
   vector<real> flatten() const noexcept; // std::bad_alloc
   vector<real> labels() const noexcept;  // std::bad_alloc

   friend string str(const TrainingDataPoint& o) { return o.to_string(); }
};

META_READ_WRITE_LOAD_SAVE(TrainingDataPoint)

TrainingDataPoint make_training_datapoint(const SceneDescription& scene_desc,
                                          const Skeleton2DInfo& skel_info,
                                          const PoseAnnotation pose,
                                          const real speed,
                                          const bool is_false_positive);

VectorXr make_feature_vector(const SceneDescription& scene_desc,
                             const Skeleton2DInfo& skel_info,
                             const real speed);

string encode_as_tpfp_training_data(const vector<TrainingDataPoint>&) noexcept;
string encode_as_pose_training_data(const vector<TrainingDataPoint>&) noexcept;

// Returns {samples, labels}
// Where samples is a matrix of float values, and lables is 2 columns int32_t
// Missing values will be NAN, or the median for the column
std::pair<cv::Mat, cv::Mat>
collate_training_samples(const vector<TrainingDataPoint>& data,
                         const bool impute_missing_values) noexcept;

// Return '?' for missing values, unless they're imputed
string encode_csv(const vector<TrainingDataPoint>&,
                  const bool impute_missing_values) noexcept;

// Returns {headers, matrix-of-data, column-medians}
std::tuple<vector<string>, cv::Mat, vector<float>>
decode_csv(const string_view raw_csv,
           const bool impute_missing_values,
           const bool has_header_line = true) noexcept(false);

vector<float> calc_column_medians(const cv::Mat& mat) noexcept;

cv::Mat impute_missing_values(const cv::Mat& mat) noexcept;

// All columns are multiplied by a scalar so that the values
// are in the range [-1..1]
std::pair<cv::Mat, vector<Vector2f>> scale_values(const cv::Mat& mat) noexcept;

// Returns {samples, labels}
std::pair<cv::Mat, cv::Mat>
prepare_training_data(const cv::Mat& raw,
                      const vector<int>& sample_cols,
                      const int label_col) noexcept;

struct TrainingData
{
   cv::Mat training_samples;
   cv::Mat training_labels;
   cv::Mat validation_samples;
   cv::Mat validation_labels;
};
TrainingData
make_training_validation_set(const cv::Mat& samples,
                             const cv::Mat& labels,
                             const real validation_set_size, // [0..1]
                             const size_t random_seed);

} // namespace perceive::calibration
