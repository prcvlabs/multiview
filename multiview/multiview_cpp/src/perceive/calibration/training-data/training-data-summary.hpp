
#pragma once

#include "training-data-point.hpp"

namespace perceive::calibration
{
struct TrainingDataSummary
{
   TrainingDataSummary();
   TrainingDataSummary(const TrainingDataSummary&) = default;
   TrainingDataSummary(TrainingDataSummary&&)      = default;
   ~TrainingDataSummary()                          = default;
   TrainingDataSummary& operator=(const TrainingDataSummary&) = default;
   TrainingDataSummary& operator=(TrainingDataSummary&&) = default;

   string set_label = "<none>"s;

   // Count tp/false-positive
   array<size_t, 2> tp_fp_counts;

   // Count poses
   array<size_t, n_pose_annotations()> pose_counts;

   size_t total_tp_fp() const noexcept;
   size_t total_poses() const noexcept;

   static string table_header_string() noexcept;
   string to_string() const noexcept;
   friend string str(const TrainingDataSummary& o) noexcept;
};

TrainingDataSummary
init_training_data_summary(const string_view set_label,
                           const vector<TrainingDataPoint>& all_data) noexcept;

string training_data_summary_table(
    const int N,
    std::function<const TrainingDataSummary&(int)> get) noexcept;

} // namespace perceive::calibration
