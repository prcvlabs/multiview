
#include "training-data-summary.hpp"

#define This TrainingDataSummary

namespace perceive::calibration
{
// ---------------------------------------------------------------- construction
//
This::This()
{
   std::fill(begin(tp_fp_counts), end(tp_fp_counts), 0);
   std::fill(begin(pose_counts), end(pose_counts), 0);
}

// ---------------------------------------------------------------------- totals
//
size_t This::total_tp_fp() const noexcept
{
   return tp_fp_counts.at(0) + tp_fp_counts.at(1);
}

size_t This::total_poses() const noexcept
{
   return std::accumulate(begin(pose_counts), end(pose_counts), size_t(0));
}

// --------------------------------------------------------- table-header-string
//
static const char* table_divider
    = "+--------------------------------+-------+-------+-------+"
      "+-------+-------+-------+------"
      "-+-------+-------+-------+";

static const char* header
    = "| name                           |   tps |   fps | "
      "total || stand |  walk |  sit  |   "
      "lay | phone | other | total |";

string This::table_header_string() noexcept
{
   return format("{}\n{}\n{}", table_divider, header, table_divider);
}

// ------------------------------------------------------------------- to-string
//
string This::to_string() const noexcept
{
   auto calc_pose_summary = [&]() {
      return implode(cbegin(pose_counts) + 1,
                     cend(pose_counts),
                     " |",
                     [](const auto count) { return format("{:6d}", count); });
   };

   return format("| {:30s} |{:6d} |{:6d} |{:6d} ||{:s} |{:6d} |",
                 set_label,
                 tp_fp_counts.at(0),
                 tp_fp_counts.at(1),
                 total_tp_fp(),
                 calc_pose_summary(),
                 total_poses() - pose_counts.at(0));
}

string str(const TrainingDataSummary& o) noexcept { return o.to_string(); }

// -------------------------------------------------- init-training-data-summary
//
TrainingDataSummary
init_training_data_summary(const string_view set_label,
                           const vector<TrainingDataPoint>& all_data) noexcept
{
   TrainingDataSummary out;
   out.set_label = set_label;
   for(const auto& data : all_data) {
      out.tp_fp_counts.at(size_t(data.is_false_positive)) += 1;
      out.pose_counts.at(size_t(data.pose)) += 1;
   }
   return out;
}

// ------------------------------------------------- training-data-summary-table
//
string training_data_summary_table(
    const int N,
    std::function<const TrainingDataSummary&(int)> get) noexcept
{
   auto table_body = [&]() {
      std::stringstream ss{""};
      for(auto ind = 0; ind < N; ++ind) ss << str(get(ind)) << "\n";
      return ss.str();
   };

   auto collate = [&](const auto& fun, auto& totals) {
      const size_t sz = fun(totals).size();
      for(size_t ind = 0; ind < size_t(N); ++ind)
         for(size_t i = 0; i < sz; ++i)
            fun(totals).at(i) += fun(get(int(ind))).at(i);
   };

   TrainingDataSummary totals;
   totals.set_label = "Grand Totals";
   collate(std::mem_fn(&TrainingDataSummary::tp_fp_counts), totals);
   collate(std::mem_fn(&TrainingDataSummary::pose_counts), totals);

   return format("{}\n{}{}\n{}\n{}\n{}\n{}",
                 calibration::TrainingDataSummary::table_header_string(),
                 table_body(),
                 table_divider,
                 str(totals),
                 table_divider,
                 header,
                 table_divider);
}

} // namespace perceive::calibration
