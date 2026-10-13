
#pragma once

#include "perceive/io/json-io.hpp"
#include "perceive/io/struct-meta.hpp"

namespace perceive::pipeline
{
struct TestOutput;

// How far a candidate run (e.g., shards, or a stereo run with a still warm-up)
// may stray from a reference run before the difference is more than noise.
// The defaults are the stereo and live-pose tolerances; a run that replays 2D
// poses without stereo should be exact, and is checked byte for byte instead.
struct NoiseTolerances final : public MetaCompatible
{
   virtual ~NoiseTolerances() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   // Against the ground truth
   real f1_drop_max            = 0.02; // f1 is the Jaccard index
   int count_abs_slack         = 5;    // tp, fp, fn may differ by the larger
   real count_rel_slack        = 0.01; // of these, `rel` times (tp + fn)
   real mean_dist_increase_max = 0.02; // metres, per true positive

   // Candidate against reference (the reference as ground truth)
   real agree_precision_min = 0.99;
   real agree_recall_min    = 0.99;
   real agree_jaccard_min   = 0.98;
   real agree_mean_dist_max = 0.03; // metres

   // Per shard boundary, against the ground truth
   int id_switch_slack = 1;
   int frag_slack      = 2;
};

// TRUE if the candidate run is within noise of the reference run.
// `truth_ref` and `truth_cand` are the two runs scored against the ground
// truth; they may both be null when there is no ground truth, and then only
// the agreement clauses apply. `agreement` scores the candidate with the
// reference as ground truth. The id-switch and fragmentation slacks are
// multiplied by max(1, n_boundaries). Two empty runs agree. `report` gets
// one line per clause.
bool within_noise(const TestOutput* truth_ref,
                  const TestOutput* truth_cand,
                  const TestOutput& agreement,
                  const NoiseTolerances& tol,
                  const int n_boundaries,
                  string& report) noexcept;

} // namespace perceive::pipeline

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::NoiseTolerances)
} // namespace perceive
