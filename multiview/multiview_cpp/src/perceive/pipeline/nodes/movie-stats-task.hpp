
#pragma once

#include "stdinc.hpp"

#include "load_scene_description.hpp"
#include "perceive/cost-functions/movie-stats/movie-stats-file.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/pipeline-task.hpp"

// Loads movie-stats
namespace perceive::pipeline::movie_stats
{
struct Params final : public MetaCompatible
{
   CUSTOM_NEW_DELETE(Params)

   virtual ~Params() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   bool feedback{true};
   string out_dir{"/tmp"s};

   bool no_stats = false;
   string save_filename; // If set, will attempt to load the stats
   vector<string> in_stats_fnames;
};

struct Result
{
   CUSTOM_NEW_DELETE(Result)

   shared_ptr<const SceneDescription> scene_desc;
   bool has_stats = false;
   MovieStatsFile stats;
};

class Task : public PipelineTask<Params, Result>
{
 public:
   CUSTOM_NEW_DELETE(Task)

 protected:
   shared_ptr<const Result>
   execute(const RunData& data,
           const Params& params,
           std::function<bool()> is_cancelled) const noexcept override;
};

} // namespace perceive::pipeline::movie_stats

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::movie_stats::Params)
} // namespace perceive
