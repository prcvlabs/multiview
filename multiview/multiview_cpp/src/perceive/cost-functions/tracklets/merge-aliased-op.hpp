
#pragma once

#include "perceive/cost-functions/floor-histogram.hpp"
#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/cost-functions/pose-skeleton/pose-skeleton-exec.hpp"
#include "perceive/cost-functions/pose-skeleton/pose-3d.hpp"
#include "perceive/cost-functions/tracklets/tracklet-exec.hpp"
#include "perceive/scene/scene-description.hpp"
#include "perceive/io/json-io.hpp"

namespace perceive
{
struct MergePose3DExec
{
   struct Params final : public MetaCompatible
   {
      virtual ~Params() {}
      bool do_merge = true;
      Pose3D::Params p3d_params;
      real iu_threshold = 0.01;
      const vector<MemberMetaData>& meta_data() const noexcept override;
   };

   static vector<Pose3D> calculate(const SceneDescription& scene_desc,
                                   const LocalizationData& ldat,
                                   const Params& params,
                                   const Tracklet::Params& tparams) noexcept;
};

} // namespace perceive
