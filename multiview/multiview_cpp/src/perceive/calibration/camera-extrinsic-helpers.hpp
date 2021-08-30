
#pragma once

#include "perceive/geometry/euclidean-transform.hpp"
#include "perceive/geometry/projective/polynomial-model.hpp"
#include "perceive/geometry/vector.hpp"

namespace perceive
{
// Estimate et for a single camera, LLS
EuclideanTransform estimate_et_LLS_one_cam(const vector<Vector3>& Ws,
                                           const vector<Vector2>& U0);

EuclideanTransform
estimate_et_one_cam(const DistortionModel& M, // make sure working format is set
                    const Vector3& C0,
                    const vector<Vector3>& W, // world coords
                    const vector<Vector2>& P, // image coords
                    real& reproj_error, const bool feedback,
                    const bool super_feedback = false);

EuclideanTransform optimize_et_one_cam(const EuclideanTransform& et0,
                                       const vector<Vector3>& Ws,
                                       const vector<Vector2>& U0);

} // namespace perceive
