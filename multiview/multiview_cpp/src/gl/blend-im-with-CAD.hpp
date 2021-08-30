
#pragma once

#include "perceive/geometry/euclidean-transform.hpp"
#include "perceive/geometry/projective/caching-undistort-inverse.hpp"
#include "perceive/geometry/projective/polynomial-model.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/graphics/sprite.hpp"

namespace perceive
{
ARGBImage blend_im_with_CAD(const ARGBImage& raw,
                            const DistortionModel& M,
                            const EuclideanTransform& et_opt,
                            const Sprite& cad,
                            const int fbo_w,
                            const int fbo_h,
                            const real fbo_hfov);

} // namespace perceive
