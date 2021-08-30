
#pragma once

#include "perceive/cost-functions/features-2d/features-2d.hpp"
#include "perceive/foundation.hpp"
#include "perceive/geometry.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/graphics/sprite.hpp"
#include "perceive/scene/scene-description.hpp"

namespace perceive
{
struct OptimizeCameraData
{
   vector<ImageFeatures2d::Params> f2d_params;

   SceneDescription scene_desc;

   // Optimization parameters
   unsigned fbo_w = 900u; // Large is slower but more precise
   unsigned fbo_h = 600u;
   real hfov      = to_radians(80.0);
   real z_near    = 0.01; // camera cannot see closer than this (1cm)
   real z_far     = dNAN; // will set from diagonal of model

   // Number of sensors
   size_t n_sensors() const noexcept { return scene_desc.sensor_ids.size(); }
};

struct OptimizeCameraResult
{
   std::string key;                           // Id that identifies the scene
   vector<EuclideanTransform> cam_transforms; // for left cams for duals
   real score = dNAN;

   // two for each camera
   vector<FloatImage> depth_maps; // dual images are stored [L|R]
};

inline void clear(OptimizeCameraResult& opt)
{
   OptimizeCameraResult t;
   opt = t;
}

OptimizeCameraResult optimize_camera_position(const OptimizeCameraData& data,
                                              const bool feedback);

OptimizeCameraResult optimize_campos_and_model(const OptimizeCameraData& data,
                                               const bool feedback);

} // namespace perceive
