
#pragma once

#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/graphics/image-container.hpp"

namespace perceive
{
struct PhaseCalibrationParams
{
   string camera_id                   = ""s;
   vector<ARGBImage> raw_input_images = {};
   vector<array<ARGBImage, 2>> images = {};
   array<DistortionModel, 2> sensors  = {};
   bool do_undistort                  = true;
   real distance                      = dNAN; // Distance to the plane
   string outdir                      = "/tmp"s;
};

bool run_phase_stereo_calibration(PhaseCalibrationParams&) noexcept;

} // namespace perceive
