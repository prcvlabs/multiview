
#pragma once

#include "binocular-camera.hpp"

namespace perceive
{
void test_distorted_bino_pair(const BinocularCameraInfo& bcam_info,
                              const BinocularCamera& bcam,
                              const cv::Mat raw[2],
                              const cv::Mat rectified[2],
                              const vector<Vector2>& d0,
                              const vector<Vector2>& d1);

void test_distorted_bino_pair(const BinocularCameraInfo& bcam_info,
                              const BinocularCamera& bcam,
                              const cv::Mat raw[2],
                              const cv::Mat rectified[2],
                              const vector<std::pair<Vector2, Vector2>>& d01);

} // namespace perceive
