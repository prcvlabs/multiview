
#pragma once

#include <string>

namespace perceive::calibration
{
bool run_stereo_cam_pos_opt(const std::string& manifest_file,
                            const bool update_E, const std::string& out_file);

} // namespace perceive::calibration
