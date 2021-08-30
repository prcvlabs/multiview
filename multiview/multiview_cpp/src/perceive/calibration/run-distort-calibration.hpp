
#pragma once

#include "perceive/foundation.hpp"
#include "perceive/io/perceive-assets.hpp"

namespace perceive::calibration
{
bool run_distort_calibration(const unsigned nx, const unsigned ny,
                             const real grid_size, const string& sensor_id,
                             const string& manifest_file,
                             const bool fast_calibration,
                             const string& out_file, const string& out_dir);

bool run_distort_calibration(const unsigned nx, const unsigned ny,
                             const real grid_size, const string& sensor_id,
                             const string& manifest_file,
                             const bool fast_calibration,
                             const DataSource out_source,
                             const string& out_dir);

} // namespace perceive::calibration
