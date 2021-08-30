
#pragma once

#include "perceive/foundation.hpp"
#include "perceive/io/perceive-assets.hpp"

namespace perceive::calibration
{
bool run_stereo_calibration(const unsigned nx, const unsigned ny,
                            const real square_size, const string& camera_id,
                            const DataSource out_source, const string& out_dir,
                            const string& distort_key0,
                            const string& distort_key1, const bool fit_splines,
                            const bool find_K, const bool estimate_error_map,
                            const real time_threshold,
                            const vector<string>& filenames);

bool run_stereo_calibration(const unsigned nx, const unsigned ny,
                            const real square_size, const string& camera_id,
                            const string& out_file, const string& out_dir,
                            const string& calib_fname0,
                            const string& calib_fname1, const bool fit_splines,
                            const bool find_K, const bool estimate_error_map,
                            const real time_threshold,
                            const vector<string>& filenames);

} // namespace perceive::calibration
