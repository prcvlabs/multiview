
#pragma once

#include "tracklet-exec.hpp"

namespace perceive
{
// ------------------------------------------------------------------- load/save
//
vector<Tracklet> load_tracklets_data(FILE* fp) noexcept(false);

void save_tracklets_data(
    FILE* fp,
    const unsigned N, // N tracklets, NOTE n-frames
    std::function<const Tracklet*(unsigned)> get_tracklet) noexcept;

// --------------------------------------------------------- make tracklet image
//
ARGBImage make_tracklet_image(const Tracklet& tracklet,
                              const LocalizationData& loc_data,
                              const int frame_no) noexcept;

ARGBImage make_tracklet_image(const Tracklet& tracklet,
                              const ARGBImage& background_image,
                              const int frame_no) noexcept;

void finialize_heights_labels_gaze_direction(
    std::function<const LocalizationData*(int)> get_loc) noexcept;

} // namespace perceive
