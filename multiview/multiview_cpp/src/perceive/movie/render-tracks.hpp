
#pragma once

#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/cost-functions/tracklets/tracklet-exec.hpp"
#include "perceive/cost-functions/tracks/tracks-exec.hpp"

namespace perceive::movie
{
// ----------------------------------------------------------- make-tracks-image
//
ARGBImage make_tracks_image(const Tracks& tracks,
                            // const Tracklet& tracklet,
                            const LocalizationData& loc_data,
                            const int frame_no) noexcept;

ARGBImage make_tracks_image(const Tracks& tracks,
                            const ARGBImage& background_image,
                            const int frame_no) noexcept;

ARGBImage make_tracks_image(const vector<Track>& tracks,
                            const ARGBImage& background_image,
                            const int frame_no) noexcept;

// --------------------------------------------------------- render-tracks-movie
//
void render_tracks_on_raw_video(ARGBImage& im,
                                const SceneDescription& scene_desc,
                                const int sensor_no,
                                const int frame_no,
                                const vector<Track>& tracks,
                                const vector<Track>* gt_tracks,
                                const LocalizationData* loc_ptr,
                                const bool gt_truth_only) noexcept;

} // namespace perceive::movie
