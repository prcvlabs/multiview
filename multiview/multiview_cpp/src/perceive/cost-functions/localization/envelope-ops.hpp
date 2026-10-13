
#pragma once

#include "localization-data.hpp"

namespace perceive::ldata
{
// The frames [start, start + n) kept from one envelope by 'cat_envelopes'.
struct KeepRange
{
   int start = 0;
   int n     = 0;
};

struct CatResult
{
   LocalizationDataEnvelope envelope   = {}; // kept frames, contiguous
   vector<Json::Value> overlap_reports = {}; // one per overlapping boundary
   string error = ""s; // non-empty => failure, and 'envelope' is empty
};

// Concatenates `envelopes`, taking `keep[i]` from `envelopes[i]`. If `keep`
// is empty, then envelope `i` contributes [start_i, start_{i+1}), and the
// last one all its frames. Fails (no FATAL) when the kept ranges do not tile
// one contiguous range in order, a kept range is outside its envelope, the
// frame durations or histogram geometries differ, or a frame's `frame_no`
// is not its position. Frames computed by two neighbouring envelopes are
// compared, and each boundary with such frames gets an overlap report.
CatResult cat_envelopes(const vector<LocalizationDataEnvelope>& envelopes,
                        const vector<KeepRange>& keep) noexcept;

// Frames are compared twice: as the bytes `LocalizationData::write` produces
// (`bytes_equal`, which is what 'identical' means), and with operator==
// (`equal`), which has a tolerance on the 2D poses and skips `prob_xy`.
struct FrameDiff
{
   int frame_no           = -1;
   bool bytes_equal       = false;
   bool equal             = false; // operator== and the same 'frame_no'
   string md5_a           = ""s;   // of the frame's bytes
   string md5_b           = ""s;
   bool header_equal      = false; // frame_no, loc-min/max, hist-sz, bounds
   bool hist_dims_equal   = false;
   int hist_cells_differ  = 0;
   int hist_max_abs       = 0;   // Grey16 units
   real hist_l1_mean      = 0.0; // per cell
   bool p2ds_equal        = false;
   int p2d_count_a        = 0;
   int p2d_count_b        = 0;
   int p2d_sensors_differ = 0;

   Json::Value to_json() const noexcept;
};

struct DiffResult
{
   string error             = ""s;   // non-empty => nothing could be compared
   bool headers_equal       = false; // start-frame, n-frames, frame-duration
   int first_frame          = -1;    // the common frame range
   int n_frames             = 0;
   int n_bytes_equal        = 0;
   int n_equal              = 0;
   FrameDiff worst          = {}; // by hist-max-abs, then cells, then p2ds
   vector<FrameDiff> frames = {}; // the frames whose bytes differ

   bool identical() const noexcept;
   Json::Value to_json(const bool verbose) const noexcept;
};

// Compares the frames that `a` and `b` have in common.
DiffResult diff_envelopes(const LocalizationDataEnvelope& a,
                          const LocalizationDataEnvelope& b) noexcept;

// Summary of an envelope, including the md5 of each frame's bytes;
// `fname`, if not empty, adds the file's size and md5.
Json::Value envelope_info(const LocalizationDataEnvelope& env,
                          const string_view fname) noexcept;

} // namespace perceive::ldata
