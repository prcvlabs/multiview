
#pragma once

#include "perceive/foundation.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/graphics/image-container.hpp"

namespace perceive
{
// -----------------------------------------------------------------------------
// --                                                               Proposals --
// -----------------------------------------------------------------------------
//
// EXAMPLE
// const double props_resolution = 10.0; // histogram bin-size is 10 pixels
// Proposals props;
// props.init(in_img_pts, props_resolution, feedback);
//
// // try to get 2 at least two images for each histogram bin.
// props.calc_proposals(prop_img_pts, 2, feedback);

// struct Proposals
// {
//     AABB aabb;
//     double resolution = 0.0;
//     vector< vector<Vector3r> > img_pts;
//     ImageContainerT<std::vector<unsigned> > histogram;

//     Point2 transform(const Vector3r& X) const;

//     // Initializes the histogram (and copies base-img-pts)
//     void init(const vector< vector<Vector3r> >& base_img_pts,
//               const double resolution_,
//               bool feedback);

//     // Copies the proposals into 'out'
//     void calc_proposals(vector< vector<Vector3r> >& out,
//                         const unsigned rounds,
//                         bool feedback) const;

// };

vector<unsigned> calc_proposals(const vector<vector<Vector3r>> pts,
                                const double resolution, const unsigned rounds,
                                const bool shuffle, // shuffle the order
                                const string feedback_image_fname);

} // namespace perceive
