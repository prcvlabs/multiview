
#pragma once

/**
 * Note that for any pair of cameras:
 * [1] A point in one corresponds to a line in another
 * [2] A line in one corresponds to a line in another.
 * Leave points aside, and note that each line represents a plane in 3D.
 * The plane as a homogeneous representation in P^3, and so we can calculate
 * the distance between two planes as the inner product of the unit-vector
 * homogenous representation.
 *
 * Now, for input, we mark line segments for each sensor view.
 * Each line segment is labelled, such that if labels correspond across cameras.
 * That is, Line A in camera 1 and Line B in camera 2 are corresponding lines.
 *
 * We mark special lines that we know are on the floor plane. This anchors
 * the total reconstruction. Each Line-Cam generates a plane. Two cams looking
 * at a floor line, should have their plane-plane intersections lying on the
 * floor plane.
 *
 * In theory, we could add constraints for 3D vertical lines... and more but
 * lets keep it simple. We can also add in point-point correspondences.
 *
 * Now, rank all the pairs of cameras by the number of common lines that they
 * can see.
 * [1] Pick the pair that can see the most common lines.
 * [2] Bundle adjust to get the relative transformation between the cameras.
 * [3] Mark all Lines that can be seen by the two cameras.
 * [4] Loop until all cameras calibrated
 *     [4a] Find the uncalibrated camera that sees the most marked lines.
 *     [4b] Mark all lines that that camera sees
 *     [4c] Bundle adjust that camera to all currently calibrated cameras.
 *     [4d] Bundle adjust all calibrated cameras collectively.
 * [5] Save the new calibration. It should be good.
 *
 * In the gui, it would be nice to be able to select lines, and see them
 * drawn in all images. Also, select points, and see them drawn.
 *
 * Also, in the gui, would be nice to be able to click and see the result.
 */

#include "perceive/geometry/projective/distorted-camera.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/io/json-io.hpp"

namespace perceive::calibration
{
struct LabelledPoint
{
   int label     = -1;
   int camera_no = -1;
   Vector2f x; // image coordinate
   bool on_floor  = false;
   bool is_origin = false;

   string to_string() const noexcept;
   friend string str(const LabelledPoint& o) noexcept;
   Json::Value to_json() const noexcept;
   bool read(const Json::Value&) noexcept;
};

struct LabelledLine
{
   enum Axis : int8_t { NONE = 0, X_AXIS, Y_AXIS, Z_AXIS };

   int label     = -1;    // all labels >= 0
   int camera_no = -1;    // we only treat the left sensor of each camera.
   Vector2f x0, x1;       // Images coorindates of two points on the line.
   Axis axis     = NONE;  // Line correspondences only useful if we know axis
   bool on_floor = false; // TRUE iff this is a floor-line

   string to_string() const noexcept;
   friend string str(const LabelledPoint& o) noexcept;
   Json::Value to_json() const noexcept;
   bool read(const Json::Value&) noexcept;
};

const string_view str(LabelledLine::Axis) noexcept;

struct CalibrationInfo
{
   vector<LabelledPoint> points;
   vector<LabelledLine> lines;

   // Returns an empty string if everything is okay
   string validate_calibration(const int n_cameras,
                               const vector<AABBi>& bounds) const noexcept;

   string to_string() const noexcept;
   friend string str(const LabelledPoint& o) noexcept;
   Json::Value to_json() const noexcept;
   bool read(const Json::Value&) noexcept;
};

//
vector<DistortedCamera>
bundle_adjust(const vector<DistortedCamera>& dcams,
              const CalibrationInfo& calib_info) noexcept;

// ----------------------------------------------------------- point-point error
//
// Some useful function that should be tested
Vector3f epipolar_line(const DistortedCamera& dcam0,
                       const DistortedCamera& dcam1,
                       const Vector3f& n0 // normalized coordinate
                       ) noexcept;

// A line in an image is a plane in 3D
Plane4f projective_line_to_plane(const DistortedCamera& dcam,
                                 const Vector3f& ray0,
                                 const Vector3f& ray1) noexcept;
Vector3f image_line(const DistortedCamera& dcam,
                    const Vector3f& axis,
                    const Vector3f& n0,
                    const Vector3f& n1,
                    const Plane4f& p3) noexcept;
float line_line_error(const DistortedCamera& dcam0,
                      const DistortedCamera& dcam1,
                      LabelledLine::Axis axis,
                      const Vector3f& a0,
                      const Vector3f& a1,
                      const Vector3f& b0,
                      const Vector3f& b1) noexcept;

} // namespace perceive::calibration
