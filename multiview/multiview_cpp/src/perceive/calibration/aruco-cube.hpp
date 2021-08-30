
#pragma once

#include <opencv2/aruco.hpp>

#include "perceive/geometry/projective/caching-undistort-inverse.hpp"
#include "perceive/graphics/image-container.hpp"

namespace perceive
{
struct CachingUndistortInverse;

struct ArucoCube
{
 public:
   // The origin... the intersection of {FRONT, EAST, BOTTOM}
   static constexpr int k_n_faces = 6;
   enum face_e : int { FRONT = 0, BACK, EAST, WEST, TOP, BOTTOM };

   struct FaceMeasurement
   {
    public:
      array<AABB, 4> marker_aabbs_;
      face_e face              = face_e::BOTTOM;
      array<int, 4> marker_ids = {{0, 0, 0, 0}};
      uint32_t kolour          = 0x00000000u; // colour for drawing this face

      // All points in Xs *must* be planar.
      // X[0] is the origin
      // X[1]-X[0] is the X-axis direction
      // X[1]-X[2] is the Y-axis direction
      //
      //    0 -- 1
      //    |    |
      //    3 -- 2
      //
      array<Vector3, 4> Xs; // 3d bounding quad

      // Derived data...
      Plane p3;               // the plane that contains 'Xs'
      real marker_size = 0.0; // Markers _must_ be square

      // 3D coords
      Vector3 x_axis() const noexcept { return (Xs[1] - Xs[0]).normalised(); }
      Vector3 y_axis() const noexcept { return (Xs[1] - Xs[2]).normalised(); }
      Vector3 z_axis() const noexcept { return cross(x_axis(), y_axis()); }
      real width() const noexcept { return (Xs[1] - Xs[0]).norm(); }
      real height() const noexcept { return (Xs[1] - Xs[2]).norm(); }

      // finds the position of 'marker_id' in 'marker_ids': {0, 1, 2, 3}
      int marker_pos(const int marker_id) const noexcept;

      // 2D coords, 'm_pos' must be in {0, 1, 2, 3}
      const AABB& marker_aabb(const int m_pos) const noexcept;

      // The 3D quad for a given marker 'pos'. Must be in {0, 1, 2, 3}
      array<Vector3, 4> marker_3d_quad(const int m_pos) const noexcept;
   };

   struct FaceDetection
   {
      face_e f_ind{TOP};
      using Quadrilateral = array<Vector2, 4>; // Quadrilateral
      using Quad3d        = array<Vector3, 4>; // Quadrilateral 3D
      vector<Quadrilateral> quads;             // detected quadrilaterals
      vector<int> marker_ids;                  // index into 'markers'
      vector<EuclideanTransform> ets;          // Initial pose estimate
      vector<Quad3d> quad3ds;                  // Initial 3d estimate
      Plane p3; // Initial estimate of face's plane

      string to_string() const noexcept;
   };

   // Given
   // @param et, The euclidean transform for world=>cam 3D coords
   // @param face_im, Face images generated from ac.draw_face(...)
   // @return One AcucoFaceProjectionInfo for each input detection.
   struct FaceProjectionInfo
   {
      ArucoCube::face_e f_ind;      // face index
      uint32_t kolour{0x00000000u}; // 'colour' of this face
      array<Vector3, 4> quad3d;     // face of image in 3D cam coords
      Plane p3;                     // the plane that contains quad3d
      Matrix3r H; // Homography, 3D cam point => 2d face image coord
   };

 private:
   // Measurements:
   //
   //    X --->
   //    | mx+ s +      +         ^
   //    |   X          0
   //  + +-----------+----------+ |
   // my |           |          | |
   //  Y |   +---+   |  +---+   | Y
   //  s |   | 0 |   |  | 1 |   |
   //    |   +---+   |  +---+   |
   //    |           |          |
   //    +-----------o----------+
   //    |           |          |
   //  1 |   +---+   |  +---+   | 1
   //    |   | 2 |   |  | 3 |   |
   //    |   +---+   |  +---+   |
   //    |           |          |
   //    +-----------+----------+
   //
   // The digits give the markers of the faces. The markers
   // are oriented in the same way as the text.
   // 'mx' is the margin in the x axis direction
   // 's' is the length of an individual marker
   // 'col' is the length from 'X' to the '0'
   // Simiarly,
   // 'my' is the margin in the y-axis direction
   // 'row' is the length from 'Y' to the '1'

   // Measurement '0' is the distance from the 'X' to '0' above.
   // Measurement '1' is the distance from the 'X' to '1' above.
   // 'X' is the origin for the measurements.
   // 'm' is the border margin.
   // 's' is the length of the side of an individual marker image.
   //

 public:
   // maps a 'markers' index (below) to cv-dict
   array<int, 4 * k_n_faces> cv_dict_ind;
   std::unordered_map<int, int> marker_lookup; // cv-dict => 'markers' index

   int cv_dict_ind_to_index(int idx) const noexcept;
   int index_to_cv_dict_ind(int idx) const noexcept;

   // Markers must be the same size on all faces
   string id        = ""s;
   real marker_size = 0.0;
   array<FaceMeasurement, k_n_faces> measures; // six faces
   array<cv::Mat, 4 * k_n_faces> markers;      // there's four per face

   // This face is visible, when seen from camera-centre 'C'
   bool face_is_visible(const FaceMeasurement& M, const Vector3& C) const
       noexcept;

   // FaceProjectionInfo
   FaceProjectionInfo calc_face_projection_info(const EuclideanTransform& et,
                                                const ArucoCube::face_e f_ind,
                                                const int width,
                                                const int height) const;

   // The returned functor references `cu` and `detects`
   std::function<real(const EuclideanTransform& et)>
   pointwise_error_fun(const CachingUndistortInverse& cu,
                       const vector<ArucoCube::FaceDetection>& detects) const;
   // The returned functor references `cu`, `image_lab`, and `detects`
   std::function<real(const EuclideanTransform& et)>
   dense_error_fun(const CachingUndistortInverse& cu,
                   const LABImage& image_lab,
                   const array<cv::Mat, 6>& face_ims,
                   const vector<ArucoCube::FaceDetection>& detects) const;

   void render_pointwise(ARGBImage& argb,
                         const vector<ArucoCube::FaceDetection>& detects,
                         const CachingUndistortInverse& cu,
                         const EuclideanTransform& cam_et, // World->cam
                         const EuclideanTransform& cube_et // Wolrd->cube
                         ) const;

   void render_dense(ARGBImage& argb,
                     const vector<ArucoCube::FaceDetection>& detects,
                     const array<cv::Mat, 6>& face_ims,
                     const CachingUndistortInverse& cu,
                     const EuclideanTransform& cam_et, // World->cam
                     const EuclideanTransform& cube_et // Wolrd->cube
                     ) const;

   static ArucoCube make_kyle_aruco_cube() noexcept;
   static const char* face_to_str(face_e) noexcept;
   static face_e str_to_face(const string_view ss) noexcept(false);

   // Cube unchanged if an exception is thrown
   void init_from_json(const Json::Value& root) noexcept(false);
   string to_json_str() const noexcept;
   friend string str(const ArucoCube& acube) noexcept;

   // 3D coords of a given face.
   // The origin is at the intersection of {FRONT, BOTTOM, EAST}
   // The X-axis points back (front-to-back)
   // The Y-axis points across (east-to-west)
   // The Z-axis points up (bottom-to-top)
   // Coordinates are returned in clock-wise order.
   // array<Vector3, 4> face_coordz(face_e) const noexcept;

   // Draw face 'ind' as an image of 'width' pixels.
   // The height is calculated automatically.
   // If 'print-coords' is TRUE, then the 3D locations of features
   // are rendered onto the face
   cv::Mat draw_face(face_e f_ind,
                     unsigned width,
                     bool print_coords,
                     bool render_labels = false) const noexcept;

   real width(face_e f_ind) const noexcept;  // X direction in metres
   real height(face_e f_ind) const noexcept; // Y direction in metres

   Vector3 center() const noexcept;

   vector<FaceDetection> detect_markers(const cv::Mat& im,
                                        const Matrix3r& K,
                                        const string_view out_filename) const;

   vector<FaceDetection> detect_markers(const cv::Mat& im,
                                        const CachingUndistortInverse& cu,
                                        const string_view out_filename) const;

   friend void read(ArucoCube& acube, const string_view ss) noexcept(false);
   friend void write(const ArucoCube& acube, string& ss) noexcept;
};

ArucoCube make_kyle_aruco_cube() noexcept;

// -------------------------------------------------- projecting faces
//
void project_aruco_face(
    const ArucoCube::FaceProjectionInfo& pinfo,
    const CachingUndistortInverse& cu,
    std::function<void(Point2 image_xy, Point2 aruco_face_xy)> g) noexcept;

// `pinfo` already contains the necessary transformations
void project_aruco_face(
    const ArucoCube::FaceProjectionInfo& pinfo,
    const CachingUndistortInverse& cu,
    const cv::Mat& face_im,
    std::function<void(int x, int y, uint32_t k)> f) noexcept;

// ---------------------------------------------------------- init-ets
//
std::pair<EuclideanTransform, bool>
estimate_init_et(const ArucoCube& ac,
                 const vector<ArucoCube::FaceDetection>& detects,
                 const CachingUndistortInverse& cu,
                 const string_view cam_pos_name, // for printing opt feedback
                 const bool feedback) noexcept;

std::pair<EuclideanTransform, bool> dense_refine_init_et(
    const EuclideanTransform& in_et,
    const ArucoCube& ac,
    const CachingUndistortInverse& cu,
    const LABImage& image_lab,
    const array<cv::Mat, 6>& face_ims,
    const vector<ArucoCube::FaceDetection>& detects,
    const string_view cam_pos_name, // for printing opt feedback
    const bool feedback) noexcept;

} // namespace perceive
