
#pragma once

#include "aruco-cube.hpp"
#include "phase-position.hpp"

#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/graphics/image-container.hpp"

namespace perceive::calibration
{
// Get the initial EuclideanTransform from the detection: World => Cam
EuclideanTransform estimate_et0(const ArucoCube& ac,
                                const vector<ArucoCube::FaceDetection>& detects,
                                const Matrix3r& K,
                                ARGBImage* argb_ptr = nullptr) noexcept;

// Get the initial EuclideanTransform from the detection: World => Cam
bool refine_pointwise_et(const ArucoCube& ac,
                         const vector<ArucoCube::FaceDetection>& detects,
                         const CachingUndistortInverse& cu,
                         const string_view cam_id,
                         EuclideanTransform& inout_et,
                         ARGBImage* argb_ptr,
                         const bool feedback) noexcept;

// Get the initial EuclideanTransform from the detection: World => Cam
bool refine_pointwise_et(const ArucoCube& ac,
                         const vector<ArucoCube::FaceDetection>& detects,
                         const Matrix3r& K,
                         const int cam_id,
                         EuclideanTransform& inout_et,
                         ARGBImage* argb_ptr,
                         const bool feedback) noexcept;

// Get the initial EuclideanTransform from the detection: World => Cam
void refine_et_dense(const ArucoCube& ac,
                     const vector<ArucoCube::FaceDetection>& detects,
                     const array<cv::Mat, 6>& face_ims,
                     const Matrix3r& K,
                     const int cam_id,
                     EuclideanTransform& inout_et,
                     ARGBImage& argb,
                     const bool feedback) noexcept;

// Use dense methods to estimate 'q' and 't' for bcam
real refine_bcam_qt_old(const ArucoCube& ac,
                        const vector<ArucoCube::FaceDetection>& detects0,
                        const vector<ArucoCube::FaceDetection>& detects1,
                        const array<cv::Mat, 6>& face_ims,
                        const Matrix3r& K,
                        const bool do_refine_bcam, // as opposed to et0
                        EuclideanTransform& inout_et0,
                        EuclideanTransform& inout_et1,
                        BinocularCameraInfo& inout_bcam_info,
                        const ARGBImage& in_argb0,
                        const ARGBImage& in_argb1,
                        const string& outdir,
                        const bool feedback) noexcept;

// Use dense methods to estimate 'q' and 't' for bcam
real refine_bcam_qt(const PhasePositionOptDetails& opt_details,
                    const bool do_refine_bcam, // as opposed to et0
                    const bool K_opt,          // slow
                    BinocularCameraInfo& inout_bcam_info,
                    EuclideanTransform& out_et0,
                    const unsigned et_result_index,
                    const string& outdir,
                    const bool feedback) noexcept;

// // Given
// // @param et, The euclidean transform for world=>cam 3D coords
// // @param ac, An aruco cube
// // @param detects, A set of 3 ArucoCube::FaceDetection objects
// // @param p3s, A set of 3 planes that for those detects
// // @param offsets, A set of 3 offsets for those detects
// // @param face_im, Face images generated from ac.draw_face(...)
// // @return One AcucoFaceProjectionInfo for each input detection.
// struct ArucoFaceProjectionInfo
// {
//    ArucoCube::face_e f_ind;      // face index
//    uint32_t kolour{0x00000000u}; // 'colour' of this face
//    array<Vector3, 4> quad3d;     // face of image in 3D cam coords
//    Plane p3;                     // the plane that contains quad3d
//    Matrix3r H; // Homography, 3D cam point => 2d face image coord
// };

using ArucoFaceProjectionInfo = ArucoCube::FaceProjectionInfo;

ArucoFaceProjectionInfo
calc_aruco_face_homography(const EuclideanTransform& et,
                           const ArucoCube& ac,
                           const ArucoCube::face_e f_ind,
                           const array<cv::Mat, 6>& face_ims);

vector<ArucoFaceProjectionInfo> calc_aruco_face_coords_and_homography(
    const EuclideanTransform& et,
    const ArucoCube& ac,
    const vector<ArucoCube::FaceDetection>& detects,
    const array<cv::Mat, 6>& face_ims);

void project_aruco_face(
    const ArucoFaceProjectionInfo& pinfo,
    const Matrix3r& K,
    const Matrix3r& K_inv,
    const cv::Mat& face_im,
    std::function<void(int x, int y, uint32_t k)> f) noexcept;

// For working directly in LAB space
void project_aruco_face(const ArucoCube::FaceProjectionInfo& pinfo,
                        const CachingUndistortInverse& cu,
                        const LABImage& face_im,
                        std::function<void(int x, int y, LAB lab)> f) noexcept;

void render_aruco_cube(ARGBImage& argb,
                       const vector<ArucoFaceProjectionInfo>& pinfos,
                       const Matrix3r& K,
                       const Matrix3r& K_inv,
                       const vector<ArucoCube::FaceDetection>& detects,
                       const vector<Plane>& p3s,
                       const array<cv::Mat, 6>& face_ims) noexcept;

// The 'at' vector is computed as looking at the center of the cube
// The 'axes_len' is for drawing the XYZ axes onto the image.
ARGBImage render_aruco_cube_helicopter(const Vector3& cam_pos,
                                       const unsigned w,
                                       const unsigned h,
                                       const real hfov,
                                       const ArucoCube& ac,
                                       const array<cv::Mat, 6>& face_ims,
                                       const real axes_len = 0.0) noexcept;

} // namespace perceive::calibration
