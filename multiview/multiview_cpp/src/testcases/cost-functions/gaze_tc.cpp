
// #include <algorithm>
// #include <iterator>

// #define CATCH_CONFIG_PREFIX_ALL
// #include "perceive/contrib/catch.hpp"
// #include "perceive/cost-functions/openpose/op-helpers.hpp"
// #include "perceive/cost-functions/pose-skeleton/pose-skeleton-exec.hpp"
// #include "perceive/geometry/projective/binocular-camera.hpp"
// #include "perceive/geometry/projective/distorted-camera.hpp"
// #include "perceive/io/perceive-assets.hpp"

// namespace perceive
// {
// static const char* pose_l_0 = R"V0G0N({
//    "score":      0.805464,
//    "keypoints": [[0, 547.109, 80.8742, 0.95811],
//         [1, 559.976, 108.382, 0.937476],
//         [2, 534.454, 108.272, 0.904699],
//         [3, 530.642, 137.71, 0.918236],
//         [4, 528.859, 166.948, 0.888107],
//         [5, 580.051, 110.203, 0.921801],
//         [6, 589.219, 152.312, 0.909375],
//         [7, 569.151, 177.834, 0.829924],
//         [8, 548.934, 177.823, 0.863602],
//         [9, 534.461, 175.994, 0.845067],
//         [10, 538.017, 223.521, 0.829268],
//         [11, 534.385, 256.555, 0.872859],
//         [12, 561.741, 179.613, 0.840713],
//         [13, 556.303, 225.395, 0.819504],
//         [14, 550.803, 269.249, 0.878438],
//         [15, 545.315, 77.2213, 0.815969],
//         [16, 549.077, 77.273, 0.928792],
//         [17, 0, 0, 0],
//         [18, 565.425, 79.0637, 0.953012],
//         [19, 545.338, 280.239, 0.756787],
//         [20, 549.039, 282.012, 0.755392],
//         [21, 549.077, 271.119, 0.750171],
//         [22, 530.698, 269.293, 0.661279],
//         [23, 525.253, 267.405, 0.63291],
//         [24, 536.212, 261.96, 0.665099]]
// })V0G0N";

// static const char* pose_l_1 = R"V0G0N({
//    "score":      0.681074,
//    "keypoints": [[0, 708.149, 194.229, 0.879602],
//         [1, 735.486, 227.285, 0.824499],
//         [2, 722.64, 210.732, 0.736405],
//         [3, 697.162, 243.705, 0.585829],
//         [4, 667.903, 282.036, 0.808391],
//         [5, 746.49, 249.126, 0.734726],
//         [6, 719.002, 314.962, 0.856397],
//         [7, 665.958, 315.055, 0.900488],
//         [8, 686.114, 327.715, 0.623551],
//         [9, 677.006, 314.998, 0.590771],
//         [10, 664.199, 377.163, 0.57151],
//         [11, 678.779, 426.512, 0.652651],
//         [12, 695.282, 335.105, 0.674602],
//         [13, 667.85, 388.168, 0.75332],
//         [14, 649.548, 446.693, 0.866],
//         [15, 709.979, 181.581, 0.37422],
//         [16, 720.889, 186.96, 0.861288],
//         [17, 0, 0, 0],
//         [18, 751.904, 196.134, 0.983879],
//         [19, 620.221, 457.588, 0.847683],
//         [20, 627.594, 461.236, 0.870573],
//         [21, 651.411, 459.36, 0.784551],
//         [22, 638.601, 404.578, 0.365737],
//         [23, 640.443, 402.789, 0.342655],
//         [24, 691.631, 430.213, 0.537519]]
// })V0G0N";

// static const char* pose_r_0 = R"V0G0N({
//    "score":      0.787046,
//    "keypoints": [[0, 528.888, 79.0595, 0.909267],
//         [1, 543.514, 108.285, 0.901248],
//         [2, 519.733, 106.485, 0.909504],
//         [3, 516.046, 137.642, 0.899197],
//         [4, 516.058, 166.81, 0.879752],
//         [5, 563.638, 108.425, 0.948545],
//         [6, 574.594, 152.162, 0.902013],
//         [7, 554.48, 176.025, 0.82977],
//         [8, 534.346, 176.024, 0.853855],
//         [9, 521.55, 174.182, 0.832334],
//         [10, 527.066, 221.721, 0.81249],
//         [11, 521.628, 258.291, 0.805454],
//         [12, 547.164, 177.839, 0.834313],
//         [13, 545.344, 223.557, 0.81724],
//         [14, 541.653, 269.243, 0.815176],
//         [15, 525.233, 70.0109, 0.807909],
//         [16, 534.317, 71.7784, 0.919026],
//         [17, 0, 0, 0],
//         [18, 549.02, 75.4636, 0.919382],
//         [19, 532.538, 280.245, 0.781196],
//         [20, 536.248, 282.007, 0.734673],
//         [21, 539.857, 271.125, 0.693252],
//         [22, 517.901, 271.029, 0.637536],
//         [23, 516.023, 267.463, 0.618752],
//         [24, 527.068, 263.808, 0.614257]]
// })V0G0N";

// static const char* pose_r_1 = R"V0G0N({
//    "score":      0.679079,
//    "keypoints": [[0, 676.914, 194.326, 0.887468],
//         [1, 704.393, 234.534, 0.746574],
//         [2, 695.26, 214.44, 0.677804],
//         [3, 673.356, 254.637, 0.469531],
//         [4, 647.748, 283.856, 0.843572],
//         [5, 709.827, 252.787, 0.781914],
//         [6, 680.605, 316.815, 0.872308],
//         [7, 636.691, 316.784, 0.880207],
//         [8, 666, 322.291, 0.602001],
//         [9, 653.207, 311.301, 0.586672],
//         [10, 640.434, 369.806, 0.464237],
//         [11, 640.444, 408.237, 0.555751],
//         [12, 678.789, 331.479, 0.614896],
//         [13, 647.668, 386.284, 0.750193],
//         [14, 629.438, 446.619, 0.819599],
//         [15, 678.733, 183.307, 0.461593],
//         [16, 682.542, 187.02, 0.84329],
//         [17, 0, 0, 0],
//         [18, 713.586, 196.22, 0.922874],
//         [19, 592.949, 457.599, 0.838566],
//         [20, 605.642, 461.282, 0.879132],
//         [21, 634.895, 457.55, 0.719901],
//         [22, 616.676, 406.422, 0.685123],
//         [23, 620.275, 404.557, 0.713989],
//         [24, 662.348, 419.201, 0.35977]]
// })V0G0N";

// // Should be:
// // render_z(Vector3(0.35, -2.67, 0.0), to_radians(80));
// // render_z(Vector3(1.85, -2.27, 0.0), to_radians(120));

// using Pose = PoseSkeletonExec::Pose;

// static Vector3 recover_gaze_vector(const DistortedCamera& dcam,
//                                    const Vector2& u,
//                                    const Vector2& v) noexcept
// {
//    // Get the plane through the two image points
//    const auto ray_u = backproject_ray(dcam, u);
//    const auto ray_v = backproject_ray(dcam, v);
//    const auto U     = dcam.C + ray_u;
//    const auto V     = dcam.C + ray_v;
//    const auto p3    = Plane(dcam.C, U, V);

//    const auto Z = Vector3{0.0, 0.0, 1.0};          // the up vector
//    const auto Y = cross(Z, p3.xyz()).normalised(); // the side vector
//    const auto X = cross(Y, Z);                     // the forward vector
//    Expects((Z - cross(X, Y)).norm() < 1e-7);       // I got the right,
//    right??

//    // We now have a 3d basis, but we may need to flip the X and Y axis
//    // 'u' should be the "right" point, and 'v' the "left"
//    //
//    // So, let's assume that 'u' is 1 meter away. i.e., U
//    const auto bilateral_p3 = Plane(Y, -dot(Y, U));        //
//    const auto F = (bilateral_p3.side(V) >= 0.0) ? X : -X; // forward vector

//    const auto theta = atan2(F.y, F.x);
//    INFO(format("{:s} => theta = {}", str(F), to_degrees(theta)));

//    return F;
// }

// //
// -----------------------------------------------------------------------------

// CATCH_TEST_CASE("Gaze", "[gaze]")
// {
//    // Load the poses
//    array<Pose, 2> l_poses, r_poses;
//    l_poses[0].read(parse_json(pose_l_0));
//    l_poses[1].read(parse_json(pose_l_1));
//    r_poses[0].read(parse_json(pose_r_0));
//    r_poses[1].read(parse_json(pose_r_1));

//    // We need to load the camera that was used

//    EuclideanTransform et0;
//    et0.translation = Vector3{-1.105606, -1.783074, 2.609029};
//    et0.rotation    = Quaternion{0.673285, -0.632994, 0.229106, -0.305805};

//    // INFO(format("et0 = {:s}", str(et0)));

//    BinocularCameraInfo bcam_info;
//    fetch(bcam_info, "C0001019_v2"s);
//    // BinocularCamera bcam;
//    const int w = 892, h = 672;   // width and height of distorted images
//    const int rw = 800, rh = 600; // rectified width and height

//    array<DistortedCamera, 2> dcams;
//    std::tie(dcams[0], dcams[1])
//        = make_distorted_camera_pair(bcam_info, et0, w, h);

//    // Matrix3r rect_K = Matrix3r::Identity();
//    // rect_K(0, 0)    = 200;
//    // rect_K(1, 1)    = 200;
//    // rect_K(0, 2)    = 400;
//    // rect_K(1, 2)    = 300;
//    // bcam.init(bcam_info, w, h, rect_K, rw, rh, false);

//    //
//    // ------------------------------------------------------------------
//    gaze-io
//    //
//    CATCH_SECTION("gaze-io")
//    {
//       auto test_it = [&](const char* s) {
//          Pose p;
//          p.read(parse_json(s));
//          const string ss = p.to_json_str();
//          CATCH_REQUIRE(trim_copy(ss) == trim_copy(s));
//       };

//       test_it(pose_l_0);
//       test_it(pose_l_1);
//       test_it(pose_r_0);
//       test_it(pose_r_1);
//    }

//    //
//    // ------------------------------------------------------- backproject
//    (dcam)
//    //
//    CATCH_SECTION("dcam-backproject")
//    {
//       auto test_it = [&](const DistortedCamera& dcam, const Vector2 x) {
//          const auto ray = backproject_ray(dcam, x);
//          const auto x1  = project_to_distorted(dcam, dcam.C + ray);
//          CATCH_REQUIRE((x - x1).norm() < 1e-4);
//       };

//       for(const auto& kp : l_poses[0].keypoints)
//          test_it(dcams[0], to_vec2(kp.xy()));
//    }

//    //
//    // ----------------------------------------------------
//    p3_from_two_distorted
//    //
//    CATCH_SECTION("dcam-p3-two-distorted")
//    {
//       auto test_it =
//           [&](const DistortedCamera& dcam, const Vector2& u, const Vector2&
//           v) {
//              const auto p3 = p3_from_two_distorted(dcam, u, v);
//              CATCH_REQUIRE(std::fabs(p3.side(dcam.C)) < 1e-6);
//              CATCH_REQUIRE(std::fabs(p3.side(dcam.C + backproject_ray(dcam,
//              u)))
//                            < 1e-6);
//              CATCH_REQUIRE(std::fabs(p3.side(dcam.C + backproject_ray(dcam,
//              v)))
//                            < 1e-6);
//           };

//       for(auto i = 0u; i < l_poses[0].keypoints.size(); ++i)
//          for(auto j = i + 1; j < l_poses[0].keypoints.size(); ++j)
//             test_it(dcams[0],
//                     to_vec2(l_poses[0].keypoints[i].xy()),
//                     to_vec2(l_poses[0].keypoints[j].xy()));
//    }

//    //
//    // ------------------------------------------------------------------
//    gaze-io
//    //
//    CATCH_SECTION("gaze-3d")
//    {
//       auto test_it = [&](int dcam_ind, int pose_ind) {
//          recover_gaze_vector(dcams[dcam_ind],
//                              to_vec2(l_poses[pose_ind].keypoints[2].xy()),
//                              to_vec2(l_poses[pose_ind].keypoints[5].xy()));
//          recover_gaze_vector(dcams[dcam_ind],
//                              to_vec2(l_poses[pose_ind].keypoints[9].xy()),
//                              to_vec2(l_poses[pose_ind].keypoints[12].xy()));
//       };

//       test_it(0, 0);
//    }
// }

// } // namespace perceive
