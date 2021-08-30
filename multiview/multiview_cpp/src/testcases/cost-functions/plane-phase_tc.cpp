

// #include <algorithm>
// #include <iterator>

// #define CATCH_CONFIG_PREFIX_ALL

// #include "perceive/contrib/catch.hpp"
// #include "perceive/calibration/plane-set/cps-opts.hpp"
// #include "perceive/geometry/projective/binocular-camera.hpp"
// #include "perceive/geometry/projective/distorted-camera.hpp"
// #include "perceive/geometry/rotation.hpp"
// #include "perceive/graphics/bresenham.hpp"
// #include "perceive/graphics/colour-set.hpp"
// #include "perceive/optimization/golden-section-search.hpp"
// #include "perceive/optimization/levenberg-marquardt.hpp"
// #include "perceive/optimization/nelder-mead.hpp"
// #include "perceive/utils/string-utils.hpp"

// namespace perceive
// {
// using namespace perceive::calibration;

// static real k_regularize(const Vector3& C, const Vector3& C0) noexcept
// {
//    return 0.1 * ((C - C0).norm() + 30 * std::fabs(C.z - C0.z));
// }

// // static Quaternion look_at(const Vector3& C, const Vecto

// using Line2D = array<Vector2, 2>;
// struct ImageLines
// {
//    // Line [x, u] and [x, v] are perpendicular
//    // [x, u] is on the two planes [X and U]
//    // [x, v] is on the two planes [X and V]
//    Vector2 x = Vector2::nan();
//    Vector2 u = Vector2::nan();
//    Vector2 v = Vector2::nan();

//    string to_string() const noexcept
//    {
//       return format("x = {:s}, u = {:s}, v= {:s}", str(x), str(u), str(v));
//    }
// };

// struct LineData
// {
//    Plane X = Plane::nan();
//    Plane U = Plane::nan();
//    Plane V = Plane::nan();
//    array<ImageLines, 2> ll; // left image and right image
//    real error(const array<DistortedCamera, 2>& dcam) const noexcept;

//    string to_string() const noexcept
//    {
//       return format("X = {:s}, U = {:s}, V = {:s}\nll[0] = {:s}\nll[1] = {:s}\n",
//                     str(X),
//                     str(U),
//                     str(V),
//                     ll[0].to_string(),
//                     ll[1].to_string());
//    }
// };

// struct RunData
// {
//    string camera_id;
//    Vector2 working_format;
//    EuclideanTransform et0;
//    Vector3 C0 = Vector3::nan();
//    vector<LineData> dat;

//    real error(const array<DistortedCamera, 2>& dcam) const noexcept;
// };

// static real dcam_dist(const DistortedCamera& dcam,
//                       const Vector2& x,
//                       const Plane& A,
//                       const Plane& B)
// {
//    Vector3 a = plane_ray_intersect(dcam, A, x);
//    Vector3 b = plane_ray_intersect(dcam, B, x);
//    // INFO(format("|{:s} - {:s}| = {}", str(a), str(b), (a - b).norm()));
//    return (a - b).norm();
// }

// real LineData::error(const array<DistortedCamera, 2>& dcam) const
// noexcept
// {
//    const auto& dat = *this;
//    auto proc_line  = [&](int i, auto& u, auto& x, auto& U, auto& X) {
//       real lerr    = 0.0;
//       int lcounter = 0;
//       bresenham(u, x, [&](int x, int y) {
//          lerr += dcam_dist(dcam[i], Vector2(x, y), U, X);
//          lcounter++;
//       });
//       return lerr / real(lcounter);
//    };

//    auto err    = 0.0;
//    int counter = 0;

//    for(auto i = 0; i < 2; ++i) {
//       err += dcam_dist(dcam[i], dat.ll[i].x, dat.X, dat.U);
//       if(dat.V.is_finite()) {
//          err += dcam_dist(dcam[i], dat.ll[i].x, dat.U, dat.V);
//          err += dcam_dist(dcam[i], dat.ll[i].x, dat.X, dat.V);
//       }

//       if(false) {
//          if(dat.V.is_finite()) {
//             err += 3.0 * proc_line(i, dat.ll[i].u, dat.ll[i].x, dat.U,
//             dat.X); err += 3.0 * proc_line(i, dat.ll[i].v, dat.ll[i].x,
//             dat.V, dat.X);
//          } else {
//             err += 1.0 * proc_line(i, dat.ll[i].u, dat.ll[i].x, dat.U,
//             dat.X);
//          }
//       } else {
//          if(dat.V.is_finite()) {
//             err += 3.0 * dcam_dist(dcam[i], dat.ll[i].u, dat.X, dat.U);
//             err += 3.0 * dcam_dist(dcam[i], dat.ll[i].v, dat.X, dat.V);
//          } else {
//             err += 1.0 * dcam_dist(dcam[i], dat.ll[i].u, dat.X, dat.U);
//          }
//       }

//       counter += (dat.V.is_finite()) ? 9 : 3;
//    }

//    if(!std::isfinite(err)) { cout << dat.to_string() << endl; }
//    Expects(std::isfinite(err));

//    return err / real(counter);
// } // namespace perceive

// real RunData::error(const array<DistortedCamera, 2>& dcam) const
// noexcept
// {
//    real err = 0.0;
//    for(const auto& ld : dat) err += ld.error(dcam);
//    err = (dat.size() == 0) ? 0.0 : (err / real(dat.size()));

//    if(C0.is_finite()) err += k_regularize(dcam[0].C, C0);

//    return err;
// }

// // static real run_error(const array<DistortedCamera, 2>& dcam,
// //                       const RunData& dat)
// // {
// //    auto proc_line = [&](int i, auto& u, auto& x, auto& U, auto& X) {
// //       real lerr    = 0.0;
// //       int lcounter = 0;
// //       bresenham(u, x, [&](int x, int y) {
// //          lerr += dcam_dist(dcam[i], Vector2(x, y), U, X);
// //          lcounter++;
// //       });
// //       return lerr / real(lcounter);
// //    };

// //    auto err    = 0.0;
// //    int counter = 0;
// //    for(auto i = 0; i < 2; ++i) {
// //       err += dcam_dist(dcam[i], dat.ll[i].x, dat.X, dat.U);
// //       err += dcam_dist(dcam[i], dat.ll[i].x, dat.X, dat.V);
// //       err += dcam_dist(dcam[i], dat.ll[i].x, dat.U, dat.V);

// //       if(true) {
// //          err += 3.0 * proc_line(i, dat.ll[i].u, dat.ll[i].x, dat.U,
// dat.X);
// //          err += 3.0 * proc_line(i, dat.ll[i].v, dat.ll[i].x, dat.V,
// dat.X);
// //       } else {
// //          err += 3.0 * dcam_dist(dcam[i], dat.ll[i].u, dat.X, dat.U);
// //          err += 3.0 * dcam_dist(dcam[i], dat.ll[i].v, dat.X, dat.V);
// //       }

// //       counter += 9;
// //    }
// //    return err / real(counter);
// // }

// static real run_opt(const RunData& dat,
//                     const BinocularCameraInfo& bcam_info,
//                     array<DistortedCamera, 2>& dcam,
//                     EuclideanTransform& inout_et,
//                     const bool use_nelder_mead,
//                     const bool feedback)
// {
//    EuclideanTransform et0 = inout_et;

//    auto pack = [&](real* X) {
//       Vector3 saa = quaternion_to_saa(et0.rotation);
//       for(auto i = 0; i < 3; ++i) *X++ = et0.translation[i];
//       //*(X - 1) = 4.288785;
//       for(auto i = 0; i < 3; ++i) *X++ = saa[i];
//    };

//    auto unpack = [&](const real* X) {
//       Vector3 saa;
//       for(auto i = 0; i < 3; ++i) et0.translation[i] = *X++;
//       for(auto i = 0; i < 3; ++i) saa[i] = *X++;
//       et0.rotation                   = saa_to_quaternion(saa);
//       std::tie(dcam[0].C, dcam[0].q) = make_camera_extrinsics(et0);
//       // dcam[0].C.z                    = 4.28;
//       // et0            = dcam_Cq_to_euclidean_transform(dcam[0].C, dcam[0].q);
//       // std::tie(dcam[0].C, dcam[0].q) = make_camera_extrinsics(et0);

//       const auto et1                 = bcam_info.make_et1(et0);
//       std::tie(dcam[1].C, dcam[1].q) = make_camera_extrinsics(et1);
//    };

//    const auto n_params = 6;
//    vector<real> xbest(n_params);
//    auto counter  = 0;
//    auto best_err = std::numeric_limits<real>::max();
//    auto fn       = [&](const real* X) {
//       unpack(X);
//       const auto err = dat.error(dcam);
//       if(err < best_err) {
//          best_err = err;
//          pack(&xbest[0]);
//          if(feedback)
//             cout << format("#{:4d}, err={:10.8f}", counter, best_err) << endl;
//          ++counter;
//       }
//       return err;
//    };

//    vector<real> start(n_params);
//    vector<real> xmin(n_params);
//    real ynewlo   = NAN;
//    real ystartlo = NAN;
//    real reqmin   = 1e-7;
//    real diffstep = 0.1;

//    int kcount = 1000; // max interations
//    int icount = 0, numres = 0, ifault = 0;
//    const char* method = nullptr;

//    pack(&start[0]);
//    ystartlo = fn(&start[0]);

//    if(!use_nelder_mead) {
//       method = "levenberg-marquardt";
//       levenberg_marquardt(fn,
//                           n_params,
//                           &start[0],
//                           &xmin[0],
//                           reqmin,
//                           diffstep,
//                           5,
//                           kcount,
//                           icount,
//                           ifault);
//       ynewlo = fn(&xmin[0]);
//    } else {
//       method = "nelder-mead";

//       vector<real> step(n_params);
//       for(auto i = 0; i < n_params; ++i) step[i] = one_degree();
//       nelder_mead(fn,
//                   n_params,
//                   &start[0],
//                   &xmin[0],
//                   ynewlo,
//                   reqmin,
//                   &step[0],
//                   10,
//                   20 * kcount,
//                   icount,
//                   numres,
//                   ifault);
//    }

//    ynewlo   = fn(&xbest[0]);
//    inout_et = et0;

//    if(feedback) {
//       INFO(format(
//           "Feedback positioning %s (%d params)", dat.camera_id, n_params));
//       cout << format("   method:               {:s}", method) << endl;
//       cout << format("   fault-code:           {}", ifault) << endl;
//       auto msg = (use_nelder_mead) ? nelder_mead_fault_str(ifault)
//                                    : levenberg_marquardt_fault_str(ifault);
//       cout << format("   fault-string:         {:s}", msg) << endl;
//       cout << endl;
//       cout << format("   initial-score:        {}", ystartlo) << endl;
//       cout << format("   final-score:          {}", ynewlo) << endl;
//       cout << endl;

//       const auto [C, q] = make_camera_extrinsics(et0);
//       cout << format("    {:s}, C={:s}, q={:s}",
//                      dat.camera_id,
//                      str(C),
//                      q.to_readable_str())
//            << endl;

//       cout << "." << endl << endl;
//    }

//    return ynewlo;
// }

// static void run_it(const SceneDescription& scene_desc, const RunData& dat)
// {
//    const auto cam_ind = scene_desc.find_camera(dat.camera_id);
//    INFO(format("{:s} ind = {}", dat.camera_id, cam_ind));
//    auto bcam_info = scene_desc.bcam_infos[cam_ind];
//    for(auto i = 0; i < 2; ++i)
//       bcam_info.M[i].set_working_format(dat.working_format.x,
//                                         dat.working_format.y);
//    // const auto& et0 = scene_desc.cam_transforms[cam_ind];
//    const auto& et0 = dat.et0;

//    array<DistortedCamera, 2> dcam;
//    std::tie(dcam[0], dcam[1]) = make_distorted_camera_pair(
//        bcam_info, et0, dat.working_format.x, dat.working_format.y);

//    //
//    EuclideanTransform et_opt = et0;
//    real best_err             = std::numeric_limits<real>::max();
//    const auto max_itr        = 10000;
//    for(auto i = 0; i < max_itr; ++i) {
//       const bool last            = i == 0 or i + 1 == max_itr;
//       const bool use_nm          = false; // i % 2 == 0;
//       EuclideanTransform et_opt0 = et_opt;
//       if(i > 0) {
//          auto& et = et_opt0;
//          for(auto j = 0; j < 3; ++j)
//             et.translation[j] = 40.0 * (uniform() - 0.5);
//          Vector3 saa
//              = Vector3{uniform() * M_PI, uniform() * M_PI, uniform() * M_PI};
//          et.rotation = saa_to_quaternion(saa);
//       }

//       const auto err = run_opt(dat, bcam_info, dcam, et_opt0, use_nm, false);

//       if(err < best_err) {
//          best_err         = err;
//          et_opt           = et_opt0;
//          const auto err_c = err - k_regularize(dcam[0].C, dat.C0);
//          cout << format("iteration #{:4d}: {}, {}", i, best_err, err_c) <<
//          endl;
//       }
//    }

//    for(auto i = 0; i < 100; ++i) {
//       const bool last   = (i + 1 == 100);
//       const bool use_nm = (i % 2 == 0);
//       const auto err    = run_opt(dat, bcam_info, dcam, et_opt, use_nm,
//       last); const auto err_c  = err - k_regularize(dcam[0].C, dat.C0); cout
//       << format("iteration #{:4d}: {}, {}", i, err, err_c) << endl;
//    }

//    for(const auto& ld : dat.dat) {
//       for(auto i = 0; i < 2; ++i) {
//          Vector3 x0 = plane_ray_intersect(dcam[i], ld.X, ld.ll[i].x);
//          Vector3 x1 = plane_ray_intersect(dcam[i], ld.U, ld.ll[i].x);
//          Vector3 x2 = plane_ray_intersect(dcam[i], ld.V, ld.ll[i].x);

//          cout << format("x0 = {:s}, x1 = {:s}, x2 = {:s}", str(x0), str(x1),
//          str(x2))
//               << endl;
//       }
//    }
//    // if(false) {
//    //    Vector3 X = bcam_info.solve3d_from_distorted(dat.ll[0].x,
//    dat.ll[1].x);
//    //    cout << format("X-cam = {:s}", str(X)) << endl;
//    //    cout << format("X-wrl = {:s}", str(et_opt.apply(X))) << endl;
//    //    cout << format("X-wrl = {:s}", str(et_opt.inverse().apply(X))) <<
//    endl;
//    // }

//    cout << et_opt.to_json_str() << endl;
//    // cout << et_opt.inverse().to_json_str() << endl;
// }

// //
// -----------------------------------------------------------------------------

// CATCH_TEST_CASE("PlanePhase", "[plane_phase]")
// {
//    const string manifest_filename
//        = format("{:s}/computer-vision/test-data/museum_videos/manifest.json",
//                 perceive_data_dir());

//    shared_ptr<SceneDescription> scene_desc_;
//    auto get_scene_desc = [&]() {
//       if(scene_desc_ == nullptr) {
//          scene_desc_ = make_shared<SceneDescription>();
//          scene_desc_->init_from_file(manifest_filename);
//       }
//       return scene_desc_;
//    };

//    const real cam_z0 = 4.28;
//    const auto Z_axis = Vector3(0, 0, 1);

//    auto make_at = [&](real x, real y) {
//       const real m_p_pix = 0.042109;
//       return Vector3((x - 860.0) * m_p_pix, (470 - y) * m_p_pix, 0.0);
//    };

//    // The planes
//    const auto p3floor     = Plane(0, 0, 1, 0);
//    const auto p3baseboard = Plane(0, 0, 1, -0.10);
//    const auto p3A         = Plane(1, 0, 0, -9.55883);
//    const auto p3B         = Plane(0, 1, 0, -14.06453);
//    const auto p3C         = Plane(0, 1, 0, -13.85398);
//    const auto p3D         = Plane(0, 1, 0, -10.73789);
//    const auto p3E         = Plane(0, 1, 0, -9.81148);
//    const auto p3F         = Plane(0, 1, 0, -5.76898);
//    const auto p3G         = Plane(1, 0, 0, -3.07398);

//    // Quaternion q = look_at(
//    //     Vector3{2.0, 0.0, 2.0}, Vector3{2.0, 2.0, 2.0}, Vector3{0.0,
//    //     0.0, 1.0});
//    // cout << format("q.rotate => {:s}", str(q.rotate(Vector3(0, 0, 1)))) <<
//    endl;
//    // FATAL("kBAM!");

//    //
//    // ------------------------------------------------ plane-phase
//    //
//    CATCH_SECTION("plane-phase_C1022")
//    {
//       auto scene_desc = get_scene_desc();
//       RunData dat;
//       dat.camera_id      = "C0001022_v2"s;
//       dat.working_format = Vector2(896, 672);
//       const Vector3 C    = Vector3(4.362411, 18.252205, cam_z0);
//       const Quaternion q = look_at(C, make_at(983, 139), Z_axis);
//       dat.et0            = dcam_Cq_to_euclidean_transform(C, q);
//       dat.C0             = C;

//       auto make_l1 = [&]() {
//          auto ld = LineData{};
//          ld.X    = p3floor;
//          ld.U    = p3A;
//          ld.V    = p3D;
//          if(false) {
//             ld.ll[0].x = Vector2(334, 190); // From the floor
//             ld.ll[0].u = Vector2(117, 413);
//             ld.ll[0].v = Vector2(552, 211);
//             ld.ll[1].x = Vector2(315, 176);
//             ld.ll[1].u = Vector2(101, 382);
//             ld.ll[1].v = Vector2(573, 208);
//          } else {
//             ld.ll[0].x = Vector2(328, 180); // From the baseboard
//             ld.ll[0].u = Vector2(79, 430);
//             ld.ll[0].v = Vector2(602, 207);
//             ld.ll[1].x = Vector2(310, 166);
//             ld.ll[1].u = Vector2(45, 414);
//             ld.ll[1].v = Vector2(583, 200);
//          }
//          return ld;
//       };

//       auto make_l2 = [&]() {
//          auto ld = LineData{};
//          ld.X    = p3floor;
//          ld.U    = p3B;
//          if(false) { // From the floor
//             ld.ll[0].x = Vector2(883, 409);
//             ld.ll[0].u = Vector2(588, 359);
//             ld.ll[1].x = Vector2(884, 410);
//             ld.ll[1].u = Vector2(565, 352);
//          } else { // From the baseboard
//             ld.ll[0].x = Vector2(632, 353);
//             ld.ll[0].u = Vector2(862, 391);
//             ld.ll[1].x = Vector2(573, 340);
//             ld.ll[1].u = Vector2(858, 392);
//          }
//          return ld;
//       };

//       dat.dat.push_back(make_l1());
//       dat.dat.push_back(make_l2());

//       run_it(*scene_desc, dat);

//       //
//       CATCH_REQUIRE(true);
//    }

//    CATCH_SECTION("plane-phase_C1023")
//    {
//       auto scene_desc = get_scene_desc();

//       RunData dat;
//       dat.camera_id      = "C0001023_v1"s;
//       dat.working_format = Vector2(896, 672);
//       const Vector3 C    = Vector3(2.273, 13.6418, cam_z0);
//       const Quaternion q = Quaternion(0, 0, 0, 1);
//       dat.et0            = dcam_Cq_to_euclidean_transform(C, q);
//       dat.C0             = C;

//       auto make_l1 = [&]() {
//          auto ld    = LineData{};
//          ld.X       = p3baseboard;
//          ld.U       = p3A;
//          ld.V       = p3D;
//          ld.ll[0].x = Vector2(225, 220);
//          ld.ll[0].u = Vector2(5, 295);
//          ld.ll[0].v = Vector2(416, 373);
//          ld.ll[1].x = Vector2(231, 244);
//          ld.ll[1].u = Vector2(6, 318);
//          ld.ll[1].v = Vector2(417, 398);
//          return ld;
//       };

//       auto make_l2 = [&]() {
//          auto ld    = LineData{};
//          ld.X       = p3baseboard;
//          ld.U       = p3C;
//          ld.ll[0].x = Vector2(105, 549);
//          ld.ll[0].u = Vector2(150, 668);
//          ld.ll[1].x = Vector2(101, 568);
//          ld.ll[1].u = Vector2(134, 668);
//          return ld;
//       };

//       auto make_l3 = [&]() {
//          auto ld    = LineData{};
//          ld.X       = p3baseboard;
//          ld.U       = p3F;
//          ld.ll[0].x = Vector2(616, 211);
//          ld.ll[0].u = Vector2(892, 312);
//          ld.ll[1].x = Vector2(604, 229);
//          ld.ll[1].u = Vector2(889, 333);
//          return ld;
//       };

//       auto make_l4 = [&]() {
//          auto ld    = LineData{};
//          ld.X       = p3baseboard;
//          ld.U       = p3G;
//          ld.V       = p3D;
//          ld.ll[0].x = Vector2(500, 441);
//          ld.ll[0].u = Vector2(557, 395);
//          ld.ll[0].v = Vector2(397, 357);
//          ld.ll[1].x = Vector2(498, 465);
//          ld.ll[1].u = Vector2(559, 419);
//          ld.ll[1].v = Vector2(378, 366);
//          return ld;
//       };

//       dat.dat.push_back(make_l1());
//       dat.dat.push_back(make_l2());
//       dat.dat.push_back(make_l3());
//       dat.dat.push_back(make_l4());

//       run_it(*scene_desc, dat);

//       //
//       CATCH_REQUIRE(true);
//    }
// }

// } // namespace perceive
