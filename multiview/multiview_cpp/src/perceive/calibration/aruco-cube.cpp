
#include "aruco-cube.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include "perceive/geometry/fitting-planes.hpp"
#include "perceive/geometry/polygon.hpp"
#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/cv-helpers.hpp"
#include "perceive/graphics/tiny-string.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"
#include "perceive/utils/opencv-helpers.hpp"

#define This ArucoCube

namespace perceive
{
static const array<uint32_t, ArucoCube::k_n_faces> k_face_kolour = {
    k_crimson,       // FRONT
    k_dark_red,      // BACK
    k_royal_blue,    // EAST
    k_midnight_blue, // WEST
    k_chocolate      // TOP
};

// -------------------------------------------------------------- calc-detect-Ws

static vector<vector<array<Vector3, 4>>>
calc_detect_Ws(const ArucoCube& ac,
               const vector<ArucoCube::FaceDetection>& detects)
{
   vector<vector<array<Vector3, 4>>> detect_Ws(detects.size());
   {
      for(size_t i = 0; i < detects.size(); ++i) {
         const auto& detect    = detects[i];
         const auto& face_spec = ac.measures[size_t(detect.f_ind)];
         auto& Ws              = detect_Ws[i];
         Ws.resize(detect.quads.size());
         Expects(Ws.size() == detect.marker_ids.size());
         for(size_t j = 0; j < Ws.size(); ++j) {
            const int marker_id = detect.marker_ids[j];
            const int m_pos     = face_spec.marker_pos(marker_id);
            Expects(m_pos >= 0 and m_pos < 4);
            Ws[j] = face_spec.marker_3d_quad(m_pos);
         }
      }
   }
   return detect_Ws;
}

// ------------------------------------------------------- get marker dictionary

static cv::Ptr<cv::aruco::Dictionary> get_marker_dictionary()
{
   return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
}

// ------------------------------------------------------------------ marker pos

int This::FaceMeasurement::marker_pos(const int m_id) const noexcept
{
   auto ii = std::find(cbegin(marker_ids), cend(marker_ids), m_id);
   if(ii == cend(marker_ids)) return -1;
   return int(std::distance(cbegin(marker_ids), ii));
}

// ---------------------------------------------------- calc marker bounding box

const AABB& This::FaceMeasurement::marker_aabb(const int m_ind) const noexcept
{
   Expects(unsigned(m_ind) < marker_aabbs_.size());
   return this->marker_aabbs_[size_t(m_ind)];
}

static array<AABB, 4> make_kyle_marker_aabbs(const Vector2& tl,
                                             const real col,
                                             const real row,
                                             const real marker_size) noexcept
{
   auto make_kyle_aabb = [&](const int m_ind) -> AABB {
      Expects(m_ind >= 0 and m_ind < 4);
      AABB aabb;
      aabb.left   = tl.x + ((m_ind == 0 or m_ind == 2) ? 0.0 : col);
      aabb.right  = aabb.left + marker_size;
      aabb.bottom = tl.y + ((m_ind == 0 or m_ind == 1) ? 0.0 : row);
      aabb.top    = aabb.bottom + marker_size;
      return aabb;
   };

   array<AABB, 4> aabbs;
   int counter = 0;
   std::generate(begin(aabbs), end(aabbs), [&counter, &make_kyle_aabb]() {
      return make_kyle_aabb(counter++);
   });
   return aabbs;
}

// -------------------------------------------------------------- marker 3d quad

array<Vector3, 4>
This::FaceMeasurement::marker_3d_quad(const int m_ind) const noexcept
{
   Expects(m_ind >= 0 and m_ind < 4);
   const auto& M    = *this;
   const AABB& aabb = marker_aabb(m_ind);
   array<Vector3, 4> Ms;
   Ms[0] = Xs[0] + aabb.left * x_axis() - aabb.bottom * y_axis();
   Ms[1] = Ms[0] + marker_size * x_axis();
   Ms[2] = Ms[0] + marker_size * x_axis() - marker_size * y_axis();
   Ms[3] = Ms[0] - marker_size * y_axis();
   return Ms;
}

// ------------------------------------------------- dealing with marker offsets

int This::cv_dict_ind_to_index(int ind) const noexcept
{
   auto ii = marker_lookup.find(ind);
   if(ii == cend(marker_lookup)) {
      return -1;
      if(false) {
         LOG_ERR(format("logic error: could not find cv-index '{}'", ind));
         cout << format("markers = [{}]",
                        implode(begin(cv_dict_ind), end(cv_dict_ind), ", "))
              << endl;
         cout << format("lookup = [{}]",
                        implode(cbegin(marker_lookup),
                                cend(marker_lookup),
                                ", ",
                                [](auto& o) {
                                   return format("({}, {})", o.first, o.second);
                                }))
              << endl;

         FATAL("kBAM!");
      }
   }
   return ii->second;
}

int This::index_to_cv_dict_ind(int marker_ind) const noexcept
{
   Expects(unsigned(marker_ind) < cv_dict_ind.size());
   return cv_dict_ind[size_t(marker_ind)];
}

// ------------------------------------------------------------------- draw face

cv::Mat This::draw_face(face_e f_ind,
                        unsigned width,
                        bool print_coords,
                        bool render_labels) const noexcept
{
   const auto& msm         = measures[size_t(f_ind)];
   const real mw           = this->width(f_ind);
   const real mh           = this->height(f_ind);
   const real pixels_per_m = real(width) / mw;
   const int w             = int(width);
   const int h             = int(std::round(mh * pixels_per_m));

   if(f_ind == ArucoCube::BOTTOM) {
      ARGBImage argb;
      argb.resize(width, width);
      argb.fill(k_black);
      return argb_to_cv(argb);
   }

   ARGBImage argb;
   argb.resize(width, unsigned(h));
   argb.fill(k_white);

   // Draw the markers...
   auto draw_marker = [&](const int m_pos) {
      const auto& in_aabb = measures[size_t(f_ind)].marker_aabb(m_pos);
      const cv::Mat& marker
          = markers[size_t(measures[size_t(f_ind)].marker_ids[size_t(m_pos)])];

      // Convert 'aabb' into raster order (i.e., flip y)
      AABB aabb = in_aabb.reflect_raster();

      const int iw = int(std::round(aabb.width() * pixels_per_m));
      const int ih = int(std::round(aabb.height() * pixels_per_m));

      auto set_map = [&](const bool is_x) {
         cv::Mat map(ih, iw, CV_32FC1);
         for(int y{0}; y < ih; ++y) {
            float* row = map.ptr<float>(y);
            for(int x{0}; x < iw; ++x) {
               if(is_x) {
                  *row++ = float(int(marker.cols * x / real(iw)));
               } else {
                  *row++ = float(int(marker.rows * y / real(ih)));
               }
            }
         }
         return map;
      };
      cv::Mat xmap = set_map(true);
      cv::Mat ymap = set_map(false);

      cv::Mat m;
      cv::remap(marker, m, xmap, ymap, cv::INTER_CUBIC);

      const Vector2 tl = (aabb.top_left()) * pixels_per_m;

      ARGBImage n = cv_to_argb(m);
      for(unsigned y{0}; y < n.height; ++y) {
         for(unsigned x{0}; x < n.width; ++x) {
            set(argb,
                int(std::round(x + tl.x)),
                int(std::round(y + tl.y)),
                n(x, y));
         }
      }

      if(print_coords) {
         const auto Ms = msm.marker_3d_quad(m_pos);
         const auto sz = msm.marker_size * pixels_per_m;

         auto render_it = [&](Vector3 X, string s, Vector2 x, real sz, int j) {
            Point2 offset;
            const Point2 xy = render_tiny_dimensions(s.c_str());
            if(j == 0) x += Vector2(0.0, 0.0);
            if(j == 1) x += Vector2(sz, 0.0);
            if(j == 2) x += Vector2(sz, sz);
            if(j == 3) x += Vector2(0.0, sz);
            if(j == 0) offset = Point2(10, 7);
            if(j == 1) offset = Point2(-xy.x - 5, 7);
            if(j == 2) offset = Point2(-xy.x - 5, -xy.y - 5);
            if(j == 3) offset = Point2(10, -xy.y - 5);
            Point2 p = to_pt2(x * pixels_per_m);
            draw_cross(argb, p, k_red, 4);
            render_string(argb, s, p + offset, k_yellow, k_black);
         };

         for(auto j = 0u; j < Ms.size(); ++j) {
            const auto& X = Ms[j]; // The 3D coord
            const auto s  = format("{:5.3f}, {:5.3f}, {:5.3f}", X.x, X.y, X.z);
            Vector2 pos2d = aabb.top_left();
            render_it(X, s, pos2d, msm.marker_size, int(j));
         }

         for(auto j = 0u; j < 4; ++j) {
            const auto& X = msm.Xs[j];
            const auto s  = format("{:5.3f}, {:5.3f}, {:5.3f}", X.x, X.y, X.z);
            render_it(X, s, Vector2(0, 0), msm.width(), int(j));
         }
      }
   };

   const bool draw_markers = (f_ind != ArucoCube::BOTTOM);
   if(draw_markers)
      for(auto i = 0; i < 4; ++i) draw_marker(i);

   cv::Mat ret = argb_to_cv(argb);

   auto put_text = [&](const string& text,
                       Vector2 pos,
                       uint32_t kolour,
                       real sz_mult,
                       int thickness) {
      cv::putText(ret,
                  text,
                  cv::Point(int(pos.x), int(pos.y)),
                  cv::FONT_HERSHEY_PLAIN,
                  sz_mult,
                  rgb_to_vec3b(kolour),
                  thickness,
                  cv::LINE_8,
                  false);
   };

   auto get_text_sz = [&](const string& text, real sz_mult, int thickness) {
      int baseline  = 0;
      const auto sz = cv::getTextSize(
          text, cv::FONT_HERSHEY_PLAIN, sz_mult, thickness, &baseline);
      return Vector2(sz.width, sz.height);
   };

   if(render_labels) {
      const real sz_mult  = 2.0;
      const int thickness = 3;
      {
         const string label = face_to_str(f_ind);
         auto sz            = get_text_sz(label, sz_mult, thickness);
         put_text(
             label, Vector2(0.5 * (w - sz.x), 30), k_red, sz_mult, thickness);
      }

      if(draw_markers) {
         for(auto i = 0; i < 4; ++i) {
            const auto aabb
                = measures[size_t(f_ind)].marker_aabb(i).reflect_raster();
            const auto iw = aabb.width() * pixels_per_m;
            const auto tl = aabb.top_left() * pixels_per_m;
            const auto marker_id
                = index_to_cv_dict_ind(msm.marker_ids[size_t(i)]);
            const auto label = format("{}", marker_id);
            auto sz          = get_text_sz(label, sz_mult, thickness);
            const auto pos   = tl + Vector2((iw - sz.x) * 0.5, 37);
            put_text(label, pos, k_yellow, sz_mult, thickness);
         }
      }
   }

   return ret;
}

// This face is visible, when seen from camera-centre 'C'
bool This::face_is_visible(const FaceMeasurement& M,
                           const Vector3& C) const noexcept
{
   auto q3d_mid
       = [&](const auto& Q) { return 0.25 * (Q[0] + Q[1] + Q[2] + Q[3]); };

   array<Vector3, 5> Xs;
   Xs[0] = q3d_mid(M.Xs);
   for(size_t i = 0; i < 4; ++i) Xs[i + 1] = q3d_mid(M.marker_3d_quad(int(i)));

   return ranges::all_of(measures, [&](const FaceMeasurement& O) {
      return (O.face == M.face) || ranges::none_of(Xs, [&](const auto& X) {
                const auto& Q = O.Xs;
                return line_segment_intersects_quad3d(
                    Q[0], Q[1], Q[2], Q[3], C, X);
             });
   });
}

// --------------------------------------------------- calc-face-projection-info
//
This::FaceProjectionInfo
This::calc_face_projection_info(const EuclideanTransform& et,
                                const ArucoCube::face_e f_ind,
                                const int width,
                                const int height) const
{
   const auto& ac = *this;
   FaceProjectionInfo pinfo;
   const auto& face_spec = ac.measures[size_t(f_ind)];
   pinfo.f_ind           = f_ind;
   pinfo.kolour          = face_spec.kolour;

   std::transform(cbegin(face_spec.Xs),
                  cend(face_spec.Xs),
                  begin(pinfo.quad3d),
                  [&et](const auto& X) { return et.apply(X); });

   pinfo.p3 = et.apply_to_plane(face_spec.p3);

   {
      // Now calculate the homography that transfers
      // points on 'pinfo.quad3d' to in the images (C[i] = H * W[i])
      vector<Vector3r> Ws(9);
      vector<Vector3r> Cs(9);
      const Vector3& A = pinfo.quad3d[0];
      const Vector3 BA = pinfo.quad3d[1] - A;
      const Vector3 DA = pinfo.quad3d[3] - A;
      size_t pos       = 0;
      for(auto i = 0; i < 3; ++i) {
         for(auto j = 0; j < 3; ++j) {
            const auto dx = real(i) * 0.5; // [0.0, 0.5, 1.0]
            const auto dy = real(j) * 0.5; // [0.0, 0.5, 1.0]
            Ws[pos]       = to_vec3r(A + dx * BA + dy * DA);
            Cs[pos]       = Vector3r(dx * width, dy * height, 1.0);
            ++pos;
         }
      }
      Expects(pos == Ws.size());
      const auto err = calibration::estimate_homography_LLS(Ws, Cs, pinfo.H);
      if(std::fabs(err) > 1e-6) { FATAL(format("err = {}", err)); }
   }

   return pinfo;
}

// --------------------------------------------------------- pointwise-error-fun
//
std::function<real(const EuclideanTransform& et)>
This::pointwise_error_fun(const CachingUndistortInverse& cu,
                          const vector<ArucoCube::FaceDetection>& detects) const
{
   // World coords for 3d quads for detected markers
   vector<vector<array<Vector3, 4>>> detect_Ws_
       = calc_detect_Ws(*this, detects);

   return [&cu, &detects, detect_Ws = std::move(detect_Ws_)](
              const EuclideanTransform& et) -> real {
      auto score_quad = [&](const auto& q2, const auto& W3) {
         Expects(q2.size() == W3.size());
         auto err = 0.0;
         for(auto i = 0u; i < W3.size(); ++i) {
            const Vector2& d = q2[i];
            const Vector3 W  = et.apply(W3[i]);
            const Vector2 w  = cu.distort(homgen_P2_to_R2(W));
            err += (d - w).norm();
         }
         return err / real(W3.size());
      };

      auto err     = 0.0;
      auto n_quads = 0;

      for(auto&& [detect, Ws] : views::zip(detects, detect_Ws)) {
         for(auto&& [Q, W] : views::zip(detect.quads, Ws)) {
            err += score_quad(Q, W);
            n_quads++;
         }
      }

      return err / real(n_quads);
   };
}

// The returned functor references `cu` and `detects`
std::function<real(const EuclideanTransform& et)>
This::dense_error_fun(const CachingUndistortInverse& cu,
                      const LABImage& image_lab,
                      const array<cv::Mat, 6>& face_ims,
                      const vector<ArucoCube::FaceDetection>& detects) const
{
   auto arr_lab_ptr = [&]() -> shared_ptr<const array<LABImage, 6>> {
      auto ptr = make_shared<array<LABImage, 6>>();
      std::transform(
          cbegin(face_ims), cend(face_ims), begin(*ptr), cv_to_LAB_im);
      return ptr;
   };

   const int width  = face_ims[0].cols;
   const int height = face_ims[0].rows;

   return [&ac = *this,
           &cu,
           &image_lab,
           labs_ptr = arr_lab_ptr(),
           &detects,
           width,
           height](const EuclideanTransform& et) -> real {
      real err    = 0.0;
      int counter = 0;

      ranges::for_each(detects, [&](const auto& detect) {
         const auto& face_lab = labs_ptr->operator[](size_t(detect.f_ind));
         const auto ac_proj_info
             = ac.calc_face_projection_info(et, detect.f_ind, width, height);
         project_aruco_face(
             ac_proj_info, cu, [&](Point2 image_xy, Point2 face_xy) {
                const auto lab_in  = in_bounds(image_lab, image_xy);
                const auto face_in = in_bounds(face_lab, face_xy);
                if(lab_in && face_in) {
                   err += real(cie1976_distance(image_lab(image_xy),
                                                face_lab(face_xy)));
                   ++counter;
                }
             });
      });

      return (counter == 0) ? std::numeric_limits<real>::max()
                            : (err / real(counter));
   };
}

// ------------------------------------------------------------ render-pointwise

void This::render_pointwise(ARGBImage& argb,
                            const vector<ArucoCube::FaceDetection>& detects,
                            const CachingUndistortInverse& cu,
                            const EuclideanTransform& cam_et,
                            const EuclideanTransform& cube_et) const
{
   const auto& ac           = *this;
   const Point2 text_offset = Point2(1, 3);
   using Quad3D             = array<Vector3, 4>;

   auto apply_et = [&](const EuclideanTransform& et, const Quad3D& quad) {
      Quad3D o;
      for(size_t i = 0; i < 4; ++i) o[i] = et.apply(quad[i]);
      return o;
   };

   auto project = [&](const Vector3& X) {
      return cu.distort(homgen_P2_to_R2(cam_et.apply(X)));
   };

   { // Let's draw the origin
      const Point2 p = to_pt2(project(Vector3(0, 0, 0)));
      fill_circle(argb, p, k_black, 7);
      fill_circle(argb, p, k_yellow, 5);
      draw_cross(argb, p, k_red, 3);
   }

   auto render_quad = [&](const array<Vector3, 4>& Xs, uint32_t k) {
      for(size_t i = 0; i < 4; ++i) {
         const auto a = project(Xs[i]);
         const auto b = project(Xs[(i + 1) % 4]);
         plot_line_AA(a, b, [&](int x, int y, float a) {
            if(argb.in_bounds(x, y)) argb(x, y) = blend(k, argb(x, y), a);
         });
      }
   };

   { // Draw each visible face
      for(const auto& detect : detects) {
         const auto& M = ac.measures[size_t(detect.f_ind)];
         const auto k  = M.kolour;

         // The bounding quad
         render_quad(apply_et(cube_et, M.Xs), M.kolour);

         // Draw the detections
         for(auto i = 0u; i < detect.quads.size(); ++i) {
            const auto m_id  = detect.marker_ids[i];
            const auto m_pos = M.marker_pos(m_id);
            Expects(m_pos >= 0 and m_pos < 4);
            const auto Ys = M.marker_3d_quad(m_pos);
            render_quad(apply_et(cube_et, Ys), M.kolour);
         }

         // Draw detection origins
         for(auto i = 0u; i < detect.quads.size(); ++i) {
            const auto& q2 = detect.quads[i];
            draw_square(argb, to_pt2(q2[0]), k_red, 4);
         }

         // Label the marker
         for(auto i = 0u; i < detect.quads.size(); ++i) {
            const auto& q2 = detect.quads[i];
            const auto c
                = 0.25 * std::accumulate(cbegin(q2), cend(q2), Vector2{});
            const auto s = format("{}", detect.marker_ids[i]);
            render_string(argb, s, to_pt2(c), k_yellow, k_black);
         }
      }
   }

   { // Label coords of TOP
      const auto& M = ac.measures[ArucoCube::TOP];
      for(size_t i = 0; i < 4; ++i) {
         const auto x = project(cube_et.apply(M.Xs[i]));
         const auto s = format("{}", str(x));
         render_string(argb, s, to_pt2(x), k_yellow, k_black);
      }
   }
}

// ---------------------------------------------------------------- render-dense

void This::render_dense(ARGBImage& argb,
                        const vector<ArucoCube::FaceDetection>& detects,
                        const array<cv::Mat, 6>& face_ims,
                        const CachingUndistortInverse& cu,
                        const EuclideanTransform& cam_et,
                        const EuclideanTransform& cube_et) const
{
   if(argb.size() == 0) return;

   const auto& ac         = *this;
   const auto relative_et = (cube_et.inverse() * cam_et.inverse());
   const auto cam_C       = cam_et.translation;

   auto face_is_visible = [&](const auto& M) -> bool {
      return ac.face_is_visible(M, cube_et.apply(cam_C));
   };

   auto score_pixel = [&](int x, int y, uint32_t k) -> float {
      if(!argb.in_bounds(x, y)) return fNAN;
      return float(cie1976_score(argb(x, y), k));
   };

   { // Draw each visible face
      ranges::for_each(ac.measures, [&](const auto& M) {
         if(!face_is_visible(M)) return;

         const auto ac_proj_info = ac.calc_face_projection_info(
             relative_et, M.face, face_ims[0].cols, face_ims[0].rows);

         project_aruco_face(ac_proj_info,
                            cu,
                            face_ims.at(size_t(M.face)),
                            [&](int x, int y, uint32_t k) {
                               const auto score = score_pixel(x, y, k);
                               if(argb.in_bounds(x, y)) {
                                  Expects(std::isfinite(score));
                                  argb(x, y) = gray_to_rgb(
                                      uint8_t((1.0f - score) * 255.0f));
                               }
                            });
      });
   }
}

// -------------------------------------------------------------- detect markers

vector<ArucoCube::FaceDetection>
This::detect_markers(const cv::Mat& im,
                     const CachingUndistortInverse& cu,
                     const string_view out_filename) const
{
   // https://docs.opencv.org/3.4/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html
   auto parameters = cv::aruco::DetectorParameters::create();
   auto dictionary = get_marker_dictionary();
   std::vector<int> marker_ids;
   std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
   std::vector<cv::Vec3d> rvecs, tvecs; // Pose

   auto to_v2 = [&](const cv::Point2f& x) {
      return to_vec2(Vector2f{x.x, x.y});
   };

   cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_32FC1);

   cv::aruco::detectMarkers(im,
                            dictionary,
                            marker_corners,
                            marker_ids,
                            parameters,
                            rejected_candidates);

   // If we found at least 1 detection, then estimate pose
   if(marker_ids.size() > 0) {
      Expects(std::isfinite(marker_size) and marker_size > 0.0);

      // Convert all detections to normalized-points
      auto norm_marker_corners = marker_corners;
      ranges::for_each(norm_marker_corners, [&](auto& vec_p2f) {
         ranges::for_each(vec_p2f, [&](cv::Point2f& x) {
            const auto o = cu.undistort(to_v2(x));
            x.x          = float(o.x);
            x.y          = float(o.y);
         });
      });

      cv::Mat cvK = to_cv_mat_32f(Matrix3r::Identity());
      cv::aruco::estimatePoseSingleMarkers(norm_marker_corners,
                                           float(marker_size),
                                           cvK,
                                           dist_coeffs,
                                           rvecs,
                                           tvecs);
   }

   if(!out_filename.empty()) { // output detection
      cv::Mat z = im.clone();
      cv::aruco::drawDetectedMarkers(z, marker_corners, marker_ids);
      cv::imwrite(out_filename.data(), z);
   }

   const real s = 0.5 * marker_size;
   const array<Vector3, 4> As{
       {{-s, s, 0.0}, {s, s, 0.0}, {s, -s, 0.0}, {-s, -s, 0.0}}};
   auto to_quad3d = [&](const std::vector<cv::Point2f>& ds,
                        const Quaternion& q,
                        const Vector3& t) {
      Expects(ds.size() == 4);
      array<Vector3, 4> Xs;
      std::transform(cbegin(As), cend(As), begin(Xs), [&](const auto& A) {
         return q.rotate(A) + t;
      });
      return Xs;
   };

   // organize the detections into faces...
   Expects(marker_ids.size() == marker_corners.size());
   vector<FaceDetection> out{k_n_faces}; // six faces
   for(unsigned i{0}; i < out.size(); ++i) out[i].f_ind = face_e(i);

   const auto N = marker_ids.size();
   for(unsigned i{0}; i < N; ++i) {
      int m_id = cv_dict_ind_to_index(marker_ids[i]);
      if(m_id == -1) {
         // DO SOMETHING
         WARN(format("detected marker '{}'; however, this marker isn't "
                     "specified on the aruco-cube!",
                     m_id));
         continue;
      }
      const std::vector<cv::Point2f>& ds = marker_corners[i];
      const auto q = rodrigues_to_quaternion(to_vec3(rvecs[i]));
      const auto t = to_vec3(tvecs[i]);

      Expects(ds.size() == 4);

      // Write results into the FaceDetect object
      auto ii
          = std::find_if(cbegin(measures), cend(measures), [&](const auto& f) {
               return f.marker_pos(m_id) >= 0;
            });
      if(ii == cend(measures)) {
         WARN(format("failed to find face '{}' in ArucoCube structure", m_id));
         continue;
      }
      auto j = std::distance(cbegin(measures), ii);
      Expects(j >= 0 and j < int(out.size()));

      FaceDetection& fout = out[size_t(j)];
      fout.marker_ids.push_back(m_id);
      fout.quads.push_back(
          {to_v2(ds[0]), to_v2(ds[1]), to_v2(ds[2]), to_v2(ds[3])});
      fout.ets.emplace_back(t, q, 1.0);
      fout.quad3ds.emplace_back(to_quad3d(ds, q, t));
      Expects(measures[size_t(fout.f_ind)].marker_pos(m_id) >= 0);
   }

   // Now remove empty detections
   auto ii = std::partition(
       begin(out), end(out), [](auto& x) { return x.quads.size() > 0; });
   out.erase(ii, out.end());

   // Finally, we estimate the initial plane equations from the faces
   for(auto& face : out) {
      vector<Vector3> Xs;
      Xs.reserve(4 * face.quad3ds.size());
      std::for_each(
          cbegin(face.quad3ds), cend(face.quad3ds), [&](const auto& q3) {
             Xs.insert(end(Xs), cbegin(q3), cend(q3));
          });
      face.p3 = fit_plane(&Xs[0], unsigned(Xs.size()));
   }

   return out;
}

vector<ArucoCube::FaceDetection>
This::detect_markers(const cv::Mat& im,
                     const Matrix3r& K,
                     const string_view out_filename) const
{
   // https://docs.opencv.org/3.4/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html
   auto parameters = cv::aruco::DetectorParameters::create();
   auto dictionary = get_marker_dictionary();
   std::vector<int> marker_ids;
   std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
   std::vector<cv::Vec3d> rvecs, tvecs; // Pose

   cv::Mat cvK         = to_cv_mat_32f(K);
   cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_32FC1);

   cv::aruco::detectMarkers(im,
                            dictionary,
                            marker_corners,
                            marker_ids,
                            parameters,
                            rejected_candidates);

   // If we found at least 1 detection, then estimate pose
   if(marker_ids.size() > 0) {
      Expects(std::isfinite(marker_size) and marker_size > 0.0);
      cv::aruco::estimatePoseSingleMarkers(
          marker_corners, float(marker_size), cvK, dist_coeffs, rvecs, tvecs);
   }

   if(!out_filename.empty()) { // output detection
      cv::Mat z = im.clone();
      cv::aruco::drawDetectedMarkers(z, marker_corners, marker_ids);

      const auto axis_len = marker_size * 0.4;
      for(auto i = 0u; i < rvecs.size(); ++i)
         cv::aruco::drawAxis(
             z, cvK, dist_coeffs, rvecs[i], tvecs[i], float(axis_len));

      cv::imwrite(out_filename.data(), z);
   }

   auto to_v2 = [&](const cv::Point2f& x) {
      return to_vec2(Vector2f{x.x, x.y});
   };

   const real s = 0.5 * marker_size;
   const array<Vector3, 4> As{
       {{-s, s, 0.0}, {s, s, 0.0}, {s, -s, 0.0}, {-s, -s, 0.0}}};
   auto to_quad3d = [&](const std::vector<cv::Point2f>& ds,
                        const Quaternion& q,
                        const Vector3& t) {
      Expects(ds.size() == 4);
      array<Vector3, 4> Xs;
      for(auto i = 0u; i < ds.size(); ++i) {
         Xs[i] = q.rotate(As[i]) + t;
         if(false) { // Okay, is this the correct position?
            const auto d = to_v2(ds[i]);
            const auto x = homgen_P2_to_R2(to_vec3(K * to_vec3r(Xs[i])));
            cout << format(
                " <=> -- |{} - {}| = {}", str(d), str(x), (x - d).norm())
                 << endl;
         }
      }
      return Xs;
   };

   // organize the detections into faces...
   Expects(marker_ids.size() == marker_corners.size());
   vector<FaceDetection> out{k_n_faces}; // six faces
   for(unsigned i{0}; i < out.size(); ++i) out[i].f_ind = face_e(i);

   const auto N = marker_ids.size();
   for(unsigned i{0}; i < N; ++i) {
      int m_id = cv_dict_ind_to_index(marker_ids[i]);
      if(m_id == -1) {
         // DO SOMETHING
         WARN(format("detected marker '{}'; however, this marker isn't "
                     "specified on the aruco-cube!",
                     m_id));
         continue;
      }
      const std::vector<cv::Point2f>& ds = marker_corners[i];
      const auto q = rodrigues_to_quaternion(to_vec3(rvecs[i]));
      const auto t = to_vec3(tvecs[i]);

      Expects(ds.size() == 4);

      // Write results into the FaceDetect object
      auto ii
          = std::find_if(cbegin(measures), cend(measures), [&](const auto& f) {
               return f.marker_pos(m_id) >= 0;
            });
      if(ii == cend(measures)) {
         WARN(format("failed to find face '{}' in ArucoCube structure", m_id));
         continue;
      }
      auto j = std::distance(cbegin(measures), ii);
      Expects(j >= 0 and j < int(out.size()));

      FaceDetection& fout = out[size_t(j)];
      fout.marker_ids.push_back(m_id);
      fout.quads.push_back(
          {to_v2(ds[0]), to_v2(ds[1]), to_v2(ds[2]), to_v2(ds[3])});
      fout.ets.emplace_back(t, q, 1.0);
      fout.quad3ds.emplace_back(to_quad3d(ds, q, t));
      Expects(measures[size_t(fout.f_ind)].marker_pos(m_id) >= 0);
   }

   // Now remove empty detections
   auto ii = std::partition(
       begin(out), end(out), [](auto& x) { return x.quads.size() > 0; });
   out.erase(ii, out.end());

   // Finally, we estimate the initial plane equations from the faces
   for(auto& face : out) {
      vector<Vector3> Xs;
      Xs.reserve(4 * face.quad3ds.size());
      std::for_each(
          cbegin(face.quad3ds), cend(face.quad3ds), [&](const auto& q3) {
             Xs.insert(end(Xs), cbegin(q3), cend(q3));
          });
      face.p3 = fit_plane(&Xs[0], unsigned(Xs.size()));
   }

   return out;
}

// ----------------------------------------------------------------- face to str

const char* This::face_to_str(face_e f_ind) noexcept
{
   switch(f_ind) {
   case FRONT: return "FRONT";
   case BACK: return "BACK";
   case EAST: return "EAST";
   case WEST: return "WEST";
   case TOP: return "TOP";
   case BOTTOM: return "BOTTOM";
   }
}

This::face_e This::str_to_face(const string_view ss) noexcept(false)
{
   if(ss == "FRONT") return FRONT;
   if(ss == "BACK") return BACK;
   if(ss == "EAST") return EAST;
   if(ss == "WEST") return WEST;
   if(ss == "TOP") return TOP;
   if(ss == "BOTTOM") return BOTTOM;
   throw std::runtime_error(format("invalid 'face' value: '{}'", ss));
   return TOP;
}

// ------------------------------------------------------ width/height of a face

real This::width(face_e f_ind) const noexcept
{
   return measures[size_t(f_ind)].width();
}

real This::height(face_e f_ind) const noexcept
{
   return measures[size_t(f_ind)].height();
}

// ---------------------------------------------------------------------- center

Vector3 This::center() const noexcept
{
   int counter = 0;
   Vector3 C{0.0, 0.0, 0.0};
   for(const auto& fm : measures) {
      if(fm.face == ArucoCube::BOTTOM) continue;
      for(auto X : fm.Xs) {
         C += X;
         counter++;
      }
   }
   return C / real(counter);
}

// ----------------------------------------------------- FaceDetection to-string

string This::FaceDetection::to_string() const noexcept
{
   std::stringstream ss{""};

   ss << format("Face:       {}", face_to_str(f_ind)) << endl;
   ss << format("# markers:  {}", marker_ids.size()) << endl;
   ss << format("p3:         {}", str(p3)) << endl;
   for(unsigned i{0}; i < marker_ids.size(); ++i) {
      ss << format(" - id={:2d} :: [{}, {}, {}, {}]",
                   marker_ids[i],
                   quads[i][0].to_string("[{}, {}]"),
                   quads[i][1].to_string("[{}, {}]"),
                   quads[i][2].to_string("[{}, {}]"),
                   quads[i][3].to_string("[{}, {}]"))
         << endl;
   }

   return ss.str();
}

// -------------------------------------------------------- make kyle aruco cube

ArucoCube This::make_kyle_aruco_cube() noexcept
{
   ArucoCube ac;

   ac.id = "kyle-cube"s;

   // ac.marker_offset_ = 1;

   std::iota(begin(ac.cv_dict_ind), end(ac.cv_dict_ind), 1);
   for(auto i = 0u; i < ac.cv_dict_ind.size(); ++i)
      ac.marker_lookup[ac.cv_dict_ind[i]] = int(i);

   // Get the markers
   // auto get_marker_dictionary = []() {
   //    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
   // };

   auto dictionary = get_marker_dictionary();
   for(size_t i = 0; i < ac.markers.size(); ++i) {
      dictionary->drawMarker(ac.index_to_cv_dict_ind(int(i)), 7, ac.markers[i]);
      // cv::imwrite(format("/tmp/zappy_{:2d}.png", i), ac.markers[i]);
   }

   // Set up the measurements.
   const real marker_size = 0.2395; // 23.95cm
   ac.marker_size         = marker_size;

   const real h0 = 0.5400;
   const real fr = 0.0220; // The foam runner under the cube
   const real h1 = h0 + fr;
   const real ew = 0.5400, eh = 0.5400;
   const real bw = 0.5400, bh = 0.5400;

   // Face 2 and face 4 (WF) have a foam
   // attachment in the bottom that increases the height.
   const array<Vector3, 4> B // Bottom quad of cube
       = {{{0.0, 0.0, fr}, {eh, 0.0, fr}, {eh, bw, fr}, {0.0, bw, fr}}};
   const array<Vector3, 4> T // Top quad of cube
       = {{{0.0, 0.0, h1}, {eh, 0.0, h1}, {eh, bw, h1}, {0.0, bw, h1}}};

   { // FRONT
      auto& face         = ac.measures[ArucoCube::FRONT];
      const auto o       = Vector3{-0.0130, 0.0, 0.0}; // thickness of face
      face.face          = ArucoCube::FRONT;
      face.Xs            = {{T[3] + o, T[0] + o, B[0] + o, B[3] + o}};
      face.marker_aabbs_ = make_kyle_marker_aabbs(
          Vector2{0.0256, 0.0273}, 0.2495, 0.2500, marker_size);
      // face.tl         = Vector2{0.0256, 0.0273};
      // face.col        = 0.2495;
      // face.row        = 0.2500;
      face.marker_ids = {{12, 13, 14, 15}};
      face.kolour     = k_face_kolour[size_t(face.face)];
   }

   { // BACK
      auto& face         = ac.measures[ArucoCube::BACK];
      const auto o       = Vector3{0.0120, 0.0, 0.0}; // thickness of face
      face.face          = ArucoCube::BACK;
      face.Xs            = {{T[1] + o, T[2] + o, B[2] + o, B[1] + o}};
      face.marker_aabbs_ = make_kyle_marker_aabbs(
          Vector2{0.0254, 0.0305}, 0.2500, 0.2495, marker_size);
      // face.tl         = Vector2{0.0254, 0.0305};
      // face.col        = 0.2500;
      // face.row        = 0.2495;
      face.marker_ids = {{0, 1, 2, 3}};
      face.kolour     = k_face_kolour[size_t(face.face)];
   }

   { // TOP
      auto& face         = ac.measures[ArucoCube::TOP];
      const auto o       = Vector3{0.0, 0.0, 0.0110}; // thickness of face
      face.face          = ArucoCube::TOP;
      face.Xs            = {{T[2] + o, T[1] + o, T[0] + o, T[3] + o}};
      face.marker_aabbs_ = make_kyle_marker_aabbs(
          Vector2{0.0264, 0.0302}, 0.2500, 0.2460, marker_size);
      // face.tl         = Vector2{0.0264, 0.0302};
      // face.col        = 0.2500;
      // face.row        = 0.2460;
      face.marker_ids = {{4, 5, 6, 7}};
      face.kolour     = k_face_kolour[size_t(face.face)];
   }

   { // BOTTOM
      auto& face         = ac.measures[ArucoCube::BOTTOM];
      const auto o       = Vector3{0.0, 0.0, 0.0}; // thickness of face
      face.face          = ArucoCube::BOTTOM;
      face.Xs            = {{B[2] + o, B[1] + o, B[0] + o, B[3] + o}};
      face.marker_aabbs_ = make_kyle_marker_aabbs(
          Vector2{0.01, 0.01}, 0.2600, 0.2600, marker_size);
      // face.tl         = Vector2{0.01, 0.01};
      // face.col        = 0.2600;
      // face.row        = 0.2600;
      face.marker_ids = {{20, 21, 22, 23}};
      face.kolour     = k_face_kolour[size_t(face.face)];
   }

   { // EAST
      auto& face         = ac.measures[ArucoCube::EAST];
      const auto o       = Vector3{0.0, -0.0120, 0.0}; // thickness of face
      face.face          = ArucoCube::EAST;
      face.Xs            = {{T[1] + o, B[1] + o, B[0] + o, T[0] + o}};
      face.marker_aabbs_ = make_kyle_marker_aabbs(
          Vector2{0.0250, 0.0251}, 0.2520, 0.2500, marker_size);
      // face.tl         = Vector2{0.0250, 0.0251};
      // face.col        = 0.2520;
      // face.row        = 0.2500;
      face.marker_ids = {{8, 9, 10, 11}};
      face.kolour     = k_face_kolour[size_t(face.face)];
   }

   { // WEST
      auto& face         = ac.measures[ArucoCube::WEST];
      const auto o       = Vector3{0.0, 0.0130, 0.0}; // thickness of face
      face.face          = ArucoCube::WEST;
      face.Xs            = {{T[2] + o, T[3] + o, B[3] + o, B[2] + o}};
      face.marker_aabbs_ = make_kyle_marker_aabbs(
          Vector2{0.0278, 0.0282}, 0.2485, 0.2465, marker_size);
      // face.tl         = Vector2{0.0278, 0.0282};
      // face.col        = 0.2485;
      // face.row        = 0.2465;
      face.marker_ids = {{16, 17, 18, 19}};
      face.kolour     = k_face_kolour[size_t(face.face)];
   }

   for(auto& face_spec : ac.measures) {
      face_spec.p3 = fit_plane(&face_spec.Xs[0], unsigned(face_spec.Xs.size()));
      face_spec.marker_size = marker_size;
   }

#ifdef DO_NOT_COMPILE
   { // Sanity checks
      bool has_error = false;
      for(auto i = 0; i < 6; ++i) {
         const auto f_ind = ArucoCube::face_e(i);
         const auto& M    = ac.measures[f_ind];

         const auto nw = M.tl.x + M.col + ac.marker_size;
         const auto nh = M.tl.y + M.row + ac.marker_size;

         if(nw > ac.width(f_ind)) {
            LOG_ERR(format(
                "face {} of the 'kyle cube' is {} wide, but that's larger"
                "than the measurements: margin-x({}) + col({}) + "
                "marker({}) = "
                "{}!",
                ArucoCube::face_to_str(f_ind),
                ac.width(f_ind),
                M.tl.x,
                M.col,
                ac.marker_size,
                nw));
            has_error = true;
         }

         if(nh > ac.height(f_ind)) {
            LOG_ERR(format(
                "face {} of the 'kyle cube' is {} high, but that's larger"
                "than the measurements: margin-y({}) + row({}) + "
                "marker({}) = "
                "{}!",
                ArucoCube::face_to_str(f_ind),
                ac.height(f_ind),
                M.tl.y,
                M.row,
                ac.marker_size,
                nh));
            has_error = true;
         }
      }

      if(has_error) FATAL("aborting");
   }
#endif

   return ac;
}

ArucoCube make_kyle_aruco_cube() noexcept
{
   return ArucoCube::make_kyle_aruco_cube();
}

// ------------------------------------------------------------------------- str

string This::to_json_str() const noexcept
{ //
   Expects(std::isfinite(marker_size));

   auto enc_marker = [&](const FaceMeasurement& fm, int m_pos) {
      const auto m   = fm.marker_3d_quad(m_pos)[0];
      const auto a   = fm.marker_aabb(m_pos);
      const auto q3d = fm.marker_3d_quad(m_pos);
      return format(R"V0G0N({{ "marker-id": {}, "xy": [{}, {}] }})V0G0N",
                    index_to_cv_dict_ind(fm.marker_ids[size_t(m_pos)]),
                    a.left,
                    a.top);
   };

   auto enc_measures = [&](const FaceMeasurement& fm) -> string {
      const auto xaxis = fm.x_axis();
      const auto yaxis = fm.y_axis();
      return format(R"V0G0N(
{{
   "face-label":   {},
   "face-x-axis":  [{}, {}, {}],
   "face-y-axis":  [{}, {}, {}],
   "origin":       [{}, {}, {}],
   "width":         {},
   "height":        {},
   "markers": [
      {},
      {},
      {},
      {}
   ]
}})V0G0N",
                    json_encode(string(face_to_str(fm.face))),
                    xaxis[0],
                    xaxis[1],
                    xaxis[2],
                    yaxis[0],
                    yaxis[1],
                    yaxis[2],
                    fm.Xs[0][0],
                    fm.Xs[0][1],
                    fm.Xs[0][2],
                    fm.width(),
                    fm.height(),
                    enc_marker(fm, 0),
                    enc_marker(fm, 1),
                    enc_marker(fm, 2),
                    enc_marker(fm, 3));
   };

   return format(
       R"V0G0N({{
   "id":            {},
   "marker-size":   {},
   "faces":        [{}]
{}}})V0G0N",
       json_encode(id),
       marker_size,
       trim_copy(indent(
           implode(cbegin(measures), cbegin(measures) + 5, ", ", enc_measures),
           6)),
       "");
}

string str(const ArucoCube& acube) noexcept { return acube.to_json_str(); }

// ------------------------------------------------------------- init-from-jsoin

static ArucoCube init_from_json_(const Json::Value& root) noexcept(false)
{
   ArucoCube acube;
   const string op = "reading aruco-cube json"s;

   acube.id          = json_load_key<string>(root, "id", op);
   acube.marker_size = json_load_key<real>(root, "marker-size", op);

   std::vector<int> cv_marker_ids;

   auto finish_marker = [&](ArucoCube::FaceMeasurement& o,
                            int i,
                            const Vector3& x_axis,
                            const Vector3& y_axis,
                            int marker_id,
                            const Vector2& xy) {
      { // Take care of the marker-id
         const auto ii
             = std::find(cbegin(cv_marker_ids), cend(cv_marker_ids), marker_id);
         if(ii != cend(cv_marker_ids))
            throw std::runtime_error(
                format("duplicate marker id = {} while {}", marker_id, op));
         cv_marker_ids.push_back(marker_id);
         o.marker_ids[size_t(i)] = int(cv_marker_ids.size()) - 1;
      }

      const auto sz              = acube.marker_size;
      o.marker_aabbs_[size_t(i)] = AABB(xy.x, xy.y, xy.x + sz, xy.y - sz);
   };

   // Convert 1 json node into a FaceMeasure, tracking the aruco markers needed
   auto read_faces_node
       = [&](const Json::Value node) -> ArucoCube::FaceMeasurement {
      ArucoCube::FaceMeasurement o;
      o.face = ArucoCube::str_to_face(
          json_load_key<string>(node, "face-label", op));
      Vector3 x_axis
          = json_load_key<Vector3>(node, "face-x-axis", op).normalised();
      Vector3 y_axis
          = json_load_key<Vector3>(node, "face-y-axis", op).normalised();

      Vector3 X0   = json_load_key<Vector3>(node, "origin", op);
      const auto w = json_load_key<real>(node, "width", op);
      const auto h = json_load_key<real>(node, "height", op);

      if(!has_key(node, "markers"))
         throw std::runtime_error(
             format("could not find 'markers' key while {}", op));
      auto mk_nodes = get_key(node, "markers");
      if(mk_nodes.type() != Json::arrayValue or mk_nodes.size() != 4)
         throw std::runtime_error(format(
             "expected 'markers' to be an array or size 4 while {}", op));

      for(auto i = 0; i < 4; ++i) {
         const auto marker_id
             = json_load_key<int>(mk_nodes[i], "marker-id", op);
         const auto xy = json_load_key<Vector2>(mk_nodes[i], "xy", op);
         finish_marker(o, i, x_axis, y_axis, marker_id, xy);
      }

      // The 3d bounding quad
      o.Xs[0] = X0;
      o.Xs[1] = X0 + w * x_axis;
      o.Xs[2] = X0 + w * x_axis - h * y_axis;
      o.Xs[3] = X0 - h * y_axis;

      // Finish the face
      o.kolour      = k_face_kolour[size_t(o.face)];
      o.p3          = fit_plane(&o.Xs[0], o.Xs.size());
      o.marker_size = acube.marker_size;

      return o;
   };

   // Read the faces node
   if(!has_key(root, "faces"))
      throw std::runtime_error(
          format("could not find 'faces' key while {}", op));
   auto faces_nodes = get_key(root, "faces");
   if(faces_nodes.type() != Json::arrayValue
      or (faces_nodes.size() != 5 and faces_nodes.size() != 6))
      throw std::runtime_error(
          format("expected 'faces' to be an array or size {} while {}", 5, op));

   // try to copy them into acube.measures
   vector<bool> faces_marks(ArucoCube::k_n_faces);
   std::fill(begin(faces_marks), end(faces_marks), false);
   for(const auto& node : faces_nodes) {
      const auto o   = read_faces_node(node);
      const auto idx = size_t(o.face);
      Expects(unsigned(idx) < acube.measures.size());
      if(faces_marks[idx]) {
         throw std::runtime_error(format("duplicate {} face found while {}",
                                         ArucoCube::face_to_str(o.face),
                                         op));
      }
      faces_marks[idx]    = true;
      acube.measures[idx] = o;
   }

   { // setup the markers
      if(cv_marker_ids.size() < 20u)
         throw std::runtime_error(
             format("expected at least {} unique marker ids, but only found {}",
                    20,
                    cv_marker_ids.size()));

      std::fill(begin(acube.cv_dict_ind), end(acube.cv_dict_ind), -1);
      for(size_t i = 0; i < cv_marker_ids.size(); ++i) {
         acube.cv_dict_ind[i]                      = cv_marker_ids[i];
         acube.marker_lookup[acube.cv_dict_ind[i]] = int(i);
      }

      auto dictionary = get_marker_dictionary();
      for(size_t i = 0; i < acube.markers.size(); ++i) {
         const auto ind = acube.index_to_cv_dict_ind(int(i));
         if(ind >= 0) dictionary->drawMarker(ind, 7, acube.markers[i]);
      }

      { // Sanity check
         if(false) {
            cout << format(
                "makers = [{}]",
                implode(begin(acube.cv_dict_ind), end(acube.cv_dict_ind), ", "))
                 << endl;
            cout << format("lookup = [{}]",
                           implode(begin(acube.marker_lookup),
                                   end(acube.marker_lookup),
                                   ", ",
                                   [](auto& o) {
                                      return format(
                                          "({}, {})", o.first, o.second);
                                   }))
                 << endl;
         }

         for(auto i = 0; i < int(acube.cv_dict_ind.size()); ++i) {
            auto x = acube.index_to_cv_dict_ind(i);
            if(x >= 0) {
               auto j = acube.cv_dict_ind_to_index(x);
               // cout << format("{}  -->  {}  -->  {}", i, x, j) << endl;
               Expects(i == j);
            }
         }
      }
   }

   return acube;
}

void This::init_from_json(const Json::Value& root) noexcept(false)
{
   *this = init_from_json_(root);
}

// ------------------------------------------------------------------ read/write

void read(ArucoCube& acube, const string_view sv) noexcept(false)
{
   acube.init_from_json(parse_json(string(sv)));
}

void write(const ArucoCube& acube, string& sv) noexcept
{
   sv = acube.to_json_str();
}

// ------------------------------------------------------ projection aruco-faces
static void project_aruco_face_worker(
    const ArucoCube::FaceProjectionInfo& pinfo,
    const CachingUndistortInverse& cu,
    std::function<void(int x,
                       int y,            // output image x, y
                       const Vector3& X, // 3D world coord
                       const Point2& p   // face-image coords
                       )> g) noexcept
{
   if(pinfo.f_ind == ArucoCube::BOTTOM) return;

   // Project the quad3d to the image
   array<Vector2, 4> quad;
   std::transform(cbegin(pinfo.quad3d),
                  cend(pinfo.quad3d),
                  begin(quad),
                  [&](auto& X) { return cu.distort(homgen_P2_to_R2(X)); });

   // Get the AABB for the 2d quad
   AABB aabb = AABB::minmax();
   for(const auto& x : quad) aabb.union_point(x);

   // n 'integer' AABB that we can iterate across: the region of interest
   AABBi roi;
   roi.left   = int(std::floor(aabb.left));
   roi.right  = int(std::ceil(aabb.right));
   roi.top    = int(std::floor(aabb.top));
   roi.bottom = int(std::ceil(aabb.bottom));

   // Render every pixel in ROI
   const Vector3 C{0.0, 0.0, 0.0};
   for(auto y = roi.top; y <= roi.bottom; ++y) {
      for(auto x = roi.left; x <= roi.right; ++x) {
         Vector3 ray
             = homgen_R2_to_P2(cu.undistort(Vector2(x, y))).normalised();
         // to_vec3(K_inv * Vector3r(x, y, 1.0)).normalised();
         Vector3 X = plane_ray_intersection(pinfo.p3, C, C + ray);
         Point2 ix = to_pt2(homgen_P2_to_R2(to_vec3(pinfo.H * to_vec3r(X))));
         g(x, y, X, ix);
      }
   }
}

// See the `face-im` version below
void project_aruco_face(
    const ArucoCube::FaceProjectionInfo& pinfo,
    const CachingUndistortInverse& cu,
    std::function<void(Point2 image_xy, Point2 aruco_face_xy)> g) noexcept
{
   project_aruco_face_worker(
       pinfo, cu, [&g](int x, int y, const Vector3& X, const Point2& p) {
          g(Point2(x, y), p);
       });
}

// `pinfo` already contains the necessary transformations
void project_aruco_face(
    const ArucoCube::FaceProjectionInfo& pinfo,
    const CachingUndistortInverse& cu,
    const cv::Mat& face_im,
    std::function<void(int x, int y, uint32_t k)> f) noexcept
{
   project_aruco_face(pinfo, cu, [&](Point2 image_xy, Point2 face_xy) {
      if(in_bounds(face_im, face_xy.x, face_xy.y))
         f(image_xy.x,
           image_xy.y,
           vec3b_to_rgb(face_im.at<cv::Vec3b>(face_xy.y, face_xy.x)));
   });
}

// ---------------------------------------------------------------- Estimate et0
//
static EuclideanTransform
estimate_et0_LLS(const ArucoCube& ac,
                 const vector<ArucoCube::FaceDetection>& detects) noexcept
{
   const Vector3 O{0.0, 0.0, 0.0};

   vector<Vector3> A, B;
   A.reserve(detects.size() * 16);
   B.reserve(A.capacity());

   // What is the error on this?
   ranges::for_each(detects, [&](const auto& d) {
      const auto N          = d.quad3ds.size();
      const auto& face_spec = ac.measures[size_t(d.f_ind)];
      for(auto i = 0u; i < N; ++i) {
         const auto marker_id = d.marker_ids[i];
         const auto m_pos     = face_spec.marker_pos(marker_id);
         Expects(m_pos >= 0 and m_pos < 4);
         const auto w3  = face_spec.marker_3d_quad(m_pos);
         const auto& q3 = d.quad3ds[i];

         A.insert(end(A), cbegin(w3), cend(w3));
         B.insert(end(B), cbegin(q3), cend(q3));
      }
   });

   return transform_between(A, B);
}

// --------------------------------------------------------- refine-pointwise-et
// refine-pointwise-et Use a pointwise cost-function to refine the `et` position
// from a set of `detects`
static std::pair<EuclideanTransform, bool>
refine_pointwise_et(const ArucoCube& ac,
                    const vector<ArucoCube::FaceDetection>& detects,
                    const CachingUndistortInverse& cu,
                    const string_view cam_pos_name,
                    const EuclideanTransform& in_et,
                    const bool feedback) noexcept
{
   EuclideanTransform et = in_et;
   const Vector3 O{0.0, 0.0, 0.0};
   const bool super_feedback = false;
   const int n_params        = 6;

   const std::function<real(const EuclideanTransform& et)> fn_et
       = ac.pointwise_error_fun(cu, detects);

   auto pack   = [&](real* X) { pack_et_6df(et, X); };
   auto unpack = [&](const real* X) { return et = unpack_et_6df(X); };

   vector<real> start(n_params);
   vector<real> step(n_params);
   vector<real> best_params(n_params);

   int counter     = 0;
   real best_score = std::numeric_limits<real>::max();
   auto fn         = [&](const real* X) -> real {
      const auto ret = fn_et(unpack(X));
      if(ret < best_score) {
         best_score = ret;
         std::copy(X, X + n_params, begin(best_params));
         if(super_feedback) {
            cout << format("   #{:8d} :: {:10.7f} :: {{}}",
                           counter,
                           best_score,
                           implode(X, X + n_params, ", "))
                 << endl;
         }
      }
      counter++;
      return ret;
   };

   auto do_refine = [&](bool use_nelder_mead) {
      vector<real> xmin(n_params);
      real ynewlo   = dNAN;
      real ystartlo = dNAN;
      real reqmin   = 1e-7;
      real diffstep = 0.1;
      int kcount    = 10000000; // max interations
      int icount = 0, numres = 0, ifault = 0;
      const char* method = nullptr;

      if(!use_nelder_mead) {
         method = "levenberg-marquardt";
         levenberg_marquardt(fn,
                             n_params,
                             &start[0],
                             &xmin[0],
                             reqmin,
                             diffstep,
                             10,
                             kcount,
                             icount,
                             ifault);
         ynewlo = fn(&xmin[0]);

      } else {
         method = "nelder-mead";

         nelder_mead(fn,
                     n_params,
                     &start[0],
                     &xmin[0],
                     ynewlo,
                     reqmin,
                     &step[0],
                     10,
                     10 * kcount,
                     icount,
                     numres,
                     ifault);
      }
   };

   pack(&start[0]);
   const real ystartlo = fn(&start[0]);
   auto now            = tick();
   for(auto i = 0; i < 10; ++i) {
      do_refine((i % 2) == 0);
      std::copy(cbegin(best_params), cend(best_params), begin(start));
      for(size_t i = 0; i < 3; ++i) { // perturb translation
         start[i] += 0.001 * (2.0 * uniform() - 1.0);
         step[i] = 0.01; // 1cm
      }
      for(size_t i = 3; i < n_params; ++i) { // pertube rotation
         start[i] += 0.001 * (uniform() * one_degree() - 0.5 * one_degree());
         step[i] = one_degree();
      }
   }

   unpack(&best_params[0]);

   const real elapsed_s = tock(now);
   const real ynewlo    = fn(&best_params[0]);
   const bool success   = ynewlo < 20.0; // TODO, should be in degrees

   if(!success or feedback) { // Print feedback
      auto feedback_string = [&]() {
         std::stringstream ss{""};
         ss << endl;
         ss << format("Feedback pointwise cam-pos refine {}", cam_pos_name)
            << endl;
         ss << format("    elapsed time:         {}s", elapsed_s) << endl;
         ss << format("    initial-score:        {}", ystartlo) << endl;
         ss << format("    final-score:          {}", ynewlo) << endl;
         ss << format("    success:              {}", str(success)) << endl;
         return ss.str();
      };
      // And output to screen
      sync_write([&]() { cout << indent(feedback_string(), 4); });
   }

   return std::make_pair(et, success);
}

std::pair<EuclideanTransform, bool>
estimate_init_et(const ArucoCube& ac,
                 const vector<ArucoCube::FaceDetection>& detects,
                 const CachingUndistortInverse& cu,
                 const string_view cam_pos_name,
                 const bool feedback) noexcept
{
   const auto et0 = estimate_et0_LLS(ac, detects);
   return refine_pointwise_et(ac, detects, cu, cam_pos_name, et0, feedback);
}

// -------------------------------------------------------- dense-refine-init-et
//
std::pair<EuclideanTransform, bool> dense_refine_init_et(
    const EuclideanTransform& in_et,
    const ArucoCube& ac,
    const CachingUndistortInverse& cu,
    const LABImage& image_lab,
    const array<cv::Mat, 6>& face_ims,
    const vector<ArucoCube::FaceDetection>& detects,
    const string_view cam_pos_name, // for printing opt feedback
    const bool feedback) noexcept
{
   EuclideanTransform et = in_et;
   const Vector3 O{0.0, 0.0, 0.0};
   const bool super_feedback = false;
   const int n_params        = 6;

   const std::function<real(const EuclideanTransform& et)> fn_et
       = ac.dense_error_fun(cu, image_lab, face_ims, detects);

   auto pack   = [&](real* X) { pack_et_6df(et, X); };
   auto unpack = [&](const real* X) { return et = unpack_et_6df(X); };

   vector<real> start(n_params);
   vector<real> step(n_params);
   vector<real> best_params(n_params);

   int counter     = 0;
   real best_score = std::numeric_limits<real>::max();
   auto fn         = [&](const real* X) -> real {
      unpack(X);
      const auto ret = fn_et(et);

      if(ret < best_score) {
         best_score = ret;
         std::copy(X, X + n_params, begin(best_params));
         if(feedback) {
            cout << format("   #{:8d} :: {:10.7f} :: {{}}",
                           counter,
                           best_score,
                           implode(X, X + n_params, ", "))
                 << endl;
         }
      }
      counter++;
      return ret;
   };

   auto do_refine = [&](bool use_nelder_mead) {
      vector<real> xmin(n_params);
      real ynewlo   = dNAN;
      real ystartlo = dNAN;
      real reqmin   = 1e-7;
      real diffstep = 0.1;
      int kcount    = 10000000; // max interations
      int icount = 0, numres = 0, ifault = 0;
      const char* method = nullptr;

      if(!use_nelder_mead) {
         method = "levenberg-marquardt";
         levenberg_marquardt(fn,
                             n_params,
                             &start[0],
                             &xmin[0],
                             reqmin,
                             diffstep,
                             10,
                             kcount,
                             icount,
                             ifault);
         ynewlo = fn(&xmin[0]);

      } else {
         method = "nelder-mead";

         nelder_mead(fn,
                     n_params,
                     &start[0],
                     &xmin[0],
                     ynewlo,
                     reqmin,
                     &step[0],
                     10,
                     1000,
                     icount,
                     numres,
                     ifault);
      }
   };

   for(size_t i = 0; i < 3; ++i) step[i] = 0.01; // 1cm
   for(size_t i = 3; i < n_params; ++i) step[i] = one_degree();

   auto now = tick();
   pack(&start[0]);
   const real ystartlo = fn(&start[0]);
   do_refine(true);
   unpack(&best_params[0]);

   const real elapsed_s = tock(now);
   const real ynewlo    = fn(&best_params[0]);
   const bool success   = true;

   if(!success or feedback) { // Print feedback
      auto feedback_string = [&]() {
         std::stringstream ss{""};
         ss << endl;
         ss << format("Feedback dense-refine on cam {}", cam_pos_name) << endl;
         ss << format("    elapsed time:         {}s", elapsed_s) << endl;
         ss << format("    initial-score:        {}", ystartlo) << endl;
         ss << format("    final-score:          {}", ynewlo) << endl;
         ss << format("    success:              {}", str(success)) << endl;
         return ss.str();
      };
      // And output to screen
      sync_write([&]() { cout << indent(feedback_string(), 4); });
   }

   return std::make_pair(et, success);
}

} // namespace perceive
