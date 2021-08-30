
#include "manifest-data.hpp"

#include <opencv2/opencv.hpp>

#include "config.hpp"
#include "print-helper.hpp"

#include "perceive/io/lazy-s3.hpp"
#include "perceive/io/perceive-assets.hpp"
#include "perceive/optimization/breadth-first-search.hpp"
#include "perceive/utils/create-cv-remap.hpp"
#include "perceive/utils/file-system.hpp"

#define This ManifestData

namespace perceive::calibration::position_scene_cameras
{
// --------------------------------------------------------- lookup-camera-index
//
size_t This::lookup_camera_index(const string_view camera_id) const noexcept
{
   auto ii
       = std::find_if(cbegin(cam_infos), cend(cam_infos), [&](const auto& o) {
            return o.camera_id == camera_id;
         });

   return (ii == cend(cam_infos))
              ? cam_infos.size()
              : size_t(std::distance(cbegin(cam_infos), ii));
}

// ------------------------------------------------------------- lookup-position
//
size_t This::lookup_position(const string_view position_name) const noexcept
{
   auto ii = ranges::find_if(positions, [&](const auto& pos_dat) {
      return pos_dat.position_name == position_name;
   });
   return (ii == cend(positions))
              ? positions.size()
              : size_t(ranges::distance(cbegin(positions), ii));
}

// -------------------------------------------------- lookup-cam-cam-constraints
//
vector<std::pair<int, int>> This::get_cam_cam_constraints() const noexcept
{
   vector<std::pair<int, int>> o;
   o.reserve(constraints.size());
   std::transform(cbegin(constraints),
                  cend(constraints),
                  std::back_inserter(o),
                  [&](const auto ij) {
                     return std::make_pair(ij.first.x(), ij.first.y());
                  });
   return o;
}

// -------------------------------------------------- calc-paths-to-ref-position
//
static vector<vector<int>> calc_paths_to_ref_position(const ManifestData& mdata)
{
   const int n_cameras = mdata.n_cameras();

   auto get_cam_id = [&](const int ind) -> const string& {
      return mdata.cam_infos.at(size_t(ind)).camera_id;
   };

   auto lookup_camera_index = [&](const string_view camera_id) -> size_t {
      const auto index = mdata.lookup_camera_index(camera_id);
      Expects(index < mdata.cam_infos.size());
      return index;
   };

   // Find all the neighbours for all the cameras
   auto calc_all_neighbours = [&]() {
      auto calc_neighbours = [&](const int u) {
         const auto& cinfo = mdata.cam_infos.at(size_t(u));
         vector<int> x;

         for(auto pos_ind : cinfo.positions)
            for(auto cdat_ind : mdata.positions.at(size_t(pos_ind)).cdats)
               if(mdata.data.at(size_t(cdat_ind)).cam_index != size_t(u))
                  x.push_back(int(mdata.data.at(size_t(cdat_ind)).cam_index));

         remove_duplicates(x);
         return x;
      };
      vector<vector<int>> o((size_t(n_cameras)));
      for(auto i = 0; i < n_cameras; ++i) o[size_t(i)] = calc_neighbours(i);
      return o;
   };
   const vector<vector<int>> cam_neighbours = calc_all_neighbours();

   // Which cameras are sink cameras?
   // A sink camera is one that can see the default-position-index
   auto calc_sink_cameras = [&]() {
      vector<bool> o(size_t(n_cameras), false);
      for(const auto& cdat : mdata.data) {
         if(cdat.position_index == mdata.default_position_index)
            o.at(lookup_camera_index(cdat.camera_id)) = true;
      }
      return o;
   };
   const vector<bool> sink_cameras = calc_sink_cameras();

   auto is_sink = [&sink_cameras](const unsigned u) -> bool {
      return sink_cameras.at(u);
   };

   auto for_each_neighbour
       = [&cam_neighbours](const unsigned u,
                           std::function<void(const unsigned)> f) {
            std::for_each(cbegin(cam_neighbours[u]),
                          cend(cam_neighbours[u]),
                          [&f](unsigned ind) { f(ind); });
         };

   auto convert_to_cdat_path = [&](const vector<unsigned>& cam_cam_path) {
      vector<int> path;

      auto get_cam_id = [&](const unsigned cam_index) -> const string_view {
         return mdata.cam_infos.at(cam_index).camera_id;
      };

      auto find_cdat = [&](const string_view camera_id) {
         const auto ii = std::find_if(
             cbegin(mdata.data),
             cend(mdata.data),
             [&](const auto& cdat) -> bool {
                return (cdat.camera_id == camera_id)
                       && (cdat.position_index == mdata.default_position_index);
             });
         Expects(ii != cend(mdata.data));
         return int(std::distance(cbegin(mdata.data), ii));
      };

      auto find_cdat_for_position_and_cam
          = [&](const size_t pos_index, const size_t cam_index) -> size_t {
         auto ii = ranges::find_if(mdata.data, [&](const auto& cdat) {
            return cdat.cam_index == cam_index
                   && cdat.position_index == pos_index;
         });
         return (ii == cend(mdata.data))
                    ? mdata.data.size()
                    : size_t(ranges::distance(cbegin(mdata.data), ii));
      };

      // Convert a pair of cameras to a pair of cdat-indices
      // EXCEPT for the final position
      for(size_t i = 0; i + 1 < cam_cam_path.size(); ++i) {
         const int cam_0 = int(cam_cam_path[i + 0]);
         const int cam_1 = int(cam_cam_path[i + 1]);

         size_t ind_0 = 0;
         size_t ind_1 = 0;
         for(size_t pindex = 0; pindex < mdata.positions.size(); ++pindex) {
            ind_0 = find_cdat_for_position_and_cam(pindex, size_t(cam_0));
            ind_1 = find_cdat_for_position_and_cam(pindex, size_t(cam_1));
            if(ind_0 < mdata.data.size() && ind_1 < mdata.data.size()) break;
         }

         Expects(ind_0 < mdata.data.size() && ind_1 < mdata.data.size());
         path.push_back(int(ind_0));
         path.push_back(int(ind_1));
      }

      if(!cam_cam_path.empty()) { // get the ref view
         const auto cam_id = get_cam_id(cam_cam_path.back());
         const auto& info  = mdata.positions.at(mdata.default_position_index);
         auto ii           = ranges::find_if(info.cdats, [&](int ind) {
            return mdata.data.at(size_t(ind)).camera_id == cam_id;
         });
         Expects(ii != cend(info.cdats));
         path.push_back(*ii);
      }

      return path;
   };

   auto make_path_to = [&](const int cam_ind) {
      const vector<unsigned> cam_cam_path = breadth_first_search(
          unsigned(cam_ind), is_sink, size_t(n_cameras), for_each_neighbour);
      const auto cdat_path = convert_to_cdat_path(cam_cam_path);
      if(cdat_path.size() == 0) {
         LOG_ERR(
             format("failed to find path to camera '{}'", get_cam_id(cam_ind)));
      } else {
         Expects(cdat_path.size() % 2 == 1);
      }

      return cdat_path;
   };

   vector<vector<int>> paths((size_t(n_cameras)));
   for(auto i = 0; i < n_cameras; ++i) paths[size_t(i)] = make_path_to(i);

   return paths;
}

// ----------------------------------------------- calc-scene-consistent-cam-ets
//
static vector<EuclideanTransform>
calc_scene_consistent_cam_ets(const ManifestData& mdata,
                              const string_view outdir)
{
   const int n_cameras = mdata.n_cameras();

   auto init_et = [&](int index) -> const EuclideanTransform& {
      return mdata.data.at(size_t(index)).init_et;
   };

   auto calc_et = [&](const auto& cinfo) {
      const vector<int>& path = cinfo.cdat_path;
      if(path.size() == 0) return EuclideanTransform::nan();
      EuclideanTransform e0 = {};
      bool is_odd           = true;
      for(auto ii = rbegin(path); ii != rend(path); ++ii) {
         e0     = (is_odd ? init_et(*ii) : init_et(*ii).inverse()) * e0;
         is_odd = !is_odd;
      }
      return e0;
   };

   vector<EuclideanTransform> ets;
   ets.resize(size_t(n_cameras));
   std::transform(
       cbegin(mdata.cam_infos), cend(mdata.cam_infos), begin(ets), calc_et);
   return ets;
}

// ---------------------------------------------- calc-scene-consistent-cube-ets
//
static vector<EuclideanTransform>
calc_scene_consistent_cube_ets(const ManifestData& mdata,
                               const string_view outdir)
{
   const int n_cameras   = mdata.n_cameras();
   const int n_positions = mdata.n_positions();

   const auto cam_ets = calc_scene_consistent_cam_ets(mdata, outdir);
   vector<EuclideanTransform> ets;
   ets.resize(size_t(n_positions), EuclideanTransform::nan());

   ranges::for_each(mdata.data, [&](const auto& cdat) {
      const auto& cinfo   = mdata.cam_infos.at(cdat.cam_index);
      const auto& cam_et  = cam_ets.at(cdat.cam_index); // cam->world
      const auto& init_et = cdat.init_et;

      auto& et = ets.at(cdat.position_index);
      if(cinfo.cdat_path.size() > 0 && !et.is_finite())
         et = (init_et.inverse() * cam_et).inverse();
   });

   return ets;
}

// --------------------------------------------------------- parse-manifest-file
//
ManifestData parse_manifest_file(const Config& config) noexcept(false)
{
   ManifestData mdata;
   ParallelJobSet pjobs;
   const string_view outdir         = config.outdir;
   const string_view manifest_fname = config.manifest_fname;
   string raw_manifest_data;
   vector<vector<string>> lines;
   std::atomic<bool> has_error{false};

   { // ---- Load the aruco cube
      try {
         if(config.cube_key.empty() and config.cube_filename.empty()) {
            mdata.ac = make_kyle_aruco_cube();
            print(g_info,
                  g_default,
                  format("set aruco cube to default bespoke `kyle-cube`"));
         } else if(!config.cube_key.empty()) {
            fetch(mdata.ac, config.cube_key);
            print(g_info,
                  g_default,
                  format("loaded aruco cube `{}`", config.cube_key));
         } else if(!config.cube_filename.empty()) {
            read(mdata.ac, file_get_contents(config.cube_filename));
            print(g_info,
                  g_default,
                  format("loaded aruco cube from file `{}`",
                         config.cube_filename));
         } else {
            FATAL("logic error");
         }
      } catch(std::exception& e) {
         print(g_skull,
               g_error,
               format("failed to load the aruco cube: {}", e.what()));
         has_error.store(true, std::memory_order_relaxed);
      }

      Expects(ArucoCube::k_n_faces == mdata.face_ims.size());
      for(auto i = 0; i < ArucoCube::k_n_faces; ++i) {
         auto f_ind           = ArucoCube::face_e(i);
         const auto face_size = 800;
         mdata.face_ims[size_t(i)]
             = mdata.ac.draw_face(f_ind, face_size, false, false);
         if(f_ind == ArucoCube::BOTTOM) continue;
         const auto fname = format(
             "{}/000_{}_{}.png", outdir, i, ArucoCube::face_to_str(f_ind));
         cv::imwrite(fname, mdata.ac.draw_face(f_ind, face_size, true, true));
      }
   }

   { // ---- Set Undistort K
      const unsigned uw = 800;
      const unsigned uh = 600;  // Undistorted width/height
      const real vfov   = 75.0; // vertical field of view

      mdata.undistorted_w = uw;
      mdata.undistorted_h = uh;
      mdata.K             = Matrix3r::Identity();
      mdata.K(0, 0)       = mdata.K(1, 1)
          = 0.5 * real(uh) / tan(0.5 * to_radians(vfov));
      mdata.K(0, 2) = uw * 0.5;
      mdata.K(1, 2) = uh * 0.5;
   }

   { // ---- Load the raw manifest data
      try {
         print(g_info,
               g_default,
               format("loading manifest file: {}", manifest_fname));
         if(is_s3_uri(manifest_fname))
            s3_load(manifest_fname, raw_manifest_data);
         else
            raw_manifest_data = file_get_contents(manifest_fname);
      } catch(std::exception& e) {
         print(g_skull, g_error, format("FAILED: {}", e.what()));
         return mdata;
      }
   }

   { // ---- Parse the manifest data into a set of fields
      std::istringstream in_data(raw_manifest_data);
      auto line_no = 1; // lines in failes are naturally '1'-indexed
      for(string line; std::getline(in_data, line); ++line_no) {
         trim(line);
         if(line.empty() || line[0] == '#') continue;

         auto parts = explode(line, " \t", true);
         if(parts.size() != 3) {
            print(
                g_skull,
                g_error,
                format("parse error at line {}: expected 3 fields, but got {}",
                       line_no,
                       parts.size()));
            has_error.store(true, std::memory_order_relaxed);
         } else {
            lines.push_back(std::move(parts));
         }
      }
   }

   { // ---- Read Positions
      auto read_position
          = [](const string_view pos) -> std::pair<string_view, bool> {
         if(pos.empty() || pos.back() != '*') return {pos, false};
         return {string_view(&pos[0], pos.size() - 1), true};
      };

      auto lookup_position = [&](const string_view position) {
         auto ret = mdata.lookup_position(position);
         Expects(ret < mdata.positions.size());
         return ret;
      };

      // Set up the positions, and the default-position-index
      string_view default_position = "";
      for(const auto& fields : lines) {
         const auto [position, is_default] = read_position(fields[0]);
         if(is_default) {
            if(default_position == position) {
               // all is good... it's just been selected twice or more times
            } else if(default_position == ""s) {
               default_position = position; // all is good... record the default
            } else {
               print(g_skull,
                     g_error,
                     format("attempted to set '{}' as the default position, "
                            "however, the default was already set to '{}'!",
                            position,
                            default_position));
               has_error.store(true, std::memory_order_relaxed);
            }
         }

         PositionInfo pos_info;
         pos_info.position_name = string(cbegin(position), cend(position));
         if(mdata.lookup_position(position) < mdata.positions.size()) {
            // We already have this position
         } else {
            mdata.positions.push_back(std::move(pos_info));
         }
      }

      // It's just nicer for output messages
      // remove_duplicates(mdata.positions);
      std::sort(begin(mdata.positions),
                end(mdata.positions),
                [&](const auto& A, const auto& B) {
                   return A.position_name < B.position_name;
                });

      // What was that default position??
      if(default_position == "") {
         print(g_skull,
               g_error,
               format("no default position was set! At least one position "
                      "entry must be marked with a '*' at the end. For "
                      "example, 'Aruco1*' would mean that the Aruco1 position "
                      "is the default position."));
         has_error.store(true, std::memory_order_relaxed);
      } else {
         mdata.default_position_index = lookup_position(default_position);
         print(g_info,
               g_default,
               format("setting default position to '{}'", default_position));
      }

      // Convert lines to cam-image-datas
      mdata.data.reserve(lines.size());
      std::transform(cbegin(lines),
                     cend(lines),
                     std::back_inserter(mdata.data),
                     [&](const auto& fields) -> CamImageData {
                        CamImageData cdat;
                        const auto [position, is_default]
                            = read_position(fields[0]);
                        cdat.position_index = lookup_position(position);
                        cdat.camera_id      = fields[1];
                        cdat.image_fname    = fields[2];
                        cdat.name           = format(
                            "{}-{}",
                            mdata.positions[cdat.position_index].position_name,
                            cdat.camera_id);
                        return cdat;
                     });
   }

   { // ---- Every camera-position must be unique
      auto is_duplicate_position_camera
          = [](const auto& cdat0, const auto& cdat1) {
               return cdat0.position_index == cdat1.position_index
                      && cdat0.camera_id == cdat1.camera_id;
            };

      auto process_duplicate = [&](const auto& cdat0, const auto& cdat1) {
         print(g_skull,
               g_error,
               format("duplicate position-camera pair {}, for "
                      "images\n\t{}\n\t{}",
                      cdat0.name,
                      cdat0.image_fname,
                      cdat1.image_fname));
         has_error.store(true, std::memory_order_relaxed);
      };

      for(size_t i = 1; i < mdata.data.size(); ++i)
         for(size_t j = 0; j < i; ++j)
            if(is_duplicate_position_camera(mdata.data[i], mdata.data[j]))
               process_duplicate(mdata.data[i], mdata.data[j]);

      if(has_error.load(std::memory_order_relaxed))
         throw std::runtime_error("aborting due to previous errors");
   }

   { // Load the images
      auto construct_path
          = [](const string_view ref_fname, const string_view fname) {
               if(!fname.empty() && (fname[0] == '/' || is_s3_uri(fname)))
                  return format("{}", fname);
               const auto path = format("{}/{}", dirname_sv(ref_fname), fname);
               return is_s3_uri(path) ? path : absolute_path(path);
            };

      vector<char> raw_bytes;
      ranges::for_each(mdata.data, [&](auto& cdat) {
         string path = ""s;
         try {
            path = construct_path(manifest_fname, cdat.image_fname);
            if(is_s3_uri(path)) {
               lazy_load(path, raw_bytes);
            } else {
               file_get_contents(path, raw_bytes);
            }
            cdat.raw_image = decode_image_to_cv_mat(raw_bytes);
            array<ARGBImage, 2> parts;
            hsplit(cv_to_argb(cdat.raw_image), parts[0], parts[1]);
            cdat.image = argb_to_cv(parts[0]);
            print(g_tick, g_default, format("loaded '{}'", path));
         } catch(std::exception& e) {
            print(g_skull,
                  g_error,
                  format("failed to read image file '{}': {}", path, e.what()));
            has_error.store(true, std::memory_order_relaxed);
         }
      });
   }

   auto get_unique_camera_ids = [&]() {
      vector<string> camera_ids;
      camera_ids.reserve(lines.size());
      std::transform(cbegin(lines),
                     cend(lines),
                     std::back_inserter(camera_ids),
                     [&](const auto& fields) { return fields.at(1); });
      remove_duplicates(camera_ids);
      return camera_ids;
   };
   const auto camera_ids = get_unique_camera_ids();

   { // Sort `cdats` into positions
      for(size_t i = 0; i < mdata.data.size(); ++i) {
         const auto& cdat   = mdata.data[i];
         const size_t index = cdat.position_index;
         Expects(index < mdata.positions.size());
         mdata.positions[index].cdats.push_back(int(i));
      }
   }

   { // Update cdat camera indices
      ranges::for_each(mdata.data, [&](auto& cdat) {
         auto ii = ranges::find(camera_ids, cdat.camera_id);
         Expects(ii != cend(camera_ids));
         cdat.cam_index = size_t(ranges::distance(cbegin(camera_ids), ii));
      });
   }

   { // Initialize cam-infos
      mdata.cam_infos.reserve(camera_ids.size());
      std::transform(cbegin(camera_ids),
                     cend(camera_ids),
                     std::back_inserter(mdata.cam_infos),
                     [&](const auto cam_id) {
                        CamInfo cinfo;
                        cinfo.camera_id = cam_id;
                        return cinfo;
                     });

      { // all the positions for a given camera
         ranges::for_each(mdata.data, [&](const auto& cdat) {
            auto& cinfo = mdata.cam_infos.at(cdat.cam_index);
            cinfo.positions.push_back(int(cdat.position_index));
         });
         ranges::for_each(mdata.cam_infos, [&](auto& cinfo) {
            remove_duplicates(cinfo.positions);
         });
      }

      { // get the path for each camera
         const auto paths = calc_paths_to_ref_position(mdata);
         Expects(paths.size() == mdata.cam_infos.size());
         for(size_t i = 0; i < paths.size(); ++i)
            mdata.cam_infos.at(i).cdat_path = paths.at(i);

         { // What are those paths??
            ranges::for_each(mdata.cam_infos, [&](const auto& cinfo) {
               const auto path_s = format(
                   "Path {}:  {}",
                   cinfo.camera_id,
                   rng::implode(
                       cinfo.cdat_path, "  -->  ", [&](const auto& cdat_ind) {
                          return format("{{{}}}",
                                        mdata.data.at(size_t(cdat_ind)).name);
                       }));
               print(g_info, g_default, path_s);
            });
         }
      }

      { // set the working formats (from cdat images)
         ranges::for_each(mdata.data, [&](auto& cdat) {
            const int iw        = cdat.image.cols;
            const int ih        = cdat.image.rows;
            const auto cdat_fmt = Point2(iw, ih);
            Point2& wfmt = mdata.cam_infos.at(cdat.cam_index).working_format;
            if(wfmt == Point2{0, 0} || wfmt == cdat_fmt) {
               wfmt = cdat_fmt;
            } else {
               has_error.store(true, std::memory_order_relaxed);
               LOG_ERR(
                   format("camera {} had at least two different image formats!",
                          cdat.camera_id));
            }
         });

         // We have a format for each camera
         Expects(ranges::all_of(mdata.cam_infos, [&](const auto& x) {
            return x.working_format.x > 0 && x.working_format.y > 0;
         }));
      }

      { // Load bcam-infos
         for(size_t i = 0; i < camera_ids.size(); ++i) {
            pjobs.schedule([i, &mdata, &has_error]() {
               auto& cinfo        = mdata.cam_infos.at(i);
               const auto& cam_id = cinfo.camera_id;
               auto& bcam_info    = cinfo.bcam_info;
               try {
                  const auto now_i = tick();
                  fetch(bcam_info, cam_id);
                  print(g_tick,
                        g_default,
                        format("loading camera '{}' - {:5.3f}s",
                               cam_id,
                               tock(now_i)));
               } catch(std::exception& e) {
                  print(g_skull,
                        g_error,
                        format("failed to load camera '{}': {}",
                               cam_id,
                               e.what()));
                  has_error.store(true, std::memory_order_relaxed);
               }
            });
         }
         pjobs.execute();
      }

      { // Fetch caching undistort inverses
         ranges::for_each(mdata.cam_infos, [&pjobs](auto& cinfo) {
            pjobs.schedule([&cinfo]() {
               const auto& bcam_info = cinfo.bcam_info;
               const auto wfmt       = cinfo.working_format;
               auto& cu              = cinfo.cu;
               const auto now_i      = tick();
               cu.init(bcam_info.M[0]);
               cu.set_working_format(wfmt.x, wfmt.y);
               print(g_tick,
                     g_default,
                     format("loading caching-undistort for {} - {:5.3f}s",
                            bcam_info.M[0].sensor_id(),
                            tock(now_i)));
            });
         });
         pjobs.execute();
      }

      { // Create undistort maps
         const Matrix3r H            = Matrix3r::Identity();
         const auto uw               = mdata.undistorted_w;
         const auto uh               = mdata.undistorted_h;
         const bool use_fast_distort = config.use_fast_distort;

         auto now = tick();
         print(g_info, g_default, "calculating/loading undistort maps", false);

         ranges::for_each(mdata.cam_infos, [&](auto& cinfo) {
            auto& mapxys          = cinfo.mapxys;
            const auto& bcam_info = cinfo.bcam_info;
            const auto& cu        = cinfo.cu; // working format set
            const auto fname      = format("{}/mapxys_{}_fast={}.data",
                                      outdir,
                                      cinfo.camera_id,
                                      str(use_fast_distort));
            if(is_regular_file(fname)) {
               cv::FileStorage file(fname, cv::FileStorage::READ);
               file[format("cam_mapx")] >> mapxys[0];
               file[format("cam_mapy")] >> mapxys[1];
            } else {
               std::function<Vector2(const Vector2&)> f
                   = [&](const Vector2& x) { return cu.fast_distort(x); };
               std::function<Vector2(const Vector2&)> g
                   = [&](const Vector2& x) { return cu.distort(x); };
               auto ff = use_fast_distort ? f : g;

               create_cv_remap_threaded(
                   uw, uh, H, ff, mdata.K, mapxys[0], mapxys[1], pjobs);

               cv::FileStorage file(fname, cv::FileStorage::WRITE);
               file << format("cam_mapx") << mapxys[0];
               file << format("cam_mapy") << mapxys[1];
            }
         });

         cout << format(" - {:5.3f}s", tock(now)) << endl;
      }
   }

   { // Undistort the images
      print(g_info, g_default, "undistorting images", false);
      const auto now = tick();
      auto f         = [&](int cdat_ind) {
         auto& cdat         = mdata.data.at(size_t(cdat_ind));
         const auto& cinfo  = mdata.cam_infos.at(size_t(cdat.cam_index));
         const auto& mapxys = cinfo.mapxys;
         cv::remap(cdat.image,
                   cdat.undistorted,
                   mapxys[0],
                   mapxys[1],
                   cv::INTER_LINEAR,
                   cv::BORDER_CONSTANT,
                   cv::Scalar(255, 255, 255));
         const auto fname
             = format("{}/002_undistorted_{}.png", outdir, cdat.name);
         cv::imwrite(fname, cdat.undistorted);
      };
      for(size_t i = 0; i < mdata.data.size(); ++i)
         pjobs.schedule([&f, i]() { f(int(i)); });
      pjobs.execute();
      cout << format(" - {:5.3f}s", tock(now)) << endl;
   }

   { // ArucoCube detections
      const auto& ac = mdata.ac;
      const auto& K  = mdata.K;

      auto process_cdat = [&](auto& cdat) {
         const auto now = tick();
         const auto out_fname
             = format("{}/003_{}_detect.png", outdir, cdat.name);

         const auto& cinfo = mdata.cam_infos.at(cdat.cam_index);
         const auto& cu    = cinfo.cu;

         cdat.detects = ac.detect_markers(cdat.image, cu, out_fname);

         { // estimate initial et
            const auto [et, success]
                = estimate_init_et(ac, cdat.detects, cu, cdat.name, false);
            ARGBImage argb = cv_to_argb(cdat.image);
            ac.render_pointwise(argb, cdat.detects, cu, et, {});
            argb.save(format("{}/008_init-et_{}.png", outdir, cdat.name));
            cdat.init_et = et.inverse();
            if(!success) has_error = true;
         }

         { // let's try refine et
            const auto [et, success]
                = dense_refine_init_et(cdat.init_et.inverse(),
                                       ac,
                                       CachingUndistortInverse(K),
                                       cv_to_LAB_im(cdat.undistorted),
                                       mdata.face_ims,
                                       cdat.detects,
                                       cdat.name,
                                       false);
            ARGBImage argb = cv_to_argb(cdat.image);
            ac.render_dense(argb, cdat.detects, mdata.face_ims, cu, et, {});
            argb.save(format("{}/009_dense-et_{}.png", outdir, cdat.name));
            cdat.init_et = et.inverse();
            if(!success) has_error = true;
         }

         const auto err_fn = ac.pointwise_error_fun(cu, cdat.detects);
         const auto err0   = err_fn(cdat.init_et.inverse());

         {
            auto redistort = [&](const Vector2& D) -> Vector2 {
               Vector3r N = to_vec3r(homgen_R2_to_P2(cinfo.cu.undistort(D)));
               return homgen_P2_to_R2(to_vec3(K * N));
            };

            auto distorted_quad_to_undistorted
                = [&](const array<Vector2, 4>& Q) {
                     array<Vector2, 4> O;
                     for(auto&& [q, o] : views::zip(Q, O)) o = redistort(q);
                     return O;
                  };

            ARGBImage argb = cv_to_argb(cdat.undistorted);
            ranges::for_each(cdat.detects, [&](const auto& detect) {
               const auto& face = ac.measures.at(size_t(detect.f_ind));
               const auto k     = face.kolour;
               for(const auto& quad : detect.quads)
                  render_quad_2d(argb, distorted_quad_to_undistorted(quad), k);
            });
            argb.save(out_fname);
         }

         const bool success = cdat.detects.size() >= 2;
         if(success) {
            print(g_info,
                  g_default,
                  format("detected {} markers on {}, err = {:5.3f} - {:5.3f}s",
                         cdat.detects.size(),
                         cdat.name,
                         err0,
                         tock(now)));
         } else {
            print(
                g_skull,
                g_error,
                format("failed to detected 2 or more markers on {} - {:5.3f}s",
                       cdat.name,
                       tock(now)));
            has_error.store(true, std::memory_order_relaxed);
         }
      };

      ranges::for_each(mdata.data, [&](auto& cdat) {
         pjobs.schedule([&cdat, &process_cdat]() { process_cdat(cdat); });
      });
      pjobs.execute();
   }

   { // get scene-consistent initial estimate for camera position
      const auto ets = calc_scene_consistent_cam_ets(mdata, outdir);
      Expects(ets.size() == mdata.cam_infos.size());
      for(size_t i = 0; i < ets.size(); ++i)
         mdata.cam_infos.at(i).init_et = ets.at(i);
   }

   { // Scene-consistent Aruco-cube positions
      const auto ets = calc_scene_consistent_cube_ets(mdata, outdir);
      Expects(ets.size() == mdata.positions.size());
      for(size_t i = 0; i < ets.size(); ++i)
         mdata.positions.at(i).init_et = ets.at(i);
   }

   { // Create constraint graph
      auto now = tick();

      print(g_info,
            g_default,
            format("finding camera-camera constraints", tock(now)));

      auto lookup_camera = [&](const auto& cam_id) {
         auto ii = ranges::find(camera_ids, cam_id);
         Expects(ii != cend(camera_ids));
         return ranges::distance(cbegin(camera_ids), ii);
      };

      auto is_pos_fun = [&](size_t pos_ind) {
         return [pos_ind](const auto& cdat) {
            return cdat.position_index == pos_ind;
         };
      };

      auto get_cdat_index
          = [&](const auto& cdat) { return int(&cdat - &mdata.data[0]); };

      auto find_it = [&](const size_t pos_ind) {
         auto rng = mdata.data | views::filter(is_pos_fun(pos_ind))
                    | views::transform(get_cdat_index);
         vector<int> cdat_inds(begin(rng), end(rng));

         for(size_t i = 0; i < cdat_inds.size(); ++i) {
            for(size_t j = i + 1; j < cdat_inds.size(); ++j) {
               OrderedPair key(
                   int(lookup_camera(
                       mdata.data[size_t(cdat_inds[i])].camera_id)),
                   int(lookup_camera(
                       mdata.data[size_t(cdat_inds[j])].camera_id)));
               auto ii = mdata.constraints.find(key);
               if(ii == cend(mdata.constraints)) {
                  vector<int> indices  = {cdat_inds[i], cdat_inds[j]};
                  CamCamConstraints cc = {key, std::move(indices)};
                  mdata.constraints.insert(std::make_pair(cc.cam_indices, cc));
               } else {
                  ii->second.cam_image_indices.push_back(cdat_inds[i]);
                  ii->second.cam_image_indices.push_back(cdat_inds[j]);
               }
            }
         }
      };

      for(size_t ind = 0; ind < mdata.positions.size(); ++ind) { find_it(ind); }

      // Print the output
      for(const auto& [key, cc] : mdata.constraints) {
         vector<string> poses;
         std::transform(cbegin(cc.cam_image_indices),
                        cend(cc.cam_image_indices),
                        std::back_inserter(poses),
                        [&](const int ind) {
                           return mdata.positions
                               .at(mdata.data.at(size_t(ind)).position_index)
                               .position_name;
                        });
         remove_duplicates(poses);
         print(g_tick,
               g_default,
               format("{}-{}: ['{}']",
                      camera_ids[size_t(key.x())],
                      camera_ids[size_t(key.y())],
                      rng::implode(poses, "', '")));
      }

      print(g_info, g_default, format("done - {:5.3f}s", tock(now)));
   }

   if(has_error.load(std::memory_order_relaxed)) {
      throw std::runtime_error("aborting due to previous errors");
   }

   { // Some trailing INFO lines
      print(g_info, g_default, "successfully loaded manifest data");
   }

   return mdata;
}

void cdat_fn_test(const ManifestData& mdata)
{
   auto test_cdat = [&](const auto& cdat) {
      const auto& cu    = mdata.cam_infos.at(cdat.cam_index).cu;
      const auto err_fn = mdata.ac.pointwise_error_fun(cu, cdat.detects);
      const auto err0   = err_fn(cdat.init_et.inverse());
      return err0;
   };
   ranges::for_each(mdata.data, [&](const auto& cdat) {
      INFO(format("{} -->  {}", cdat.name, test_cdat(cdat)));
   });
}

} // namespace perceive::calibration::position_scene_cameras
