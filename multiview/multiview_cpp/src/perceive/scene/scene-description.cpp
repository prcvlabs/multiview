
#include "scene-description.hpp"
#include "stdinc.hpp"

#include "perceive/geometry/polygon.hpp"
#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/cv-helpers.hpp"
#include "perceive/graphics/sprite.hpp"
#include "perceive/graphics/tiny-string.hpp"
#include "perceive/io/lazy-s3.hpp"
#include "perceive/io/perceive-assets.hpp"
#include "perceive/io/versioned-resource.hpp"
#include "perceive/movie/draw-floor-grid.hpp"
#include "perceive/movie/ffmpeg.hpp"
#include "perceive/utils/cuda-spec.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/md5.hpp"
#include "perceive/utils/opencv-helpers.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifdef USING_OPENGL
#include "gl/gl-inc.hpp"
#include "gl/platform/glew-bridge.hpp"
#include <GL/gl.h>
#endif

#define This SceneDescription

namespace perceive
{
// --------------------------------------------------------------- Find bcam key

static string remove_version_number(const string_view in_s)
{
   const auto rsrc = VersionedResource::make(in_s);
   // LOG_ERR(format("rsrc = {}", rsrc.to_string()));
   return rsrc.name;
}

template<typename InputItr>
static InputItr
find_bcam_key(InputItr begin, InputItr end, const std::string& val)
{
   for(auto ii = begin; ii != end; ++ii) {
      const auto& s = *ii;

      if(val == s) return ii;

      // Try again, but without and version number if it is there
      if(val == remove_version_number(s)) return ii;
   }

   return end;
}

// ------------------------------------------------------------ calc-frame-pairs
//
vector<Point2> calc_frame_pairs(const real in_fps,
                                const real target_fps,
                                const int N) noexcept(false)
{
   vector<Point2> ret;
   ret.reserve(size_t(N));

   const real i_duration = 1.0 / in_fps;
   const real o_duration = 1.0 / target_fps;

   auto calc_err = [&](const int i, const int o) {
      return std::fabs((i * i_duration) - (o * o_duration));
   };

   // We want to keep 'it' and 'ot' as close as possible togeather
   int i_counter = 0;
   int o_counter = 0;

   if(i_counter < N) ret.emplace_back(i_counter, o_counter);

   while(i_counter < N) {
      // we must increment 'i', 'o', or 'b' (both)
      const auto i_err = calc_err(i_counter + 1, o_counter);
      const auto o_err = calc_err(i_counter, o_counter + 1);
      const auto b_err = calc_err(i_counter + 1, o_counter + 1);

      // int i0 = i_counter, o0 = o_counter;

      if(i_err < o_err and i_err < b_err) {
         i_counter++;
      } else if(o_err < b_err) {
         o_counter++;
      } else {
         i_counter++;
         o_counter++;
      }

      if(i_counter < N)
         ret.emplace_back(i_counter, o_counter);
      else
         break;
   }

   return ret;
}

// ------------------------------------------ scene-frame_to_movie-frame_mapping
//
static vector<unsigned>
scene_frame_to_movie_frame_mapping(const real video_fps,
                                   const real target_fps,
                                   const int n_video_frames) noexcept
{
   const auto xys = calc_frame_pairs(video_fps, target_fps, n_video_frames);
   vector<unsigned> ret;
   ret.reserve(xys.size());

   int next_frame = 0;
   for(const auto& xy : xys) {
      if(xy.y == next_frame) {
         ret.push_back(unsigned(xy.x)); // Why did I have "+1"?
         next_frame++;
      }
   }

   if(false) {
      for(const auto& xy : xys) {
         cout << format("{:03d} => {:03d}", xy.x, xy.y) << endl;
      }
      cout << "--------------------------" << endl;
      for(size_t i = 0; i < ret.size(); ++i) {
         cout << format("{:03d} => {:03d}", i, ret[i]) << endl;
      }

      FATAL("kBAM!");
   }

   return ret;
}

// ----------------------------------------------------------------------- Pimpl

static cv::Mat empty_mat;

struct This::Pimpl
{
   vector<unique_ptr<cv::VideoCapture>> videos;

   // If any files are on s3, then we need to load them
   // locally in order to open them in a cv::VideoCapture object.
   // We later want to delete these files
   vector<string> to_del_filename;

   mutable SpinLock padlock;

   // ---- Conversions from sensors to cameras
   struct BCamIndex
   {
      BCamIndex(unsigned cam_ind = 0, unsigned idx = 0)
          : camera_ind(cam_ind)
          , index(idx)
      {}
      unsigned camera_ind{0}; // index of bcam
      unsigned index{0};      // index of sensor within bcam
   };

   vector<BCamIndex> camera_lookup;    // sensor => bcam, ind
   hashmap<Point2, int> sensor_lookup; // bcam, ind => sensor
   vector<int> movie_frame_offsets;
   vector<int> movie_frame_counts;

   /// 'video_movie_frame_no' the frame number when accessing the 'vidoes'
   mutable int video_movie_frame_no{0}; // frame-no when reading video data

   int n_movie_frames{0}; // number of movie-frames
   int n_scene_frames{0};

   real movie_fps{0.0};            // fps of INPUT movies
   vector<unsigned> frame_mapping; // Maps a scene-frame to a movie-frame

   bool has_current_frame{false};
   int current_scene_frame_no{0}; // for image files in scene-description

   InitOptions init_opts = {};

   using MatrixXi8 = Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic>;

   mutable MatrixXi8 accumulated_frames;
   mutable vector<Vec3fImage> accumulated_blurred_lab;
   mutable vector<unique_ptr<SpinLock>> accumulater_padlocks;
   mutable vector<int> accumulator_counters;

   std::once_flag floor_grid_flag;
   shared_ptr<const vector<ARGBImage>> floor_grids_ptr = nullptr;

   void accumulate_sensor(const cv::Mat& im, int frame_no, int sensor_no) const
   {
      {
         lock_guard lock(padlock);
         Expects(unsigned(frame_no) < accumulated_frames.rows());
         Expects(unsigned(sensor_no) < accumulated_frames.cols());
         Expects(size_t(sensor_no) < accumulated_blurred_lab.size());
         if(accumulated_frames(frame_no, sensor_no)) return;
         accumulated_frames(frame_no, sensor_no) = 1;
      }

      auto lab_blurred = [](const cv::Mat& im) {
         cv::Mat blurred{im.rows, im.cols, im.type()};
         cv::blur(im, blurred, cv::Size(5, 5));
         return argb_to_LAB_vec3f_im(cv_to_argb(blurred));
      };
      const Vec3fImage blurred = lab_blurred(im);
      auto& dest               = accumulated_blurred_lab[size_t(sensor_no)];

      {
         lock_guard lock(*accumulater_padlocks[size_t(sensor_no)]);
         if(dest.size() == 0) {
            dest.resize(unsigned(im.cols), unsigned(im.rows));
            dest.fill(Vector3f{0.0f, 0.0f, 0.0f});
         }

         Expects(int(dest.width) == im.cols);
         Expects(int(dest.height) == im.rows);
         dest += blurred;
         accumulator_counters.at(size_t(sensor_no)) += 1;
      }
   }

   int get_movie_frame_no(int scene_frame_no) noexcept
   {
      if(unsigned(scene_frame_no) < unsigned(frame_mapping.size()))
         return int(frame_mapping[size_t(scene_frame_no)]);
      return -1;
   }

   Pimpl() {}

   ~Pimpl()
   {
      for(const auto& fname : to_del_filename)
         if(fname.size() > 0) delete_file(fname);
   }

   void calc_floor_grids_(This* o)
   {
      auto& ptr = floor_grids_ptr;
      std::call_once(floor_grid_flag, [this, o, &ptr]() {
         auto p = make_shared<vector<ARGBImage>>(o->get_floor_grids());
         {
            lock_guard lock(padlock);
            ptr = p;
         }
         Expects(ptr->size() == size_t(o->n_sensors()));
      });
   }

   const ARGBImage& floor_grid(int sensor_ind)
   {
      bool first = true;

      auto get_ptr = [&]() {
         lock_guard lock(padlock);
         return floor_grids_ptr.get();
      };

      while(get_ptr() == nullptr) {
         if(first) WARN(format("floor grids being calculated..."));
         first = false;
         std::this_thread::yield();
      }

      Expects(size_t(sensor_ind) < floor_grids_ptr->size());
      return get_ptr()->at(size_t(sensor_ind));
   }
};

// ---------------------------------------------------------------- Construction

This::This()
    : pimpl_(make_unique<Pimpl>())
{}
This::~This() = default;

Timestamp This::current_timestamp() const noexcept
{
   return add_seconds(timestamp0, current_frame_no() / scene_fps);
}

int This::current_frame_no() const noexcept
{
   return pimpl_->current_scene_frame_no;
}

int This::n_frames() const noexcept { return pimpl_->n_scene_frames; }

bool This::has_current_frame() const noexcept
{
   return pimpl_->has_current_frame;
}

int This::find_camera(const string_view in_key) const noexcept
{
   string key = strip_suffix(string(in_key), '_');
   auto ii    = std::find_if(
       cbegin(bcam_infos), cend(bcam_infos), [&](const auto& bi) {
          return key == bi.camera_id or in_key == bi.camera_id;
       });
   if(ii == cend(bcam_infos)) return -1;
   return int(std::distance(cbegin(bcam_infos), ii));
}

int This::sensor_lookup(int bcam_index, int index) const noexcept
{
   auto ii = pimpl_->sensor_lookup.find(Point2(bcam_index, index));
   if(ii != cend(pimpl_->sensor_lookup)) return ii->second;
   return -1;
}

Point2 This::bcam_lookup(int sensor_index) const noexcept
{
   if(unsigned(sensor_index) < pimpl_->camera_lookup.size()) {
      auto x = pimpl_->camera_lookup[size_t(sensor_index)];
      return Point2(int(x.camera_ind), int(x.index));
   }
   return Point2(-1, -1);
}

std::pair<int, int> This::bcam_lookup2(int sensor_index) const noexcept
{
   const auto xx = bcam_lookup(sensor_index);
   return std::make_pair(xx.x, xx.y);
}

int This::sensor_position(int sensor_index) const noexcept
{
   return bcam_lookup(sensor_index).y;
}

vector<cv::Mat> This::grab_video_frame_(int scene_frame_no) const noexcept
{
   static ParallelJobSet pjobs;

   auto& P = *pimpl_;

   const auto sz_n_cams = size_t(n_cameras());
   vector<cv::Mat> ret(sz_n_cams);
   vector<int> fnums(sz_n_cams);

   // Get the mapping from the scene-frame to the movie-frame
   const int movie_frame_no = P.get_movie_frame_no(scene_frame_no);
   if(movie_frame_no < 0) {
      ret.clear();
      return ret;
   }

   auto process_i = [&](const unsigned i, const bool advance_one) {
      fnums[i] = -1;
      try {
         if(is_video[i]) {
            Expects(P.videos[i]);
            if(false && advance_one) {
               *P.videos[i] >> ret[i];
            } else {
               int fnum = movie_frame_no + P.movie_frame_offsets[i];
               fnums[i] = fnum;
               if(fnum >= 0 and fnum < P.movie_frame_counts[i]) {
                  P.videos[i]->set(cv::CAP_PROP_POS_FRAMES, fnum);
                  P.videos[i]->retrieve(ret[i]);
               } else {
                  LOG_ERR(
                      format("Attempt to get scene frame {} -> {}+{} = {} on "
                             "camera {}, but "
                             "that video stream has frames [{}..{}]. Scene as "
                             "{} scene frames.",
                             scene_frame_no,
                             movie_frame_no,
                             P.movie_frame_offsets[i],
                             fnum,
                             i,
                             0,
                             P.movie_frame_counts[i],
                             P.n_scene_frames));
                  P.videos[i] = {};
               }
            }
         }
      } catch(std::exception& e) {
         FATAL(format("uncaught exception decoding video data: {}", e.what()));
      }
   };

   try {
      lock_guard lock(P.padlock);

      const bool advance_one // ie., advance one MOVIE frame
          = (movie_frame_no > 1
             and movie_frame_no == P.video_movie_frame_no + 1);

      for(auto i = 0; i < n_cameras(); ++i)
         pjobs.schedule([i, advance_one, &process_i]() {
            process_i(unsigned(i), advance_one);
         });
      pjobs.execute_non_parallel();

      bool has_error = false;
      for(auto i = 0; i < n_cameras(); ++i)
         if(ret[size_t(i)].empty()) has_error = true;

      P.video_movie_frame_no = movie_frame_no;
      if(has_error) ret.clear();

      if(false && multiview_trace_mode()) {
         TRACE(format("load scene-frame-#: {} ==> movie-frame-#:{}. "
                      "Success: {}. md5: {}. fnums: {}",
                      scene_frame_no,
                      movie_frame_no,
                      str(!has_error),
                      hexdigest(ret),
                      implode(cbegin(fnums), cend(fnums), ", ")));
      }

   } catch(std::exception& e) {}

   return ret;
}

// Advance on SCENE frame
bool This::advance_one_frame_()
{
   auto& P = *pimpl_;

   const int scene_frame_no = P.current_scene_frame_no + 1;
   auto raw                 = grab_video_frame_(scene_frame_no);
   const bool ret           = int(raw.size()) == n_cameras();

   if(ret) {
      bool has_one_video     = false;
      const size_t sz_n_cams = size_t(n_cameras());
      for(size_t i = 0; i < sz_n_cams; ++i) {
         if(is_video[i]) {
            Expects(P.videos[i]);

            // Move the current frame to the previous frame
            raw_prev_frame[i] = std::move(raw_images[i]);
            raw_images[i]     = std::move(raw[i]);
            has_one_video     = true;
         }
      }
   }

   if(ret) unpack_raw_images(true, scene_frame_no);
   P.has_current_frame = ret;

   return ret;
}

bool This::seek_frame_(int scene_frame_no)
{
   auto& P = *pimpl_;

   const bool advance_one
       = P.has_current_frame and (scene_frame_no > 1)
         and (scene_frame_no == P.current_scene_frame_no + 1);
   if(advance_one) return advance_one_frame_();

   auto prev           = grab_video_frame_(scene_frame_no - 1);
   auto raw            = grab_video_frame_(scene_frame_no);
   const bool has_prev = int(prev.size()) == n_cameras();
   const bool ret      = int(raw.size()) == n_cameras();

   const size_t sz_n_cams = size_t(n_cameras());
   for(size_t i = 0; i < sz_n_cams; ++i) {
      if(is_video[i]) {
         Expects(P.videos[i]);
         if(has_prev) {
            raw_prev_frame[i] = std::move(prev[i]);
         } else {
            raw_prev_frame[i] = empty_mat;
         }
         raw_images[i] = std::move(raw[i]);
      }
   }

   if(ret) unpack_raw_images(false, scene_frame_no);
   P.has_current_frame = ret;

   return ret;
}

This::FrameImages This::get_images_for_frame(int scene_frame_no) const noexcept
{
   auto& P = *pimpl_;
   FrameImages ret;
   ParallelJobSet pjobs;

   const size_t sz_n_sensors = size_t(n_sensors());

   ret.frame_no   = scene_frame_no;
   ret.raw_images = grab_video_frame_(scene_frame_no);
   ret.sensor_images.resize(sz_n_sensors);
   ret.labs.resize(sz_n_sensors);
   ret.frame_loaded = int(ret.raw_images.size()) == n_cameras();

   if(ret.frame_loaded) {
      auto process_cam_i = [&](const int i) {
         const auto n_sensors = bcam_infos[size_t(i)].n_sensors();
         Expects(n_sensors == 2);
         const auto ind0 = sensor_lookup(i, 0);
         const auto ind1 = sensor_lookup(i, 1);
         Expects(ind0 >= 0 and ind0 < this->n_sensors());
         Expects(ind1 >= 0 and ind1 < this->n_sensors());

         hsplit(ret.raw_images[size_t(i)],
                ret.sensor_images[size_t(ind0)],
                ret.sensor_images[size_t(ind1)]);

         if(P.init_opts.no_stereo) { // Delete the right sensor
            ret.sensor_images[size_t(ind1)] = empty_mat;
         }

         P.accumulate_sensor(
             ret.sensor_images[size_t(ind0)], scene_frame_no, ind0);
         if(!P.init_opts.no_stereo)
            P.accumulate_sensor(
                ret.sensor_images[size_t(ind1)], scene_frame_no, ind1);
      };

      for(auto i = 0; i < n_cameras(); ++i)
         pjobs.schedule([&process_cam_i, i]() { process_cam_i(i); });
      pjobs.execute();

      if(false) {
         for(auto i = 0; i < n_sensors(); ++i) {
            cv::imwrite(format("/tmp/{}.png", sensor_ids[size_t(i)]),
                        ret.sensor_images[size_t(i)]);
         }
      }
   }

   return ret;
}

bool This::is_no_stereo() const noexcept { return pimpl_->init_opts.no_stereo; }

// Every frame that's read is accumulated into buffer... one sensor
// per camera. (i.e., sensor zero).
Vec3fImage This::accum_blurred_LAB(int sensor_no) const noexcept
{
   Expects(sensor_no >= 0 && sensor_no < n_sensors());
   auto& P = *pimpl_;

   Vec3fImage out;
   int count = 0;
   {
      lock_guard lock(*P.accumulater_padlocks[size_t(sensor_no)]);
      out   = P.accumulated_blurred_lab.at(size_t(sensor_no));
      count = P.accumulator_counters.at(size_t(sensor_no));
   }

   if(count > 0) {
      const float count_inv = 1.0f / float(count);
      const auto end        = out.end();
      for(auto ii = out.begin(); ii != out.end(); ++ii) *ii *= count_inv;
   }

   return out;
}

// ----------------------------------------------------------- unpack-raw-images
// If 'is_advance_one_frame' is TRUE, then we're unpacking after
// advancing one frame, which means we can copy the sensor frames
// to the previous sensor frames
void This::unpack_raw_images(bool is_advance_one_frame, int scene_frame_no)
{
   auto& P                  = *pimpl_;
   P.current_scene_frame_no = scene_frame_no;

   if(sensor_image.size() != size_t(n_sensors()))
      sensor_image.resize(size_t(n_sensors()));
   if(sensor_previous.size() != size_t(n_sensors()))
      sensor_previous.resize(size_t(n_sensors()));

   const size_t sz_n_cams = size_t(n_cameras());
   for(size_t i = 0; i < sz_n_cams; ++i) {
      const auto n_sensors = bcam_infos[i].n_sensors();

      if(n_sensors == 1) {
         const size_t ind = size_t(sensor_lookup(int(i), 0));
         Expects(ind < size_t(this->n_sensors()));
         sensor_image[ind]    = raw_images[i];
         sensor_previous[ind] = raw_prev_frame[i];
         continue;
      }

      if(n_sensors == 2) {
         auto test_ind = [&](int sensor_no) {
            const auto& bcam_info = bcam_infos[i];
            const auto ind        = sensor_lookup(int(i), sensor_no);
            if(ind >= 0 and ind < this->n_sensors()) {
               // All good
            } else {
               FATAL(format("bcam '{}', sensor-number {} => index {}",
                            bcam_info.camera_id,
                            sensor_no,
                            ind));
            }
            return ind;
         };

         const size_t ind0 = size_t(test_ind(0));
         const size_t ind1 = size_t(test_ind(1));

         // Set 'sensor-previous' images...
         if(raw_prev_frame[i].empty()) {
            sensor_previous[ind0] = empty_mat;
            sensor_previous[ind1] = empty_mat;
         } else if(is_advance_one_frame) { // We're advancing 1 frame
            sensor_previous[ind0] = std::move(sensor_image[ind0]);
            sensor_previous[ind1] = std::move(sensor_image[ind1]);
         } else {
            hsplit(raw_prev_frame[i], // We gotta split
                   sensor_previous[ind0],
                   sensor_previous[ind1]);
            if(P.init_opts.no_stereo) sensor_previous[ind1] = empty_mat;
         }

         // Split the raw-image into the two sensor images
         hsplit(raw_images[i], sensor_image[ind0], sensor_image[ind1]);
         if(P.init_opts.no_stereo) sensor_image[ind1] = empty_mat;

         continue;
      }

      FATAL(format("SORRY, have to write code here to handle more than "
                   "two sensors!"));
   }
}

// ------------------------------------------------------------------------ init

void This::init(const SceneManifest& SM,
                const InitOptions& opts) noexcept(false)
{
   const auto now = tick();

   const DataSource source           = opts.source;
   const string optional_search_path = opts.optional_search_path;
   const bool verbose                = true; // opts.verbose;

   if(verbose) INFO(format("[{}s] init scene-description", tock(now)));

   ParallelJobSet pjobs;

   { // clear everything out
      This tmp;
      *this = std::move(tmp);
   }

   auto& P = *pimpl_;

   P.init_opts = opts;

   { // ---- Ensure cuda is initialized ---- (idempotent)
      if(perceive::cuda::cuda_is_available()) perceive::cuda::init_cuda();
   }

   { // ---- Load scene     ----
      TRACE(format("fetch scene: {}", SM.scene_key));
      fetch(scene_info, SM.scene_key, source);
      if(scene_info.scene_key != SM.scene_key) {
         WARN(format("Loaded scene '{}', however, that particular scene "
                     "thinks that it's key is '{}'. This is almost "
                     "certainly a mistake. Please update the scene file. "
                     "<multiview> will use '{}'",
                     SM.scene_key,
                     scene_info.scene_key,
                     SM.scene_key));
         scene_info.scene_key = SM.scene_key;
      }
   }
   this->epoch = SM.epoch;

   vector<unsigned> SM_map(SM.size());
   { // ---- Create mapping for manifest => scene ----
      if(SM.size() != scene_info.bcam_keys.size())
         throw std::runtime_error(format("Manfest file had {} cameras, but "
                                         "the scene ({}) had {} cameras!",
                                         SM.size(),
                                         scene_info.scene_key,
                                         scene_info.bcam_keys.size()));

      for(unsigned i = 0u; i < SM.size(); ++i) {
         auto ii = find_bcam_key(cbegin(scene_info.bcam_keys),
                                 cend(scene_info.bcam_keys),
                                 SM.camera_ids[i]);
         if(ii == cend(scene_info.bcam_keys))
            throw std::runtime_error(format("Failed to find camera '{}' in "
                                            "scene '{}'",
                                            SM.camera_ids[i],
                                            scene_info.scene_key));
         auto pos  = unsigned(std::distance(cbegin(scene_info.bcam_keys), ii));
         SM_map[i] = pos;

         Ensures(
             SM.camera_ids[i] == scene_info.bcam_keys[SM_map[i]]
             or SM.camera_ids[i]
                    == remove_version_number(scene_info.bcam_keys[SM_map[i]]));
      }
   }

   { // ---- Load CAD model ----
      if(scene_info.cad_model_key != "") {
         if(verbose) INFO(format("[{}s] load CAD model", tock(now)));
         auto sprite_ptr = make_shared<Sprite>();
         fetch(*sprite_ptr, scene_info.cad_model_key, source);
         sprite_ptr->apply_transform(scene_info.cad_model_transform);
         cad_model_ptr = sprite_ptr;
         cad_model_key = scene_info.cad_model_key;
      }
   }

   { // ---- Load bcams infos ----
      if(verbose) INFO(format("[{}s] loading binocular cams", tock(now)));
      bcam_infos.resize(scene_info.bcam_keys.size());

      auto load_bcam_info = [&](unsigned i) {
         const std::string key = scene_info.bcam_keys[i];
         fetch(bcam_infos[i], key, source);
      };

      if(false) {
         for(auto i = 0u; i < bcam_infos.size(); ++i) { load_bcam_info(i); }
      } else {
         for(auto i = 0u; i < bcam_infos.size(); ++i)
            pjobs.schedule([load_bcam_info, i]() { load_bcam_info(i); });
         pjobs.execute();
      }

      cam_transforms = scene_info.bcam_transforms;

      // Sensor-ids must be initialized from bcam_infos...
      for(auto j = 0u; j < bcam_infos.size(); ++j) {
         const auto& bcam = bcam_infos[j];
         const auto& et0  = cam_transforms[j];
         const auto et1   = bcam.make_et1(et0);

         if(false) {
            FATAL(format(
                "{}",
                str(et0.apply(Vector3(0.0, 0.0, -1.0)) - et0.translation)));
         }

         auto has_sensor_id = [&](const string& sensor_id) {
            auto ii
                = std::find(cbegin(sensor_ids), cend(sensor_ids), sensor_id);
            return ii != cend(sensor_ids);
         };

         auto safe_push_sensor_id = [&](string sensor_id) {
            if(has_sensor_id(sensor_id)) {
               vector<string> found_bcams;
               found_bcams.reserve(bcam_infos.size());
               for(const auto& bb : bcam_infos)
                  if(bb.M[0].sensor_id() == sensor_id
                     or bb.M[1].sensor_id() == sensor_id)
                     found_bcams.push_back(bb.camera_id);
               FATAL(format(
                   "duplicate sensor-id: id '{}' found in bcams [{}]",
                   sensor_id,
                   implode(cbegin(found_bcams), cend(found_bcams), ", ")));
            } else {
               sensor_ids.emplace_back(std::move(sensor_id));
            }
         };

         if(bcam.n_sensors() == 2) {
            safe_push_sensor_id(bcam.M[0].sensor_id());
            safe_push_sensor_id(bcam.M[1].sensor_id());
            sensor_transforms.push_back(et0);
            sensor_transforms.push_back(et1);
         } else {
            Expects(false);
         }
      }
   }

   { // ---- Load aruco-result
      if(verbose) INFO(format("[{}s] loading Aruco result", tock(now)));
      if(scene_info.aruco_result_key != "") {
         ArucoResultInfo ainfo;
         fetch(ainfo, scene_info.aruco_result_key, source);
         sensor_ids       = ainfo.sensor_keys;
         aruco_transforms = ainfo.global_aruco_transforms();

      } else {
         aruco_transforms = sensor_transforms;
      }

      auto find_sensor_in_bcams = [&](const string& s) -> Pimpl::BCamIndex {
         for(auto bcam_ind = 0u; bcam_ind < bcam_infos.size(); ++bcam_ind) {
            const auto& bcam = bcam_infos[bcam_ind];
            for(auto cam_idx = 0u; cam_idx < bcam.M.size(); ++cam_idx)
               if(bcam.M[cam_idx].sensor_id() == s)
                  return Pimpl::BCamIndex(bcam_ind, cam_idx);
         }
         throw std::runtime_error(format("failed to find sensor '{}' in "
                                         "camera data for scene.",
                                         s));
      };

      auto& P = *pimpl_;
      P.camera_lookup.resize(sensor_ids.size());
      for(auto i = 0u; i < sensor_ids.size(); ++i) {
         // Find the relavent bcam sensor...
         auto bcam_ind      = find_sensor_in_bcams(sensor_ids[i]);
         P.camera_lookup[i] = bcam_ind;
         P.sensor_lookup[Point2(int(bcam_ind.camera_ind), int(bcam_ind.index))]
             = int(i);
      }

      // Ensure that every bcam-index has a sensor
      for(size_t i = 0; i < size_t(n_cameras()); ++i)
         for(size_t j = 0; j < size_t(bcam_infos[i].n_sensors()); ++j)
            if(sensor_lookup(int(i), int(j)) < 0)
               throw std::runtime_error(format("failed to find sensor "
                                               "'{}'",
                                               bcam_infos[i].M[j].sensor_id()));
   }

   { // ---- Init the raw image information
      const auto SM_filenames
          = SM.resolve_video_filenames(optional_search_path);
      filenames.resize(SM_filenames.size());
      for(auto i = 0u; i < filenames.size(); ++i)
         filenames[SM_map[i]] = SM_filenames[i];
      is_video.resize(filenames.size());
      raw_images.resize(filenames.size());
      raw_prev_frame.resize(filenames.size());
      sensor_image.resize(sensor_ids.size());

      for(auto i = 0u; i < filenames.size(); ++i) {
         const auto& fname = filenames[i];
         if(fname.empty())
            throw std::runtime_error(format("found empty file for camera '{}'",
                                            scene_info.bcam_keys[i]));
      }
   }

   { // ---- What is a video file, and what is an image file?
      constexpr array<const char*, 2> img_exts{{".png", ".jpg"}};
      constexpr array<const char*, 2> vid_exts{{".mp4", ".h264"}};
      auto it_is = [&](const auto& arr, string s) {
         std::transform(s.begin(), s.end(), s.begin(), ::tolower);
         for(const auto& p : arr)
            if(strcmp(&s[0], p) == 0) return true;
         return false;
      };

      for(auto i = 0u; i < filenames.size(); ++i) {
         const auto ext = file_ext(filenames[i]);
         auto is_image  = it_is(img_exts, ext);
         auto is_video  = it_is(vid_exts, ext);

         if(not is_image and not is_video)
            throw std::runtime_error(format("file '{}' appears to be "
                                            "neither a video nor an image",
                                            filenames[i]));

         Expects(not(is_image and is_video));
         this->is_video[i] = is_video;
      }
   }

   { // ---- Load up video files
      if(verbose) INFO(format("[{}s] loading video files", tock(now)));
      P.videos.resize(filenames.size());
      P.to_del_filename.resize(filenames.size());
      // std::fill(begin(P.to_del_filename), end(P.to_del_filename),
      // string(""));

      for(auto i = 0u; i < filenames.size(); ++i) {
         string fname     = filenames[i];
         const bool is_s3 = begins_with(fname, string("s3://"));
         if(is_s3) {
            // Copy to /tmp
            INFO(format("fetching {}", fname));
            std::vector<char> data;
            lazy_s3_load(filenames[i], data);
            // P.to_del_filename[i] = format("/tmp/{}", basename(fname));
            fname = lazy_s3_cache_filename(filenames[i]);
            if(!is_regular_file(fname))
               FATAL(format("something went wrong with lazy-s3 cache: couldn't "
                            "find s3-cache file '{}'",
                            fname));
            // file_put_contents(fname, data);
         }

         if(is_video[i])
            P.videos[i] = make_unique<cv::VideoCapture>(fname);
         else
            raw_images[i] = cv::imread(fname, cv::IMREAD_COLOR);

         if(is_video[i] and not P.videos[i]->isOpened())
            throw std::runtime_error(
                format("failed to load '{}'.", filenames[i]));
      }
   }

   { // ---- Set up frame counts, timestamp0, frame offsets,
      // TRACE(format("[{}s] Set frame counts", tock(now)));
      P.movie_frame_counts.resize(size_t(n_cameras()));
      std::fill(begin(P.movie_frame_counts), end(P.movie_frame_counts), 0);
      for(size_t i = 0; i < size_t(n_cameras()); ++i) {
         if(!is_video[i]) continue;
         P.movie_frame_counts[i]
             = int(P.videos[i]->get(cv::CAP_PROP_FRAME_COUNT));
         if(P.movie_frame_counts[i] == 0)
            throw std::runtime_error(format("could not get frame-count "
                                            "from video file '{}'",
                                            basename(filenames[i])));
      }

      // movie-fps
      P.movie_fps
          = (SM.fps.size() > 0) ? SM.fps[0] : k_perceive_default_frame_rate;

      // timestamp0
      if(SM.timestamps.size() > 0)
         timestamp0
             = *std::max_element(cbegin(SM.timestamps), cend(SM.timestamps));

      // offsets
      P.movie_frame_offsets.resize(size_t(n_cameras()));
      std::fill(begin(P.movie_frame_offsets), end(P.movie_frame_offsets), 0);
      for(size_t i = 0; i < size_t(n_cameras()); ++i) {
         if(!is_video[i]) continue;
         P.movie_frame_offsets[SM_map[i]] = calc_movie_frame_offset(
             P.movie_fps, timestamp0, SM.timestamps[i]);
         Expects(P.movie_frame_offsets[SM_map[i]] >= 0);
      }

      // n-movie frames
      auto calc_n_movie_frames = [&]() {
         int n_frames = std::numeric_limits<int>::max();
         for(size_t i = 0; i < size_t(n_cameras()); ++i) {
            if(!is_video[i]) continue;
            int count = P.movie_frame_counts[i] - P.movie_frame_offsets[i];
            Expects(count >= 0);
            if(count < n_frames) n_frames = count;
         }
         return (n_frames == std::numeric_limits<int>::max()) ? 1 : n_frames;
      };
      P.n_movie_frames = calc_n_movie_frames();
   }

   { // ---- Set the scene frame mapping
      scene_fps       = (opts.target_fps > 0.0) ? opts.target_fps : P.movie_fps;
      P.frame_mapping = scene_frame_to_movie_frame_mapping(
          P.movie_fps, scene_fps, P.n_movie_frames);
      P.n_scene_frames = int(P.frame_mapping.size());
   }

   if(P.n_scene_frames > 0) { // ---- Sanity
      bool has_error                 = false;
      auto check_can_get_scene_frame = [&](const int scene_frame_no) {
         for(size_t i = 0; i < size_t(n_cameras()); ++i) {
            try {
               if(!is_video.at(i)) continue;
               const int movie_frame_no = P.get_movie_frame_no(scene_frame_no);
               const int fnum = movie_frame_no + P.movie_frame_offsets[i];
               if(fnum >= 0 and fnum < P.movie_frame_counts[i]) {
                  // All is good
               } else {
                  LOG_ERR(
                      format("Attempt to get video frame {} -> {}+{} = {} on "
                             "camera {}, but "
                             "that video stream has frames [{}..{}]",
                             scene_frame_no,
                             movie_frame_no,
                             P.movie_frame_offsets[i],
                             fnum,
                             i,
                             0,
                             P.movie_frame_counts[i]));
                  has_error = true;
               }
            } catch(std::exception& e) {
               LOG_ERR(format("uncaught exception decoding video data: {}",
                              e.what()));
               has_error = true;
            }
         }
      };

      check_can_get_scene_frame(0);
      check_can_get_scene_frame(P.n_scene_frames - 1);

      if(has_error) FATAL("Aborting");
   }

   { // ---- Setup image accumulation
      P.accumulated_frames
          = Pimpl::MatrixXi8::Zero(P.n_scene_frames, n_sensors());
      P.accumulator_counters.resize(size_t(n_sensors()), 0);
      P.accumulated_blurred_lab.resize(size_t(n_sensors()));
      P.accumulater_padlocks.reserve(size_t(n_sensors()));
      for(auto i = 0; i < n_sensors(); ++i)
         P.accumulater_padlocks.emplace_back(make_unique<SpinLock>());
   }

   { // ---- Load current frame
      seek_frame_(0);
   }

   { // ---- Set the working image format
      for(size_t i = 0; i < bcam_infos.size(); ++i) {
         const auto& im0 = sensor_image[size_t(sensor_lookup(int(i), 0))];
         Expects(!im0.empty());
         for(size_t j = 0; j < size_t(bcam_infos[i].n_sensors()); ++j) {
            const auto& im
                = sensor_image[size_t(sensor_lookup(int(i), int(j)))];
            if(im.empty()) {
               bcam_infos[i].M[j].set_working_format(unsigned(im0.cols),
                                                     unsigned(im0.rows));
            } else {
               bcam_infos[i].M[j].set_working_format(unsigned(im.cols),
                                                     unsigned(im.rows));
            }
         }
      }
   }

   { // ---- Load caching undistort inverse objects...
      if(verbose) INFO(format("[{}s] load caching undistort files", tock(now)));
      cu_.clear();
      cu_.resize(size_t(n_sensors()));

      // bcam_infos[i].M[j].set_working_format(im.cols, im.rows);

      auto process_i = [&](const int i) {
         const auto& im = sensor_image[size_t(i)];
         cu_[size_t(i)].init(this->model(i));
         cu_[size_t(i)].set_working_format(unsigned(im.cols),
                                           unsigned(im.rows));
      };

      for(auto i = 0; i < n_sensors(); ++i)
         pjobs.schedule([i, &process_i]() { process_i(i); });
      pjobs.execute();
   }

   { // ---- Create distorted cameras
      if(verbose) INFO(format("[{}s] creating distorted cameras", tock(now)));
      dcams.resize(size_t(n_cameras()));

      auto process_dcam_i = [&](const int bcam_ind) {
         auto& dcam = dcams[size_t(bcam_ind)];
         dcam.clear();
         const size_t N = size_t(n_sensors_for(bcam_ind));
         if(N == 0) return; // Seriously??

         dcam.resize(N);
         const auto& et0 = cam_transforms[size_t(bcam_ind)];

         auto get_et = [&](const size_t i) {
            if(i == 0) return et0;
            if(i == 1) return bcam_infos[size_t(bcam_ind)].make_et1(et0);
            FATAL(format("only supports monocular and binocular cameras!"));
            return et0;
         };

         for(size_t i = 0; i < N; ++i) {
            const int sensor_ind = sensor_lookup(bcam_ind, int(i));
            const auto& im       = sensor_image[size_t(sensor_ind)];
            const unsigned w     = unsigned(im.cols);
            const unsigned h     = unsigned(im.rows);
            dcam[i] = make_distorted_camera(get_et(i), cu(sensor_ind), w, h);
         }
      };

      for(auto i = 0; i < n_cameras(); ++i)
         pjobs.schedule([i, &process_dcam_i]() { process_dcam_i(i); });
      pjobs.execute();
   }

#ifdef USING_OPENGL
   if(verbose) INFO(format("[{}s] setting up opengl", tock(now)));
   ensure_glew_and_gl_setup();
   stock_gl_setup(fbo_w, fbo_h);
   check_and_warn_on_gl_error("after setting up gl-context");
#endif

   // ---- Create depth maps for each sensor view
   if(!cad_model_ptr) {
      // Create a stack of empty images
      depth_buffers.resize(sensor_image.size());

   } else {
      const auto& cad  = *cad_model_ptr;
      const auto z_far = (cad.min_xyz() - cad.max_xyz()).norm();
      if(!std::isfinite(z_far))
         FATAL(format("z-far wasn't finite, CAD.min = {}, CAD.max = {}",
                      str(cad.min_xyz()),
                      str(cad.max_xyz())));

#ifdef USING_OPENGL
      // ensure_glew_and_gl_setup();
      // stock_gl_setup(fbo_w, fbo_h);
      // check_and_warn_on_gl_error("before making display-list");
      const auto display_list_index = make_model_displaylist(cad);

      check_and_warn_on_gl_error("before making FBO");
      auto fbo_ptr = make_unique<FramebufferObject>(fbo_w, fbo_h);
      check_and_warn_on_gl_error("after making FBO");

      // Create the depth maps for each image
      auto set_depth_im = [&](const unsigned i) {
         const auto& im = sensor_image[i];
         auto& buffer   = depth_buffers[i];

         check_and_warn_on_gl_error("before binding FBO");
         fbo_ptr->bind();
         check_and_warn_on_gl_error("after binding FBO");

         // Set up the viewport, and projection matrix
         buffer.port.init(0.0, 0.0, fbo_w, fbo_h, fbo_hfov, 0.100, z_far);
         buffer.port.set_cam_et_from_perceive_cam_et(sensor_transforms[i]);

         // INFO(format("sensor = {}, et = {}",
         //             sensor_ids[i],
         //             str(sensor_transforms[i])));

         // Clear everything that was there
         glClearColor(0.0, 0.0, 0.0, 0.0);
         glClearDepth(1.0);
         glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

         buffer.port.gl_projection_apply();
         buffer.port.gl_modelview_apply();

         // Now render
         check_and_warn_on_gl_error("before rendering CAD model");
         glCallList(display_list_index);
         check_and_warn_on_gl_error("after rendering CAD model");
         glFinish();

         // Read back the z-buffer
         buffer.z_buffer.read_from_fbo(buffer.port, *fbo_ptr);

         GreyImage z_im = buffer.z_buffer.to_grey_image();
         z_im.save(format("/tmp/z-buffer_{:2d}.png", i));
      };

      depth_buffers.resize(sensor_image.size());
      for(auto i = 0u; i < depth_buffers.size(); ++i) set_depth_im(i);

      destroy_model_displaylist(display_list_index);
#endif
   }

   { // ---- load reference disparities ----
      ref_disparities.resize(bcam_infos.size());
   }

   if(false) { // ---- load reference disparities ----
      if(verbose)
         INFO(format("[{}s] loading reference disparities", tock(now)));
      ref_disparities.resize(bcam_infos.size());

      bool has_error = false;
      for(auto i = 0u; i < bcam_infos.size(); ++i) {
         string key
             = format("{}_{}", scene_info.scene_key, bcam_infos[i].camera_id);
         FloatImage fim;
         fim.resize(0, 0);

         try {
            if(!opts.no_stereo) fetch(fim, key);
         } catch(std::runtime_error& e) {
            WARN(format("failed to load reference disparity: {}", key));
            fim.resize(0, 0);
         }
         ref_disparities[i] = float_im_to_cv(fim);

         if(false) {
            const auto x = Point2(234, 205);
            WARN(format("bcam = {}, [{}x{}], I({}, {}) = {}",
                        bcam_infos[i].camera_id,
                        fim.width,
                        fim.height,
                        x.x,
                        x.y,
                        fim(x)));
         }
      }

      if(has_error) {
         WARN(format("failed to load reference disparities... hopefully you're "
                     "calibrating"));
         ref_disparities.clear();
      }
   }

   { // ---- load background image ----
      if(verbose) INFO(format("[{}s] loading background image", tock(now)));
      if(!scene_info.background_image.empty()) {
         try {
            fetch_bg_image(background_image, scene_info.background_image);
#ifdef USING_OPENGL
            background_image_GL          = make_background_image_GL();
            auto& gl_im                  = background_image_GL;
            background_image_h_tex_coord = real(background_image.height)
                                           / real(background_image_GL.height);
            background_image_w_tex_coord = real(background_image.width)
                                           / real(background_image_GL.width);
#endif
         } catch(std::exception& e) {
            FATAL(format("failed to load image '{}': {}",
                         scene_info.background_image,
                         e.what()));
         }
      }
   }

   { // ---- Asychronously calculate the floor grids
      schedule([this]() { pimpl_->calc_floor_grids_(this); });
   }

   { // ---- sanity checks ----
      if(!scene_info.hist_bounds.is_finite())
         WARN(format("no 'hist-bounds' set"));
   }

   if(verbose) INFO(format("[{}s] done loading scene, total time", tock(now)));
}

bool This::has_background_image() const noexcept
{
   return background_image.width > 0;
}

void This::init_from_json(const Json::Value& data,
                          const InitOptions& opts) noexcept(false)
{
   SceneManifest manifest;
   read(manifest, data);
   init(manifest, opts);
}

#ifdef USING_OPENGL
ARGBImage This::make_background_image_GL() const noexcept
{
   ARGBImage out;

   // background_image must have dimensions that are a power of 2
   const auto w0 = background_image.width;
   const auto h0 = background_image.height;
   const int w   = int(std::pow(2, std::ceil(std::log(w0) / std::log(2))));
   const int h   = int(std::pow(2, std::ceil(std::log(h0) / std::log(2))));

   out.resize(w, h);
   out.fill(k_black);

   for(auto y = 0u; y < h0; ++y) {
      auto dst = out.row_ptr(h0 - y - 1);
      auto src = background_image.row_ptr(y);
      for(auto x = 0u; x < w0; ++x) *dst++ = src[x];
   }

   // Now fix the alpha values
   for(auto ii = out.begin(); ii != out.end(); ++ii) *ii = 0xff000000u | *ii;

   return out;
}

unsigned int This::background_tex_id() const noexcept
{
   if(background_tex_ == 0 and has_background_image()) {
      const auto& gl_im = background_image_GL;

      glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

      unsigned int val = 0;
      glGenTextures(1, &val);
      const_cast<This*>(this)->background_tex_ = val;
      glBindTexture(GL_TEXTURE_2D, background_tex_);

      const auto w    = GLsizei(gl_im.width);
      const auto h    = GLsizei(gl_im.height);
      const auto data = gl_im.pixels;

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexImage2D(
          GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
      check_and_warn_on_gl_error("after initializing texture");
   }
   return background_tex_;
}
#endif

// ------------------------------------------------------------- get-floor-grids
//
vector<ARGBImage> This::get_floor_grids() const noexcept
{
   ParallelJobSet pjobs;
   vector<ARGBImage> floor_grids;
   floor_grids.resize(size_t(n_sensors()));

   auto process_floor_grid = [&](const size_t sensor_no) {
      const auto& dcam           = this->dcam(int(sensor_no));
      const auto& et             = this->sensor_transforms[sensor_no];
      const auto& cu             = this->cu(int(sensor_no));
      auto& argb                 = floor_grids.at(sensor_no);
      const uint32_t mask_colour = 0xff000000u;
      argb.resize(unsigned(dcam.w), unsigned(dcam.h)); // Wrong width/height
      argb.fill(mask_colour);
      draw_floor_grid_in_situ(argb, scene_info.hist_bounds, cu, et);
   };

   const int n_jobs = 2;
   auto process_n   = [&](const int n) {
      for(auto i = 0; i < n_sensors(); ++i)
         if(i % n_jobs == n) process_floor_grid(size_t(i));
   };

   for(auto n = 0; n < n_jobs; ++n)
      pjobs.schedule([n, &process_n]() { process_n(n); });
   pjobs.execute();
   return floor_grids;
}

// -------------------------------------------------------------- to-json-string
//
string This::to_string() const noexcept
{
   const auto& P = *pimpl_;

   auto camera_info_string = [&]() {
      std::stringstream ss("");
      for(size_t i = 0; i < size_t(n_cameras()); ++i) {
         auto vid_s = ""s;
         if(is_video[i])
            vid_s = format("[{}/{}]",
                           P.video_movie_frame_no + P.movie_frame_offsets[i],
                           P.movie_frame_counts[i]);
         ss << format(
             "{}: {}{}", bcam_infos[i].camera_id, basename(filenames[i]), vid_s)
            << endl;
      }
      return ss.str();
   };

   auto sensor_info_string = [&]() {
      std::stringstream ss("");
      for(size_t i = 0; i < size_t(n_sensors()); ++i) {
         auto p = bcam_lookup(int(i));
         ss << format("{} (bcam {} #{}) ",
                      sensor_ids[i],
                      bcam_infos[size_t(p.x)].camera_id,
                      p.y)
            << endl;
      }
      return ss.str();
   };

   auto planes_info_string = [&]() {
      std::stringstream ss("");
      for(const auto& np3 : scene_info.known_planes) {
         const Plane& p3 = np3.second;
         ss << format("{}: [{}, {}, {}, {}]", np3.first, p3.x, p3.y, p3.z, p3.w)
            << endl;
      }
      return ss.str();
   };

   auto stills_info_string = [&]() {
      return implode(cbegin(scene_info.stills),
                     cend(scene_info.stills),
                     "\n",
                     [&](auto& s) { return s; });
   };

   return format(R"V0G0N(
SceneDescription:
   scene-key:          {}
   time0:              {}
   movie-fps:          {}
   n-movie-frames:     {}
   scene-fps:          {}
   n-frames:           {}   
   n-cameras:          {}
   n-sensors:          {}  
   cad-model-key:      {}
   current-frame-no:   {}/{}
   current-timestamp:  {}
   Cameras:
{}
   Sensors:
{}
   Planes:
{}
   Stills:
{}
{})V0G0N",
                 scene_info.scene_key,
                 str(timestamp0),
                 P.movie_fps,
                 P.n_movie_frames,
                 scene_fps,
                 0,
                 n_cameras(),
                 n_sensors(),
                 cad_model_key,
                 current_frame_no(),
                 n_frames(),
                 str(current_timestamp()),
                 indent(camera_info_string(), 6),
                 indent(sensor_info_string(), 6),
                 indent(planes_info_string(), 6),
                 indent(stills_info_string(), 6),
                 "");
}

shared_ptr<SceneDescription>
load_scene_from_manifest(const Json::Value& data,
                         const SceneDescription::InitOptions& opts) noexcept
{
   shared_ptr<SceneDescription> out{nullptr};
   try {
      auto ret = make_shared<SceneDescription>();
      ret->init_from_json(data, opts);
      out = std::move(ret);
   } catch(std::exception& e) {
      LOG_ERR(format("failed to load manifest: {}", e.what()));
   }
   return out;
}

void export_movie_and_manifest(const SceneDescription& scene_desc,
                               const string_view outdir,
                               const real frame_rate,
                               const int start_frame,
                               const int n_frames)

{
   const size_t N = size_t(scene_desc.n_cameras());
   vector<string> movie_files;
   movie_files.reserve(N);

   for(size_t cam_no = 0; cam_no < N; ++cam_no) {
      const auto cam_name    = scene_desc.scene_info.bcam_keys[cam_no];
      const auto movie_fname = format("{}.mp4", cam_name);
      const auto out_file    = format("{}/{}", outdir, movie_fname);
      movie_files.push_back(movie_fname);

      int frame_no = start_frame;
      auto ims     = scene_desc.get_images_for_frame(frame_no);
      if(!ims.frame_loaded) FATAL(format("could not seek frame {}", frame_no));
      const int w = ims.raw_images[cam_no].cols;
      const int h = ims.raw_images[cam_no].rows;

      auto encoder
          = movie::StreamingFFMpegEncoder::create(out_file, w, h, frame_rate);

      do {
         encoder.push_frame(ims.raw_images[cam_no]);
         ims = scene_desc.get_images_for_frame(++frame_no);

         if(n_frames >= 0 && frame_no >= start_frame + n_frames) break;

      } while(ims.frame_loaded);

      encoder.close();
   }

   { // Now write out the manifest file
      SceneManifest m;
      m.store_id   = int(scene_desc.scene_info.store_id);
      m.scene_key  = scene_desc.scene_info.scene_key;
      m.epoch      = Timestamp{};
      m.camera_ids = scene_desc.scene_info.bcam_keys;
      m.videos     = movie_files;
      m.timestamps.resize(N);
      m.fps.resize(N);
      std::fill(begin(m.fps), end(m.fps), frame_rate);

      save(m, format("{}/manifest.json", outdir));
   }
}

int calc_movie_frame_offset(const real movie_fps,
                            const Timestamp& timestamp0,
                            const Timestamp& movie_timestamp) noexcept
{
   const auto s     = distance(movie_timestamp, timestamp0);
   const int offset = int(std::round(s * movie_fps) + 1e-9);

   // OpenCV says that frame indicies are zero index,
   // but sometimes frame [1] has the same checksum as frame [0],
   // and reading frame [0] results in the last frame being returned.
   // This is a problem when seeking frames. So we skip the first
   // frame of the video, if, indeed, OpenCVs documentation is correct.

   return offset + 1;
}

// ------------------------------------------------------------- Still Functions
//
template<typename T>
static bool images_same_size(const T& A, const T& B) noexcept
{
   return (A.width == B.width) && (A.height == B.height);
}

ARGBImage This::argb_still(int cam_no) const noexcept
{
   const int N = n_sensors_for(cam_no);
   // TRACE(format("N = {}, no-streo = {}", N, str(is_no_stereo())));
   if(N == 1 || (N == 2 && is_no_stereo())) {
      return LAB_vec3f_im_to_argb(accum_blurred_LAB(sensor_lookup(cam_no, 0)));
   } else if(N == 2) {
      auto A = accum_blurred_LAB(sensor_lookup(cam_no, 0));
      auto B = accum_blurred_LAB(sensor_lookup(cam_no, 1));
      // TRACE(
      //     format("A={}x{}, and B={}x{}", A.width, A.height, B.width,
      //     B.height));
      return images_same_size(A, B)
                 ? hcat(LAB_vec3f_im_to_argb(A), LAB_vec3f_im_to_argb(B))
                 : hcat(LAB_vec3f_im_to_argb(A), LAB_vec3f_im_to_argb(A));
   } else {
      FATAL("logic error");
   }
   return {};
}

LABImage This::LAB_still(int cam_no) const noexcept
{
   const int N = n_sensors_for(cam_no);
   //   TRACE(format("N = {}, no-streo = {}", N, str(is_no_stereo())));
   if(N == 1 || (N == 2 && is_no_stereo())) {
      return LAB_vec3f_im_to_LAB(accum_blurred_LAB(sensor_lookup(cam_no, 0)));
   } else if(N == 2) {
      auto A = accum_blurred_LAB(sensor_lookup(cam_no, 0));
      auto B = accum_blurred_LAB(sensor_lookup(cam_no, 1));

      // TRACE(
      //     format("A={}x{}, and B={}x{}", A.width, A.height, B.width,
      //     B.height));
      return images_same_size(A, B)
                 ? hcat(LAB_vec3f_im_to_LAB(A), LAB_vec3f_im_to_LAB(B))
                 : hcat(LAB_vec3f_im_to_LAB(A), LAB_vec3f_im_to_LAB(A));
   } else {
      FATAL("logic error");
   }
   return {};
}

LABImage This::sensor_LAB_still(int sensor_ind) const noexcept
{
   return LAB_vec3f_im_to_LAB(accum_blurred_LAB(sensor_ind));
}

const ARGBImage& This::floor_grid(int sensor_ind) const noexcept
{
   return pimpl_->floor_grid(sensor_ind);
}

const vector<ARGBImage>& This::all_floor_grids() const noexcept
{
   Expects(n_sensors() > 0);
   floor_grid(0); // forces calculation
   return *pimpl_->floor_grids_ptr;
}

} // namespace perceive
