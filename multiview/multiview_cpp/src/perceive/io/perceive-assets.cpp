
#include <boost/endian/conversion.hpp>

#include "ieee754-packing.hpp"
#include "perceive-assets.hpp"
#include "stdinc.hpp"

#include "perceive/calibration/aruco-cube.hpp"
#include "perceive/calibration/plane-set/phase-plane-data.hpp"
#include "perceive/cost-functions/classifier/classifier.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/geometry/projective/caching-undistort-inverse.hpp"
#include "perceive/geometry/projective/polynomial-model.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/graphics/sprite.hpp"
#include "perceive/io/lazy-s3.hpp"
#include "perceive/io/s3.hpp"
#include "perceive/scene/aruco-result-info.hpp"
#include "perceive/scene/scene-description-info.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/md5.hpp"

namespace perceive
{
static std::once_flag init_perceive_files_flag;

static DataSource source_  = DataSource::LAZY_S3;
static string s3_bucket_   = ""s;
static string mv_data_dir_ = ""s;

// ------------------------------------------------------------------ Initialize

static void run_once_init_perceive_files()
{
   bool has_error = false;

   s3_bucket_   = multiview_asset_s3_bucket();
   mv_data_dir_ = multiview_asset_dir();

   // ------------------------ Data Source
   source_            = DataSource::LAZY_S3;
   const string value = string_to_uppercase(multiview_asset_store());
   if(value == "S3"s) {
      source_ = DataSource::S3;
   } else if(value == "MULTIVIEW_ASSET_DIR"s) {
      source_ = DataSource::MULTIVIEW_ASSET_DIR;
      if(mv_data_dir_.empty() or !is_directory(mv_data_dir_)) {
         LOG_ERR(
             format("env variable 'MULTIVIEW_ASSET_DIR' set to '{}'; however, "
                    "this directory was not found.",
                    mv_data_dir_));
         has_error = true;
      } else {
         for(auto ftype : k_filetypes) {
            auto path
                = format("{}/{}", mv_data_dir_, file_type_relative_path(ftype));
            if(!is_directory(path)) {
               INFO(format("creating MULTIVIEW_ASSET_DIR '{}'", path));
               mkdir_p(path);
            }
         }
      }
   } else if(value == "LAZY_S3"s) {
      source_ = DataSource::LAZY_S3;
   } else {
      LOG_ERR(
          format("env variable 'MULTIVIEW_STORE' set to an invalid value '{}'. "
                 "Note that if you want to set it to a path, it "
                 "must be an absolute path, i.e., beginning with, "
                 "'/'.",
                 value));
      has_error = true;
   }

   if(has_error) FATAL("aborting");
}

void init_perceive_files()
{
   std::call_once(init_perceive_files_flag, run_once_init_perceive_files);
}

static void ensure_init() { init_perceive_files(); }

// ----------------------------------------------------------------- Data Source

DataSource default_data_source() noexcept
{
   ensure_init();
   return source_;
}

void set_default_data_source(DataSource value) { ensure_init(); }

// ------------------------------------------------------------------ File Types

const string& file_type_relative_path(AssetType type) noexcept
{
   static string cad_model_relpath        = "calibration/cad-models"s;
   static string aruco_relpath            = "calibration/aruco-results"s;
   static string aruco_cubes_relpath      = "calibration/aruco-cubes"s;
   static string cache_file_relpath       = "cached-data"s;
   static string distortion_model_relpath = "calibration/sensors"s;
   static string bcam_info_relpath        = "calibration/binocular"s;
   static string scene_still_relpath      = "calibration/stills"s;
   static string phase_plane_data_relpath = "calibration/plane-data"s;
   static string classifier_relpath       = "calibration/classifiers"s;
   static string scene_bg_image_relpath   = "calibration/backgrounds"s;
   static string scene_disparity_relpath  = "calibration/disparities"s;
   static string scene_desc_relpath       = "calibration/scenes"s;

   switch(type) {
   case AssetType::CAD_MODEL: return cad_model_relpath;
   case AssetType::ARUCO_RESULT: return aruco_relpath;
   case AssetType::ARUCO_CUBE: return aruco_cubes_relpath;
   case AssetType::CACHE_FILE: return cache_file_relpath;
   case AssetType::DISTORTION_MODEL: return distortion_model_relpath;
   case AssetType::BCAM_INFO: return bcam_info_relpath;
   case AssetType::SCENE_STILL: return scene_still_relpath;
   case AssetType::PHASE_PLANE_DATA: return phase_plane_data_relpath;
   case AssetType::CLASSIFIER: return classifier_relpath;
   case AssetType::SCENE_BACKGROUND_IMAGE: return scene_bg_image_relpath;
   case AssetType::SCENE_DISPARITY: return scene_disparity_relpath;
   case AssetType::SCENE_DESCRIPTION: return scene_desc_relpath;
   }
}

const string& file_type_extension(AssetType type) noexcept
{
   static string cad_model_ext        = ""s;
   static string aruco_ext            = ".json"s;
   static string aruco_cubes_ext      = ".json"s;
   static string cache_file_ext       = ".mdata"s;
   static string distortion_model_ext = ".json"s;
   static string bcam_info_ext        = ".json"s;
   static string scene_still_ext      = ".ppm"s;
   static string phase_plane_data_ext = ".json"s;
   static string classifier_ext       = ".ml.gz"s;
   static string scene_bg_image_ext   = ".png"s;
   static string scene_disparity_ext  = ".float-image"s;
   static string scene_desc_ext       = ".json"s;

   switch(type) {
   case AssetType::CAD_MODEL: return cad_model_ext;
   case AssetType::ARUCO_RESULT: return aruco_ext;
   case AssetType::ARUCO_CUBE: return aruco_cubes_ext;
   case AssetType::CACHE_FILE: return cache_file_ext;
   case AssetType::DISTORTION_MODEL: return distortion_model_ext;
   case AssetType::BCAM_INFO: return bcam_info_ext;
   case AssetType::PHASE_PLANE_DATA: return phase_plane_data_ext;
   case AssetType::SCENE_STILL: return scene_still_ext;
   case AssetType::CLASSIFIER: return classifier_ext;
   case AssetType::SCENE_BACKGROUND_IMAGE: return scene_bg_image_ext;
   case AssetType::SCENE_DISPARITY: return scene_disparity_ext;
   case AssetType::SCENE_DESCRIPTION: return scene_desc_ext;
   }
}

// ----------------------------------------------------------------- Resolve Key

static string
resolve_key_(AssetType type, const string& key, DataSource source) noexcept
{
   if(source == DataSource::DEFAULT) source = source_;
   Expects(source != DataSource::DEFAULT);

   const auto& path = file_type_relative_path(type);
   auto ext         = file_type_extension(type);

   switch(source) {
   case DataSource::DEFAULT: FATAL("logic error");
   case DataSource::LAZY_S3:
   case DataSource::S3: return format("{}/{}/{}{}", s3_bucket_, path, key, ext);
   case DataSource::MULTIVIEW_ASSET_DIR:
      return format("{}/{}/{}{}", mv_data_dir_, path, key, ext);
   }

   return ""s;
}

string
resolve_key(AssetType type, const string& key, DataSource source) noexcept
{
   ensure_init();
   return resolve_key_(type, key, source);
}

// ----------------------------------------------------------- Data Source Fetch

template<typename T>
static string data_source_fetchT_(T& data,
                                  AssetType type,
                                  const string& key,
                                  DataSource source) noexcept(false)
{
   if(source == DataSource::DEFAULT) source = source_;
   Expects(source != DataSource::DEFAULT);
   string path = ""s;

   switch(source) {
   case DataSource::DEFAULT: FATAL("logic error"); break;
   case DataSource::S3:
      TRACE(format("s3 load {}", path));
      path = resolve_key_(type, key, source);
      s3_load(path, data);
      break;
   case DataSource::MULTIVIEW_ASSET_DIR:
      TRACE(format("load {}", path));
      path = resolve_key_(type, key, source);
      file_get_contents(path, data);
      break;
   case DataSource::LAZY_S3:
      path = resolve_key_(type, key, source);
      TRACE(format("lazy-load {}", path));
      lazy_s3_load(path, data);
      break;
   }

   {
      MD5 md5;
      md5.update(&data[0], unsigned(data.size()));
      const string ref_digest = md5.finalize().hexdigest();
      const string md5_digest = extract_md5_hexdigest(path);

      // if(!k_is_testcase_build) INFO(format("fetch {} {}", path,
      // ref_digest));
      if(!md5_digest.empty() and md5_digest != ref_digest) {
         data.clear();
         throw std::runtime_error(
             format("failed to load asset '{}', because md5 "
                    "check failed. Calculated digest was: {}",
                    path,
                    ref_digest));
      }
   }

   return path;
}

void data_source_fetch(string& data,
                       AssetType type,
                       const string& key,
                       DataSource source) noexcept(false)
{
   ensure_init();
   data_source_fetchT_(data, type, key, source);
}

void data_source_fetch(vector<char>& data,
                       AssetType type,
                       const string& key,
                       DataSource source) noexcept(false)
{
   ensure_init();
   data_source_fetchT_(data, type, key, source);
}

// ----------------------------------------------------------- Data Source
// Store

template<typename T>
static void data_source_storeT_(const T& data,
                                AssetType type,
                                const string& key,
                                DataSource source)
{
   if(source == DataSource::DEFAULT) source = source_;
   Expects(source != DataSource::DEFAULT);

   auto path = resolve_key_(type, key, source);

   {
      const string md5_digest = extract_md5_hexdigest(path);
      if(!md5_digest.empty()) {
         MD5 md5;
         md5.update(&data[0], unsigned(data.size()));
         const string ref_digest = md5.finalize().hexdigest();
         if(md5_digest != ref_digest) {
            throw std::runtime_error(
                format("failed to store asset '{}', because md5 "
                       "check failed. Calculated digest was: {}",
                       ref_digest));
         }
      }
   }

   if(source == DataSource::S3) {
      s3_store(path, data);
      return;
   } else if(source == DataSource::LAZY_S3) {
      lazy_s3_store(path, data);
      return;
   }

   file_put_contents(path, data);
}

void data_source_store(const string& data,
                       AssetType type,
                       const string& key,
                       DataSource source)
{
   ensure_init();
   data_source_storeT_(data, type, key, source);
}

void data_source_store(const vector<char>& data,
                       AssetType type,
                       const string& key,
                       DataSource source)
{
   ensure_init();
   data_source_storeT_(data, type, key, source);
}

// ---------------------------------------------------------------------- Exists

static bool
data_source_exists_(AssetType type, const string& key, DataSource source)
{
   if(source == DataSource::DEFAULT) source = source_;
   Expects(source != DataSource::DEFAULT);

   auto path = resolve_key(type, key, source);

   { // Handle S3
      if(source == DataSource::S3) {
         const auto ret = s3_exists(path);
         return ret;
      }
      if(source == DataSource::LAZY_S3) {
         const auto ret = lazy_s3_exists(path) != LazyExists::NOWHERE;
         return ret;
      }
   }

   return is_regular_file(path);
}

bool data_source_exists(AssetType type, const string& key, DataSource source)
{
   ensure_init();
   return data_source_exists_(type, key, source);
}

// ------------------------------------------------------------- DistortionModel

void fetch(DistortionModel& model,
           string key,
           DataSource source) noexcept(false)
{
   ensure_init();
   string data;
   data_source_fetch(data, AssetType::DISTORTION_MODEL, key, source);
   read(model, data);
}

void store(const DistortionModel& model,
           string key,
           DataSource source) noexcept(false)
{
   ensure_init();
   string data;
   write(model, data);
   data_source_store(data, AssetType::DISTORTION_MODEL, key, source);
}

// --------------------------------------------------- Caching Undistort Inverse

void fetch(CachingUndistortInverse& data,
           string key,
           DataSource source) noexcept(false)
{
   ensure_init();
   if(source == DataSource::DEFAULT) source = source_;

   if(source != DataSource::S3 and source != DataSource::LAZY_S3) {
      auto fname = resolve_key(AssetType::CACHE_FILE, key, source);

      // INFO(format("Loading cache-undistort file {} {}",
      //             fname,
      //             md5(file_get_contents(fname))));

      load(data, fname);
   } else {
      vector<char> buffer;
      data_source_fetch(buffer, AssetType::CACHE_FILE, key, source);
      FILE* fp = fmemopen(&buffer[0], buffer.size(), "rb");

      try {
         read(data, fp);
         fclose(fp);
      } catch(std::exception& e) {
         LOG_ERR(format("failed to read cache-undistort file: {}", e.what()));
         if(fp != nullptr) fclose(fp);
         throw e;
      }
   }
}

void store(const CachingUndistortInverse& data,
           string key,
           DataSource source) noexcept(false)
{
   ensure_init();
   if(source == DataSource::DEFAULT) source = source_;

   if(source != DataSource::S3 and source != DataSource::LAZY_S3) {
      auto fname = resolve_key(AssetType::CACHE_FILE, key, source);
      save(data, fname);

      INFO(format("Stored cache-undistort file {} {}",
                  fname,
                  md5(file_get_contents(fname))));
   } else {
      FILE* fp = nullptr;
      try {
         vector<char> buffer;
         fp = tmpfile();

         write(data, fp);

         buffer.resize(size_t(ftell(fp)));
         fseek(fp, 0, SEEK_SET);
         if(buffer.size() != fread(&buffer[0], 1, buffer.size(), fp))
            throw std::runtime_error("failed to write file");
         fclose(fp);

         data_source_store(buffer, AssetType::CACHE_FILE, key, source);
      } catch(std::exception& e) {
         if(fp != nullptr) fclose(fp);
         throw e;
      }
   }
}

// ------------------------------------------------------- Binocular Camera Info

void fetch(BinocularCameraInfo& out,
           string key,
           DataSource source) noexcept(false)
{
   const auto s0 = time_thunk([&]() { ensure_init(); });
   string data;
   const auto s1 = time_thunk(
       [&]() { data_source_fetch(data, AssetType::BCAM_INFO, key, source); });
   const auto s2 = time_thunk([&]() { read(out, data); });
   if(false) {
      TRACE(format(
          "fetch(\"{}\") timing: init={:6.3f}s, fetch={:6.3f}s, read={:6.3f}s",
          key,
          s0,
          s1,
          s2));
   }
}

void store(const BinocularCameraInfo& in,
           string key,
           DataSource source) noexcept(false)
{
   ensure_init();
   string data;
   write(in, data);
   data_source_store(data, AssetType::BCAM_INFO, key, source);
}

// ------------------------------------------------------- Scene Descrption Info

void fetch(SceneDescriptionInfo& out,
           string key,
           DataSource source) noexcept(false)
{
   ensure_init();
   string data;
   data_source_fetch(data, AssetType::SCENE_DESCRIPTION, key, source);
   read(out, data);
}

void store(const SceneDescriptionInfo& in,
           string key,
           DataSource source) noexcept(false)
{
   ensure_init();
   string data;
   write(in, data);
   data_source_store(data, AssetType::SCENE_DESCRIPTION, key, source);
}

// ----------------------------------------------------------- Aruco Result Info

void fetch(ArucoResultInfo& out, string key, DataSource source) noexcept(false)
{
   ensure_init();
   string data;
   data_source_fetch(data, AssetType::ARUCO_RESULT, key, source);
   read(out, data);
}

void store(const ArucoResultInfo& in,
           string key,
           DataSource source) noexcept(false)
{
   ensure_init();
   string data;
   write(in, data);
   data_source_store(data, AssetType::ARUCO_RESULT, key, source);
}

// ------------------------------------------------------------------ Aruco Cube

void fetch(ArucoCube& out, string key, DataSource source) noexcept(false)
{
   ensure_init();
   string data;
   data_source_fetch(data, AssetType::ARUCO_CUBE, key, source);
   read(out, data);
}

void store(const ArucoCube& in, string key, DataSource source) noexcept(false)
{
   ensure_init();
   string data;
   write(in, data);
   data_source_store(data, AssetType::ARUCO_CUBE, key, source);
}

// ---------------------------------------------------------------------- Sprite

void fetch(Sprite& out, string key, DataSource source) noexcept(false)
{
   ensure_init();
   string data;
   data_source_fetch(data, AssetType::CAD_MODEL, key, source);
   read(out, data);
}

// ----------------------------------------------------------------- Scene Still

void fetch(ARGBImage& out, string key, DataSource source) noexcept(false)
{
   ensure_init();
   string data;
   data_source_fetch(data, AssetType::SCENE_STILL, key, source);

   string magic;
   int width = 0, height = 0, max_val = 0;

   std::stringstream ss(data);
   ss >> magic >> width >> height >> max_val;

   // INFO(
   //      format("magin = {}, wh = {}x{}, val={}", magic, width,
   //      height, max_val));

   if(ss.peek() != '\n')
      throw std::runtime_error(
          "expected newline character at end of PPM header");
   ss.get();

   out.resize(width, height);

   int r = 0, g = 0, b = 0;
   for(auto y = 0u; y < out.height; ++y) {
      for(auto x = 0u; x < out.width; ++x) {
         r         = ss.get();
         g         = ss.get();
         b         = ss.get();
         out(x, y) = make_colour(r, g, b);
      }
   }
}

void store(const ARGBImage& in, string key, DataSource source) noexcept(false)
{
   ensure_init();

   std::stringstream ss("");
   ss << format("P6\n{} {}\n255\n", in.width, in.height);
   char r = 0, g = 0, b = 0;
   for(auto y = 0u; y < in.height; ++y) {
      for(auto x = 0u; x < in.width; ++x) {
         auto k = in(x, y);
         r      = char(red(k));
         g      = char(green(k));
         b      = char(blue(k));

         ss.write(&r, 1);
         ss.write(&g, 1);
         ss.write(&b, 1);
      }
   }

   data_source_store(ss.str(), AssetType::SCENE_STILL, key, source);
}

// ------------------------------------------------------------- Scene Disparity

void fetch(FloatImage& out, string key, DataSource source) noexcept(false)
{
   ensure_init();
   string data;
   data_source_fetch(data, AssetType::SCENE_DISPARITY, key, source);

   string magic;
   int width = 0, height = 0;

   std::stringstream ss(data);
   ss >> magic >> width >> height;

   // INFO(
   //      format("magin = {}, wh = {}x{}, val={}", magic, width,
   //      height, max_val));

   if(ss.peek() != '\n')
      throw std::runtime_error(
          "expected newline character at end of FloatImage header");
   ss.get();

   out.resize(width, height);

   for(auto y = 0u; y < out.height; ++y) {
      for(auto x = 0u; x < out.width; ++x) {
         uint32_t v = 0;
         ss.read(reinterpret_cast<char*>(&v), sizeof(v));
         out(x, y) = unpack_f32(boost::endian::little_to_native(v));
      }
   }
}

void store(const FloatImage& in, string key, DataSource source) noexcept(false)
{
   ensure_init();

   std::stringstream ss("");
   ss << format("FI\n{} {}\n", in.width, in.height);

   for(auto y = 0u; y < in.height; ++y) {
      for(auto x = 0u; x < in.width; ++x) {
         uint32_t v = boost::endian::native_to_little(pack_f32(in(x, y)));
         ss.write(reinterpret_cast<char*>(&v), sizeof(v));
      }
   }

   data_source_store(ss.str(), AssetType::SCENE_DISPARITY, key, source);
}

// ------------------------------------------------------------ Phase Plane Data

void fetch(calibration::PhasePlaneData& out,
           string key,
           DataSource source) noexcept(false)
{
   ensure_init();
   string data;
   data_source_fetch(data, AssetType::PHASE_PLANE_DATA, key, source);
   read(out, data);
}

void store(const calibration::PhasePlaneData& in,
           string key,
           DataSource source) noexcept(false)
{
   ensure_init();
   string data;
   write(in, data);
   data_source_store(data, AssetType::PHASE_PLANE_DATA, key, source);
}

// ------------------------------------------------------------------ Classifier

void fetch(Classifier& out, string key, DataSource source) noexcept(false)
{
   ensure_init();
   string data;
   data_source_fetch(data, AssetType::CLASSIFIER, key, source);

   out.read(data);
}

void store(const Classifier& in, string key, DataSource source) noexcept(false)
{
   ensure_init();
   string data = in.write();
   data_source_store(data, AssetType::CLASSIFIER, key, source);
}

// ------------------------------------------------------ Scene Background Image

void fetch_bg_image(ARGBImage& im,
                    string key,
                    DataSource source) noexcept(false)
{
   ensure_init();
   vector<char> data;
   data_source_fetch(data, AssetType::SCENE_BACKGROUND_IMAGE, key, source);
   im = decode_image(data);
}

} // namespace perceive
