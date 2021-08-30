
#pragma once

#include "perceive/foundation.hpp"
#include "perceive/graphics/image-container.hpp"

namespace perceive
{
// -------------------------------------------------------------- Predefinitions

template<unsigned K> class PolynomialModel;
using DistortionModel = PolynomialModel<8>;
struct CachingUndistortInverse;
struct BinocularCameraInfo;
struct SceneDescriptionInfo;
class Sprite;
struct ArucoResultInfo;
struct ArucoCube;
class Classifier;

namespace calibration
{
   struct PhasePlaneData;
} // namespace calibration

// ----------------------------------------------------------------- Data Source

enum class DataSource : int {
   DEFAULT             = 0, // Use system default (LAZY_S3)
   S3                  = 1, // load/save to correct location in S3
   LAZY_S3             = 2, // Looks in a local cache, and then in S3
   MULTIVIEW_ASSET_DIR = 3  // load/save to correct location in $PERCEIVE_DATA
};

void init_perceive_files(); // idempotent
DataSource default_data_source() noexcept;
void set_default_data_source(DataSource value);

// ------------------------------------------------------------------ File Types
// These map to specific paths
enum class AssetType : int {
   CAD_MODEL = 0,
   ARUCO_RESULT,
   ARUCO_CUBE,
   CACHE_FILE,
   DISTORTION_MODEL,
   BCAM_INFO,
   SCENE_STILL,
   PHASE_PLANE_DATA,
   CLASSIFIER,
   SCENE_BACKGROUND_IMAGE,
   SCENE_DISPARITY,
   SCENE_DESCRIPTION
};

constexpr array<AssetType, int(AssetType::SCENE_DESCRIPTION) + 1> k_filetypes{
    {AssetType::CAD_MODEL,
     AssetType::ARUCO_RESULT,
     AssetType::ARUCO_CUBE,
     AssetType::CACHE_FILE,
     AssetType::DISTORTION_MODEL,
     AssetType::BCAM_INFO,
     AssetType::SCENE_STILL,
     AssetType::PHASE_PLANE_DATA,
     AssetType::CLASSIFIER,
     AssetType::SCENE_BACKGROUND_IMAGE,
     AssetType::SCENE_DISPARITY,
     AssetType::SCENE_DESCRIPTION}};

// Get the specific directory path for a file-type
const string& file_type_relative_path(AssetType type) noexcept;

const string& file_type_extension(AssetType type) noexcept;

// ------------------------------------------------------------------ Raw Access

string resolve_key(AssetType type,
                   const string& key,
                   DataSource source = DataSource::DEFAULT) noexcept;

// ---- Fetch
void data_source_fetch(string& data,
                       AssetType type,
                       const string& key,
                       DataSource source = DataSource::DEFAULT) noexcept(false);
void data_source_fetch(vector<char>& data,
                       AssetType type,
                       const string& key,
                       DataSource source = DataSource::DEFAULT) noexcept(false);

// ---- Store
void data_source_store(const string& data,
                       AssetType type,
                       const string& key,
                       DataSource source = DataSource::DEFAULT);

void data_source_store(const vector<char>& data,
                       AssetType type,
                       const string& key,
                       DataSource source = DataSource::DEFAULT);

// ---- Exists
bool data_source_exists(AssetType type,
                        const string& key,
                        DataSource source = DataSource::DEFAULT);

// ---- Remove
// void data_source_remove(AssetType type,
//                         const string& key,
//                         DataSource source = DataSource::DEFAULT);

// ----------------------------------------------------------- Distortion Models

void fetch(DistortionModel& data,
           string key,
           DataSource source = DataSource::DEFAULT) noexcept(false);
void store(const DistortionModel& data,
           string key,
           DataSource source = DataSource::DEFAULT) noexcept(false);

// --------------------------------------------------- Caching Undistort Inverse

void fetch(CachingUndistortInverse& data,
           string key,
           DataSource source = DataSource::DEFAULT) noexcept(false);
void store(const CachingUndistortInverse& data,
           string key,
           DataSource source = DataSource::DEFAULT) noexcept(false);

// ------------------------------------------------------- Binocular Camera Info

void fetch(BinocularCameraInfo& data,
           string key,
           DataSource source = DataSource::DEFAULT) noexcept(false);
void store(const BinocularCameraInfo& data,
           string key,
           DataSource source = DataSource::DEFAULT) noexcept(false);

// ------------------------------------------------------- Scene Descrption Info

void fetch(SceneDescriptionInfo& data,
           string key,
           DataSource source = DataSource::DEFAULT) noexcept(false);
void store(const SceneDescriptionInfo& data,
           string key,
           DataSource source = DataSource::DEFAULT) noexcept(false);

// ----------------------------------------------------------- Aruco Result Info

void fetch(ArucoResultInfo& data,
           string key,
           DataSource source = DataSource::DEFAULT) noexcept(false);
void store(const ArucoResultInfo& data,
           string key,
           DataSource source = DataSource::DEFAULT) noexcept(false);

// ------------------------------------------------------------------ Aruco Cube

void fetch(ArucoCube& data,
           string key,
           DataSource source = DataSource::DEFAULT) noexcept(false);
void store(const ArucoCube& data,
           string key,
           DataSource source = DataSource::DEFAULT) noexcept(false);

// ---------------------------------------------------------------------- Sprite

void fetch(Sprite& data,
           string key,
           DataSource source = DataSource::DEFAULT) noexcept(false);

// ----------------------------------------------------------------- Scene Still

void fetch(ARGBImage& data,
           string key,
           DataSource source = DataSource::DEFAULT) noexcept(false);
void store(const ARGBImage& data,
           string key,
           DataSource source = DataSource::DEFAULT) noexcept(false);

// ------------------------------------------------------------- Scene Disparity

void fetch(FloatImage& data,
           string key,
           DataSource source = DataSource::DEFAULT) noexcept(false);
void store(const FloatImage& data,
           string key,
           DataSource source = DataSource::DEFAULT) noexcept(false);

// ------------------------------------------------------------ Phase Plane Data

void fetch(calibration::PhasePlaneData& data,
           string key,
           DataSource source = DataSource::DEFAULT) noexcept(false);
void store(const calibration::PhasePlaneData& data,
           string key,
           DataSource source = DataSource::DEFAULT) noexcept(false);

// ------------------------------------------------------------------ Classifier

void fetch(Classifier& data,
           string key,
           DataSource source = DataSource::DEFAULT) noexcept(false);
void store(const Classifier& data,
           string key,
           DataSource source = DataSource::DEFAULT) noexcept(false);

// ------------------------------------------------------ Scene Background Image

void fetch_bg_image(ARGBImage& data,
                    string key,
                    DataSource source = DataSource::DEFAULT) noexcept(false);

} // namespace perceive
