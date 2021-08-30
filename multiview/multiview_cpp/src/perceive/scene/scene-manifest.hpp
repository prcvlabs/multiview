
#pragma once

#include "perceive/geometry/vector.hpp"
#include "perceive/utils/timestamp.hpp"
#include "json/json.h"

namespace perceive
{
constexpr real k_perceive_default_frame_rate = 15.0; // 15 frames per second

struct SceneManifest
{
   int store_id{0};
   string scene_key{""};

   Timestamp epoch;

   vector<string> camera_ids;
   vector<string> videos;
   vector<Timestamp> timestamps;
   vector<real> fps; // defaults to 15.0fps

   CUSTOM_NEW_DELETE(SceneManifest)

   bool operator==(const SceneManifest& o) const noexcept
   {
      return scene_key == o.scene_key and store_id == o.store_id
             and camera_ids == o.camera_ids and videos == o.videos
             and epoch == o.epoch and timestamps == o.timestamps;
   }

   bool operator!=(const SceneManifest& o) const noexcept
   {
      return !(*this == o);
   }

   string to_string() const noexcept;
   string to_json_string() const noexcept;

   size_t size() const noexcept { return videos.size(); }

   vector<string> resolve_video_filenames(const string& dirname) const;

   friend string str(const SceneManifest& sd) { return sd.to_string(); }

   // void load(SceneManifest& data, const string& fname) noexcept(false);
   friend void read(SceneManifest& data, const std::string& in) noexcept(false);
   friend void read(SceneManifest& data,
                    const Json::Value& node) noexcept(false);

   friend void save(const SceneManifest& data,
                    const std::string& fname) noexcept(false);
   friend void write(const SceneManifest& data,
                     std::string& out) noexcept(false);
};

} // namespace perceive
