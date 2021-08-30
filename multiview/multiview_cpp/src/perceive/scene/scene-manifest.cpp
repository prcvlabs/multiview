
#include "perceive/io/json-io.hpp"
#include "perceive/io/perceive-assets.hpp"
#include "perceive/io/s3.hpp"
#include "perceive/utils/file-system.hpp"
#include "scene-manifest.hpp"

#define This SceneManifest

namespace perceive
{
// ---------------------------------------------- SceneDecriptionInfo::to_string

string This::to_string() const noexcept
{
   auto json_encode_str
       = [&](const string& s) -> string { return json_encode(s); };

   auto ss = std::stringstream("");
   for(auto i = 0u; i < size(); ++i) {
      if(i > 0) ss << ",";
      ss << format(R"V0G0N(
{{
    "camera":    {},
    "file":      {},
    "timestamp": {},
    "fps":       {}
}}{})V0G0N",
                   json_encode(camera_ids[i]),
                   json_encode(videos[i]),
                   json_encode(timestamps[i]),
                   fps[i],
                   "");
   }

   auto videos_s = indent(ss.str(), 8);

   return format(R"V0G0N(
{{
   "type":                  "SceneManifest",
   "store":                 {},
   "scene-key":             {},
   "epoch":                 {},
   "videos":               [{}]
}}
{})V0G0N",
                 store_id,
                 json_encode_str(scene_key),
                 json_encode(epoch),
                 videos_s,
                 "");
}

string This::to_json_string() const noexcept { return to_string(); }

// ----------------------------------------------------- Resolve Video Filenames

vector<string> This::resolve_video_filenames(const string& directory) const
{
   auto resolve_filename = [&](const string& fname) -> string {
      if(fname.empty())
         throw std::runtime_error(
             format("empty filename found in scene manifest!"));

      if(begins_with(fname, string("s3://"))) {
         return fname;
      } else if(directory == "") {
         if(is_regular_file(fname)) return fname;
      } else {
         auto fn = format("{}/{}", directory, fname);
         if(is_regular_file(fn)) return fn;
      }

      auto msg = format("failed to resolve filename '{}'", fname);
      LOG_ERR(msg);
      throw std::runtime_error(msg);
      return "";
   };

   vector<string> out(videos.size());
   std::transform(cbegin(videos), cend(videos), begin(out), resolve_filename);

   if(false) {
      for(auto i = 0u; i < videos.size(); ++i)
         TRACE(format("resolved '{}' ==> '{}'", videos[i], out[i]));
   }

   return out;
}

// -------------------------------------------------------- Load/Save Read/Write

void load(SceneManifest& data, const string& fname) noexcept(false)
{
   read(data, file_get_contents(fname));
}

void read(SceneManifest& data, const std::string& in) noexcept(false)
{
   read(data, parse_json(in));
}

void read(SceneManifest& data, const Json::Value& node) noexcept(false)
{
   std::string err = "";

   auto from_json = [&](const Json::Value& root) {
      SceneManifest tmp;

      json_load(get_key(root, "store"), tmp.store_id);
      json_load(get_key(root, "scene-key"), tmp.scene_key);
      json_load(get_key(root, "epoch"), tmp.epoch);

      // tmp.aruco_result_key = "";
      // if(has_key(root, "aruco-result-key"))
      //     json_load(get_key(root, "aruco-result-key"),
      //                       tmp.aruco_result_key);

      auto video_keys = get_key(root, "videos");

      if(!video_keys.isArray())
         throw std::runtime_error("expecting JSON array value for "
                                  "'videos'");

      const auto N = video_keys.size();
      tmp.camera_ids.resize(N);
      tmp.videos.resize(N);
      tmp.timestamps.resize(N);
      tmp.fps.resize(N);
      std::fill(begin(tmp.fps), end(tmp.fps), 15.0);
      for(auto i = 0u; i < N; ++i) {
         const auto& node = video_keys[i];
         json_load(get_key(node, "camera"), tmp.camera_ids[i]);
         json_load(get_key(node, "file"), tmp.videos[i]);
         json_load(get_key(node, "timestamp"), tmp.timestamps[i]);
         if(has_key(node, "fps")) json_load(get_key(node, "fps"), tmp.fps[i]);
      }

      { // Check that all the 'fps' are almost the same
         const auto ii
             = std::adjacent_find(cbegin(tmp.fps),
                                  cend(tmp.fps),
                                  [&](const real A, const real B) -> bool {
                                     return !is_close(A, B);
                                  });
         if(ii != cend(tmp.fps))
            throw std::runtime_error(
                format("all the fps must be the same! fps array was [{}]",
                       implode(cbegin(tmp.fps), cend(tmp.fps), ", ")));
      }

      data = tmp;
   };

   try {
      from_json(node);
   } catch(std::logic_error& e) {
      err = strlen(e.what()) == 0 ? "logic error" : e.what();
   } catch(std::runtime_error& e) {
      err = strlen(e.what()) == 0 ? "runtime error" : e.what();
   } catch(std::exception& e) {
      // JSON parse error
      err = strlen(e.what()) == 0 ? "exception" : e.what();
   } catch(...) {
      err = format("unknown error");
   }

   if(err != "") throw std::runtime_error(err);
}

void save(const SceneManifest& data, const std::string& fname) noexcept(false)
{
   std::string data_s;
   write(data, data_s);
   file_put_contents(fname, data_s);
}

void write(const SceneManifest& data, std::string& out) noexcept(false)
{
   out = data.to_json_string();
}

} // namespace perceive
