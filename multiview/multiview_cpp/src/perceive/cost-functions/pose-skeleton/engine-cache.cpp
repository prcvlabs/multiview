
#include "engine-cache.hpp"

#include "perceive/utils/md5.hpp"

#include <filesystem>

#include <unistd.h>

namespace perceive
{
// -------------------------------------------------------- EngineKey::meta-data
//
const vector<MemberMetaData>& EngineKey::meta_data() const noexcept
{
#define ThisParams EngineKey
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, STRING, model_md5, true));
      m.push_back(MAKE_META(ThisParams, INT, max_batch_size, true));
      m.push_back(MAKE_META(ThisParams, INT, input_h, true));
      m.push_back(MAKE_META(ThisParams, INT, input_w, true));
      m.push_back(MAKE_META(ThisParams, STRING, precision, true));
      m.push_back(MAKE_META(ThisParams, INT, trt_version, true));
      m.push_back(MAKE_META(ThisParams, STRING, gpu_name, true));
      m.push_back(MAKE_META(ThisParams, INT, compute_major, true));
      m.push_back(MAKE_META(ThisParams, INT, compute_minor, true));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

// ----------------------------------------------------------- EngineKey::digest
//
string EngineKey::digest() const noexcept { return md5(to_json_string()); }

// --------------------------------------------------------- make-engine-sidecar
//
string make_engine_sidecar(const EngineKey& key,
                           const string_view plan_md5) noexcept
{
   Json::Value o = key.to_json();
   o["plan_md5"] = string(plan_md5);
   std::stringstream ss{""};
   ss << o;
   return ss.str();
}

// ------------------------------------------------------ engine-sidecar-matches
//
bool engine_sidecar_matches(const string& sidecar,
                            const EngineKey& key,
                            const string_view plan_md5) noexcept
{
   try {
      const auto o = parse_json(sidecar);
      EngineKey saved;
      saved.read(o); // throws if a field is missing
      const auto saved_md5
          = json_load_key<string>(o, "plan_md5", "reading an engine sidecar");
      return (saved == key) and (saved_md5 == plan_md5);
   } catch(std::exception& e) {
      // malformed sidecar
   }
   return false;
}

// ---------------------------------------------------------- find-cached-engine
//
string find_cached_engine(const string_view dir, const EngineKey& key) noexcept
{
   const auto digest        = key.digest();
   const auto plan_fname    = format("{}/{}.plan", dir, digest);
   const auto sidecar_fname = format("{}/{}.json", dir, digest);
   if(!is_regular_file(plan_fname) or !is_regular_file(sidecar_fname))
      return ""s;

   string plan    = ""s;
   string sidecar = ""s;
   if(file_get_contents(plan_fname, plan)
      or file_get_contents(sidecar_fname, sidecar)
      or !engine_sidecar_matches(sidecar, key, md5(plan))) {
      WARN(format("ignoring cached TensorRT engine '{}', because it does not "
                  "match its sidecar",
                  plan_fname));
      return ""s;
   }

   return plan_fname;
}

// ---------------------------------------------------------------- cache-engine
//
string cache_engine(const string_view dir,
                    const EngineKey& key,
                    std::function<void(const string& filename)> save) noexcept
{
   namespace fs = std::filesystem;

   const auto digest        = key.digest();
   const auto plan_fname    = format("{}/{}.plan", dir, digest);
   const auto sidecar_fname = format("{}/{}.json", dir, digest);
   const auto tmp_plan      = format("{}.{}.tmp", plan_fname, getpid());
   const auto tmp_sidecar   = format("{}.{}.tmp", sidecar_fname, getpid());

   try {
      if(!is_directory(dir) and !mkdir_p(dir))
         throw std::runtime_error(format("could not make directory '{}'", dir));

      save(tmp_plan);
      string plan = ""s;
      if(file_get_contents(tmp_plan, plan) or plan.empty())
         throw std::runtime_error(format("failed to save '{}'", tmp_plan));
      if(file_put_contents(tmp_sidecar, make_engine_sidecar(key, md5(plan))))
         throw std::runtime_error(format("failed to write '{}'", tmp_sidecar));

      fs::rename(tmp_plan, plan_fname);
      fs::rename(tmp_sidecar, sidecar_fname);
      return plan_fname;
   } catch(std::exception& e) {
      WARN(format("failed to save TensorRT engine to '{}': {}", dir, e.what()));
   }

   delete_file(tmp_plan);
   delete_file(tmp_sidecar);
   return ""s;
}

} // namespace perceive
