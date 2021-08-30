
#include "stdinc.hpp"

#include "movie-stats-task.hpp"
#include "movie-task.hpp"

#include "perceive/utils/file-system.hpp"

namespace perceive::pipeline::movie_stats
{
const vector<MemberMetaData>& Params::meta_data() const noexcept
{
#define ThisParams Params
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, BOOL, feedback, false));
      m.push_back(MAKE_META(ThisParams, STRING, out_dir, false));

      m.push_back(MAKE_META(ThisParams, BOOL, no_stats, true));
      m.push_back(MAKE_META(ThisParams, STRING, save_filename, true));

      m.push_back({meta_type::JSON_VALUE,
                   "in_stats_fnames"s,
                   true,
                   [](const void* ptr) -> std::any {
                      const auto& o = *reinterpret_cast<const ThisParams*>(ptr);
                      Json::Value z{Json::arrayValue};
                      z.resize(unsigned(o.in_stats_fnames.size()));
                      for(auto i = 0u; i < z.size(); ++i)
                         z[i] = Json::Value(o.in_stats_fnames[i]);
                      return std::any(z);
                   },
                   [](void* ptr, const std::any& x) -> void {
                      auto& o = *reinterpret_cast<ThisParams*>(ptr);
                      const Json::Value& z
                          = std::any_cast<const Json::Value>(x);
                      if(z.type() != Json::arrayValue)
                         throw std::runtime_error("expected Json::array");
                      o.in_stats_fnames.resize(z.size());
                      for(auto i = 0u; i < z.size(); ++i)
                         o.in_stats_fnames[i] = z[i].asString();
                   }});

      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

// --------------------------------------------------------------- Task::execute

shared_ptr<const Result>
Task::execute(const RunData& data,
              const Params& p,
              std::function<bool()> is_cancelled) const noexcept
{
   if(is_cancelled()) return nullptr;

   auto ret       = make_shared<Result>();
   bool has_stats = false;
   bool has_error = false;

   auto load_scene_task = data.match_result<load_scene_description::Result>(
       "load_scene_description");
   if(!load_scene_task) {
      LOG_ERR("Can't find dependency: 'load_scene_description'");
      return nullptr;
   }
   ret->scene_desc = load_scene_task->scene_desc;

   if(p.in_stats_fnames.size() > 0) {
      vector<MovieStatsFile> in_stats(p.in_stats_fnames.size());
      for(auto i = 0u; i < p.in_stats_fnames.size(); ++i) {
         const string& fname = p.in_stats_fnames[i];
         try {
            load(in_stats[i], fname);
            if(p.feedback) INFO(format("loaded movie-stats: '{}'", fname));
         } catch(std::exception& e) {
            LOG_ERR(format("could not load movie-stats: {}", e.what()));
            has_error = true;
            return nullptr;
         }
      }

      if(in_stats.size() > 1 and p.feedback)
         INFO(format("combining {} movie stats into one.", in_stats.size()));

      while(in_stats.size() > 1) {
         auto pos = 0u;
         for(auto i = 0u; i < in_stats.size(); i += 2) {
            if((i + 1) >= in_stats.size()) {
               MovieStatsFile a = std::move(in_stats[i + 0]);
               in_stats[pos++]  = std::move(a);
            } else {
               MovieStatsFile a = std::move(in_stats[i + 0]);
               MovieStatsFile b = std::move(in_stats[i + 1]);
               in_stats[pos++]  = combine(a, b);
            }
         }
         Expects(pos < in_stats.size());
         in_stats.resize(pos);
      }

      Expects(in_stats.size() == 1);
      ret->stats = std::move(in_stats[0]);
      if(!has_error) has_stats = true;
   }

   if(has_stats and !p.out_dir.empty()) {
      try {
         ret->stats.export_image(p.out_dir);
      } catch(std::exception& e) {
         LOG_ERR(format("could not export stats data: {}", e.what()));
      }
   }

   if(!has_stats and !p.no_stats) {
      LOG_ERR(format("failed to load movie stats"));
      return nullptr;
   }

   ret->has_stats = has_stats;

   return ret;
}

} // namespace perceive::pipeline::movie_stats
