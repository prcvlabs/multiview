
#include "envelope-ops.hpp"

#include "perceive/utils/file-system.hpp"
#include "perceive/utils/md5.hpp"

namespace perceive::ldata
{
// ------------------------------------------------------------ envelope helpers
//
static int env_start(const LocalizationDataEnvelope& env) noexcept
{
   return int(env.start_frame);
}

static int env_end(const LocalizationDataEnvelope& env) noexcept
{
   return int(env.start_frame) + int(env.loc_data.size());
}

static const LocalizationData& env_frame(const LocalizationDataEnvelope& env,
                                         const int frame_no) noexcept
{
   Expects(frame_no >= env_start(env) and frame_no < env_end(env));
   return env.loc_data[size_t(frame_no - env_start(env))];
}

static bool same_geometry(const LocalizationData& a,
                          const LocalizationData& b) noexcept
{
   return a.loc_hist.width == b.loc_hist.width
          and a.loc_hist.height == b.loc_hist.height and a.hist_sz == b.hist_sz
          and a.bounds == b.bounds;
}

// ----------------------------------------------------------------- frame-bytes
//
// The bytes `LocalizationData::write` produces for `ldat`, or nothing if the
// frame cannot be written (a 2D pose without its skeleton).
static std::optional<string> frame_bytes(const LocalizationData& ldat) noexcept
{
   for(const auto& p2d_vec : ldat.p2ds)
      for(const auto& info : p2d_vec)
         if(info.p2d_ptr == nullptr) return std::nullopt;

   char* buffer = nullptr;
   size_t size  = 0;
   FILE* fp     = open_memstream(&buffer, &size);
   if(fp == nullptr) return std::nullopt;
   ldat.write(fp);
   fclose(fp);
   string ret(buffer, size);
   free(buffer);
   return ret;
}

static string frame_md5(const LocalizationData& ldat) noexcept
{
   const auto bytes = frame_bytes(ldat);
   return bytes.has_value() ? md5(*bytes) : ""s;
}

static int count_p2ds(const LocalizationData& ldat) noexcept
{
   int ret = 0;
   for(const auto& p2d_vec : ldat.p2ds) ret += int(p2d_vec.size());
   return ret;
}

// ------------------------------------------------------------------ diff-frame
//
static FrameDiff diff_frame(const LocalizationData& a,
                            const LocalizationData& b) noexcept
{
   FrameDiff ret;
   ret.frame_no     = a.frame_no;
   ret.header_equal = a.frame_no == b.frame_no and a.loc_min == b.loc_min
                      and a.loc_max == b.loc_max and a.hist_sz == b.hist_sz
                      and a.bounds == b.bounds;

   const auto& ha      = a.loc_hist;
   const auto& hb      = b.loc_hist;
   ret.hist_dims_equal = ha.width == hb.width and ha.height == hb.height;
   if(ret.hist_dims_equal) {
      uint64_t l1 = 0;
      for(auto y = 0u; y < ha.height; ++y) {
         for(auto x = 0u; x < ha.width; ++x) {
            const int delta = std::abs(int(ha(x, y)) - int(hb(x, y)));
            if(delta == 0) continue;
            ret.hist_cells_differ += 1;
            ret.hist_max_abs = std::max(ret.hist_max_abs, delta);
            l1 += uint64_t(delta);
         }
      }
      ret.hist_l1_mean = (ha.size() == 0) ? 0.0 : real(l1) / real(ha.size());
   } else {
      ret.hist_cells_differ = int(std::max(ha.size(), hb.size()));
      ret.hist_max_abs
          = int(std::numeric_limits<Grey16Image::value_type>::max());
   }

   ret.p2d_count_a      = count_p2ds(a);
   ret.p2d_count_b      = count_p2ds(b);
   const auto n_sensors = std::max(a.p2ds.size(), b.p2ds.size());
   for(size_t i = 0; i < n_sensors; ++i)
      if(i >= a.p2ds.size() or i >= b.p2ds.size() or a.p2ds[i] != b.p2ds[i])
         ret.p2d_sensors_differ += 1;
   ret.p2ds_equal = (ret.p2d_sensors_differ == 0);

   ret.equal = ret.header_equal and ret.hist_cells_differ == 0
               and ret.p2ds_equal and a == b;

   const auto bytes_a = frame_bytes(a);
   const auto bytes_b = frame_bytes(b);
   ret.bytes_equal
       = bytes_a.has_value() and bytes_b.has_value() and *bytes_a == *bytes_b;
   ret.md5_a = bytes_a.has_value() ? md5(*bytes_a) : ""s;
   ret.md5_b = bytes_b.has_value() ? md5(*bytes_b) : ""s;
   return ret;
}

// Is `a` a worse difference than `b`?
static bool is_worse(const FrameDiff& a, const FrameDiff& b) noexcept
{
   auto key = [](const FrameDiff& o) {
      return std::make_tuple(o.hist_max_abs,
                             o.hist_cells_differ,
                             !o.p2ds_equal,
                             !o.header_equal,
                             !o.equal);
   };
   return key(a) > key(b);
}

// ------------------------------------------------------------------ diff-range
//
static void diff_range(DiffResult& ret,
                       const LocalizationDataEnvelope& a,
                       const LocalizationDataEnvelope& b,
                       const int first_frame,
                       const int n_frames) noexcept
{
   ret.first_frame   = first_frame;
   ret.n_frames      = n_frames;
   ret.n_bytes_equal = 0;
   ret.n_equal       = 0;
   ret.frames.clear();
   for(auto t = first_frame; t < first_frame + n_frames; ++t) {
      const auto diff = diff_frame(env_frame(a, t), env_frame(b, t));
      if(diff.equal) ret.n_equal += 1;
      if(diff.bytes_equal) {
         ret.n_bytes_equal += 1;
         continue;
      }
      if(ret.frames.empty() or is_worse(diff, ret.worst)) ret.worst = diff;
      ret.frames.push_back(diff);
   }
}

// ------------------------------------------------------------------ frame-diff
//
Json::Value FrameDiff::to_json() const noexcept
{
   Json::Value o{Json::objectValue};
   o["frame_no"]           = frame_no;
   o["bytes_equal"]        = bytes_equal;
   o["equal"]              = equal;
   o["md5_a"]              = md5_a;
   o["md5_b"]              = md5_b;
   o["header_equal"]       = header_equal;
   o["hist_dims_equal"]    = hist_dims_equal;
   o["hist_cells_differ"]  = hist_cells_differ;
   o["hist_max_abs"]       = hist_max_abs;
   o["hist_l1_mean"]       = hist_l1_mean;
   o["p2ds_equal"]         = p2ds_equal;
   o["p2d_count_a"]        = p2d_count_a;
   o["p2d_count_b"]        = p2d_count_b;
   o["p2d_sensors_differ"] = p2d_sensors_differ;
   return o;
}

// ----------------------------------------------------------------- diff-result
//
bool DiffResult::identical() const noexcept
{
   return error.empty() and headers_equal and n_frames > 0
          and n_bytes_equal == n_frames;
}

Json::Value DiffResult::to_json(const bool verbose) const noexcept
{
   Json::Value o{Json::objectValue};
   o["identical"] = identical();
   if(!error.empty()) o["error"] = error;
   o["headers_equal"] = headers_equal;
   o["first_frame"]   = first_frame;
   o["n_frames"]      = n_frames;
   o["n_bytes_equal"] = n_bytes_equal;
   o["n_equal"]       = n_equal;
   o["n_differ"]      = int(frames.size());
   if(!frames.empty()) o["worst"] = worst.to_json();
   if(verbose) {
      Json::Value arr{Json::arrayValue};
      for(const auto& diff : frames) arr.append(diff.to_json());
      o["frames"] = arr;
   }
   return o;
}

// -------------------------------------------------------------- diff-envelopes
//
DiffResult diff_envelopes(const LocalizationDataEnvelope& a,
                          const LocalizationDataEnvelope& b) noexcept
{
   DiffResult ret;
   ret.headers_equal = a.start_frame == b.start_frame
                       and a.loc_data.size() == b.loc_data.size()
                       and a.frame_duration == b.frame_duration;

   const int first_frame = std::max(env_start(a), env_start(b));
   const int end_frame   = std::min(env_end(a), env_end(b));
   if(end_frame <= first_frame) {
      ret.error = format("no frames in common: [{}, {}) and [{}, {})",
                         env_start(a),
                         env_end(a),
                         env_start(b),
                         env_end(b));
      return ret;
   }

   diff_range(ret, a, b, first_frame, end_frame - first_frame);
   return ret;
}

// --------------------------------------------------------------- cat-envelopes
//
CatResult cat_envelopes(const vector<LocalizationDataEnvelope>& envelopes,
                        const vector<KeepRange>& keep) noexcept
{
   CatResult ret;

   auto fail = [&](string message) {
      ret.envelope = {};
      ret.overlap_reports.clear();
      ret.error = std::move(message);
      return ret;
   };

   if(envelopes.empty()) return fail("no localization data to concatenate");
   if(!keep.empty() and keep.size() != envelopes.size())
      return fail(format("{} keep ranges given for {} envelopes",
                         keep.size(),
                         envelopes.size()));

   const size_t n_envs    = envelopes.size();
   vector<KeepRange> kept = keep;
   if(kept.empty()) {
      kept.resize(n_envs);
      for(size_t i = 0; i < n_envs; ++i) {
         const int end = (i + 1 < n_envs) ? env_start(envelopes[i + 1])
                                          : env_end(envelopes[i]);
         kept[i].start = env_start(envelopes[i]);
         kept[i].n     = end - kept[i].start;
      }
   }

   try {
      const auto& env0                 = envelopes[0];
      const LocalizationData* geometry = nullptr;
      size_t n_frames                  = 0;

      for(size_t i = 0; i < n_envs; ++i) {
         const auto& env = envelopes[i];
         const auto& k   = kept[i];
         const int k_end = k.start + k.n;

         if(k.n <= 0)
            return fail(format(
                "envelope {} keeps no frames: [{}, {})", i, k.start, k_end));
         if(k.start < env_start(env) or k_end > env_end(env))
            return fail(format("envelope {} keeps [{}, {}), which is outside "
                               "its frames [{}, {})",
                               i,
                               k.start,
                               k_end,
                               env_start(env),
                               env_end(env)));
         if(i > 0 and k.start != kept[i - 1].start + kept[i - 1].n)
            return fail(format("keep ranges do not tile: envelope {} keeps "
                               "from frame {}, but envelope {} ends at {}",
                               i,
                               k.start,
                               i - 1,
                               kept[i - 1].start + kept[i - 1].n));
         if(!(std::fabs(env.frame_duration - env0.frame_duration) <= 1e-9))
            return fail(format("envelope {} has frame-duration {}, but "
                               "envelope 0 has {}",
                               i,
                               env.frame_duration,
                               env0.frame_duration));

         for(auto t = k.start; t < k_end; ++t) {
            const auto& ldat = env_frame(env, t);
            if(ldat.frame_no != t)
               return fail(format("envelope {} holds frame-no {} at frame {}",
                                  i,
                                  ldat.frame_no,
                                  t));
            if(geometry == nullptr) geometry = &ldat;
            if(!same_geometry(ldat, *geometry))
               return fail(format("frame {} of envelope {} has a different "
                                  "histogram geometry than frame {}",
                                  t,
                                  i,
                                  geometry->frame_no));
         }
         n_frames += size_t(k.n);
      }

      // Frames computed by both neighbours
      for(size_t i = 0; i + 1 < n_envs; ++i) {
         const auto& a         = envelopes[i];
         const auto& b         = envelopes[i + 1];
         const int first_frame = std::max(env_start(a), env_start(b));
         const int end_frame   = std::min(env_end(a), env_end(b));
         if(end_frame <= first_frame) continue;

         DiffResult diff;
         diff.headers_equal = true;
         diff_range(diff, a, b, first_frame, end_frame - first_frame);
         Json::Value o       = diff.to_json(false);
         o["boundary"]       = int(i);
         o["boundary_frame"] = kept[i + 1].start;
         ret.overlap_reports.push_back(o);
      }

      ret.envelope.start_frame    = unsigned(kept[0].start);
      ret.envelope.frame_duration = env0.frame_duration;
      ret.envelope.loc_data.reserve(n_frames);
      for(size_t i = 0; i < n_envs; ++i)
         for(auto t = kept[i].start; t < kept[i].start + kept[i].n; ++t)
            ret.envelope.loc_data.push_back(env_frame(envelopes[i], t));

   } catch(std::exception& e) {
      return fail(
          format("exception concatenating localization data: {}", e.what()));
   }

   return ret;
}

// --------------------------------------------------------------- envelope-info
//
Json::Value envelope_info(const LocalizationDataEnvelope& env,
                          const string_view fname) noexcept
{
   Json::Value o{Json::objectValue};
   const int n_frames = int(env.loc_data.size());

   if(!fname.empty()) {
      o["file"] = string(fname);
      string dat;
      if(!file_get_contents(fname, dat)) {
         o["bytes"] = Json::UInt64(dat.size());
         o["md5"]   = md5(dat);
         if(n_frames > 0) o["bytes_per_frame"] = real(dat.size()) / n_frames;
      }
   }

   o["start_frame"]    = env_start(env);
   o["n_frames"]       = n_frames;
   o["end_frame"]      = env_end(env); // exclusive
   o["frame_duration"] = env.frame_duration;
   o["fps"]            = 1.0 / env.frame_duration;
   o["memory_usage"]   = Json::UInt64(env.memory_usage());

   if(n_frames > 0) {
      const auto& ldat0 = env.loc_data.front();
      o["hist_w"]       = ldat0.loc_hist.width;
      o["hist_h"]       = ldat0.loc_hist.height;
      o["hist_sz"]      = ldat0.hist_sz;
      Json::Value bounds{Json::arrayValue};
      for(auto i = 0; i < 4; ++i) bounds.append(ldat0.bounds[i]);
      o["bounds"]    = bounds;
      o["n_sensors"] = int(ldat0.p2ds.size());

      int p2d_total            = 0;
      int p2d_min              = std::numeric_limits<int>::max();
      int p2d_max              = 0;
      bool frame_no_contiguous = true;
      for(auto i = 0; i < n_frames; ++i) {
         const auto& ldat = env.loc_data[size_t(i)];
         const int count  = count_p2ds(ldat);
         p2d_total += count;
         p2d_min = std::min(p2d_min, count);
         p2d_max = std::max(p2d_max, count);
         if(ldat.frame_no != env_start(env) + i) frame_no_contiguous = false;
      }
      o["p2d_count_total"]     = p2d_total;
      o["p2d_count_min"]       = p2d_min;
      o["p2d_count_max"]       = p2d_max;
      o["p2d_count_mean"]      = real(p2d_total) / n_frames;
      o["frame_no_contiguous"] = frame_no_contiguous;
   }

   { // md5 of each frame's bytes, and of all of them in order
      Json::Value md5s{Json::arrayValue};
      string all_md5s = ""s;
      for(const auto& ldat : env.loc_data) {
         const auto s = frame_md5(ldat);
         md5s.append(s);
         all_md5s += s;
      }
      o["frame_md5"]  = md5s;
      o["frames_md5"] = md5(all_md5s);
   }

   return o;
}

} // namespace perceive::ldata
