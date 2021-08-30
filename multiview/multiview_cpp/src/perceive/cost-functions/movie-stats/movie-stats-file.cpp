
#include "movie-stats-file.hpp"

#include "perceive/graphics/colour-set.hpp"
#include "perceive/io/fp-io.hpp"
#include "perceive/io/lazy-s3.hpp"
#include "perceive/utils/file-system.hpp"
#define This MovieStatsFile

namespace perceive
{
static const string k_legacy_load_save_magic{"MovieStatsFile_v01\n"};
static const string k_load_save_magic{"MovieStatsFile_v02\n"};

size_t This::Accumulator::memory_usage() const noexcept
{
   return sizeof(decltype(*this)) + sum_ss.memory_usage()
          - sizeof(decltype(sum_ss));
}

// ---------------------------------------------------------- incremental update
//
void This::Accumulator::incremental_update(const FloorHistogram& fhist)
{
   const auto w = fhist.hist.width;
   const auto h = fhist.hist.height;

   // -- Process the first frame
   if(N == 0) {
      bounds  = fhist.bounds;
      hist_sz = fhist.hist_sz;
      Expects(bounds.is_finite());
      sum_ss.resize(w, h);
      sum_ss.fill(Vector2{0.0, 0.0});
      K = 0.0;
      for(auto y = 0u; y < h; ++y)
         for(auto x = 0u; x < w; ++x) K += real(fhist.hist(x, y));
      K /= (w * h);
      Expects(std::isfinite(K));
   }

   for(auto y = 0u; y < h; ++y) {
      for(auto x = 0u; x < w; ++x) {
         const auto v = real(fhist.hist(x, y)) - K;
         sum_ss(x, y).x += v;     // sum
         sum_ss(x, y).y += v * v; // sum
      }
   }

   N += 1;
}

// ------------------------------------------------------------------ initialize
//
bool This::initialize(const SceneDescriptionInfo& scene_info,
                      const Accumulator& accumulator)
{
   MovieStatsFile o;
   o.scene_info = scene_info;
   o.bounds     = accumulator.bounds;
   o.hist_sz    = accumulator.hist_sz;
   o.N          = accumulator.N;

   if(o.N == 0) {
      LOG_ERR(
          format("expects at least 1 frames when initializing movie-stats"));
      return false;
   }

   if(!o.bounds.is_finite()) {
      LOG_ERR(format(
          "failed to load finite histogram bounds... aborting operation"));
      return false;
   }

   const auto w     = accumulator.sum_ss.width;
   const auto h     = accumulator.sum_ss.height;
   const auto N     = accumulator.N;
   const auto K     = accumulator.K;
   const real N_inv = 1.0 / real(N);

   o.hist_stats.resize(w, h);

   // Convert to averages and standard-deviation
   for(auto y = 0u; y < h; ++y) {
      for(auto x = 0u; x < w; ++x) {
         const real kx  = accumulator.sum_ss(x, y).x;
         const real kxx = accumulator.sum_ss(x, y).y;

         real sx  = kx * N_inv + K; // average
         real sxx = sqrt(std::fabs((kxx - square(kx) * N_inv) * N_inv));
         if(!std::isfinite(sxx)) {
            WARN(format("sxx(x, y) = sqrt({}) = {}!, using 0.0",
                        (kxx - square(kx) / real(N)) / real(N),
                        sxx));
            sxx = 0.0;
         }

         auto& stat   = o.hist_stats(x, y);
         stat.average = float(sx);
         stat.stddev  = float(sxx);
      }
   }

   *this = o;
   return true;
}

// ------------------------------------------------------------------ initialize
//
bool This::initialize(const SceneDescriptionInfo& in_scene_info,
                      const unsigned n_frames,
                      std::function<const FloorHistogram*(unsigned)> get_hist)
{
   if(n_frames < 1)
      FATAL(format("expects at least 1 frames when initializing movie-stats"));

   Accumulator acc;
   bool has_error = false;
   for(auto i = 0u; i < n_frames and !has_error; ++i) {
      const FloorHistogram* fhist = get_hist(i);
      if(fhist == nullptr) {
         LOG_ERR(format("failed to load floor-histogram for frame {}", i));
         has_error = true;
      } else {
         acc.incremental_update(*fhist);
      }
   }

   if(!has_error) { return initialize(in_scene_info, acc); }
   return !has_error;
}

// --------------------------------------------------------------- export images

void This::export_image(const string& outdir) const
{
   // for(auto i = 0u; i < av_imgs.size(); ++i)
   //    av_imgs[i].save(
   //        format("{}/movie-stats_accumulated_image_{}.png", outdir, i));

   const auto w = hist_stats.width;
   const auto h = hist_stats.height;
   FloatImage median(w, h);
   std::transform(hist_stats.cbegin(),
                  hist_stats.cend(),
                  median.begin(),
                  [](const auto& stat) -> float { return stat.average; });

   const auto [min_val, max_val] = median.minmax();
   const auto inv                = 1.0f / std::min(max_val, 500.0f);
   ARGBImage im                  = float_im_to_argb(
       median, [&](float val) -> float { return 1.0f - val * inv; });

   im.save(format("{}/movie-stats_histogram.png", outdir));
}

// --------------------------------------------------------------------- Combine

MovieStatsFile combine(const MovieStatsFile& a,
                       const MovieStatsFile& b) noexcept
{
   Expects(a.w() == b.w());
   Expects(a.h() == b.h());

   const auto w = a.w();
   const auto h = a.h();

   MovieStatsFile out;
   out   = a;
   out.N = a.N + b.N;

   for(auto y = 0u; y < h; ++y) {
      for(auto x = 0u; x < w; ++x) {
         const auto& a1                  = a.hist_stats(x, y);
         const auto& b1                  = b.hist_stats(x, y);
         auto& s1                        = out.hist_stats(x, y);
         std::tie(s1.average, s1.stddev) = combine_mu_sigma(
             a.N, a1.average, a1.stddev, b.N, b1.average, b1.stddev);
      }
   }

   return out;
}

// ------------------------------------------------------------------------ Load

void load(This& data, const string& fname) noexcept(false)
{
   auto legacy_load_it = [](FILE* fp, This& tmp) -> string {
      try {
         string load_save_magic;
         string scene_info_str;

         // Load-save magic
         legacy_load_str(fp, load_save_magic);
         if(load_save_magic != k_legacy_load_save_magic)
            throw std::runtime_error(format("file identifier '{}' != '{}'",
                                            load_save_magic,
                                            k_legacy_load_save_magic));

         // Scene Info
         legacy_load_str(fp, scene_info_str);
         try {
            read(tmp.scene_info, scene_info_str);
         } catch(std::exception& e) {
            LOG_ERR(
                format("failed to read scene-info from movie-stats file: {}",
                       e.what()));
            cout << scene_info_str;
            FATAL("kBAM!");
         }

         for(auto i = 0; i < 4; ++i)
            legacy_load_real(fp, tmp.bounds[i]); // Bounds
         legacy_load_real(fp, tmp.hist_sz);      // hist-sz
         legacy_load_uint(fp, tmp.N);

         // Hist stats
         unsigned w = 0, h = 0;
         legacy_load_uint(fp, w);
         legacy_load_uint(fp, h);
         tmp.hist_stats.resize(w, h);
         for(auto y = 0u; y < h; ++y)
            for(auto x = 0u; x < w; ++x) {
               auto& s    = tmp.hist_stats(x, y);
               double val = 0.0;
               legacy_load_real(fp, val);
               s.average = float(val);
               legacy_load_real(fp, val);
               s.stddev = float(val);
            }
      } catch(std::exception& e) {
         return e.what();
      }
      return ""s;
   };

   auto load_it = [](FILE* fp, This& tmp) -> string {
      try {
         string load_save_magic;
         string scene_info_str;

         // Load-save magic
         load_str(fp, load_save_magic);
         if(load_save_magic != k_load_save_magic)
            throw std::runtime_error(format("file identifier {} != {}",
                                            load_save_magic,
                                            k_load_save_magic));

         // Scene Info
         load_str(fp, scene_info_str);
         try {
            read(tmp.scene_info, scene_info_str);
         } catch(std::exception& e) {
            LOG_ERR(
                format("failed to read scene-info from movie-stats file: {}",
                       e.what()));
            cout << scene_info_str;
            FATAL("kBAM!");
         }

         for(auto i = 0; i < 4; ++i) load_real(fp, tmp.bounds[i]); // Bounds
         load_real(fp, tmp.hist_sz);                               // hist-sz
         load_uint(fp, tmp.N);

         // Hist stats
         unsigned w = 0, h = 0;
         load_uint(fp, w);
         load_uint(fp, h);
         tmp.hist_stats.resize(w, h);
         for(auto y = 0u; y < h; ++y)
            for(auto x = 0u; x < w; ++x) {
               auto& s    = tmp.hist_stats(x, y);
               double val = 0.0;
               load_real(fp, val);
               s.average = float(val);
               load_real(fp, val);
               s.stddev = float(val);
            }
      } catch(std::exception& e) {
         return e.what();
      }
      return ""s;
   };

   FILE* fp = nullptr;

   try {
      vector<char> buffer;
      This tmp;
      string err_msg;

      lazy_load(fname, buffer);
      fp = fmemopen(&buffer[0], buffer.size(), "rb");
      Expects(fp != nullptr);
      const auto pos = ftell(fp);
      err_msg        = load_it(fp, tmp);
      if(!err_msg.empty()) {
         fseek(fp, pos, SEEK_SET);
         err_msg = legacy_load_it(fp, tmp);
      }
      if(!err_msg.empty()) {
         FATAL(format("kbam = '{}'", err_msg));
         throw std::runtime_error(err_msg);
      }
      data = std::move(tmp);
   } catch(std::runtime_error& e) {
      fclose(fp);
      throw e;
   } catch(std::exception& e) {
      fclose(fp);
      throw e;
   }

   if(fp != nullptr) fclose(fp);
}

// ------------------------------------------------------------------------ Save

void save(const This& data, const std::string& fname) noexcept(false)
{
   auto save_it = [&data](FILE* fp) {
      save_str(fp, k_load_save_magic); // Identifier
      // save_str(fp, data.params.save_to_string()); // Parameters
      string scene_info_str; // Scene-info
      write(data.scene_info, scene_info_str);
      save_str(fp, scene_info_str);

      for(auto i = 0; i < 4; ++i) save_real(fp, data.bounds[i]); // Bounds
      save_real(fp, data.hist_sz);                               // hist-sz
      save_uint(fp, data.N);                                     // n-frames

      save_uint(fp, data.hist_stats.width); // Hist-stats
      save_uint(fp, data.hist_stats.height);
      for(auto y = 0u; y < data.hist_stats.height; ++y)
         for(auto x = 0u; x < data.hist_stats.width; ++x) {
            const auto& s = data.hist_stats(x, y);
            save_real(fp, real(s.average));
            save_real(fp, real(s.stddev));
         }
   };

   FILE* fp = nullptr;
   try {
      fp = tmpfile();
      save_it(fp);

      vector<char> buffer;
      buffer.resize(size_t(ftell(fp)));
      fseek(fp, 0, SEEK_SET);
      if(buffer.size() != fread(&buffer[0], 1, buffer.size(), fp))
         throw std::runtime_error("failed to write file");

      lazy_store(fname, buffer);

   } catch(std::exception& e) {
      if(fp != nullptr) fclose(fp);
      throw e;
   }

   fclose(fp);
}

} // namespace perceive
