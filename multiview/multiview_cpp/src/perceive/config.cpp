
#include "config.hpp"

#include "stdinc.hpp"

#include "perceive/utils/file-system.hpp"

#include "json/json.h"

#include <stdlib.h>
#include <unistd.h>

#include <mutex>

#include <boost/lexical_cast.hpp>

#ifdef WITH_CUDA
#include <cuda_runtime.h>
#endif

namespace perceive
{
struct EnvironmentVariables
{
   bool is_init                       = false;
   std::string installation_root      = ""s;
   std::string hyperpose_path         = ""s;
   std::string openpose_path          = ""s;
   std::string asset_store            = ""s;
   std::string asset_dir              = ""s;
   std::string asset_s3_bucket        = ""s;
   std::string cache_dir              = ""s;
   std::string lazy_s3_dir            = ""s;
   std::string testcase_s3_dir        = ""s;
   std::string testcase_cache_dir     = ""s;
   std::string cam_pos_annotation_dir = ""s;
   std::string svm_train_gpu_exec     = ""s;
   bool openpose_serial               = false;
   bool openpose_disabled             = false;
   bool trace_mode                    = false;
   bool trace_pipeline_mode           = false;
   bool debug_locks                   = false;
   int cuda_num_gpus                  = -1;
   int cuda_gpu0                      = -1;
   Json::Value env_obj                = Json::Value{Json::nullValue};
   bool hermes_build                  = false;

   string make_config_info_str();
   void init_config(const Json::Value& o);
};

static EnvironmentVariables env_vars_;

static void init_instance(const Json::Value& o) noexcept
{
   env_vars_.init_config(o);
}

static EnvironmentVariables& instance()
{
   if(!env_vars_.is_init)
      FATAL(format("Must call 'load_environment_variables()' before attempting "
                   "to load any environmental variables"));
   return env_vars_;
}

// -------------------------------------------------------- make config info str
//
string EnvironmentVariables::make_config_info_str()
{
   auto make_build_str = []() {
      std::stringstream ss{""};
      bool needs_comma = false;
      auto push_ss     = [&](const string_view s) {
         if(needs_comma) ss << ", ";
         ss << s;
         needs_comma = true;
      };
      auto push_bool = [&](bool val, const string_view s) {
         if(val) push_ss(s);
      };
      push_bool(k_is_cli_build, "cli");
      push_bool(k_is_testcase_build, "testcases");
      push_bool(k_is_gui_build, "gui");
      push_bool(k_is_debug_build, "debug");
      push_bool(k_is_release_build, "release");
      push_bool(k_is_asan_build, "asan");
      push_bool(k_is_usan_build, "usan");
      push_bool(k_is_tsan_build, "tsan");
      push_bool(k_is_benchmark_build, "benchmark");
      push_bool(k_cuda_is_available, "cuda");
      push_bool(k_using_opengl, "opengl");
      return ss.str();
   };

   return format(R"V0G0N(
   k-multiview-version           = '{}'
   build-configuration           =  {}
   installation-root             = '{}'
   hyperpose-path                = '{}'   
   openpose-path                 = '{}'   
   MULTIVIEW_ASSET_STORE         = '{}'
    - MULTIVIEW_ASSET_S3_BUCKET  = '{}'
    - MULTIVIEW_ASSET_DIR        = '{}'
   MULTIVIEW_CACHE_DIR           = '{}'   
   MULTIVIEW_LAZY_S3_DIR         = '{}'
   MULTIVIEW_CUDA_NUM_GPUS       =  {}
   MULTIVIEW_CUDA_GPU0           =  {}
   MULTIVIEW_OPENPOSE_SERIAL     =  {}
   MULTIVIEW_OPENPOSE_DISABLED   =  {}
   MULTIVIEW_TRACE_MODE          =  {}
   MULTIVIEW_TRACE_PIPELINE_MODE =  {}
   MULTIVIEW_DEBUG_LOCKS         =  {}
   HERMES_BUILD                  =  {}
)V0G0N",
                 k_version,
                 make_build_str(),
                 str(installation_root),
                 hyperpose_path,
                 openpose_path,
                 asset_store,
                 asset_s3_bucket,
                 asset_dir,
                 cache_dir,
                 lazy_s3_dir,
                 cuda_num_gpus,
                 cuda_gpu0,
                 str(openpose_serial),
                 str(openpose_disabled),
                 str(trace_mode),
                 str(trace_pipeline_mode),
                 str(debug_locks),
                 str(hermes_build));
}

// -------------------------------------------------------------------- read-env
//
static Json::Value read_env()
{
   Json::Value o{Json::objectValue};

   auto get_it = [](const std::string_view name) -> std::string {
      const char* ss = getenv(name.data());
      if(ss == nullptr) {
         WARN(perceive::format("failed to load environment variable '{}'",
                               name));
         return std::string("");
      }
      return std::string(ss);
   };

   auto get_w_default
       = [&o](const std::string_view name,
              const std::string_view default_value) -> std::string {
      const char* ss = getenv(name.data());
      const auto ret
          = (ss == nullptr) ? std::string(default_value) : std::string(ss);
      o[string(name)] = ret;
      return ret;
   };

   auto get_bool_w_default = [&](const std::string_view name) -> bool {
      const auto val  = get_w_default(name, "");
      const auto ret  = (val == std::string("1") or val == std::string("true"));
      o[string(name)] = ret;
      return ret;
   };

   auto get_int_w_default = [&](const std::string_view name, int def) -> int {
      const auto s = get_w_default(name, "");
      int ret      = def;
      if(s.size() > 0) {
         using boost::lexical_cast;
         using boost::bad_lexical_cast;
         try {
            ret = lexical_cast<int>(s);
         } catch(bad_lexical_cast&) {
            FATAL(
                format("bad lexical cast reading environment variable {}='{}' "
                       "as an integer",
                       name,
                       s));
         }
      }
      o[string(name)] = ret;
      return ret;
   };

   get_w_default("MULTIVIEW_ASSET_STORE", "LAZY_S3");
   get_w_default("MULTIVIEW_ASSET_S3_BUCKET", "s3://perceive-multiview");
   get_w_default("MULTIVIEW_ASSET_DIR", ""s);
   get_w_default("MULTIVIEW_CACHE_DIR", ""s);
   get_w_default("MULTIVIEW_LAZY_S3_DIR", ""s);
   get_bool_w_default("MULTIVIEW_TRACE_MODE");
   get_bool_w_default("MULTIVIEW_DEBUG_LOCKS");
   get_bool_w_default("MULTIVIEW_OPENPOSE_SERIAL");
   get_bool_w_default("MULTIVIEW_OPENPOSE_DISABLED");
   get_int_w_default("MULTIVIEW_CUDA_NUM_GPUS", -1);
   get_int_w_default("MULTIVIEW_CUDA_GPU0", 0);
   get_bool_w_default("HERMES_BUILD");

   return o;
}

const Json::Value& get_env_data()
{
   static std::mutex padlock_;
   static bool first_run_ = true;
   static Json::Value env_data_;
   {
      lock_guard<decltype(padlock_)> lock(padlock_);
      if(first_run_) {
         env_data_  = read_env();
         first_run_ = false;
      }
   }

   return env_data_;
}

// ----------------------------------------------------------------- init config
//
void EnvironmentVariables::init_config(const Json::Value& o)
{
   auto make_cache_base_dir = []() {
      const auto home_s     = getenv("HOME");
      const string home_dir = (home_s == nullptr) ? "" : home_s;
      if(home_dir.empty()) {
         FATAL(format("failed to find $HOME environment variable, aborting."));
      } else if(!is_directory(home_dir)) {
         FATAL(format("Environment variable HOME='{}' is not a directory!",
                      home_dir));
      }
      const auto cache_dir = format("{}/.cache/multiview-data", home_dir);
      if(!is_directory(cache_dir)) mkdir_p(cache_dir);
      if(!is_directory(cache_dir))
         FATAL(format("failed to fine/create cache-dir `{}`", cache_dir));

      return cache_dir;
   };

   const string cache_base_dir = make_cache_base_dir();

   auto load_dir_env_variable
       = [&o](const char* key, const string_view default_value) -> string {
      string dir = o[key].asString();
      if(dir.empty()) {
         dir = default_value.data();
         if(!is_directory(dir))
            if(!mkdir_p(dir))
               FATAL(format(
                   "Could not create {} directory '{}'", key, default_value));
      } else {
         if(!is_directory(dir))
            FATAL(format("{}='{}' does not exist. Aborting.", key, dir));
      }
      Expects(is_directory(dir));
      return dir;
   };

   installation_root = k_installation_root;
   if(!is_directory(installation_root))
      FATAL(format("failed to find installation root: {}", installation_root));

   hyperpose_path = format("{}/opt/hyperpose", installation_root);
#ifdef WITH_HYPERPOSE
   if(!is_directory(hyperpose_path))
      WARN(format("failed to find hyperpose directory = '{}'", hyperpose_path));
#endif
   openpose_path = format("{}/opt/openpose", installation_root);
#ifdef WITH_OPENPOSE
   if(!is_directory(openpose_path))
      WARN(format("failed to find openpose directory = '{}'", openpose_path));
#endif

   asset_store         = o["MULTIVIEW_ASSET_STORE"].asString();
   asset_s3_bucket     = o["MULTIVIEW_ASSET_S3_BUCKET"].asString();
   asset_dir           = o["MULTIVIEW_ASSET_DIR"].asString();
   cache_dir           = load_dir_env_variable("MULTIVIEW_CACHE_DIR",
                                     format("{}/cache", cache_base_dir));
   lazy_s3_dir         = load_dir_env_variable("MULTIVIEW_LAZY_S3_DIR",
                                       format("{}/lazy-s3", cache_base_dir));
   trace_mode          = o["MULTIVIEW_TRACE_MODE"].asBool();
   trace_pipeline_mode = o["MULTIVIEW_TRACE_PIPELINE_MODE"].asBool();
   debug_locks         = o["MULTIVIEW_DEBUG_LOCKS"].asBool();
   openpose_serial     = o["MULTIVIEW_OPENPOSE_SERIAL"].asBool();
   openpose_disabled   = o["MULTIVIEW_OPENPOSE_DISABLED"].asBool();
   cuda_num_gpus       = o["MULTIVIEW_CUDA_NUM_GPUS"].asInt();
   cuda_gpu0           = o["MULTIVIEW_CUDA_GPU0"].asInt();
   env_obj             = o;

   svm_train_gpu_exec
       = format("{}/opt/cc/bin/svm-train-gpu", installation_root);

   testcase_s3_dir    = format("{}/testcases", asset_s3_bucket);
   testcase_cache_dir = absolute_path(format("{}/../testcases", cache_dir));

   cam_pos_annotation_dir = format(
       "{}/calibration/camera-positioning-annotations"s, asset_s3_bucket);

   is_init = true;
}

// -------------------------------------------------- load environment variables
//

void load_environment_variables() noexcept { init_instance(get_env_data()); }

void set_environment_variables(const Json::Value o) noexcept
{
   init_instance(o);
}

// --------------------------------------------------------------------- getters
//
const std::string& multiview_asset_store() noexcept
{
   return instance().asset_store;
}

const std::string& multiview_asset_dir() noexcept
{
   return instance().asset_dir;
}

const std::string& multiview_asset_s3_bucket() noexcept
{
   return instance().asset_s3_bucket;
}

const std::string& multiview_cache_dir() noexcept
{
   return instance().cache_dir;
}

const std::string& multiview_lazy_s3_dir() noexcept
{
   return instance().lazy_s3_dir;
}

const std::string& multiview_testcase_s3_dir() noexcept
{
   return instance().testcase_s3_dir;
}

const std::string& multiview_testcase_cache_dir() noexcept
{
   return instance().testcase_cache_dir;
}

const std::string& multiview_camera_pos_annotation_dir() noexcept
{
   return instance().cam_pos_annotation_dir;
}

const std::string& multiview_hyperpose_path() noexcept
{
   return instance().hyperpose_path;
}

const std::string& multiview_openpose_path() noexcept
{
   return instance().openpose_path;
}

const std::string& svm_train_gpu_exec() noexcept
{
   return instance().svm_train_gpu_exec;
}

bool multiview_openpose_serial() noexcept { return instance().openpose_serial; }

bool multiview_trace_mode() noexcept { return instance().trace_mode; }

bool multiview_trace_pipeline_mode() noexcept
{
   return instance().trace_pipeline_mode;
}

bool multiview_debug_locks() noexcept { return instance().debug_locks; }

int multiview_cuda_num_gpus() noexcept { return instance().cuda_num_gpus; }

int multiview_cuda_gpu0() noexcept { return instance().cuda_gpu0; }

bool multiview_openpose_disabled() noexcept
{
   return instance().openpose_disabled;
}

// ---------------------------------------------------------- configuration-info
//
string environment_info() noexcept { return instance().make_config_info_str(); }

std::string environment_json_str() noexcept
{
   std::stringstream ss{""};
   ss << instance().env_obj;
   return ss.str();
}

// ------------------------------------------------------------- is-hermes-build
//
bool is_hermes_build() noexcept { return instance().hermes_build; }

// ------------------------------------------------------- set-cuda-device-reset
//
static void cuda_device_reset_handler()
{
#ifdef WITH_CUDA
   cudaDeviceReset();
#endif
}

void set_cuda_device_reset() noexcept
{
#ifdef WITH_CUDA
   std::atexit(cuda_device_reset_handler);
#endif
}

} // namespace perceive
