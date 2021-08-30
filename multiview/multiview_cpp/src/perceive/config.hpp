
#pragma once

#include <string>

namespace Json
{
class Value;
}

namespace perceive
{
#ifdef USING_OPENGL
constexpr bool k_using_opengl = true;
#else
constexpr bool k_using_opengl = false;
#endif

#ifdef WITH_CUDA
constexpr bool k_cuda_is_available = true;
#else
constexpr bool k_cuda_is_available = false;
#endif

#ifdef TESTCASE_BUILD
constexpr bool k_is_testcase_build = true;
#else
constexpr bool k_is_testcase_build = false;
#endif

#ifdef GUI_BUILD
constexpr bool k_is_gui_build = true;
static_assert(!k_is_testcase_build, "cannot build gui and testcases together");
#else
constexpr bool k_is_gui_build = false;
#endif

constexpr bool k_is_cli_build = !k_is_gui_build and !k_is_testcase_build;

#ifdef DEBUG_BUILD
constexpr bool k_is_debug_build = true;
#else
constexpr bool k_is_debug_build = false;
#endif

#ifdef RELEASE_BUILD
constexpr bool k_is_release_build = true;
#else
constexpr bool k_is_release_build = false;
#endif

#ifdef ADDRESS_SANITIZE
constexpr bool k_is_asan_build = true;
#else
constexpr bool k_is_asan_build = false;
#endif

#ifdef UNDEFINED_SANITIZE
constexpr bool k_is_usan_build = true;
#else
constexpr bool k_is_usan_build = false;
#endif

#ifdef THREAD_SANITIZE
constexpr bool k_is_tsan_build = true;
#else
constexpr bool k_is_tsan_build = false;
#endif

#ifdef BENCHMARK
constexpr bool k_is_benchmark_build = true;
#else
constexpr bool k_is_benchmark_build = false;
#endif

#ifdef WITH_HYPERPOSE
constexpr bool k_has_hyperpose = true;
#else
constexpr bool k_has_hyperpose = false;
#endif

#ifdef WITH_OPENPOSE
constexpr bool k_has_openpose = true;
#else
constexpr bool k_has_openpose = false;
#endif

constexpr const char* k_installation_root = INSTALLATION_ROOT;

constexpr const char* k_version = MULTIVIEW_VERSION;

// Must be called before any of the functions below
void load_environment_variables() noexcept;

// NOT thread safe
void set_environment_variables(const Json::Value o) noexcept;

const std::string& multiview_asset_store() noexcept;
const std::string& multiview_asset_dir() noexcept;
const std::string& multiview_asset_s3_bucket() noexcept; // has s3://
const std::string& multiview_cache_dir() noexcept;
const std::string& multiview_lazy_s3_dir() noexcept;
const std::string& multiview_testcase_s3_dir() noexcept;
const std::string& multiview_testcase_cache_dir() noexcept;
const std::string& multiview_camera_pos_annotation_dir() noexcept;
const std::string& multiview_hyperpose_path() noexcept;
const std::string& multiview_openpose_path() noexcept;
const std::string& svm_train_gpu_exec() noexcept;

bool multiview_openpose_serial() noexcept;
bool multiview_trace_mode() noexcept;
bool multiview_trace_pipeline_mode() noexcept;
bool multiview_debug_locks() noexcept;
int multiview_cuda_num_gpus() noexcept;
int multiview_cuda_gpu0() noexcept;
bool multiview_openpose_disabled() noexcept;

std::string environment_info() noexcept;
std::string environment_json_str() noexcept;

bool is_hermes_build() noexcept;

// sets a std::atexit() call iff cuda is compiled in
void set_cuda_device_reset() noexcept;

} // namespace perceive
