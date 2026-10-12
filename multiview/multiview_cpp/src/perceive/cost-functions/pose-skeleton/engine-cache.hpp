
#pragma once

#include "perceive/io/struct-meta.hpp"

namespace perceive
{
// ------------------------------------------------------------------- EngineKey
// A serialized TensorRT engine (a "plan") is only valid for the model, batch
// size, input size and precision it was built with, and only on the TensorRT
// version and the GPU that built it.
//
struct EngineKey final : public MetaCompatible
{
   virtual ~EngineKey() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   string model_md5   = ""s; // of the onnx file
   int max_batch_size = 0;
   int input_h        = 0;
   int input_w        = 0;
   string precision   = ""s; // "fp32" or "fp16"
   int trt_version    = 0;   // getInferLibVersion()
   string gpu_name    = ""s;
   int compute_major  = 0; // compute capability
   int compute_minor  = 0;

   string digest() const noexcept; // md5 of the fields above
};

// ---------------------------------------------------------------- Engine Cache
// A plan is cached as `<dir>/<digest>.plan`, beside a `<dir>/<digest>.json`
// sidecar that holds the key and the md5 of the plan. TensorRT exits the
// process when it reads a corrupt plan, so a plan is only used when it matches
// its sidecar.
//
string make_engine_sidecar(const EngineKey& key,
                           const string_view plan_md5) noexcept;

bool engine_sidecar_matches(const string& sidecar,
                            const EngineKey& key,
                            const string_view plan_md5) noexcept;

// The filename of the cached plan, or "" if there is no plan, or it does not
// match its sidecar.
string find_cached_engine(const string_view dir, const EngineKey& key) noexcept;

// `save(filename)` writes the plan. The plan and its sidecar are written under
// temporary names, and then renamed. Returns the plan's filename, or "" on
// failure.
string cache_engine(const string_view dir,
                    const EngineKey& key,
                    std::function<void(const string& filename)> save) noexcept;

} // namespace perceive
