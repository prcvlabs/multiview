
#include "rgb-to-lab.hpp"

#include "cuda-check.hpp"
#include "perceive/utils/cuda-spec.hpp"

namespace perceive::cuda
{
void cuda_convert_rgb_to_lab(
    const uint32_t* rgb_pixel_start,
    const uint32_t* rgb_pixel_end,
    float* lab_pixel_start, // pointer to 'L' value of first pixel. L A and
                            // B values are assumed to be contiguous
    size_t lab_pixel_size,  // bytes from one pixel to next
    cudaStream_t stream);

// Zero-copy (mapped, pinned) host memory that the kernel reads/writes
// directly.
struct MappedBuffer
{
   void* data{nullptr};
   size_t size{0};
   MappedBuffer()                               = default;
   MappedBuffer(const MappedBuffer&)            = delete;
   MappedBuffer& operator=(const MappedBuffer&) = delete;
   ~MappedBuffer()
   {
      if(data != nullptr) CUDA_CHECK(cudaFreeHost(data));
   }

   void create(size_t n_bytes) // no-op if already large enough
   {
      if(n_bytes <= size) return;
      if(data != nullptr) CUDA_CHECK(cudaFreeHost(data));
      data = nullptr;
      size = 0;
      CUDA_CHECK(cudaHostAlloc(&data, n_bytes, cudaHostAllocMapped));
      size = n_bytes;
   }
};

struct ARGBToLABConversion::Pimpl
{
   MappedBuffer input_buffer;
   MappedBuffer output_buffer;
   cudaStream_t stream{nullptr};
   std::mutex lock;

   Pimpl();
   ~Pimpl();
};

ARGBToLABConversion::Pimpl::Pimpl() { CUDA_CHECK(cudaStreamCreate(&stream)); }

ARGBToLABConversion::Pimpl::~Pimpl() { CUDA_CHECK(cudaStreamDestroy(stream)); }

ARGBToLABConversion::ARGBToLABConversion()
{
   if(cuda_is_available()) pimpl_ = std::make_unique<Pimpl>();
}

ARGBToLABConversion::~ARGBToLABConversion() = default;

Vec3fImage ARGBToLABConversion::convert(const ARGBImage& argb) const
{
   Expects(argb.width > 0 and argb.height > 0);
   Expects(argb.width == argb.row_stride);
   Expects(pimpl_ != nullptr);

   auto& P = *pimpl_;
   lock_guard<std::mutex> padlock(P.lock);

   P.input_buffer.create(argb.n_bytes());
   P.output_buffer.create(sizeof(Vector3f) * argb.width * argb.height);

   ARGBImage argb_shared{false};
   argb_shared.set_pix(argb.width,
                       argb.height,
                       argb.width,
                       reinterpret_cast<uint32_t*>(P.input_buffer.data));
   Vec3fImage lab_shared{false};
   lab_shared.set_pix(argb.width,
                      argb.height,
                      argb.width,
                      reinterpret_cast<Vector3f*>(P.output_buffer.data));

   for(unsigned y = 0; y < argb.height; ++y) {
      memcpy(argb_shared.ptr(y), argb.ptr(y), argb.row_bytes());
   }

   float* lab_start = lab_shared(0, 0).ptr();
   cuda_convert_rgb_to_lab(argb_shared.data(),
                           argb_shared.data() + argb.height * argb.row_stride,
                           lab_start,
                           sizeof(Vector3f),
                           P.stream);

   CUDA_CHECK(cudaStreamSynchronize(P.stream));

   Vec3fImage result{lab_shared};
   return result;
}

} // namespace perceive::cuda
