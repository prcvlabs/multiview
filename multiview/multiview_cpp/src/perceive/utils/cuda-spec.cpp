
#include "cuda-spec.hpp"

#include "perceive/foundation.hpp"

#ifdef WITH_CUDA
#include <cuda_runtime.h>
#endif

namespace perceive::cuda
{
void init_cuda() noexcept { get_device_count(); }
bool cuda_is_available() noexcept
{
   // Compiled with CUDA *and* a usable device is present
   static const bool available
       = k_cuda_is_available and (get_device_count() > 0);
   return available;
}

#ifdef WITH_CUDA

int get_device_count() noexcept
{
   int n = 0;
   if(cudaGetDeviceCount(&n) != cudaSuccess) {
      cudaGetLastError(); // clear the error state
      return 0;
   }
   return n;
}

void set_device(int device) noexcept { cudaSetDevice(device); }

int get_device() noexcept
{
   int device = -1;
   if(cudaGetDevice(&device) != cudaSuccess) return -1;
   return device;
}

DeviceInfo get_device_info() noexcept { return get_device_info(get_device()); }

DeviceInfo get_device_info(int device_id) noexcept
{
   DeviceInfo di{};
   if(device_id < 0 or device_id >= get_device_count()) return di;

   cudaDeviceProp prop{};
   if(cudaGetDeviceProperties(&prop, device_id) != cudaSuccess) return di;
   di.device_id     = device_id;
   di.name          = prop.name;
   di.major_version = prop.major;
   di.minor_version = prop.minor;
   // Compatible if the kernels can run on this device (cudaFuncGetAttributes
   // would be exact; checking for a usable device is close enough here)
   di.is_compatible = (prop.major > 0) and (prop.computeMode != cudaComputeModeProhibited);
   return di;
}

void print_cuda_report() noexcept
{
   const auto N = get_device_count();

   cout << format("Cuda report: ") << endl;
   cout << format("{} devices", N) << endl;
   cout << format("current device: {}", get_device()) << endl;
   cout << endl;
   for(auto i = 0; i < N; ++i) cout << get_device_info(i).to_string() << endl;
}

#else

int get_device_count() noexcept { return 0; }
void set_device(int) noexcept {}
int get_device() noexcept { return -1; }
DeviceInfo get_device_info() noexcept { return get_device_info(0); }
DeviceInfo get_device_info(int) noexcept { return DeviceInfo{}; }
void print_cuda_report() noexcept { cout << "Cuda not available." << endl; }

#endif

std::string DeviceInfo::to_string() const noexcept
{
   return format(R"V0G0N(Cuda Device Info
   device-id:   {}
   name:        {}
   compute:     {}.{}
   compatible:  {}
{})V0G0N",
                 device_id,
                 name,
                 major_version,
                 minor_version,
                 str(is_compatible),
                 "");
}

} // namespace perceive::cuda
