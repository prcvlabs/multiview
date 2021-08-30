
#pragma once

#include <string>

namespace perceive::cuda
{
// These functions have no effect if CUDA is not compiled in

void init_cuda() noexcept;
bool cuda_is_available() noexcept;
int get_device_count() noexcept;
void set_device(int device) noexcept;
int get_device() noexcept;

struct DeviceInfo
{
   int device_id{0};
   std::string name;
   int major_version{0}; // 0 indicates no compute capability
   int minor_version{0};
   bool is_compatible{false};

   std::string to_string() const noexcept;
};

DeviceInfo get_device_info() noexcept;
DeviceInfo get_device_info(int device) noexcept;

void print_cuda_report() noexcept;

} // namespace perceive::cuda
