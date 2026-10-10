/*
Copyright 2016 Fixstars Corporation

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http ://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include "internal.h"

// 3x3 median filter. This replaces nppiFilterMedian_16u_C1R, which read
// outside the image (anchor {0,0} over the full ROI) and pulled in NPP.
// Border pixels are copied through unfiltered.
namespace {

	__device__ inline void sort2(uint16_t& a, uint16_t& b) {
		const uint16_t t = min(a, b); b = max(a, b); a = t;
	}

	__global__ void median_filter_3x3_kernel(const uint16_t* __restrict__ d_src, uint16_t* __restrict__ d_dst, int width, int height) {
		const int x = blockIdx.x * blockDim.x + threadIdx.x;
		const int y = blockIdx.y * blockDim.y + threadIdx.y;
		if (x >= width || y >= height) { return; }
		if (x == 0 || y == 0 || x == width - 1 || y == height - 1) {
			d_dst[y * width + x] = d_src[y * width + x];
			return;
		}
		uint16_t v[9];
		for (int dy = -1, k = 0; dy <= 1; ++dy)
			for (int dx = -1; dx <= 1; ++dx)
				v[k++] = d_src[(y + dy) * width + (x + dx)];
		// median-of-9 sorting network
		sort2(v[1], v[2]); sort2(v[4], v[5]); sort2(v[7], v[8]);
		sort2(v[0], v[1]); sort2(v[3], v[4]); sort2(v[6], v[7]);
		sort2(v[1], v[2]); sort2(v[4], v[5]); sort2(v[7], v[8]);
		sort2(v[0], v[3]); sort2(v[5], v[8]); sort2(v[4], v[7]);
		sort2(v[3], v[6]); sort2(v[1], v[4]); sort2(v[2], v[5]);
		sort2(v[4], v[7]); sort2(v[4], v[2]); sort2(v[6], v[4]);
		sort2(v[4], v[2]);
		d_dst[y * width + x] = v[4];
	}

}

namespace sgm {
	namespace details {

		void median_filter(const uint16_t* d_src, uint16_t* d_dst, void* /* unused */, int width, int height) {
			const dim3 threads(32, 8);
			const dim3 blocks((width + threads.x - 1) / threads.x, (height + threads.y - 1) / threads.y);
			median_filter_3x3_kernel<<<blocks, threads>>>(d_src, d_dst, width, height);
			CudaKernelCheck();
		}

	}
}
