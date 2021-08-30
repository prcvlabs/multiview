
#include "slic_impl.hpp"
#include <algorithm>

namespace perceive
{
namespace cuda
{
   namespace slic
   {
      static __device__ void
      init_one_cluster(ImageLayout<Cluster>& clusters,
                       const ImageLayout<Color>& input_image,
                       const GridSampling& x_samples,
                       const GridSampling& y_samples,
                       unsigned spixel_col,
                       unsigned spixel_row)
      {
         const auto x{x_samples.sample_center(spixel_col)};
         const auto y{y_samples.sample_center(spixel_row)};

         Cluster& cluster{clusters.pixel(spixel_col, spixel_row)};
         cluster.x     = float(x);
         cluster.y     = float(y);
         cluster.color = input_image.pixel(x, y);
      }

      static __device__ float dist_to_cluster(const unsigned x,
                                              const unsigned y,
                                              const Color& color,
                                              const Cluster& cluster,
                                              const float xy_weight)
      {
         float dist_lab_sq{0.0f};
         for(int i = 0; i < 3; ++i) {
            const float d{color[i] - cluster.color[i]};
            dist_lab_sq += d * d;
         }

         const float dx{x - cluster.x};
         const float dy{y - cluster.y};
         const float dist_xy_sq{dx * dx + dy * dy};

         return dist_lab_sq + dist_xy_sq * xy_weight;
      }

      static __device__ void
      assign_one_pixel(const ImageLayout<Cluster>& clusters,
                       const ImageLayout<Color>& input_image,
                       ImageLayout<int>& labels,
                       const GridSampling& x_samples,
                       const GridSampling& y_samples,
                       const unsigned x,
                       const unsigned y,
                       const float xy_weight)
      {
         const int center_clust_col = x_samples.sample_index(x);
         const int center_clust_row = y_samples.sample_index(y);

         const int ncols = x_samples.n_samples();
         const int nrows = y_samples.n_samples();

         int closest_cluster{-1};
         float closest_dist{INFINITY};

         const Color& pixel{input_image.pixel(x, y)};

         for(auto clust_row = center_clust_row - 1;
             clust_row <= center_clust_row + 1;
             ++clust_row) {
            for(auto clust_col = center_clust_col - 1;
                clust_col <= center_clust_col + 1;
                ++clust_col) {
               if(clust_col >= 0 && clust_row >= 0 && clust_col < ncols
                  && clust_row < nrows) {
                  unsigned cluster_idx = ncols * clust_row + clust_col;
                  const Cluster& cluster{clusters.pixel(clust_col, clust_row)};
                  auto d = dist_to_cluster(x, y, pixel, cluster, xy_weight);
                  if(d < closest_dist) {
                     closest_dist    = d;
                     closest_cluster = cluster_idx;
                  }
               }
            }
         }

         labels.pixel(x, y) = closest_cluster;
      }

      static __device__ void
      calc_one_cluster_center(ImageLayout<Cluster>& clusters,
                              const unsigned cluster_col,
                              const unsigned cluster_row,
                              const unsigned step,
                              const ImageLayout<Color>& colors,
                              const ImageLayout<int>& labels)
      {
         Color color_sum(0, 0, 0);
         float x_sum{0};
         float y_sum{0};
         unsigned n_pixels{0};
         Cluster& cluster{clusters.pixel(cluster_col, cluster_row)};
         const auto label_idx = int(cluster_row * clusters.width + cluster_col);

         for(int y{int(cluster.y) - int(step)}; y < int(cluster.y) + int(step);
             ++y) {
            for(int x{int(cluster.x) - int(step)};
                x < int(cluster.x) + int(step);
                ++x) {
               if(x >= 0 && y >= 0 && unsigned(x) < labels.width
                  && unsigned(y) < labels.height) {
                  if(labels.pixel(x, y) == label_idx) {
                     ++n_pixels;
                     const Color& pixel_color{colors.pixel(x, y)};
                     for(int i{0}; i < 3; ++i) {
                        color_sum[i] += pixel_color[i];
                     }
                     x_sum += x;
                     y_sum += y;
                  }
               }
            }
         }

         const float fraction{1.0f / n_pixels};
         cluster.x = x_sum * fraction;
         cluster.y = y_sum * fraction;
         for(int i{0}; i < 3; ++i) {
            cluster.color[i] = color_sum[i] * fraction;
         }
      }

      static __device__ void
      enforce_connectivity_for_pixel(const ImageLayout<Cluster>& clusters,
                                     const ImageLayout<Color>& colors,
                                     const ImageLayout<int>& labels,
                                     ImageLayout<int>& new_labels,
                                     const unsigned pix_x,
                                     const unsigned pix_y)
      {
         constexpr int border{2};
         constexpr unsigned min_same{10};
         const int pix_label{labels.pixel(pix_x, pix_y)};
         const int pix_ix = int(pix_x);
         const int pix_iy = int(pix_y);
         const int w      = int(colors.width);
         const int h      = int(colors.height);

         int other_label{-1};
         int other_label_dist{border + border + 1};
         unsigned same_count{0};

         for(int dy = -border; dy <= border; ++dy) {
            int y = pix_iy + dy;
            for(int dx = -border; dx <= border; ++dx) {
               int x = pix_ix + dx;
               if(x >= 0 && x < w && y >= 0 && y < h) {
                  const int neighbor_label{labels.pixel(x, y)};
                  const int neighbor_dist{abs(dx) + abs(dy)};
                  if(neighbor_label == pix_label) {
                     ++same_count;
                  } else if(neighbor_dist < other_label_dist) {
                     other_label      = neighbor_label;
                     other_label_dist = neighbor_dist;
                  }
               }
            }
         }

         if(other_label != -1 && same_count < min_same) {
            new_labels.pixel(pix_x, pix_y) = other_label;
         } else {
            new_labels.pixel(pix_x, pix_y) = pix_label;
         }
      }

      template<typename F>
      __device__ static void strided_loop_1d(const unsigned width,
                                             const F& body)
      {
         const unsigned chunk_width{blockDim.x * gridDim.x};
         const unsigned chunk_x{blockIdx.x * blockDim.x + threadIdx.x};
         for(unsigned x{chunk_x}; x < width; x += chunk_width) { body(x); }
      }

      template<typename F>
      __device__ static void strided_loop_2d(const unsigned width,
                                             const unsigned height,
                                             const F& body)
      {
         const unsigned chunk_width{blockDim.x * gridDim.x};
         const unsigned chunk_height{blockDim.y * gridDim.y};
         const unsigned chunk_x{blockIdx.x * blockDim.x + threadIdx.x};
         const unsigned chunk_y{blockIdx.y * blockDim.y + threadIdx.y};

         for(unsigned y{chunk_y}; y < height; y += chunk_height) {
            for(unsigned x{chunk_x}; x < width; x += chunk_width) {
               body(x, y);
            }
         }
      }

      __global__ void kern_init_clusters(ImageLayout<Cluster> clusters,
                                         const ImageLayout<Color> input_image,
                                         const GridSampling x_samples,
                                         const GridSampling y_samples)
      {
         const unsigned cols = x_samples.n_samples();
         const unsigned rows = y_samples.n_samples();

         strided_loop_2d(cols, rows, [&](unsigned col, unsigned row) {
            init_one_cluster(
                clusters, input_image, x_samples, y_samples, col, row);
         });
      }

      __global__ void kern_assign_pixels(const ImageLayout<Cluster> clusters,
                                         const ImageLayout<Color> input_image,
                                         ImageLayout<int> labels,
                                         const GridSampling x_samples,
                                         const GridSampling y_samples,
                                         const float xy_weight)
      {
         const auto width{input_image.width};
         const auto height{input_image.height};

         strided_loop_2d(width, height, [&](unsigned x, unsigned y) {
            assign_one_pixel(clusters,
                             input_image,
                             labels,
                             x_samples,
                             y_samples,
                             x,
                             y,
                             xy_weight);
         });
      }

      __global__ void kern_calc_cluster_centers(ImageLayout<Cluster> clusters,
                                                const unsigned step,
                                                const ImageLayout<Color> colors,
                                                const ImageLayout<int> labels)
      {
         strided_loop_2d(
             clusters.width, clusters.height, [&](unsigned col, unsigned row) {
                calc_one_cluster_center(
                    clusters, col, row, step, colors, labels);
             });
      }

      __global__ void
      kern_enforce_connectivity(const ImageLayout<Cluster> clusters,
                                const ImageLayout<Color> colors,
                                const ImageLayout<int> labels,
                                ImageLayout<int> new_labels)
      {
         strided_loop_2d(
             colors.width, colors.height, [&](unsigned x, unsigned y) {
                enforce_connectivity_for_pixel(
                    clusters, colors, labels, new_labels, x, y);
             });
      }

      __global__ void
      kern_import_lab_from_linear_pixels(const float* lab_pixels,
                                         ImageLayout<Color> dest)
      {
         strided_loop_2d(dest.width, dest.height, [&](unsigned x, unsigned y) {
            Color& dest_pixel      = dest.pixel(x, y);
            const float* src_pixel = lab_pixels + (3 * (y * dest.width + x));
            for(int i{0}; i < 3; ++i) { dest_pixel[i] = src_pixel[i]; }
         });
      }

      __global__ void
      kern_export_labels_to_linear_pixels(const ImageLayout<int> src,
                                          int* label_pixels)
      {
         strided_loop_2d(src.width, src.height, [&](unsigned x, unsigned y) {
            label_pixels[y * src.width + x] = src.pixel(x, y);
         });
      }

      static dim3
      calc_grid_size(dim3& block_size, unsigned width, unsigned height)
      {
         return dim3{std::min(16u, (width + block_size.x - 1) / block_size.x),
                     std::min(16u, (height + block_size.y - 1) / block_size.y)};
      }

      static constexpr bool sync_everything{false};

      void init_clusters(ClusterImage& clusters,
                         const ColorImage& input_image,
                         const GridSampling& x_samples,
                         const GridSampling& y_samples,
                         cudaStream_t stream)
      {
         dim3 block_size{16, 16};
         dim3 grid_size{calc_grid_size(
             block_size, x_samples.n_samples(), y_samples.n_samples())};
         kern_init_clusters<<<block_size, grid_size, 0, stream>>>(
             clusters.layout(), input_image.layout(), x_samples, y_samples);
         CUDA_CHECK(cudaGetLastError());
         if(sync_everything) { CUDA_CHECK(cudaDeviceSynchronize()); }
      }

      void assign_pixels(const ClusterImage& clusters,
                         const ColorImage& input_image,
                         LabelImage& labels,
                         const GridSampling& x_samples,
                         const GridSampling& y_samples,
                         const float xy_weight,
                         cudaStream_t stream)
      {
         dim3 block_size{16, 16};
         dim3 grid_size{
             calc_grid_size(block_size, labels.width(), labels.height())};
         kern_assign_pixels<<<block_size, grid_size, 0, stream>>>(
             clusters.layout(),
             input_image.layout(),
             labels.layout(),
             x_samples,
             y_samples,
             xy_weight);
         CUDA_CHECK(cudaGetLastError());
         if(sync_everything) { CUDA_CHECK(cudaDeviceSynchronize()); }
      }

      void calc_cluster_centers(ClusterImage& clusters,
                                const unsigned step,
                                const ColorImage& colors,
                                const LabelImage& labels,
                                cudaStream_t stream)
      {
         dim3 block_size{16, 16};
         dim3 grid_size{
             calc_grid_size(block_size, clusters.width(), clusters.height())};
         kern_calc_cluster_centers<<<block_size, grid_size, 0, stream>>>(
             clusters.layout(), step, colors.layout(), labels.layout());
         CUDA_CHECK(cudaGetLastError());
         if(sync_everything) { CUDA_CHECK(cudaDeviceSynchronize()); }
      }

      void enforce_connectivity(const ClusterImage& clusters,
                                const ColorImage& colors,
                                const LabelImage& labels,
                                LabelImage& new_labels,
                                cudaStream_t stream)
      {
         dim3 block_size{16, 16};
         dim3 grid_size{
             calc_grid_size(block_size, labels.width(), labels.height())};
         kern_enforce_connectivity<<<block_size, grid_size, 0, stream>>>(
             clusters.layout(),
             colors.layout(),
             labels.layout(),
             new_labels.layout());
         CUDA_CHECK(cudaGetLastError());
         if(sync_everything) { CUDA_CHECK(cudaDeviceSynchronize()); }
      }

      void import_lab_from_linear_pixels(const float* lab_pixels,
                                         ColorImage& dest,
                                         cudaStream_t stream)
      {
         dim3 block_size{16, 16};
         dim3 grid_size{
             calc_grid_size(block_size, dest.width(), dest.height())};
         kern_import_lab_from_linear_pixels<<<block_size,
                                              grid_size,
                                              0,
                                              stream>>>(lab_pixels,
                                                        dest.layout());
         CUDA_CHECK(cudaGetLastError());
         if(sync_everything) { CUDA_CHECK(cudaDeviceSynchronize()); }
      }

      void export_labels_to_linear_pixels(const LabelImage& src,
                                          int* label_pixels,
                                          cudaStream_t stream)
      {
         dim3 block_size{16, 16};
         dim3 grid_size{calc_grid_size(block_size, src.width(), src.height())};
         kern_export_labels_to_linear_pixels<<<block_size,
                                               grid_size,
                                               0,
                                               stream>>>(src.layout(),
                                                         label_pixels);
         CUDA_CHECK(cudaGetLastError());
         if(sync_everything) { CUDA_CHECK(cudaDeviceSynchronize()); }
      }

   } // namespace slic
} // namespace cuda
} // namespace perceive
