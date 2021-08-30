
#ifndef NAN
#define NAN (0.0 / 0.0)
#endif

#include "canny.hpp"

#include <stack>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

namespace perceive
{
template<typename T>
void cannyT(const uint8_t* grey,
            uint32_t width,
            uint32_t height,
            uint8_t* edges,
            double& sigma,
            double& thres1,
            double& thres2)
{
   const unsigned rows = height;
   const unsigned cols = width;

   assert(sizeof(T) == 4 || sizeof(T) == 8);
   int mat_type = sizeof(T) == 4 ? CV_32F : CV_64F;

   Mat src_im(int(height), int(width), mat_type);
   Mat blurred_im(int(height), int(width), mat_type);
   Mat dx(int(height), int(width), mat_type);
   Mat dy(int(height), int(width), mat_type);

   // Switch to plain old vectors for magnitude, orientation, and nms
   vector<T> mag(height * width);
   vector<T> nms(height * width); // for calculating non-maximal suppresion
   vector<uint8_t> ori(height * width); // major orientation: see below

   // Copy im => src_im
   for(unsigned y = 0; y < height; ++y) {
      const uint8_t* src = &grey[y * width];
      T* dst             = src_im.ptr<T>(int(y));
      for(unsigned x = 0; x < width; ++x) *dst++ = T(*src++);
   }

   // Smooth the image
   if(sigma > 0.0)
      cv::GaussianBlur(src_im, blurred_im, cv::Size(0, 0), sigma);
   else
      blurred_im = src_im;

   // Apply sobel operators
   const unsigned aperture_size = 3;
   cv::Sobel(blurred_im, dx, mat_type, 1, 0, aperture_size);
   cv::Sobel(blurred_im, dy, mat_type, 0, 1, aperture_size);

   // Get magnitudes
   for(unsigned y = 0; y < height; ++y) {
      const T* dx_ptr = dx.ptr<const T>(int(y));
      const T* dy_ptr = dy.ptr<const T>(int(y));
      T* mag_ptr      = &mag[y * width];
      for(unsigned x = 0; x < width; ++x)
         *mag_ptr++ = sqrt(dx_ptr[x] * dx_ptr[x] + dy_ptr[x] * dy_ptr[x]);
   }

   // What are the threshold parameters...
   if(std::isnan(thres1) && std::isnan(thres2)) {
      T mean     = T(0.0);
      T inv_size = T(1.0) / T(width * height);
      for(const auto& magnitude : mag) mean += magnitude * inv_size;
      thres1 = 0.4 * double(mean);
      thres2 = double(mean);
   } else if(std::isnan(thres1) && !std::isnan(thres2)) {
      thres1 = 0.4 * thres2;
   } else if(!std::isnan(thres1) && std::isnan(thres2)) {
      thres2 = thres1 / 0.4;
   }

   const T low  = T(thres1);
   const T high = T(thres2);

   // Zero memory used for nms
   std::fill(nms.begin(), nms.end(), 0.0f); // zero the data

   // Initialize output image
   memset(edges, 0, rows * cols * sizeof(uint8_t));

   // Hysteresis requires a stack
   std::stack<std::pair<int, int>> S;

   { // Non-maximumal supression
      // nms offsets for each of the four directions
      const int w
          = int(width); // dr (row) and dc (col) used in pointer arithmetic
      const int dr[4][2] = {{0, 0}, {-w, w}, {-w, w}, {-w, w}};
      const int dc[4][2] = {{-1, 1}, {1, -1}, {0, 0}, {-1, 1}};

      // dxs contains the dx values on the unit circle for
#define TORAD(t) ((t) *M_PI / 180.0)
      const T dxs[] = {
          // dx values on unit circle
          T(cos(TORAD(0.5 * (0.0 + 45.0)))),   // 22.5 degrees
          T(cos(TORAD(0.5 * (45.0 + 90.0)))),  // 67.5
          T(cos(TORAD(0.5 * (90.0 + 135.0)))), // 112.5
          T(cos(TORAD(0.5 * (135.0 + 180.0)))) // 157.5
      };
#undef TORAD

      for(int y = 1; y < int(rows) - 1; ++y) {
         // For finding best orientation
         const T* dx_ptr = (dx.ptr<const T>(y)) + 1;
         const T* dy_ptr = (dy.ptr<const T>(y)) + 1;

         unsigned offset  = unsigned(y) * width + 1;
         const T* mag_ptr = &mag[offset]; // retrieving orientation magntude
         auto* ori_ptr    = &ori[offset]; // storing best orientation
         T* nms_ptr       = &nms[offset]; // storing nms result

         for(int x = 1; x < int(cols) - 1; ++x) {
            const T magnitude = *mag_ptr;

            T dx     = (*dy_ptr > T(0.0) ? -(*dx_ptr) : *dx_ptr) / magnitude;
            int best = 0; // and the best direction is:
            if(dx > dxs[0])
               best = 0; // 0 degrees
            else if(dx > dxs[1])
               best = 1; // 45 degrees
            else if(dx > dxs[2])
               best = 2; // 90 degrees
            else if(dx > dxs[3])
               best = 3; // 135 degrees
            else
               best = 0;              // back to 0 degrees
            *ori_ptr = uint8_t(best); // store best orientation

            // If magnitude is good for given orientation
            if(magnitude > *(mag_ptr + dc[best][0] + dr[best][0])
               && magnitude > *(mag_ptr + dc[best][1] + dr[best][1]))
               *nms_ptr = magnitude;

            // Seeds for Hysteresis
            if(*nms_ptr > high) S.push(std::make_pair(y, x));

            mag_ptr++;
            ori_ptr++;
            nms_ptr++;
            dx_ptr++;
            dy_ptr++;
         }
      }
   }

   { // Hysteresis
      const int w        = int(width);
      const int dy[4][2] = {{-1, 1}, {-1, 1}, {0, 0}, {-1, 1}};
      const int dx[4][2] = {{0, 0}, {-1, 1}, {-1, 1}, {1, -1}};
      const int dr[4][2] = {{-w, w}, {-w, w}, {0, 0}, {-w, w}};

      while(!S.empty()) {
         auto p = S.top();
         S.pop();

         const int r = p.first, c = p.second;

         if(r <= 0 || r + 1 >= int(rows) || c <= 0 || c + 1 >= int(cols))
            continue;

         unsigned index = unsigned(r) * width + unsigned(c);
         if(edges[index] > 0) continue;

         edges[index] = 255;
         const int d  = ori[index]; // orientation direction
         assert(d >= 0 && d < 4);
         if(nms[unsigned(int(index) + dx[d][0] + dr[d][0])] > low)
            S.push(std::make_pair(r + dy[d][0], c + dx[d][0]));
         if(nms[unsigned(int(index) + dx[d][1] + dr[d][1])] > low)
            S.push(std::make_pair(r + dy[d][1], c + dx[d][1]));
      }
   }
}

void canny(const uint8_t* grey,
           uint32_t width,
           uint32_t height,
           uint8_t* edges,
           double sigma,
           double* low_ptr,
           double* high_ptr)
{
   double low  = low_ptr == nullptr ? dNAN : *low_ptr;
   double high = high_ptr == nullptr ? dNAN : *high_ptr;

   cannyT<float>(grey, width, height, edges, sigma, low, high);

   if(low_ptr != NULL) *low_ptr = low;
   if(high_ptr != NULL) *high_ptr = high;
}

} // namespace perceive
