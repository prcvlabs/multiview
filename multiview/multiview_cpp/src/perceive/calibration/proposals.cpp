
#include "perceive/graphics/colour-set.hpp"
#include "proposals.hpp"

#include <set>

#define This Proposals

namespace perceive
{
vector<unsigned> calc_proposals(const vector<vector<Vector3r>> img_pts,
                                const double resolution,
                                const unsigned rounds,
                                const bool shuffle,
                                const string feedback_image_fname)
{
   AABB aabb;
   ImageContainerT<std::vector<unsigned>> histogram;
   std::set<unsigned> inds;
   ARGBImage counters;
   vector<unsigned> out;

   auto transform = [&](const Vector3r& X) -> Point2 {
      auto x = (Vector2(X(0), X(1)) - aabb.top_left()) / resolution;
      auto p = to_pt2(x);
      if(!histogram.in_bounds(p)) FATAL("kBAM!");
      return p;
   };

   { // Find the bounds
      aabb = AABB::minmax();
      for(const auto& img : img_pts)
         for(const auto& X : img) aabb.union_point(Vector2(X(0), X(1)));
   }

   { // Count into the histogram
      histogram.resize(int(ceil(aabb.width() / resolution)),
                       int(ceil(aabb.height() / resolution)));

      for(auto i = 0u; i < img_pts.size(); ++i)
         for(const auto& X : img_pts[i]) histogram(transform(X)).push_back(i);
   }

   if(shuffle) { // Shuffle the orders of everything in the histograms...
      std::mt19937 gen(pure_random());
      for(unsigned y = 0; y < histogram.height; ++y) {
         for(unsigned x = 0; x < histogram.width; ++x) {
            auto& idxs = histogram(x, y);
            std::shuffle(idxs.begin(), idxs.end(), gen);
         }
      }
   }

   { // Prepare the 'counters' where we "count-add" in images
      counters.resize(histogram.width, histogram.height);
      counters.zero();
   }

   auto add_ind = [&](unsigned grid_ind) {
      for(const auto& X : img_pts[grid_ind]) {
         auto p = transform(X);
         counters(p) += 1;
      }
      inds.insert(grid_ind);
   };

   for(unsigned y = 0; y < histogram.height; ++y)
      for(unsigned x = 0; x < histogram.width; ++x)
         for(const auto grid_ind : histogram(x, y))
            if(inds.count(grid_ind) == 0 && counters(x, y) < rounds)
               add_ind(grid_ind);

   out.insert(out.end(), inds.begin(), inds.end());

   if(!feedback_image_fname.empty()) {
      ARGBImage im;
      im.resize(int(ceil(aabb.right)), int(ceil(aabb.bottom)));
      im.zero();
      unsigned counter = 0;
      for(const auto& ind : out) {
         for(const auto& Xr : img_pts[ind]) {
            Point2 p = to_pt2(Vector2(Xr(0), Xr(1)));
            if(im.in_bounds(p)) im(p) = k_crayons[counter % n_crayons()];
         }
         counter++;
      }
      auto filename = feedback_image_fname;
      im.save(filename);
   }

   return out;
}

// Point2 This::transform(const Vector3r& X) const
// {
//     auto x = (Vector2(X(0), X(1)) - aabb.top_left()) / resolution;
//     auto p = Point2(x.x, x.y);
//     if(!histogram.in_bounds(p))
//         FATAL("kBAM!");
//     return p;
// }

// void This::init(const vector< vector<Vector3r> >& base_img_pts,
//                 const double resolution_,
//                 bool feedback)
// {
//     if(feedback)
//         INFO(format("Initializing proposals with resolution = {}",
//                     resolution_));

//     resolution = resolution_;

//     // Find the bounds
//     aabb = AABB::minmax();
//     for(const auto& img_pts: base_img_pts)
//         for(const auto& X: img_pts)
//             aabb.union_point(Vector2(X(0), X(1)));

//     // Count into the histogram
//     histogram.resize(ceil(aabb.width() / resolution),
//                      ceil(aabb.height() / resolution));

//     for(unsigned i = 0; i < base_img_pts.size(); ++i)
//         for(const auto& X: base_img_pts[i])
//             histogram(transform(X)).push_back(i);

//     // Shuffle the orders of everything in the histograms...
//     for(unsigned y = 0; y < histogram.height; ++y) {
//         for(unsigned x = 0; x < histogram.width; ++x) {
//             auto& idxs = histogram(x, y);
//             std::random_shuffle(idxs.begin(), idxs.end());
//         }
//     }

//     img_pts = base_img_pts;
// }

// void This::calc_proposals(vector< vector<Vector3r> >& out,
//                           const unsigned rounds,
//                           bool feedback) const
// {
//     std::set<unsigned> inds;

//     ARGBImage counters;
//     counters.resize(histogram.width, histogram.height);
//     counters.zero();

//     auto add_ind = [&] (unsigned grid_ind) {
//         for(const auto& X: img_pts[grid_ind]) {
//             auto p = transform(X);
//             counters(p) += 1;
//         }
//         inds.insert(grid_ind);
//     };

//     for(unsigned y = 0; y < histogram.height; ++y) {
//         for(unsigned x = 0; x < histogram.width; ++x) {
//             unsigned counter = 0;
//             for(const auto grid_ind: histogram(x, y)) {
//                 if(inds.count(grid_ind) == 0 && counters(x, y) < rounds) {
//                     add_ind(grid_ind);
//                 }
//             }
//         }
//     }

//     out.clear();
//     out.reserve(inds.size());
//     for(auto ind: inds)
//         out.push_back(img_pts[ind]);

//     if(feedback) {
//         ARGBImage im;
//         im.resize(ceil(aabb.right), ceil(aabb.bottom));
//         im.zero();
//         unsigned counter = 0;
//         for(const auto& pts: out) {
//             for(const auto& Xr: pts) {
//                 Point2 p(Xr(0), Xr(1));
//                 if(im.in_bounds(p))
//                     im(p) = k_crayons[counter % n_crayons()];
//             }
//             counter++;
//         }
//         auto filename = "/tmp/yy-proposals-image.png";
//         im.save(filename);
//         INFO(format("Calced proposals of size {}, saved to {}",
//                     out.size(), filename));
//     }
// }

} // namespace perceive
