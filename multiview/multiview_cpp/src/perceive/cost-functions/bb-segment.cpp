
#include "stdinc.hpp"

#include "bb-segment.hpp"

#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/graphics/integral-image.hpp"
#include "perceive/utils/tick-tock.hpp"

#include <deque>

#define This BBSegment

namespace perceive
{
using std::deque;
using std::function;

// ------------------------------------------------------------------ operator==

bool This::Params::operator==(const Params& o) const
{
#define TEST(field) (field == o.field)
   return true && TEST(object_threshold) && TEST(max_object_radius)
          && TEST(min_object_radius) && TEST(object_skirt) && TEST(max_regions)
          && TEST(nms_region_overlap) && TEST(region_overlap)
          && TEST(run_choose_best);
#undef TEST
}

// ------------------------------------------------------------------- to-string

string This::Params::to_string() const
{
   return format(R"V0G0N(
Segment
  object threshold   = {}
  max object radius  = {}
  min object radius  = {}
  object skirt       = {}
  max num regions    = {}
  nms region overlap = {}
  final overlap      = {}
  run choose-best    = {}
{})V0G0N",
                 object_threshold,
                 max_object_radius,
                 min_object_radius,
                 object_skirt,
                 max_regions,
                 nms_region_overlap,
                 region_overlap,
                 str(run_choose_best),
                 "");
}

// ------------------------------------------------------------- ImageIntegrator

struct ImageIntegrator
{
 private:
   int w{0};
   int h{0};
   vector<float> data;

 public:
   ImageIntegrator() = default;
   ImageIntegrator(const FloatImage& im) { init(im); }

   void init(const FloatImage& im)
   {
      w = int(im.width);
      h = int(im.height);
      data.resize(size_t(w * h));
      make_integral_image(im.pixels, unsigned(w), unsigned(h), &data[0]);
   }

   float sum(int l, int t, int r, int b) const noexcept
   {
      return integral_region(&data[0], w, h, l, t, r, b);
   }

   float sum(AABBi x) const noexcept
   {
      return integral_region(&data[0], w, h, x.left, x.top, x.right, x.bottom);
   }

   auto width() const noexcept { return w; }
   auto height() const noexcept { return h; }
};

// ----------------------------------------------------------- Object Hypothesis

This::ObjectHypothesis::ObjectHypothesis(Point2 X_, float score_, AABBi aabb_)
    : X(X_)
    , score(score_)
    , aabb(aabb_)
{}

// --------------------------------------------------------------- Extract AABBs

static void extract_aabbs(const vector<This::ObjectHypothesis>& in,
                          vector<AABBi>& out)
{
   out.resize(in.size());
   std::transform(
       cbegin(in), cend(in), begin(out), [&](auto& X) { return X.aabb; });
}

static vector<AABBi> extract_aabbs(const vector<This::ObjectHypothesis>& in)
{
   vector<AABBi> out(in.size());
   extract_aabbs(in, out);
   return out;
}

// -------------------------------------------------- Make AABBs non-overlapping

static void make_AABBs_non_overlapping(const vector<AABBi>& in,
                                       vector<AABBi>& out)
{
   out.clear();

   std::function<void(const AABBi&)> add_non_overlapping = [&](const AABBi& x) {
      const auto x_area = x.area();

      for(auto ii = begin(out); ii != end(out); ++ii) {
         const auto box = *ii;
         auto overlap   = intersection_area(box, x);

         if(overlap > 0) {
            if(overlap == x_area) return; // box 'x' doesn't add anything

            // We have overlap, so we split 'x' into parts, and
            // attempt to add them.
            // Eventually we should find that the smaller parts
            // don't add any "area", or don't overlap with anything
            // in which canse it is added to 'out' below

            int total_pushed_area = 0;
            auto push_aabb        = [&](int l, int t, int r, int b) {
               const AABBi new_aabb(l, t, r, b);
               Expects(intersection_area(new_aabb, box) == 0);
               Expects(intersection_area(new_aabb, x) > 0);
               total_pushed_area += new_aabb.area();
               add_non_overlapping(new_aabb);
            };

            // Split aabb into 1 or 2 non-overlapping regions
            const bool spills_left  = x.left < box.left;
            const bool spills_up    = x.top < box.top;
            const bool spills_right = x.right > box.right;
            const bool spills_down  = x.bottom > box.bottom;

            if(spills_left) push_aabb(x.left, x.top, box.left, x.bottom);
            if(spills_right) push_aabb(box.right, x.top, x.right, x.bottom);
            if(spills_up)
               push_aabb(std::max(x.left, box.left),
                         x.top,
                         std::min(x.right, box.right),
                         box.top);
            if(spills_down)
               push_aabb(std::max(x.left, box.left),
                         box.bottom,
                         std::min(x.right, box.right),
                         x.bottom);

            Expects((overlap + total_pushed_area - x.area()) == 0);
            return;
         }
      }

      out.push_back(x); //
   };

   for(const auto& x : in) add_non_overlapping(x);
}

// -------------------------------------------------------------- score function

static float score_func(const ImageIntegrator& integrator,
                        int skirt,
                        int l,
                        int t,
                        int r,
                        int b)
{
   if(r < l or b < t) return 0.0;
   auto s           = skirt;
   auto inner_score = integrator.sum(l, t, r, b);
   auto skirt_score = integrator.sum(l - s, t - s, r + s, b + s) - inner_score;
   auto score       = inner_score - skirt_score;
   return (score < 0.0f) ? 0.0f : score;
}

static float
score_func(const ImageIntegrator& integrator, int skirt, AABBi aabb)
{
   return score_func(
       integrator, skirt, aabb.left, aabb.top, aabb.right, aabb.bottom);
}

static int calc_object_skirt(const real hist_side, const This::Params& p)
{
   const auto score_skirt = (p.object_skirt - p.max_object_radius) / hist_side;
   return int(std::round(score_skirt)); // as an integer
}

// ----------------------------------------------------- Make Initial Hypotheses

static vector<This::ObjectHypothesis>
make_initial_hypotheses(const ImageIntegrator& integrator,
                        const real hist_side,
                        const This::Params& p)
{
   // Create an integral image
   const auto w         = integrator.width();
   const auto h         = integrator.height();
   const int max_radius = int(std::round(p.max_object_radius / hist_side));
   const int min_radius = int(std::round(p.min_object_radius / hist_side));
   const int object_threshold = int(p.object_threshold);
   vector<This::ObjectHypothesis> objects;
   objects.reserve(size_t(w * h));

   const int i_skirt = calc_object_skirt(hist_side, p);

   auto make_aabb = [](int x, int y, int radius) {
      const int l = x - radius;
      const int t = y - radius;
      const int r = x + radius - 1; // inclusive
      const int b = y + radius - 1; // inclusive
      return AABBi(l, t, r, b);
   };

   auto score_radius = [&](int x, int y, int radius) {
      return score_func(integrator, i_skirt, make_aabb(x, y, max_radius));
   };

   auto make_object_hypothesis = [&](int x, int y) {
      // We want the min object size that captures 95% of max score
      const auto max_score = score_radius(x, y, max_radius);
      if(max_score > float(object_threshold)) {
         const auto target_score = 0.95f * max_score;
         int found_r             = 0;
         for(int r = min_radius; r < max_radius and found_r == 0; ++r) {
            const auto score = score_radius(x, y, r);
            if(score >= target_score) found_r = r + 4;
         }

         if(found_r > max_radius) found_r = max_radius;

         if(found_r > 0) {
            const auto aabb  = make_aabb(x, y, found_r);
            const auto score = score_func(integrator, i_skirt, aabb);
            auto object = This::ObjectHypothesis(Point2(x, y), score, aabb);
            if(object.score >= float(object_threshold))
               objects.push_back(object);
         }
      }
   };

   for(auto y = 0; y < h; ++y)
      for(auto x = 0; x < w; ++x) make_object_hypothesis(x, y);

   return objects;
}

// ----------------------------------------------------- non-maximal suppression

static vector<This::ObjectHypothesis>
non_maximal_suppression(const vector<This::ObjectHypothesis>& init_objects,
                        const This::Params& p)
{
   vector<This::ObjectHypothesis> sorted(cbegin(init_objects),
                                         cend(init_objects));
   std::sort(begin(sorted), end(sorted), [&](auto& a, auto& b) {
      return a.score > b.score;
   });

   vector<This::ObjectHypothesis> out;

   for(const auto& object : sorted) {
      if(out.size() >= p.max_regions) break; // We're done

      // Do we have a collision?
      bool collision = false;
      for(const auto& box : out) {
         const float i_area = float(intersection_area(box.aabb, object.aabb));
         const float u_area = float(union_area(box.aabb, object.aabb));
         if(i_area / u_area > float(p.nms_region_overlap)) {
            collision = true;
            break;
         }
      }

      if(!collision) // if no collision, then add to result
         out.push_back(object);
   }

   return out;
}

// ------------------------------------------------------------ branch and bound

static vector<This::ObjectHypothesis>
bnb_choose_best_objects(const vector<This::ObjectHypothesis>& objects,
                        const ImageIntegrator& integrator,
                        const This::Params& p)
{
   typedef vector<int> obj_set_t;

   // Test if two regions conflict
   auto conflicts = [&](const size_t ind1, const size_t ind2) {
      float i_area
          = float(intersection_area(objects[ind1].aabb, objects[ind2].aabb));
      float u_area = float(union_area(objects[ind1].aabb, objects[ind2].aabb));
      return (i_area / u_area) > float(p.region_overlap);
   };

   auto now = tick(); // We will time-limit bnb

   unsigned counter;
   float best_bound = 0;
   obj_set_t best;
   obj_set_t initial_sol;                              // empty
   obj_set_t initial_rem(objects.size());              // Initial remainder set
   std::iota(begin(initial_rem), end(initial_rem), 0); // is _everything_

   vector<AABBi> aabbs1, aabbs2;

   std::function<void(const obj_set_t&, const int, const obj_set_t&)> bb
       = [&](const obj_set_t& parent_sol,
             const float score,
             const obj_set_t& parent_rem) {
            if(parent_rem.size() == 0) {
               // We have a solution, is it better?
               if(score > best_bound) {
                  best_bound = score;
                  best       = parent_sol;
               }
               return;
            }

            // Copy the parent's solution
            obj_set_t sol;
            sol.reserve(parent_sol.size() + 1);
            sol.insert(end(sol), cbegin(parent_sol), cend(parent_sol));

            // The remainder set
            obj_set_t rem;

            for(const auto i : parent_rem) {
               // Add 'i' to the solution
               sol.push_back(i);

               // Remove conflicting with 'i' from the remainder
               rem.clear();
               for(const auto j : parent_rem)
                  if(i != j and !conflicts(size_t(i), size_t(j)))
                     rem.push_back(j);

               // Current score for this solution...
               const float sol_score = score + objects[size_t(i)].score;

               // What is the bound on this solution? A tight lower bound
               // means far fewer function calls.

               // First decompose solution and remaining aabbs into
               // non-overlapping regions
               aabbs1.resize(rem.size());
               std::transform(
                   cbegin(rem), cend(rem), begin(aabbs1), [&](int ind) {
                      return objects[size_t(ind)].aabb;
                   });
               make_AABBs_non_overlapping(aabbs1, aabbs2);
               auto rem_score = 0.0f;
               for(const auto& aabb : aabbs2) rem_score += integrator.sum(aabb);

               auto bound = sol_score + rem_score;

               // Is this actually correct?
               if(bound > best_bound) bb(sol, int(sol_score), rem);

               // Remove 'i' from the solution
               sol.pop_back();
            }
         };

   // Run branch-n-bound
   bb(initial_sol, 0.0f, initial_rem);

   // Copy the result to 'out'
   vector<This::ObjectHypothesis> out(best.size());
   std::transform(cbegin(best), cend(best), begin(out), [&](auto ind) {
      return objects[size_t(ind)];
   });

   { // set labels
      int counter = 0;
      for(auto& o : out) o.label = counter++;
   }

   return out;
}

// ---------------------------------------------------------------- Draw Regions

static void draw_regions(const vector<AABBi>& aabbs,
                         ARGBImage& im,
                         real alpha         = 0.5,
                         real outer_grow_sz = 0.0)
{
   const int w = int(im.width), h = int(im.height);

   auto colour_region = [&](const AABBi& aabb, unsigned colour, real alpha) {
      int bi = aabb.bottom;
      int ri = aabb.right;
      for(int y = aabb.top; y <= bi; ++y)
         for(int x = aabb.left; x <= ri; ++x)
            if(y >= 0 and x >= 0 and y < h and x < w) {
               auto& pixel = im(x, y);
               pixel       = blend(pixel, colour, float(alpha));
            }
   };

   auto counter = 0u;
   for(const auto& aabb : aabbs) {
      uint32_t colour = colour_set_2(counter++);
      colour_region(aabb, colour, alpha);
      if(outer_grow_sz > 0.0) {
         AABBi large = aabb;
         large.grow(int(outer_grow_sz));
         colour_region(large, colour, alpha);
      }
   }
}

static void draw_regions(const vector<This::ObjectHypothesis>& objects,
                         ARGBImage& im,
                         real alpha         = 0.5,
                         real outer_grow_sz = 0.0)
{
   const int w = int(im.width), h = int(im.height);

   auto colour_region = [&](const AABBi& aabb, unsigned colour, real alpha) {
      int bi = aabb.bottom;
      int ri = aabb.right;
      for(int y = aabb.top; y <= bi; ++y)
         for(int x = aabb.left; x <= ri; ++x)
            if(y >= 0 and x >= 0 and y < h and x < w) {
               auto& pixel = im(x, y);
               pixel       = blend(pixel, colour, float(alpha));
            }
   };

   for(const auto& o : objects) {
      uint32_t colour = colour_set_2(unsigned(o.label));
      colour_region(o.aabb, colour, alpha);
   }
}

// ------------------------------------------------------------------ bb-segment

vector<This::ObjectHypothesis> This::bb_segment(const FloatImage& hist,
                                                const real hist_side,
                                                const This::Params& p)
{
   const auto integrator = ImageIntegrator(hist);
   const auto w          = hist.width;
   const auto h          = hist.height;

   // Test for an object at each point of the histogram (threshold to remove)
   auto init_objects = make_initial_hypotheses(integrator, hist_side, p);

   // Apply non-maximal suppression
   auto nms_objects = non_maximal_suppression(init_objects, p);

   // Run branch and bound
   auto bnb_objects = bnb_choose_best_objects(nms_objects, integrator, p);

   return bnb_objects;
}

// ------------------------------------------------------------- Test bb-segment

void This::test_bb_segment(const FloatImage& hist,
                           const real hist_side,
                           const This::Params& p,
                           const string& outdir)
{
   INFO("test bb-segment");
   Expects(hist.row_stride == hist.width);

   const auto integrator = ImageIntegrator(hist);
   const auto w          = hist.width;
   const auto h          = hist.height;

   // Test for an object at each point of the histogram (threshold to remove)
   auto init_objects = make_initial_hypotheses(integrator, hist_side, p);

   // Apply non-maximal suppression
   auto nms_objects = non_maximal_suppression(init_objects, p);

   // Run branch and bound
   auto bnb_objects = bnb_choose_best_objects(nms_objects, integrator, p);

   vector<AABBi> non_overlapping;
   make_AABBs_non_overlapping(extract_aabbs(nms_objects), non_overlapping);

   // SAVE information
   INFO(format("init-objects.size() = {}", init_objects.size()));
   INFO(format("nms-objects.size() = {}", nms_objects.size()));

   GreyImage g = float_im_to_grey(hist, [&](float v) { return 1.0f - v; });
   g.save(format("{}/out-hist_bw.png", outdir));

   {
      ARGBImage argb = grey_to_colour(g);
      draw_regions(init_objects, argb);
      argb.save(format("{}/out-hist_init.png", outdir));
   }

   {
      ARGBImage argb = grey_to_colour(g);
      draw_regions(nms_objects, argb);
      argb.save(format("{}/out-hist_nms.png", outdir));
   }

   {
      ARGBImage argb = grey_to_colour(g);
      draw_regions(non_overlapping, argb);
      argb.save(format("{}/out-hist_non-overlapping.png", outdir));
   }

   {
      ARGBImage argb = grey_to_colour(g);
      draw_regions(bnb_objects, argb);
      argb.save(format("{}/out-hist_bnb-objects.png", outdir));
   }
}

// ------------------------------------------------------------------ set labels

void BBSegment::set_labels(vector<ObjectHypothesis>& current,
                           const vector<ObjectHypothesis>& previous)
{
   // Update current labels if they overlap (most overlapping) with
   // a label in previous

   return;

   auto find_closest = [&](ObjectHypothesis& o) {
      auto best_ind = -1;
      auto best_iu  = 0.0f;

      for(auto i = 0u; i < previous.size(); ++i) {
         const auto& u = previous[i];
         float i_area  = float(intersection_area(o.aabb, u.aabb));
         float u_area  = float(union_area(o.aabb, u.aabb));
         float iu      = i_area / u_area;
         if(iu >= 0.5f and iu > best_iu) {
            best_ind = int(i);
            best_iu  = iu;
         }
      }

      if(best_ind >= 0) o.label = previous[size_t(best_ind)].label;
   };

   for(auto& o : current)
      o.label = -1; // reset the label to something nonsensical

   for(auto& o : current)
      find_closest(o); // update labels for overlapping boxes

   // fill in the missing labels
   std::unordered_set<int> used_labels;
   for(auto& o : current)
      if(o.label >= 0) used_labels.insert(o.label);

   int counter         = 0;
   auto get_next_label = [&]() {
      while(used_labels.count(counter) > 0) counter++;
      return counter;
   };

   for(auto& o : current)
      if(o.label < 0) o.label = get_next_label();
}

// ---------------------------------------------------------------------- render

ARGBImage BBSegment::render(const vector<ObjectHypothesis>& objects,
                            const vector<ObjectHypothesis>& previous,
                            const FloatImage& hist,
                            const float cap)
{
   ARGBImage argb;
   argb.resize(hist.width, hist.height, hist.width);

   for(auto y = 0u; y < hist.height; ++y) {
      for(auto x = 0u; x < hist.width; ++x) {
         auto val     = hist(x, y);
         uint8_t grey = uint8_t((1.0f - clamp(val / cap, 0.0f, 1.0f)) * 255.0f);
         argb(x, y)   = make_colour(grey, grey, grey);
      }
   }

   draw_regions(objects, argb);

   return argb;
}

} // namespace perceive
