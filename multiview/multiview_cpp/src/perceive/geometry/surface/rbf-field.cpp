
#include <alglib/ap.h>
#include <alglib/interpolation.h>

#include "perceive/utils/md5.hpp"
#include "rbf-field.hpp"

#define This RBFField

namespace perceive
{
// ----------------------------------------------------------------------- Pimpl

enum FieldType : unsigned { FIELD_NONE = 0, RBF, QNN, BICUBIC };

class This::Pimpl
{
 public:
   Pimpl()
   {
      alglib::rbfcreate(2, 2, model); // 2d => 2d field
      src_aabb = AABB::minmax();
      dst_aabb = AABB::minmax();
   }

   ~Pimpl() = default;

   Pimpl(const Pimpl& o)
   {
      src_aabb = o.src_aabb;
      dst_aabb = o.dst_aabb;
      type     = o.type;
      if(type == FieldType::BICUBIC) {
         x_map = o.x_map;
         y_map = o.y_map;
      } else if(type == FieldType::FIELD_NONE) {
         // Do nothing
      } else {
         std::stringstream ss("");
         rbfserialize(const_cast<alglib::rbfmodel&>(o.model), ss);
         std::string s = ss.str();
         std::istringstream os(s);
         rbfunserialize(s, model);
      }
   }

   AABB src_aabb;
   AABB dst_aabb;

   FieldType type{FIELD_NONE};

   alglib::rbfreport rep;
   alglib::rbfmodel model;

   BicubicSpline x_map, y_map;

   void init(const vector<Vector2>& src,
             const vector<Vector2>& dst,
             FieldType type_,
             double rbase,
             unsigned layers,
             double smoothing,
             double qq,
             double zz);
};

// ---------------------------------------------------------------- Construction

This::RBFField()
    : pimpl_(new This::Pimpl())
{}
This::RBFField(const RBFField& o) { *this = o; }
This::RBFField(RBFField&&) = default;
This::~RBFField()          = default;
RBFField& This::operator   =(const RBFField& o)
{
   if(this == &o) return *this;
   pimpl_ = unique_ptr<Pimpl>(new This::Pimpl(*o.pimpl_));
   return *this;
}
RBFField& This::operator=(RBFField&&) = default;

// --------------------------------------------------------------------- Getters

const AABB& This::src_aabb() const { return pimpl_->src_aabb; }
const BicubicSpline& This::x_map() const { return pimpl_->x_map; }
const BicubicSpline& This::y_map() const { return pimpl_->y_map; }

// ------------------------------------------------------------------------ Init

void This::init(const vector<Vector2>& src,
                const vector<Vector2>& dst,
                double rbase,
                unsigned layers,
                double smoothing)
{
   return pimpl_->init(src, dst, RBF, rbase, layers, smoothing, 0.0, 0.0);
}

void This::Pimpl::init(const vector<Vector2>& src,
                       const vector<Vector2>& dst,
                       FieldType type_,
                       double rbase,
                       unsigned layers,
                       double smoothing,
                       double qq,
                       double zz)
{
   if(src.size() == 0 || src.size() != dst.size()) {
      throw std::runtime_error(format("something is wrong with 'src' "
                                      "(size={}) or 'dst' (size={})",
                                      src.size(),
                                      dst.size()));
   }

   type = type_;

   src_aabb = AABB::minmax();
   dst_aabb = AABB::minmax();

   // Fill the points array
   alglib::real_2d_array xy;
   xy.setlength(alglib::ae_int_t(src.size()), 4);
   for(unsigned i = 0; i < src.size(); ++i) {
      xy[i][0] = src[i].x;
      xy[i][1] = src[i].y;
      xy[i][2] = dst[i].x;
      xy[i][3] = dst[i].y;

      src_aabb.union_point(src[i]);
      dst_aabb.union_point(dst[i]);
   }

   { // Add a margin to the src-aabb
      auto mw = 0.005 * src_aabb.width();
      auto mh = 0.005 * src_aabb.height();
      src_aabb.left -= mw;
      src_aabb.right += mw;
      src_aabb.top -= mh;
      src_aabb.bottom += mh;
   }

   if(type == QNN || type == RBF) {
      try {
         // Create the model
         alglib::rbfcreate(2, 2, model);
         // Set the points into the model
         rbfsetpoints(model, xy);
         // Build the model
         if(type == RBF) {
            rbfsetalgohierarchical(model, rbase, layers, smoothing);
         } else if(type == QNN) {
            rbfsetalgoqnn(model, qq, zz);
         } else {
            FATAL("LOGIC ERROR");
         }
         rbfbuildmodel(model, rep);
      } catch(std::exception& e) {
         LOG_ERR(e.what());
      } catch(...) {
         LOG_ERR("unknown exception");
      }
   } else if(type == BICUBIC) {
      FATAL("LOGIC ERROR");

   } else {
      FATAL(format("Logic error: type not supported"));
   }
}

void This::init_qnn(const vector<Vector2>& src,
                    const vector<Vector2>& dst,
                    double q,
                    double z)
{
   pimpl_->init(src, dst, QNN, 0.0, 0, 0.0, q, z);
}

void This::init_bicubic(const BicubicSpline& x_map, const BicubicSpline& y_map)
{
   pimpl_->type  = BICUBIC;
   pimpl_->x_map = x_map;
   pimpl_->y_map = y_map;
}

static std::pair<Vector4r, Vector4r>
calc_f_dx_dy_dxy(const Vector2& p,
                 std::function<Vector2(const Vector2& x)> func,
                 const real dx = 0.01,
                 const real dy = 0.01)
{
   Vector4r U[2];

   auto f00 = func(p);

   // First order derivatives
   auto fx0 = func(Vector2(p.x - dx, p.y));
   auto fx1 = func(Vector2(p.x + dx, p.y));
   auto fy0 = func(Vector2(p.x, p.y - dy));
   auto fy1 = func(Vector2(p.x, p.y + dy));

   // One second order derivative (dxy)
   auto fx0y0 = func(Vector2(p.x - dx, p.y - dy));
   auto fx1y0 = func(Vector2(p.x + dx, p.y - dy));
   auto fx0y1 = func(Vector2(p.x - dx, p.y + dy));
   auto fx1y1 = func(Vector2(p.x + dx, p.y + dy));

   auto fxy0 = (fx1y0 - fx0y0) / (dx + dy);
   auto fxy1 = (fx1y1 - fx0y1) / (dx + dy);

   for(int i = 0; i < 2; ++i) {
      U[i](0) = f00(i);
      U[i](1) = (fx1(i) - fx0(i)) / (dx + dx);
      U[i](2) = (fy1(i) - fy0(i)) / (dy + dy);
      U[i](3) = (fxy1(i) - fxy0(i)) / (dx + dy);
   }

   // cout << format("dxdy_dxy, p = {}", str(p)) << endl;
   // cout << format("mapx = {:9.6f}, {:9.6f}, {:9.6f}, {:9.6f}",
   //                U[0](0), U[0](1), U[0](2), U[0](3)) << endl;
   // cout << format("mapy = {:9.6f}, {:9.6f}, {:9.6f}, {:9.6f}",
   //                U[1](0), U[1](1), U[1](2), U[1](3)) << endl;
   // cout << endl << endl;

   return std::pair(U[0], U[1]);
}

void This::init_bicubic(const AABB region,
                        const real grid_sz,
                        std::function<Vector2(const Vector2& x)> func,
                        const real dxy_step)
{
   pimpl_->src_aabb = region;
   pimpl_->dst_aabb = AABB::minmax();

   const unsigned spline_cols = unsigned(std::ceil(region.w() / grid_sz));
   const unsigned spline_rows = unsigned(std::ceil(region.h() / grid_sz));

   BicubicSpline mapx, mapy;
   mapx.init(region, spline_cols, spline_rows);
   mapy.init(region, spline_cols, spline_rows);

   const real dxy = std::isfinite(dxy_step) ? dxy_step : (grid_sz * 1e-6);
   for(int j = 0; j < int(spline_rows); ++j)
      for(int i = 0; i < int(spline_cols); ++i)
         std::tie(mapx.X(i, j), mapy.X(i, j))
             = calc_f_dx_dy_dxy(mapx.position(i, j), func, dxy, dxy);

   mapx.finalize();
   mapy.finalize();

   pimpl_->type  = BICUBIC;
   pimpl_->x_map = mapx;
   pimpl_->y_map = mapy;
}

// ------------------------------------------------------------------ Fill Field

void This::fill_field(EmpiricalField& f, unsigned w, unsigned h) const
{
   const auto& src_aabb = pimpl_->src_aabb;

   // Initialize the field metrics
   f.field.resize(w, h);

   Matrix3r H = rescale_homography(AABB(0, 0, w, h), src_aabb);

   f.fxy.x = H(0, 0);
   f.fxy.y = H(1, 1);
   f.ppt.x = H(0, 2);
   f.ppt.y = H(1, 2);

   if(pimpl_->type == FIELD_NONE) {
      f.field.zero();
      return;
   } else if(pimpl_->type == BICUBIC) {
      for(unsigned y = 0; y < h; ++y) {
         for(unsigned x = 0; x < w; ++x) {
            Vector2 F     = Vector2(x, y);
            Vector2 U     = f.inv_transform(F);
            Vector2 V     = this->evaluate(U);
            f.field(x, y) = V;
         }
      }

   } else {
      alglib::real_1d_array out;
      alglib::real_1d_array x_axis, y_axis;
      x_axis.setlength(w);
      y_axis.setlength(h);

      for(unsigned i = 0; i < w; ++i)
         x_axis[i] = f.inv_transform(Vector2(i, 0)).x;
      for(unsigned i = 0; i < h; ++i)
         y_axis[i] = f.inv_transform(Vector2(0, i)).y;
      alglib::smp_rbfgridcalc2v(pimpl_->model, x_axis, w, y_axis, h, out);

      const auto expected_length = 2 * w * h;
      if(out.length() != expected_length) {
         LOG_ERR(format(
             "Expected {} outputs, but got {}", expected_length, out.length()));
         return;
      }

      unsigned pos = 0;
      for(unsigned y = 0; y < h; ++y)
         for(unsigned x = 0; x < w; ++x)
            for(unsigned i = 0; i < 2; ++i) f.field(x, y)(int(i)) = out[pos++];
   }
}

Vector2 This::evaluate(const Vector2& x) const
{
   static thread_local alglib::real_1d_array xx, xy;

   switch(pimpl_->type) {
   case FIELD_NONE: return Vector2::nan();
   case QNN:
   case RBF:
      if(xx.length() != 2) {
         xx.setlength(2);
         xy.setlength(2);
      }
      xx[0] = x.x;
      xx[1] = x.y;
      alglib::rbfcalc(pimpl_->model, xx, xy);
      return Vector2(xy[0], xy[1]);
   case BICUBIC:
      return Vector2(pimpl_->x_map.evaluate(x.x, x.y),
                     pimpl_->y_map.evaluate(x.x, x.y));
   default: FATAL("LOGIC ERROR");
   }

   return Vector2::nan();
}

void This::serialize(std::ostream& s_out) const
{
   const auto& src_aabb = pimpl_->src_aabb;
   const auto& dst_aabb = pimpl_->dst_aabb;

   s_out << format("{} {} {} {}",
                   src_aabb.left,
                   src_aabb.top,
                   src_aabb.right,
                   src_aabb.bottom)
         << endl;
   s_out << format("{} {} {} {}",
                   dst_aabb.left,
                   dst_aabb.top,
                   dst_aabb.right,
                   dst_aabb.bottom)
         << endl;
   s_out << int(pimpl_->type) << endl;

   if(pimpl_->type == FIELD_NONE) {
      // Do nothing
   } else if(pimpl_->type == QNN || pimpl_->type == RBF) {
      rbfserialize(pimpl_->model, s_out);
   } else if(pimpl_->type == BICUBIC) {
      pimpl_->x_map.serialize(s_out);
      pimpl_->y_map.serialize(s_out);
   }
}

void This::unserialize(std::istream& s_in)
{
   auto& src_aabb = pimpl_->src_aabb;
   auto& dst_aabb = pimpl_->dst_aabb;

   s_in >> src_aabb.left;
   s_in >> src_aabb.top;
   s_in >> src_aabb.right;
   s_in >> src_aabb.bottom;
   s_in >> dst_aabb.left;
   s_in >> dst_aabb.top;
   s_in >> dst_aabb.right;
   s_in >> dst_aabb.bottom;

   int val = 0;
   s_in >> val;
   pimpl_->type = FieldType(val);

   if(pimpl_->type == FIELD_NONE) {
      // Do nothing
   } else if(pimpl_->type == QNN || pimpl_->type == RBF) {
      rbfunserialize(s_in, pimpl_->model);
   } else if(pimpl_->type == BICUBIC) {
      pimpl_->x_map.unserialize(s_in);
      pimpl_->y_map.unserialize(s_in);
   }
}

std::string This::to_string() const
{
   const auto& src_aabb = pimpl_->src_aabb;
   const auto& dst_aabb = pimpl_->dst_aabb;

   const auto& model                = pimpl_->model;
   const alglib_impl::ae_vector& cw = model.c_ptr()->model2.cw;
   unsigned cww                     = unsigned(cw.cnt);
   double cwh                       = cww / 4.0;

   auto type_s = "NONE";
   switch(pimpl_->type) {
   case FIELD_NONE: type_s = "NONE"; break;
   case QNN: type_s = "QNN"; break;
   case RBF: type_s = "RBF"; break;
   case BICUBIC: type_s = "BICUBIC"; break;
   default: FATAL("Logic Error");
   }

   std::stringstream ss("");
   serialize(ss);
   MD5 md5(ss.str());

   return format(R"V0G0N(RBF-Field
   type:     {}
   src-aabb: {}
   dst-aabb: {}
   checksum: {}
   cw:       {}x{}
-----
)V0G0N",
                 type_s,
                 src_aabb.to_string(),
                 dst_aabb.to_string(),
                 md5.hexdigest(),
                 cww,
                 cwh);
}

} // namespace perceive
