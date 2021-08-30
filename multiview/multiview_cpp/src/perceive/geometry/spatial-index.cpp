
#include <list>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace bg  = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<float, 2, bg::cs::cartesian> point2;
typedef bg::model::box<point2> box2;
typedef std::pair<point2, uint> value2;
typedef bgi::rtree<value2, bgi::quadratic<16>> r2tree;

typedef bg::model::point<float, 3, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef std::pair<point, uint> value;
typedef bgi::rtree<value, bgi::quadratic<16>> r3tree;

#include "spatial-index.hpp"

namespace perceive
{
// ------------------------------------------------------------- Spatial-2-Index

inline LabeledVector2 value_to_lv2(const value2& v, const size_t& size)
{
   Expects(v.second < size);
   return std::make_pair(to_vec2(Vector2f(v.first.get<0>(), v.first.get<1>())),
                         v.second);
}

struct Spatial2Index::Pimpl
{
   Pimpl(Spatial2Index& si) {}

   r2tree index;
   size_t size = 0;

   template<typename Iterator> void init(Iterator begin, Iterator end)
   {
      index.clear();
      auto to_r2tree = [&](const Vector2& p, uint idx) {
         return std::make_pair(point2(float(p.x), float(p.y)), idx);
      };
      uint32_t counter = 0;
      while(begin != end) index.insert(to_r2tree(*begin++, counter++));
      size = counter;
   }
};

Spatial2Index::Spatial2Index() { pimpl_ = new Pimpl(*this); }
Spatial2Index::~Spatial2Index() { delete pimpl_; }
Spatial2Index::Spatial2Index(const Spatial2Index& rhs)
    : pimpl_(nullptr)
{
   pimpl_ = new Pimpl(*this);
   *this  = rhs;
}
Spatial2Index::Spatial2Index(Spatial2Index&& rhs)
    : pimpl_(nullptr)
{
   std::swap(pimpl_, rhs.pimpl_);
}
Spatial2Index& Spatial2Index::operator=(const Spatial2Index& rhs)
{
   if(this == &rhs) return *this;
   *pimpl_ = *(rhs.pimpl_);
   return *this;
}
Spatial2Index& Spatial2Index::operator=(Spatial2Index&& rhs)
{
   if(this == &rhs) return *this;
   delete pimpl_;
   pimpl_ = nullptr;
   std::swap(pimpl_, rhs.pimpl_);
   return *this;
}
void Spatial2Index::clear() { pimpl_->index.clear(); }
void Spatial2Index::init(const vector<Vector2>& pts)
{
   pimpl_->init(pts.begin(), pts.end());
}
void Spatial2Index::init(const std::deque<Vector2>& pts)
{
   pimpl_->init(pts.begin(), pts.end());
}
size_t Spatial2Index::size() const noexcept { return pimpl_->size; }

bool Spatial2Index::empty() const noexcept { return size() == 0; }

void Spatial2Index::query_nearest(const Vector2& P,
                                  uint n_points,
                                  std::deque<LabeledVector2>& ret) const
{
   ret.clear();
   if(!empty()) {
      vector<value2> values;
      values.reserve(n_points);
      pimpl_->index.query(
          bgi::nearest(point2(float(P.x), float(P.y)), n_points),
          std::back_inserter(values));
      for(const value2& v : values) ret.push_back(value_to_lv2(v, size()));
   }
}

void Spatial2Index::query_nearest(const Vector2& P,
                                  uint n_points,
                                  vector<LabeledVector2>& ret) const
{
   ret.clear();
   if(!empty()) {
      vector<value2> values;
      values.reserve(n_points);
      pimpl_->index.query(
          bgi::nearest(point2(float(P.x), float(P.y)), n_points),
          std::back_inserter(values));
      ret.resize(values.size());
      for(uint i = 0; i < values.size(); ++i)
         ret[i] = value_to_lv2(values[i], size());
   }
}

LabeledVector2 Spatial2Index::query_nearest(const Vector2& P) const
{
   if(empty()) {
      return std::make_pair(Vector2::nan(), 0u);
   } else {
      std::list<value2> values;
      pimpl_->index.query(bgi::nearest(point2(float(P.x), float(P.y)), 1),
                          std::back_inserter(values));
      return value_to_lv2(values.front(), size());
   }
}

void Spatial2Index::query_region(const AABB& aabb,
                                 std::deque<LabeledVector2>& ret) const
{
   // find values intersecting some area defined by a box
   box2 query_box(point2(float(aabb.left), float(aabb.top)),
                  point2(float(aabb.right), float(aabb.bottom)));
   std::vector<value2> result_s;
   const auto& rtree = pimpl_->index;
   rtree.query(bgi::intersects(query_box), std::back_inserter(result_s));
   for(const value2& v : result_s) ret.push_back(value_to_lv2(v, size()));
}

void Spatial2Index::query_region(const AABB& aabb,
                                 vector<LabeledVector2>& ret) const
{
   // find values intersecting some area defined by a box
   box2 query_box(point2(float(aabb.left), float(aabb.top)),
                  point2(float(aabb.right), float(aabb.bottom)));
   std::vector<value2> values;
   const auto& rtree = pimpl_->index;
   rtree.query(bgi::intersects(query_box), std::back_inserter(values));
   ret.resize(values.size());
   for(uint i = 0; i < values.size(); ++i)
      ret[i] = value_to_lv2(values[i], size());
}

void Spatial2Index::query_region(const AABB& aabb, vector<Vector2>& ret) const
{
   // find values intersecting some area defined by a box
   box2 query_box(point2(float(aabb.left), float(aabb.top)),
                  point2(float(aabb.right), float(aabb.bottom)));
   std::vector<value2> values;
   const auto& rtree = pimpl_->index;
   rtree.query(bgi::intersects(query_box), std::back_inserter(values));
   ret.resize(values.size());
   for(uint i = 0; i < values.size(); ++i)
      ret[i] = to_vec2(
          Vector2f(values[i].first.get<0>(), values[i].first.get<1>()));
}

// ------------------------------------------------------------- Spatial-3-Index

inline LabeledVector3 value_to_lv3(const value& v)
{
   return std::make_pair(
       to_vec3(Vector3f(v.first.get<0>(), v.first.get<1>(), v.first.get<2>())),
       v.second);
}

struct Spatial3Index::Pimpl
{
   Pimpl(Spatial3Index& si) {}

   r3tree index;

   template<typename Iterator> void init(Iterator begin, Iterator end)
   {
      index.clear();
      auto to_r3tree = [&](const Vector3& q, uint idx) {
         const Vector3f p = to_vec3f(q);
         return std::make_pair(point(p.x, p.y, p.z), idx);
      };
      uint counter = 0;
      while(begin != end) index.insert(to_r3tree(*begin++, counter++));
   }
};

Spatial3Index::Spatial3Index() { pimpl_ = new Pimpl(*this); }
Spatial3Index::~Spatial3Index() { delete pimpl_; }
Spatial3Index::Spatial3Index(const Spatial3Index& rhs)
    : pimpl_(nullptr)
{
   pimpl_ = new Pimpl(*this);
   *this  = rhs;
}
Spatial3Index::Spatial3Index(Spatial3Index&& rhs)
    : pimpl_(nullptr)
{
   pimpl_     = rhs.pimpl_;
   rhs.pimpl_ = nullptr;
}
Spatial3Index& Spatial3Index::operator=(const Spatial3Index& rhs)
{
   if(this == &rhs) return *this;
   *pimpl_ = *(rhs.pimpl_);
   return *this;
}
Spatial3Index& Spatial3Index::operator=(Spatial3Index&& rhs)
{
   if(this == &rhs) return *this;
   delete pimpl_;
   pimpl_     = rhs.pimpl_;
   rhs.pimpl_ = nullptr;
   return *this;
}
void Spatial3Index::clear() { pimpl_->index.clear(); }
void Spatial3Index::init(const vector<Vector3>& pts)
{
   pimpl_->init(pts.begin(), pts.end());
}
void Spatial3Index::init(const std::deque<Vector3>& pts)
{
   pimpl_->init(pts.begin(), pts.end());
}

void Spatial3Index::query_nearest(const Vector3& Q,
                                  uint n_points,
                                  std::deque<LabeledVector3>& ret) const
{
   vector<value> values;
   values.reserve(n_points);
   const Vector3f P = to_vec3f(Q);
   pimpl_->index.query(bgi::nearest(point(P.x, P.y, P.z), n_points),
                       std::back_inserter(values));
   for(const value& v : values) ret.push_back(value_to_lv3(v));
}

LabeledVector3 Spatial3Index::query_nearest(const Vector3& Q) const
{
   std::list<value> values;
   const Vector3f P = to_vec3f(Q);
   pimpl_->index.query(bgi::nearest(point(P.x, P.y, P.z), 1),
                       std::back_inserter(values));
   return value_to_lv3(values.front());
}

} // namespace perceive
