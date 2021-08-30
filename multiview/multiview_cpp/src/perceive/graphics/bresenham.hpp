
#pragma once

#include <deque>
#include <functional>
#include <vector>

#include "perceive/geometry/aabb.hpp"
#include "perceive/geometry/cylinder.hpp"
#include "perceive/geometry/vector.hpp"

namespace perceive
{
inline void bresenham(double x1,
                      double y1,
                      double x2,
                      double y2,
                      std::function<void(int, int)> func);

void bresenham(Point2 p1, Point2 p2, std::function<void(int, int)> func);
void bresenham(Vector2 p1, Vector2 p2, std::function<void(int, int)> func);

// Clip the line [p1-p2] to 'bounds' _BEFORE_ drawing
void bresenham(Vector2 p1,
               Vector2 p2,
               AABB bounds,
               std::function<void(int, int)> func);

void bresenham(double x1,
               double y1,
               double x2,
               double y2,
               AABB bounds,
               std::function<void(int, int)> func);

void bresenham(Point2 p1,
               Point2 p2,
               AABBi bounds,
               std::function<void(int, int)> func);

// ----------------------------------------------------- plot-line Anti-Aliasing

void plot_line_AA(float x1,
                  float y1,
                  float x2,
                  float y2,
                  std::function<void(int, int, float)> f,
                  bool draw_ends = true,
                  int width      = 3);

void plot_line_AA(Vector2 a,
                  Vector2 b,
                  std::function<void(int, int, float)> f,
                  bool draw_ends = true,
                  int width      = 3);

void plot_line_AA(Point2 a,
                  Point2 b,
                  std::function<void(int, int, float)> f,
                  bool draw_ends = true,
                  int width      = 3);

void plot_line_AA(Vector2 a,
                  Vector2 b,
                  AABB bounds,
                  std::function<void(int, int, float)> f,
                  bool draw_ends = true,
                  int width      = 3);

void plot_line_AA(Point2 a,
                  Point2 b,
                  AABBi bounds,
                  std::function<void(int, int, float)> f,
                  bool draw_ends = true,
                  int width      = 3);

// ------------------------------------------------ Quadric Bezier Anti-Aliasing
//
void plot_quad_bezier_AA(const Vector2& A,
                         const Vector2& B,
                         const Vector2& C,
                         std::function<void(int, int, float)> f);

void plot_quad_bezier_AA2(
    const Vector2& A,
    const Vector2& B,
    const Vector2& C,
    std::function<void(int, int, float, const Vector2&)> f);

} // namespace perceive
