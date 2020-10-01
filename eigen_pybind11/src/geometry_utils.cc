#include "geometry_utils.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <unordered_map>
#include <utility>
#include <vector>

//#define __LOGGING_TIME__

#ifdef __LOGGING_TIME__
#include <chrono>
#include <iostream>
using namespace std::chrono;
#endif

using namespace Eigen;

namespace bingjian {

std::vector<std::tuple<int, int, int>> LatticePointsInTriangle(
    const Vector2d& p1, const Vector2d& p2, const Vector2d& p3);

std::vector<std::tuple<int, int, int>> LatticePointsInTriangle(
    const MatrixX2d& triangle) {
  assert(triangle.rows() == 3);
  return LatticePointsInTriangle(triangle.row(0), triangle.row(1),
                                 triangle.row(2));
}

std::vector<std::tuple<int, int, int>> LatticePointsInTriangle(
    const Vector2d& p1, const Vector2d& p2, const Vector2d& p3) {
  // Sort by x coordinates
  std::vector<std::pair<double, double>> xys;
  xys.emplace_back(p1(0), p1(1));
  xys.emplace_back(p2(0), p2(1));
  xys.emplace_back(p3(0), p3(1));
  std::sort(xys.begin(), xys.end());
  double x1 = xys[0].first;
  double y1 = xys[0].second;
  double x2 = xys[1].first;
  double y2 = xys[1].second;
  double x3 = xys[2].first;
  double y3 = xys[2].second;
  // assume x1 <= x2 <= x3
  std::vector<std::tuple<int, int, int>> res;
  if ((x1 == x2) && (x2 == x3)) {
    return res;
  }
  res.reserve(int(x3 - x1 + 1));
  // Process segment from x1 to x2
  int x1i = std::ceil(x1);
  int x2i = std::ceil(x2);
  int x3i = std::ceil(x3);
  double coeff_21 = (y2 - y1) / (x2 - x1);
  double coeff_31 = (y3 - y1) / (x3 - x1);
  double coeff_32 = (y3 - y2) / (x3 - x2);

  int y_min, y_max = 0;
  if (x2 == x1) {
    if (y1 < y2) {
      y_min = std::ceil(y1);
      y_max = std::ceil(y2);
    } else {
      y_min = std::ceil(y2);
      y_max = std::ceil(y1);
    }
    res.emplace_back(x1i, y_min, y_max);
  } else {
    for (int x = x1i; x < x2i; ++x) {
      // find intersection of line x=x with lines AB
      double y_AB = (x - x1) * coeff_21 + y1;
      // find intersection of line x=x with lines AC
      double y_AC = (x - x1) * coeff_31 + y1;
      if (y_AB < y_AC) {
        y_min = std::ceil(y_AB);
        y_max = std::ceil(y_AC);
      } else {
        y_min = std::ceil(y_AC);
        y_max = std::ceil(y_AB);
      }
      res.emplace_back(x, y_min, y_max);
    }
  }
  // Process segment from x2 to x3
  if (x3 == x2) {
    if (y3 < y2) {
      y_min = std::ceil(y3);
      y_max = std::ceil(y2);
    } else {
      y_min = std::ceil(y2);
      y_max = std::ceil(y3);
    }
    res.emplace_back(x2i, y_min, y_max);
  } else {
    for (int x = x2i; x < x3i; ++x) {
      // find intersection of line x=x with lines BC
      double y_BC = (x - x2) * coeff_32 + y2;
      // find intersection of line x=x with lines AC
      double y_AC = (x - x1) * coeff_31 + y1;
      if (y_BC < y_AC) {
        y_min = std::ceil(y_BC);
        y_max = std::ceil(y_AC);
      } else {
        y_min = std::ceil(y_AC);
        y_max = std::ceil(y_BC);
      }
      res.emplace_back(x, y_min, y_max);
    }
  }
  return res;
}

std::tuple<MatrixXd, MatrixXd, MatrixXd> CreateTextureImage(
    const MatrixX3i& faces, const MatrixX2d& xy_2d, const MatrixX3d& rgb,
    int height, int width) {
  MatrixXd dst_r(height, width);
  MatrixXd dst_g(height, width);
  MatrixXd dst_b(height, width);
  dst_r.setZero();
  dst_g.setZero();
  dst_b.setZero();

  int num_of_faces = faces.rows();

  for (int face_idx = 0; face_idx < num_of_faces; ++face_idx) {
    int v1_idx = faces(face_idx, 0);
    int v2_idx = faces(face_idx, 1);
    int v3_idx = faces(face_idx, 2);
    Vector2d p1 = xy_2d.row(v1_idx);
    Vector2d p2 = xy_2d.row(v2_idx);
    Vector2d p3 = xy_2d.row(v3_idx);
    Vector3d rgb1 = rgb.row(v1_idx);
    Vector3d rgb2 = rgb.row(v2_idx);
    Vector3d rgb3 = rgb.row(v3_idx);
    double x1 = p1(0);
    double y1 = p1(1);
    double x2 = p2(0);
    double y2 = p2(1);
    double x3 = p3(0);
    double y3 = p3(1);
    double det = ((y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3));

    auto res = LatticePointsInTriangle(p1, p2, p3);
    for (const auto& pt : res) {
      // Assume faces do not overlap in 2d. Otherwise we need to sort
      // faces based on visibility. Closest faces should come last.
      int x = std::get<0>(pt);
      if (x < 0 || x >= width) {
        continue;
      }
      int y_min = std::get<1>(pt);
      int y_max = std::get<2>(pt);
      for (int y = y_min; y < y_max; ++y) {
        if (y < 0 || y >= height) {
          continue;
        }
        double c1 = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / det;
        double c2 = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / det;
        double c3 = 1 - c1 - c2;
        Vector3d rgb_at_yx = c1 * rgb1 + c2 * rgb2 + c3 * rgb3;
        dst_r(y, x) = rgb_at_yx(0);
        dst_g(y, x) = rgb_at_yx(1);
        dst_b(y, x) = rgb_at_yx(2);
      }
    }
  }
  return std::make_tuple(std::move(dst_r), std::move(dst_g), std::move(dst_b));
}

}  // namespace bingjian
