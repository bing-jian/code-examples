#ifndef GEOMETRY_UTILS_H_
#define GEOMETRY_UTILS_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <utility>
#include <vector>

namespace bingjian {

std::vector<std::tuple<int, int, int>> LatticePointsInTriangle(
    const Eigen::MatrixX2d& triangle);

}  // namespace bingjian

#endif  // GEOMETRY_UTILS_H_
