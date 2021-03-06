#ifndef GEOMETRY_UTILS_H_
#define GEOMETRY_UTILS_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <utility>
#include <vector>

namespace bingjian {

std::vector<std::tuple<int, int, int>> LatticePointsInTriangle(
    const Eigen::MatrixX2d& triangle);

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>
CreateTextureImageSimple(const Eigen::MatrixX3i& faces,
                         const Eigen::MatrixX2d& xy_2d,
                         const Eigen::MatrixX3d& rgb, int height, int width);

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXi>
CreateTextureImage(const std::vector<double>& depth,
                   const Eigen::MatrixX3i& faces,
                   const std::vector<int>& valid_ids,
                   const Eigen::MatrixX2d& xy_2d, const Eigen::MatrixX3d& rgb,
                   int height, int width);

}  // namespace bingjian

#endif  // GEOMETRY_UTILS_H_
