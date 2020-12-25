#ifndef DISTANCE_TRANSFORM_UTILS_H_
#define DISTANCE_TRANSFORM_UTILS_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>
#include <utility>
#include <vector>

namespace bingjian {

float ChamferDistance(const Eigen::Tensor<float, 3>& tensor, float d1, float d2,
                      float d3, int z, int y, int x, float infty, bool forward);

Eigen::Tensor<float, 3> ComputeDistanceField(const Eigen::MatrixX3d& pts,
                                             float x_min, float y_min,
                                             float z_min, float x_max,
                                             float y_max, float z_max,
                                             float voxel_size, float max_dist);

}  // namespace bingjian

#endif  // DISTANCE_TRANSFORM_UTILS_H_
