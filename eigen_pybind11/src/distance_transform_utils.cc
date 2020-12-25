#include "distance_transform_utils.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <unsupported/Eigen/CXX11/Tensor>
#include <vector>

namespace bingjian {

template <typename T>
T SQR(T x) {
  return x * x;
}

// https://www.microsoft.com/en-us/research/publication/robust-registration-of-2d-and-3d-point-sets/
// https://github.com/loui0620/DMT-1_tracking/blob/master/GoICP/jly_3ddt.cpp
// https://github.com/PrincetonVision/SUN3Dsfm/blob/master/lib/awf/icp_3ddt.cxx
float ChamferDistance(const Eigen::Tensor<float, 3>& tensor, float d1, float d2,
                      float d3, int z, int y, int x, float infty,
                      bool forward) {
  float min_val = infty;
  int xDim = tensor.dimension(0);
  int yDim = tensor.dimension(1);
  int zDim = tensor.dimension(2);

  float mask[14] = {infty};
  mask[0] = tensor(x, y, z);

  for (int i = 1; i < 14; ++i) {
    mask[i] = infty;
  }

  // MASK:
  //  (+d3) (+d2) (+d3) | (+d2) (+d1) (+d2)
  //  (+d2) (+d1) (+d2) | (+d1) (0)
  //  (+d3) (+d2) (+d3) |
  int forward_dx[13] = {0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1};
  int forward_dy[13] = {0, -1, -1, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1};
  int forward_dz[13] = {-1, -1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1};

  // MASK:
  //                    |  (+d3) (+d2) (+d3)
  //          (0) (+d1) |  (+d2) (+d1) (+d2)
  //  (+d2) (+d1) (+d2) |  (+d3) (+d2) (+d3)
  int backward_dx[13] = {0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  int backward_dy[13] = {0, 1, 1, 1, -1, -1, -1, 0, 0, 0, 1, 1, 1};
  int backward_dz[13] = {1, 1, 0, -1, -1, 0, 1, -1, 0, 1, -1, 0, 1};

  float dd[3] = {d1, d2, d3};

  int *dx, *dy, *dz;
  if (forward) {
    dx = &forward_dx[0];
    dy = &forward_dy[0];
    dz = &forward_dz[0];
  } else {
    dx = &backward_dx[0];
    dy = &backward_dy[0];
    dz = &backward_dz[0];
  }

  for (int i = 1; i < 14; ++i) {
    if (x + dx[i - 1] < xDim && x + dx[i - 1] >= 0 && y + dy[i - 1] < yDim &&
        y + dy[i - 1] >= 0 && z + dz[i - 1] < zDim && z + dz[i - 1] >= 0) {
      int d = dd[std::abs(dx[i - 1]) + std::abs(dy[i - 1]) +
                 std::abs(dz[i - 1]) - 1];
      mask[i] = tensor(x + dx[i - 1], y + dy[i - 1], z + dz[i - 1]) + d;
    }
  }
  // find minimum
  for (int i = 0; i < 14; ++i) {
    if (mask[i] < min_val) {
      min_val = mask[i];
    }
  }
  return min_val;
}

Eigen::Tensor<float, 3> ComputeDistanceField(const Eigen::MatrixX3d& pts,
                                             float x_min, float y_min,
                                             float z_min, float x_max,
                                             float y_max, float z_max,
                                             float voxel_size, float max_dist) {
  int dim_z = (int)std::ceil((z_max - z_min) / voxel_size);
  int dim_y = (int)std::ceil((y_max - y_min) / voxel_size);
  int dim_x = (int)std::ceil((x_max - x_min) / voxel_size);

  std::cout << "Initializing output 3D tensor ...\n";
  std::cout << "dim_z: " << dim_z << ", dim_y: " << dim_y
            << ", dim x: " << dim_x << std::endl;
  Eigen::Tensor<float, 3> out_tensor(dim_x, dim_y,
                                     dim_z);  // Eigen::tensor is column major

  out_tensor.setConstant(max_dist);
  for (int i = 0; i < pts.rows(); ++i) {
    float x = (pts(i, 0) - x_min) / voxel_size;
    float y = (pts(i, 1) - y_min) / voxel_size;
    float z = (pts(i, 2) - z_min) / voxel_size;
    int x0 = (int)std::floor(x);
    int y0 = (int)std::floor(y);
    int z0 = (int)std::floor(z);

    for (int dz = 0; dz <= 1; dz++) {
      int zg = z0 + dz;
      for (int dy = 0; dy <= 1; dy++) {
        int yg = y0 + dy;
        for (int dx = 0; dx <= 1; dx++) {
          int xg = x0 + dx;
          out_tensor(xg, yg, zg) =
              std::min(out_tensor(xg, yg, zg),
                       std::sqrt(SQR(x - xg) + SQR(y - yg) + SQR(z - zg)));
        }
      }
    }
  }
  std::cout << "Initialization from point cloud completed.\n";

  for (int iter = 0; iter < 1; ++iter) {
    int z, y, x;
    std::cout << "Computing forward Chamfer distance ...\n";
    for (x = 0; x < dim_x; ++x) {
      for (y = 0; y < dim_y; ++y) {
        for (z = 0; z < dim_z; ++z) {
          out_tensor(x, y, z) = ChamferDistance(out_tensor, 1.0, 1.414, 1.732,
                                                z, y, x, max_dist, true);
        }
      }
    }
    std::cout << "Computing backward Chamfer distance  ...\n";
    for (x = dim_x - 1; x >= 0; --x) {
      for (y = dim_y - 1; y >= 0; --y) {
        for (z = dim_z - 1; z >= 0; --z) {
          out_tensor(x, y, z) = ChamferDistance(out_tensor, 1.0, 1.414, 1.732,
                                                z, y, x, max_dist, false);
        }
      }
    }
  }
  std::cout << "Chamfer distance field computed.\n";
  return out_tensor;
}

}  // namespace bingjian
