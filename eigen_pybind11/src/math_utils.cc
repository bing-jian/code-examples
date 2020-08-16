#include "math_utils.h"

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace bingjian {

using namespace Eigen;

// Originally adapted from https://gist.github.com/JiaxiangZheng/8168862
// References:
//   [1] https://igl.ethz.ch/projects/ARAP/svd_rot.pdf
//   [2] http://graphics.stanford.edu/~smr/ICP/comparison/eggert_comparison_mva97.pdf
// TODO(bing.jian): Consider using Eigen::Affine3d in Eigen/Geometry
TransformType ComputeRigidTransform(const Points3DType& src,
                                    const Points3DType& dst) {
  assert(src.rows() == dst.rows());
  assert(src.rows() > 3);
  assert(3 == src.cols() == dst.cols());

  RowVector3d center_src = src.colwise().mean();
  RowVector3d center_dst = dst.colwise().mean();

  MatrixXd S(src), D(dst);
  S.rowwise() -= center_src;
  D.rowwise() -= center_dst;

  JacobiSVD<MatrixXd> svd;
  MatrixXd H = D.transpose() * S;  // 3x3
  svd.compute(H, ComputeThinU | ComputeThinV);
  if (!svd.computeU() || !svd.computeV()) {
    std::cerr << "SVD error" << std::endl;
    return std::make_pair(Matrix3d::Identity(), Vector3d::Zero());
  }
  Matrix3d Vt = svd.matrixV().transpose();
  Matrix3d R = svd.matrixU() * Vt;
  if (R.determinant() < 0) { // Orientation rectification
      Matrix3d I = Matrix3d::Identity();
      I(2, 2) = -1;
      R = svd.matrixU() * I * Vt;
  }
  Vector3d t = center_dst.transpose() - R * center_src.transpose();

  return std::make_pair(R, t);
}

}  // namespace bingjian
