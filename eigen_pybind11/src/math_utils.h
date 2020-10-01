#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

#include <Eigen/Core>
#include <vector>

namespace bingjian {

typedef std::pair<Eigen::Matrix3d, Eigen::Vector3d> TransformType;
typedef Eigen::MatrixX3d Points3DType;

// Compute the best-fitting rigid transformation that aligns
// two sets of corresponding 3D points.
TransformType ComputeRigidTransform(const Points3DType& src,
                                    const Points3DType& dst);

}  // namespace bingjian

#endif  // MATH_UTILS_H_
