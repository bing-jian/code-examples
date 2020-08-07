#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

#include <Eigen/Core>
#include <vector>

namespace bingjian {

typedef std::pair<Eigen::Matrix3d, Eigen::Vector3d> TransformType;
typedef std::vector<Eigen::Vector3d> PointsType;

TransformType ComputeRigidTransform(const PointsType& src,
                                    const PointsType& dst);

}  // namespace bingjian

#endif  // MATH_UTILS_H_
