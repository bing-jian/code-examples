#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <Eigen/LU>

#include "../distance_transform_utils.h"
#include "../funcs.h"
#include "../geometry_utils.h"
#include "../math_utils.h"


using namespace pybind11::literals;

namespace py = pybind11;

double square(double x) { return x * x; }

Eigen::MatrixXd inv(Eigen::MatrixXd xs) { return xs.inverse(); }

double det(Eigen::MatrixXd xs) { return xs.determinant(); }

// https://stackoverflow.com/questions/58412795/can-i-use-pybind11-to-pass-a-numpy-array-to-a-function-accepting-a-eigentensor
pybind11::array_t<float> return_array(Eigen::Tensor<float, 3> inp) {
  // Note that numpy array is row major whle Eigen::tensor is column major.
  std::vector<ssize_t> shape(3);
  shape[0] = inp.dimension(2);
  shape[1] = inp.dimension(1);
  shape[2] = inp.dimension(0);
  return pybind11::array_t<float>(
      shape,  // shape
      {shape[2] * shape[1] * sizeof(float), shape[2] * sizeof(float),
       sizeof(float)},  // strides
      inp.data());      // data pointer
}

pybind11::array_t<float> ComputeDistanceFieldWrapper(
    const Eigen::MatrixX3d& pts, float x_min, float y_min, float z_min,
    float x_max, float y_max, float z_max, float voxel_size, float max_dist) {
  return return_array(bingjian::ComputeDistanceField(
      pts, x_min, y_min, z_min, x_max, y_max, z_max, voxel_size, max_dist));
}

PYBIND11_MODULE(wrapper, m) {
  m.doc() = "documentation string";  // optional
  m.def("add", &add, "A function which adds two numbers", "i"_a = 1, "j"_a = 2);
  m.def("square", py::vectorize(square));
  m.def("inv", &inv);
  m.def("det", &det);
  m.def("rigid_transform", &bingjian::ComputeRigidTransform,
        py::return_value_policy::move);
  m.def("lattices_in_triangle", &bingjian::LatticePointsInTriangle,
        py::return_value_policy::move);
  m.def("create_texture_simple", &bingjian::CreateTextureImageSimple,
        py::return_value_policy::move);
  m.def("create_texture", &bingjian::CreateTextureImage,
        py::return_value_policy::move);
  m.def("distance_transform_3d", &ComputeDistanceFieldWrapper,
        py::return_value_policy::move);
}
