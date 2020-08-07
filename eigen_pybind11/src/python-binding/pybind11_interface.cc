#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <Eigen/LU>

#include "../funcs.h"
#include "../math_utils.h"

using namespace pybind11::literals;

namespace py = pybind11;

double square(double x) { return x * x; }

Eigen::MatrixXd inv(Eigen::MatrixXd xs) { return xs.inverse(); }

double det(Eigen::MatrixXd xs) { return xs.determinant(); }

PYBIND11_MODULE(wrapper, m) {
  m.doc() = "documentation string";  // optional
  m.def("add", &add, "A function which adds two numbers", "i"_a = 1, "j"_a = 2);
  m.def("square", py::vectorize(square));
  m.def("inv", &inv);
  m.def("det", &det);
  m.def("rigid_transform", &bingjian::ComputeRigidTransform,
        py::return_value_policy::move);
}
