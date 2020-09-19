#ifndef MAT_WRAPPER_H_
#define MAT_WRAPPER_H_

// https://blog.csdn.net/non_hercules/article/details/105095153
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <opencv2/opencv.hpp>

namespace py = pybind11;

cv::Mat numpy_uint8_1c_to_cv_mat(const py::array_t<unsigned char>& input);

cv::Mat numpy_uint8_3c_to_cv_mat(const py::array_t<unsigned char>& input);

py::array_t<unsigned char> cv_mat_uint8_1c_to_numpy(const cv::Mat& input);

py::array_t<unsigned char> cv_mat_uint8_3c_to_numpy(const cv::Mat& input);

#endif  // MAT_WRAPPER_H_
