#ifndef DEPTH_UTILS_H_
#define DEPTH_UTILS_H_

#include <opencv2/core.hpp>
#include <tuple>

namespace bingjian {

// This function performs in-place hole filling using median filtering.
// This function expects the following inputs:
//  1) an image represented as cv::Mat<CV_16UC1>;
//  2) a positive integer indicting the kernel radius of median filter;
//  3) a boolean flag indicting whether to skip nonzero pixels.
// Besides updating the input image in-place, this function also returns
// a tuple of three elements where
//  1) the first element is the number of zeros in the input images;
//  2) the second element is the number of zeros that got filled;
//  3) and the last element is the run time in milliseconds.
std::tuple<int, int, int> FillHoleByMedianFiltering(cv::Mat& depth, int radius,
                                                    bool skip_nonzero);

}  // namespace bingjian

#endif  // DEPTH_UTILS_H_
