#ifndef DEPTH_UTILS_H_
#define DEPTH_UTILS_H_

#include <opencv2/core.hpp>

namespace bingjian {

// Perform in-place hole filling using median filtering.
// The data type of input cv::Mat is expected to be CV_16UC1.
// Returns the number of pixels that got value changed.
int FillHoleByMedianFiltering(cv::Mat& depth, int radius);

}  // namespace bingjian

#endif  // DEPTH_UTILS_H_
