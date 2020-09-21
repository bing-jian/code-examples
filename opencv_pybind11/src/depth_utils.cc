#include "depth_utils.h"

#include <algorithm>
#include <chrono>
#include <opencv2/core.hpp>
#include <tuple>
#include <vector>

using namespace std::chrono;

namespace bingjian {

std::tuple<int, int, int> FillHoleByMedianFiltering(cv::Mat& depth, int radius,
                                                    bool skip_nonzero) {
  // TODO: Consider a more efficient median filtering implementation such as
  // Median Filtering in Constant Time by Simon Perreault and Patrick Hebert
  // https://nomis80.org/ctmf.pdf
  int zero_cnt = 0;
  int filled_cnt = 0;

  auto t0 = high_resolution_clock::now();
  for (int y = 0; y < depth.rows; ++y) {
    for (int x = 0; x < depth.cols; ++x) {
      bool is_zero = false;
      if (depth.at<ushort>(y, x) != 0) {
        if (skip_nonzero) {
          continue;
        }
      } else {
        is_zero = true;
        ++zero_cnt;
      }
      std::vector<ushort> neighbor_vals;
      int valid_neighbors = 0;
      for (int dy = -radius; dy <= radius; ++dy) {
        if (y + dy < 0 || y + dy >= depth.rows) {
          continue;
        }
        for (int dx = -radius; dx <= radius; ++dx) {
          if (x + dx < 0 || x + dx >= depth.cols) {
            continue;
          }
          ++valid_neighbors;
          int val = depth.at<ushort>(y + dy, x + dx);
          if (val != 0) {
            neighbor_vals.push_back(val);
          }
        }
      }
      if (neighbor_vals.size() * 2 > valid_neighbors) {
        std::nth_element(neighbor_vals.begin(),
                         neighbor_vals.begin() + neighbor_vals.size() / 2,
                         neighbor_vals.end());
        depth.at<ushort>(y, x) = neighbor_vals[neighbor_vals.size() / 2];
        if (is_zero) {
          ++filled_cnt;
        }
      }
    }
  }
  auto t1 = high_resolution_clock::now();
  auto duration = duration_cast<milliseconds>(t1 - t0);
  return std::make_tuple(zero_cnt, filled_cnt, duration.count());
}

}  // namespace bingjian
