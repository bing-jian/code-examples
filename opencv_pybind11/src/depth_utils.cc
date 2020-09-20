#include "depth_utils.h"

#include <algorithm>
#include <opencv2/core.hpp>
#include <vector>

namespace bingjian {

int FillHoleByMedianFiltering(cv::Mat& depth, int radius) {
  int filled_cnt = 0;
  for (int y = 0; y < depth.rows; ++y) {
    for (int x = 0; x < depth.cols; ++x) {
      if (depth.at<ushort>(y, x) != 0) {
        continue;
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
        ++filled_cnt;
      }
    }
  }
  return filled_cnt;
}

}  // namespace bingjian
