#pragma once

#include <opencv2/core.hpp>

#include <cmath>

namespace stereo_depth
{

inline void applyVerticalPrincipalPointOffset(cv::Mat & projection, double offset_pixels)
{
  CV_Assert(projection.rows == 3 && projection.cols == 3 && projection.type() == CV_64F);
  projection.at<double>(1, 2) += offset_pixels;
}

inline void applyRectificationRoll(cv::Mat & rectification, double roll_degrees)
{
  CV_Assert(rectification.rows == 3 && rectification.cols == 3 &&
    rectification.type() == CV_64F);
  const double radians = roll_degrees * CV_PI / 180.0;
  const double cosine = std::cos(radians);
  const double sine = std::sin(radians);
  const cv::Mat roll = (cv::Mat_<double>(3, 3) <<
    cosine, -sine, 0.0,
    sine, cosine, 0.0,
    0.0, 0.0, 1.0);
  rectification = roll * rectification;
}

}  // namespace stereo_depth
