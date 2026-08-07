#include "stereo_depth/rectification_utils.hpp"

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

TEST(RectificationUtils, PrincipalPointOffsetReplacesSecondImageWarp)
{
  const cv::Size size(64, 48);
  const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
    50.0, 0.0, 32.0,
    0.0, 50.0, 24.0,
    0.0, 0.0, 1.0);
  const cv::Mat distortion = cv::Mat::zeros(1, 5, CV_64F);
  const cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat source(size, CV_32F);
  for (int y = 0; y < source.rows; ++y) {
    for (int x = 0; x < source.cols; ++x) {
      source.at<float>(y, x) = static_cast<float>(y * source.cols + x);
    }
  }

  cv::Mat old_map_x;
  cv::Mat old_map_y;
  cv::initUndistortRectifyMap(
    camera_matrix, distortion, rotation, camera_matrix, size, CV_16SC2,
    old_map_x, old_map_y);
  cv::Mat old_rectified;
  cv::remap(source, old_rectified, old_map_x, old_map_y, cv::INTER_LINEAR);
  const cv::Mat translation = (cv::Mat_<double>(2, 3) <<
    1.0, 0.0, 0.0,
    0.0, 1.0, 2.0);
  cv::warpAffine(
    old_rectified, old_rectified, translation, size, cv::INTER_LINEAR,
    cv::BORDER_CONSTANT);

  cv::Mat shifted_projection = camera_matrix.clone();
  stereo_depth::applyVerticalPrincipalPointOffset(shifted_projection, 2.0);
  cv::Mat new_map_x;
  cv::Mat new_map_y;
  cv::initUndistortRectifyMap(
    camera_matrix, distortion, rotation, shifted_projection, size, CV_16SC2,
    new_map_x, new_map_y);
  cv::Mat new_rectified;
  cv::remap(source, new_rectified, new_map_x, new_map_y, cv::INTER_LINEAR);

  EXPECT_EQ(cv::norm(old_rectified, new_rectified, cv::NORM_INF), 0.0);
}

TEST(RectificationUtils, RollCorrectionRotatesTheRectifiedCameraFrame)
{
  cv::Mat rectification = cv::Mat::eye(3, 3, CV_64F);

  stereo_depth::applyRectificationRoll(rectification, 90.0);

  const cv::Mat expected = (cv::Mat_<double>(3, 3) <<
    0.0, -1.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 0.0, 1.0);
  EXPECT_LT(cv::norm(rectification, expected, cv::NORM_INF), 1.0e-12);
}
