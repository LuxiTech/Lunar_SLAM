#include "hikrobot_camera_driver/bayer_conversion.hpp"

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <array>

TEST(BayerConversion, FusedColorSwapIsByteIdentical)
{
  cv::Mat raw(96, 128, CV_8UC1);
  cv::randu(raw, 0, 256);
  const std::array<MvGvspPixelType, 4> formats{
    PixelType_Gvsp_BayerGR8,
    PixelType_Gvsp_BayerRG8,
    PixelType_Gvsp_BayerGB8,
    PixelType_Gvsp_BayerBG8};
  for (const auto format : formats) {
    cv::Mat old_color;
    cv::Mat old_swapped;
    cv::Mat fused;
    cv::cvtColor(
      raw, old_color,
      hikrobot_camera_driver::bayerColorConversionCode(format, false));
    cv::cvtColor(old_color, old_swapped, cv::COLOR_BGR2RGB);
    cv::cvtColor(
      raw, fused,
      hikrobot_camera_driver::bayerColorConversionCode(format, true));
    EXPECT_EQ(cv::norm(old_swapped, fused, cv::NORM_INF), 0.0);
  }
}

TEST(BayerConversion, DirectGrayPreservesOldDepthInputWithinOneLevel)
{
  cv::Mat raw(96, 128, CV_8UC1);
  cv::randu(raw, 0, 256);
  const std::array<MvGvspPixelType, 4> formats{
    PixelType_Gvsp_BayerGR8,
    PixelType_Gvsp_BayerRG8,
    PixelType_Gvsp_BayerGB8,
    PixelType_Gvsp_BayerBG8};
  for (const auto format : formats) {
    cv::Mat old_color;
    cv::Mat old_swapped;
    cv::Mat old_gray;
    cv::Mat direct_gray;
    cv::cvtColor(
      raw, old_color,
      hikrobot_camera_driver::bayerColorConversionCode(format, false));
    cv::cvtColor(old_color, old_swapped, cv::COLOR_BGR2RGB);
    cv::cvtColor(old_swapped, old_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(
      raw, direct_gray,
      hikrobot_camera_driver::bayerGrayConversionCode(format, true));
    EXPECT_LE(cv::norm(old_gray, direct_gray, cv::NORM_INF), 1.0);
  }
}
