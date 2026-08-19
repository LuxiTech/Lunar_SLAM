#pragma once

#include "MvCameraControl.h"
#include <opencv2/imgproc.hpp>

namespace hikrobot_camera_driver
{

inline int bayerColorConversionCode(
    const MvGvspPixelType pixel_type, const bool swap_red_blue)
{
    switch (pixel_type) {
        case PixelType_Gvsp_BayerGR8:
            return swap_red_blue ? cv::COLOR_BayerGR2RGB : cv::COLOR_BayerGR2BGR;
        case PixelType_Gvsp_BayerRG8:
            return swap_red_blue ? cv::COLOR_BayerRG2RGB : cv::COLOR_BayerRG2BGR;
        case PixelType_Gvsp_BayerGB8:
            return swap_red_blue ? cv::COLOR_BayerGB2RGB : cv::COLOR_BayerGB2BGR;
        case PixelType_Gvsp_BayerBG8:
            return swap_red_blue ? cv::COLOR_BayerBG2RGB : cv::COLOR_BayerBG2BGR;
        default:
            return -1;
    }
}

inline int bayerGrayConversionCode(
    const MvGvspPixelType pixel_type, const bool swap_red_blue)
{
    if (!swap_red_blue) {
        switch (pixel_type) {
            case PixelType_Gvsp_BayerGR8: return cv::COLOR_BayerGR2GRAY;
            case PixelType_Gvsp_BayerRG8: return cv::COLOR_BayerRG2GRAY;
            case PixelType_Gvsp_BayerGB8: return cv::COLOR_BayerGB2GRAY;
            case PixelType_Gvsp_BayerBG8: return cv::COLOR_BayerBG2GRAY;
            default: return -1;
        }
    }
    // Preserve the old Bayer->BGR->RGB->GRAY channel interpretation without
    // allocating either three-channel intermediate image.
    switch (pixel_type) {
        case PixelType_Gvsp_BayerGR8: return cv::COLOR_BayerGB2GRAY;
        case PixelType_Gvsp_BayerRG8: return cv::COLOR_BayerBG2GRAY;
        case PixelType_Gvsp_BayerGB8: return cv::COLOR_BayerGR2GRAY;
        case PixelType_Gvsp_BayerBG8: return cv::COLOR_BayerRG2GRAY;
        default: return -1;
    }
}

}  // namespace hikrobot_camera_driver
