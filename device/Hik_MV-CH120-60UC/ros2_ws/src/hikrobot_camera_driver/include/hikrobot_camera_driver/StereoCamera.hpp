#pragma once


#include "HikCamera.hpp"
#include "CameraConfig.hpp"


class StereoCamera
{

public:

    StereoCamera();

    bool setAutoExposure(bool enable);

    bool setAutoGain(bool enable);

    bool setPixelFormat(
        const std::string& format
    );

    bool open(const StereoCameraConfig& config);


    bool start();


    bool grab(
        cv::Mat& left,
        cv::Mat& right,
        uint64_t& left_ts,
        uint64_t& right_ts,
        int64_t& left_host_ts,
        int64_t& right_host_ts
    );


    void stop();


    void close();



private:


    HikCamera left_camera_;

    HikCamera right_camera_;


};
