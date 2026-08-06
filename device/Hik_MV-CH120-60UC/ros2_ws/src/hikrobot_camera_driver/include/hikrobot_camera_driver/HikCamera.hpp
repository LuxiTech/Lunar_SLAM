#pragma once

#include "MvCameraControl.h"
#include <opencv2/opencv.hpp>
#include <vector>


class HikCamera
{

public:

    HikCamera();

    ~HikCamera();


    bool open(
        MV_CC_DEVICE_INFO* device
    );

    bool isOpened() const;
    
    std::string getSerial();



    bool start();

    bool loadUserSet(const std::string& user_set);

    void printCurrentSettings() const;

    // Read the physical Line0 input for a short interval.  This is intended
    // for diagnosing hardware-trigger setups without changing trigger nodes.
    void printLine0Diagnostics() const;

    bool setAcquisitionModeContinuous();

    bool setAcquisitionFrameRate(float frame_rate_hz);

    bool setImageRoi(int width, int height, int offset_x, int offset_y);

    bool setImageSampling(
        int binning_horizontal,
        int binning_vertical,
        int decimation_horizontal,
        int decimation_vertical);

    // =====================
    // Exposure
    // =====================
    bool setAutoExposure(bool enable);

    bool setExposureTime(float exposure);
    
    // =====================
    // Gain
    // =====================
    bool setGain(float gain);

    bool setAutoGain(bool enable);

    bool setAutoWhiteBalance(bool enable);

    // =====================
    // Pixel Format
    // =====================
    bool setPixelFormat(const std::string& format);

    void setSwapRedBlue(bool enable);

    std::string getPixelFormat();
    

    bool grab(
        cv::Mat& image,
        uint64_t& device_timestamp,
        int64_t& host_timestamp,
        uint32_t& frame_number
    );

    // =====================
    // TriggerMode
    // =====================
    bool getTriggerMode();

    bool setTriggerMode(bool enable);

    bool getTriggerSourceName();

    bool setTriggerSourceLine0();

    bool setTriggerSource(const std::string& source);
    

    void stop();


    void close();


private:

    void* handle_;

    int width_;

    int height_;

    bool opened_;

    bool grabbing_;

    bool swap_red_blue_{false};

    int grab_fail_count_{0};

    std::vector<unsigned char> frame_buffer_;

    std::string serial_;

    std::string model_;

};
