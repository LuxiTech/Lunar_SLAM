#include "hikrobot_camera_driver/StereoCamera.hpp"

#include <cmath>
#include <future>
#include <iostream>

namespace
{
int64_t hostTimestampToNs(int64_t timestamp)
{
    if (timestamp <= 0) {
        return 0;
    }
    if (timestamp >= 1000000000000000000LL) {
        return timestamp;
    }
    if (timestamp >= 1000000000000000LL) {
        return timestamp * 1000LL;
    }
    if (timestamp >= 1000000000000LL) {
        return timestamp * 1000000LL;
    }
    return 0;
}

double hostTimestampDeltaMs(int64_t left, int64_t right)
{
    const int64_t left_ns = hostTimestampToNs(left);
    const int64_t right_ns = hostTimestampToNs(right);
    if (left_ns == 0 || right_ns == 0) {
        return -1.0;
    }
    return std::abs(static_cast<double>(left_ns - right_ns)) / 1.0e6;
}
}  // namespace


StereoCamera::StereoCamera()
{

}


bool StereoCamera::open(const StereoCameraConfig& config)
{
    MV_CC_DEVICE_INFO_LIST list{};


    int ret =
    MV_CC_EnumDevices(
        MV_USB_DEVICE,
        &list
    );


    if(ret!=MV_OK)
    {
        std::cerr << "EnumDevices failed: 0x" << std::hex
                  << static_cast<unsigned int>(ret) << std::dec << std::endl;
        return false;
    }



    for(int i=0;i<list.nDeviceNum;i++)
    {

        auto device =
        list.pDeviceInfo[i];


        std::string serial;


        if(device->nTLayerType==MV_USB_DEVICE)
        {

            serial =
            (char*)device
            ->SpecialInfo
            .stUsb3VInfo
            .chSerialNumber;

        }



        if(serial == config.left_serial)
        {

            if (!left_camera_.open(device)) {
                close();
                return false;
            }

            left_camera_.loadUserSet(config.user_set);

            if (config.trigger_source != "Default" && config.trigger_source != "default") {
                if (!left_camera_.setTriggerMode(config.external_trigger) ||
                    (config.external_trigger &&
                     !left_camera_.setTriggerSource(config.trigger_source))) {
                    close();
                    return false;
                }
            }

            if (!left_camera_.setAcquisitionFrameRate(config.frame_rate_hz)) {
                close();
                return false;
            }

            if (!left_camera_.setImageSampling(
                config.binning_horizontal,
                config.binning_vertical,
                config.decimation_horizontal,
                config.decimation_vertical)) {
                close();
                return false;
            }

            if (!left_camera_.setImageRoi(
                config.image_width,
                config.image_height,
                config.offset_x,
                config.offset_y)) {
                close();
                return false;
            }


            if (config.exposure_time_us >= 0.0F) {
                if (!left_camera_.setAutoExposure(config.auto_exposure) ||
                    (!config.auto_exposure && !left_camera_.setExposureTime(config.exposure_time_us))) {
                    close();
                    return false;
                }
            }


            if ((config.gain >= 0.0F &&
                 (!left_camera_.setAutoGain(config.auto_gain) ||
                  (!config.auto_gain && !left_camera_.setGain(config.gain)))) ||
                (config.override_white_balance &&
                 !left_camera_.setAutoWhiteBalance(config.auto_white_balance)) ||
                !left_camera_.setPixelFormat(config.pixel_format)) {
                close();
                return false;
            }
            left_camera_.setSwapRedBlue(config.swap_red_blue);
            left_camera_.printCurrentSettings();

            //固件开启自动曝光和自动增益，手动设置无效
            // left_camera_.setExposureTime(40000);
            // left_camera_.setGain(0);


        }



        if(serial == config.right_serial)
        {

            if (!right_camera_.open(device)) {
                close();
                return false;
            }

            right_camera_.loadUserSet(config.user_set);

            if (config.trigger_source != "Default" && config.trigger_source != "default") {
                if (!right_camera_.setTriggerMode(config.external_trigger) ||
                    (config.external_trigger &&
                     !right_camera_.setTriggerSource(config.trigger_source))) {
                    close();
                    return false;
                }
            }

            if (!right_camera_.setAcquisitionFrameRate(config.frame_rate_hz)) {
                close();
                return false;
            }

            if (!right_camera_.setImageSampling(
                config.binning_horizontal,
                config.binning_vertical,
                config.decimation_horizontal,
                config.decimation_vertical)) {
                close();
                return false;
            }

            if (!right_camera_.setImageRoi(
                config.image_width,
                config.image_height,
                config.offset_x,
                config.offset_y)) {
                close();
                return false;
            }


            if (config.exposure_time_us >= 0.0F) {
                if (!right_camera_.setAutoExposure(config.auto_exposure) ||
                    (!config.auto_exposure && !right_camera_.setExposureTime(config.exposure_time_us))) {
                    close();
                    return false;
                }
            }


            if ((config.gain >= 0.0F &&
                 (!right_camera_.setAutoGain(config.auto_gain) ||
                  (!config.auto_gain && !right_camera_.setGain(config.gain)))) ||
                (config.override_white_balance &&
                 !right_camera_.setAutoWhiteBalance(config.auto_white_balance)) ||
                !right_camera_.setPixelFormat(config.pixel_format)) {
                close();
                return false;
            }
            right_camera_.setSwapRedBlue(config.swap_red_blue);
            right_camera_.printCurrentSettings();


            //固件开启自动曝光和自动增益，手动设置无效
            // left_camera_.setExposureTime(40000);
            // left_camera_.setGain(0);


        }


    }


    if (!left_camera_.isOpened() || !right_camera_.isOpened()) {
        std::cerr << "Stereo cameras not found. Expected serials: "
                  << config.left_serial << ", " << config.right_serial
                  << std::endl;
        close();
        return false;
    }
    return true;

}


bool StereoCamera::start()
{
    // This passively verifies that the external trigger pulses actually reach
    // each camera input.  It is especially useful when UserSet owns the
    // trigger configuration and no SDK trigger setting is overridden.  Run it
    // before acquisition: sampling while the camera buffers are filling can
    // otherwise overflow the small SDK image queue at high trigger rates.
    left_camera_.printLine0Diagnostics();
    right_camera_.printLine0Diagnostics();

    if (!left_camera_.start() || !right_camera_.start()) {
        stop();
        return false;
    }
    return true;

}


bool StereoCamera::grab(
    cv::Mat& left,
    cv::Mat& right,
    uint64_t& left_ts,
    uint64_t& right_ts,
    int64_t& left_host_ts,
    int64_t& right_host_ts
)
{


    // Both cameras receive the same hardware trigger. Retrieve and convert the
    // two frames concurrently so one Bayer conversion cannot delay the other.
    auto left_result = std::async(std::launch::async, [&]() {
        return left_camera_.grab(left, left_ts, left_host_ts);
    });

    const bool r = right_camera_.grab(right, right_ts, right_host_ts);
    const bool l = left_result.get();

    static unsigned int asymmetric_failures = 0;
    if (l != r) {
        ++asymmetric_failures;
        if (asymmetric_failures <= 5 || asymmetric_failures % 30 == 0) {
            std::cerr << "Stereo grab mismatch: left=" << (l ? "ok" : "timeout")
                      << ", right=" << (r ? "ok" : "timeout") << std::endl;
        }
    } else if (l && r) {
        asymmetric_failures = 0;
    }


    if (!l || !r) {
        return false;
    }

    // Each SDK handle owns an independent queue. A delayed callback can
    // retrieve frame N from one queue and frame N+1 from the other even though
    // both cameras share Line0. Replace the older candidate until both host
    // capture timestamps belong to the same trigger period.
    constexpr double kMaxPairDeltaMs = 8.0;
    constexpr int kMaxRealignAttempts = 6;
    for (int attempt = 0; attempt < kMaxRealignAttempts; ++attempt) {
        const double delta_ms = hostTimestampDeltaMs(left_host_ts, right_host_ts);
        if (delta_ms < 0.0 || delta_ms <= kMaxPairDeltaMs) {
            return true;
        }

        const bool left_is_older = left_host_ts < right_host_ts;
        const bool replaced = left_is_older
            ? left_camera_.grab(left, left_ts, left_host_ts)
            : right_camera_.grab(right, right_ts, right_host_ts);
        if (!replaced) {
            std::cerr << "Stereo realignment failed while replacing "
                      << (left_is_older ? "left" : "right") << " frame" << std::endl;
            return false;
        }
    }

    static unsigned int pair_mismatch_count = 0;
    ++pair_mismatch_count;
    if (pair_mismatch_count <= 5 || pair_mismatch_count % 30 == 0) {
        std::cerr << "Stereo frame pairing failed: host delta="
                  << hostTimestampDeltaMs(left_host_ts, right_host_ts)
                  << " ms after " << kMaxRealignAttempts << " retries" << std::endl;
    }
    return false;

}


void StereoCamera::stop()
{

    left_camera_.stop();

    right_camera_.stop();

}


void StereoCamera::close()
{

    left_camera_.close();

    right_camera_.close();

}
