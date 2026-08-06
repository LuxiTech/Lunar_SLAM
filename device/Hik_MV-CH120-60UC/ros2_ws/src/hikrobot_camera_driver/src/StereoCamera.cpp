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

            if (!config.external_trigger) {
                if (!left_camera_.setTriggerMode(false)) {
                    close();
                    return false;
                }
            } else if (config.trigger_source != "Default" && config.trigger_source != "default") {
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

            if (!config.external_trigger) {
                if (!right_camera_.setTriggerMode(false)) {
                    close();
                    return false;
                }
            } else if (config.trigger_source != "Default" && config.trigger_source != "default") {
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

    frame_number_offset_.reset();
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
    int64_t& right_host_ts,
    uint32_t& left_frame_number,
    uint32_t& right_frame_number
)
{


    // Both cameras receive the same hardware trigger. Retrieve and convert the
    // two frames concurrently so one Bayer conversion cannot delay the other.
    auto left_result = std::async(std::launch::async, [&]() {
        return left_camera_.grab(left, left_ts, left_host_ts, left_frame_number);
    });

    bool r = right_camera_.grab(right, right_ts, right_host_ts, right_frame_number);
    bool l = left_result.get();

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

    if (!frame_number_offset_) {
        frame_number_offset_ =
            static_cast<int64_t>(right_frame_number) - static_cast<int64_t>(left_frame_number);
        std::cout << "Stereo frame-number offset initialized: right-left="
                  << *frame_number_offset_ << std::endl;
    }

    // A one-sided timeout advances only one SDK queue. Use each camera's MVS
    // acquisition counter to catch up the lagging queue instead of silently
    // pairing adjacent trigger events.
    for (int attempt = 0; attempt < 2; ++attempt) {
        const int64_t observed_offset =
            static_cast<int64_t>(right_frame_number) - static_cast<int64_t>(left_frame_number);
        const int64_t offset_error = observed_offset - *frame_number_offset_;
        if (offset_error == 0) {
            break;
        }
        if (offset_error < 0) {
            r = right_camera_.grab(right, right_ts, right_host_ts, right_frame_number);
        } else {
            l = left_camera_.grab(left, left_ts, left_host_ts, left_frame_number);
        }
        if (!l || !r) {
            return false;
        }
    }
    const int64_t final_offset =
        static_cast<int64_t>(right_frame_number) - static_cast<int64_t>(left_frame_number);
    if (final_offset != *frame_number_offset_) {
        static unsigned int frame_mismatch_count = 0;
        ++frame_mismatch_count;
        if (frame_mismatch_count <= 5 || frame_mismatch_count % 30 == 0) {
            std::cerr << "Stereo frame counter mismatch: left=" << left_frame_number
                      << " right=" << right_frame_number
                      << " expected_offset=" << *frame_number_offset_ << std::endl;
        }
        return false;
    }

    // The cameras share a hardware trigger, therefore the first frame obtained
    // concurrently from both SDK queues is the correct stereo pair. USB host
    // arrival timestamps describe transfer scheduling, not exposure time: on
    // this dual-USB setup they may differ by tens of milliseconds even for the
    // same trigger edge. Retrying based on that host-time difference discarded
    // valid frames and reduced a 10 Hz stream to about 5 Hz.
    const double host_delta_ms = hostTimestampDeltaMs(left_host_ts, right_host_ts);
    if (host_delta_ms > 30.0) {
        static unsigned int delayed_delivery_count = 0;
        ++delayed_delivery_count;
        if (delayed_delivery_count <= 5 || delayed_delivery_count % 100 == 0) {
            std::cerr << "Stereo USB delivery skew=" << host_delta_ms
                      << " ms; keeping the concurrently acquired hardware-trigger pair"
                      << std::endl;
        }
    }
    return true;

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
