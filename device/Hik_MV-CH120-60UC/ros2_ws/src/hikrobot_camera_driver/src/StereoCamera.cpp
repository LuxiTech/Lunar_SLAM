#include "hikrobot_camera_driver/StereoCamera.hpp"

#include <future>
#include <iostream>


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
                !left_camera_.setAutoWhiteBalance(config.auto_white_balance) ||
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
                !right_camera_.setAutoWhiteBalance(config.auto_white_balance) ||
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
    uint64_t& right_ts
)
{


    // Both cameras receive the same hardware trigger. Retrieve and convert the
    // two frames concurrently so one Bayer conversion cannot delay the other.
    auto left_result = std::async(std::launch::async, [&]() {
        return left_camera_.grab(left, left_ts);
    });

    const bool r = right_camera_.grab(right, right_ts);
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


    return l&&r;

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
