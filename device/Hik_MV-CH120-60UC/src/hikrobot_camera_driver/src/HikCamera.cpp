#include "hikrobot_camera_driver/HikCamera.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <vector>

namespace
{
void printSdkError(const char* operation, int ret)
{
    std::cerr << operation << " failed: 0x" << std::hex
              << static_cast<unsigned int>(ret) << std::dec;
    if (ret == static_cast<int>(MV_E_ACCESS_DENIED)) {
        std::cerr << " (camera is already opened by another process)";
    } else if (ret == static_cast<int>(MV_E_CALLORDER)) {
        std::cerr << " (invalid SDK call order)";
    }
    std::cerr << std::endl;
}

const char* pixelTypeName(MvGvspPixelType pixel_type)
{
    switch (pixel_type) {
        case PixelType_Gvsp_Mono8: return "Mono8";
        case PixelType_Gvsp_BGR8_Packed: return "BGR8Packed";
        case PixelType_Gvsp_RGB8_Packed: return "RGB8Packed";
        case PixelType_Gvsp_BayerGR8: return "BayerGR8";
        case PixelType_Gvsp_BayerRG8: return "BayerRG8";
        case PixelType_Gvsp_BayerGB8: return "BayerGB8";
        case PixelType_Gvsp_BayerBG8: return "BayerBG8";
        default: return "Unknown";
    }
}

std::string getEnumSymbol(void* handle, const char* key)
{
    MVCC_ENUMVALUE value{};
    if (MV_CC_GetEnumValue(handle, key, &value) != MV_OK) {
        return "unsupported";
    }

    MVCC_ENUMENTRY entry{};
    entry.nValue = value.nCurValue;
    if (MV_CC_GetEnumEntrySymbolic(handle, key, &entry) != MV_OK) {
        return std::to_string(value.nCurValue);
    }
    return reinterpret_cast<const char*>(entry.chSymbolic);
}

std::string getIntText(void* handle, const char* key)
{
    MVCC_INTVALUE_EX value{};
    if (MV_CC_GetIntValueEx(handle, key, &value) != MV_OK) {
        return "unsupported";
    }
    return std::to_string(value.nCurValue);
}

std::string getFloatText(void* handle, const char* key)
{
    MVCC_FLOATVALUE value{};
    if (MV_CC_GetFloatValue(handle, key, &value) != MV_OK) {
        return "unsupported";
    }
    return std::to_string(value.fCurValue);
}
}  // namespace

HikCamera::HikCamera()
{
    handle_=nullptr;
    opened_=false;
    grabbing_=false;
    swap_red_blue_=false;
    grab_fail_count_=0;
}



HikCamera::~HikCamera()
{
    close();
}



bool HikCamera::open(
    MV_CC_DEVICE_INFO* device
)
{

    if (device == nullptr || handle_ != nullptr) {
        return false;
    }

    int ret = MV_CC_CreateHandle(
        &handle_,
        device
    );


    if(ret != MV_OK)
    {
        printSdkError("CreateHandle", ret);
        handle_ = nullptr;
        return false;
    }



    ret = MV_CC_OpenDevice(
        handle_
    );


    if(ret != MV_OK)
    {
        printSdkError("OpenDevice", ret);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
        return false;
    }


    opened_=true;


    // 获取相机信息

    if(device->nTLayerType == MV_USB_DEVICE)
    {
        serial_ =
        (char*)device->SpecialInfo.stUsb3VInfo.chSerialNumber;


        model_ =
        (char*)device->SpecialInfo.stUsb3VInfo.chModelName;
    }


    std::cout << "Opened camera: model=" << model_
              << ", serial=" << serial_ << std::endl;


    return true;

}

bool HikCamera::isOpened() const
{
    return opened_;
}

bool HikCamera::loadUserSet(const std::string& user_set)
{
    if (!opened_) {
        return false;
    }

    if (user_set.empty() || user_set == "Default" || user_set == "default") {
        std::cout << "UserSet unchanged" << std::endl;
        return true;
    }

    int ret = MV_CC_SetEnumValueByString(
        handle_,
        "UserSetSelector",
        user_set.c_str()
    );
    if (ret != MV_OK) {
        printSdkError(("Set UserSetSelector " + user_set).c_str(), ret);
        return true;
    }

    ret = MV_CC_SetCommandValue(
        handle_,
        "UserSetLoad"
    );
    if (ret != MV_OK) {
        printSdkError(("UserSetLoad " + user_set).c_str(), ret);
        return true;
    }

    std::cout << "UserSetLoad=" << user_set << std::endl;
    return true;
}

void HikCamera::printCurrentSettings() const
{
    if (!opened_) {
        return;
    }

    std::cout << "Camera settings [" << serial_ << "]: "
              << "TriggerMode=" << getEnumSymbol(handle_, "TriggerMode")
              << ", TriggerSource=" << getEnumSymbol(handle_, "TriggerSource")
              << ", TriggerActivation=" << getEnumSymbol(handle_, "TriggerActivation")
              << ", Width=" << getIntText(handle_, "Width")
              << ", Height=" << getIntText(handle_, "Height")
              << ", Binning=" << getIntText(handle_, "BinningHorizontal")
              << "x" << getIntText(handle_, "BinningVertical")
              << ", Decimation=" << getIntText(handle_, "DecimationHorizontal")
              << "x" << getIntText(handle_, "DecimationVertical")
              << ", PixelFormat=" << getEnumSymbol(handle_, "PixelFormat")
              << ", ExposureAuto=" << getEnumSymbol(handle_, "ExposureAuto")
              << ", ExposureTime=" << getFloatText(handle_, "ExposureTime")
              << ", BalanceWhiteAuto=" << getEnumSymbol(handle_, "BalanceWhiteAuto")
              << std::endl;
}


bool HikCamera::start()
{

    if(!opened_ || grabbing_)
        return false;


    int ret = MV_CC_SetImageNodeNum(handle_, 8);
    if (ret != MV_OK) {
        printSdkError("SetImageNodeNum(8)", ret);
    }

    ret =
    MV_CC_StartGrabbing(
        handle_
    );


    if(ret==MV_OK)
    {
        grabbing_=true;
        return true;
    }


    printSdkError("StartGrabbing", ret);
    return false;

}

bool HikCamera::setAcquisitionModeContinuous()
{
    if (!opened_) {
        return false;
    }

    int ret = MV_CC_SetEnumValueByString(
        handle_,
        "AcquisitionMode",
        "Continuous"
    );

    if (ret != MV_OK) {
        ret = MV_CC_SetEnumValue(
            handle_,
            "AcquisitionMode",
            2
        );
    }

    if (ret != MV_OK) {
        printSdkError("Set AcquisitionMode Continuous", ret);
        return false;
    }

    std::cout << "AcquisitionMode=Continuous" << std::endl;
    return true;
}

bool HikCamera::setAcquisitionFrameRate(float frame_rate_hz)
{
    if (!opened_) {
        return false;
    }

    if (frame_rate_hz <= 0.0F) {
        std::cout << "AcquisitionFrameRate unchanged" << std::endl;
        return true;
    }

    int ret = MV_CC_SetBoolValue(
        handle_,
        "AcquisitionFrameRateEnable",
        true
    );
    if (ret != MV_OK) {
        printSdkError("Set AcquisitionFrameRateEnable", ret);
        return false;
    }

    ret = MV_CC_SetFloatValue(
        handle_,
        "AcquisitionFrameRate",
        frame_rate_hz
    );
    if (ret != MV_OK) {
        printSdkError("Set AcquisitionFrameRate", ret);
        return false;
    }

    std::cout << "AcquisitionFrameRate=" << frame_rate_hz << " Hz" << std::endl;
    return true;
}

bool HikCamera::setImageRoi(int width, int height, int offset_x, int offset_y)
{
    if (!opened_) {
        return false;
    }

    if (width <= 0 || height <= 0) {
        return true;
    }

    const std::pair<const char*, int64_t> settings[] = {
        {"OffsetX", 0},
        {"OffsetY", 0},
        {"Width", width},
        {"Height", height},
        {"OffsetX", offset_x},
        {"OffsetY", offset_y},
    };

    for (const auto& setting : settings) {
        const int ret = MV_CC_SetIntValueEx(handle_, setting.first, setting.second);
        if (ret != MV_OK) {
            printSdkError(("Set " + std::string(setting.first)).c_str(), ret);
        }
    }

    std::cout << "ROI width=" << width
              << ", height=" << height
              << ", offset_x=" << offset_x
              << ", offset_y=" << offset_y << std::endl;
    return true;
}

bool HikCamera::setImageSampling(
    int binning_horizontal,
    int binning_vertical,
    int decimation_horizontal,
    int decimation_vertical)
{
    if (!opened_) {
        return false;
    }

    if (binning_horizontal == 1 &&
        binning_vertical == 1 &&
        decimation_horizontal == 1 &&
        decimation_vertical == 1) {
        std::cout << "Sampling override disabled; keeping UserSet values" << std::endl;
        return true;
    }

    const std::pair<const char*, int64_t> settings[] = {
        {"BinningHorizontal", binning_horizontal},
        {"BinningVertical", binning_vertical},
        {"DecimationHorizontal", decimation_horizontal},
        {"DecimationVertical", decimation_vertical},
    };

    for (const auto& setting : settings) {
        const int ret = MV_CC_SetIntValueEx(handle_, setting.first, setting.second);
        if (ret != MV_OK) {
            printSdkError(("Set " + std::string(setting.first)).c_str(), ret);
        }
    }

    std::cout << "Sampling binning="
              << binning_horizontal << "x" << binning_vertical
              << ", decimation="
              << decimation_horizontal << "x" << decimation_vertical
              << " (unsupported nodes are ignored)" << std::endl;
    return true;
}



bool HikCamera::grab(
    cv::Mat& image,
    uint64_t& timestamp
)
{

    if (!grabbing_) {
        return false;
    }

    MV_FRAME_OUT frame{};

    int ret =
    MV_CC_GetImageBuffer(
        handle_,
        &frame,
        1000
    );


    if(ret!=MV_OK)
    {
        ++grab_fail_count_;
        if (grab_fail_count_ <= 5 || grab_fail_count_ % 30 == 0) {
            printSdkError(("GetImageBuffer [" + serial_ + "]").c_str(), ret);
        }
        return false;
    }

    grab_fail_count_ = 0;



    const auto pixel_type = frame.stFrameInfo.enPixelType;
    const int width = static_cast<int>(frame.stFrameInfo.nWidth);
    const int height = static_cast<int>(frame.stFrameInfo.nHeight);
    const uint64_t captured_timestamp =
        (static_cast<uint64_t>(frame.stFrameInfo.nDevTimeStampHigh) << 32) |
        frame.stFrameInfo.nDevTimeStampLow;

    // Do not keep an SDK buffer occupied while OpenCV performs Bayer conversion.
    // This matters for two cameras receiving the same hardware trigger.
    frame_buffer_.resize(frame.stFrameInfo.nFrameLen);
    std::memcpy(frame_buffer_.data(), frame.pBufAddr, frame.stFrameInfo.nFrameLen);

    const int free_ret = MV_CC_FreeImageBuffer(handle_, &frame);
    if (free_ret != MV_OK) {
        printSdkError("FreeImageBuffer", free_ret);
        return false;
    }

    timestamp = captured_timestamp;
    bool converted = false;

    if (pixel_type == PixelType_Gvsp_BGR8_Packed) {
        image = cv::Mat(height, width, CV_8UC3, frame_buffer_.data()).clone();
        converted = true;
    } else if (pixel_type == PixelType_Gvsp_RGB8_Packed) {
        cv::cvtColor(cv::Mat(height, width, CV_8UC3, frame_buffer_.data()), image, cv::COLOR_RGB2BGR);
        converted = true;
    } else {
        int conversion = -1;
        switch (pixel_type) {
            case PixelType_Gvsp_Mono8: conversion = cv::COLOR_GRAY2BGR; break;
            case PixelType_Gvsp_BayerGR8: conversion = cv::COLOR_BayerGR2BGR; break;
            case PixelType_Gvsp_BayerRG8: conversion = cv::COLOR_BayerRG2BGR; break;
            case PixelType_Gvsp_BayerGB8: conversion = cv::COLOR_BayerGB2BGR; break;
            case PixelType_Gvsp_BayerBG8: conversion = cv::COLOR_BayerBG2BGR; break;
            default: break;
        }
        if (conversion >= 0) {
            cv::cvtColor(cv::Mat(height, width, CV_8UC1, frame_buffer_.data()), image, conversion);
            converted = true;
        }
    }

    if (converted && swap_red_blue_ && image.channels() == 3) {
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    }

    if (!converted) {
        std::cerr << "Unsupported pixel type: " << pixelTypeName(pixel_type)
                  << " (0x" << std::hex << static_cast<unsigned int>(pixel_type)
                  << std::dec << ")" << std::endl;
    }
    return converted;

}


void HikCamera::stop()
{

    if(grabbing_)
    {
        MV_CC_StopGrabbing(handle_);
        grabbing_=false;
    }

}



void HikCamera::close()
{

    if(handle_)
    {

        stop();

        MV_CC_CloseDevice(handle_);

        MV_CC_DestroyHandle(handle_);

        handle_=nullptr;
        opened_=false;
        grabbing_=false;
    }

}

std::string HikCamera::getSerial()
{
    return serial_;
}

//获取相机触发状态
bool HikCamera::getTriggerMode()
{

    MVCC_ENUMVALUE value;

    int ret =
    MV_CC_GetEnumValue(
        handle_,
        "TriggerMode",
        &value
    );


    if(ret != MV_OK)
    {
        std::cout
        <<"get trigger failed:"
        <<std::hex
        <<ret
        <<std::endl;

        return false;
    }


    std::cout
    <<"TriggerMode="
    <<value.nCurValue
    <<std::endl;


    return true;

}

//设置相机触发状态
bool HikCamera::setTriggerMode(bool enable)
{

    const int selector_ret = MV_CC_SetEnumValueByString(
        handle_,
        "TriggerSelector",
        "FrameStart"
    );
    if (selector_ret != MV_OK) {
        printSdkError("Set TriggerSelector FrameStart", selector_ret);
    }

    int ret = MV_CC_SetEnumValueByString(
        handle_,
        "TriggerMode",
        enable ? "On" : "Off"
    );

    if (ret != MV_OK) {
        ret = MV_CC_SetEnumValue(
            handle_,
            "TriggerMode",
            enable ? 1 : 0
        );
    }

    if(ret != MV_OK)
    {
        printSdkError("Set TriggerMode", ret);

        return false;
    }

    std::cout << "TriggerMode=" << (enable ? "On" : "Off") << std::endl;

    return true;

}

/*
int ret =
    MV_CC_SetEnumValue(
        handle_,
        "TriggerMode",
        enable ? 1 : 0
    );


    if(ret != MV_OK)
    {
        std::cout<<"TriggerMode failed:"<<std::hex<<ret<<std::endl;

        return false;
    }


    return true;

}
*/

//获取当前触发源
bool HikCamera::getTriggerSourceName()
{

    MVCC_ENUMVALUE value;


    int ret =
    MV_CC_GetEnumValue(
        handle_,
        "TriggerSource",
        &value
    );


    if(ret != MV_OK)
    {
        std::cout<<"Get TriggerSource failed:"<<std::hex<<ret<<std::endl;

        return false;
    }


    MVCC_ENUMENTRY entry;

    memset(&entry,0,sizeof(entry));


    entry.nValue =
        value.nCurValue;


    ret =
    MV_CC_GetEnumEntrySymbolic(
        handle_,
        "TriggerSource",
        &entry
    );


    if(ret != MV_OK)
    {
        std::cout<<"Get symbolic failed:"<<std::hex<<ret<<std::endl;

        return false;
    }


    std::cout<<"TriggerSource="<<entry.chSymbolic<<std::endl;


    return true;
}

//设置相机触发源为Line0
bool HikCamera::setTriggerSourceLine0()
{
    return setTriggerSource("Line0");
}

bool HikCamera::setTriggerSource(const std::string& source)
{
    if (source.empty() || source == "Default" || source == "default") {
        std::cout << "TriggerSource unchanged" << std::endl;
        return true;
    }

    int ret =
    MV_CC_SetEnumValueByString(
        handle_,
        "TriggerSource",
        source.c_str()
    );


    if(ret != MV_OK)
    {
        printSdkError(("Set TriggerSource " + source).c_str(), ret);

        return false;
    }

    std::cout << "TriggerSource=" << source << std::endl;

    return true;

}

//设置自动曝光API
bool HikCamera::setAutoExposure(bool enable)
{

    if(!handle_)
        return false;


    if(enable)
    {

        return MV_CC_SetEnumValue(
            handle_,
            "ExposureAuto",
            2
        )
        == MV_OK;

    }
    else
    {

        return MV_CC_SetEnumValue(
            handle_,
            "ExposureAuto",
            0
        )
        == MV_OK;

    }

}

//曝光设置API
bool HikCamera::setExposureTime(float exposure)
{

    if(!opened_)
        return false;


    int ret =
    MV_CC_SetFloatValue(
        handle_,
        "ExposureTime",
        exposure
    );


    if(ret != MV_OK)
    {
        std::cout<<"set exposure failed:"<<std::hex<<ret<<std::endl;

        return false;
    }

    return true;
}

//设置自动增益API
bool HikCamera::setAutoGain(bool enable)
{

    return
    MV_CC_SetEnumValue(
        handle_,
        "GainAuto",
        enable ? 2 : 0
    )
    == MV_OK;

}

bool HikCamera::setAutoWhiteBalance(bool enable)
{
    if (!opened_) {
        return false;
    }

    const int ret = MV_CC_SetEnumValue(
        handle_,
        "BalanceWhiteAuto",
        enable ? 2 : 0
    );

    if (ret != MV_OK) {
        printSdkError("Set BalanceWhiteAuto", ret);
        return true;
    }

    std::cout << "BalanceWhiteAuto=" << getEnumSymbol(handle_, "BalanceWhiteAuto") << std::endl;
    return true;
}

//设置增益API
bool HikCamera::setGain(float gain)
{

    if(!opened_)
        return false;


    int ret =
    MV_CC_SetFloatValue(
        handle_,
        "Gain",
        gain
    );


    if(ret != MV_OK)
    {
        std::cout<<"set gain failed:"<<std::hex<<ret<<std::endl;

        return false;
    }


    return true;
}

//设置像素格式API
bool HikCamera::setPixelFormat(
    const std::string& format
)
{
    if (format.empty() || format == "Default" || format == "default") {
        std::cout << "PixelFormat unchanged" << std::endl;
        return true;
    }

    std::vector<std::string> formats{format};
    if (format == "RGB8Packed" || format == "BGR8Packed" || format == "Color") {
        formats = {
            format,
            "BGR8Packed",
            "RGB8Packed",
            "BayerGB8",
            "BayerRG8",
            "BayerGR8",
            "BayerBG8",
        };
    }

    formats.erase(
        std::unique(formats.begin(), formats.end()),
        formats.end()
    );

    int last_ret = MV_OK;
    for (const auto& candidate : formats) {
        last_ret = MV_CC_SetEnumValueByString(
            handle_,
            "PixelFormat",
            candidate.c_str()
        );

        if (last_ret == MV_OK) {
            std::cout << "PixelFormat=" << getPixelFormat() << std::endl;
            return true;
        }
    }

    printSdkError(("Set PixelFormat " + format).c_str(), last_ret);
    return false;

}

void HikCamera::setSwapRedBlue(bool enable)
{
    swap_red_blue_ = enable;
    std::cout << "SwapRedBlue=" << (swap_red_blue_ ? "true" : "false") << std::endl;
}

//查看当前像素格式
std::string HikCamera::getPixelFormat()
{

    MVCC_ENUMVALUE value;


    memset(
        &value,
        0,
        sizeof(value)
    );


    if(MV_CC_GetEnumValue(
        handle_,
        "PixelFormat",
        &value
    ) != MV_OK)
    {
        return "";
    }

    MVCC_ENUMENTRY entry;
    memset(&entry, 0, sizeof(entry));
    entry.nValue = value.nCurValue;

    if (MV_CC_GetEnumEntrySymbolic(
        handle_,
        "PixelFormat",
        &entry
    ) == MV_OK)
    {
        return reinterpret_cast<char*>(entry.chSymbolic);
    }

    return std::to_string(
        value.nCurValue
    );

}
