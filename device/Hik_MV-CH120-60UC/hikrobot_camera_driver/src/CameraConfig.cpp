#include "hikrobot_camera_driver/CameraConfig.hpp"

#include <tinyxml2.h>

namespace
{
template<typename T>
bool readAttribute(const tinyxml2::XMLElement* element, const char* name, T& value);

template<>
bool readAttribute(const tinyxml2::XMLElement* element, const char* name, bool& value)
{
    return element != nullptr && element->QueryBoolAttribute(name, &value) == tinyxml2::XML_SUCCESS;
}

template<>
bool readAttribute(const tinyxml2::XMLElement* element, const char* name, float& value)
{
    return element != nullptr && element->QueryFloatAttribute(name, &value) == tinyxml2::XML_SUCCESS;
}

template<>
bool readAttribute(const tinyxml2::XMLElement* element, const char* name, int& value)
{
    return element != nullptr && element->QueryIntAttribute(name, &value) == tinyxml2::XML_SUCCESS;
}

bool readStringAttribute(const tinyxml2::XMLElement* element, const char* name,
                         std::string& value)
{
    if (element == nullptr) {
        return false;
    }
    const char* attribute = element->Attribute(name);
    if (attribute == nullptr) {
        return false;
    }
    value = attribute;
    return true;
}
}  // namespace

bool loadStereoCameraConfig(const std::string& path, StereoCameraConfig& config,
                            std::string& error)
{
    tinyxml2::XMLDocument document;
    if (document.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
        error = document.ErrorStr();
        return false;
    }

    const auto* root = document.FirstChildElement("stereo_camera");
    const auto* left = root == nullptr ? nullptr : root->FirstChildElement("left_camera");
    const auto* right = root == nullptr ? nullptr : root->FirstChildElement("right_camera");
    if (root == nullptr || !readStringAttribute(left, "serial", config.left_serial) ||
        !readStringAttribute(right, "serial", config.right_serial)) {
        error = "expected <stereo_camera> with left_camera/right_camera serial attributes";
        return false;
    }

    const auto* acquisition = root->FirstChildElement("acquisition");
    const auto* exposure = root->FirstChildElement("exposure");
    const auto* gain = root->FirstChildElement("gain");
    const auto* image = root->FirstChildElement("image");
    readStringAttribute(acquisition, "user_set", config.user_set);
    readAttribute(acquisition, "external_trigger", config.external_trigger);
    readStringAttribute(acquisition, "trigger_source", config.trigger_source);
    readAttribute(acquisition, "frame_rate_hz", config.frame_rate_hz);
    readAttribute(exposure, "auto", config.auto_exposure);
    readAttribute(exposure, "time_us", config.exposure_time_us);
    readAttribute(gain, "auto", config.auto_gain);
    readAttribute(gain, "value", config.gain);
    readStringAttribute(image, "pixel_format", config.pixel_format);
    readAttribute(image, "width", config.image_width);
    readAttribute(image, "height", config.image_height);
    readAttribute(image, "offset_x", config.offset_x);
    readAttribute(image, "offset_y", config.offset_y);
    readAttribute(image, "binning_horizontal", config.binning_horizontal);
    readAttribute(image, "binning_vertical", config.binning_vertical);
    readAttribute(image, "decimation_horizontal", config.decimation_horizontal);
    readAttribute(image, "decimation_vertical", config.decimation_vertical);
    readAttribute(image, "swap_red_blue", config.swap_red_blue);
    readAttribute(image, "auto_white_balance", config.auto_white_balance);
    return true;
}
