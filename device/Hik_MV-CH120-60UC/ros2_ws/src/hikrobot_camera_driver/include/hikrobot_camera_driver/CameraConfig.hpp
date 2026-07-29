#pragma once

#include <string>

struct StereoCameraConfig
{
    std::string left_serial;
    std::string right_serial;
    std::string user_set{"UserSet1"};
    bool external_trigger{false};
    std::string trigger_source{"Line0"};
    float frame_rate_hz{25.0F};
    int image_width{0};
    int image_height{0};
    int offset_x{0};
    int offset_y{0};
    int binning_horizontal{1};
    int binning_vertical{1};
    int decimation_horizontal{1};
    int decimation_vertical{1};
    bool auto_exposure{true};
    float exposure_time_us{40000.0F};
    bool auto_gain{true};
    float gain{0.0F};
    std::string pixel_format{"BayerGB8"};
    bool swap_red_blue{true};
    bool override_white_balance{false};
    bool auto_white_balance{true};
};

// Returns false and sets error when the XML cannot be parsed or is incomplete.
bool loadStereoCameraConfig(const std::string& path, StereoCameraConfig& config,
                            std::string& error);
