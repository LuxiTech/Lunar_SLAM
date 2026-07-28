// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "event_camera_renderer/renderer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vector>

#include "event_camera_renderer/check_endian.h"

namespace event_camera_renderer
{
bool Renderer::PeriodEstimator::update(uint64_t t)
{
  if (num_initial_ >= 1) {
    if (t <= last_time_) {
      return (false);
    }
    const double dt = static_cast<int64_t>(t) - static_cast<int64_t>(last_time_);
    if (num_initial_ > 1) {
      est_period_ = est_period_ * 0.9 + 0.1 * dt;
    } else {
      est_period_ = dt;
    }
  } else {
    num_initial_++;
  }
  last_time_ = t;
  return (true);
}

Renderer::Renderer(const rclcpp::NodeOptions & options)
: Node(
    "event_camera_renderer",
    rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
{
  this->get_parameter_or("display_type", displayType_, std::string("time_slice"));
  display_ = Display::newInstance(displayType_);
  if (!display_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "invalid display type: " << displayType_);
    throw std::runtime_error("invalid display type!");
  }
  this->get_parameter_or("event_qos_depth", eventQosDepth_, 10);
  eventQosDepth_ = std::max(1, eventQosDepth_);
  this->get_parameter_or("event_queue_memory_limit", eventQueueMemoryLimit_, 10 * 1024 * 1024);
  double fps;
  this->get_parameter_or("fps", fps, 25.0);
  if (fps <= 0.0) {
    throw std::invalid_argument("fps must be greater than zero");
  }
  sliceTime_ = 1.0 / fps;
  int maxWaitFrames;
  this->get_parameter_or("max_wait_frames", maxWaitFrames, 5);
  maxDelay_ = rclcpp::Duration::from_seconds(sliceTime_ * maxWaitFrames);

  imageMsgTemplate_.height = 0;
#ifdef IMAGE_TRANSPORT_USE_QOS
  const auto qosProf = rclcpp::SystemDefaultsQoS();
#else
  const auto qosProf = rmw_qos_profile_default;
#endif
  imagePub_ = image_transport::create_publisher(
#ifdef IMAGE_TRANSPORT_USE_NODEINTERFACE
    *this,
#else
    this,
#endif
    "~/image_raw", qosProf);

  this->get_parameter_or("show_window", showWindow_, false);
  this->get_parameter_or("preview_in_main_thread", previewRunsOnMainThread_, false);
  this->get_parameter_or("preview_fps", previewFps_, 30.0);
  this->get_parameter_or(
    "preview_window_name", previewWindowName_, std::string("EVK4 Low-Latency Event View"));
  if (showWindow_) {
    if (previewFps_ <= 0.0) {
      throw std::invalid_argument("preview_fps must be greater than zero");
    }
    previewActive_.store(true);
    if (!previewRunsOnMainThread_) {
      previewThread_ = std::thread(&Renderer::runPreview, this);
    }
  }

  RCLCPP_INFO(this->get_logger(), "renderer_node started up, waiting for subscribers");
  // check by polling b/c ROS2 image transport does not notify about subscription changes
  subscriptionCheckTimer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration(1, 0),
    std::bind(&Renderer::subscriptionCheckTimerExpired, this));
}

Renderer::~Renderer()
{
  if (frameTimer_) {
    frameTimer_->cancel();
  }
  if (subscriptionCheckTimer_) {
    subscriptionCheckTimer_->cancel();
  }
  stopPreview_.store(true);
  previewCv_.notify_all();
  if (previewThread_.joinable()) {
    previewThread_.join();
  }
}

bool Renderer::hasOutputInterest()
{
  return previewActive_.load() || imagePub_.getNumSubscribers() != 0;
}

void Renderer::subscriptionCheckTimerExpired()
{
  // this silly dance is only necessary because ROS2 at this time does not support
  // callbacks when subscribers come and go
  if (hasOutputInterest()) {
    // -------------- subscribers ---------------------
    if (!display_->hasImage()) {
      // we have subscribers but no image is being updated yet, so start doing so
      startNewImage();
    }
    if (!eventSub_) {
      RCLCPP_INFO(this->get_logger(), "subscribing to events!");
      auto qos =
        rclcpp::QoS(rclcpp::KeepLast(eventQosDepth_)).best_effort().durability_volatile();
      eventSub_ = this->create_subscription<event_camera_msgs::msg::EventPacket>(
        "~/events", qos, std::bind(&Renderer::eventMsg, this, std::placeholders::_1));
    }
    if (!frameTimer_) {
      // start publishing frames if there is interest in either camerainfo or image
      frameTimer_ = rclcpp::create_timer(
        this, get_clock(), rclcpp::Duration::from_seconds(sliceTime_),
        std::bind(&Renderer::frameTimerExpired, this));
    }
  } else {
    // -------------- no subscribers -------------------
    bool resetState = false;
    if (eventSub_) {
      RCLCPP_INFO(this->get_logger(), "unsubscribing from events!");
      eventSub_.reset();
      resetState = true;
    }
    if (frameTimer_) {
      // if nobody is listening, stop publishing frames if this is currently happening
      frameTimer_->cancel();
      frameTimer_.reset();
    }
    if (resetState) {
      resetRendererState();
    }
  }
}

void Renderer::resetRendererState()
{
  frames_.clear();
  events_ = std::queue<EventPacket::ConstSharedPtr>();
  eventQueueMemory_ = 0;
  framePeriod_ = PeriodEstimator();
  imageMsgTemplate_ = sensor_msgs::msg::Image();
  encoding_.clear();
  clearPreviewImage();

  // Recreate the display to clear decoder state and SharpDisplay's event window.
  display_ = Display::newInstance(displayType_);
  if (!display_) {
    throw std::runtime_error("failed to reset display!");
  }
}

void Renderer::addNewFrame(const FrameTime & ft)
{
  // If no events have come in for a while, publish empty frames to avoid burst publishing later

  while (!frames_.empty() && (ft.ros_time - frames_.front().ros_time) >= maxDelay_) {
    const auto delay = ft.ros_time - frames_.front().ros_time;
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "low event rate or slow processing, publishing empty frames!");
    publishFrame(frames_.front());
    frames_.pop_front();
  }
  frames_.push_back(ft);
  processEventMessages();
}

void Renderer::eventMsg(EventPacket::ConstSharedPtr msg)
{
  const auto age = this->get_clock()->now() - rclcpp::Time(msg->header.stamp);
  if (age > maxDelay_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "dropping stale event packet to keep display live!");
    if (display_->isInitialized()) {
      resetRendererState();
    }
    return;
  }

  if (!display_->isInitialized()) {
    RCLCPP_INFO_STREAM(
      get_logger(), "initializing display for image size " << msg->width << " x " << msg->height
                                                           << " encoding: " << msg->encoding);
    encoding_ = msg->encoding;
    imageMsgTemplate_.header = msg->header;
    imageMsgTemplate_.width = msg->width;
    imageMsgTemplate_.height = msg->height;
    imageMsgTemplate_.encoding = "bgr8";
    imageMsgTemplate_.is_bigendian = check_endian::isBigEndian();
    imageMsgTemplate_.step = 3 * imageMsgTemplate_.width;
    startNewImage();
    display_->initialize(*msg);
    display_->setHeaderTime(rclcpp::Time(msg->header.stamp));
  } else {
    if (
      imageMsgTemplate_.height != msg->height || imageMsgTemplate_.width != msg->width ||
      encoding_ != msg->encoding) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "cannot change encoding type or sensor size on the fly!");
      return;
    }
  }
  if (msg->events.size() >= static_cast<size_t>(eventQueueMemoryLimit_)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "incoming event message exceeds queue memory limit, dropping it!");
    return;
  }
  if (eventQueueMemory_ + msg->events.size() >= static_cast<size_t>(eventQueueMemoryLimit_)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "event message queue full, clearing stale renderer state!");
    resetRendererState();
    return;
  }
  events_.push(msg);
  eventQueueMemory_ += msg->events.size();
  processEventMessages();
}

void Renderer::resetTime()
{
  RCLCPP_WARN(get_logger(), "time is going backwards, resetting!");
  frames_.clear();
  events_ = std::queue<EventPacket::ConstSharedPtr>();
  eventQueueMemory_ = 0;
  framePeriod_ = PeriodEstimator();
  if (display_) {
    display_->resetTime();
  }
}

void Renderer::frameTimerExpired()
{
  const rclcpp::Time t = this->get_clock()->now();
  if (!framePeriod_.update(t.nanoseconds())) {
    resetTime();  // time went backwards, reset all time related state
    return;
  }
  if (!hasOutputInterest()) {
    return;
  }
  if (display_->isInitialized()) {
    // must have ros-to-sensor time offset before adding frame
    addNewFrame(FrameTime(t, display_->rosToSensorTime(t)));
  }
}

void Renderer::processEventMessages()
{
  if (!display_->isInitialized()) {
    return;
  }
  auto & frames = frames_;
  while (!events_.empty() && !frames.empty()) {
    const auto & msg = events_.front();
    if (!display_->setHeaderTime(rclcpp::Time(msg->header.stamp))) {
      resetTime();  // will empty the event queue!
      return;
    }
    while (!frames.empty()) {
      const uint64_t time_limit = frames.front().sensor_time;
      uint64_t next_time = 0;
      const auto dt = this->get_clock()->now() - rclcpp::Time(msg->header.stamp);
      if (dt > maxDelay_) {
        RCLCPP_WARN_STREAM_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "display is lagging by " << dt.seconds() << " seconds!");
      }
      if (!display_->update(*msg, time_limit, &next_time)) {
        // event message was completely decoded. Cannot emit frame yet
        // because more events may arrive that are before the frame time
        eventQueueMemory_ -= msg->events.size();
        events_.pop();
        display_->setIsFirstTimeInPacket(true);
        break;
      }
      while (!frames.empty() && frames.front().sensor_time <= next_time) {
        publishFrame(frames.front());
        frames.pop_front();
      }
    }
  }
}

void Renderer::publishFrame(const FrameTime & ft)
{
  const bool hasImageSubscribers = imagePub_.getNumSubscribers() != 0;
  if ((hasImageSubscribers || previewActive_.load()) && display_->hasImage()) {
    // Take memory management from the image updater.
    sensor_msgs::msg::Image::UniquePtr updated_img = display_->getImage();
    updated_img->header.stamp = ft.ros_time;

    if (previewActive_.load()) {
      if (hasImageSubscribers) {
        queuePreviewImage(std::make_unique<sensor_msgs::msg::Image>(*updated_img));
      } else {
        queuePreviewImage(std::move(updated_img));
      }
    }
    if (hasImageSubscribers) {
      imagePub_.publish(std::move(updated_img));
    }
    startNewImage();
  }
}

void Renderer::queuePreviewImage(sensor_msgs::msg::Image::UniquePtr image)
{
  {
    std::lock_guard<std::mutex> lock(previewMutex_);
    previewImage_ = std::move(image);
    ++previewImageSequence_;
  }
  previewCv_.notify_one();
}

void Renderer::clearPreviewImage()
{
  std::lock_guard<std::mutex> lock(previewMutex_);
  previewImage_.reset();
}

void Renderer::runPreview()
{
  using Clock = std::chrono::steady_clock;
  using Seconds = std::chrono::duration<double>;

  sensor_msgs::msg::Image::UniquePtr currentImage;
  uint64_t consumedSequence = 0;
  {
    std::unique_lock<std::mutex> lock(previewMutex_);
    while (!stopPreview_.load() && rclcpp::ok() && !previewImage_) {
      previewCv_.wait_for(lock, std::chrono::milliseconds(100));
    }
    if (stopPreview_.load() || !rclcpp::ok()) {
      previewActive_.store(false);
      return;
    }
    currentImage = std::move(previewImage_);
    consumedSequence = previewImageSequence_;
  }

  try {
    cv::namedWindow(previewWindowName_, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow(
      previewWindowName_, static_cast<int>(currentImage->width),
      static_cast<int>(currentImage->height));

    const auto period = std::chrono::duration_cast<Clock::duration>(Seconds(1.0 / previewFps_));
    auto nextRefresh = Clock::now();
    auto statsStart = nextRefresh;
    auto previousRefresh = nextRefresh;
    size_t refreshCount = 0;
    size_t newImageCount = 0;
    size_t repeatedImageCount = 0;
    double maxIntervalMs = 0.0;
    double sumIntervalMs = 0.0;
    double sumSquaredIntervalMs = 0.0;

    while (!stopPreview_.load() && rclcpp::ok()) {
      std::this_thread::sleep_until(nextRefresh);
      const auto refreshTime = Clock::now();
      if (refreshCount != 0) {
        const double intervalMs =
          std::chrono::duration<double, std::milli>(refreshTime - previousRefresh).count();
        maxIntervalMs = std::max(maxIntervalMs, intervalMs);
        sumIntervalMs += intervalMs;
        sumSquaredIntervalMs += intervalMs * intervalMs;
      }
      previousRefresh = refreshTime;

      bool receivedNewImage = false;
      {
        std::lock_guard<std::mutex> lock(previewMutex_);
        if (previewImage_ && previewImageSequence_ != consumedSequence) {
          currentImage = std::move(previewImage_);
          consumedSequence = previewImageSequence_;
          receivedNewImage = true;
        }
      }
      if (receivedNewImage) {
        ++newImageCount;
      } else {
        ++repeatedImageCount;
      }

      cv::Mat image(
        static_cast<int>(currentImage->height), static_cast<int>(currentImage->width), CV_8UC3,
        currentImage->data.data(), currentImage->step);
      cv::imshow(previewWindowName_, image);
      const int key = cv::pollKey();
      ++refreshCount;

      const auto now = Clock::now();
      const double statsSeconds = Seconds(now - statsStart).count();
      if (statsSeconds >= 5.0 && refreshCount > 1) {
        const double intervalCount = static_cast<double>(refreshCount - 1);
        const double meanIntervalMs = sumIntervalMs / intervalCount;
        const double variance =
          std::max(0.0, sumSquaredIntervalMs / intervalCount - meanIntervalMs * meanIntervalMs);
        const double repeatPercent =
          100.0 * static_cast<double>(repeatedImageCount) / static_cast<double>(refreshCount);
        const double newImageRate = static_cast<double>(newImageCount) / statsSeconds;
        RCLCPP_INFO(
          get_logger(),
          "preview: %.2f Hz, new images %.2f Hz, interval std %.2f ms, max %.2f ms, repeated "
          "%.1f%% (%zu/%zu)",
          static_cast<double>(refreshCount) / statsSeconds, newImageRate, std::sqrt(variance),
          maxIntervalMs, repeatPercent, repeatedImageCount, refreshCount);
        statsStart = now;
        refreshCount = 0;
        newImageCount = 0;
        repeatedImageCount = 0;
        maxIntervalMs = 0.0;
        sumIntervalMs = 0.0;
        sumSquaredIntervalMs = 0.0;
      }

      if (key == 27 || key == 'q' || key == 'Q') {
        break;
      }
      if (cv::getWindowProperty(previewWindowName_, cv::WND_PROP_VISIBLE) < 1.0) {
        break;
      }

      nextRefresh += period;
      if (nextRefresh <= Clock::now()) {
        nextRefresh = Clock::now() + period;
      }
    }
    cv::destroyWindow(previewWindowName_);
  } catch (const cv::Exception & error) {
    RCLCPP_ERROR(get_logger(), "preview window failed: %s", error.what());
  }
  previewActive_.store(false);
}

void Renderer::startNewImage()
{
  if (imageMsgTemplate_.height != 0) {
    sensor_msgs::msg::Image::UniquePtr img(new sensor_msgs::msg::Image(imageMsgTemplate_));
    img->data.resize(img->height * img->step, 0);  // allocate memory and set all bytes to zero
    display_->setImage(&img);                      // event publisher will also render image now
  }
}

}  // namespace event_camera_renderer

RCLCPP_COMPONENTS_REGISTER_NODE(event_camera_renderer::Renderer)
