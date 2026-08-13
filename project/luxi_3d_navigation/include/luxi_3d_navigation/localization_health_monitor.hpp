#pragma once

#include <optional>
#include <stdexcept>

namespace luxi_3d_navigation
{

enum class LocalizationHealthAction
{
  kTrack,
  kPause,
  kStop,
};

class LocalizationHealthMonitor
{
public:
  explicit LocalizationHealthMonitor(const double recovery_grace_period)
  : recovery_grace_period_(recovery_grace_period)
  {
    if (recovery_grace_period_ <= 0.0) {
      throw std::invalid_argument("localization recovery grace period must be positive");
    }
  }

  LocalizationHealthAction update(const bool healthy, const double now_seconds)
  {
    if (stopped_) {
      return LocalizationHealthAction::kStop;
    }
    if (healthy) {
      fault_started_at_.reset();
      return LocalizationHealthAction::kTrack;
    }
    if (!fault_started_at_.has_value()) {
      fault_started_at_ = now_seconds;
      return LocalizationHealthAction::kPause;
    }
    if (now_seconds - *fault_started_at_ < recovery_grace_period_) {
      return LocalizationHealthAction::kPause;
    }
    stopped_ = true;
    return LocalizationHealthAction::kStop;
  }

  void reset()
  {
    fault_started_at_.reset();
    stopped_ = false;
  }

private:
  double recovery_grace_period_;
  std::optional<double> fault_started_at_;
  bool stopped_{false};
};

}  // namespace luxi_3d_navigation
