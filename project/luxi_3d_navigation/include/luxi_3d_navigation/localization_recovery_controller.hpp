#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

namespace luxi_3d_navigation
{

enum class LocalizationRecoveryAction
{
  kTrack,
  kDeadReckon,
  kHold,
  kRotate,
  kStop,
};

struct LocalizationRecoveryParameters
{
  double dead_reckoning_duration{0.80};
  // Zero keeps recovery active until localization succeeds or the navigation
  // task is explicitly cancelled. A positive value retains the optional
  // bounded behaviour for deployments that require it.
  double recovery_timeout{0.0};
  double healthy_confirmation_time{1.0};
};

class LocalizationRecoveryController
{
public:
  explicit LocalizationRecoveryController(LocalizationRecoveryParameters parameters = {})
  : parameters_(parameters)
  {
    if (parameters_.dead_reckoning_duration < 0.0 ||
      parameters_.recovery_timeout < 0.0 ||
      (parameters_.recovery_timeout > 0.0 &&
      parameters_.recovery_timeout <= parameters_.dead_reckoning_duration) ||
      parameters_.healthy_confirmation_time < 0.0)
    {
      throw std::invalid_argument("localization recovery parameters are invalid");
    }
  }

  LocalizationRecoveryAction update(
    const std::string & health, const double now_seconds,
    const bool forward_clear_for_dead_reckoning = true)
  {
    if (!std::isfinite(now_seconds)) {
      return LocalizationRecoveryAction::kStop;
    }
    if (health == "tracking") {
      if (!fault_started_at_.has_value()) {
        return LocalizationRecoveryAction::kTrack;
      }
      if (!healthy_started_at_.has_value()) {
        healthy_started_at_ = now_seconds;
      }
      if (now_seconds - *healthy_started_at_ < parameters_.healthy_confirmation_time) {
        return LocalizationRecoveryAction::kHold;
      }
      reset();
      return LocalizationRecoveryAction::kTrack;
    }

    healthy_started_at_.reset();
    if (!fault_started_at_.has_value()) {
      fault_started_at_ = now_seconds;
    }
    const double elapsed = std::max(0.0, now_seconds - *fault_started_at_);
    if (parameters_.recovery_timeout > 0.0 && elapsed >= parameters_.recovery_timeout) {
      return LocalizationRecoveryAction::kStop;
    }
    if (elapsed < parameters_.dead_reckoning_duration) {
      return forward_clear_for_dead_reckoning &&
        (health == "degraded" || health == "dead_reckoning") ?
        LocalizationRecoveryAction::kDeadReckon : LocalizationRecoveryAction::kHold;
    }
    return LocalizationRecoveryAction::kRotate;
  }

  void reset()
  {
    fault_started_at_.reset();
    healthy_started_at_.reset();
  }

private:
  LocalizationRecoveryParameters parameters_;
  std::optional<double> fault_started_at_;
  std::optional<double> healthy_started_at_;
};

}  // namespace luxi_3d_navigation
