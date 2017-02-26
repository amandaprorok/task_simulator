#include "task_robot.h"

#include <cmath>
#include <iostream>

#include "gflags/gflags.h"
#include "util/utils.h"

DECLARE_double(hide_colors_until);

namespace {
constexpr double kEpsilon = 0.001;

// Control constants.
constexpr double kGainU = 2.0;
constexpr double kGainW = 4.0;
constexpr double kGainOrientation = 4.0;
constexpr double kMinOrientationDistance = 0.1;
constexpr double kSafetyDistance = 0.3;
constexpr double kActiveRegion = 0.6;
constexpr double kGainAvoidanceU = 4.0;
constexpr double kGainAvoidanceW = 8.0;
}  // namespace

void TaskRobot::SetPath(const Path& path) {
  path_ = path;
}

bool TaskRobot::Initialize() {
  first_execution_ = true;
  return true;
}

void TaskRobot::Execute(double t, double dt) {
  static double avoidance_factor = 1.0 / sqrt(kSafetyDistance) - 1.0 / sqrt(kActiveRegion);

  if (first_execution_) {
    original_color_ = color();
    first_execution_ = false;
  }
  if (FLAGS_hide_colors_until >= 0.0 && t < FLAGS_hide_colors_until) {
    SetColor({0.5, 0.5, 0.5});
  } else {
    SetColor(original_color_);
  }

  // Follow reference path.
  double xd, yd, dxd, dyd, ddxd, ddyd;
  bool reverse;
  path_.Get(t, &xd, &yd, &dxd, &dyd, &ddxd, &ddyd, &reverse);
  double direction = (reverse) ? -1.0 : 1.0;
  double u_reference = sqrt(dxd * dxd + dyd * dyd);
  double w_reference = 0.0;
  double yaw_reference = 0.0;
  if (std::abs(u_reference) < kEpsilon) {
    u_reference = 0.0;
  } else {
    w_reference = (-dyd * ddxd + dxd * ddyd) / (dxd * dxd + dyd * dyd);
    yaw_reference = atan2(dyd, dxd);
  }

  // Adjust orientation on path if not correct.
  double gain_orientation = std::max(u_reference, kMinOrientationDistance);
  double a_orientation = NormalizeAngle(
      atan2(yd + sin(yaw_reference) * gain_orientation-y(),
            xd + cos(yaw_reference) * gain_orientation-x()) -
      yaw() + (double)reverse * M_PI);

  // Reach desired position on path if not correct.
  double dx = xd - x();
  double dy = yd - y();
  double e = sqrtf(dx * dx + dy * dy);
  double a = NormalizeAngle(atan2(dy, dx) - yaw());
  double u = direction * u_reference;
  double w = w_reference + kGainOrientation * a_orientation;
  if (std::abs(e) > kEpsilon) {
    u += kGainU * e * cos(a);
    w += kGainW * e * sin(a);
  }

  // Correct inputs for neighbor avoidance.
  for (const Robot* neighbor : neighbors()) {
    double dx = neighbor->x() - x();
    double dy = neighbor->y() - y();
    e = sqrt(dx * dx + dy * dy);
    a = NormalizeAngle(atan2(dy, dx) - yaw());
    if (e < kActiveRegion) {
      e = std::max(e, 0.05);
      double f = avoidance_factor * (1.0 / sqrtf(e) - 1.0 / sqrt(kActiveRegion));
      u -= kGainAvoidanceU * e * cosf(a) * f;
      w -= kGainAvoidanceW * sinf(a) * f;
    }
  }

  // Sets controls.
  SetControlInputs(u, w);
}
