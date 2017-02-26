#include "path.h"

#include <math.h>

namespace {
constexpr double kEpsilon = 0.0001;
}  // namespace

void Path::SetTimeSpacing(double dt) {
  dt_ = dt;
}

void Path::Clear() {
  points_.clear();
}

void Path::AddPoint(const PathPoint& point) {
  points_.push_back(point);
}

void Path::AdjustVelocityAutomatically(double dt) {
  dt_ = dt;
  for (int i = 0; i < points_.size() - 1; ++i) {
    double dx = points_[i + 1].x - points_[i].x;
    double dy = points_[i + 1].y - points_[i].y;
    points_[i].u = sqrtf(dx*dx + dy*dy) / dt_;
  }
  points_.back().u = 0.0;
}

double Path::GetInitialAngle() const {
  double dx;
  double dy;
  Get(0.1, nullptr, nullptr, &dx, &dy, nullptr, nullptr, nullptr);
  return atan2(dy, dx);
}

double Path::GetInitialX() const {
  if (points_.size() > 0) {
    return points_[0].x;
  }
  return 0.0;
}

double Path::GetInitialY() const {
  if (points_.size() > 0) {
    return points_[0].y;
  }
  return 0.0;
}

void Path::Get(double t,
               double* x, double* y,
               double* dx, double* dy,
               double* ddx, double* ddy,
               bool* reverse) const {
  int index = (int)(t / dt_);
  if (points_.size() == 0) {
    if (x) *x = 0.0;
    if (y) *y = 0.0;
    if (dx) *dx = 0.0;
    if (dy) *dy = 0.0;
    if (ddx) *ddx = 0.0;
    if (ddy) *ddy = 0.0;
    if (reverse) *reverse = false;
    return;
  } else if (points_.size() == 1 || t < 0.0) {
    if (x) *x = points_[0].x;
    if (y) *y = points_[0].y;
    if (dx) *dx = 0.0;
    if (dy) *dy = 0.0;
    if (ddx) *ddx = 0.0;
    if (ddy) *ddy = 0.0;
    if (reverse) *reverse = (points_[0].u < 0.0);
    return;
  } else if (index >= points_.size()-1) {
    if (x) *x = points_.back().x;
    if (y) *y = points_.back().y;
    if (dx) *dx = 0.0;
    if (dy) *dy = 0.0;
    if (ddx) *ddx = 0.0;
    if (ddy) *ddy = 0.0;
    if (reverse) *reverse = (points_.back().u < 0.0);
    return;
  } else if (points_.size() == 2) {
    float u, n, xd, yd, ua;
    u = -points_[0].u / dt_ * t + points_[0].u;
    ua = (u > 0.0) ? u : -u;
    xd = points_[1].x - points_[0].x;
    yd = points_[1].y - points_[0].y;
    n = sqrt(xd * xd + yd * yd);
    if (x) *x = xd / dt_ * t + points_[0].x;
    if (y) *y = yd / dt_ * t + points_[0].y;
    if (dx) *dx = ua * xd / n;
    if (dy) *dy = ua * yd / n;
    if (ddx) *ddx = 0.0;
    if (ddy) *ddy = 0.0;
    if (reverse) *reverse = (u < 0.0);
    return;
  }

  // Which part of the segment are we in?
  int part = 1;
  if (index == 0) {
    part = 0;
  } else if (index == points_.size() - 2) {
    part = 2;
  }

  // Do simple 3rd order interpolation (with correct boundary conditions)
  // Don't forget to scale both the speed and curvature.
  float a_x, b_x, c_x, d_x;
  float a_y, b_y, c_y, d_y;
  if (part == 0) {
    float d1x, d2x, d1y, d2y;
    float uindex1 = (points_[index + 1].u > 0.0) ? points_[index + 1].u : -points_[index + 1].u;

    float n = std::max(sqrt((points_[index+2].x - points_[index].x)*(points_[index+2].x - points_[index].x)+(points_[index+2].y - points_[index].y) * (points_[index+2].y - points_[index].y)), kEpsilon);
    float kx = (points_[index + 2].x - points_[index].x) / n * uindex1 * dt_;
    float ky = (points_[index + 2].y - points_[index].y) / n * uindex1 * dt_;

    d1x = points_[index].u * dt_;
    d1y = 0.0;
    d2x = kx;
    d2y = ky;

    a_x = points_[index].x;
    b_x = d1x;
    c_x = 3.0 * (points_[index + 1].x - points_[index].x) - 2.0 * d1x - d2x;
    d_x = 2.0 * (points_[index].x - points_[index + 1].x) + d1x + d2x;

    a_y = points_[index].y;
    b_y = d1y;
    c_y = 3.0 * (points_[index + 1].y - points_[index].y) - 2.0 * d1y - d2y;
    d_y = 2.0 * (points_[index].y - points_[index + 1].y) + d1y + d2y;
  } else if (part == 2) {
    float d2x, d3x, d2y, d3y;
    float uindex = (points_[index].u > 0.0) ? points_[index].u : -points_[index].u;

    float n = std::max(sqrt((points_[index + 1].x - points_[index - 1].x) * (points_[index + 1].x - points_[index - 1].x) + (points_[index + 1].y - points_[index - 1].y) * (points_[index + 1].y - points_[index - 1].y)), kEpsilon);
    float kx = (points_[index + 1].x - points_[index - 1].x) / n * uindex * dt_;
    float ky = (points_[index + 1].y - points_[index - 1].y) / n * uindex * dt_;

    d2x = kx; d2y = ky;
    d3x = 0.0; d3y = 0.0;

    a_x = points_[index].x;
    b_x = d2x;
    c_x = 3.0 * (points_[index + 1].x - points_[index].x) - 2.0 * d2x - d3x;
    d_x = 2.0 * (points_[index].x - points_[index + 1].x) + d2x + d3x;

    a_y = points_[index].y;
    b_y = d2y;
    c_y = 3.0 * (points_[index + 1].y - points_[index].y) - 2.0 * d2y - d3y;
    d_y = 2.0 * (points_[index].y - points_[index + 1].y) + d2y + d3y;
  } else {
    float d2x, d3x, d2y, d3y;
    float uindex = (points_[index].u > 0.0) ? points_[index].u : -points_[index].u;
    float uindex1 = (points_[index + 1].u > 0.0) ? points_[index + 1].u : -points_[index + 1].u;

    float n1 = std::max(sqrt((points_[index + 1].x-points_[index - 1].x)*(points_[index + 1].x-points_[index - 1].x)+(points_[index + 1].y-points_[index - 1].y)*(points_[index + 1].y-points_[index - 1].y)), kEpsilon);
    float k1x = (points_[index + 1].x - points_[index - 1].x) / n1 * uindex * dt_;
    float k1y = (points_[index + 1].y - points_[index - 1].y) / n1 * uindex * dt_;

    float n2 = std::max(sqrt((points_[index + 2].x - points_[index].x) * (points_[index + 2].x - points_[index].x) + (points_[index + 2].y - points_[index].y) * (points_[index + 2].y - points_[index].y)), 0.0001);
    float k2x = (points_[index + 2].x - points_[index].x) / n2 * uindex1 * dt_;
    float k2y = (points_[index + 2].y - points_[index].y) / n2 * uindex1 * dt_;

    d2x = k1x; d2y = k1y;
    d3x = k2x; d3y = k2y;

    a_x = points_[index].x;
    b_x = d2x;
    c_x = 3.0 * (points_[index + 1].x - points_[index].x) - 2.0 * d2x - d3x;
    d_x = 2.0 * (points_[index].x - points_[index + 1].x) + d2x + d3x;

    a_y = points_[index].y;
    b_y = d2y;
    c_y = 3.0 * (points_[index + 1].y - points_[index].y) - 2.0 * d2y - d3y;
    d_y = 2.0 * (points_[index].y - points_[index + 1].y) + d2y + d3y;
  }

  t = (t - (float)index * dt_) / dt_;
  if (x) *x = a_x + b_x * t + c_x * t * t + d_x * t * t * t;
  if (y) *y = a_y + b_y * t + c_y * t * t + d_y * t * t * t;
  if (dx) *dx = (b_x + 2.0 * c_x * t + 3.0 * d_x * t * t) / dt_;
  if (dy) *dy = (b_y + 2.0 * c_y * t + 3.0 * d_y * t * t) / dt_;
  if (ddx) *ddx = (2.0 * c_x + 6.0*d_x * t) / dt_ / dt_;
  if (ddy) *ddy = (2.0 * c_y + 6.0*d_y * t) / dt_ / dt_;
  if (reverse) *reverse = (points_[index].u < 0.0);
}
