#include "default_flying_robot.h"

#include <cmath>
#include <iostream>

#include "util/utils.h"

namespace {
constexpr double kDistanceThreshold = 0.2;
constexpr double kGoalRadius = 0.05;
}  // namespace

bool DefaultFlyingRobot::Initialize() {
  // Pick a random goal point .
  goal_x_ = RandomUniform() * 3.0 - 1.5;
  goal_y_ = RandomUniform() * 3.0 - 1.5;
  goal_z_ = RandomUniform() * 1.0 + 1.0;
  return true;
}

void DefaultFlyingRobot::Execute(double t, double dt) {
  double dx = goal_x_ - x();
  double dy = goal_y_ - y();
  double dz = goal_z_ - z();
  double e = sqrtf(dx * dx + dy * dy + dz * dz);
  while (e < kDistanceThreshold) {
    goal_x_ = RandomUniform() * 3.0 - 1.5;
    goal_y_ = RandomUniform() * 3.0 - 1.5;
    goal_z_ = RandomUniform() * 1.0 + 1.0;
    dx = goal_x_ - x();
    dy = goal_y_ - y();
    dz = goal_z_ - z();
    e = sqrtf(dx * dx + dy * dy + dz * dz);
  }
  // Go to goal point.
  SetGoal(goal_x_, goal_y_, goal_z_);
}

void DefaultFlyingRobot::Draw(VisualizerWindow* window) {
  FlyingRobot::Draw(window);

  glPushMatrix();
  glTranslatef(goal_x_, goal_z_, -goal_y_);
  auto robot_color = color();
  glColor4f(std::get<0>(robot_color) * 0.5, std::get<1>(robot_color) * 0.5, std::get<2>(robot_color) * 0.5, 0.6f);
  glutWireSphere(kGoalRadius, 8, 8);
  glPopMatrix();
}
