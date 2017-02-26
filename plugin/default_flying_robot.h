#ifndef _DEFAULT_FLYING_ROBOT_H
#define _DEFAULT_FLYING_ROBOT_H

#include "core/flying_robot.h"
#include "core/robot.h"
#include "display/window.h"
#include "util/registerer.h"

class DefaultFlyingRobot : public FlyingRobot {
  REGISTER("DefaultFlyingRobot", Robot);

 public:
  void Draw(VisualizerWindow* window) override;

 private:
  bool Initialize() override;
  void Execute(double t, double dt) override;
  bool GoalBased() override { return true; }

  double goal_x_;
  double goal_y_;
  double goal_z_;
};

#endif
