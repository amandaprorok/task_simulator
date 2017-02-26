#ifndef _DEFAULT_GROUND_ROBOT_H
#define _DEFAULT_GROUND_ROBOT_H

#include "core/ground_robot.h"
#include "core/robot.h"
#include "display/window.h"
#include "util/registerer.h"

class DefaultGroundRobot : public GroundRobot {
  REGISTER("DefaultGroundRobot", Robot);

 public:
  void Draw(VisualizerWindow* window) override;

 private:
  bool Initialize() override;
  void Execute(double t, double dt) override;
  bool GoalBased() override { return true; }

  double goal_x_;
  double goal_y_;
};

#endif
