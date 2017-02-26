#ifndef _TASK_ROBOT_H
#define _TASK_ROBOT_H

#include "core/ground_robot.h"
#include "core/robot.h"
#include "path.h"
#include "util/registerer.h"

class TaskRobot : public GroundRobot {
  REGISTER("TaskRobot", Robot);

 public:
  void SetPath(const Path& path);

 private:
  bool Initialize() override;
  void Execute(double t, double dt) override;

  Path path_;

  bool first_execution_;
  std::tuple<float, float, float> original_color_;
};

#endif
