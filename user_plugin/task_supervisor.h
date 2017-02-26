#ifndef _TASK_SUPERVISOR_H
#define _TASK_SUPERVISOR_H

#include <tuple>
#include <unordered_map>
#include <vector>

#include "core/supervisor.h"
#include "display/window.h"
#include "util/registerer.h"

struct TaskInformation {
  double x;
  double y;
  double radius;
  std::vector<double> desired_traits;
  std::vector<double> current_traits;
};

class TaskSupervisor : public Supervisor {
  REGISTER("TaskSupervisor", Supervisor);

 public:
  TaskSupervisor();

 private:
  bool Initialize() override;
  void Draw(double t, VisualizerWindow* window) override;
  void Update(double t, double dt) override;

  // Tasks.
  std::vector<TaskInformation> tasks_;

  // Species.
  std::unordered_map<int, std::vector<bool>> species_traits_;

  // Colors.
  int num_traits_;
  int num_species_;
  std::vector<std::tuple<float, float, float>> trait_colors_;

  // Adjust camera.
  bool camera_adjusted_;
};

#endif
