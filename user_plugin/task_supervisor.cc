#include "task_supervisor.h"


#include <math.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "csv_parser.h"
#include "gflags/gflags.h"
#include "path.h"
#include "task_robot.h"
#include "util/utils.h"

DEFINE_string(trajectories, "", "CSV file of trajectories.");
DEFINE_string(tasks, "", "CSV file of task locations.");
DEFINE_string(species, "", "CSV file of species traits.");

DEFINE_double(hide_colors_until, -1.0, "All robots and tasks appear gray until this many seconds into the run. Set to -1 to have them colorful all the time.");

DEFINE_bool(task_supervisor_axes, false, "Draw axes if desired.");

DECLARE_string(robot);

namespace {
// Control constants.
constexpr float kTaskHeight = 0.4f;
constexpr float kMaxTraitHeight = 3.0f;
constexpr double kExtraRadius = 0.5;
constexpr double kWidenTaskRadius = 1.2;

bool ReadTrajectoryFile(std::unordered_map<int, int>* species, std::unordered_map<int, Path>* trajectories) {
  species->clear();
  trajectories->clear();
  // Read input trajectory file.
  float delta_t = -1.0f;
  if (FLAGS_trajectories.empty()) {
    std::cerr << "--trajectories must be specified." << std::endl;
    return false;
  } else {
    // Format is: time, robot_id, species_id, x, y
    // Assumes that for each robot, timesteps are consecutive and start at 0.
    CSVParser parser;
    parser.Open(FLAGS_trajectories);
    while (parser.Next()) {
      std::vector<float> fields = parser.NumberFields();
      if (fields.size() != 5) continue;
      int robot_id = (int)fields[1];
      // Update species.
      (*species)[robot_id] = (int)fields[2];
      // Update trajectory (set velocity later on).
      double x = fields[3];
      double y = fields[4];
      (*trajectories)[robot_id].AddPoint({x, y, 0.0});
      // Update delta_t.
      if (delta_t < 0.0 && fields[0] > 0.0) {
        delta_t = fields[0];
      }
    }
    parser.Close();
  }
  if (trajectories->empty()) {
    std::cerr << "Trajectories file does not contain any robots (or has wrong format)." << std::endl;
    return false;
  }
  for (auto it = trajectories->begin(); it != trajectories->end(); ++it) {
    it->second.AdjustVelocityAutomatically(delta_t);
  }
  return true;
}

bool ReadTaskFile(std::vector<TaskInformation>* tasks) {
  tasks->clear();
  if (FLAGS_tasks.empty()) {
    std::cerr << "--tasks must be specified." << std::endl;
    return false;
  } else {
    // Format is: task_id, x, y, radius, trait_1, trait_2,...
    CSVParser parser;
    parser.Open(FLAGS_tasks);
    while (parser.Next()) {
      std::vector<float> fields = parser.NumberFields();
      if (fields.size() < 5) continue;
      std::vector<double> traits(fields.begin() + 4, fields.end());
      tasks->push_back({fields[1], fields[2], fields[3] + kExtraRadius, traits, std::vector<double>(traits.size())});
    }
    parser.Close();
  }
  if (tasks->empty()) {
    std::cerr << "Task file does not contain any tasks (or has wrong format)." << std::endl;
    return false;
  }
  return true;
}

bool ReadSpeciesFile(std::unordered_map<int, std::vector<bool>>* species_traits) {
  species_traits->clear();
  if (FLAGS_species.empty()) {
    std::cerr << "--species must be specified." << std::endl;
    return false;
  } else {
    // Format is: species_ids, trait_1, trait_2,...
    CSVParser parser;
    parser.Open(FLAGS_species);
    while (parser.Next()) {
      std::vector<float> fields = parser.NumberFields();
      if (fields.size() < 2) continue;
      std::vector<bool> traits(fields.begin() + 1, fields.end());
      (*species_traits)[(int)fields[0]] = traits;
    }
    parser.Close();
  }
  if (species_traits->empty()) {
    std::cerr << "Species file does not contain any entries (or has wrong format)." << std::endl;
    return false;
  }
  return true;
}

void DrawSolidCylinder(float diameter, float height) {
  glPushMatrix();
  glRotatef(90.0, 1.0, 0.0, 0.0);
  glTranslatef(0.0, 0.0, -height / 2.0);
  GLUquadricObj* qobj;
  qobj = gluNewQuadric();
  gluQuadricDrawStyle(qobj, GLU_FILL);
  gluCylinder(qobj, diameter / 2.0, diameter / 2.0, height, 32, 1);
  gluDeleteQuadric(qobj);
  qobj = gluNewQuadric();
  gluQuadricDrawStyle(qobj, GLU_FILL);
  gluQuadricOrientation(qobj, GLU_INSIDE);
  gluDisk(qobj, 0, diameter / 2.0, 32, 5);
  gluDeleteQuadric(qobj);
  glPopMatrix();
}

void DrawWireCylinder(float diameter, float height) {
  glPushMatrix();
  glRotatef(90.0, 1.0, 0.0, 0.0);
  glTranslatef(0.0, 0.0, -height / 2.0);
  GLUquadricObj* qobj;
  qobj = gluNewQuadric();
  gluQuadricDrawStyle(qobj, GLU_LINE);
  gluCylinder(qobj, diameter / 2.0, diameter / 2.0, height, 32, 1);
  gluDeleteQuadric(qobj);
  glPopMatrix();
}

void RenderBitmapString(float x, float y, float z, void* font, char* str)
{
  char* c;
  glRasterPos3f(x, y, z);
  for (c = str; *c != '\0'; c++) {
    glutBitmapCharacter(font, *c);
  }
}

}  // namespace

TaskSupervisor::TaskSupervisor() : num_traits_(0), num_species_(0), camera_adjusted_(false) {}

bool TaskSupervisor::Initialize() {
  // Read trajectory file.
  std::unordered_map<int, int> species;
  std::unordered_map<int, Path> trajectories;
  if (!ReadTrajectoryFile(&species, &trajectories)) {
    return false;
  }

  // Create robots.
  std::cout << "Number of robots: " << trajectories.size() << std::endl;
  std::unordered_set<int> unique_species;
  for (const auto& kv_pair : trajectories) {
    Robot* robot = CreateRobot("TaskRobot");
    robot->SetPosition(kv_pair.second.GetInitialX(), kv_pair.second.GetInitialY(), kv_pair.second.GetInitialAngle());
    robot->SetType(species[kv_pair.first]);
    unique_species.insert(species[kv_pair.first]);
    dynamic_cast<TaskRobot*>(robot)->SetPath(kv_pair.second);
  }
  for (int i = 0; i < unique_species.size(); ++i) {
    if (unique_species.find(i) == unique_species.end()) {
      std::cerr << "Species provided must be contiguous." << std::endl;
      return false;
    }
    num_species_ = unique_species.size();
  }

  // Read task file.
  if (!ReadTaskFile(&tasks_)) {
    return false;
  }
  // There is a guarantee to be at least one task.
  trait_colors_.clear();
  RandomColors colors(0.3);
  num_traits_ = tasks_[0].desired_traits.size();
  for (int i = 0; i < num_traits_; ++i) {
    trait_colors_.push_back(colors.Next());
  }

  // Read species file.
  if (!ReadSpeciesFile(&species_traits_)) {
    return false;
  }

  return true;
}

void TaskSupervisor::Update(double t, double dt) {
  // Update robot positions for NN and tasks
  for (auto it = tasks_.begin(); it != tasks_.end(); ++it) {
    for (int i = 0; i < num_traits_; ++i) {
      it->current_traits[i] = 0.0;
    }
  }
  for (int i = 0; i < NumRobots(); ++i) {
    const Robot& robot = GetRobot(i);
    // Find the closest task.
    int task_index = -1;
    for (int j = 0; j < tasks_.size(); ++j) {
      double dist = sqrt((robot.x() - tasks_[j].x) * (robot.x() - tasks_[j].x) + (robot.y() - tasks_[j].y) * (robot.y() - tasks_[j].y));
      if (dist < tasks_[j].radius * kWidenTaskRadius) {
        task_index = j;
        break;
      }
    }
    if (task_index >= 0) {
      for (int j = 0; j < num_traits_; j++) {
        tasks_[task_index].current_traits[j] += species_traits_[robot.type()][j];
      }
    }
  }
}

void TaskSupervisor::Draw(double t, VisualizerWindow* window) {
  // Axes.
  if (FLAGS_task_supervisor_axes) {
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.005f, 0.0f);
    glVertex3f(1.0f, 0.005f, 0.0f);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.005f, 0.0f);
    glVertex3f(0.0f, 0.005f, -1.0f);
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 1.005f, 0.0f);
    glVertex3f(0.0f, 0.005f, 0.0f);
    glEnd();
  }

  // Adjust camera position based on number of robots.
  if (!camera_adjusted_) {
    camera_adjusted_ = true;
    double sqrt_robots = sqrt((double)NumRobots());
    window->SetCameraParameters(-1.7, 0.0, 1.7, sqrt_robots + 10.0, 0.63, -0.9);
  }

  // Draw tasks.
  glEnable(GL_BLEND);
  glDepthMask(GL_FALSE);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  for (const auto& task : tasks_) {
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslatef(task.x, kTaskHeight / 2.0f, -task.y);
    glScalef(task.radius * 2.0f, kTaskHeight, task.radius * 2.0f);
    glColor4f(1.0f, 1.0f, 1.0f, 0.6f);
    DrawSolidCylinder(1.0, 1.0);
    glColor3f(1.0f, 1.0f, 1.0f);
    DrawWireCylinder(1.0, 1.0);
    glDepthMask(GL_TRUE);
    glDisable(GL_LIGHTING);
    for (int j = 0; j < num_traits_; ++j) {
      float height = (task.current_traits[j] + 0.01) / NumRobots() * kMaxTraitHeight * task.radius / kTaskHeight;
      glPushMatrix();
      glTranslatef(-0.9 / 2.0 + (0.9 / 2.0 / num_traits_) + (0.9 / num_traits_) * j, height / 2.0 + 0.5, 0.0);
      glScalef(0.9 / num_traits_, height, 0.9 / num_traits_);
      if (FLAGS_hide_colors_until >= 0.0 && t < FLAGS_hide_colors_until) {
        glColor3f(0.5, 0.5, 0.5);
      } else {
        glColor3f(std::get<0>(trait_colors_[j]), std::get<1>(trait_colors_[j]), std::get<2>(trait_colors_[j]));
      }
      glutSolidCube(1.0);
      glPopMatrix();
      glPushMatrix();
      height = (task.desired_traits[j] + 0.01) / NumRobots() * kMaxTraitHeight * task.radius / kTaskHeight;
      glTranslatef(-0.9 / 2.0 + (0.9 / 2.0 / num_traits_) + (0.9 / num_traits_) * j, height / 2.0 + 0.5, 0.0);
      glScalef(0.9 / num_traits_, height, 0.9 / num_traits_);
      glColor3f(1.0, 1.0, 1.0);
      glutWireCube(1.0);
      glPopMatrix();
    }
    glEnable(GL_LIGHTING);
    glDepthMask(GL_FALSE);
    glPopMatrix();
  }
  glDepthMask(GL_TRUE);
  glDisable(GL_BLEND);

  // Draw legend.
  glDisable(GL_LIGHTING);
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0, window->w(), 0, window->h());
  glScalef(1, -1, 1);
  glTranslatef(0, -window->h(), 0);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  char str[30];
  const std::unordered_map<int, std::tuple<float, float, float>>& species_colors = RobotColors();
  for (int i = 0; i < num_species_; ++i) {
    if (FLAGS_hide_colors_until >= 0.0 && t < FLAGS_hide_colors_until) {
      glColor3f(0.5, 0.5, 0.5);
    } else {
      glColor3f(std::get<0>(species_colors.at(i)), std::get<1>(species_colors.at(i)), std::get<2>(species_colors.at(i)));
    }
    snprintf(str, 30, "Species %d:", i);
    RenderBitmapString(window->w()-100 - num_traits_ * 20, 25 * (i + 1), 0, GLUT_BITMAP_HELVETICA_18, str);
    for (int j = 0; j < num_traits_; j++) {
      if (species_traits_[i][j]) {
        if (FLAGS_hide_colors_until >= 0.0 && t < FLAGS_hide_colors_until) {
          glColor3f(0.5, 0.5, 0.5);
        } else {
          glColor3f(std::get<0>(trait_colors_[j]), std::get<1>(trait_colors_[j]), std::get<2>(trait_colors_[j]));
        }
        glBegin(GL_QUADS);
        glVertex2f(window->w() + 15 - (num_traits_ - j) * 20, 25 * (i + 1) - 20 + 3);
        glVertex2f(window->w() + 15  - (num_traits_ - j + 1) * 20, 25 * (i + 1) - 20 + 3);
        glVertex2f(window->w() + 15  - (num_traits_ - j + 1) * 20, 25 * (i + 1) + 3);
        glVertex2f(window->w() + 15  - (num_traits_ - j) * 20, 25 * (i + 1) + 3);
        glEnd();
      }
    }
  }
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glEnable(GL_LIGHTING);
}

