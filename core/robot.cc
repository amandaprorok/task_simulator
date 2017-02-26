#include "robot.h"

Robot::Robot()
    : x_(0.0), y_(0.0), z_(0.0), yaw_(0.0), roll_(0.0), pitch_(0.0), type_(0) {}

bool Robot::Init() {
  return Initialize();
}

void Robot::Step(double t, double dt) {
  Execute(t, dt);
}

void Robot::SetPosition(double x, double y, double yaw) {
  x_ = x;
  y_ = y;
  yaw_ = yaw;
}

void Robot::SetPosition(double x, double y, double z, double roll, double pitch, double yaw) {
  x_ = x;
  y_ = y;
  z_ = z;
  yaw_ = yaw;
  pitch_ = pitch;
  roll_ = roll;
}

void Robot::ClearNeighbors() {
  neighbors_.clear();
}

void Robot::AddNeighbor(const Robot* neighbor) {
  neighbors_.push_back(neighbor);
}

void Robot::SetType(int type) {
  type_ = type;
}

void Robot::SetColor(const std::tuple<float, float, float>& color) {
  color_ = color;
}
