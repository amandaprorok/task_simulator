#ifndef _PATH_H
#define _PATH_H

#include <vector>

struct PathPoint {
  double x;
  double y;
  double u;
};

class Path {
 public:
  void SetTimeSpacing(double dt);
  double dt() const { return dt_; }
  void Clear();
  void AddPoint(const PathPoint& point);
  void AdjustVelocityAutomatically(double dt);
  double GetInitialAngle() const;
  double GetInitialX() const;
  double GetInitialY() const;
  void Get(double t, double* x, double* y, double* dx, double* dy, double* ddx, double* ddy, bool* reverse) const;

 private:
  double dt_;
  std::vector<PathPoint> points_;
};

#endif
