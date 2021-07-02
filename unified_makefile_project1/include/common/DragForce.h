#pragma once

#include "RigidObject.h"
#include "Particle.h"
#include <Eigen/Dense>

class DragForce {
 public:
  DragForce(Particle * p1, RigidObject * p2, double dist, double ks, double kd);
  virtual ~DragForce(void);
  void draw();
  void apply_spring();

 private:

  RigidObject * const m_p2;// The object to drag
  Particle * const m_p1;   // A particle at mouse position to attach the drag to
  double const m_dist;     // rest length
  double const m_ks, m_kd; // spring strength constants
};
