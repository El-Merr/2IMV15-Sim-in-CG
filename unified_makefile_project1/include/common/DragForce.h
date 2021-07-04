#pragma once

#include "RigidObject.h"
#include "Particle.h"
#include <Eigen/Dense>

class DragForce {
 public:
  DragForce(Particle * p1, Particle * p2, RigidObject * rb, double dist, double ks, double kd);
  virtual ~DragForce(void);
  void draw();
  void apply_spring();

 private:

  Particle * const m_p1;   // A particle at mouse position to attach the drag to
  Particle * const m_p2;    // The particle from the object to drag
  RigidObject * const m_rb;     // rigid object that is being dragged
  double const m_dist;     // rest length
  double const m_ks, m_kd; // spring strength constants
};
