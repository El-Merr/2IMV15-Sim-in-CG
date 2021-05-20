#pragma once

#include "Particle.h"

class SpringForce {
 public:
  SpringForce(Particle * p1, Particle * p2, double dist, double ks, double kd);
 // SpringForce(MouseParticle *p1, Particle * p2, double dist, double ks, double kd);

  void draw();
  void applySpring();

 private:

  Particle * const m_p1;   // particle 1
 // MouseParticle m_mousep1; // mouse particle
  Particle * const m_p2;   // particle 2 
  double const m_dist;     // rest length
  double const m_ks, m_kd; // spring strength constants
};
