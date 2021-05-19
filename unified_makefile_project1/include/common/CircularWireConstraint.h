#pragma once

#include "Particle.h"
#include <gfx/mat2.h>

class CircularWireConstraint {
 public:
  CircularWireConstraint(Particle *p, const Vec2f & center, const double radius);

  void draw();
  void apply_constraint();


 private:

  Particle * const m_p;
  Vec2f const m_center;
  double const m_radius;
  Mat2 MatVec_tMult(Mat2 m, Vec2 v);
  Mat2 inverse(Mat2 m);
};
