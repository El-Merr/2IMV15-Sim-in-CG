#pragma once

#include "Particle.h"
#include "gfx/vec2.h"
#include <Eigen/Dense>
#include "Eigen/IterativeLinearSolvers"
#include "Constraint.h"

#include <vector>

using namespace Eigen;

class CircularWireConstraint : public Constraint {
 public:
  CircularWireConstraint(Particle* p, const Vec2f & center, const double radius);
  virtual ~CircularWireConstraint(void);

  void draw() override;
  std::vector<Particle*> getParticles() override;

    float constraint() override;
    float constraint_derivative() override;
    std::vector<Vec2f> J() override;
    std::vector<Vec2f> J_derivative() override;


 private:

  Particle * const m_p;
  Vec2f const m_center;
  double const m_radius;
};
