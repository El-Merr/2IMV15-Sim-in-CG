#pragma once

#include "Particle.h"
#include "Constraint.h"
#include "gfx/vec2.h"
#include <Eigen/Dense>
#include "Eigen/IterativeLinearSolvers"

#include <vector>

class RodConstraint : public Constraint {
 public:
  RodConstraint(Particle *p1, Particle * p2, double dist);

  void draw() override;
  std::vector<Particle*> getParticles() override;

    float constraint() override;
    float constraint_derivative() override;
    std::vector<Vec2f> J() override;
    std::vector<Vec2f> J_derivative() override;

 private:

  Particle * const m_p1;
  Particle * const m_p2;
  double const m_dist;
};
