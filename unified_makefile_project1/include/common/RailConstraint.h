//
// Created by 20174019 on 24/05/2021.
//

#pragma once

#include "Particle.h"
#include "gfx/vec2.h"
#include <Eigen/Dense>
#include "Eigen/IterativeLinearSolvers"
#include "Constraint.h"

#include <vector>

using namespace Eigen;

class RailConstraint : public Constraint {
public:
    RailConstraint(Particle* p, const Vec2f & start, const Vec2f & end);

    void draw() override;
    std::vector<Particle*> getParticles() override;

    float constraint() override;
    float constraint_derivative() override;
    std::vector<Vec2f> J() override;
    std::vector<Vec2f> J_derivative() override;


private:

    Particle* const m_p;
    Vec2f const m_start;
    Vec2f const m_end;

    Vec2f get_rail_point();

};