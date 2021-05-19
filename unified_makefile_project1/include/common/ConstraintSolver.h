//
// Created by 20174019 on 18/05/2021.
//
#pragma once
#include "Particle.h"
#include "Constraint.h"
#include <Eigen/Dense>
#include "Eigen/IterativeLinearSolvers"

#ifndef UNIFIED_MAKEFILE_PROJECT1_CONSTRAINTSOLVER_H
#define UNIFIED_MAKEFILE_PROJECT1_CONSTRAINTSOLVER_H



class ConstraintSolver {
public:
    ConstraintSolver(std::vector<Particle*> pVector, std::vector<Constraint*> cVector);

//    void add_constraint();
    void apply_constraint();

private:
    std::vector<Particle*> m_pVector;
    std::vector<Constraint*> m_cVector;

};


#endif //UNIFIED_MAKEFILE_PROJECT1_CONSTRAINTSOLVER_H
