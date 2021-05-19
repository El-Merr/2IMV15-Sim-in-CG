//
// Created by 20174019 on 18/05/2021.
//
#pragma once
#include "Particle.h"

#ifndef UNIFIED_MAKEFILE_PROJECT1_CONSTRAINTSOLVER_H
#define UNIFIED_MAKEFILE_PROJECT1_CONSTRAINTSOLVER_H



class ConstraintSolver {
public:
    ConstraintSolver(std::vector<Particle*> pVector);

    void add_constraint();

private:
    std::vector<Particle*> m_pVec;

};


#endif //UNIFIED_MAKEFILE_PROJECT1_CONSTRAINTSOLVER_H
