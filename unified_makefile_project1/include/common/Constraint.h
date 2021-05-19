//
// Created by 20174019 on 19/05/2021.
//

#ifndef UNIFIED_MAKEFILE_PROJECT1_CONSTRAINT_H
#define UNIFIED_MAKEFILE_PROJECT1_CONSTRAINT_H

#include <vector>
#include "gfx/vec2.h"
#include "Particle.h"

class Constraint {
protected:
//    std::vector<Particle*> pVector;

public:
//    Constraint(std::vector<Particle*> pVector): pVector(pVector) {};

    virtual void draw();
    virtual float constraint();
    virtual float constraint_derivative();
    virtual std::vector<Vec2f> J();
    virtual std::vector<Vec2f> J_derivative();

};


#endif //UNIFIED_MAKEFILE_PROJECT1_CONSTRAINT_H
