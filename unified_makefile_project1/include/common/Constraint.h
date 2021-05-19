//
// Created by 20174019 on 19/05/2021.
//

#ifndef UNIFIED_MAKEFILE_PROJECT1_CONSTRAINT_H
#define UNIFIED_MAKEFILE_PROJECT1_CONSTRAINT_H

#include "vector"

class Constraint {
protected:
    vector<Particle*> particles;

public:
    virtual void draw();
    virtual float constraint();
    virtual float constraint_derivative();
    virtual Vec2f J();
    virtual Vec2f J_derivative();

};


#endif //UNIFIED_MAKEFILE_PROJECT1_CONSTRAINT_H
