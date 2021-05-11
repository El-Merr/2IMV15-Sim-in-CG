#pragma once
#include "Particle.h"

#define G 1.0f

class GravityForce {

    public:
        GravityForce(Particle *p);
        void draw();

    private:

        Particle * const m_p;   // particle
        float m_Gravity;
};



