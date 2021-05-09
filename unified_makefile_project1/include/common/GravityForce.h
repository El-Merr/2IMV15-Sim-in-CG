#pragma once
#include "Particle.h"

#define G 0.00001f

class GravityForce {

    public:
        GravityForce(Particle *p);
        void draw();

    private:

        Particle * const m_p;   // particle
        float m_Gravity;
};



