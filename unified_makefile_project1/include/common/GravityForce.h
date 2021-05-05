#pragma once
#include "Particle.h"

class GravityForce {

    public:
        GravityForce(Particle *p);
        void draw();

    private:

        Particle * const m_p;   // particle
        double G; // gravitation constant (9.8)
};



