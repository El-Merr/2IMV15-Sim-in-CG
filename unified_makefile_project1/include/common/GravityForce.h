#pragma once
#include "Particle.h"

#define G 0.001f

class GravityForce {

    public:
        GravityForce(std::vector<Particle*> pVector);
        void draw();
        void apply_gravity();

    private:

        std::vector<Particle*> const m_pVec;   // particle
//        std::vector<float> m_Gravity;

        void draw_arrow(Particle *p, float g);
};



