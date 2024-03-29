#pragma once
#include "Particle.h"

#define G 0.01f

class GravityForce {

    public:
        GravityForce(std::vector<Particle*> pVector);
        virtual ~GravityForce(void);

        void draw();
        void apply_gravity();
        bool drawArrows;

    private:

        std::vector<Particle*> const m_pVec;   // particle

        void draw_arrow(Particle *p, float g);
};



