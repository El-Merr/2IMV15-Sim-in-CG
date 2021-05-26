//
// Created by 20173613 on 24/05/2021.
//
#pragma once

#include <gfx/vec2.h>
#include <vector>
#include "Particle.h"


class Wall {
public:
    Wall(const Vec2f & start, const Vec2f & end);
    virtual ~Wall(void);
    void draw();
    void detectCollision(std::vector<Particle*> pVector);
    void crudeCollision(std::vector<Particle*> pVector, int ii);
    void elasticCollision(std::vector<Particle*> pVector, int ii);
    bool crude;
private:
    Vec2f const m_start;
    Vec2f const m_end;
};

