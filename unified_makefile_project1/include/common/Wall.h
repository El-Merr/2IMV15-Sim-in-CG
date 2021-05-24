//
// Created by 20173613 on 24/05/2021.
//
#pragma once

#include <gfx/vec2.h>
#include <vector>


class Wall {
public:
    Wall(float x1, float y1, float x2, float y2);
    virtual ~Wall(void);
    void draw();
private:
    Vec2f m_start;
    Vec2f m_end;
};

