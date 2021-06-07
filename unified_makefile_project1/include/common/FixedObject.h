#pragma once
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <gfx/vec2.h>

class FixedObject {
    public:
        FixedObject(std::vector<Vec2f> pointsVector);
        virtual ~FixedObject(void);
        void DrawFixedObject();
    private:
        std::vector<Vec2f> points;
};


