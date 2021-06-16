#pragma once
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <gfx/vec2.h>
#include "Object.h"

class FixedObject : public Object {
    public:
        FixedObject(std::vector<Vec2f> pointsVector);
        virtual ~FixedObject(void);

        std::vector<Vec2f> get_points() override;
        void draw_object() override;

    private:
        std::vector<Vec2f> points;
};


