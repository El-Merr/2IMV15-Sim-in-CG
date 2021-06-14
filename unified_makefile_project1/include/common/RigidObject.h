#pragma once
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <gfx/vec2.h>

class RigidObject {
    public:
        RigidObject(std::vector<Vec2f> pointsVector);
        virtual ~RigidObject(void);

        std::vector<Vec2f> get_points();
        void DrawRigidObject();

        Vec2f position;

    private:
        std::vector<Vec2f> points;
};


