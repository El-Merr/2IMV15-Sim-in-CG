#pragma once
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <gfx/vec2.h>

class RigidObject {
    public:
        RigidObject(Vec2f pointsVector);
        virtual ~RigidObject(void);
        void clear_force();
        void reset();
        void computeBodySpace();
        std::vector<Vec2f> get_points();
        void drawRigidObject();
        std::vector<Vec2f> derive_eval();
        void set_state(Vec2f pos, Vec2f vel);
        std::vector<Vec2f> get_state();
        Vec2f center;
        Vec2f m_Velocity;
        Vec2f m_Force;
        float m_Mass;
        std::vector<Vec2f> m_Direction;
    private:
        std::vector<Vec2f> points;
};


