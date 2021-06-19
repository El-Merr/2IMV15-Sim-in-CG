#pragma once
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <gfx/vec2.h>
#include "Object.h"
#include "Particle.h"

class RigidObject : public Object {
    public:
        RigidObject(Vec2f pointsVector);
        virtual ~RigidObject(void);

        void draw_object() override;
        std::vector<Vec2f> get_points() override;

        std::vector<Vec2f> derive_eval();

        void set_state(Vec2f pos, Vec2f vel);
        std::vector<Vec2f> get_state();

        void clear_force();
        void reset();

    private:

        Particle* pCenter;
        std::vector<Particle*> pVector;

        /** spatial variables */
//        Vec2f m_center;                 // x (t)
//        Matrix2f m_Rotation;            // R (t)
//        Vec2f m_Velocity;               // x' (t)
//        Vector2f m_spin;                // w (t)
//
//        /** points in body space */
//        std::vector<Vec2f> m_points;    // p0
//        std::vector<Vector2f> p_body;   // p0 body space
//        std::vector<Vector2f> p_pos;
//        std::vector<Vector2f> p_velocity;   // v_i (t)
//
//        Vec2f m_Force;
//        Vector2f m_Torque;
//        float m_Mass;
//        std::vector<Vec2f> m_Direction;     // r (t)
//
//        Matrix3f m_I;

};


