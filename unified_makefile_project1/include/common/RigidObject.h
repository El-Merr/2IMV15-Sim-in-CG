#pragma once
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <Eigen/Dense>
//#include <Eigen/IterativeLinearSolvers>
#include <gfx/vec2.h>
#include "Object.h"
#include "Particle.h"
#include <iostream>

using namespace Eigen;

class RigidObject : public Object {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        RigidObject(Vec2f pointsVector);
        virtual ~RigidObject(void);

        void draw_object() override;
        std::vector<Vec2f> get_points() override;

        VectorXf derive_eval();

//        void set_state(Vec2f pos, Vec2f vel);
//        std::vector<Vec2f> get_state();
        void set_state(VectorXf state);
        VectorXf get_state();
//        VectorXf get_ddt_state();

        void clear_force();
        void reset();

        void calc_force_and_torque();
        void calc_aux_variables();
        Vector2f vec_to_Eigen( Vec2f v );

//        Particle* pCenter;
        std::vector<Particle*> pVector;
        double M;               // Mass



    private:



        Vector2f position;
        Vector2f construct_position;
//
        Vector2f force;
        Vector2f torque;
        Matrix2f R;             // rotation
        Quaternionf q;

        Matrix2f I_body;
        Matrix2f I_body_inv;
        Matrix2f I_inverse;
//
        Vector2f velocity;
        Vector2f omega;         // angular velocity w(t)
        Vector2f P;             // linear momentum
        Vector2f L;             // angular momentum



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


