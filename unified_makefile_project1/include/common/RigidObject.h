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

        RigidObject(std::vector<Particle*> particles);
        virtual ~RigidObject(void);
        void draw_object() override;
        std::vector<Vec2f> get_points() override;

        void set_state(VectorXf state);
        VectorXf get_state();
        VectorXf derive_eval();

        void clear_force();
        void reset();

        std::vector<Particle*> pVector;
        double M;               // Mass
        Vector2f position;
        Vector2f velocity;
        Vector2f force;

    private:

        Vector2f construct_position;
        Matrix2f R;             // rotation

        // tensor of Inertia
        Matrix2f I_body;
        Matrix2f I_body_inv;
        Matrix2f I_inverse;

        // linear
        Vector2f P;             // linear momentum

        // angular
        float torque;           // angular force
        float omega;            // angular velocity w(t)
        float L;                // angular momentum

        void calc_center_of_mass();
        void calc_force_and_torque();
        void calc_aux_variables();
        Vector2f vec_to_Eigen( Vec2f v );

        float crossProduct( const Vector2f& a, const Vector2f& b );
};


