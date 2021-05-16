#include "CircularWireConstraint.h"
#include <GL/glut.h>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <gfx/mat2.h>

#define PI 3.1415926535897932384626433832795

static void draw_circle(const Vec2f & vect, float radius)
{
	glBegin(GL_LINE_LOOP);
	glColor3f(0.0,1.0,0.0); 
	for (int i=0; i<360; i=i+18)
	{
		float degInRad = i*PI/180;
		glVertex2f(vect[0]+cos(degInRad)*radius,vect[1]+sin(degInRad)*radius);
	}
	glEnd();
}

CircularWireConstraint::CircularWireConstraint(Particle *p, const Vec2f & center, const double radius) :
	m_p(p), m_center(center), m_radius(radius) {


}

void CircularWireConstraint::apply_constraint()
{
    const int dimensions = 2;
    //int vectorLength = pVector.size() * dimensions;
    int constraintsLength = 1; // cVector.size();

    Vec2f q = Vec2f(0.0, 0.0);
    Vec2f Q = Vec2f(0.0, 0.0);
    Mat2 W = Mat2();

//    Vec2f C = Vec2f(0.0, 0.0);
//    VectorXf Cder = VectorXf::Zero(constraintsLength);
//    MatrixXf J = MatrixXf::Zero(constraintsLength, dimensions);
//    MatrixXf Jt = MatrixXf::Zero(dimensions, constraintsLength);
//    MatrixXf Jder = MatrixXf::Zero(constraintsLength, dimensions);

    for (int d = 0; d < dimensions; d++) {
        W(d, d) = 1 / m_p->m_Mass;
        Q[d] = m_p->m_Force[d];
        q[d] = m_p -> m_Velocity[d];
    }

    Vec2f diff = m_p->m_Position - m_center;

    float c = pow(diff[0], 2) + pow(diff[1], 2) - pow(m_radius, 2);

    float c_derivative = 2 * diff * m_p->m_Velocity;

    Vec2f J = diff * 2;

    Vec2f J_derivative = m_p->m_Velocity * 2;

    Mat2 JW = Mat2(J[0]*W[0][0] + J[0]*W[0][1], J[0]*W[1][0] + J[0]*W[1][1],
                   J[1]*W[0][0] + J[1]*W[0][1], J[1]*W[1][0] + J[1]*W[1][1]);

//    Mat2 JWJ_t = Mat2(JW[0] * J[0],  JW[1] * J[0], //j * j_transposed
//                      JW[0] * J[1], JW[1] * J[1]);

    Mat2 JWJ_t = MatVec_tMult(JW, J);

    Vec2f J_dq = J_derivative * q;

    Mat2 JWQ_t = MatVec_tMult(JW, Q);

    Vec2f lambda_t = invert(JWJ_t) * (-J_dq - JWQ_t);

//    float p_r2 = pow(m_p->m_Force[0],2) + pow(m_p->m_Force[0],2);
//    float r_scale = pow(m_radius, 2) / p_r2;
//
//    m_p->m_Force[0] = sqrt(pow(m_p->m_Force[0],2) / r_scale);
//    m_p->m_Force[1] = sqrt(pow(m_p->m_Force[1],2) / r_scale);

}

Mat2 CircularWireConstrainst::MatVec_tMult(Mat2 m, Vec2 v) {
    return Mat2(m[0][0] * v[0] + m[1][0] * v[0], m[0][0] * v[1] + m[1][0] * v[1],
                m[0][1] * v[0] + m[1][1] * v[0], m[0][1] * v[1] + m[1][1] * v[1]);
}

void CircularWireConstraint::draw()
{
	draw_circle(m_center, m_radius);
}
