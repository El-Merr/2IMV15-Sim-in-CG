#include "CircularWireConstraint.h"
#include <GL/glut.h>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <gfx/mat2.h>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>

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

CircularWireConstraint::CircularWireConstraint(Particle* p, const Vec2f & center, const double radius) :
	m_p(p), m_center(center), m_radius(radius) {
}

CircularWireConstraint::~CircularWireConstraint(void) {}

std::vector<Particle*> CircularWireConstraint::getParticles()
{
    std::vector<Particle*> particles;
    particles.push_back(m_p);
    return particles;
}


void CircularWireConstraint::draw()
{
	draw_circle(m_center, m_radius);
}

float CircularWireConstraint::constraint()
{
    Vec2f pos_diff = m_p->m_Position - m_center;
    return pow(pos_diff[0], 2) + pow(pos_diff[1], 2) - pow(m_radius, 2);
}

float CircularWireConstraint::constraint_derivative()
{
    Vec2f pos_diff = m_p->m_Position - m_center;
    return 2 * pos_diff * m_p->m_Velocity;
}

std::vector<Vec2f> CircularWireConstraint::J()
{
    Vec2f pos_diff = m_p->m_Position - m_center;
    Vec2f j = pos_diff * 2;
    std::vector<Vec2f> res;
    res.push_back(j);
    return res;
}

std::vector<Vec2f> CircularWireConstraint::J_derivative()
{
    Vec2f j_deriv = m_p->m_Velocity * 2;
    std::vector<Vec2f> res;
    res.push_back(j_deriv);
    return res;
}