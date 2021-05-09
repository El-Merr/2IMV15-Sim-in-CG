#include "CircularWireConstraint.h"
#include <GL/glut.h>

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
	m_p(p), m_center(center), m_radius(radius) {}

void CircularWireConstraint::draw()
{
    float p_r2 = pow(m_p->m_Position[0],2) + pow(m_p->m_Position[0],2);
    float r_scale = pow(m_radius, 2) / p_r2;

    m_p->m_Position[0] = sqrt(pow(m_p->m_Position[0],2) / r_scale);
    m_p->m_Position[1] = sqrt(pow(m_p->m_Position[1],2) / r_scale);

	draw_circle(m_center, m_radius);
}
