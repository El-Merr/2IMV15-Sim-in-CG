#include "Particle.h"
#include <GL/glut.h>

Particle::Particle(const Vec2f & ConstructPos) :
	m_ConstructPos(ConstructPos), m_Position(Vec2f(0.0, 0.0)),
	m_Velocity(Vec2f(0.0, 0.0)), m_Force(Vec2f(0.0, 0.0)), m_Mass(1)
{
}

Particle::~Particle(void)
{
}

void Particle::reset()
{
	m_Position = m_ConstructPos;
	m_Velocity = Vec2f(0.0, 0.0);
	m_Force = Vec2f(0.0, 0.0);
	m_Mass = 1;
}

Vec2f Particle::getState()
{
    return NULL; //Vec2f(m_Position, m_Velocity);
}

Vec2f Particle::setState()
{
    return NULL; //Vec2f(m_Position, m_Velocity);
}

Vec2f Particle::deriveEval() { // returns a vector of the velocity and the acceleration
    return NULL; //Vec2f(m_Velocity, m_Force / m_Mass);
}

void Particle::draw()
{
	const double h = 0.03;
	glColor3f(1.f, 1.f, 1.f); 
	glBegin(GL_QUADS);
	glVertex2f(m_Position[0]-h/2.0, m_Position[1]-h/2.0);
	glVertex2f(m_Position[0]+h/2.0, m_Position[1]-h/2.0);
	glVertex2f(m_Position[0]+h/2.0, m_Position[1]+h/2.0);
	glVertex2f(m_Position[0]-h/2.0, m_Position[1]+h/2.0);
	glEnd();
}
