#include "Particle.h"
#include <vector>
#include <GL/glut.h>

Particle::Particle(const Vec2f & ConstructPos) :
	m_ConstructPos(ConstructPos), m_Position(Vec2f(0.0, 0.0)),
	m_Velocity(Vec2f(0.0, 0.0)), m_Force(Vec2f(0.0, 0.0)), m_Mass(0.01)
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
	m_Mass = 0.01;
}

Vec2f Particle::getState()
{
    return NULL; //Vec2f(m_Position, m_Velocity);
}

void Particle::setState(Vec2f pos, Vec2f vel)
{
    m_Position = pos;
    m_Velocity = vel;
}

std::vector<Vec2f> Particle::deriveEval() { // returns a vector of the velocity and the acceleration
    std::vector<Vec2f> eval;
    eval.push_back(m_Velocity);
    eval.push_back(m_Force / m_Mass);
    return eval;
}

void Particle::computeVelocity() {
    std::vector<Vec2f> eval = deriveEval();
    m_Velocity += eval[1];
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
