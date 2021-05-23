#include "Particle.h"
#include <vector>
#include <GL/glut.h>

Particle::Particle(const Vec2f & ConstructPos, float mass) :
	m_ConstructPos(ConstructPos), m_Position(ConstructPos),
	m_Velocity(Vec2f(0.0, 0.0)), m_Force(Vec2f(0.0, 0.0)), m_Mass(mass)
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

void Particle::clear_force()
{
    m_Force = Vec2f(0.0, 0.0);
}

Vec2f Particle::get_state()
{
    return NULL; //Vec2f(m_Position, m_Velocity);
}

void Particle::set_state(Vec2f pos, Vec2f vel)
{
    m_Position = pos;
    m_Velocity = vel;
}

std::vector<Vec2f> Particle::derive_eval() { // returns a vector of the velocity and the acceleration
    std::vector<Vec2f> eval;
    eval.push_back(m_Velocity);
    eval.push_back(m_Force / m_Mass);
    return eval;
}

void Particle::compute_velocity() {
    std::vector<Vec2f> eval = derive_eval();
    m_Velocity += eval[1];
    //these lines aim to remove that the forces go to inf and leave the window
    m_Velocity[0] = std::min(m_Velocity[0], 0.1f);
    m_Velocity[1] = std::min(m_Velocity[1], 0.1f);
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
