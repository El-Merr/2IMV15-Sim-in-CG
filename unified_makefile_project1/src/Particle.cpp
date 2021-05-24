#include "Particle.h"
#include <vector>
#include <GL/glut.h>

static std::vector<Vec2f> integration;
static float counter;

Particle::Particle(const Vec2f & ConstructPos, float mass) :
	m_ConstructPos(ConstructPos), m_Position(ConstructPos),
	m_Velocity(Vec2f(0.0, 0.0)), m_Force(Vec2f(0.0, 0.0)), m_Mass(mass)
{
    integration.push_back(ConstructPos);
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
	integration.clear();
	counter = 0;
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
}

void Particle::compute_integration(float dt) {
    if (counter <= 0.0) {
        counter = H;
        Vec2f dest = m_Position + H * m_Velocity;
        integration.push_back(dest);
        if (integration.size() > MAX_SIZE) {
            integration.erase(integration.begin());
        }
//        printf("pos: %f %f\n", m_Position[0], m_Position[1]);
//        printf("vel: %f %f\n", m_Velocity[0], m_Velocity[1]);
//        printf("dest: %f %f\n", dest[0], dest[1]);
    }
    counter -= dt;
}

void Particle::draw_integration() {
    for (int i = 1; i<integration.size(); i++) {
        glBegin( GL_LINES );
        glColor3f(1.0, 1.0, 0.0);
        glVertex2f(integration[i-1][0], integration[i-1][1]);
        glColor3f(1.0, 1.0, 0.0);
        glVertex2f(integration[i][0], integration[i][1]);
        glEnd();
    }
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
