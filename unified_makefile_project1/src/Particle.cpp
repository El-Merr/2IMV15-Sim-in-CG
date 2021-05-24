#include "Particle.h"
#include <vector>
#include <GL/glut.h>

//static std::vector<Vec2f> integration;
//static float counter;

Particle::Particle(const Vec2f & ConstructPos, float mass) :
	m_ConstructPos(ConstructPos), m_Position(ConstructPos),
	m_Velocity(Vec2f(0.0, 0.0)), m_Force(Vec2f(0.0, 0.0)), m_Mass(mass),
	m_Integration(std::vector<Vec2f>()), m_Counter(0)
{
    m_Integration.push_back(ConstructPos);
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
	m_Integration.clear();
	m_Counter = 0;
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

/**
 * Computes the numerical integration.
 * @param dt        The time step size of the simulation.
 * @param scheme    An integer denoting the integration scheme.
 */
void Particle::compute_integration(float dt, int scheme) {

    switch(scheme) {
        case 0: // Euler
            if (m_Counter <= 0.0) {
                m_Counter = H;
                compute_euler();
            }
            break;
        case 1: // Mid-point
            compute_midpoint();
            break;
        case 2: // Runge-Kutta 4
            compute_rungekutta();
            break;
    }

    // Limit the number of integration lines to avoid cluttering.
//    if (m_Integration.size() > MAX_SIZE) {
//        m_Integration.erase(m_Integration.begin());
//    }
//        printf("pos: %f %f\n", m_Position[0], m_Position[1]);
//        printf("vel: %f %f\n", m_Velocity[0], m_Velocity[1]);
//        printf("dest: %f %f\n", dest[0], dest[1]);
    m_Counter -= dt;
}

/**
 * Computes the Euler step.
 */
void Particle::compute_euler() {
    Vec2f dest = m_Position + H * m_Velocity;
    m_Integration[0] = m_Position;
    m_Integration[1] = dest;
}

/**
 * Computes the mid-point step.
 */
void Particle::compute_midpoint() {
    Vec2f dest = m_Position + H * m_Velocity;
    m_Integration.push_back(dest);
}

/**
 * Computes the Runge-Kutta 4 step.
 */
void Particle::compute_rungekutta() {

}

/**
 * Draws integration scheme lines.
 */
void Particle::draw_integration() {
//    for (int i = 1; i < m_Integration.size(); i++) {
//        glBegin(GL_LINES);
//        glColor3f(1.0, 1.0, 0.0);
//        glVertex2f(m_Integration[i - 1][0], m_Integration[i - 1][1]);
//        glColor3f(1.0, 1.0, 0.0);
//        glVertex2f(m_Integration[i][0], m_Integration[i][1]);
//        glEnd();
//    }
    const float dx = m_Integration[1][0] - m_Integration[0][0];
    const float dy = m_Integration[1][1] - m_Integration[0][1];
    const double arrow_dist = 0.3;

    glBegin(GL_LINES);
    glColor3f(1.0, 1.0, 0.0);
    glVertex2f(m_Integration[0][0], m_Integration[0][1]);
    glColor3f(1.0, 1.0, 0.0);
    glVertex2f(m_Integration[1][0], m_Integration[1][1]);
    glEnd();

    // arrowhead right
    glBegin( GL_LINES );
    glColor3f(1.0, 1.0, 0.0);
    glVertex2f( m_Integration[1][0], m_Integration[1][1] );
    glColor3f(1.0, 1.0, 0.0);
    glVertex2f( m_Integration[1][0] - arrow_dist * (dx + dy), m_Integration[1][1] - arrow_dist * (dy - dx));
    glEnd();
    // arrowhead left
    glBegin( GL_LINES );
    glColor3f(1.0, 1.0, 0.0);
    glVertex2f( m_Integration[1][0], m_Integration[1][1] );
    glColor3f(1.0, 1.0, 0.0);
    glVertex2f( m_Integration[1][0] - arrow_dist * (dx - dy), m_Integration[1][1] - arrow_dist * (dy + dx));
    glEnd();
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
