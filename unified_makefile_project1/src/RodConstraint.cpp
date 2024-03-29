#include "RodConstraint.h"
#include <GL/glut.h>

RodConstraint::RodConstraint(Particle *p1, Particle * p2, double dist) :
  m_p1(p1), m_p2(p2), m_dist(dist) {}

RodConstraint::~RodConstraint(void) {}

void RodConstraint::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();

}

std::vector<Particle*> RodConstraint::getParticles()
{
    std::vector<Particle*> particles;
    particles.push_back(m_p1);
    particles.push_back(m_p2);
    return particles;
}

float RodConstraint::constraint()
{
    Vec2f pos_diff = m_p1->m_Position - m_p2->m_Position;
    return pow(pos_diff[0], 2) + pow(pos_diff[1], 2) - pow(m_dist, 2);
}

float RodConstraint::constraint_derivative()
{
    Vec2f pos_diff = m_p1->m_Position - m_p2->m_Position;
    return 2 * pos_diff * 2 * (m_p1->m_Velocity - m_p2->m_Velocity);
}

std::vector<Vec2f> RodConstraint::J()
{
    Vec2f pos_diff1 = m_p1->m_Position - m_p2->m_Position;
    Vec2f pos_diff2 = m_p2->m_Position - m_p1->m_Position;
    Vec2f j_p1 = pos_diff1 * 2;
    Vec2f j_p2 = pos_diff2 * 2;
    std::vector<Vec2f> res;
    res.push_back(j_p1);
    res.push_back(j_p2);
    return res;
}

std::vector<Vec2f> RodConstraint::J_derivative()
{
    Vec2f j_deriv1 = (m_p1->m_Velocity - m_p2->m_Velocity) * 2;
    Vec2f j_deriv2 = (m_p2->m_Velocity - m_p1->m_Velocity) * 2;
    std::vector<Vec2f> res;
    res.push_back(j_deriv1);
    res.push_back(j_deriv2);
    return res;
}