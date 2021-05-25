//
// Created by 20174019 on 24/05/2021.
//

#include "RailConstraint.h"
#include <GL/glut.h>


RailConstraint::RailConstraint(Particle* p, const Vec2f & start, const Vec2f & end, double dist) :
        m_p(p), m_start(start), m_end(end), m_dist(dist) {
}

std::vector<Particle*> RailConstraint::getParticles()
{
    std::vector<Particle*> particles;
    particles.push_back(m_p);
    return particles;
}

void RailConstraint::draw()
{
    // draws the rail
    glBegin( GL_LINES );
    glColor3f(0.5, 1.0, 0.5);
    glVertex2f( m_start[0], m_start[1] );
    glColor3f(0.5, 1.0, 0.5);
    glVertex2f( m_end[0], m_end[1] );
    glEnd();

    // draws the line from particle to closest point on the rail
    Vec2f rail_point = get_rail_point();
    glBegin( GL_LINES );
    glColor3f(0.5, 1.0, 0.5);
    glVertex2f( rail_point[0], rail_point[1] );
    glColor3f(0.5, 1.0, 0.5);
    glVertex2f( m_p->m_Position[0], m_p->m_Position[1] );
    glEnd();
}

/**
 * Get the point on the rail that is closest to the particle
 * @return rail_point
 */
Vec2f RailConstraint::get_rail_point() {
    // line y = ax + b
    float a = (m_start[1] - m_end[1]) / (m_start[0] - m_end[0]);
    float b = m_start[1] - a * m_start[0];

    Vec2f rail_point = Vec2f(m_p->m_Position[0], a*m_p->m_Position[0]+b);
    if (rail_point[0] < m_start[0]) {
        rail_point[0] = m_start[0];
    } else if (m_end[0] < rail_point[0]) {
        rail_point[0] = m_end[0];
    }

    return rail_point;
}

float RailConstraint::constraint()
{
    Vec2f pos_diff = m_p->m_Position - get_rail_point();
    return pow(pos_diff[0], 2) + pow(pos_diff[1], 2) - pow(m_dist, 2);
}

float RailConstraint::constraint_derivative()
{
    Vec2f pos_diff = m_p->m_Position - get_rail_point();
    return 2 * pos_diff * 2 * m_p->m_Velocity;
}

std::vector<Vec2f> RailConstraint::J()
{
    Vec2f pos_diff = m_p->m_Position - get_rail_point();
    Vec2f j = pos_diff * 2;
    std::vector<Vec2f> res;
    res.push_back(j);
    return res;
}

std::vector<Vec2f> RailConstraint::J_derivative()
{
    Vec2f j_deriv = m_p->m_Velocity * 2;
    std::vector<Vec2f> res;
    res.push_back(j_deriv);
    return res;
}