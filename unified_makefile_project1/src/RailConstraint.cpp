//
// Created by 20174019 on 24/05/2021.
//

#include "RailConstraint.h"
#include <GL/glut.h>


RailConstraint::RailConstraint(Particle* p, const Vec2f & start, const Vec2f & end) :
        m_p(p), m_start(start), m_end(end) {
}

std::vector<Particle*> RailConstraint::getParticles()
{
    std::vector<Particle*> particles;
    particles.push_back(m_p);
    return particles;
}

void RailConstraint::draw()
{
    glBegin( GL_LINES );
    glColor3f(0.5, 1.0, 0.5);
    glVertex2f( m_start[0], m_start[1] );
    glColor3f(0.5, 1.0, 0.5);
    glVertex2f( m_end[0], m_end[1] );
    glEnd();
}

Vec2f RailConstraint::get_rail_point() {
    float a = (m_start[1] - m_end[1]) / (m_start[0] - m_end[0]);
    float b = m_start[1] - a * m_start[0];
//    return a * m_p->m_Position[0] - m_p->m_Position[1] + b;

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
//    Vec2f pos_diff = m_p->m_Position - m_start;
//    return pow(pos_diff[0], 2) + pow(pos_diff[1], 2) - pow(m_radius, 2);

//    float a = (m_start[1] - m_end[1]) / (m_start[0] - m_end[0]);
//    float b = m_start[1] - a * m_start[0];
////    return a * m_p->m_Position[0] - m_p->m_Position[1] + b;
//
//    Vec2f rail_point = Vec2f(m_p->m_Position[0], a*m_p->m_Position[0]+b);
    Vec2f pos_diff = m_p->m_Position - get_rail_point();
    return pow(pos_diff[0], 2) + pow(pos_diff[1], 2) - pow(0.2, 2);
//    return pow(pos_diff[1], 2);
}

float RailConstraint::constraint_derivative()
{
//    Vec2f pos_diff = m_p->m_Position[1] - m_start[1];
//    return 2 * pos_diff * m_p->m_Velocity;
//    float a = (m_start[1] - m_end[1]) / (m_start[0] - m_end[0]);
//    float b = m_start[1] - a * m_start[0];
//    return a;// * (m_p->m_Velocity[0] + m_p->m_Velocity[1]);
    Vec2f pos_diff = m_p->m_Position - get_rail_point();
    return 2 * pos_diff * 2 * m_p->m_Velocity;
//    return 2 * pos_diff * m_p->m_Velocity;

}

std::vector<Vec2f> RailConstraint::J()
{
    Vec2f line = Vec2f(0.0, m_start[1]);
    Vec2f pos_diff = m_p->m_Position - get_rail_point();
    Vec2f j = pos_diff * 2;
    std::vector<Vec2f> res;
    res.push_back(j);
    return res;
}

std::vector<Vec2f> RailConstraint::J_derivative()
{
    Vec2f j_deriv = m_p->m_Velocity * 2; //[1] * Vec2f(2.0,1.0);
    std::vector<Vec2f> res;
    res.push_back(j_deriv);
    return res;
}