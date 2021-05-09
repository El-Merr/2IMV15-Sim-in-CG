//
// Created by 20173613 on 05/05/2021.
//

#include "GravityForce.h"
#include <GL/glut.h>

GravityForce::GravityForce(Particle *p) : m_p(p), m_Gravity(G) {
    m_Gravity = m_p->m_Mass * G;
    m_p->m_Force[1] = -m_Gravity;
}

void GravityForce::draw()
{
    const double scale = 1;
    const double arrow_dist = 0.03;

    glBegin( GL_LINES );
    glColor3f(1.0, 0.0, 0.0);   // red
    glVertex2f( m_p->m_Position[0], m_p->m_Position[1] );
    glColor3f(1.0, 0.0, 0.0);
    glVertex2f( m_p->m_Position[0], m_p->m_Position[1] - m_Gravity*scale );
    glEnd();

    // arrowhead right
    glBegin( GL_LINES );
    glColor3f(1.0, 0.0, 0.0);
    glVertex2f( m_p->m_Position[0], m_p->m_Position[1] - m_Gravity*scale );
    glColor3f(1.0, 0.0, 0.0);
    glVertex2f( m_p->m_Position[0] + arrow_dist, m_p->m_Position[1] - m_Gravity*scale + arrow_dist );
    glEnd();
    // arrowhead left
    glBegin( GL_LINES );
    glColor3f(1.0, 0.0, 0.0);
    glVertex2f( m_p->m_Position[0], m_p->m_Position[1] - m_Gravity*scale );
    glColor3f(1.0, 0.0, 0.0);
    glVertex2f( m_p->m_Position[0] - arrow_dist, m_p->m_Position[1] - m_Gravity*scale + arrow_dist );
    glEnd();
}