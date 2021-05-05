//
// Created by 20173613 on 05/05/2021.
//

#include "GravityForce.h"
#include <GL/glut.h>

float G = 9.8;

GravityForce::GravityForce(Particle *p) : m_p(p) {
    //m_p->f += m_p->m_Mass * F->G;
    auto gravity = m_p->m_Mass * G;
    m_p->Force = gravity;
}

void GravityForce::draw()
{
    glBegin( GL_LINES );
    glColor3f(0.21, 0.82, 0.6);
    glVertex2f( m_p->m_Position[0], m_p->m_Position[1] );
    glColor3f(0.21, 0.82, 0.6);
    glVertex2f( m_p->m_Position[0], m_p->m_Position[1]-gravity );
    glEnd();
}