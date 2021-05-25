//
// Created by 20173613 on 05/05/2021.
//

#include "GravityForce.h"
#include <GL/glut.h>

static std::vector<float> m_Gravity;

GravityForce::GravityForce(std::vector<Particle*> pVector) : m_pVec(pVector) {
    m_Gravity.clear();
    int size = m_pVec.size();
    for(int ii=0; ii < size; ii++)
    {
        m_Gravity.push_back(m_pVec[ii]->m_Mass * G);
    }
}

GravityForce::~GravityForce(void) {}

void GravityForce::apply_gravity()
{
    int size = m_pVec.size();
    for(int ii=0; ii < size; ii++)
    {
        m_pVec[ii]->m_Force[1] -= m_Gravity[ii];
    }

}

void GravityForce::draw_arrow(Particle* p, float g)
{
    const float scale = 30/G;
    const double arrow_dist = 0.03;

    glBegin( GL_LINES );
    glColor3f(1.0, 0.0, 0.0);   // red
    glVertex2f( p->m_Position[0], p->m_Position[1] );
    glColor3f(1.0, 0.0, 0.0);
    glVertex2f( p->m_Position[0], p->m_Position[1] - g*scale );
    glEnd();

    // arrowhead right
    glBegin( GL_LINES );
    glColor3f(1.0, 0.0, 0.0);
    glVertex2f( p->m_Position[0], p->m_Position[1] - g*scale );
    glColor3f(1.0, 0.0, 0.0);
    glVertex2f( p->m_Position[0] + arrow_dist, p->m_Position[1] - g*scale + arrow_dist );
    glEnd();
    // arrowhead left
    glBegin( GL_LINES );
    glColor3f(1.0, 0.0, 0.0);
    glVertex2f( p->m_Position[0], p->m_Position[1] - g*scale );
    glColor3f(1.0, 0.0, 0.0);
    glVertex2f( p->m_Position[0] - arrow_dist, p->m_Position[1] - g*scale + arrow_dist );
    glEnd();
}

void GravityForce::draw()
{
    int size = m_pVec.size();
    if (drawArrows) {
        for (int ii = 0; ii < size; ii++) {
            draw_arrow(m_pVec[ii], m_Gravity[ii]);
        }
    }
}