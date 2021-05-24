//
// Created by 20173613 on 24/05/2021.
//

#include "Wall.h"
#include <GL/glut.h>
#include "iostream"
#include "Particle.h"

const float eps = 0.05;

Wall::Wall(float x1, float y1, float x2, float y2) : m_start(Vec2f(x1, y1)), m_end(Vec2f(x2, y2))
{}

Wall::~Wall(void) {

}

void Wall::detectCollision(std::vector<Particle*> pVector) {
    printf("detectColission\n");
    int size = pVector.size();
    for(int ii=0; ii < size; ii++)
    {
        Vec2f V_normal = Vec2f(0, -pVector[ii]->m_Force[1]);
        // if a particle is within epsilon of the wall, and it is heading into the wall, we detect collision
        if ( (pVector[ii]->m_Position-Vec2f(0, -0.6)) * V_normal < eps
                && V_normal * pVector[ii]->m_Force < 0 ) {
            printf("Particle is close\n");
            Vec2f V_tang = pVector[ii]->m_Force - V_normal;
            pVector[ii]->m_Force = V_tang - 0.4 * V_normal;
        }
    }
}

void Wall::draw()
{
    //printf("wall start: %f%f\n", m_start[0],m_start[1]);
    glBegin( GL_LINES );
    glColor3f(0.8, 0.0, 0.0);
    //glVertex2f( m_start[0], m_start[1] );
    glVertex2f( -0.6, -0.6);
    glColor3f(0.8, 0.0, 0.0);
    //glVertex2f( m_end[0], m_end[1] );
    glVertex2f( 0.6, -0.6 );
    glEnd();

}
