//
// Created by 20173613 on 24/05/2021.
//

#include "Wall.h"
#include <GL/glut.h>
#include "iostream"

Wall::Wall(float x1, float y1, float x2, float y2) : m_start(Vec2f(x1, y1)), m_end(Vec2f(x2, y2))
{}

Wall::~Wall(void) {

}

void Wall::draw()
{
    //printf("wall start: %f%f\n", m_start[0],m_start[1]);
    glBegin( GL_LINES );
    glColor3f(0.8, 0.0, 0.0);
    //glVertex2f( m_start[0], m_start[1] );
    glVertex2f( 0.4, 0.1);
    glColor3f(0.8, 0.0, 0.0);
    //glVertex2f( m_end[0], m_end[1] );
    glVertex2f( 0.6, 0.1 );
    glEnd();

}
