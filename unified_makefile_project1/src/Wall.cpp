#include "Wall.h"
#include <GL/glut.h>
#include "iostream"
#include "Particle.h"
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>

const float eps = 0.05;

Wall::Wall(const Vec2f & start, const Vec2f & end) : m_start(start), m_end(end)
{}

Wall::~Wall(void) {

}

void Wall::detectCollision(std::vector<Particle*> pVector) {
    int size = pVector.size();
    for(int ii=0; ii < size; ii++)
    {
            collisionResponse(pVector, ii);
    }
}

void Wall::collisionResponse(std::vector<Particle*> pVector, int ii) {
    Vec2f V_normal = Vec2f(0, -pVector[ii]->m_Force[1]);
    // if a particle is within epsilon of the wall, and it is heading into the wall, we detect collision
    if ( norm(pVector[ii]->m_Position - Vec2f(pVector[ii]->m_Position[0], m_start[1])) < eps
         && V_normal * pVector[ii]->m_Force < 0 ) {

        Vec2f V_tang = pVector[ii]->m_Force + V_normal;
        if (norm(pVector[ii]->m_Position - Vec2f(pVector[ii]->m_Position[0], m_start[1])) < 0.01) {
            pVector[ii]->m_Force[1] =0;
        }else {
            pVector[ii]->m_Force += V_tang + 14 * V_normal;
        }
    }
}

void Wall::draw()
{
    glColor3f(0.8, 0.0, 0.0);
    glBegin( GL_QUADS );
    glVertex2f( m_start[0], m_start[1] );
    glVertex2f( m_start[0], m_start[1]+0.02 );
    glVertex2f( m_end[0], m_end[1]+0.02 );
    glVertex2f( m_end[0], m_end[1] );

    glEnd();
}
