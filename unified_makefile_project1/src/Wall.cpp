//
// Created by 20173613 on 24/05/2021.
//

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
            crudeCollision(pVector, ii);


    }
}

void Wall::elasticCollision(std::vector<Particle*> pVector, int ii) {
    Vec2f N = Vec2f(0, -pVector[ii]->m_Force[1]);

    Vec2f x_normal = (N * pVector[ii]->m_Force)*N;
    Vec2f x_tang = pVector[ii]->m_Force - x_normal;

    // if a particle is within epsilon of the wall, and it is heading into the wall, we detect collision
    if ( norm(pVector[ii]->m_Position - Vec2f(pVector[ii]->m_Position[0], m_start[1])) < eps
         && N * pVector[ii]->m_Force < 0 ) {
        printf("elastic\n");
        Vec2f V_tang = pVector[ii]->m_Force - x_normal;
        //printf("V_tang: %f\n", pVector[ii]->m_Force[1]);
        pVector[ii]->m_Force += V_tang - 0.4 * x_normal;
        //printf("force after: %f\n", pVector[ii]->m_Force[1]);
    }
}

void Wall::crudeCollision(std::vector<Particle*> pVector, int ii) {
    Vec2f V_normal = Vec2f(0, -pVector[ii]->m_Force[1]);
    // if a particle is within epsilon of the wall, and it is heading into the wall, we detect collision
    if ( norm(pVector[ii]->m_Position - Vec2f(pVector[ii]->m_Position[0], m_start[1])) < eps
         && V_normal * pVector[ii]->m_Force < 0 ) {
        //printf("eps larger than: %f\n", V_normal[1]);
        Vec2f V_tang = pVector[ii]->m_Force + V_normal;
        //printf("V_tang: %f\n", pVector[ii]->m_Force[1]);
        pVector[ii]->m_Force += V_tang + 17 * V_normal;
        //printf("force after: %f\n", pVector[ii]->m_Force[1]);
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

//    if(crude) {
//        glColor3f(0.0, 0.8, 0.0);
//        glBegin( GL_QUADS );
//        glVertex2f( -0.5, 0.3 );
//        glVertex2f( -0.5, 0.5  );
//        glVertex2f( -0.7, 0.5 );
//        glVertex2f( -0.7, 0.3  );
//    }

    glEnd();
}
