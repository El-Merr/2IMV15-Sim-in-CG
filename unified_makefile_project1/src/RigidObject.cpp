#include "RigidObject.h"
#include <GL/glut.h>

RigidObject::RigidObject(Vec2f centerPoint) :
    pCenter(new Particle(centerPoint, 1))
{
    // init particles in body space
    float offset = 0.05;
    pVector.push_back( new Particle(Vec2f(-offset, -offset), 1) );
    pVector.push_back( new Particle(Vec2f(-offset, offset), 1) );
    pVector.push_back( new Particle(Vec2f(offset, offset), 1) );
    pVector.push_back( new Particle(Vec2f(offset, -offset), 1) );

}

RigidObject::~RigidObject(void) {}

std::vector<Vec2f> RigidObject::get_state()
{
    return pCenter->get_state();
}

void RigidObject::set_state(Vec2f pos, Vec2f vel)
{
    pCenter->set_state(pos, vel);
}

void RigidObject::reset()
{
    pCenter->reset();
    for ( int p=0; p < pVector.size(); p++ ) {
        pVector[p]->reset();
    }
}

void RigidObject::clear_force()
{
    pCenter->clear_force();
    for ( int p=0; p < pVector.size(); p++ ) {
        pVector[p]->clear_force();
    }
}

/* returns a vector of the velocity and the acceleration */
std::vector<Vec2f> RigidObject::derive_eval() {
    return pCenter->derive_eval();
}

std::vector<Vec2f> RigidObject::get_points() {
    std::vector<Vec2f> worldPoints;
    for (int i = 0; i < pVector.size(); i ++) {
        worldPoints.push_back(Vec2f(pCenter->m_Position[0] + pVector[i]->m_Position[0],
                                    pCenter->m_Position[1] + pVector[i]->m_Position[1]));
    }
    return worldPoints;
}

void RigidObject::draw_object() {
    glColor3f(2.f, 0.7f, 4.f);
    std::vector<Vec2f> worldPoints = get_points();
    glBegin(GL_POLYGON);
    for (int i = 0; i < worldPoints.size(); i ++) {
        glVertex2f(worldPoints[i][0], worldPoints[i][1]);
    }
    glEnd();
}