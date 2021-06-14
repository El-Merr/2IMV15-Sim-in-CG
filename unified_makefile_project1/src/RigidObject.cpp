#include "RigidObject.h"
#include <vector>
#include <GL/glut.h>
#include <stdlib.h>
#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <gfx/vec2.h>

RigidObject::RigidObject(Vec2f centerPoint) :
    center(centerPoint), m_Velocity(Vec2f(0.0, 0.0)), m_Force(Vec2f(0.0, 0.0)), m_Mass(0.001),
    m_Direction(std::vector<Vec2f>())
{
    computeBodySpace();
    m_Direction.push_back(Vec2f(0.0, 0.0));
    m_Direction.push_back(center);
}

RigidObject::~RigidObject(void) {}

void RigidObject::computeBodySpace() {
    float offset = 0.1;
    points.clear();
    points.push_back(Vec2f(offset, -offset)); //bottomright
    points.push_back(Vec2f(offset, offset));  //topright
    points.push_back(Vec2f(-offset, offset)); //topleft
    points.push_back(Vec2f(-offset, -offset));//bottomleft

}

std::vector<Vec2f> RigidObject::get_state()
{
    std::vector<Vec2f> state;
    state.push_back(center);
    state.push_back(m_Velocity);
    m_Direction[0] = center;
    return state;
}

void RigidObject::set_state(Vec2f pos, Vec2f vel)
{
    center = pos;
    m_Velocity = vel;
    m_Direction[1] = pos;
}

void RigidObject::reset()
{
//    center = centerPoint;
    m_Velocity = Vec2f(0.0, 0.0);
    m_Force = Vec2f(0.0, 0.0);
    m_Direction.clear();
}

void RigidObject::clear_force()
{
    m_Force = Vec2f(0.0, 0.0);
}

std::vector<Vec2f> RigidObject::derive_eval() { // returns a vector of the velocity and the acceleration
    std::vector<Vec2f> eval;
    eval.push_back(m_Velocity);
    eval.push_back(m_Force / m_Mass);
    return eval;
}

std::vector<Vec2f> RigidObject::get_points() {
    std::vector<Vec2f> worldPoints;
    for (int i = 0; i < points.size(); i ++) {
        worldPoints.push_back(Vec2f(center[0]+points[i][0], center[1]+points[i][1]));
    }
    return worldPoints;
}

void RigidObject::drawRigidObject() {
    glColor3f(2.f, 0.7f, 4.f);
    std::vector<Vec2f> worldPoints = get_points();
    glBegin(GL_POLYGON);
    for (int i = 0; i < points.size(); i ++) {
        glVertex2f(worldPoints[i][0], worldPoints[i][1]);
    }
    glEnd();
}