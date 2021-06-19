#include "RigidObject.h"
//#include <vector>
#include <GL/glut.h>
//#include <stdlib.h>
//#include <stdio.h>
//#include <Eigen/Dense>
//#include <Eigen/IterativeLinearSolvers>
//#include <gfx/vec2.h>

RigidObject::RigidObject(Vec2f centerPoint) :
    pCenter(new Particle(centerPoint, 1)), m_Velocity(Vec2f(0.0, 0.0)), m_Force(Vec2f(0.0, 0.0)),
    m_Mass(0.01)//, m_Direction(std::vector<Vec2f>())
{
//    compute_body_space();
//    m_Rotation = (0f, 0f, 0f, 0f);
//
//    m_Direction.push_back(Vec2f(0.0, 0.0));
//    m_Direction.push_back(m_center);



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
    std::vector<Vec2f> state;
    state.push_back(m_center);
    state.push_back(m_Velocity);
//    m_Direction[0] = m_center;
    return state;
}

void RigidObject::set_state(Vec2f pos, Vec2f vel)
{
    m_center = pos;
    m_Velocity = vel;
//    m_Direction[1] = pos;
}

void RigidObject::reset()
{
//    center = centerPoint;
    m_Velocity = Vec2f(0.0, 0.0);
    m_Force = Vec2f(0.0, 0.0);
//    m_Direction.clear();
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
    for (int i = 0; i < pVector.size(); i ++) {
        worldPoints.push_back(Vec2f(pCenter->m_Position[0] + pVector[i]->m_Position[0],
                                    pCenter->m_Position[1] + pVector[i]->m_Position[1]));
    }
    return worldPoints;
}

void RigidObject::draw_object() {
    glColor3f(2.f, 0.7f, 4.f);
//    std::vector<Vec2f> worldPoints = get_points();
    glBegin(GL_POLYGON);
//    for (int i = 0; i < m_points.size(); i ++) {
//        glVertex2f(worldPoints[i][0], worldPoints[i][1]);
//    }
    for (int i=0; i < pVector.size(); i++) {
        glVertex2f(pCenter->m_Position[0] + pVector[i]->m_Position[0],
                   pCenter->m_Position[1] + pVector[i]->m_Position[1]);
    }
    glEnd();
}

//void RigidObject::apply_transformation() {
//    /**
//     * R ' = w * R
//     * r_i = R * m_points + center
//     * v_i = w * R r_0i + v = w x (r_i - center) + v(t)
//     *
//     * torque_i = (r_i - center) X F_i
//     *
//     */
//
//}