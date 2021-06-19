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

    // init variables
    position = vec_to_Eigen(centerPoint);
    rb_Force = Vector2f(0, 0);
    rb_Torque = Vector2f(0, 0);
    calc_force_and_torque();

    R = Matrix2f::Identity();
    P = Vector2f(0, 0);
    L = Vector2f(0, 0);

    I_body = Matrix2f::Identity();
    for ( int i=0; i<pVector.size(); i++ ) {
        auto pos = vec_toEigen(pVector[i]->m_Position)
        I_body(i, i) = pos.transpose() * pos  * Matrix2f::Identity() - pos * pos.transpose();
    }
    I_inverse = R * I_body.inverse() * R.transpose();


}

RigidObject::~RigidObject(void) {}

Vector2f RigidObject::vec_to_Eigen( Vec2f v ) {
    return Vector2f(v[0], v[1]);
}

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

void RigidObject::calc_force_and_torque() {
    rb_Force = Vector2f(0, 0);
    rb_Torque = Vector2f(0, 0);
    for ( Particle* p : pVector ) {
        rb_Force += vec_to_Eigen(p->m_Force);
        Vector2f rel_pos = vec_to_Eigen(p->m_Position) - position;
        Vector2f force = vec_to_Eigen(p->m_Force);
        Vector3f torque = Vector3f(rel_pos[0], rel_pos[1], 0).cross(Vector3f(force[0], force[1], 0));
        rb_Torque += Vector2f(torque[0], torque[1]);
    }
}