#include "RigidObject.h"
#include <GL/glut.h>

#define IX(i,j) ((i)+(N+2)*(j))

RigidObject::RigidObject(std::vector<Particle*> particles) :
    pVector(particles)
{

    // calculate center point and total mass
    calc_center_of_mass();

    //transform particle positions from world to body
    for (Particle* p : pVector) {
        p->m_ConstructPos = p->m_Position - Vec2f(construct_position[0], construct_position[1]);
        p->m_Position = p->m_ConstructPos;
    }

    // init variables
    reset();

    // init body matrix
    I_body = Matrix2f::Identity();
    for ( int i=0; i<pVector.size(); i++ ) {
        auto pos = vec_to_Eigen(pVector[i]->m_Position);
        I_body += pos.transpose() * pos  * Matrix2f::Identity() - pos * pos.transpose();
    }
    I_body_inv = I_body.inverse();
    I_inverse = R * I_body_inv * R.transpose();
    omega = I_inverse * L;

}

RigidObject::~RigidObject(void) {}

void RigidObject::reset()
{
    // init variables
    position = construct_position;
    force = Vector2f(0, 0);
    torque = Vector2f(0, 0);
    velocity = Vector2f(0, 0);

    R = Matrix2f::Identity();
    q = Quaternionf(1.0, 0.0, 0.0, 0.0);
    P = Vector2f(0, 0);
    L = Vector2f(0, 0);

    I_inverse = I_body_inv;
    omega = I_inverse * L;

    for ( int p=0; p < pVector.size(); p++ ) {
        pVector[p]->reset();
    }
}

void RigidObject::clear_force()
{
    force = Vector2f(0, 0);
    for ( int p=0; p < pVector.size(); p++ ) {
        pVector[p]->clear_force();
    }
}

void RigidObject::calc_center_of_mass() {
    construct_position = Vector2f(0,0);
    M = 0.0;
    for (Particle* p : pVector) {
        Vec2f pos = p->m_Mass * p->m_Position;
        construct_position += Vector2f(pos[0], pos[1]);
        M += p->m_Mass;
    }
    construct_position = construct_position / M;
}

Vector2f RigidObject::vec_to_Eigen( Vec2f v ) {
    return Vector2f(v[0], v[1]);
}

VectorXf RigidObject::get_state()
{
    VectorXf state(9);

    state[0] = position[0];
    state[1] = position[1];

    state[2] = q.w();
    state[3] = q.x();
    state[4] = q.y();

    state[5] = P[0];
    state[6] = P[1];
    state[7] = L[0];
    state[8] = L[1];

    return state;
}

/* get ydot state */
VectorXf RigidObject::derive_eval()
{
    calc_force_and_torque();
    VectorXf state(9);

    state[0] = velocity[0];
    state[1] = velocity[1];

    Quaternionf omega_quad(0, omega[0], omega[1], 1);
    Quaternionf qdot( omega_quad * q );
    state[2] = qdot.w() * 10;
    state[3] = qdot.x() * 10;
    state[4] = qdot.y() * 10;

    state[5] = force[0];
    state[6] = force[1];
    state[7] = torque[0];
    state[8] = torque[1];

//    std::cout << "eval state: \n" << state << "\n\n";
    return state;

}

void RigidObject::set_state(VectorXf state)
{

    int y = 0;
    position[0] = state[0];
    position[1] = state[1];

    q.w() = state[2];
    q.x() = state[3];
    q.y() = state[4];

    P[0] = state[5];
    P[1] = state[6];
    L[0] = state[7];
    L[1] = state[8];

    calc_aux_variables();

    for (Particle* p : pVector) {
        Vector2f p_pos =  R * vec_to_Eigen(p->m_ConstructPos);
        // help merel dis is bad (N is default 128 dus vandaar)
        if (velocity.norm() < 0.01) {
            p->m_Position = Vec2f(((int) p_pos[0] * 128) / 128, ((int) p_pos[1] * 128) / 128);
        }
    }

}

std::vector<Vec2f> RigidObject::get_points() {
    std::vector<Vec2f> worldPoints;
    for (Particle* p : pVector) {
        worldPoints.push_back(Vec2f(position[0] + p->m_Position[0],
                                    position[1] + p->m_Position[1]));
    }
    return worldPoints;
}



void RigidObject::calc_force_and_torque() {
    force = Vector2f(0, 0);
    torque = Vector2f(0, 0);
    for ( Particle* p : pVector ) {
        Vector2f p_force = vec_to_Eigen(p->m_Force);
        force += p_force;
        Vector2f rel_pos = vec_to_Eigen(p->m_Position);
//        Vector3f p_torque = Vector3f(rel_pos[0], rel_pos[1], 0).cross(Vector3f(p_force[0], p_force[1], 0));
        float p_torque = rel_pos[0] * p_force[1] - rel_pos[1] * p_force[0];
        torque += Vector2f(p_torque, p_torque);
    }
}

void RigidObject::calc_aux_variables() {
    Matrix3f rot = q.normalized().toRotationMatrix();
    R = rot.block(0,0,2,2);
    velocity = P / M;
    I_inverse = R * I_body.inverse() * R.transpose();
    omega = I_inverse * L;
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