#include "RigidObject.h"
#include <GL/glut.h>

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
    for ( auto p : pVector ) {
        auto pos = vec_to_Eigen(p->m_Position);
        I_body += pos.transpose() * pos  * Matrix2f::Identity() - pos * pos.transpose();
    }
    I_body_inv = I_body.inverse();
    I_inverse = R * I_body_inv * R.transpose();
    omega = 0; //I_inverse * L;

}

RigidObject::~RigidObject(void) {}

void RigidObject::reset()
{
    // init variables
    position = construct_position;
    force = Vector2f(0, 0);
    torque = 0; //Vector2f(0, 0);
    velocity = Vector2f(0, 0);

    R = Matrix2f::Identity();
    q = Quaternionf(1.0, 0.0, 0.0, 0.0);
    P = Vector2f(0, 0);
    L = 0;//Vector2f(0, 0);

    I_inverse = I_body_inv;
    omega = 0; //I_inverse * L;

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

    state[2] = R(0,0);
    state[3] = R(0,1);
    state[4] = R(1,0);
    state[5] = R(1,1);

    state[6] = P[0];
    state[7] = P[1];
    state[8] = L;

    return state;
}

/* get ydot state */
VectorXf RigidObject::derive_eval()
{
    calc_force_and_torque();
    VectorXf state(9);

    state[0] = velocity[0];
    state[1] = velocity[1];
    
    Matrix2f Rdot = Matrix2f::Zero(); //R * omega;
//    for (int r=0; r<2; r++) {
//        Rdot(0, r) = omega[r] * R(0, 0) + omega[r] * R(0, 1);
//        Rdot(1, r) = omega[r] * R(1, 0) + omega[r] * R(1, 1);
//    }
//    Rdot *= 0.5;

    float angle = std::acos(omega);
    printf("omega: %f       angle: %f\n", omega, angle);
//    printf("angle: %f,  cos: %f\n", angle, (float)std::cos(angle));
    Matrix2f rot = Matrix2f::Zero();
    rot(0,0) = 0;//std::cos(angle);
    rot(0,1) = -std::sin(omega*10);
    rot(1,0) = std::sin(omega*10);
    rot(1,1) = 0; //std::cos(angle);
//    Rdot = R;
    Rdot = R * rot;

    state[2] = Rdot(0,0);
    state[3] = Rdot(0,1);
    state[4] = Rdot(1,0);
    state[5] = Rdot(1,1);

    state[6] = force[0];
    state[7] = force[1];
    state[8] = torque;

//    std::cout << "eval state: \n" << state << "\n\n";
    return state;

}

void RigidObject::set_state(VectorXf state)
{

    int y = 0;
    position[0] = state[0];
    position[1] = state[1];

    R(0,0) = state[2];
    R(0,1) = state[3];
    R(1,0) = state[4];
    R(1,1) = state[5];

    P[0] = state[6];
    P[1] = state[7];
    L = state[8];



    for (Particle* p : pVector) {
        Vector2f p_pos =  R * vec_to_Eigen(p->m_ConstructPos);
        p->m_Position = Vec2f(p_pos[0], p_pos[1]);
    }

    calc_aux_variables();
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
    torque = 0; //Vector2f(0, 0);
    for ( Particle* p : pVector ) {
        Vector2f p_force = vec_to_Eigen(p->m_Force);
        force += p_force;
        Vector2f rel_pos = vec_to_Eigen(p->m_Position);
//        Vector3f p_torque = Vector3f(rel_pos[0], rel_pos[1], 0).cross(Vector3f(p_force[0], p_force[1], 0));
//        Vector2f perp_r = Vector2f(-rel_pos[1], rel_pos[0]);
//        Vector2f angular_force = Vector2f(0, 0);
//        if (p_force[0] > 0) {
//            angular_force = p_force - rel_pos * (p_force.dot(rel_pos) / rel_pos.dot(p_force));
//        }
//        float p_torque = rel_pos[0] * p_force[1] - rel_pos[1] * p_force[0];
//        std::cout << "ang: \n" << angular_force << "\n";
//        Vector2f p_torque = p_force - rel_pos * (p_force.dot(rel_pos) / rel_pos.dot(p_force)); //Vector2f(rel_pos.dot(p_force), rel_pos.dot(p_force));
////        std::cout << p_torque << "\n";
//        float p_torque = rel_pos.dot(p_force);

        float p_torque = crossProduct( rel_pos, p_force );
        torque += p_torque; //Vector2f(p_torque[0], p_torque[1]);
    }
//    std::cout << torque << "\n";
}

void RigidObject::calc_aux_variables() {
//    Matrix3f rot = q.normalized().toRotationMatrix();
//    R = rot.block(0,0,2,2);
    velocity = P / M;
    I_inverse = R * I_body.inverse() * R.transpose();
    omega = I_inverse.norm() * L ; //I_inverse * L;
}

// Two crossed vectors return a scalar
float RigidObject::crossProduct( const Vector2f& a, const Vector2f& b )
{
    return a[0] * b[1] - a[1] * b[0];
}

// More exotic (but necessary) forms of the cross product
// with a vector a and scalar s, both returning a vector
Vector2f RigidObject::crossProduct( const Vector2f& a, float s )
{
    return Vector2f( s * a[1], -s * a[0] );
}

Vector2f RigidObject::crossProduct( float s, const Vector2f& a )
{
    return Vector2f( -s * a[1], s * a[0] );
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