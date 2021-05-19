//
// Created by 20174019 on 18/05/2021.
//

#include "ConstraintSolver.h"

ConstraintSolver::ConstraintSolver(std::vector<Particle*> pVector, std::vector<Constraint*> cVector ):
        m_pVector(pVector), m_cVector(cVector)
{
}

void ConstraintSolver::apply_constraint()
{
    const int dimensions = 2;
    //int vectorLength = m_pVector.size() * dimensions;
    int constraintsLength = 1; // cVector.size();
//
//    Vec2f q = Vec2f(0.0, 0.0);
//    Vec2f Q = Vec2f(0.0, 0.0);
//    Mat2 W = Mat2();
//
//    Vec2f C = Vec2f(0.0, 0.0);
//    VectorXf Cder = VectorXf::Zero(constraintsLength);
////    MatrixXf J = MatrixXf::Zero(constraintsLength, dimensions);
////    MatrixXf Jt = MatrixXf::Zero(dimensions, constraintsLength);
//    MatrixXf Jder = MatrixXf::Zero(constraintsLength, dimensions);
//
//
//
//    for (int d = 0; d < dimensions; d++) {
//        W(d, d) = 1 / m_p->m_Mass;
//        Q[d] = m_p->m_Force[d];
//        q[d] = m_p -> m_Velocity[d];
//    }
//
//    Vec2f diff = m_p->m_Position - m_center;
//
//    float c = pow(diff[0], 2) + pow(diff[1], 2) - pow(m_radius, 2);
//
//    float c_derivative = 2 * diff * m_p->m_Velocity;
//
//    Vec2f J = diff * 2;
//
//    Vec2f J_derivative = m_p->m_Velocity * 2;
//
////    Mat2 JW = Mat2(J[0]*W[0][0] + J[0]*W[0][1], J[0]*W[1][0] + J[0]*W[1][1],
////                   J[1]*W[0][0] + J[1]*W[0][1], J[1]*W[1][0] + J[1]*W[1][1]);
////
//////    Mat2 JWJ_t = Mat2(JW[0] * J[0],  JW[1] * J[0], //j * j_transposed
//////                      JW[0] * J[1], JW[1] * J[1]);
////
////    Mat2 JWJ_t = MatVec_tMult(JW, J);
////
////    Vec2f J_dq = J_derivative * q;
////
////    Mat2 JWQ_t = transpose(MatVec_tMult(JW, Q));
////
////    Vec2f lambda_t = inverse(JWJ_t) * JWQ_t; //(-J_dq - JWQ_t);
////
////    Vec2f Q_hat = lambda_t_t * J;
////
////    m_p->m_Force += Q_hat;
//
////    float p_r2 = pow(m_p->m_Force[0],2) + pow(m_p->m_Force[0],2);
////    float r_scale = pow(m_radius, 2) / p_r2;
////
////    m_p->m_Force[0] = sqrt(pow(m_p->m_Force[0],2) / r_scale);
////    m_p->m_Force[1] = sqrt(pow(m_p->m_Force[1],2) / r_scale);
//
}
//
//Mat2 CircularWireConstraint::MatVec_tMult(Mat2 m, Vec2 v) {
//    return Mat2(m[0][0] * v[0] + m[1][0] * v[0], m[0][0] * v[1] + m[1][0] * v[1],
//                m[0][1] * v[0] + m[1][1] * v[0], m[0][1] * v[1] + m[1][1] * v[1]);
//}
//
//Mat2 CircularWireConstraint::inverse(Mat2 m) {
//    float a = m[0][0];
//    float b = m[0][1];
//    float c = m[1][0];
//    float d = m[1][1];
//    return Mat2(d, -b, -c, a) * (1/(a*d-b*c));
//}

