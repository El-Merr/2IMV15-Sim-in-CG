//
// Created by 20174019 on 18/05/2021.
//

#include "ConstraintSolver.h"

using namespace Eigen;

ConstraintSolver::ConstraintSolver(std::vector<Particle*> pVector, std::vector<Constraint*> cVector ):
        m_pVector(pVector), m_cVector(cVector)
{
}

void ConstraintSolver::apply_constraint()
{
    const int dimensions = 2;
    int vector_size = m_pVector.size() * dimensions;
    int num_constraints = m_cVector.size();

    VectorXf q = VectorXf::Zero(vector_size);
    VectorXf Q = VectorXf::Zero(vector_size);
    MatrixXf W = MatrixXf::Zero(vector_size, vector_size);
    VectorXf C = VectorXf::Zero(num_constraints);
    VectorXf C_deriv = VectorXf::Zero(num_constraints);
    MatrixXf J = MatrixXf::Zero(num_constraints, vector_size);
    MatrixXf Jt = MatrixXf::Zero(vector_size, num_constraints);
    MatrixXf J_deriv = MatrixXf::Zero(num_constraints, vector_size);

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
    // Fill W, Q and q with all particles
    for ( int p = 0; p < m_pVector.size(); p++) {
        Particle* particle = m_pVector[p];
        for (int d = 0; d < dimensions; d++) {
            W(p*dimensions + d, p*dimensions + d) = 1 / particle->m_Mass;
            Q[p*dimensions + d] = particle->m_Force[d];
            q[p*dimensions + d] = particle->m_Velocity[d];
        }
    }

    // Fill constraints
    for ( int c = 0; c < m_cVector.size(); c++ ) {
        Constraint* constraint = m_cVector[c];

        // get constraint calculations
        C[c] = constraint->constraint();
        C_deriv[c] = constraint->constraint_derivative();
        std::vector<Vec2f> j = constraint->J();
        std::vector<Vec2f> j_deriv = constraint->J_derivative();

        // get all particles affected by constraint
        std::vector<Particle*> particles = constraint->getParticles();
        for ( int p = 0; p < particles.size(); p++ ) {
            // find particle index
            auto it = std::find(m_pVector.begin(), m_pVector.end(), particles[p]);
            int p_i = distance(m_pVector.begin(), it);
            if ( p_i < m_pVector.size() ) {     // particle found
                for (int d = 0; d < dimensions; d++) {
                    int index = p_i * dimensions + d;
                    J(c, index ) = j[p][d];
                    J_deriv(c, index ) = j_deriv[p][d];
                    Jt(index, c) = j[p][d];
                }
            } else {
                printf("ERROR: Particle not found");
            }
        }
    }

    MatrixXf JW = J * W;
    MatrixXf JWJt = JW * Jt;
    VectorXf Jdq = -1 * J_deriv * q;
    VectorXf JWQ = JW * Q;
//    RowVectorXf JWQt = JW * Q.transpose();
//    VectorXf temp = -Jdq.transpose() - JWQ.transpose();
    VectorXf x = Jdq - JWQ - C - C_deriv;

    ConjugateGradient<MatrixXf, Lower|Upper> cg;
    cg.compute(JWJt);
    auto lambda = cg.solve(x);
//    auto lambda = ConjGrad();

//    VectorXf lambda_t = JWJt.inverse() * ( -Jdq.transpose() - JWQ.transpose());
    VectorXf Q_hat = Jt * lambda;

//
    for ( int p = 0; p < m_pVector.size(); p++ ) {
        Particle* particle = m_pVector[p];
        particle->m_Force[0] += Q_hat[p*dimensions];
        particle->m_Force[1] += Q_hat[p*dimensions + 1];
    }

//    VectorXf KsC = ks * C;
//    VectorXf KdCd = kd * Cder;
//    VectorXf rhs = Jderq - JWQ - KsC - KdCd;

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

