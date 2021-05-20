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

    VectorXf JWJt_lambda = Jdq - JWQ - C - C_deriv;

    // use linear Conjugate Gradient to solve x for Ax=b
    // where JWJt * lambda = JWJt_lambda
    ConjugateGradient<MatrixXf, Lower|Upper> cg;
    cg.compute(JWJt);
    auto lambda = cg.solve(JWJt_lambda);

    // calculate the force to compensate illegal movement
    VectorXf Q_hat = Jt * lambda;

    // apply Q_hat for every particle
    for ( int p = 0; p < m_pVector.size(); p++ ) {
        Particle* particle = m_pVector[p];
        particle->m_Force[0] += Q_hat[p*dimensions];
        particle->m_Force[1] += Q_hat[p*dimensions + 1];
    }
}

