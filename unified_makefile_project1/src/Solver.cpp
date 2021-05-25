#include "Particle.h"

#include <vector>
#include <Eigen/Dense>

#define DAMP 0.98f
//#define RAND (((rand()%2000)/1000.f)-1.f)

/**
 * Computes the Euler step.
 * @param p     The particle for which to do the computation.
 * @param dt    The time step size.
 */
void compute_euler(Particle* p, float dt) {
    std::vector<Vec2f> old = p->get_state();
    std::vector<Vec2f> deriv = p->derive_eval();
    Vec2f position = old[0] + dt * deriv[0];
    Vec2f velocity = DAMP * (old[1] + dt * deriv[1]);
    p->set_state(position, velocity);
}

/**
 * Computes the mid-point step.
 * @param p     The particle for which to do the computation.
 * @param dt    The time step size.
 */
void compute_midpoint(Particle* p, float dt) {

}

/**
 * Computes the Runge-Kutta 4 step.
 * @param p     The particle for which to do the computation.
 * @param dt    The time step size.
 */
void compute_rungekutta(Particle* p, float dt) {

}

/**
 * Computes the simulation step.
 * @param pVector   List of particles.
 * @param dt        The time step size of the simulation.
 * @param scheme    An integer denoting the integration scheme.
 */
void simulation_step( std::vector<Particle*> pVector, float dt, bool slomo, int scheme ) {
    for (int ii = 0; ii < pVector.size(); ii++) {
        switch (scheme) {
            case 0: // Euler
                compute_euler(pVector[ii], dt);
                if (slomo) {
                    pVector[ii]->m_Velocity = Vec2f(pVector[ii]->m_Velocity[0] / 3, pVector[ii]->m_Velocity[1] / 3);
                }
                break;
            case 1: // Mid-point
                compute_midpoint(pVector[ii], dt);
                break;
            case 2: // Runge-Kutta 4
                compute_rungekutta(pVector[ii], dt);
                break;
        }
    }
}

