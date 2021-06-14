#include "RigidObject.h"

#include <vector>
#include <Eigen/Dense>

#define DAMP 0.98f
//#define RAND (((rand()%2000)/1000.f)-1.f)


void compute_rigidObject(RigidObject* p, float dt) {
    std::vector <Vec2f> old = p->get_state();
    // 1st order
    std::vector <Vec2f> deriv = p->derive_eval();

    // 2nd order
    Vec2f position = old[0] + 0.5f * dt * deriv[0];
    Vec2f velocity = DAMP * (old[1] + 0.5f * dt * deriv[1]);
    p->set_state(position, velocity);
    deriv = p->derive_eval();

    // 3rd order
    position = old[0] + 0.5f * dt * deriv[0];
    velocity = DAMP * (old[1] + 0.5f * dt * deriv[1]);
    p->set_state(position, velocity);
    deriv = p->derive_eval();

    // 4th order
    position = old[0] + dt * deriv[0];
    velocity = DAMP * (old[1] + dt * deriv[1]);
    p->set_state(position, velocity);
    deriv = p->derive_eval();

    // Update step
    position = old[0] + dt * deriv[0];
    velocity = DAMP * (old[1] + dt * deriv[1]);
    p->set_state(position, velocity);
}

/**
 * Computes the simulation step.
 * @param pVector   List of particles.
 * @param dt        The time step size of the simulation.
 * @param scheme    An integer denoting the integration scheme.
 */
void rigid_simulation_step( std::vector<RigidObject*> rigidObjects, float dt, bool slomo, int scheme ) {
    if (slomo) {
        dt = dt / 3;
    }

    for (int i = 0; i < rigidObjects.size(); i++) {
        compute_rigidObject(rigidObjects[i], dt);
    }
}

