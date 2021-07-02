#include "RigidObject.h"

#include <vector>
#include <Eigen/Dense>

#define DAMP 0.98f
//#define RAND (((rand()%2000)/1000.f)-1.f)

Vector2f get_vec(VectorXf state, int i) {
    int d = 2;  // dimention
    return Vector2f( state[i*d], state[i*d + 1] );
}

void compute_rigidObject(RigidObject* rb, float dt) {
    VectorXf old = rb->get_state();
    // 1st order
    VectorXf deriv = rb->derive_eval();

    // 2nd order
//    Vector2f position = get_vec(old, 0) * get_vec(deriv, 0);
//    Vec2f velocity = DAMP * (get_vec(old, 1) * get_vec(deriv, 1));

    VectorXf newState = old + dt * deriv;
    rb->set_state(newState);

    VectorXf new_deriv = rb->derive_eval();
    newState[0] = old[0] + dt * new_deriv[0];   // position
    newState[1] = old[1] + dt * new_deriv[1];
    newState[2] = old[2] + dt * new_deriv[2];   // q
    newState[3] = old[3] + dt * new_deriv[3];
    newState[3] = old[4] + dt * new_deriv[4];
    rb->set_state(newState);



//    deriv = p->derive_eval();

//    // 3rd order
//    position = old[0] + 0.5f * dt * deriv[0];
//    velocity = DAMP * (old[1] + 0.5f * dt * deriv[1]);
//    p->set_state(position, velocity);
//    deriv = p->derive_eval();
//
//    // 4th order
//    position = old[0] + dt * deriv[0];
//    velocity = DAMP * (old[1] + dt * deriv[1]);
//    p->set_state(position, velocity);
//    deriv = p->derive_eval();
//
//    // Update step
//    position = old[0] + dt * deriv[0];
//    velocity = DAMP * (old[1] + dt * deriv[1]);
//    p->set_state(position, velocity);
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

