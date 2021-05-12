#include "Particle.h"

#include <vector>

#define DAMP 0.98f
#define RAND (((rand()%2000)/1000.f)-1.f)
void simulation_step( std::vector<Particle*> pVector, float dt )
{
    int size = pVector.size();
	
	for(int ii=0; ii<size; ii++)
	{
        pVector[ii]->computeVelocity();
        pVector[ii]->m_Velocity = DAMP*pVector[ii]->m_Velocity;
		pVector[ii]->m_Position += dt*pVector[ii]->m_Velocity;

	}
//    printf("%f \n",dt*pVector[0]->m_Velocity[1]);
}

