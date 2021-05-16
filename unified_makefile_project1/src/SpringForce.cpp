#include "SpringForce.h"
#include "linearSolver.h"
#include <vector>
#include <GL/glut.h>

static Vec2f m_SpringForce;

SpringForce::SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd) :
  m_p1(p1), m_p2(p2), m_dist(dist), m_ks(ks), m_kd(kd) {
}

void SpringForce::applyForce()
{
    const Vec2f l = m_p1->m_Position - m_p2->m_Position;
    printf("length: %f %f \n",(l[0], l[1]));
    float lMagnitude = sqrt(l[0] * l[0] + l[1] * l[1]);
    printf("l magni: %f \n",lMagnitude);
    const Vec2f lDiff = m_p1->m_Velocity - m_p2->m_Velocity;
    printf("l deriv: %f %f \n",(lDiff[0], lDiff[1]));
    float dot = lDiff[0] * l[0] + lDiff[1] * l[1] ;
    printf("dot: %f \n",dot);
    if (lMagnitude != 0.0) {
        m_SpringForce = (m_ks * (lMagnitude - m_dist) + m_kd * (dot / lMagnitude)) * l / lMagnitude;
        printf("%f", m_SpringForce[0]);
        printf("%f \n", m_SpringForce[1]);
    }

    for (int i = 0; i < 2; i++) {
        m_p1->m_Force[i] += m_SpringForce[i];
        m_p2->m_Force[i] -= m_SpringForce[i];
    }
}

void SpringForce::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();
}
