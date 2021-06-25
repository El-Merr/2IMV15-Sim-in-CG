#include "DragForce.h"
#include "Particle.h"
#include "linearSolver.h"
#include <vector>
#include <cmath>
#include <GL/glut.h>

static Vec2f m_DragForce;

DragForce::DragForce(Particle *p1, RigidObject *p2, double dist, double ks, double kd) :
  m_p1(p1), m_p2(p2), m_dist(dist), m_ks(ks), m_kd(kd) {
}

DragForce::~DragForce( void ) {}

/**
 * Applies spring force between two points.
 */
void DragForce::apply_spring()
{
    auto rb_points = m_p2->get_points();
    int i = 0;
    Vec2f p = rb_points[i] - m_p1->m_Position;
    float length = sqrt(p * p);
    if (length != 0.0) {
        Vec2f v = m_p2->pVector[i]->m_Velocity - m_p1->m_Velocity;
        // rest length set to m_dist/2
        m_DragForce = (m_ks * (length - m_dist/2) + m_kd * ((v * p) / length)) * (p / length) * 1000;
    }

//    printf("DragForce: %f, %f\n", m_DragForce[0], m_DragForce[1]);
    m_p1->m_Force += m_DragForce;
    m_p2->pVector[i]->m_Force -= m_DragForce;

}

void DragForce::draw()
{
    auto rb_points = m_p2->get_points();
    glBegin( GL_LINES );
    glColor3f(0.6, 0.7, 0.8);
    glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
    glColor3f(0.6, 0.7, 0.8);
    glVertex2f( rb_points[0][0], rb_points[0][1] );
    glEnd();
}
