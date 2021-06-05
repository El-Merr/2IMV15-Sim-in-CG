#include "FixedObject.h"
#include <vector>
#include <GL/glut.h>
#include <stdlib.h>
#include <stdio.h>

FixedObject::FixedObject(std::vector<float> pointVector) {
    points = pointVector;
}

//virtual ~FixedObject::FixedObject() {
//
//}

void FixedObject::DrawFixedObject() {
    printf("are we getting here?");
    glColor3f(1.f, 0.6f, 0.f);
    glBegin(GL_QUADS);
    glVertex2f(-.5, -.5);
    glVertex2f(0.5, -.5);
    glVertex2f(0.5, .5);
    glVertex2f(-.5, .5);
    glEnd();
}