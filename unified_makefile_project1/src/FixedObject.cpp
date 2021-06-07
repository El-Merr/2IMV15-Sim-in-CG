#include "FixedObject.h"
#include <vector>
#include <GL/glut.h>
#include <stdlib.h>
#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <gfx/vec2.h>

FixedObject::FixedObject(std::vector<Vec2f> pointVector) {
    points = pointVector;
}

FixedObject::~FixedObject(void) {

}

void FixedObject::DrawFixedObject() {
    glColor3f(1.f, 0.7f, 0.f);
    glBegin(GL_POLYGON);
    for (int i = 0; i < points.size(); i ++) {
        glVertex2f(points[i][0], points[i][1]);
    }
    glEnd();
}