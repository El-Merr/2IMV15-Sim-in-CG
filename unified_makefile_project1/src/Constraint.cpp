//
// Created by 20174019 on 19/05/2021.
//

#include "Constraint.h"

void draw() {};
std::vector<Particle*> getParticles() { return std::vector<Particle*>({}); };

float constraint() { return 0; };
float constraint_derivative() { return 0; };
std::vector<Vec2f> J() { return std::vector<Vec2f>({(0.0)}); };
std::vector<Vec2f> J_derivative() { return std::vector<Vec2f>({(0.0)}); };