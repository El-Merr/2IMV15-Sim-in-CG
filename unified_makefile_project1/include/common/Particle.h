#pragma once

#include <gfx/vec2.h>
#include <vector>

#define H 5.0f
//#define MAX_SIZE 2

class Particle
{
public:

	Particle(const Vec2f & ConstructPos, float mass);
	virtual ~Particle(void);

	void reset();
	void clear_force();
	void draw();
    void draw_integration();
	Vec2f get_state();
	void set_state(Vec2f pos, Vec2f vel);
	std::vector<Vec2f> derive_eval();
    void compute_velocity();
    void compute_integration(float dt, int scheme);
    void compute_euler();
    void compute_midpoint();
    void compute_rungekutta();
	Vec2f m_ConstructPos;
	Vec2f m_Position;
	Vec2f m_Velocity;
	Vec2f m_Force;
	float m_Mass;
    std::vector<Vec2f> m_Integration;
    int m_Counter;
};
