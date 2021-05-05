#pragma once

#include <gfx/vec2.h>

class Particle
{
public:

	Particle(const Vec2f & ConstructPos);
	virtual ~Particle(void);

	void reset();
	void draw();
	Vec2f getState();
	void setState(Vec2f pos, Vec2f vel);
	std::vector<Vec2f,Vec2f> deriveEval();

	Vec2f m_ConstructPos;
	Vec2f m_Position;
	Vec2f m_Velocity;
	Vec2f m_Force;
	float m_Mass;
};
