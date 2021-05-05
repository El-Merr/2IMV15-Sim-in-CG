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
	Vec2f setState();
	Vec2f deriveEval();

	Vec2f m_ConstructPos;
	Vec2f m_Position;
	Vec2f m_Velocity;
	Vec2f m_ForceAccumulator;
	float m_Mass;
};
