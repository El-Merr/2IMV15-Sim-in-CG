#pragma once

#include <gfx/vec2.h>
#include <vector>

class MouseParticle //singleton
{
public:
   // static MouseParticle& GetInstance(const Vec2f & ConstructPos, float mass);
    Vec2f m_ConstructPos;
    Vec2f m_Position;
    Vec2f m_Velocity;
    Vec2f m_Force;
    float m_Mass;

    void draw();
    MouseParticle(MouseParticle const&) = delete;
    void operator=(MouseParticle const&) = delete;

private:
    MouseParticle(const Vec2f & ConstructPos, float mass); //this is the private constructor



//	Particle(const Vec2f & ConstructPos, float mass);
//	virtual ~Particle(void);
//
//	void reset();
//	void clearForce();
//	void draw();
//	Vec2f getState();
//	void setState(Vec2f pos, Vec2f vel);
//	std::vector<Vec2f> deriveEval();
//    void computeVelocity();
//	Vec2f m_ConstructPos;
//	Vec2f m_Position;
//	Vec2f m_Velocity;
//	Vec2f m_Force;
//	float m_Mass;
};
