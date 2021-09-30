#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include <map>
#include <vector>
#include "vecmath.h"

#include "particlesystem.h"

class TimeStepper
{
public:
    virtual ~TimeStepper() {}
	virtual void takeStep(ParticleSystem* particleSystem, float stepSize) = 0;
};

//IMPLEMENT YOUR TIMESTEPPERS

class RK4 : public TimeStepper
{
	void takeStep(ParticleSystem* particleSystem, float stepSize) override;
};

/////////////////////////
#endif
