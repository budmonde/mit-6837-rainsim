#include "timestepper.h"

#include <cstdio>

using namespace std;

void RK4::takeStep(ParticleSystem* particleSystem, float stepSize) {

    particleSystem->takeStep(stepSize);

}

