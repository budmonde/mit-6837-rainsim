#include "timestepper.h"

#include <cstdio>

using namespace std;

void ForwardEuler::takeStep(ParticleSystem* particleSystem, float stepSize) {
    vector<Vector3f> curr = particleSystem->getState();
    vector<Vector3f> f = particleSystem->evalF(curr);
    for (int i=0; i < (int)curr.size(); ++i) {
        curr[i] += stepSize * f[i];
    }
    particleSystem->setState(curr);
}

void Trapezoidal::takeStep(ParticleSystem* particleSystem, float stepSize) {
    vector<Vector3f> curr = particleSystem->getState();
    vector<Vector3f> f0 = particleSystem->evalF(curr);

    vector<Vector3f> next;
    for (int i=0; i < (int)curr.size(); ++i) {
        next.push_back(curr[i] + stepSize * f0[i]);
    }
    vector<Vector3f> f1 = particleSystem->evalF(next);

    for (int i=0; i < (int)curr.size(); ++i) {
        curr[i] += (stepSize/2) * (f0[i] + f1[i]);
    }
    particleSystem->setState(curr);
}

void RK4::takeStep(ParticleSystem* particleSystem, float stepSize) {
    vector<Vector3f> curr = particleSystem->getState();
    vector<Vector3f> k1 = particleSystem->evalF(curr);

    vector<Vector3f> addK1;
    for (int i=0; i < (int)curr.size(); ++i) {
        addK1.push_back(curr[i] + (stepSize/2) * k1[i]);
    }
    vector<Vector3f> k2 = particleSystem->evalF(addK1);

    vector<Vector3f> addK2;
    for (int i=0; i < (int)curr.size(); ++i) {
        addK2.push_back(curr[i] + (stepSize/2) * k2[i]);
    }
    vector<Vector3f> k3 = particleSystem->evalF(addK2);

    vector<Vector3f> addK3;
    for (int i=0; i < (int)curr.size(); ++i) {
        addK3.push_back(curr[i] + stepSize * k3[i]);
    }
    vector<Vector3f> k4 = particleSystem->evalF(addK3);

    for (int i=0; i < (int)curr.size(); ++i) {
        curr[i] += (stepSize/6) * (k1[i] + 2.f*k2[i] + 2.f*k3[i] + k4[i]);
    }
    particleSystem->setState(curr);
}

