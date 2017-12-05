#include "timestepper.h"

#include <cstdio>

using namespace std;

void RK4::takeStep(ParticleSystem* particleSystem, float stepSize) {

    particleSystem->takeStep();

    map<int, Vector3f> curr = particleSystem->getState();
    map<int, Vector3f> k1 = particleSystem->evalF(curr);

    map<int, Vector3f> addK1;
    for (map<int, Vector3f>::iterator it=curr.begin(); it != curr.end(); ++it) {
        int i = it->first;
        addK1.insert(pair <int, Vector3f> (i, curr[i] + (stepSize/2) * k1[i]));
    }
    map<int, Vector3f> k2 = particleSystem->evalF(addK1);

    map<int, Vector3f> addK2;
    for (map<int, Vector3f>::iterator it=curr.begin(); it != curr.end(); ++it) {
        int i = it->first;
        addK2.insert(pair <int, Vector3f> (i, curr[i] + (stepSize/2) * k2[i]));
    }
    map<int, Vector3f> k3 = particleSystem->evalF(addK2);

    map<int, Vector3f> addK3;
    for (map<int, Vector3f>::iterator it=curr.begin(); it != curr.end(); ++it) {
        int i = it->first;
        addK3.insert(pair <int, Vector3f> (i, curr[i] + stepSize * k3[i]));
    }
    map<int, Vector3f> k4 = particleSystem->evalF(addK3);

    for (map<int, Vector3f>::iterator it=curr.begin(); it != curr.end(); ++it) {
        int i = it->first;
        curr[i] += (stepSize/6) * (k1[i] + 2.f*k2[i] + 2.f*k3[i] + k4[i]);
    }
    particleSystem->setState(curr);
}

