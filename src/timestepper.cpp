#include "timestepper.h"

#include <cstdio>

using namespace std;

void RK4::takeStep(ParticleSystem* particleSystem, float stepSize) {

    particleSystem->takeStep();

    // first term
    map<int, Vector3f> pos_1 = particleSystem->getPositions();
    map<int, Vector3f> vel_1 = particleSystem->getVelocities();
    map<int, Vector3f> accel_1 = particleSystem->evalAccel(pos_1, vel_1);

    // second term
    map<int, Vector3f> pos_2;
    map<int, Vector3f> vel_2;
    map<int, Vector3f> accel_2;
    for (const auto& it : pos_1) {
        int i = it.first;
        pos_2.insert(pair <int, Vector3f> (i, pos_1[i] + (stepSize/2) * vel_1[i]));
    }
    for (const auto& it : vel_1) {
        int i = it.first;
        vel_2.insert(pair <int, Vector3f> (i, vel_1[i] + (stepSize/2) * accel_1[i]));
    }
    accel_2 = particleSystem->evalAccel(pos_2, vel_2);

    // third term
    map<int, Vector3f> pos_3;
    map<int, Vector3f> vel_3;
    map<int, Vector3f> accel_3;
    for (const auto& it : pos_1) {
        int i = it.first;
        pos_3.insert(pair <int, Vector3f> (i, pos_1[i] + (stepSize/2) * vel_2[i]));
    }
    for (const auto& it : vel_1) {
        int i = it.first;
        vel_3.insert(pair <int, Vector3f> (i, vel_1[i] + (stepSize/2) * accel_2[i]));
    }
    accel_3 = particleSystem->evalAccel(pos_3, vel_3);

    // fourth term
    map<int, Vector3f> pos_4;
    map<int, Vector3f> vel_4;
    map<int, Vector3f> accel_4;
    for (const auto& it : pos_1) {
        int i = it.first;
        pos_4.insert(pair <int, Vector3f> (i, pos_1[i] + stepSize * vel_3[i]));
    }
    for (const auto& it : vel_1) {
        int i = it.first;
        vel_4.insert(pair <int, Vector3f> (i, vel_1[i] + stepSize * accel_3[i]));
    }
    accel_4 = particleSystem->evalAccel(pos_4, vel_4);

    // putting it together
    for (const auto& it : pos_1) {
        int i = it.first;
        pos_1[i] += (stepSize/6) * (vel_1[i] + vel_2[i]*2.f + vel_3[i]*2.f + vel_4[i]);
    }
    for (const auto& it : vel_1) {
        int i = it.first;
        vel_1[i] += (stepSize/6) * (accel_1[i] + accel_2[i]*2.f + accel_3[i]*2.f + accel_4[i]);
    }

    particleSystem->setPositions(pos_1);
    particleSystem->setVelocities(vel_1);
}

