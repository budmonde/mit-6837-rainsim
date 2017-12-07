#include "windowsystem.h"

#include <cfloat>
#include <iostream>

#include "camera.h"
#include "vertexrecorder.h"

WindowSystem::WindowSystem(Vector3f origin_) : origin(origin_) {
    // initing random seed
    srand(time(0));
}

void WindowSystem::takeStep() {
    float mass = 1.f;
    float pr = 0.9f;
    if (rand_uniform(0.f, 1.f) < pr) {
        int idx = droplets.size();
        droplets.insert(pair <int, Droplet *> (idx, new Droplet(idx, mass)));
        posState.insert(pair <int, Vector3f> (idx, Vector3f(rand_uniform(-5.f, 5.f), rand_uniform(-5.f, 5.f), 0.f)));
        velState.insert(pair <int, Vector3f> (idx, Vector3f(0.f, 0.f, 0.f)));
    }
    for (const auto& it : droplets) {
        int i = it.first;
        if (posState[i].y() < 0.f) {
            droplets.erase(i);
            posState.erase(i);
            velState.erase(i);
        }
    }
}

map<int, Vector3f> WindowSystem::evalAccel(map<int, Vector3f> posState, map<int, Vector3f> velState) {
    map<int, Vector3f> accel;
    for (const auto& it : droplets) {
        int i = it.first;
        accel[i] = -Vector3f::UP;
    }
    return accel;
}

void WindowSystem::draw(GLProgram& gl) {
    const Vector3f DROPLET_COLOR(0.9f, 0.9f, 0.9f);
    gl.updateMaterial(DROPLET_COLOR);
    gl.updateModelMatrix(Matrix4f::translation(origin));

    VertexRecorder rec;
    for (const auto& it : droplets) {
        int i = it.first;
        gl.updateModelMatrix(Matrix4f::translation(origin+posState[i]));
        drawSphere(0.075f, 10, 10);
    }
    rec.draw();
}

