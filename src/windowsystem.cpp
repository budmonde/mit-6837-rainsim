#include "windowsystem.h"

#include <cfloat>
#include <iostream>

#include "camera.h"
#include "vertexrecorder.h"

WindowSystem::WindowSystem(Vector3f origin_) : origin(origin_) {
}

void WindowSystem::takeStep() {
    float mass = 1.f;
    float pr = 0.3f;
    if (rand_uniform(0.f, 1.f) < pr) {
        int idx = droplets.size();
        droplets.push_back(new Droplet(idx, mass));
        m_vVecState.insert(pair <int, Vector3f> (idx*2, Vector3f(rand_uniform(-5.f, 5.f), rand_uniform(-5.f, 5.f), 0.f)));
        m_vVecState.insert(pair <int, Vector3f> (idx*2+1, Vector3f(0.f, 0.f, 0.f)));
    }
}

map<int, Vector3f> WindowSystem::evalF(map<int, Vector3f> state) {
    map<int, Vector3f> f;
    for (size_t i=0; i < droplets.size(); ++i) {
        f[i*2] = state[i*2+1];
        f[i*2+1] = -Vector3f::UP;
    }
    return f;
}

void WindowSystem::draw(GLProgram& gl) {
    const Vector3f DROPLET_COLOR(0.9f, 0.9f, 0.9f);
    gl.updateMaterial(DROPLET_COLOR);
    gl.updateModelMatrix(Matrix4f::translation(origin));

    VertexRecorder rec;
    for (size_t i=0; i < droplets.size(); ++i) {
        gl.updateModelMatrix(Matrix4f::translation(origin+m_vVecState[i*2]));
        drawSphere(0.075f, 10, 10);
    }
    rec.draw();
}

