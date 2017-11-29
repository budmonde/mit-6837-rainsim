#include "pendulumsystem.h"

#include <cassert>
#include "camera.h"
#include "vertexrecorder.h"

PendulumSystem::PendulumSystem(Vector3f origin_, int num_parts_, float step_sz_, float spring_k_) : origin(origin_), num_parts(num_parts_), step_sz(step_sz_), spring_k(spring_k_) {

    float height = 0.f;
    bodies.push_back(new Body(bodies.size(), true, spring_k));
    m_vVecState.push_back(Vector3f(0.f,height,rand_uniform(-0.1f, 0.1f)));
    m_vVecState.push_back(Vector3f(0.f,0.f,0.f));
    height -= step_sz;
    for (int i=1; i < num_parts; ++i) {
        bodies.push_back(new Body(bodies.size(), false, spring_k));
        m_vVecState.push_back(Vector3f(rand_uniform(-0.1f, 0.1f),height,rand_uniform(-0.1f, 0.1f)));
        m_vVecState.push_back(Vector3f(0.f,0.f,0.f));
        height -= step_sz;

        Vector3f d = m_vVecState[i*2] - m_vVecState[(i-1)*2];
        connectBodies(bodies[i], bodies[i-1], d.abs());
    }

}

vector<Vector3f> PendulumSystem::evalF(vector<Vector3f> state) {
    vector<Vector3f> f(state.size());
    for (int i=0; i < (int) bodies.size(); ++i) {
        Body * b1 = bodies[i];
        Vector3f vel = state[i*2+1];
        Vector3f force = Vector3f::ZERO;
        if (!b1->stat) {
            force += -Vector3f::UP * b1->mass;
            force += vel * (-b1->drag);
            for (int j=0; j < (int) b1->s_bodies.size(); ++j) {
                Body * b2 = b1->s_bodies[j];
                Vector3f d = state[i*2] - state[b2->idx*2];
                force += d / d.abs() * -b1->s_ks[j] * (d.abs() - b1->s_drs[j]);
            }
        }
        f[i*2] = vel;
        f[i*2+1] = force;
    }
    return f;
}

// render the system (ie draw the particles)
void PendulumSystem::draw(GLProgram& gl) {
    const Vector3f PENDULUM_COLOR(0.73f, 0.0f, 0.83f);
    gl.updateMaterial(PENDULUM_COLOR);

    // example code. Replace with your own drawing  code
    for (int i=0; i < (int)bodies.size(); ++i) {
        gl.updateModelMatrix(Matrix4f::translation(origin + m_vVecState[i*2]));
        drawSphere(0.075f, 10, 10);
    }
}
