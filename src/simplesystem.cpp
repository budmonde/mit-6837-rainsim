#include "simplesystem.h"

#include "camera.h"
#include "vertexrecorder.h"

using namespace std;

SimpleSystem::SimpleSystem(Vector3f origin_, float radius_, float speed_) : origin(origin_), radius(radius_), speed(speed_) {
    Vector3f state(radius, 0.f, 0.f);
    m_vVecState.push_back(state);
}

vector<Vector3f> SimpleSystem::evalF(vector<Vector3f> state) {
    vector<Vector3f> f;
    Vector3f dstate(-state[0].y()*speed, state[0].x()*speed, 0.f);
    f.push_back(dstate);
    return f;
}

// render the system (ie draw the particles)
void SimpleSystem::draw(GLProgram& gl)
{

    // TODO 3.2: draw the particle. 
    //           we provide code that draws a static sphere.
    //           you should replace it with your own
    //           drawing code.
    //           In this assignment, you must manage two
    //           kinds of uniforms before you draw
    //            1. Update material uniforms (color)
    //            2. Update transform uniforms
    //           GLProgram is a helper object that has
    //           methods to set the uniform state.

    const Vector3f PARTICLE_COLOR(0.4f, 0.7f, 1.0f);
    gl.updateMaterial(PARTICLE_COLOR);
    Vector3f pos = getState()[0];
    gl.updateModelMatrix(Matrix4f::translation(origin + pos));
    drawSphere(0.075f, 10, 10);
}
