#ifndef PARTICLESYSTEM_H
#define PARTICLESYSTEM_H

#include <map>
#include <vector>
#include <vecmath.h>

#include <cstdint>

using namespace std;

// helper for uniform distribution
float rand_uniform(float low, float hi);

struct GLProgram;
class ParticleSystem {
public:
    virtual ~ParticleSystem() {}

    // take a step
    virtual void takeStep() {}

    // for a given state, evaluate derivative f(X,t)
    virtual map<int, Vector3f> evalAccel(map<int, Vector3f> posState, map<int, Vector3f> velState) = 0;

    // getter method for the system's state
    map<int, Vector3f> getState() { return m_vVecState; };
    map<int, Vector3f> getPositions() { return posState; };
    map<int, Vector3f> getVelocities() { return velState; };

    // setter method for the system's state
    void setState(const map<int, Vector3f>  & newState) { m_vVecState = newState; };
    void setPositions(const map<int, Vector3f>  & newPosState) { posState = newPosState; };
    void setVelocities(const map<int, Vector3f>  & newVelState) { velState = newVelState; };

 protected:
    map<int, Vector3f> m_vVecState;
    map<int, Vector3f> posState;
    map<int, Vector3f> velState;
};

/* GLProgram is a helper for updating uniform variables.
   Before drawing geometry, update the model matrix and diffuse color.

   You don't have to update the lighting uniforms (they are set at the
   beginning of the frame for you)
*/
class Camera;
struct GLProgram {
    // constructor
    GLProgram(uint32_t program_light, uint32_t program_color, Camera* camera);

    // Update the model matrix. View and projection matrix
    // are read from the camera.
	void updateModelMatrix(Matrix4f M) const;

    // Update material properties.
    // - The one argument version just sets the diffuse color
    // - With 2-3 arguments, also sets specular color
	void updateMaterial(Vector3f diffuseColor, 
        Vector3f ambientColor = Vector3f(-1, -1, -1),
        Vector3f specularColor = Vector3f(0, 0, 0), 
        float shininess = 1.0f,
        float alpha = 1.0f) const;

    // Update lighting. Sets position and color of a single light source
    // in world space.
	void updateLight(Vector3f pos, Vector3f color = Vector3f(1, 1, 1)) const;

    void enableLighting();
    void disableLighting();

private:
    // member variables
    uint32_t active_program;
    uint32_t program_light;
    uint32_t program_color;
    const Camera* camera;
};
#endif
