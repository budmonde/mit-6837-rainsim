#ifndef PENDULUMSYSTEM_H
#define PENDULUMSYSTEM_H

#include <vector>

#include "body.h"
#include "particlesystem.h"

using namespace std;

class PendulumSystem : public ParticleSystem {
public:
    PendulumSystem(Vector3f origin_, int num_parts_, float step_sz_, float spring_k);

    std::vector<Vector3f> evalF(std::vector<Vector3f> state) override;
    void draw(GLProgram&);

protected:
    // inherits 
    // std::vector<Vector3f> m_vVecState;
    Vector3f origin;
    int num_parts;
    float step_sz;
    float spring_k;
    vector<Body *> bodies;
};

#endif
