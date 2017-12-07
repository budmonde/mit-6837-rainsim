#ifndef WINDOWSYSTEM_H
#define WINDOWSYSTEM_H

#include <map>
#include <vector>
#include <vecmath.h>

#include "droplet.h"
#include "particlesystem.h"

using namespace std;

class WindowSystem : public ParticleSystem
{
    ///ADD MORE FUNCTION AND FIELDS HERE
public:
    WindowSystem(Vector3f origin_);

    void takeStep() override;

    map<int, Vector3f> evalAccel(map<int, Vector3f> posState, map<int, Vector3f> velState) override;

    // draw is called once per frame
    void draw(GLProgram& ctx);

protected:
    // inherits
    // vector<Vector3f> m_vVecState;
    // vector<Vector3f> posState;
    // vector<Vector3f> velState;
    Vector3f origin;
    map<int, Droplet *> droplets;
};


#endif
