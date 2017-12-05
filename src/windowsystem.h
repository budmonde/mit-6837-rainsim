#ifndef WINDOWSYSTEM_H
#define WINDOWSYSTEM_H

#include <map>
#include <vector>
#include <vecmath.h>

#include "droplet.h"
#include "particlesystem.h"

class WindowSystem : public ParticleSystem
{
    ///ADD MORE FUNCTION AND FIELDS HERE
public:
    WindowSystem(Vector3f origin_);

    void takeStep() override;

    // evalF is called by the integrator at least once per time step
    std::map<int, Vector3f> evalF(std::map<int, Vector3f> state) override;

    // draw is called once per frame
    void draw(GLProgram& ctx);

protected:
    // inherits
    // std::vector<Vector3f> m_vVecState;
    Vector3f origin;
    vector<Droplet *> droplets;
};


#endif
