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
    WindowSystem(
            Vector3f origin_ = Vector3f(-2.5f, -2.5f, 0.f),
            float size_ = 5.f,
            float granularity_ = 0.5f,
            float raininess_ = 0.3f,
            vector<float> droplet_sz = vector<float>({0.f, 2.f})
            );

    vector<int> getGridIdx(Vector3f pos);
    vector<int> clipIdx(vector<int> idx);

    void resetIDMap();
    void debugIDMap();

    void takeStep() override;

    map<int, Vector3f> evalAccel(map<int, Vector3f> posState, map<int, Vector3f> velState) override;

    // draw is called once per frame
    void draw(GLProgram& ctx);

protected:
    // inherits
    // vector<Vector3f> posState;
    // vector<Vector3f> velState;
    Vector3f origin;

    float size;
    float granularity;
    int maxGridIdx;

    vector<vector<int>> IDMap;

    float raininess;
    vector<float> droplet_sz;

    map<int, Droplet *> droplets;
    int maxDropletIdx;
};


#endif
