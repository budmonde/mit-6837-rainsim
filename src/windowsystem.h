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
            float granularity_ = 0.1f,
            float raininess_ = 0.005f,
            vector<float> dropletSize_ = vector<float>({1.f, 2.f})
            );

    static const float G_NORM;
    static const Vector3f G_DIR;


    Vector3f collisionVelocity(int i, int j);

    vector<int> getGridIdx(Vector3f pos);
    vector<int> clipIdx(vector<int> idx);

    void resetIDMap();
    void debugIDMap();
    void debugDroplets();

    void addDroplet(float mass, Vector3f pos, Vector3f vel);

    void takeStep(float stepSize) override;

    map<int, Vector3f> evalAccel() override;

    // draw is called once per frame
    void draw(GLProgram& ctx);

protected:
    // inherits
    // vector<Vector3f> posState;
    // vector<Vector3f> velState;
    Vector3f origin;

    float size;
    float granularity;
    int gridSize;

    vector<vector<int>> IDMap;

    float raininess;
    vector<float> dropletSize;

    map<int, Droplet *> droplets;
    int maxDropletIdx;
};


#endif
