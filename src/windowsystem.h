#ifndef WINDOWSYSTEM_H
#define WINDOWSYSTEM_H

#include <map>
#include <vector>
#include <vecmath.h>

#include "droplet.h"
#include "particlesystem.h"

using namespace std;

class WindowSystem : public ParticleSystem {
public:
    // Constructor, Destructor
    WindowSystem(
            Vector3f origin_ = Vector3f(-2.5f, -2.5f, 0.f),
            float size_ = 5.f,
            float granularity_ = 0.1f,
            float raininess_ = 0.005f,
            vector<float> dropletSize_ = vector<float>({1.f, 2.f})
            );
    ~WindowSystem() {};


    // Static Constants
    static const float G_NORM;
    static const Vector3f G_DIR;

    // Helper Observers
    Vector3f collisionVelocity(int i, int j);
    vector<int> getGridIdx(Vector3f pos);
    vector<int> clipIdx(vector<int> idx);
    map<int, Vector3f> evalAccel() override;

    // State Mutators
    void resetIdMap();
    void resetHeightMap();
    void addDroplet(float mass, Vector3f pos, Vector3f vel);
    void takeStep(float stepSize) override;

    // Debug Helpers
    void debugIdMap();
    void debugDroplets();

    // OpenGL function
    void draw(GLProgram& ctx);

protected:
    // inherits
    // vector<Vector3f> posState;
    // vector<Vector3f> velState;

    // OpenGL related vars
    Vector3f origin;

    // Grid Representation
    float size;                     // width of the entire grid
    float granularity;              // width of a grid cell
    int gridSize;                   // number of cells in a row

    vector<vector<int>> idMap;
    vector<vector<float>> heightMap;

    // Droplet Represenation
    float raininess;                // probability of a droplet appearing on the grid
    vector<float> dropletSize;      // range of masses droplet can have

    map<int, Droplet *> droplets;
    int maxDropletIdx;
};


#endif
