#ifndef WINDOWSYSTEM_H
#define WINDOWSYSTEM_H

#include <map>
#include <set>
#include <vector>
#include <vecmath.h>

#include "droplet.h"
#include "particlesystem.h"
#include "Image.h"

using namespace std;

class WindowSystem : public ParticleSystem {
public:
    // Constructor, Destructor
    WindowSystem(
            Vector3f origin_ = Vector3f(-2.5f, -2.5f, 0.f),
            float size_ = 5.0f,
            float granularity_ = 0.01f,
            float raininess_ = 0.05f,
            vector<float> dropletSize_ = vector<float>({0.f, 1.3f})
            );
    ~WindowSystem() {};


    // Static Constants
    static const float G_NORM;
    static const Vector3f G_DIR;

    // Helper Observers
    vector<int> getGridIdx(Vector3f pos);
    Vector3f getGridPos(vector<int> idx);

    vector<int> clipIdx(vector<int> idx);
    map<int, Vector3f> evalAccel() override;

    // State Mutators
    void resetIdMap();
    void resetHeightMap();
    void resetAffinityMap();
    void addDroplet(float mass, Vector3f pos, Vector3f vel);
    void takeStep(float stepSize) override;
    void blurHeightMap(float epsilon=0.01f);
    void erodeHeightMap(float factor=0.5f);

    // Debug Helpers
    void debugIdMap();
    void debugHeightMap();
    void debugAffinityMap();
    void debugDroplets();

    // OpenGL function
    Vector3f computeNormal(int y, int x);
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

    // TODO: Change rep to Image classes
    vector<vector<int>> idMap;
    vector<vector<float>> heightMap;
    vector<vector<float>> affinityMap;

    // Droplet Represenation
    float raininess;                // probability of a droplet appearing on the grid
    vector<float> dropletSize;      // range of masses droplet can have

    map<int, Droplet *> droplets;
    int maxDropletIdx;

    int frameNo;
};


#endif
