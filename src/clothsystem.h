#ifndef CLOTHSYSTEM_H
#define CLOTHSYSTEM_H

#include <vector>

#include "body.h"
#include "particlesystem.h"

class ClothSystem : public ParticleSystem
{
    ///ADD MORE FUNCTION AND FIELDS HERE
public:
    ClothSystem(Vector3f origin_, int width_, int height_, float step_, float spring_k_);

    // evalF is called by the integrator at least once per time step
    std::vector<Vector3f> evalF(std::vector<Vector3f> state) override;

    // toggle wind
    void toggleWind() { wind = !wind; };

    // move point to coordinate
    void movePoint(int idx, Vector3f pos);

    // find closest point to pressed screen location
    int findClosest(float x, float y);

    // draw is called once per frame
    void draw(GLProgram& ctx);

protected:
    // helper methods
    int getIdx(int x, int y) { return y*width+x; }
    vector<int> getCoords(int idx) { return vector<int>{idx / width, idx % width}; }
    Vector3f computeNormal(int x, int y);

    // inherits
    // std::vector<Vector3f> m_vVecState;
    vector<Body *> bodies;
    Vector3f origin;
    int width;
    int height;
    float step;
    float spring_k;
    bool wind;
};


#endif
