#ifndef GELATINSYSTEM_H
#define GELATINSYSTEM_H

#include <vector>

#include "body.h"
#include "particlesystem.h"

class GelatinSystem : public ParticleSystem
{
    ///ADD MORE FUNCTION AND FIELDS HERE
public:
    GelatinSystem(Vector3f origin_, int width_, int height_, int depth_, float step_, float spring_k_);

    // evalF is called by the integrator at least once per time step
    std::vector<Vector3f> evalF(std::vector<Vector3f> state) override;

    // draw is called once per frame
    void draw(GLProgram& ctx);

protected:
    // helper methods
    int getIdx(int x, int y, int z) { return z*width*height+y*width+x; }
    vector<int> getCoords(int idx) { return vector<int>{idx / (height*width), (idx % (height*width)) / width, (idx % (height*width)) % width}; }
    Vector3f computeNormalX(int x, int y, int z);
    Vector3f computeNormalY(int x, int y, int z);
    Vector3f computeNormalZ(int x, int y, int z);

    // inherits
    // std::vector<Vector3f> m_vVecState;
    vector<Body *> bodies;
    Vector3f origin;
    int width;
    int height;
    int depth;
    float step;
    float spring_k;
};


#endif
