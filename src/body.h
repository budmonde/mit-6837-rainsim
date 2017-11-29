#ifndef BODY_H
#define BODY_H

#include <vector>
#include <vecmath.h>
#include <cstdint>

using namespace std;

class Body {
public:
    Body(int idx_, bool stat_=false, float mass_=1.f, float drag_=1.f);

    // Add a spring connection to the body
    void addSpring(Body * body, float len, float ks);
    void print();

    // representation
    int idx;
    bool stat;
    float mass;
    float drag;
    // spring meta
    vector<Body *> s_bodies;
    vector<float> s_ks;
    vector<float> s_drs;
};

void connectBodies(Body * body1, Body * body2, float len, float ks=10.f);

#endif
