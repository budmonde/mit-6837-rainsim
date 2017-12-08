#ifndef DROPLET_H
#define DROPLET_H

#include <vector>
#include <vecmath.h>
#include <cstdint>

using namespace std;

class Droplet {
public:
    Droplet(int idx_, float mass_=1.f);
    ~Droplet() {};

    float splitProb(float stepSize);

    static const float MAX_SPLIT_TIME;
    static const float STATIC_MASS;

    // Add a spring connection to the body
    void print();

    // representation
    int idx;
    float mass;
    float split_time;
};

#endif
