#ifndef DROPLET_H
#define DROPLET_H

#include <vector>
#include <vecmath.h>
#include <cstdint>

using namespace std;

class Droplet {
public:
    // Constructor, Destructor
    Droplet(int idx_, float mass_=1.f);
    ~Droplet() {};

    // Static Constants
    static const float MAX_SPLIT_TIME;
    static const float STATIC_MASS;

    // Helper Observers
    float splitProb(float stepSize);
    float radius();

    // Debug Helpers
    void print();

    // representation
    int idx;
    float mass;
    float split_time;
};

#endif
