#ifndef DROPLET_H
#define DROPLET_H

#include <vector>
#include <vecmath.h>
#include <cstdint>

using namespace std;

class Droplet {
public:
    // Constructor, Destructor
    Droplet(int idx_, float mass_, float granularity_);
    ~Droplet() {};

    // Static Constants
    static const float MAX_SPLIT_TIME;
    static const float STATIC_MASS;

    // Static Helpers
    const float radius(float m);

    // Helper Observers
    float splitProb(float stepSize);
    vector<Vector3f> getOffsetChain();
    vector<float> getDist();

    // Debug Helpers
    void print();

    // representation
    int idx;
    float mass;
    vector<int> offset_chain_idx;
    vector<Vector3f> OFFSET_DOMAIN;
    vector<float> dist;
    float split_time;
};

#endif
