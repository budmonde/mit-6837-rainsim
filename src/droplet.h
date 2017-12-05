#ifndef DROPLET_H
#define DROPLET_H

#include <vector>
#include <vecmath.h>
#include <cstdint>

using namespace std;

class Droplet {
public:
    Droplet(int idx_, float mass_=1.f);

    // Add a spring connection to the body
    void print();

    // representation
    int idx;
    float mass;
};

#endif
