#include "droplet.h"

#include <iostream>

Droplet::Droplet(int idx_, float mass_) : idx(idx_), mass(mass_) {
    split_time = 0.f;
}

const float Droplet::MAX_SPLIT_TIME = .4f;
const float Droplet::STATIC_MASS = 1.f;

float Droplet::splitProb(float stepSize) {
    return min(1.f, 3.f*stepSize/MAX_SPLIT_TIME*min(1.f, split_time/MAX_SPLIT_TIME));
}

void Droplet::print() {
    cout << "Droplet " << idx << " with mass " << mass << " at " << this << endl;
}

