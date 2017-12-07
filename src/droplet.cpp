#include "droplet.h"

#include <iostream>

Droplet::Droplet(int idx_, float mass_) : idx(idx_), mass(mass_) { }

const float Droplet::STATIC_MASS = 1.f;

void Droplet::print() {
    cout << endl;
    cout << "Droplet " << idx << " at " << this << endl;
    cout << "Mass: " << mass << endl;
}

