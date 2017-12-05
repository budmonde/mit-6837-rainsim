#include "droplet.h"

#include <iostream>

Droplet::Droplet(int idx_, float mass_) : idx(idx_), mass(mass_) { }

void Droplet::print() {
    cout << endl;
    cout << "Droplet " << idx << " at " << this << endl;
    cout << "Mass: " << mass << endl;
}

