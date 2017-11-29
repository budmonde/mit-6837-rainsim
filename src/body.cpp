#include "body.h"

#include <iostream>

Body::Body(int idx_, bool stat_, float mass_, float drag_) : idx(idx_), stat(stat_), mass(mass_), drag(drag_) { }

void Body::addSpring(Body * body, float len, float ks) {
    s_bodies.push_back(body);
    s_ks.push_back(ks);
    s_drs.push_back(len);
}

void connectBodies(Body * body1, Body * body2, float len, float ks) {
    body1->addSpring(body2, len, ks);
    body2->addSpring(body1, len, ks);
}

void Body::print() {
    cout << endl;
    if (stat)
        cout << "Static ";
    else
        cout << "Dynamic ";
    cout << "Body " << idx << " at " << this << endl;
    cout << "Mass: " << mass << ", Drag: " << drag << endl;
    for (int i=0; i < (int) s_bodies.size(); ++i) {
        cout << "Connected to body at: " << s_bodies[i] << " w/ d_0=" << s_drs[i] << " and k=" << s_ks[i] << endl;
    }
}

