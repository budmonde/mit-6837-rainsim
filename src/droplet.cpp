#include "droplet.h"
#include "particlesystem.h"

#include <iostream>


Droplet::Droplet(int idx_, float mass_, float granularity_) : idx(idx_), mass(mass_) {
    split_time = 0.f;
    int N;
    if (mass < STATIC_MASS) {
        N = 3;
    } else {
        N = 1;
    }
    OFFSET_DOMAIN = vector<Vector3f>({
            Vector3f::RIGHT * granularity_,
            Vector3f::RIGHT * granularity_ - Vector3f::UP * granularity_,
            -Vector3f::UP * granularity_,
            -Vector3f::RIGHT * granularity_ - Vector3f::UP * granularity_,
            -Vector3f::RIGHT * granularity_,
    });
    float leftover_dist = 1.f;
    for (int i=0; i<N; ++i) {
        if (i != 0) {
            if (offset_chain_idx[i-1] == 0) {
                offset_chain_idx.push_back((int)floor(rand_uniform(0.f, 4.f)));
            } else if (offset_chain_idx[i-1] == 4) {
                offset_chain_idx.push_back((int)floor(rand_uniform(1.f, 5.f)));
            } else {
                offset_chain_idx.push_back((int)floor(rand_uniform(0.f, 5.f)));
            }
        } else {
            offset_chain_idx.push_back((int)floor(rand_uniform(0.f, 5.f)));
        }
        if (i != N-1) {
            float curr_dist = rand_uniform(0.f, leftover_dist);
            leftover_dist -= curr_dist;
            dist.push_back(curr_dist);
        } else {
            dist.push_back(leftover_dist);
        }
    }
}

const float Droplet::MAX_SPLIT_TIME = .4f;
const float Droplet::STATIC_MASS = 1.f;

float Droplet::splitProb(float stepSize) {
    return min(1.f, 3.f*stepSize/MAX_SPLIT_TIME*min(1.f, split_time/MAX_SPLIT_TIME));
}

const float Droplet::radius(float m) {
    return cbrt(m*.0005f);
}

vector<Vector3f> Droplet::getOffsetChain() {
    vector<Vector3f> out;
    for (int i=0; i<(int)offset_chain_idx.size(); ++i) {
        out.push_back(OFFSET_DOMAIN[offset_chain_idx[i]]);
    }
    return out;
}

vector<float> Droplet::getDist() {
    return dist;
}

void Droplet::print() {
    cout << "Droplet " << idx << " with mass " << mass << " at " << this << endl;
    for (int i=0; i<(int)offset_chain_idx.size(); ++i) {
        cout << "offset vector";
        getOffsetChain()[i].print();
        cout << "mass p: " << getDist()[i] << endl;
    }
}

