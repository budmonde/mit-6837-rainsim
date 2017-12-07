#include "windowsystem.h"

#include <cfloat>
#include <iostream>

#include "camera.h"
#include "vertexrecorder.h"

const float GRAVITY = 1.f;

WindowSystem::WindowSystem(
        Vector3f origin_,
        float size_,
        float granularity_,
        float raininess_,
        vector<float> droplet_sz_) :
    origin(origin_),
    size(size_),
    granularity(granularity_),
    raininess(raininess_),
    droplet_sz(droplet_sz_) {
    // initing random seed
    srand(time(0));

    maxGridIdx = (int)floor(size / granularity);

    resetIDMap();

    maxDropletIdx = -1;
}

void WindowSystem::resetIDMap() {
    IDMap = vector<vector<int>>(maxGridIdx, vector<int>(maxGridIdx, -1));
}

void WindowSystem::debugIDMap() {
    cout << "Height: " << IDMap.size() << endl;
    cout << "Width: " << IDMap[0].size() << endl;

    for (int y=IDMap.size()-1; y >= 0; --y) {
        for (int cell : IDMap[y]) {
            cout << cell << " ";
        }
        cout << endl;
    }
}

void WindowSystem::takeStep() {

    // Droplet Generation
    if (rand_uniform(0.f, 1.f) < raininess) {
        ++maxDropletIdx;
        int droplet_idx = maxDropletIdx;
        float mass = rand_uniform(droplet_sz[0], droplet_sz[1]);
        Vector3f pos = Vector3f(rand_uniform(0.f, size), rand_uniform(0.f, size), 0.f);
        //Vector3f pos = Vector3f(size/2, rand_uniform(0.f, size), 0.f);

        droplets.insert(pair <int, Droplet *> (droplet_idx, new Droplet(droplet_idx, mass)));
        posState.insert(pair <int, Vector3f> (droplet_idx, pos));
        velState.insert(pair <int, Vector3f> (droplet_idx, Vector3f::ZERO));
    }

    // Delete clipped droplets
    for (const auto& it : droplets) {
        int i = it.first;
        if (posState[i].y() < 0.f || posState[i].y() > size ||
                posState[i].x() < 0.f || posState[i].x() > size) {

            delete droplets[i];
            droplets.erase(i);
            posState.erase(i);
            velState.erase(i);
        }
    }

    // Update IDMap
    resetIDMap();
    vector<int> toErase;
    for (const auto& it : droplets) {
        int i = it.first;
        Vector3f pos = posState[i];
        vector<int> gridIdx = getGridIdx(pos);
        int dropletIdx = IDMap[gridIdx[0]][gridIdx[1]];
        if (dropletIdx == -1) {
            IDMap[gridIdx[0]][gridIdx[1]] = i;
        } else {
            // combine droplet into pre-existing droplet
            droplets[dropletIdx]->mass += droplets[i]->mass;
            // TODO: do actual collision equation for new velocity calculation
            velState[dropletIdx] =
                velState[i].abs() > velState[dropletIdx].abs() ? velState[i] : velState[dropletIdx];
            posState[dropletIdx] =
                posState[i].y() < posState[dropletIdx].y() ? posState[i] : posState[dropletIdx];
            toErase.push_back(i);
        }
    }
    for (int i : toErase) {
        delete droplets[i];
        droplets.erase(i);
        posState.erase(i);
        velState.erase(i);
    }
}

map<int, Vector3f> WindowSystem::evalAccel(map<int, Vector3f> posState, map<int, Vector3f> velState) {
    map<int, Vector3f> accel;
    for (const auto& it : droplets) {
        int i = it.first;
        float accelNorm = 0.f;
        accelNorm += GRAVITY * droplets[i]->mass;
        accelNorm += -GRAVITY * min(droplets[i]->mass, droplets[i]->STATIC_MASS);
        accelNorm /= droplets[i]->mass;

        vector<int> gridIdx = getGridIdx(posState[i]);

        float maxMass = 0.f;
        int bestX = (int)floor(rand_uniform(0.f, 3.f));
        for (int x=0; x < 3; ++x) {

            vector<int> gi({gridIdx[0]-2, gridIdx[1]-3+x*2});
            vector<int> gr({gi[0]+3, gi[1]+3});
            vector<int> clipped_gi = clipIdx(gi);
            vector<int> clipped_gr = clipIdx(gr);


            float mass = 0.f;
            for (int fy=clipped_gi[0]; fy < clipped_gr[0]; ++fy) {
                for (int fx=clipped_gi[1]; fx < clipped_gr[1]; ++fx) {
                    if (IDMap[fy][fx] != -1) {
                        mass += droplets[IDMap[fy][fx]]->mass;
                    }
                }
            }
            if (mass > maxMass) {
                maxMass = mass;
                bestX = x;
            }
        }

        Vector3f accelDir = -Vector3f::UP + Vector3f::RIGHT * (bestX - 1);

        accel[i] = accelDir * accelNorm;
    }
    return accel;
}

vector<int> WindowSystem::getGridIdx(Vector3f pos) {
    return vector<int>({
            (int)floor(pos.y()/granularity),
            (int)floor(pos.x()/granularity)
    });
}

vector<int> WindowSystem::clipIdx(vector<int> idx) {
    return vector<int>({
            max(min(idx[0],maxGridIdx),0),
            max(min(idx[1],maxGridIdx),0),
    });
}

void WindowSystem::draw(GLProgram& gl) {
    const Vector3f DROPLET_COLOR(0.9f, 0.9f, 0.9f);
    gl.updateMaterial(DROPLET_COLOR);
    gl.updateModelMatrix(Matrix4f::translation(origin));

    VertexRecorder rec;
    for (const auto& it : droplets) {
        int i = it.first;
        gl.updateModelMatrix(Matrix4f::translation(origin+posState[i]));
        drawSphere(0.075f, 10, 10);
    }
    rec.draw();
}

