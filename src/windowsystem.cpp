#include "windowsystem.h"

#include <cfloat>
#include <iostream>

#include "camera.h"
#include "vertexrecorder.h"

WindowSystem::WindowSystem(
        Vector3f origin_,
        float size_,
        float granularity_,
        float raininess_,
        vector<float> dropletSize_) :
    origin(origin_),
    size(size_),
    granularity(granularity_),
    raininess(raininess_),
    dropletSize(dropletSize_) {
    // initing random seed
    srand(time(0));

    // set object attributes
    gridSize = (int)floor(size / granularity);
    resetIdMap();
    maxDropletIdx = -1;
}

const float WindowSystem::G_NORM = 1.f;
const Vector3f WindowSystem::G_DIR = Vector3f(0.f, -1.f, 0.f);

void WindowSystem::resetIdMap() {
    idMap = vector<vector<int>>(gridSize, vector<int>(gridSize, -1));
}

void WindowSystem::addDroplet(float mass, Vector3f pos, Vector3f vel) {
    ++maxDropletIdx;
    int droplet_idx = maxDropletIdx;
    droplets.insert(pair <int, Droplet *> (droplet_idx, new Droplet(droplet_idx, mass)));
    posState.insert(pair <int, Vector3f> (droplet_idx, pos));
    velState.insert(pair <int, Vector3f> (droplet_idx, vel));
}


Vector3f WindowSystem::collisionVelocity(int i, int j) {
    float mi = droplets[i]->mass,
          mj = droplets[j]->mass,
          vi = velState[i].abs(),
          vj = velState[j].abs();
    float velNorm = sqrt((mi*vi*vi+mj*vj*vj)/(mi+mj));
    Vector3f velDir = (velState[i] + velState[j]);
    velDir = velDir == Vector3f::ZERO ? Vector3f::ZERO : velDir.normalized();
    return velDir*velNorm;
}

vector<int> WindowSystem::getGridIdx(Vector3f pos) {
    return vector<int>({
            (int)floor(pos.y()/granularity),
            (int)floor(pos.x()/granularity)
    });
}

vector<int> WindowSystem::clipIdx(vector<int> idx) {
    return vector<int>({
            max(min(idx[0],gridSize-1),0),
            max(min(idx[1],gridSize-1),0),
    });
}

map<int, Vector3f> WindowSystem::evalAccel() {
    map<int, Vector3f> accel;
    for (const auto& it : droplets) {
        int i = it.first;

        // calculate external forces
        Vector3f extAccel = Vector3f::ZERO;
        extAccel += G_DIR * G_NORM * droplets[i]->mass;
        if (velState[i] != Vector3f::ZERO)
            extAccel += -velState[i].normalized() * G_NORM * droplets[i]->STATIC_MASS;
        else
            extAccel += G_DIR * G_NORM * droplets[i]->mass;
        extAccel /= droplets[i]->mass;

        // calculate droplet "tug" forces
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
                    if (idMap[fy][fx] != -1) {
                        mass += droplets[idMap[fy][fx]]->mass;
                    }
                }
            }
            if (mass > maxMass) {
                maxMass = mass;
                bestX = x;
            }
        }

        Vector3f accelDir = Vector3f::RIGHT * (bestX - 1);
        float accelNorm = 1.f;
        accel[i] = accelDir * accelNorm + extAccel;
    }
    return accel;
}

void WindowSystem::takeStep(float stepSize) {

    // Generate new droplets
    if (rand_uniform(0.f, 1.f) < raininess) {
        float mass = rand_uniform(dropletSize[0], dropletSize[1]);
        Vector3f pos = Vector3f(rand_uniform(0.f, size), rand_uniform(0.f, size), 0.f);
        Vector3f vel = Vector3f::ZERO;
        addDroplet(mass, pos, vel);
    }

    // Generate residual droplets
    for (const auto& it : droplets) {
        int i = it.first;
        if (droplets[i]->mass >= droplets[i]->STATIC_MASS) {
            droplets[i]->split_time += stepSize;
            if (rand_uniform(0.f, 1.f) < droplets[i]->splitProb(stepSize)) {
                // add new droplet
                float mass = min(droplets[i]->STATIC_MASS, rand_uniform(0.1f, 0.3f)*droplets[i]->mass);
                // TODO: magic number
                Vector3f pos = posState[i] - velState[i] * stepSize * 20;
                Vector3f vel = Vector3f::ZERO;
                addDroplet(mass, pos, vel);

                // update OG droplet
                droplets[i]->mass -= mass;
                droplets[i]->split_time = 0.f;
            }
        }
    }

    // Apply movement to existing droplets
    map<int, Vector3f> accelState = evalAccel();

    for (const auto& it : droplets) {
        int i = it.first;
        posState[i] += velState[i] * stepSize;
        velState[i] += accelState[i] * stepSize;
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

    // Update idMap and detect merging
    resetIdMap();
    vector<int> toErase;
    for (const auto& it : droplets) {
        int i = it.first;
        Vector3f pos = posState[i];
        vector<int> gridIdx = getGridIdx(pos);
        int dropletIdx = idMap[gridIdx[0]][gridIdx[1]];
        if (dropletIdx == -1) {
            idMap[gridIdx[0]][gridIdx[1]] = i;
        } else {
            // combine droplet into pre-existing droplet
            droplets[dropletIdx]->mass += droplets[i]->mass;
            posState[dropletIdx] =
                posState[i].y() < posState[dropletIdx].y() ? posState[i] : posState[dropletIdx];
            velState[dropletIdx] = collisionVelocity(i, dropletIdx);
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

void WindowSystem::debugIdMap() {
    cout << "Height: " << idMap.size() << endl;
    cout << "Width: " << idMap[0].size() << endl;

    for (int y=idMap.size()-1; y >= 0; --y) {
        for (int cell : idMap[y]) {
            cout << cell << " ";
        }
        cout << endl;
    }
}

void WindowSystem::debugDroplets() {
    cout << "###Droplets Debug###" << endl << endl;
    for (const auto& it : droplets) {
        int i = it.first;
        cout << "\t";
        droplets[i]->print();
        cout << "\t";
        posState[i].print();
        cout << "\t";
        velState[i].print();
        cout << endl;
    }
}


void WindowSystem::draw(GLProgram& gl) {
    const Vector3f DROPLET_COLOR(0.9f, 0.9f, 0.9f);
    gl.updateMaterial(DROPLET_COLOR);
    gl.updateModelMatrix(Matrix4f::translation(origin));

    VertexRecorder rec;
    for (const auto& it : droplets) {
        int i = it.first;
        gl.updateModelMatrix(Matrix4f::translation(origin+posState[i]));
        drawSphere(cbrt(droplets[i]->mass*.0005f), 10, 10);
    }
    rec.draw();
}

