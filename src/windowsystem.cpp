#include "windowsystem.h"

#include <cfloat>
#include <iostream>
#include <iomanip>

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
    resetHeightMap();
    resetAffinityMap();
    maxDropletIdx = -1;
    frameNo = 0;

    // Debug Droplet generation
    //addDroplet(0.9f, Vector3f(1.6f, 1.5f, 0.f), Vector3f::ZERO);
    //addDroplet(0.9f, Vector3f(1.4f, 1.5f, 0.f), Vector3f::ZERO);
    //addDroplet(2.5f, Vector3f(1.5f, 1.8f, 0.f), Vector3f::ZERO);

}

const float WindowSystem::G_NORM = 1.f;
const Vector3f WindowSystem::G_DIR = Vector3f(0.f, -1.f, 0.f);

void WindowSystem::resetIdMap() {
    idMap = vector<vector<int>>(gridSize, vector<int>(gridSize, -1));
}

void WindowSystem::resetHeightMap() {
    heightMap = vector<vector<float>>(gridSize, vector<float>(gridSize, 0.f));
}

void WindowSystem::resetAffinityMap() {
    affinityMap = vector<vector<float>>(gridSize, vector<float>(gridSize, 0.f));
    for (int y=0; y<gridSize; ++y) {
        for (int x=0; x<gridSize; ++x) {
            affinityMap[y][x] = rand_uniform(0.f, 1.f);
        }
    }
}

void WindowSystem::addDroplet(float mass, Vector3f pos, Vector3f vel) {
    ++maxDropletIdx;
    int droplet_idx = maxDropletIdx;
    droplets.insert(pair <int, Droplet *> (droplet_idx, new Droplet(droplet_idx, mass, granularity)));
    // for debugging
    Vector3f aligned_pos = getGridPos(getGridIdx(pos));
    posState.insert(pair <int, Vector3f> (droplet_idx, aligned_pos));
    velState.insert(pair <int, Vector3f> (droplet_idx, vel));
}

vector<int> WindowSystem::getGridIdx(Vector3f pos) {
    return vector<int>({
            (int)floor(pos.y()/granularity),
            (int)floor(pos.x()/granularity)
    });
}

Vector3f WindowSystem::getGridPos(vector<int> idx) {
    return Vector3f(granularity*(idx[1]+0.5f), granularity*(idx[0]+0.5f), 0.f);
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
        float maxAffinity = 0.f;
        int bestX = 1;//(int)floor(rand_uniform(0.f, 3.f));
        int bestAX = (int)floor(rand_uniform(0.f, 3.f));

        // check which direction has more water
        for (int x=0; x < 3; ++x) {

            vector<int> gi({gridIdx[0]-2, gridIdx[1]-3+x*2});
            vector<int> gr({gi[0]+3, gi[1]+3});
            vector<int> clipped_gi = clipIdx(gi);
            vector<int> clipped_gr = clipIdx(gr);

            float mass = 0.f;
            float affinity = 0.f;
            for (int fy=clipped_gi[0]; fy < clipped_gr[0]; ++fy) {
                for (int fx=clipped_gi[1]; fx < clipped_gr[1]; ++fx) {
                    if (idMap[fy][fx] != i)
                        mass += heightMap[fy][fx];
                    affinity += affinityMap[fy][fx];
                }
            }
            if (mass > maxMass) {
                maxMass = mass;
                bestX = x;
            }
            if (affinity > maxAffinity) {
                maxAffinity = affinity;
                bestAX = x;
            }
        }
        if (maxMass == 0.f) {
            bestX = bestAX;
        }

        Vector3f accelDir = Vector3f::RIGHT * (bestX - 1);
        float accelNorm = 1.f;
        accel[i] = accelDir * accelNorm + extAccel;
    }
    return accel;
}

void WindowSystem::takeStep(float stepSize) {

    ++frameNo;

    // Apply movement to existing droplets
    map<int, Vector3f> accelState = evalAccel();

    for (const auto& it : droplets) {
        int i = it.first;
        posState[i] += velState[i] * stepSize;
        velState[i] += accelState[i] * stepSize;
    }
    
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

    // Construct Height Map and idMap
    vector<set<int>> toMerge;
    map<int, int> setLookupTable;

    resetIdMap();

    for (const auto& it : droplets) {
        int i = it.first;
        Droplet * d = droplets[i];
        vector<Vector3f> offsetChain = d->getOffsetChain();

        Vector3f center = posState[i];
        for (int sd_i=0; sd_i<(int)offsetChain.size(); ++sd_i) {
            float r = d->radius(d->mass * d->getDist()[sd_i]);
            float rSq = r*r;
            center += offsetChain[sd_i];
            vector<int> lo = clipIdx(getGridIdx(center + Vector3f(-r, -r, 0.f)));
            vector<int> hi = clipIdx(getGridIdx(center + Vector3f(r, r, 0.f)));
            for (int y=lo[0]; y <= hi[0]; ++y) {
                for (int x=lo[1]; x <= hi[1]; ++x) {
                    float heightSq = rSq - (getGridPos(vector<int>({y, x})) - center).absSquared();
                    if (heightSq > 0 && heightSq > pow(heightMap[y][x],2)) {
                        heightMap[y][x] = sqrt(heightSq);
                        if (idMap[y][x] != -1) {
                            int j = idMap[y][x];
                            bool iSetExists = setLookupTable.find(i) != setLookupTable.end();
                            bool jSetExists = setLookupTable.find(j) != setLookupTable.end();
                            int setIdx;
                            if (iSetExists) {
                                setIdx = setLookupTable[i];
                            } else if (jSetExists) {
                                setIdx = setLookupTable[j];
                            } else {
                                toMerge.push_back(set<int>());
                                setIdx = toMerge.size() - 1;
                            }
                            toMerge[setIdx].insert(idMap[y][x]);
                            toMerge[setIdx].insert(i);
                            setLookupTable[idMap[y][x]] = setIdx;
                            setLookupTable[i] = setIdx;
                        }
                        idMap[y][x] = i;
                    }
                }
            }
        }
    }

    // find adjacent idmaps to merge
    for (int y=1; y<gridSize-1; ++y) {
        for (int x=1; x<gridSize-1; ++x) {
            if (idMap[y][x] == -1) continue;

            for (int fy=-1; fy<2; ++fy) {
                for (int fx=-1; fx<2; ++fx) {
                    if (fy == 0 && fx == 0) continue;
                    if (idMap[y+fy][x+fx] == -1) continue;
                    if (idMap[y+fy][x+fx] == idMap[y][x]) continue;
                    // neighbor merge between the two should happen
                    int i = idMap[y][x],
                        j = idMap[y+fy][x+fx];
                    bool iSetExists = setLookupTable.find(i) != setLookupTable.end();
                    bool jSetExists = setLookupTable.find(j) != setLookupTable.end();
                    int setIdx;
                    if (iSetExists) {
                        setIdx = setLookupTable[i];
                    } else if (jSetExists) {
                        setIdx = setLookupTable[j];
                    } else {
                        toMerge.push_back(set<int>());
                        setIdx = toMerge.size() - 1;
                    }
                    toMerge[setIdx].insert(i);
                    toMerge[setIdx].insert(j);
                    setLookupTable[i] = setIdx;
                    setLookupTable[j] = setIdx;
                }
            }
        }
    }

    // Clean up IDMap
    set<int> dirtyIds;
    for (const auto& blob : toMerge) {
        dirtyIds.insert(blob.begin(), blob.end());
    }

    for (int y=0; y<gridSize; ++y) {
        for (int x=0; x<gridSize; ++x) {
            if (dirtyIds.find(idMap[y][x]) != dirtyIds.end()) {
                idMap[y][x] = -1;
            }
        }
    }
    //cout << "DEBUG" << endl;
    //for (const auto& blob : toMerge) {
    //    for (const auto& idx : blob) {
    //        cout << idx << "  ";
    //    }
    //    cout << endl;
    //}
    //for (const auto& foo : setLookupTable) {
    //    cout << foo.first << "->" << foo.second << endl;
    //}
    //cout << "DEBUG END" << endl;

    for (const auto& blob : toMerge) {
        // Calculate state for new droplet
        float mass = 0.f;
        Vector3f pos(0.f, FLT_MAX, 0.f);
        Vector3f vel = Vector3f::ZERO;
        for (const auto& idx : blob) {
            mass += droplets[idx]->mass;
            pos = posState[idx].y() < pos.y() ? posState[idx] : pos;
            vel += droplets[idx]->mass * velState[idx];

            delete droplets[idx];
            droplets.erase(idx);
            posState.erase(idx);
            velState.erase(idx);
        }
        vel *= 1.6f / mass;
        // Init new drops
        addDroplet(mass, pos, vel);
        // Update idMap for the new droplet
        int i = maxDropletIdx;
        float r = droplets[i]->radius(droplets[i]->mass);
        float rSq = r*r;
        vector<int> lo = clipIdx(getGridIdx(posState[i] + Vector3f(-r, -r, 0.f)));
        vector<int> hi = clipIdx(getGridIdx(posState[i] + Vector3f(r, r, 0.f)));
        for (int y=lo[0]; y < hi[0]; ++y) {
            for (int x=lo[1]; x < hi[1]; ++x) {
                float heightSq = rSq - (getGridPos(vector<int>({y, x})) - posState[i]).absSquared();
                if (heightSq > 0 && heightSq > pow(heightMap[y][x],2)) {
                    idMap[y][x] = i;
                }
            }
        }
    }

    // Blur Height Map
    blurHeightMap();
    erodeHeightMap();

    // Store Height Map
    Image im(gridSize, gridSize, 1);
    for (int y=0; y<gridSize; ++y) {
        for (int x=0; x<gridSize; ++x) {
            im(x,gridSize-1-y) = heightMap[y][x] * 20;
        }
    }
    ostringstream fname;
    fname << "../Output/heightmap";
    fname << setfill('0') << setw(4);
    fname << frameNo;
    fname << ".png";
    cout << fname.str() << endl;
    im.write(fname.str());

}

void WindowSystem::blurHeightMap(float epsilon) {
    vector<vector<float>> newMap(heightMap.begin(), heightMap.end());
    for (int y=0; y<gridSize; ++y) {
        for (int x=0; x<gridSize; ++x) {
            float newHeight = 0.f;
            for (int fx=-1; fx<2; ++fx) {
                int xx = max(0, min(gridSize-1, x+fx));
                newHeight += heightMap[y][xx];
            }
            newMap[y][x] = newHeight / 3.f;
        }
    }
    for (int y=0; y<gridSize; ++y) {
        for (int x=0; x<gridSize; ++x) {
            float newHeight = 0.f;
            for (int fy=-1; fy<2; ++fy) {
                int yy = max(0, min(gridSize-1, y+fy));
                newHeight += newMap[yy][x];
            }
            heightMap[y][x] = newHeight / 3.f;
        }
    }
    for (int y=0; y<gridSize; ++y) {
        for (int x=0; x<gridSize; ++x) {
            heightMap[y][x] = heightMap[y][x] >= epsilon ? heightMap[y][x] : 0.f;
        }
    }
}

void WindowSystem::erodeHeightMap(float factor) {
    for (int y=0; y<gridSize; ++y) {
        vector<float> row(heightMap[y].begin(), heightMap[y].end());
        for (int x=0; x<gridSize; ++x) {
            if (idMap[y][x] != -1) continue;
            if (row[x] == 0.f) continue;
            bool erodeRight = (x == 0 || row[x-1] == 0.f);
            bool erodeLeft = (x == gridSize-1 || row[x+1] == 0.f);
            if (erodeRight && erodeLeft) {
                heightMap[y][x] = 0.f;
            } else if (erodeRight) {
                if (x<gridSize-1) heightMap[y][x+1] += row[x] * factor;
                heightMap[y][x] = 0.f;
            } else if (erodeLeft) {
                if (x>0) heightMap[y][x-1] += row[x] * factor;
                heightMap[y][x] = 0.f;
            }

        }
    }
}

void WindowSystem::debugIdMap() {
    cout << "Height: " << idMap.size() << endl;
    cout << "Width: " << idMap[0].size() << endl;

    for (int y=idMap.size()-1; y >= 0; --y) {
        for (int cell : idMap[y]) {
            if (cell == -1) {
                cout << "- ";
            } else {
                cout << cell << " ";
            }
        }
        cout << endl;
    }
}

void WindowSystem::debugHeightMap() {
    cout << "Height: " << heightMap.size() << endl;
    cout << "Width: " << heightMap[0].size() << endl;

    for (int y=heightMap.size()-1; y >= 0; --y) {
        for (float cell : heightMap[y]) {
            cout << (int)ceil(cell) << " ";
        }
        cout << endl;
    }
}

void WindowSystem::debugAffinityMap() {
    cout << "Height: " << affinityMap.size() << endl;
    cout << "Width: " << affinityMap[0].size() << endl;

    for (int y=affinityMap.size()-1; y >= 0; --y) {
        for (float cell : affinityMap[y]) {
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

Vector3f WindowSystem::computeNormal(int y, int x) {
    float heightDiff[2] = {
        heightMap[y][min(x+1,gridSize-1)] - heightMap[y][max(x-1,0)],
        heightMap[min(y+1,gridSize-1)][x] - heightMap[max(y-1,0)][x]
    };
    Vector3f a = Vector3f(granularity*2, 0.f, heightDiff[0]);
    Vector3f b = Vector3f(0.f, granularity*2, heightDiff[1]);
    Vector3f out = Vector3f::cross(a,b).normalized();
    return out;
}


void WindowSystem::draw(GLProgram& gl) {
    const Vector3f DROPLET_COLOR(0.7f, 0.7f, 0.7f);
    gl.updateMaterial(DROPLET_COLOR);
    //gl.updateModelMatrix(Matrix4f::translation(origin));

    //VertexRecorder rec;
    //for (const auto& it : droplets) {
    //    int i = it.first;
    //    Droplet * d = droplets[i];
    //    Vector3f center = posState[i];
    //    vector<Vector3f> offsetChain = d->getOffsetChain();
    //    for (int sd_i=0; sd_i<(int)offsetChain.size(); ++sd_i) {
    //        float r = d->radius(d->mass * d->getDist()[sd_i]);
    //        center += offsetChain[sd_i];
    //        gl.updateModelMatrix(Matrix4f::translation(origin+center-Vector3f::FORWARD));
    //        drawSphere(r, 10, 10);
    //    }
    //}
    //rec.draw();

    gl.updateModelMatrix(Matrix4f::translation(origin));
    VertexRecorder rec2;
    for (int y=1; y < gridSize; ++y) {
        for (int x=1; x < gridSize; ++x) {

            Vector3f d = getGridPos(vector<int>({y,x})) - Vector3f::FORWARD*heightMap[y][x];
            Vector3f c = getGridPos(vector<int>({y,x-1})) - Vector3f::FORWARD*heightMap[y][x-1];
            Vector3f b = getGridPos(vector<int>({y-1,x})) - Vector3f::FORWARD*heightMap[y-1][x];
            Vector3f a = getGridPos(vector<int>({y-1,x-1})) - Vector3f::FORWARD*heightMap[y-1][x-1];
            rec2.record(d, computeNormal(y,x));
            rec2.record(b, computeNormal(y-1,x));
            rec2.record(a, computeNormal(y-1,x-1));

            rec2.record(d, computeNormal(y,x));
            rec2.record(a, computeNormal(y-1,x-1));
            rec2.record(c, computeNormal(y,x-1));
        }
    }
    rec2.draw();
}

