#include "clothsystem.h"

#include <cfloat>
#include <iostream>

#include "camera.h"
#include "vertexrecorder.h"


ClothSystem::ClothSystem(Vector3f origin_, int width_, int height_, float step_, float spring_k_) : origin(origin_), width(width_), height(height_), step(step_), spring_k(spring_k_) { 

    float diag = pow(pow(step,2)*2,0.5f);
    wind = false;

    for (int y=0; y < height; ++y) {
        for (int x=0; x < width; ++x) {

            bool stat = (x == 0 || x == width-1) && y == 0;
            bodies.push_back(new Body(bodies.size(), stat));
            m_vVecState.push_back(Vector3f(x*step,-y*step,rand_uniform(-0.1f,0.1f)));
            m_vVecState.push_back(Vector3f(0.f,0.f,0.f));

            // structural springs
            if (x > 0)
                connectBodies(bodies[getIdx(x,y)], bodies[getIdx(x-1,y)], step, spring_k);
            if (y > 0)
                connectBodies(bodies[getIdx(x,y)], bodies[getIdx(x,y-1)], step, spring_k);

            // shear springs
            if (x > 0 && y > 0) {
                connectBodies(bodies[getIdx(x,y)], bodies[getIdx(x-1,y-1)], diag, spring_k);
                connectBodies(bodies[getIdx(x-1,y)], bodies[getIdx(x,y-1)], diag, spring_k);
            }

            // flex springs
            if (x > 1)
                connectBodies(bodies[getIdx(x,y)], bodies[getIdx(x-2,y)], step*2, spring_k);
            if (y > 1)
                connectBodies(bodies[getIdx(x,y)], bodies[getIdx(x,y-2)], step*2, spring_k);

        }
    }
}


vector<Vector3f> ClothSystem::evalF(vector<Vector3f> state) {

    vector<Vector3f> f(state.size());
    for (int i=0; i < (int) bodies.size(); ++i) {
        Body * b1 = bodies[i];
        Vector3f vel = state[i*2+1];
        Vector3f force = Vector3f::ZERO;

        if (!b1->stat) {
            force += -Vector3f::UP * b1->mass;
            force += vel * (-b1->drag);
            for (int j=0; j < (int) b1->s_bodies.size(); ++j) {
                Body * b2 = b1->s_bodies[j];
                Vector3f d = state[i*2] - state[b2->idx*2];
                force += d / d.abs() * -b1->s_ks[j] * (d.abs() - b1->s_drs[j]);
            }
            if (wind) {
                force += -Vector3f::FORWARD * rand_uniform(-1.0f,3.0f);
            }
        }
        f[i*2] = vel;
        f[i*2+1] = force;
    }
    return f;
}

void ClothSystem::movePoint(int idx, Vector3f pos) {
    m_vVecState[idx*2] = pos;
}

int ClothSystem::findClosest(float x, float y) {
    Vector3f pos(x,y,0.f);
    pos.print();
    float minDist = FLT_MAX;
    int best_idx;
    for (int i=0; i < (int) bodies.size(); ++i) {
        Vector3f dist = pos - m_vVecState[i*2];
        if (dist.abs() < minDist) {
            minDist = dist.abs();
            best_idx = i;
        }
    }
    return best_idx;
}

Vector3f ClothSystem::computeNormal(int x, int y) {
    int count = 0;
    Vector3f normals = Vector3f::ZERO;
    if (x > 0 && y > 0) {
        Vector3f a = m_vVecState[getIdx(x,y)*2] - m_vVecState[getIdx(x,y-1)*2];
        Vector3f b = m_vVecState[getIdx(x,y)*2] - m_vVecState[getIdx(x-1,y-1)*2];
        normals = normals + Vector3f::cross(a,b).normalized();

        a = m_vVecState[getIdx(x,y)*2] - m_vVecState[getIdx(x-1,y-1)*2];
        b = m_vVecState[getIdx(x,y)*2] - m_vVecState[getIdx(x-1,y)*2];
        normals = normals +Vector3f::cross(a,b).normalized();
        count += 2;
    } 
    if (x > 0 && y < height-1) {
        Vector3f a = m_vVecState[getIdx(x,y)*2] - m_vVecState[getIdx(x-1,y)*2];
        Vector3f b = m_vVecState[getIdx(x,y)*2] - m_vVecState[getIdx(x-1,y+1)*2];
        normals = normals + Vector3f::cross(a,b).normalized();

        a = m_vVecState[getIdx(x,y)*2] - m_vVecState[getIdx(x-1,y+1)*2];
        b = m_vVecState[getIdx(x,y)*2] - m_vVecState[getIdx(x,y+1)*2];
        normals = normals + Vector3f::cross(a,b).normalized();
        count += 2;
    }
    if (x < width-1 && y < height-1) {
        Vector3f a = m_vVecState[getIdx(x,y)*2] - m_vVecState[getIdx(x,y+1)*2];
        Vector3f b = m_vVecState[getIdx(x,y)*2] - m_vVecState[getIdx(x+1,y+1)*2];
        normals = normals + Vector3f::cross(a,b).normalized();

        a = m_vVecState[getIdx(x,y)*2] - m_vVecState[getIdx(x+1,y+1)*2];
        b = m_vVecState[getIdx(x,y)*2] - m_vVecState[getIdx(x+1,y)*2];
        normals = normals + Vector3f::cross(a,b).normalized();
        count += 2;
    }
    if (x < width-1 && y > 0) {
        Vector3f a = m_vVecState[getIdx(x,y)*2] - m_vVecState[getIdx(x+1,y)*2];
        Vector3f b = m_vVecState[getIdx(x,y)*2] - m_vVecState[getIdx(x+1,y-1)*2];
        normals = normals + Vector3f::cross(a,b).normalized();

        a = m_vVecState[getIdx(x,y)*2] - m_vVecState[getIdx(x+1,y-1)*2];
        b = m_vVecState[getIdx(x,y)*2] - m_vVecState[getIdx(x,y-1)*2];
        normals = normals + Vector3f::cross(a,b).normalized();
        count += 2;
    }
    return normals / count;
}

void ClothSystem::draw(GLProgram& gl) {
    const Vector3f CLOTH_COLOR(0.9f, 0.9f, 0.9f);
    gl.updateMaterial(CLOTH_COLOR);
    gl.updateModelMatrix(Matrix4f::translation(origin));

    VertexRecorder rec;
    for (int y=1; y < height; ++y) {
        for (int x=1; x < width; ++x) {

            rec.record(m_vVecState[getIdx(x,y)*2], computeNormal(x,y));
            rec.record(m_vVecState[getIdx(x,y-1)*2], computeNormal(x,y-1));
            rec.record(m_vVecState[getIdx(x-1,y-1)*2], computeNormal(x-1,y-1));

            rec.record(m_vVecState[getIdx(x,y)*2], computeNormal(x,y));
            rec.record(m_vVecState[getIdx(x-1,y-1)*2], computeNormal(x-1,y-1));
            rec.record(m_vVecState[getIdx(x-1,y)*2], computeNormal(x-1,y));
        }
    }
    rec.draw();
}

