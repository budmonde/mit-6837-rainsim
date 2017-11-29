#include "gelatinsystem.h"

#include <cfloat>
#include <iostream>

#include "camera.h"
#include "vertexrecorder.h"


GelatinSystem::GelatinSystem(Vector3f origin_, int width_, int height_, int depth_, float step_, float spring_k_) : origin(origin_), width(width_), height(height_), depth(depth_), step(step_), spring_k(spring_k_) { 

    float diag = pow(pow(step,2)*2,0.5f);
    float cross = pow(pow(diag,2)*2,0.5f);

    for (int z=0; z < depth; ++z) {
        for (int y=0; y < height; ++y) {
            for (int x=0; x < width; ++x) {

                bool stat = (y == 0);

                bodies.push_back(new Body(bodies.size(), stat));
                m_vVecState.push_back(Vector3f(x*step,y*step,z*step));
                m_vVecState.push_back(Vector3f(0.f,0.f,0.f));

                // structural springs
                if (x > 0)
                    connectBodies(bodies[getIdx(x,y,z)], bodies[getIdx(x-1,y,z)], step, spring_k);
                if (y > 0)
                    connectBodies(bodies[getIdx(x,y,z)], bodies[getIdx(x,y-1,z)], step, spring_k);
                if (z > 0)
                    connectBodies(bodies[getIdx(x,y,z)], bodies[getIdx(x,y,z-1)], step, spring_k);

                // shear springs
                if (x > 0 && y > 0) {
                    connectBodies(bodies[getIdx(x,y,z)], bodies[getIdx(x-1,y-1,z)], diag, spring_k);
                    connectBodies(bodies[getIdx(x-1,y,z)], bodies[getIdx(x,y-1,z)], diag, spring_k);
                }
                if (x > 0 && z > 0) {
                    connectBodies(bodies[getIdx(x,y,z)], bodies[getIdx(x-1,y,z-1)], diag, spring_k);
                    connectBodies(bodies[getIdx(x-1,y,z)], bodies[getIdx(x,y,z-1)], diag, spring_k);
                }
                if (y > 0 && z > 0) {
                    connectBodies(bodies[getIdx(x,y,z)], bodies[getIdx(x,y-1,z-1)], diag, spring_k);
                    connectBodies(bodies[getIdx(x,y-1,z)], bodies[getIdx(x,y,z-1)], diag, spring_k);
                }

                // cross springs
                if (x > 0 && y > 0 && z > 0) {
                    connectBodies(bodies[getIdx(x,y,z)], bodies[getIdx(x-1,y-1,z-1)], cross, spring_k);
                    connectBodies(bodies[getIdx(x,y,z-1)], bodies[getIdx(x-1,y-1,z)], cross, spring_k);
                    connectBodies(bodies[getIdx(x,y-1,z)], bodies[getIdx(x-1,y,z-1)], cross, spring_k);
                    connectBodies(bodies[getIdx(x,y-1,z-1)], bodies[getIdx(x-1,y,z)], cross, spring_k);
                }

                // flex springs
                if (x > 1)
                    connectBodies(bodies[getIdx(x,y,z)], bodies[getIdx(x-2,y,z)], step*2, spring_k);
                if (y > 1)
                    connectBodies(bodies[getIdx(x,y,z)], bodies[getIdx(x,y-2,z)], step*2, spring_k);
                if (z > 1)
                    connectBodies(bodies[getIdx(x,y,z)], bodies[getIdx(x,y,z-2)], step*2, spring_k);

            }
        }
    }
}


vector<Vector3f> GelatinSystem::evalF(vector<Vector3f> state) {

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
        }
        f[i*2] = vel;
        f[i*2+1] = force;
    }
    return f;
}

Vector3f GelatinSystem::computeNormalX(int x, int y, int z) {
    int count = 0;
    Vector3f normals = Vector3f::ZERO;
    if (z > 0 && y > 0) {
        Vector3f a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y-1,z)*2];
        Vector3f b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y-1,z-1)*2];
        normals = normals + Vector3f::cross(a,b).normalized();

        a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y-1,z-1)*2];
        b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y,z-1)*2];
        normals = normals +Vector3f::cross(a,b).normalized();
        count += 2;
    } 
    if (z > 0 && y < height-1) {
        Vector3f a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y,z-1)*2];
        Vector3f b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y+1,z-1)*2];
        normals = normals + Vector3f::cross(a,b).normalized();

        a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y+1,z-1)*2];
        b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y+1,z)*2];
        normals = normals + Vector3f::cross(a,b).normalized();
        count += 2;
    }
    if (z < depth-1 && y < height-1) {
        Vector3f a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y+1,z)*2];
        Vector3f b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y+1,z+1)*2];
        normals = normals + Vector3f::cross(a,b).normalized();

        a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y+1,z+1)*2];
        b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y,z+1)*2];
        normals = normals + Vector3f::cross(a,b).normalized();
        count += 2;
    }
    if (z < depth-1 && y > 0) {
        Vector3f a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y,z+1)*2];
        Vector3f b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y-1,z+1)*2];
        normals = normals + Vector3f::cross(a,b).normalized();

        a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y-1,z+1)*2];
        b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y-1,z)*2];
        normals = normals + Vector3f::cross(a,b).normalized();
        count += 2;
    }
    return normals / count;
}

Vector3f GelatinSystem::computeNormalY(int x, int y, int z) {
    int count = 0;
    Vector3f normals = Vector3f::ZERO;
    if (x > 0 && z > 0) {
        Vector3f a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y,z-1)*2];
        Vector3f b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x-1,y,z-1)*2];
        normals = normals + Vector3f::cross(a,b).normalized();

        a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x-1,y,z-1)*2];
        b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x-1,y,z)*2];
        normals = normals +Vector3f::cross(a,b).normalized();
        count += 2;
    } 
    if (x > 0 && z < depth-1) {
        Vector3f a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x-1,y,z)*2];
        Vector3f b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x-1,y,z+1)*2];
        normals = normals + Vector3f::cross(a,b).normalized();

        a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x-1,y,z+1)*2];
        b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y,z+1)*2];
        normals = normals + Vector3f::cross(a,b).normalized();
        count += 2;
    }
    if (x < width-1 && z < depth-1) {
        Vector3f a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y,z+1)*2];
        Vector3f b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x+1,y,z+1)*2];
        normals = normals + Vector3f::cross(a,b).normalized();

        a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x+1,y,z+1)*2];
        b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x+1,y,z)*2];
        normals = normals + Vector3f::cross(a,b).normalized();
        count += 2;
    }
    if (x < width-1 && z > 0) {
        Vector3f a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x+1,y,z)*2];
        Vector3f b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x+1,y,z-1)*2];
        normals = normals + Vector3f::cross(a,b).normalized();

        a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x+1,y,z-1)*2];
        b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y,z-1)*2];
        normals = normals + Vector3f::cross(a,b).normalized();
        count += 2;
    }
    return normals / count;
}

Vector3f GelatinSystem::computeNormalZ(int x, int y, int z) {
    int count = 0;
    Vector3f normals = Vector3f::ZERO;
    if (x > 0 && y > 0) {
        Vector3f a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y-1,z)*2];
        Vector3f b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x-1,y-1,z)*2];
        normals = normals + Vector3f::cross(a,b).normalized();

        a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x-1,y-1,z)*2];
        b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x-1,y,z)*2];
        normals = normals +Vector3f::cross(a,b).normalized();
        count += 2;
    } 
    if (x > 0 && y < height-1) {
        Vector3f a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x-1,y,z)*2];
        Vector3f b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x-1,y+1,z)*2];
        normals = normals + Vector3f::cross(a,b).normalized();

        a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x-1,y+1,z)*2];
        b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y+1,z)*2];
        normals = normals + Vector3f::cross(a,b).normalized();
        count += 2;
    }
    if (x < width-1 && y < height-1) {
        Vector3f a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y+1,z)*2];
        Vector3f b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x+1,y+1,z)*2];
        normals = normals + Vector3f::cross(a,b).normalized();

        a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x+1,y+1,z)*2];
        b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x+1,y,z)*2];
        normals = normals + Vector3f::cross(a,b).normalized();
        count += 2;
    }
    if (x < width-1 && y > 0) {
        Vector3f a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x+1,y,z)*2];
        Vector3f b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x+1,y-1,z)*2];
        normals = normals + Vector3f::cross(a,b).normalized();

        a = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x+1,y-1,z)*2];
        b = m_vVecState[getIdx(x,y,z)*2] - m_vVecState[getIdx(x,y-1,z)*2];
        normals = normals + Vector3f::cross(a,b).normalized();
        count += 2;
    }
    return -normals / count;
}
void GelatinSystem::draw(GLProgram& gl) {
    const Vector3f CLOTH_COLOR(1.0f, 1.0f, 0.0f);
    gl.updateMaterial(CLOTH_COLOR);
    gl.updateModelMatrix(Matrix4f::translation(origin));
    VertexRecorder rec;
    int xs[2] = {0, width-1};
    int ys[2] = {0, height-1};
    int zs[2] = {0, depth-1};
    for (int i=0; i < 2; ++i) {
        int z = zs[i];
        for (int y=1; y < height; ++y) {
            for (int x=1; x < width; ++x) {

                rec.record(m_vVecState[getIdx(x,y,z)*2], computeNormalZ(x,y,z));
                rec.record(m_vVecState[getIdx(x,y-1,z)*2], computeNormalZ(x,y-1,z));
                rec.record(m_vVecState[getIdx(x-1,y-1,z)*2], computeNormalZ(x-1,y-1,z));

                rec.record(m_vVecState[getIdx(x,y,z)*2], computeNormalZ(x,y,z));
                rec.record(m_vVecState[getIdx(x-1,y-1,z)*2], computeNormalZ(x-1,y-1,z));
                rec.record(m_vVecState[getIdx(x-1,y,z)*2], computeNormalZ(x-1,y,z));
            }
        }
    }
    for (int i=0; i < 2; ++i) {
        int y = ys[i];
        for (int z=1; z < depth; ++z) {
            for (int x=1; x < width; ++x) {

                rec.record(m_vVecState[getIdx(x,y,z)*2], computeNormalY(x,y,z));
                rec.record(m_vVecState[getIdx(x,y,z-1)*2], computeNormalY(x,y,z-1));
                rec.record(m_vVecState[getIdx(x-1,y,z-1)*2], computeNormalY(x-1,y,z-1));

                rec.record(m_vVecState[getIdx(x,y,z)*2], computeNormalY(x,y,z));
                rec.record(m_vVecState[getIdx(x-1,y,z-1)*2], computeNormalY(x-1,y,z-1));
                rec.record(m_vVecState[getIdx(x-1,y,z)*2], computeNormalY(x-1,y,z));
            }
        }
    }
    for (int i=0; i < 2; ++i) {
        int x = xs[i];
        for (int y=1; y < height; ++y) {
            for (int z=1; z < depth; ++z) {

                rec.record(m_vVecState[getIdx(x,y,z)*2], computeNormalX(x,y,z));
                rec.record(m_vVecState[getIdx(x,y-1,z)*2], computeNormalX(x,y-1,z));
                rec.record(m_vVecState[getIdx(x,y-1,z-1)*2], computeNormalX(x,y-1,z-1));

                rec.record(m_vVecState[getIdx(x,y,z)*2], computeNormalX(x,y,z));
                rec.record(m_vVecState[getIdx(x,y-1,z-1)*2], computeNormalX(x,y-1,z-1));
                rec.record(m_vVecState[getIdx(x,y,z-1)*2], computeNormalX(x,y,z-1));
            }
        }
    }
    rec.draw();
}

