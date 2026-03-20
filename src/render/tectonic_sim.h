#pragma once
#include <glad/glad.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include "../math/math3d.h"

namespace Tectonic {

struct Plate {
    Vec3 pos;       // Seed position on unit sphere
    Vec3 eulerPole; // Axis of rotation
    float omega;    // Rotation speed
    float baseElev; // Base height (0.05 for all initially)
    float sizeWeight; 
    float age;      // Randomized age (0.0 to 1.0)
    float density;  // Derived from age (older = denser)
};

class TectonicSimulator {
public:
    int width, height;
    std::vector<float> gridHeight;
    std::vector<int> gridPlate;
    std::vector<Plate> plates;
    GLuint textureID = 0;

    TectonicSimulator(int w, int h) : width(w), height(h) {
        gridHeight.assign(w * h, 0.0f);
        gridPlate.assign(w * h, -1);
    }

    void initializePlates(int count) {
        plates.clear();
        std::mt19937 gen(1337);
        std::uniform_real_distribution<float> dist(0, 1);
        
        // ALL plates start as Oceanic (0.05) per user request
        // 1. 4 Super-Giant Oceanic Plates
        for (int i = 0; i < 4; i++) {
            float theta = dist(gen) * 2.0f * 3.14159f;
            float phi = acosf(dist(gen) * 2.0f - 1.0f);
            Vec3 p(cosf(theta) * sinf(phi), cosf(phi), sinf(theta) * sinf(phi));
            Vec3 axis(dist(gen) * 2 - 1, dist(gen) * 2 - 1, dist(gen) * 2 - 1);
            float omega = (dist(gen) * 0.08f + 0.02f);
            float sWeight = 4.5f + dist(gen) * 2.0f;
            float age = 0.5f + dist(gen) * 0.5f; // Generally older
            plates.push_back({p, axis.normalized(), omega, 0.05f, sWeight, age, 1.0f + age * 0.5f});
        }

        // 2. 4 Large Plates
        for (int i = 0; i < 4; i++) {
            float theta = dist(gen) * 2.0f * 3.14159f;
            float phi = acosf(dist(gen) * 2.0f - 1.0f);
            Vec3 p(cosf(theta) * sinf(phi), cosf(phi), sinf(theta) * sinf(phi));
            Vec3 axis(dist(gen) * 2 - 1, dist(gen) * 2 - 1, dist(gen) * 2 - 1);
            float omega = (dist(gen) * 0.06f + 0.04f);
            float sWeight = 2.5f + dist(gen) * 1.5f;
            float age = dist(gen) * 0.8f; 
            plates.push_back({p, axis.normalized(), omega, 0.05f, sWeight, age, 1.0f + age * 0.5f});
        }

        // 3. 8 Small-Medium Mixed
        for (int i = 0; i < 8; i++) {
            float theta = dist(gen) * 2.0f * 3.14159f;
            float phi = acosf(dist(gen) * 2.0f - 1.0f);
            Vec3 p(cosf(theta) * sinf(phi), cosf(phi), sinf(theta) * sinf(phi));
            Vec3 axis(dist(gen) * 2 - 1, dist(gen) * 2 - 1, dist(gen) * 2 - 1);
            float omega = (dist(gen) * 0.15f + 0.05f);
            float sWeight = 0.5f + dist(gen) * 1.2f;
            float age = dist(gen);
            plates.push_back({p, axis.normalized(), omega, 0.05f, sWeight, age, 1.0f + age * 0.5f});
        }
    }

    Vec3 rotateVector(Vec3 v, Vec3 axis, float angle) {
        float c = cosf(angle);
        float s = sinf(angle);
        return v * c + axis.cross(v) * s + axis * axis.dot(v) * (1.0f - c);
    }

    float sampleHeight(Vec3 p) {
        float phi = acosf(std::clamp(p.y, -1.0f, 1.0f));
        float theta = atan2f(p.z, p.x);
        if (theta < 0) theta += 2.0f * 3.14159f;
        float fx = theta / (2.0f * 3.14159f) * (width - 1);
        float fy = phi / 3.14159f * (height - 1);
        int x0 = (int)fx, y0 = (int)fy;
        int x1 = (x0 + 1) % width, y1 = std::min(height - 1, y0 + 1);
        float tx = fx - x0, ty = fy - y0;
        float h00 = gridHeight[y0 * width + x0];
        float h10 = gridHeight[y0 * width + x1];
        float h01 = gridHeight[y1 * width + x0];
        float h11 = gridHeight[y1 * width + x1];
        return h00*(1-tx)*(1-ty) + h10*tx*(1-ty) + h01*(1-tx)*ty + h11*tx*ty;
    }

    void advectHeight(float dt) {
        std::vector<float> nextHeight = gridHeight;
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int pIdx = gridPlate[y * width + x];
                if (pIdx < 0) continue;
                Vec3 pos = getSphericalPos(x, y);
                // Where did this content come from? Reverse rotation of the plate.
                Vec3 prevPos = rotateVector(pos, plates[pIdx].eulerPole, -plates[pIdx].omega * dt * 0.5f);
                nextHeight[y * width + x] = sampleHeight(prevPos);
            }
        }
        gridHeight = nextHeight;
    }

    void lloydRelaxation(int iterations) {
        for (int iter = 0; iter < iterations; iter++) {
            std::vector<Vec3> centroids(plates.size(), Vec3(0, 0, 0));
            std::vector<int> counts(plates.size(), 0);
            for (int y = 0; y < height; y++) {
                float phi = (float)y / (height - 1) * 3.14159f;
                for (int x = 0; x < width; x++) {
                    float theta = (float)x / (width - 1) * 2.0f * 3.14159f;
                    Vec3 p(cosf(theta) * sinf(phi), cosf(phi), sinf(theta) * sinf(phi));
                    int bestIdx = 0;
                    float minD = 1e10f;
                    for (int i = 0; i < (int)plates.size(); i++) {
                        float dist = (p - plates[i].pos).lengthSq();
                        if (dist < minD) { minD = dist; bestIdx = i; }
                    }
                    centroids[bestIdx] = centroids[bestIdx] + p;
                    counts[bestIdx]++;
                }
            }
            for (int i = 0; i < (int)plates.size(); i++) {
                if (counts[i] > 0) plates[i].pos = (centroids[i] * (1.0f / counts[i])).normalized();
            }
        }
    }

    void updatePlateMapWithWarping() {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                Vec3 p = getSphericalPos(x, y);
                int idx1 = -1, idx2 = -1;
                float d1 = 1e10f, d2 = 1e10f;

                for (int i = 0; i < (int)plates.size(); i++) {
                    // WEIGHTED DISTANCE logic: d = (dist + noise) / sizeWeight
                    float noiseDist = (simpleNoise(p*8.0f) - 0.5f) * 0.12f; 
                    float d = ((p - plates[i].pos).length() + noiseDist) / plates[i].sizeWeight;
                    
                    if (d < d1) { d2 = d1; idx2 = idx1; d1 = d; idx1 = i; }
                    else if (d < d2) { d2 = d; idx2 = i; }
                }

                // Continuous Interpolation
                float blend = 1.0f - smoothstep(0.0f, 0.20f, d2 - d1);
                float h1 = plates[idx1].baseElev;
                float h2 = (idx2 != -1) ? plates[idx2].baseElev : h1;
                
                gridPlate[y * width + x] = idx1; 
                gridHeight[y * width + x] = h1 * (1.0f - blend * 0.5f) + h2 * (blend * 0.5f);
            }
        }
    }

    void updateHeightFromForces(float dt) {
        std::vector<float> deltaHeight(width * height, 0.0f);
        const float CONT_THRESHOLD = 0.45f; // Height above which it's considered stable continent
        
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;
                int p1 = gridPlate[idx];
                Vec3 pPos = getSphericalPos(x, y);
                Vec3 v1 = plates[p1].eulerPole.cross(pPos) * plates[p1].omega;

                auto handleBoundary = [&](int i1, int i2, int nx, int ny, bool isY) {
                    int p2 = gridPlate[i2];
                    if (p1 == p2) return;

                    Vec3 nPos = getSphericalPos(nx, ny);
                    Vec3 v2 = plates[p2].eulerPole.cross(nPos) * plates[p2].omega;
                    Vec3 norm = (pPos - nPos).normalized();
                    float force = (v1 - v2).dot(norm);

                    if (force < -0.01f) { // Convergence (Subduction or Collision)
                        float h1 = gridHeight[i1];
                        float h2 = gridHeight[i2];
                        bool isC1 = h1 > CONT_THRESHOLD;
                        bool isC2 = h2 > CONT_THRESHOLD;

                        int overridingIdx = -1;
                        int subductingIdx = -1;

                        if (isC1 && !isC2) { overridingIdx = i1; subductingIdx = i2; }
                        else if (!isC1 && isC2) { overridingIdx = i2; subductingIdx = i1; }
                        else if (!isC1 && !isC2) { // Ocean-Ocean: Younger (lighter) plate overrides
                            if (plates[p1].density < plates[p2].density) { overridingIdx = i1; subductingIdx = i2; }
                            else { overridingIdx = i2; subductingIdx = i1; }
                        } else { // Continent-Continent: Collision (No subduction, creates mountains and slows plates)
                            float uplift = -force * 2.8f * dt;
                            deltaHeight[i1] += uplift;
                            deltaHeight[i2] += uplift;
                            // DRAG: Large continents slow down plate motion
                            plates[p1].omega *= 0.985f;
                            plates[p2].omega *= 0.985f;
                            return;
                        }

                        // Subduction Physics:
                        // 1. Arc Volcanism & Permanent Felsic accumulation (Increased width and rate)
                        float felsicRate = 0.045f * dt; 
                        deltaHeight[overridingIdx] += -force * 1.8f * dt; // Immediate Orogeny
                        
                        // Apply growth to a small radius to simulate the wide volcanic arc / mantle wedge
                        int ox = overridingIdx % width;
                        int oy = overridingIdx / width;
                        for (int dy=-1; dy<=1; dy++) {
                            for (int dx=-1; dx<=1; dx++) {
                                int sIdx = std::clamp(oy+dy, 0, height-1) * width + ((ox+dx+width)%width);
                                deltaHeight[sIdx] += felsicRate;
                            }
                        }
                        
                        // 2. Trench formation (Subducting side)
                        deltaHeight[subductingIdx] += force * 0.8f * dt; 
                    }
                    else if (force > 0.01f) { // Divergence (Rifting)
                        float rift = -force * 0.45f * dt;
                        deltaHeight[i1] += rift;
                        deltaHeight[i2] += rift;
                    }
                };

                // X-direction sampling
                handleBoundary(idx, y * width + (x + 1) % width, (x + 1) % width, y, false);
                // Y-direction sampling
                if (y < height - 1) handleBoundary(idx, (y + 1) * width + x, x, y + 1, true);
            }
        }
        for (int i = 0; i < width * height; i++) {
            gridHeight[i] += deltaHeight[i];
            gridHeight[i] = std::clamp(gridHeight[i], 0.02f, 0.95f);
        }
    }

    void handleReorganization() {
        std::mt19937 gen(999);
        std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
        for (auto& p : plates) {
            // If continental drag has slowed the plate significantly, trigger reorganization (Rifting)
            if (p.omega < 0.02f) {
                p.eulerPole = Vec3(dist(gen), dist(gen), dist(gen)).normalized();
                p.omega = 0.05f + (dist(gen) + 1.0f) * 0.05f;
            }
        }
    }

    float simpleNoise(Vec3 p) {
        return (sinf(p.x * 12.9898f + p.y * 78.233f + p.z * 45.123f) * 43758.5453123f) - floorf(sinf(p.x * 12.9898f + p.y * 78.233f + p.z * 45.123f) * 43758.5453123f);
    }

    Vec3 getSphericalPos(int x, int y) {
        float phi = (float)y / (height - 1) * 3.14159f;
        float theta = (float)x / (width - 1) * 2.0f * 3.14159f;
        return Vec3(cosf(theta) * sinf(phi), cosf(phi), sinf(theta) * sinf(phi));
    }

    float smoothstep(float edge0, float edge1, float x) {
        float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }

    void smoothHeight(int passes) {
        for (int p = 0; p < passes; p++) {
            std::vector<float> temp = gridHeight;
            for (int y = 1; y < height - 1; y++) {
                for (int x = 0; x < width; x++) {
                    float sum = 0.0f;
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            int nx = (x + dx + width) % width;
                            sum += temp[(y + dy) * width + nx];
                        }
                    }
                    gridHeight[y * width + x] = sum / 9.0f;
                }
            }
        }
    }

    void bake() {
        if (textureID == 0) glGenTextures(1, &textureID);
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, width, height, 0, GL_RED, GL_FLOAT, gridHeight.data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }

    void simulate(int generations) {
        // Reduced to 12 plates for cleaner, larger landmass building
        initializePlates(12);
        lloydRelaxation(4);
        updatePlateMapWithWarping();
        
        float dt = 2.0f; // Larger time step for faster geological drift
        // Increased iterations to allow for full Wilson cycle (Birth to Supercontinent)
        int totalGens = 180; 
        
        for (int gen = 0; gen < totalGens; gen++) {
            // 1. Move plate seeds
            for (auto& p : plates) {
                p.pos = rotateVector(p.pos, p.eulerPole, p.omega * dt);
            }
            // 2. Move existing terrain (Advection)
            advectHeight(dt);
            // 3. Update plate boundaries based on new seed positions
            updatePlateMapWithWarping();
            // 4. Crustal growth and tectonic forces at new boundaries
            updateHeightFromForces(dt);
            // 5. Plate reorganization (Less frequent to allow stable subduction)
            if (gen > 0 && gen % 45 == 0) handleReorganization();
        }

        // Final detailing
        for (int i = 0; i < width * height; i++) {
            Vec3 p = getSphericalPos(i % width, i / width);
            float detail = (simpleNoise(p * 25.0f) - 0.5f) * 0.04f;
            gridHeight[i] += detail;
        }
        smoothHeight(1);
        bake();
    }
};

} // namespace Tectonic
