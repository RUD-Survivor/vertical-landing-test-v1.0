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
    std::vector<Vec3> hotspots; // Stationary mantle plumes
    GLuint textureID = 0;
    static constexpr float CONT_THRESHOLD = 0.45f;

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
        float phi = acosf(std::clamp(p.y, -1.0, 1.0));
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

    void updatePlateMapWithWarping(float time) {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                Vec3 p = getSphericalPos(x, y);
                // Fractal Noise Warping to break circular boundaries
                float n1 = simpleNoise(p * 4.0f + Vec3(time*0.1f, 0, 0)) * 0.15f;
                float n2 = simpleNoise(p * 12.0f - Vec3(0, time*0.1f, 0)) * 0.05f;
                Vec3 warpedP = (p + Vec3(n1, n2, n1)).normalized();

                int idx1 = -1;
                float d1 = 1e10f;
                for (int i = 0; i < (int)plates.size(); i++) {
                    float d = (warpedP - plates[i].pos).lengthSq() / (plates[i].sizeWeight * plates[i].sizeWeight);
                    if (d < d1) { d1 = d; idx1 = i; }
                }
                gridPlate[y * width + x] = idx1; 
            }
        }
    }

    void handleReorganization() {
        std::mt19937 gen(1337);
        std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
        
        // Detect "Trapped Plates" - Plates with too much convergence
        // Not implemented as a full counting pass for performance, instead randomizing seeds
        // to encourage breaking the current state.
        for (int i = 0; i < (int)plates.size(); i++) {
            auto& p = plates[i];
            // If plate is too slow or stuck, kick it to a new orientation
            if (p.omega < 0.035f || (rand() % 100 < 5)) {
                p.eulerPole = Vec3(dist(gen), dist(gen), dist(gen)).normalized();
                p.omega = 0.04f + (dist(gen) + 1.0f) * 0.06f;
                // Move seed slightly to break enclosure patterns
                p.pos = (p.pos + Vec3(dist(gen)*0.2f, dist(gen)*0.2f, dist(gen)*0.2f)).normalized();
            }
        }
    }

    void updateHeightFromForces(float dt, int gen) {
        std::vector<float> deltaHeight(width * height, 0.0f);
        
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;
                int p1 = gridPlate[idx];
                Vec3 pPos = getSphericalPos(x, y);
                Vec3 v1 = plates[p1].eulerPole.cross(pPos) * plates[p1].omega;

                auto handleBoundary = [&](int i1, int i2, int nx, int ny) {
                    int p2 = gridPlate[i2];
                    if (p1 == p2) return;

                    Vec3 nPos = getSphericalPos(nx, ny);
                    Vec3 v2 = plates[p2].eulerPole.cross(nPos) * plates[p2].omega;
                    Vec3 norm = (pPos - nPos).normalized();
                    Vec3 relativeV = v1 - v2;
                    float force = relativeV.dot(norm);

                    if (force < -0.015f) { // Convergence
                        float h1 = gridHeight[i1], h2 = gridHeight[i2];
                        bool isC1 = h1 > CONT_THRESHOLD, isC2 = h2 > CONT_THRESHOLD;
                        int overridingIdx = -1, subductingIdx = -1;

                        if (isC1 && !isC2) { overridingIdx = i1; subductingIdx = i2; }
                        else if (!isC1 && isC2) { overridingIdx = i2; subductingIdx = i1; }
                        else if (!isC1 && !isC2) { 
                            if (plates[p1].density < plates[p2].density) { overridingIdx = i1; subductingIdx = i2; }
                            else { overridingIdx = i2; subductingIdx = i1; }
                        } else { // Continent-Continent collision: 5x5 WELDING
                            float uplift = -force * 6.5f * dt; 
                            deltaHeight[i1] += uplift; deltaHeight[i2] += uplift;
                            
                            int cx = i1 % width, cy = i1 / width;
                            for (int dy=-2; dy<=2; dy++) {
                                for (int dx=-2; dx<=2; dx++) {
                                    int nIdx = std::clamp(cy+dy, 0, height-1) * width + ((cx+dx+width)%width);
                                    deltaHeight[nIdx] += uplift * (1.0f / (1.2f + dx*dx + dy*dy));
                                }
                            }
                            plates[p1].omega *= 0.94f; plates[p2].omega *= 0.94f; 
                            plates[p1].pos = (plates[p1].pos + norm * 0.006f).normalized(); 
                            plates[p2].pos = (plates[p2].pos - norm * 0.006f).normalized();
                            return;
                        }

                        // ANGLE-BASED ACCRETION
                        float convergenceAngle = relativeV.normalized().dot(-norm);
                        if (convergenceAngle > 0.65f) { 
                             float subductionEfficiency = simpleNoise(pPos * 25.0f + Vec3((float)gen, 0, 0));
                             if (subductionEfficiency > 0.35f) { 
                                 float felsicRate = 0.17f * dt * (subductionEfficiency - 0.25f); 
                                 deltaHeight[overridingIdx] += -force * 4.8f * dt; 
                                 
                                 int ox = overridingIdx % width, oy = overridingIdx / width;
                                 for (int dy=-1; dy<=1; dy++) { // Reverting to 3x3 for sharper spines
                                     for (int dx=-1; dx<=1; dx++) {
                                         int sIdx = std::clamp(oy+dy, 0, height-1) * width + ((ox+dx+width)%width);
                                         deltaHeight[sIdx] += felsicRate;
                                     }
                                 }
                             }
                        } else {
                             deltaHeight[overridingIdx] += -force * 0.85f * dt; 
                        }
                        deltaHeight[subductingIdx] += force * 0.95f * dt; 
                    }
                    else if (force > 0.012f) { // Divergence (Spreading Ridge)
                        bool isLand = gridHeight[i1] > CONT_THRESHOLD || gridHeight[i2] > CONT_THRESHOLD;
                        float riftStrength = isLand ? 0.32f : 0.98f; 
                        float rift = -force * riftStrength * dt;
                        deltaHeight[i1] += rift; deltaHeight[i2] += rift;
                        
                        plates[p1].pos = (plates[p1].pos + norm * (force * 0.006f)).normalized();
                        plates[p2].pos = (plates[p2].pos - norm * (force * 0.006f)).normalized();
                    }
                };

                handleBoundary(idx, y * width + (x + 1) % width, (x + 1) % width, y);
                if (y < height - 1) handleBoundary(idx, (y + 1) * width + x, x, y + 1);
            }
        }

        // Sedimentary Infilling (Natural Inland Sea Closure)
        // Instead of a hard count, use a height-based weight to "ooze" land into lakes
        for (int i = 0; i < width * height; i++) {
            if (gridHeight[i] < CONT_THRESHOLD) {
                int cx = i % width, cy = i / width;
                float sumNeighborHeight = 0.0f;
                int countLand = 0;
                for (int dy=-1; dy<=1; dy++) {
                    for (int dx=-1; dx<=1; dx++) {
                        int nx = (cx + dx + width) % width;
                        int ny = std::clamp(cy + dy, 0, height-1);
                        float nh = gridHeight[ny * width + nx];
                        if (nh > CONT_THRESHOLD) {
                             sumNeighborHeight += nh;
                             countLand++;
                        }
                    }
                }
                if (countLand >= 4) {
                     // Gradient Sedimentation: Shallow area becomes shallower
                     float avgH = sumNeighborHeight / countLand;
                     deltaHeight[i] += (avgH - gridHeight[i]) * 0.075f * dt;
                }
            }
        }


        // Continental Lateral Broadening (The "Ooze" factor)
        // Helps turn lines into blobs
        for (int i = 0; i < width * height; i++) {
            if (gridHeight[i] > CONT_THRESHOLD) {
                int cx = i % width, cy = i / width;
                // Recruit neighbors
                int nx = (cx + (rand()%3-1) + width) % width;
                int ny = std::clamp(cy + (rand()%3-1), 0, height-1);
                int nIdx = ny * width + nx;
                if (gridHeight[nIdx] < CONT_THRESHOLD) 
                    deltaHeight[nIdx] += 0.02f * dt;
            }
        }

        for (int i = 0; i < width * height; i++) {
            gridHeight[i] += deltaHeight[i];
            gridHeight[i] = std::clamp(gridHeight[i], 0.02f, 0.95f);
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

    void applyHotspots(float dt, int gen) {
        for (const auto& hPos : hotspots) {
            // Hotspots are stationary in the mantle frame. 
            // We need to find which pixel is currently AT this fixed coordinate.
            float phi = acosf(std::clamp(hPos.y, -1.0, 1.0));
            float theta = atan2f(hPos.z, hPos.x);
            if (theta < 0) theta += 2.0f * 3.14159f;

            int y = (int)(phi / 3.14159f * (height - 1));
            int x = (int)(theta / (2.0f * 3.14159f) * (width - 1));
            
            // Periodic Eruption: Don't grow constantly, grow in pulses every few Ma
            if ((gen + (int)(hPos.x * 10)) % 5 == 0) {
                float intensity = 0.38f * dt; // Taller peaks (up to 2500m+)
                // Sharp peak falloff: mostly 1 pixel, minor 3x3 base
                for (int dy=-1; dy<=1; dy++) {
                    for (int dx=-1; dx<=1; dx++) {
                        int nx = (x + dx + width) % width;
                        int ny = std::clamp(y + dy, 0, height-1);
                        float distSq = (float)(dx*dx + dy*dy);
                        gridHeight[ny * width + nx] += intensity * expf(-distSq * 2.5f); // Ultra-sharp Gaussian
                    }
                }
            }
        }
    }

    void simulate(int generations) {
        // High-fidelity 1Ma-resolution simulation (50 Steps)
        initializePlates(12); 
        lloydRelaxation(4);
        updatePlateMapWithWarping(0);

        // Initialize stationary mantle hotspots
        hotspots.clear();
        std::mt19937 hGen(88); // Different seed for variety
        std::uniform_real_distribution<float> hDist(0, 1);
        for (int i = 0; i < 4; i++) {
            float hTheta = hDist(hGen) * 2.0f * 3.14159f;
            float hPhi = acosf(hDist(hGen) * 2.0f - 1.0f);
            hotspots.push_back(Vec3(cosf(hTheta) * sinf(hPhi), cosf(hPhi), sinf(hTheta) * sinf(hPhi)));
        }
        
        float dt = 1.2f; 
        int totalGens = 80; 
        
        for (int gen = 0; gen < totalGens; gen++) {
            for (auto& p : plates) p.pos = rotateVector(p.pos, p.eulerPole, p.omega * dt);
            advectHeight(dt);
            updatePlateMapWithWarping((float)gen);
            updateHeightFromForces(dt * 3.5f, gen); 
            applyHotspots(dt * 2.5f, gen); 
            
            // Global Tectonic Sinking / Erosion (Age-based shrinkage)
            for (int i = 0; i < width * height; i++) {
                float h = gridHeight[i];
                if (h > 0.15f) { // All oceanic/island crust above basaltic floor
                    // Islands/Arcs (below 0.6) sink faster as they age
                    // Continental cores (above 0.6) are isostatically stable
                    float sinkRate = (h < 0.6f) ? 0.0065f : 0.0015f; 
                    gridHeight[i] = std::max(0.05f, h - sinkRate * dt);
                }
            }
            
            if (gen > 0 && gen % 25 == 0) handleReorganization();
        }

        // Final Post-Simulation Refinement: Shredding thick islands into finer chains
        std::vector<float> refinedHeight = gridHeight;
        for (int i = 0; i < width * height; i++) {
            float h = gridHeight[i];
            if (h > CONT_THRESHOLD) {
                Vec3 p = getSphericalPos(i % width, i / width);
                // Selective Erosion: Carve into wide landmasses using fractal "shredding" noise
                float shred = simpleNoise(p * 50.0f) * 0.15f; 
                float boundaryNoise = simpleNoise(p * 15.0f) * 0.1f;
                
                // Shredding target: Only thin coastlines and young island arcs (0.45-0.55)
                if (h < 0.55f) { 
                    refinedHeight[i] -= (shred + boundaryNoise);
                }
            }
        }
        gridHeight = refinedHeight;

        // Final detailing tailored for 50-step runs
        for (int i = 0; i < width * height; i++) {
            Vec3 p = getSphericalPos(i % width, i / width);
            // High-frequency craggy noise
            float detail = (simpleNoise(p * 55.0f) - 0.5f) * 0.035f;
            gridHeight[i] += detail;
        }
        smoothHeight(1);
        bake();
    }
};

} // namespace Tectonic
