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
    float baseElev; // Continental (0.6) or Oceanic (0.1)
};

class TectonicSimulator {
public:
    int width, height;
    std::vector<float> gridHeight;
    std::vector<int> gridPlate;
    std::vector<Plate> plates;

    TectonicSimulator(int w, int h) : width(w), height(h) {
        gridHeight.assign(w * h, 0.0f);
        gridPlate.assign(w * h, -1);
    }

    // Stage 1: Poisson-like Seeds & Lloyd Relaxation
    void initializePlates(int count) {
        plates.clear();
        std::mt19937 gen(42);
        std::uniform_real_distribution<float> dist(0, 1);

        for (int i = 0; i < count; i++) {
            float theta = dist(gen) * 2.0f * 3.14159f;
            float phi = acosf(dist(gen) * 2.0f - 1.0f);
            Vec3 p(cosf(theta) * sinf(phi), cosf(phi), sinf(theta) * sinf(phi));
            
            // Random Euler Pole (Rotation axis)
            Vec3 axis(dist(gen) * 2 - 1, dist(gen) * 2 - 1, dist(gen) * 2 - 1);
            float omega = (dist(gen) * 0.1f + 0.05f); // Rotation speed
            float elev = (dist(gen) < 0.4f) ? 0.05f : 0.55f; // 40% Ocean, 60% Continent

            plates.push_back({p, axis.normalized(), omega, elev});
        }
    }

    void lloydRelaxation(int iterations) {
        for (int iter = 0; iter < iterations; iter++) {
            std::vector<Vec3> centroids(plates.size(), Vec3(0, 0, 0));
            std::vector<int> counts(plates.size(), 0);

            // 1. Assign each grid cell to closest plate
            for (int y = 0; y < height; y++) {
                float phi = (float)y / (height - 1) * 3.14159f;
                for (int x = 0; x < width; x++) {
                    float theta = (float)x / (width - 1) * 2.0f * 3.14159f;
                    Vec3 p(cosf(theta) * sinf(phi), cosf(phi), sinf(theta) * sinf(phi));

                    int bestIdx = 0;
                    float minD = 1e10f;
                    for (int i = 0; i < (int)plates.size(); i++) {
                        float d = (p - plates[i].pos).lengthSq();
                        if (d < minD) { minD = d; bestIdx = i; }
                    }
                    gridPlate[y * width + x] = bestIdx;
                    centroids[bestIdx] = centroids[bestIdx] + p;
                    counts[bestIdx]++;
                }
            }

            // 2. Move seeds to centroids
            for (int i = 0; i < (int)plates.size(); i++) {
                if (counts[i] > 0) {
                    plates[i].pos = (centroids[i] * (1.0f / counts[i])).normalized();
                }
            }
        }
    }

    Vec3 getSphericalPos(int x, int y) {
        float phi = (float)y / (height - 1) * 3.14159f;
        float theta = (float)x / (width - 1) * 2.0f * 3.14159f;
        return Vec3(cosf(theta) * sinf(phi), cosf(phi), sinf(theta) * sinf(phi));
    }

    // Stage 3 & 4: Boundary Logic & Stage 5: Isostasy
    void updateHeightFromForces(float dt) {
        std::vector<float> deltaHeight(width * height, 0.0f);
        
        for (int y = 1; y < height - 1; y++) {
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;
                int p1 = gridPlate[idx];
                Vec3 pPos = getSphericalPos(x, y);
                Vec3 v1 = plates[p1].eulerPole.cross(pPos) * plates[p1].omega;

                // Check East neighbor (x+1)
                int nIdx = y * width + (x + 1) % width;
                int p2 = gridPlate[nIdx];
                if (p1 != p2) {
                    Vec3 nPos = getSphericalPos((x + 1) % width, y);
                    Vec3 v2 = plates[p2].eulerPole.cross(nPos) * plates[p2].omega;
                    Vec3 norm = (pPos - nPos).normalized();
                    float force = (v1 - v2).dot(norm); // < 0 is convergent
                    
                    if (force < -0.01f) deltaHeight[idx] -= force * 2.0f * dt; // Mountain
                    else if (force > 0.01f) deltaHeight[idx] -= force * 0.5f * dt; // Ridge
                }
            }
        }

        // Apply changes and Stage 5: Isostasy (Slowly return to base level)
        for (int i = 0; i < width * height; i++) {
            gridHeight[i] += deltaHeight[i];
            float target = plates[gridPlate[i]].baseElev;
            gridHeight[i] = gridHeight[i] * (1.0f - 0.05f * dt) + target * (0.05f * dt);
        }
    }

    GLuint textureID = 0;
    void bake() {
        if (textureID == 0) glGenTextures(1, &textureID);
        glBindTexture(GL_TEXTURE_2D, textureID);
        // Use R32F for high-precision height data
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, width, height, 0, GL_RED, GL_FLOAT, gridHeight.data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }

    void simulate(int generations) {
        initializePlates(24);
        lloydRelaxation(10);
        
        // Initial height
        for (int i = 0; i < width * height; i++) {
            gridHeight[i] = plates[gridPlate[i]].baseElev;
        }

        // Run generations
        float dt = 1.0f;
        for (int gen = 0; gen < generations; gen++) {
            updateHeightFromForces(dt);
        }

        bake();
    }
};

} // namespace Tectonic
