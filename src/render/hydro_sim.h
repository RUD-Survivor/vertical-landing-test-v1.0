#pragma once
#include <vector>
#include <queue>
#include <algorithm>
#include "../math/math3d.h"

namespace Hydro {

struct HydroData {
    std::vector<float> accumulation; // Flow accumulation (River volume)
    std::vector<int> flowDir;       // D8 Flow direction index (0-7)
    std::vector<int> strahler;      // Strahler Stream Order
    std::vector<float> waterTable;   // Surface water / Lake depth
    std::vector<float> filledHeight; // Terrain height after depression filling
};

class HydroSimulator {
public:
    int width, height;
    HydroData data;

    HydroSimulator(int w, int h) : width(w), height(h) {
        data.accumulation.assign(w * h, 0.0f);
        data.flowDir.assign(w * h, -1);
        data.strahler.assign(w * h, 0);
        data.waterTable.assign(w * h, 0.0f);
        data.filledHeight.assign(w * h, 0.0f);
    }

    // Main Entry Point: Disabling coarse global erosion to prevent 78km-wide riverbed artifacts.
    // Carving will now be handled at the high-precision LOD level in the vertex shader.
    void simulate(const std::vector<float>& heightMap, const std::vector<float>& precipitation, const std::vector<float>& temperature) {
        fillDepressions(heightMap);
        calculateFlowDirections();
        calculateAccumulation(precipitation, temperature, heightMap);
        
        // --- Lake Breaching & Evaporation Model ---
        // Priority-Flood routing assumes basins fill to the brim. Here we physically
        // lower the water surface for rendering based on evaporation and flow carving.
        for (int i = 0; i < width * height; i++) {
            float lakeDepth = data.filledHeight[i] - heightMap[i];
            if (lakeDepth > 0.002f) { // Significant basin
                if (data.accumulation[i] < 20.0f) {
                    // Evaporation dominant: Drop water level for low flow areas in huge basins
                    data.filledHeight[i] = heightMap[i] + std::min(lakeDepth, 0.001f);
                } else {
                    // River Breaching: High flow carves a deep gorge through the basin rim
                    data.filledHeight[i] = std::max(heightMap[i] + 0.002f, data.filledHeight[i] - 0.005f);
                }
            }
        }
        
        calculateStrahlerOrder();
    }

private:
    void fillDepressions(const std::vector<float>& heightMap) {
        data.filledHeight = heightMap;
        
        struct Node {
            int x, y;
            float h;
            bool operator>(const Node& other) const { return h > other.h; }
        };
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
        std::vector<bool> visited(width * height, false);

        float seaLevel = 0.45f;
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;
                if (heightMap[idx] <= seaLevel) {
                    pq.push({x, y, heightMap[idx]});
                    visited[idx] = true;
                }
            }
        }

        int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
        int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};

        while (!pq.empty()) {
            Node curr = pq.top(); pq.pop();

            for (int i = 0; i < 8; i++) {
                int nx = (curr.x + dx[i] + width) % width;
                int ny = std::clamp(curr.y + dy[i], 0, height - 1);
                int nIdx = ny * width + nx;

                if (!visited[nIdx]) {
                    visited[nIdx] = true;
                    // Add a tiny epsilon (1e-6) to force a drainage slope
                    // This prevents massive flat "blobs" by giving water a direction
                    if (data.filledHeight[nIdx] < curr.h + 1e-6f) {
                        data.filledHeight[nIdx] = curr.h + 1e-6f;
                    }
                    pq.push({nx, ny, data.filledHeight[nIdx]});
                }
            }
        }
    }

    void calculateFlowDirections() {
        int dx[] = {1, 1, 0, -1, -1, -1, 0, 1}; 
        int dy[] = {0, 1, 1, 1, 0, -1, -1, -1};

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;
                float maxSlope = -1.0f; 
                int bestDir = -1;

                for (int i = 0; i < 8; i++) {
                    int nx = (x + dx[i] + width) % width;
                    int ny = std::clamp(y + dy[i], 0, height-1);
                    int nIdx = ny * width + nx;

                    float drop = data.filledHeight[idx] - data.filledHeight[nIdx];
                    if (drop > maxSlope) {
                        maxSlope = drop;
                        bestDir = i;
                    }
                }
                data.flowDir[idx] = bestDir;
            }
        }
    }

    void calculateAccumulation(const std::vector<float>& precip, const std::vector<float>& temp, const std::vector<float>& heightMap) {
        struct Cell { int idx; float h; };
        std::vector<Cell> sortedCells;
        sortedCells.reserve(width * height);
        for (int i = 0; i < width * height; i++) sortedCells.push_back({i, data.filledHeight[i]});
        
        std::sort(sortedCells.begin(), sortedCells.end(), [](const Cell& a, const Cell& b) {
            return a.h > b.h;
        });

        for (int i = 0; i < width * height; i++) {
            float evap = std::max(0.0f, temp[i] + 10.0f) * 0.009f; 
            float netWater = std::max(0.0f, precip[i] - evap); 
            if (heightMap[i] <= 0.45f) netWater = 0.0f;
            data.accumulation[i] = netWater;
        }

        int dx[] = {1, 1, 0, -1, -1, -1, 0, 1}; 
        int dy[] = {0, 1, 1, 1, 0, -1, -1, -1};

        for (auto& cell : sortedCells) {
            int idx = cell.idx;
            int dir = data.flowDir[idx];
            if (dir != -1) {
                int nx = (idx % width + dx[dir] + width) % width;
                int ny = std::clamp(idx / width + dy[dir], 0, height - 1);
                int nIdx = ny * width + nx;
                data.accumulation[nIdx] += data.accumulation[idx];
            }
        }

        // --- ENHANCED: Hydraulic Erosion & Sedimentation ---
        // We modify the WATER SURFACE level (filledHeight) to reflect carved valleys
        for (int i = 0; i < width * height; i++) {
            if (heightMap[i] > 0.455f && data.accumulation[i] > 1.0f) { 
                int dir = data.flowDir[i];
                if (dir != -1) {
                    float slope = 0.01f; // assume some slope for carving
                    // Carve valley deeper where flow is strong (Strahler-like strength)
                    float carve = std::min(0.03f, powf(data.accumulation[i], 0.5f) * 0.005f);
                    data.filledHeight[i] -= carve;
                }
            }
        }
    }

    void calculateStrahlerOrder() {
        data.strahler.assign(width * height, 0);
        
        std::vector<std::vector<int>> contributors(width * height);
        int dx[] = {1, 1, 0, -1, -1, -1, 0, 1}; 
        int dy[] = {0, 1, 1, 1, 0, -1, -1, -1};

        for (int i = 0; i < width * height; i++) {
            int dir = data.flowDir[i];
            if (dir != -1) {
                int nx = (i % width + dx[dir] + width) % width;
                int ny = std::clamp(i / width + dy[dir], 0, height - 1);
                contributors[ny * width + nx].push_back(i);
            }
        }

        struct Cell { int idx; float h; };
        std::vector<Cell> sortedCells;
        sortedCells.reserve(width * height);
        for (int i = 0; i < width * height; i++) sortedCells.push_back({i, data.filledHeight[i]});
        
        std::sort(sortedCells.begin(), sortedCells.end(), [](const Cell& a, const Cell& b) {
            return a.h > b.h;
        });

        for (auto& cell : sortedCells) {
            int idx = cell.idx;
            if (contributors[idx].empty()) {
                // Lower threshold for river tips (0.01 instead of 0.05)
                if (data.accumulation[idx] > 0.01f) data.strahler[idx] = 1;
                else data.strahler[idx] = 0;
            } else {
                int maxOrder = 0;
                int countMax = 0;
                for (int cIdx : contributors[idx]) {
                    int cOrder = data.strahler[cIdx];
                    if (cOrder > maxOrder) {
                        maxOrder = cOrder;
                        countMax = 1;
                    } else if (cOrder == maxOrder && maxOrder > 0) {
                        countMax++;
                    }
                }
                if (countMax > 1) data.strahler[idx] = maxOrder + 1;
                else data.strahler[idx] = maxOrder;
            }
        }
    }

public:
    void bake() {
        if (textureID == 0) glGenTextures(1, &textureID);
        glBindTexture(GL_TEXTURE_2D, textureID);
        
        // R: FilledHeight (for Lake flattening), G: Accumulation, B: Strahler, A: Placeholder
        std::vector<float> textureData(width * height * 4, 0.0f);
        for (int i = 0; i < width * height; i++) {
            textureData[i * 4 + 0] = data.filledHeight[i];
            textureData[i * 4 + 1] = data.accumulation[i];
            textureData[i * 4 + 2] = (float)data.strahler[i];
            textureData[i * 4 + 3] = 0.0f;
        }

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, textureData.data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }

    GLuint textureID = 0;
};

} // namespace Hydro
