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

    // Main Entry Point
    void simulate(const std::vector<float>& heightMap, const std::vector<float>& precipitation, const std::vector<float>& temperature) {
        data.flowDir.assign(width * height, -1);
        fillDepressions(heightMap);
        calculateAccumulation(precipitation, temperature, heightMap);
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

        int dx[] = {1, 1, 0, -1, -1, -1, 0, 1}; // E, SE, S, SW, W, NW, N, NE
        int dy[] = {0, 1, 1, 1, 0, -1, -1, -1};
        int reverseDir[] = {4, 5, 6, 7, 0, 1, 2, 3}; 

        // Seed with oceanic pixels
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

        while (!pq.empty()) {
            Node curr = pq.top(); pq.pop();

            for (int i = 0; i < 8; i++) {
                int nx = (curr.x + dx[i] + width) % width;
                int ny = std::clamp(curr.y + dy[i], 0, height - 1);
                int nIdx = ny * width + nx;

                if (!visited[nIdx]) {
                    visited[nIdx] = true;
                    if (data.filledHeight[nIdx] < curr.h) {
                        data.filledHeight[nIdx] = curr.h;
                    }
                    // Flow direction points back to the explorer that provided path to ocean
                    data.flowDir[nIdx] = reverseDir[i];
                    pq.push({nx, ny, data.filledHeight[nIdx]});
                }
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

        data.accumulation.assign(width * height, 0.0f);
        for (int i = 0; i < width * height; i++) {
            float evap = std::max(0.0f, temp[i] + 12.0f) * 0.006f; 
            float netWater = std::max(0.001f, precip[i] - evap);
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
        for (int i = 0; i < width * height; i++) sortedCells.push_back({i, data.filledHeight[i]});
        std::sort(sortedCells.begin(), sortedCells.end(), [](const Cell& a, const Cell& b) {
            return a.h > b.h;
        });

        for (auto& cell : sortedCells) {
            int idx = cell.idx;
            if (contributors[idx].empty()) {
                if (data.accumulation[idx] > 0.05f) data.strahler[idx] = 1;
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
};

} // namespace Hydro
