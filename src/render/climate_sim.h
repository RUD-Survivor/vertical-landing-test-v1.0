#pragma once
#include <glad/glad.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include "../math/math3d.h"

namespace Climate {

struct ClimateData {
    std::vector<float> temperature;    
    std::vector<float> precipitation; 
    std::vector<float> pressure;      
    std::vector<Vec2> wind;           
    std::vector<float> moisture;      
    GLuint textureID = 0;
};

class ClimateSimulator {
public:
    int width, height;
    ClimateData data;

    ClimateSimulator(int w, int h) : width(w), height(h) {
        data.temperature.assign(w * h, 0.0f);
        data.precipitation.assign(w * h, 0.0f);
        data.pressure.assign(w * h, 0.0f);
        data.wind.assign(w * h, Vec2(0, 0));
        data.moisture.assign(w * h, 0.0f);
    }

    void simulate(const std::vector<float>& heightMap, float axialTilt = 23.5f) {
        calculateTemperature(heightMap, axialTilt);
        calculatePressure(axialTilt);
        calculateWind();
        calculateOceanCurrents(heightMap);
        applyOceanHeatTransport(heightMap); // Refines temperature
        calculateMoistureAndPrecipitation(heightMap);
        bake();
    }

    void bake() {
        if (data.textureID == 0) glGenTextures(1, &data.textureID);
        glBindTexture(GL_TEXTURE_2D, data.textureID);
        
        std::vector<float> textureData(width * height * 4, 0.0f);
        for (int i = 0; i < width * height; i++) {
            textureData[i * 4 + 0] = data.temperature[i];
            textureData[i * 4 + 1] = data.precipitation[i];
            textureData[i * 4 + 2] = data.pressure[i];
            textureData[i * 4 + 3] = data.moisture[i];
        }

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, textureData.data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }

private:
    void calculateTemperature(const std::vector<float>& heightMap, float currentTilt) {
        float tiltRad = currentTilt * 0.0174533f;
        for (int y = 0; y < height; y++) {
            float latRad = ((float)y / (height - 1)) * 3.14159f - 1.5708f;
            float cosAng = cosf(latRad - tiltRad); // Max insolation at the thermal equator

            for (int x = 0; x < width; x++) {
                int idx = y * width + x;
                float baseTemp = -30.0f + 65.0f * cosAng;
                float isLand = heightMap[idx] > 0.45f ? 1.0f : 0.0f;
                
                // Altitude Lapse Rate
                float alt = std::max(0.0f, heightMap[idx] - 0.45f);
                float lapseRate = alt * 65.0f; 
                
                // Continentality: Land heats up and cools down more than ocean
                baseTemp += isLand * 5.0f * (baseTemp > 10.0f ? 1.0f : -1.0f);
                
                data.temperature[idx] = baseTemp - lapseRate;
            }
        }
    }

    void calculatePressure(float currentTilt) {
        for (int y = 0; y < height; y++) {
            float latDeg = (float)y / (height - 1) * 180.0f - 90.0f;
            float thermalEquator = currentTilt * 0.8f; // ITCZ follows the sun with some lag/dampening
            float relLat = latDeg - thermalEquator;
            float absRelLat = fabsf(relLat);
            
            float zonalP = 0.0f;
            if (absRelLat < 30.0f) zonalP = lerp(1000.0f, 1025.0f, absRelLat / 30.0f);
            else if (absRelLat < 60.0f) zonalP = lerp(1025.0f, 990.0f, (absRelLat - 30.0f) / 30.0f);
            else zonalP = lerp(990.0f, 1035.0f, (absRelLat - 60.0f) / 30.0f);

            for (int x = 0; x < width; x++) {
                data.pressure[y * width + x] = zonalP;
            }
        }
    }

    void calculateWind() {
        for (int y = 1; y < height - 1; y++) {
            float latDeg = (float)y / (height - 1) * 180.0f - 90.0f;
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;
                float dpdy = data.pressure[(y+1)*width + x] - data.pressure[(y-1)*width + x];
                float windX = 0, windY = -dpdy * 0.6f;
                
                if (latDeg > -30 && latDeg < 30) windX = -1.2f; // Trades
                else if (latDeg > 30 && latDeg < 60) windX = 1.4f; // Westerlies
                else if (latDeg < -30 && latDeg > -60) windX = 1.4f;
                else windX = -1.0f; // Polar Easterlies
                
                data.wind[idx] = Vec2(windX, windY);
            }
        }
    }

    std::vector<Vec2> oceanCurrents;
    void calculateOceanCurrents(const std::vector<float>& heightMap) {
        oceanCurrents.assign(width * height, Vec2(0, 0));
        for (int y = 1; y < height - 1; y++) {
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;
                if (heightMap[idx] > 0.45f) continue; // No currents on land

                Vec2 w = data.wind[idx];
                // Simplify: surface currents follow wind but blocked by land
                oceanCurrents[idx] = w * 0.35f;
            }
        }
    }

    void applyOceanHeatTransport(const std::vector<float>& heightMap) {
        // Run 32 iterations of ocean heat advection
        for (int pass = 0; pass < 32; pass++) {
            std::vector<float> nextTemp = data.temperature;
            for (int y = 1; y < height - 1; y++) {
                for (int x = 0; x < width; x++) {
                    int idx = y * width + x;
                    if (heightMap[idx] > 0.45f) continue; // Only advect ocean heat

                    Vec2 v = oceanCurrents[idx];
                    int sx = (x - (int)roundf(v.x) + width) % width;
                    int sy = std::clamp(y - (int)roundf(v.y), 0, height - 1);
                    
                    // Only transport heat if the source is also ocean (coastal boundary handling)
                    if (heightMap[sy * width + sx] <= 0.45f) {
                        float diff = data.temperature[sy * width + sx] - data.temperature[idx];
                        nextTemp[idx] += diff * 0.15f; 
                    }
                }
            }
            data.temperature = nextTemp;
        }
        // Coastal "Thermal Smoothing": bleed ocean temperature onto coasts
        std::vector<float> coastalTemp = data.temperature;
        for (int y = 1; y < height - 1; y++) {
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;
                if (heightMap[idx] > 0.45f) { // Coastline check
                    float oceanSum = 0; int oceanCount = 0;
                    for (int dy=-1; dy<=1; dy++) {
                        for (int dx=-1; dx<=1; dx++) {
                            int nidx = (y+dy)*width + (x+dx+width)%width;
                            if (heightMap[nidx] <= 0.45f) {
                                oceanSum += data.temperature[nidx];
                                oceanCount++;
                            }
                        }
                    }
                    if (oceanCount > 0) coastalTemp[idx] = lerp(data.temperature[idx], oceanSum/oceanCount, 0.4f);
                }
            }
        }
        data.temperature = coastalTemp;
    }

    void calculateMoistureAndPrecipitation(const std::vector<float>& heightMap) {
        data.moisture.assign(width * height, 0.0f);
        data.precipitation.assign(width * height, 0.0f);

        for (int pass = 0; pass < 128; pass++) {
            std::vector<float> nextMoisture = data.moisture;
            for (int y = 1; y < height - 1; y++) {
                for (int x = 0; x < width; x++) {
                    int idx = y * width + x;
                    if (heightMap[idx] <= 0.45f) {
                        float evap = std::max(0.0f, data.temperature[idx] + 20.0f) * 0.08f;
                        nextMoisture[idx] += evap;
                    }

                    Vec2 w = data.wind[idx];
                    int nx = (x + (int)roundf(w.x) + width) % width;
                    int ny = std::clamp(y + (int)roundf(w.y), 0, height - 1);
                    float flow = data.moisture[idx] * 0.5f;
                    nextMoisture[idx] -= flow;
                    nextMoisture[ny * width + nx] += flow;

                    float precip = 0.0f;
                    float itczStrength = smoothstep(1018.0f, 990.0f, data.pressure[idx]);
                    if (itczStrength > 0.01f) {
                        precip += nextMoisture[idx] * 0.45f * itczStrength;
                    }
                    
                    int pnx = (x + (int)roundf(w.x) + width) % width;
                    int pny = std::clamp(y + (int)roundf(w.y), 0, height - 1);
                    float dh = heightMap[pny * width + pnx] - heightMap[idx];
                    if (dh > 0.0002f) {
                         float liftFactor = smoothstep(0.0002f, 0.008f, dh);
                         precip += liftFactor * nextMoisture[idx] * 35.0f;
                    }

                    precip = std::clamp(precip, 0.0f, nextMoisture[idx] * 0.85f);
                    nextMoisture[idx] -= precip;
                    data.precipitation[idx] += precip;
                }
            }
            for (float& m : nextMoisture) if (m < 0) m = 0;
            data.moisture = nextMoisture;
        }

        smoothField(data.precipitation, 3); 
        smoothField(data.moisture, 2);
        for (float& p : data.precipitation) p *= 18.0f;
    }

    void smoothField(std::vector<float>& field, int passes) {
        for (int p = 0; p < passes; p++) {
            std::vector<float> temp = field;
            for (int y = 1; y < height - 1; y++) {
                for (int x = 0; x < width; x++) {
                    float sum = 0.0f;
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            sum += temp[(y + dy) * width + (x + dx + width) % width];
                        }
                    }
                    field[y * width + x] = sum / 9.0f;
                }
            }
        }
    }

    float lerp(float a, float b, float t) { return a + (b - a) * t; }
    float smoothstep(float edge0, float edge1, float x) {
        float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }
};

} // namespace Climate
