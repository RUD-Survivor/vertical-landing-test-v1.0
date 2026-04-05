#pragma once
// ==========================================================
// model_loader.h — Simple OBJ Model Loader
// Parses .obj files to be used with the Renderer3D Mesh system.
// ==========================================================

#include "render/renderer3d.h"
#include "math/math3d.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>

namespace ModelLoader {

    struct Material {
        std::string name;
        float Kd[3] = {1.0f, 1.0f, 1.0f};
    };

    inline std::map<std::string, Material> loadMTL(const std::string& filepath) {
        std::map<std::string, Material> materials;
        std::ifstream file(filepath);
        if (!file.is_open()) {
            std::cerr << ">> [MODEL LOADER] Failed to open MTL file: " << filepath << std::endl;
            return materials;
        }

        std::string line;
        Material currentMaterial;
        bool hasMaterial = false;

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;

            std::istringstream iss(line);
            std::string prefix;
            iss >> prefix;

            if (prefix == "newmtl") {
                if (hasMaterial) {
                    materials[currentMaterial.name] = currentMaterial;
                }
                iss >> currentMaterial.name;
                currentMaterial.Kd[0] = 1.0f;
                currentMaterial.Kd[1] = 1.0f;
                currentMaterial.Kd[2] = 1.0f;
                hasMaterial = true;
            } else if (prefix == "Kd") {
                iss >> currentMaterial.Kd[0] >> currentMaterial.Kd[1] >> currentMaterial.Kd[2];
            }
        }
        if (hasMaterial) {
            materials[currentMaterial.name] = currentMaterial;
        }
        return materials;
    }

    // Simple hashing for unique vertices in the OBJ file to build an indexed mesh
    struct VertexKey {
        int v, vt, vn, mtlId;
        bool operator<(const VertexKey& other) const {
            if (v != other.v) return v < other.v;
            if (vt != other.vt) return vt < other.vt;
            if (vn != other.vn) return vn < other.vn;
            return mtlId < other.mtlId;
        }
    };

    // Helper to parse face indices "v/vt/vn"
    inline bool parseFaceVertex(const std::string& token, VertexKey& key, int mtlId) {
        key.v = 0; key.vt = 0; key.vn = 0; key.mtlId = mtlId;
        std::istringstream stream(token);
        std::string part;
        
        // v
        if (std::getline(stream, part, '/')) {
            if (!part.empty()) key.v = std::stoi(part);
            else return false;
        } else return false;
        
        // vt
        if (std::getline(stream, part, '/')) {
            if (!part.empty()) key.vt = std::stoi(part);
        }
        
        // vn
        if (std::getline(stream, part, '/')) {
            if (!part.empty()) key.vn = std::stoi(part);
        }
        return true;
    }

    // Loads an OBJ file into a Mesh object
    inline Mesh loadOBJ(const std::string& filepath) {
        Mesh finalMesh;
        std::vector<Vertex3D> verticesOut;
        std::vector<unsigned int> indicesOut;

        std::ifstream file(filepath);
        if (!file.is_open()) {
            std::cerr << ">> [MODEL LOADER] Failed to open OBJ file: " << filepath << std::endl;
            return finalMesh; // Return empty mesh
        }

        std::vector<Vec3> temp_vertices;
        std::vector<Vec2> temp_uvs;
        std::vector<Vec3> temp_normals;
        
        std::map<VertexKey, unsigned int> uniqueVertices;

        std::map<std::string, Material> materials;
        Material currentMaterial;
        int currentMtlId = 0;
        int nextMtlId = 1;
        std::map<std::string, int> mtlIds;

        // Base directory for MTL resolution
        std::string directory = "";
        size_t lastSlash = filepath.find_last_of("/\\");
        if (lastSlash != std::string::npos) {
            directory = filepath.substr(0, lastSlash + 1);
        }

        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;

            std::istringstream iss(line);
            std::string prefix;
            iss >> prefix;

            if (prefix == "mtllib") {
                std::string mtlFilename;
                // If mtllib has spaces in it, this might fail, but usually it doesn't.
                // Let's get the rest of the line to support spaces just in case:
                // Actually standard extraction is fine for basic MTLLIB
                std::getline(iss >> std::ws, mtlFilename);
                std::string mtlPath = directory + mtlFilename;
                materials = loadMTL(mtlPath);
            }
            else if (prefix == "usemtl") {
                std::string mtlName;
                std::getline(iss >> std::ws, mtlName);
                if (materials.find(mtlName) != materials.end()) {
                    currentMaterial = materials[mtlName];
                    if (mtlIds.find(mtlName) == mtlIds.end()) {
                        mtlIds[mtlName] = nextMtlId++;
                    }
                    currentMtlId = mtlIds[mtlName];
                }
            }
            else if (prefix == "v") {
                Vec3 vertex;
                iss >> vertex.x >> vertex.y >> vertex.z;
                temp_vertices.push_back(vertex);
            } 
            else if (prefix == "vt") {
                Vec2 uv;
                iss >> uv.x >> uv.y;
                temp_uvs.push_back(uv);
            } 
            else if (prefix == "vn") {
                Vec3 normal;
                iss >> normal.x >> normal.y >> normal.z;
                temp_normals.push_back(normal);
            } 
            else if (prefix == "f") {
                std::string vertexStr;
                std::vector<VertexKey> faceKeys;
                
                while (iss >> vertexStr) {
                    VertexKey key;
                    if (parseFaceVertex(vertexStr, key, currentMtlId)) {
                        faceKeys.push_back(key);
                    }
                }

                if (faceKeys.size() >= 3) {
                    // Simple triangulation (assumes convex polygons, breaking into fans)
                    for (size_t i = 1; i < faceKeys.size() - 1; ++i) {
                        VertexKey keys[3] = {faceKeys[0], faceKeys[i], faceKeys[i+1]};
                        
                        for (int j = 0; j < 3; ++j) {
                            VertexKey currentKey = keys[j];
                            
                            // Check if this combined vertex has been seen before
                            auto it = uniqueVertices.find(currentKey);
                            if (it == uniqueVertices.end()) {
                                // Create new vertex
                                Vertex3D vertex;
                                
                                // OBJ indices are 1-based, convert to 0-based
                                if (currentKey.v > 0 && currentKey.v <= (int)temp_vertices.size()) {
                                    Vec3 p = temp_vertices[currentKey.v - 1];
                                    vertex.px = p.x;
                                    vertex.py = p.y;
                                    vertex.pz = p.z;
                                }
                                
                                if (currentKey.vt > 0 && currentKey.vt <= (int)temp_uvs.size()) {
                                    // Assimp or Blender might flip V
                                    Vec2 uv = temp_uvs[currentKey.vt - 1];
                                    vertex.u = uv.x;
                                    vertex.v = uv.y;
                                } else {
                                    vertex.u = 0; vertex.v = 0;
                                }
                                
                                if (currentKey.vn > 0 && currentKey.vn <= (int)temp_normals.size()) {
                                    Vec3 n = temp_normals[currentKey.vn - 1];
                                    vertex.nx = n.x;
                                    vertex.ny = n.y;
                                    vertex.nz = n.z;
                                } else {
                                    // Fallback placeholder normal
                                    vertex.nx = 0; vertex.ny = 1; vertex.nz = 0;
                                }
                                
                                // Apply color from current material
                                vertex.r = currentMaterial.Kd[0]; 
                                vertex.g = currentMaterial.Kd[1]; 
                                vertex.b = currentMaterial.Kd[2]; 
                                vertex.a = 1.0f;

                                unsigned int newIndex = (unsigned int)verticesOut.size();
                                uniqueVertices[currentKey] = newIndex;
                                verticesOut.push_back(vertex);
                                indicesOut.push_back(newIndex);
                            } else {
                                // Reuse existing vertex index
                                indicesOut.push_back(it->second);
                            }
                        }
                    }
                }
            }
        }

        // Calculate bounding box and geometric properties
        if (!verticesOut.empty()) {
            finalMesh.minX = finalMesh.maxX = verticesOut[0].px;
            finalMesh.minY = finalMesh.maxY = verticesOut[0].py;
            finalMesh.minZ = finalMesh.maxZ = verticesOut[0].pz;
            
            for (const auto& v : verticesOut) {
                if (v.px < finalMesh.minX) finalMesh.minX = v.px;
                if (v.px > finalMesh.maxX) finalMesh.maxX = v.px;
                if (v.py < finalMesh.minY) finalMesh.minY = v.py;
                if (v.py > finalMesh.maxY) finalMesh.maxY = v.py;
                if (v.pz < finalMesh.minZ) finalMesh.minZ = v.pz;
                if (v.pz > finalMesh.maxZ) finalMesh.maxZ = v.pz;
            }
            finalMesh.width = finalMesh.maxX - finalMesh.minX;
            finalMesh.height = finalMesh.maxY - finalMesh.minY;
            finalMesh.depth = finalMesh.maxZ - finalMesh.minZ;
            finalMesh.centerX = (finalMesh.minX + finalMesh.maxX) * 0.5f;
            finalMesh.centerY = (finalMesh.minY + finalMesh.maxY) * 0.5f;
            finalMesh.centerZ = (finalMesh.minZ + finalMesh.maxZ) * 0.5f;

            // Ensure we don't have zero dimensions
            if (finalMesh.width < 0.001f) finalMesh.width = 1.0f;
            if (finalMesh.height < 0.001f) finalMesh.height = 1.0f;
            if (finalMesh.depth < 0.001f) finalMesh.depth = 1.0f;
        }

        finalMesh.upload(verticesOut, indicesOut);
        return finalMesh;
    }
}
