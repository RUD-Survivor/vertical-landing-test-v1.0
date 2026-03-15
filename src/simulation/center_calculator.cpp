// ==========================================================
// center_calculator.cpp — Center Point Calculation Implementation
// ==========================================================

#include "center_calculator.h"

// Include Renderer to satisfy rocket_builder.h inline functions
#include "render/renderer_2d.h"
// We need access to PART_CATALOG which is defined in rocket_builder.h
#include "simulation/rocket_builder.h"

#include <cmath>

namespace CenterCalculator {

// Helper: Check if a part is an engine
bool isEngine(const PartDef& part) {
    return part.category == CAT_ENGINE || part.category == CAT_BOOSTER;
}

// Helper: Check if a part is an aerodynamic surface
bool isAerodynamicSurface(const PartDef& part) {
    // Nose cones are aerodynamic surfaces
    if (part.category == CAT_NOSE_CONE) {
        return true;
    }
    // Fin sets (id=16 in structural category) are aerodynamic surfaces
    if (part.category == CAT_STRUCTURAL && part.id == 16) {
        return true;
    }
    return false;
}

// Helper: Get effective aerodynamic area for a part
float getAerodynamicArea(const PartDef& part) {
    if (part.category == CAT_NOSE_CONE) {
        // Cross-sectional area based on diameter
        float radius = part.diameter / 2.0f;
        return 3.14159f * radius * radius;
    }
    if (part.category == CAT_STRUCTURAL && part.id == 16) {
        // Fin set: use fixed area coefficient
        // Assuming 4 fins with reasonable surface area
        return 2.0f; // m^2 effective area
    }
    return 0.0f;
}

// Calculate hash of assembly configuration
size_t hashAssembly(const RocketAssembly& assembly) {
    size_t hash = 0;
    
    // Hash based on number of parts
    hash ^= assembly.parts.size() * 0x9e3779b9;
    
    // Hash each part's properties
    for (size_t i = 0; i < assembly.parts.size(); i++) {
        const PlacedPart& pp = assembly.parts[i];
        
        // Combine part ID, position, and symmetry
        size_t part_hash = pp.def_id;
        part_hash ^= (size_t)(pp.pos.x * 1000.0f) << 4;
        part_hash ^= (size_t)(pp.pos.y * 1000.0f) << 8;
        part_hash ^= (size_t)(pp.pos.z * 1000.0f) << 12;
        part_hash ^= (size_t)pp.symmetry << 20;
        
        // Combine with running hash
        hash ^= part_hash * 0x9e3779b9 + (hash << 6) + (hash >> 2);
    }
    
    return hash;
}

// Calculate center of mass
Vec3 calculateCenterOfMass(const RocketAssembly& assembly) {
    if (assembly.parts.empty()) {
        return Vec3(0, 0, 0);
    }
    
    float total_mass = 0.0f;
    Vec3 weighted_pos = Vec3(0, 0, 0);
    
    for (size_t i = 0; i < assembly.parts.size(); i++) {
        const PlacedPart& pp = assembly.parts[i];
        const PartDef& def = PART_CATALOG[pp.def_id];
        
        float part_mass = (def.dry_mass + def.fuel_capacity) * pp.symmetry;
        if (part_mass <= 0.0f) continue;
        
        // For symmetry, we assume parts are radially balanced around (0, y, 0)
        Vec3 local_com = pp.pos + Vec3(0, def.height / 2.0f, 0);
        if (pp.symmetry > 1) {
            local_com.x = 0;
            local_com.z = 0;
        }

        total_mass += part_mass;
        weighted_pos = weighted_pos + local_com * part_mass;
    }
    
    if (total_mass <= 0.0f) return Vec3(0, 0, 0);
    return weighted_pos * (1.0f / total_mass);
}

// Calculate center of lift
Vec3 calculateCenterOfLift(const RocketAssembly& assembly) {
    if (assembly.parts.empty()) return Vec3(0, 0, 0);
    
    float total_area = 0.0f;
    Vec3 weighted_pos = Vec3(0, 0, 0);
    
    for (size_t i = 0; i < assembly.parts.size(); i++) {
        const PlacedPart& pp = assembly.parts[i];
        const PartDef& def = PART_CATALOG[pp.def_id];
        if (!isAerodynamicSurface(def)) continue;
        
        float area = getAerodynamicArea(def) * pp.symmetry;
        if (area <= 0.0f) continue;
        
        Vec3 local_col = pp.pos + Vec3(0, def.height / 2.0f, 0);
        if (pp.symmetry > 1) { local_col.x = 0; local_col.z = 0; }

        total_area += area;
        weighted_pos = weighted_pos + local_col * area;
    }
    
    if (total_area <= 0.0f) return Vec3(0, 0, 0);
    return weighted_pos * (1.0f / total_area);
}

// Calculate center of thrust
Vec3 calculateCenterOfThrust(const RocketAssembly& assembly) {
    if (assembly.parts.empty()) return Vec3(0, 0, 0);
    
    float total_thrust = 0.0f;
    Vec3 weighted_pos = Vec3(0, 0, 0);
    
    for (size_t i = 0; i < assembly.parts.size(); i++) {
        const PlacedPart& pp = assembly.parts[i];
        const PartDef& def = PART_CATALOG[pp.def_id];
        if (!isEngine(def)) continue;
        
        float thrust = def.thrust * pp.symmetry;
        if (thrust <= 0.0f) continue;
        
        Vec3 local_cot = pp.pos + Vec3(0, def.height / 2.0f, 0);
        if (pp.symmetry > 1) { local_cot.x = 0; local_cot.z = 0; }

        total_thrust += thrust;
        weighted_pos = weighted_pos + local_cot * thrust;
    }
    
    if (total_thrust <= 0.0f) return Vec3(0, 0, 0);
    return weighted_pos * (1.0f / total_thrust);
}

} // namespace CenterCalculator
