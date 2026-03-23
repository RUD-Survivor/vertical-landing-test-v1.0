#pragma once
// ==========================================================
// rocket_builder.h — KSP-like Rocket Builder System (V2)
// ==========================================================

#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include <string>
#include "math/math3d.h"

#ifndef PI
#define PI 3.1415926535f
#endif

// 1. Forward Declarations
struct RocketAssembly;
struct RocketConfig;
namespace StageManager { void BuildStages(const RocketAssembly& assembly, RocketConfig& config); }

// 2. Core Part Enums
enum PartCategory {
    CAT_NOSE_CONE,
    CAT_COMMAND_POD,
    CAT_FUEL_TANK,
    CAT_ENGINE,
    CAT_BOOSTER,
    CAT_STRUCTURAL,
    CAT_COUNT
};

static const char* CATEGORY_NAMES[] = {
    "NOSECONES", "COMMAND", "FUEL TANKS", "ENGINES", "BOOSTERS", "STRUCTURAL"
};

// 3. Structural Definitions (Independent)
struct SnapNode {
    Vec3 pos;       
    Vec3 dir;       
    float diameter; 
    int type;       
};

struct PartDef {
    int id;
    const char* name;
    PartCategory category;
    float dry_mass;
    float fuel_capacity;
    float isp;
    float thrust;
    float consumption;
    float height;
    float diameter;
    float drag_coeff;
    float r, g, b;
    const char* description;
    std::vector<SnapNode> snap_nodes;
    bool surf_attach;
    const char* model_path;   // Path to .obj file
    const char* texture_path; // Path to texture file
};

struct PlacedPart {
    int def_id;          
    int stage;
    Vec3 pos;            
    Quat rot;            
    int parent_idx;      
    int symmetry;        
    float stack_y;       

    PlacedPart() : def_id(0), stage(0), pos(Vec3(0,0,0)), rot(Quat()), parent_idx(-1), symmetry(1), stack_y(0) {}
};

// 4. Catalog Implementation (Header-only static internal linkage)
inline std::vector<SnapNode> stackNodes(float h, float d) {
    std::vector<SnapNode> v;
    v.push_back({Vec3(0, h, 0), Vec3(0, 1, 0), d, 0});
    v.push_back({Vec3(0, 0, 0), Vec3(0, -1, 0), d, 0});
    return v;
}

// Using static to avoid LNK2005 (one copy per including TU)
static const PartDef PART_CATALOG[] = {
    {0,  "Standard Fairing", CAT_NOSE_CONE, 500.0f, 0.0f, 0.0f, 0.0f, 0.0f, 8.0f, 3.7f, 0.2f, 0.85f, 0.85f, 0.90f, "Basic aerodynamic fairing", stackNodes(0, 3.7f), false, nullptr, nullptr},
    {1,  "Aero Nosecone",    CAT_NOSE_CONE, 350.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 3.7f, 0.15f, 0.70f, 0.70f, 0.75f, "Low-drag pointed nosecone", stackNodes(0, 3.7f), false, nullptr, nullptr},
    {2,  "Payload Bay",      CAT_NOSE_CONE, 1200.0f, 0.0f, 0.0f, 0.0f, 0.0f, 12.0f, 4.0f, 0.3f, 0.90f, 0.90f, 0.85f, "Large payload enclosure", stackNodes(0, 4.0f), false, nullptr, nullptr},
    {3,  "Mk1 Capsule",      CAT_COMMAND_POD, 1500.0f, 0.0f, 0.0f, 0.0f, 0.0f, 4.0f, 2.5f, 0.25f, 0.30f, 0.30f, 0.35f, "Single-crew capsule", stackNodes(4.0f, 2.5f), false, nullptr, nullptr},
    {4,  "Probe Core",       CAT_COMMAND_POD, 200.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.1f, 0.20f, 0.25f, 0.20f, "Guidance unit", stackNodes(1.0f, 1.0f), false, nullptr, nullptr},
    {5,  "Small Tank 50t",   CAT_FUEL_TANK, 3000.0f, 50000.0f, 0.0f, 0.0f, 0.0f, 25.0f, 3.7f, 0.3f, 0.92f, 0.92f, 0.92f, "Small fuel storage", stackNodes(25.0f, 3.7f), true, nullptr, nullptr},
    {6,  "Medium Tank 100t", CAT_FUEL_TANK, 5000.0f, 100000.0f, 0.0f, 0.0f, 0.0f, 35.0f, 3.7f, 0.3f, 0.90f, 0.90f, 0.92f, "Standard fuel tank", stackNodes(35.0f, 3.7f), true, nullptr, nullptr},
    {7,  "Large Tank 200t",  CAT_FUEL_TANK, 8000.0f, 200000.0f, 0.0f, 0.0f, 0.0f, 50.0f, 3.7f, 0.3f, 0.88f, 0.88f, 0.92f, "High-capacity tank", stackNodes(50.0f, 3.7f), true, nullptr, nullptr},
    {8,  "Jumbo Tank 500t",  CAT_FUEL_TANK, 15000.0f, 500000.0f, 0.0f, 0.0f, 0.0f, 80.0f, 5.0f, 0.3f, 0.85f, 0.85f, 0.90f, "Massive reservoir", stackNodes(80.0f, 5.0f), true, nullptr, nullptr},
    {9,  "Raptor",           CAT_ENGINE, 2000.0f, 0.0f, 1500.0f, 2200000.0f, 100.0f, 4.0f, 3.7f, 0.05f, 0.35f, 0.35f, 0.38f, "High-ISP engine", stackNodes(4.0f, 3.7f), true, nullptr, nullptr},
    {10, "Merlin 1D",        CAT_ENGINE, 1500.0f, 0.0f, 1200.0f, 845000.0f, 80.0f, 3.0f, 3.0f, 0.05f, 0.40f, 0.35f, 0.30f, "Workhorse engine", stackNodes(3.0f, 3.0f), true, nullptr, nullptr},
    {11, "Vacuum Engine",    CAT_ENGINE, 800.0f, 0.0f, 1800.0f, 300000.0f, 20.0f, 5.0f, 4.2f, 0.02f, 0.6f, 0.6f, 0.7f, "Deep space efficient", stackNodes(5.0f, 4.2f), true, nullptr, nullptr},
    {12, "Decoupler",        CAT_STRUCTURAL, 300.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 3.7f, 0.0f, 0.25f, 0.25f, 0.25f, "Stage separator", stackNodes(1.0f, 3.7f), false, nullptr, nullptr},
    {13, "Fin Set (x4)",     CAT_STRUCTURAL, 200.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 3.7f, -0.1f, 0.30f, 0.30f, 0.30f, "Stability fins", stackNodes(2.0f, 3.7f), true, nullptr, nullptr}, 
    {14, "Girder XL",        CAT_STRUCTURAL, 400.0f, 0.0f, 0.0f, 0.0f, 0.0f, 6.0f, 1.0f, 0.1f, 0.40f, 0.40f, 0.42f, "Structural beam", stackNodes(6.0f, 1.0f), true, nullptr, nullptr},
    {15, "Gold Leaf Tank",   CAT_FUEL_TANK, 1000.0f, 20000.0f, 0.0f, 0.0f, 0.0f, 12.0f, 2.0f, 0.2f, 1.0f, 0.84f, 0.0f, "Titanium-gold tank", stackNodes(12.0f, 2.0f), true, nullptr, nullptr},
    {16, "Landing Leg",      CAT_STRUCTURAL, 500.0f, 0.0f, 0.0f, 0.0f, 0.0f, 8.0f, 1.0f, 0.4f, 0.7f, 0.7f, 0.75f, "Heavy landing gear", stackNodes(8.0f, 1.0f), true, nullptr, nullptr},
    {17, "Solar Panel V3",   CAT_STRUCTURAL, 100.0f, 0.0f, 0.0f, 0.0f, 0.0f, 4.0f, 5.0f, 0.0f, 0.1f, 0.3f, 0.8f, "Efficient power gen", stackNodes(4.0f, 5.0f), true, nullptr, nullptr},
    {18, "Radial Decoupler", CAT_STRUCTURAL, 150.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f, 0.6f, 0.2f, 0.2f, "Side-mounted separator", stackNodes(2.0f, 1.0f), true, nullptr, nullptr},
};

static const int PART_CATALOG_SIZE = sizeof(PART_CATALOG) / sizeof(PART_CATALOG[0]);

// 5. Project Dependency Includes (after structs)
#include "core/rocket_state.h"
#include "core/agency_state.h"
#include "simulation/resource_system.h"
#include "render/renderer_2d.h"

// 6. Assembly & State Logic
struct RocketAssembly {
    std::vector<PlacedPart> parts;
    float total_dry_mass = 0, total_fuel = 0, total_height = 0, max_diameter = 0;
    float total_thrust = 0, avg_isp = 0, total_consumption = 0, total_delta_v = 0, twr = 0, total_drag = 0;
    Vec3 com = Vec3(0, 0, 0); // Center of Mass (local space)

    void recalculate() {
        total_dry_mass = 0; total_fuel = 0; total_height = 0; max_diameter = 0;
        total_thrust = 0; total_consumption = 0; total_drag = 0;
        float thrust_isp_sum = 0;

        for (size_t i = 0; i < parts.size(); i++) {
            int did = parts[i].def_id;
            if (did < 0 || did >= PART_CATALOG_SIZE) continue; 
            const PartDef& def = PART_CATALOG[did];
            float sym = (float)parts[i].symmetry;
            total_dry_mass += def.dry_mass * sym;
            total_fuel += def.fuel_capacity * sym;
            total_thrust += def.thrust * sym;
            total_consumption += def.consumption * sym;
            total_drag += def.drag_coeff * sym;
            if (def.thrust > 0) thrust_isp_sum += (def.thrust * sym) * def.isp;
            total_height = std::max(total_height, (float)parts[i].pos.y + def.height);
            max_diameter = std::max(max_diameter, def.diameter + (float)sqrt(parts[i].pos.x*parts[i].pos.x + parts[i].pos.z*parts[i].pos.z)*2.0f);
        }
        avg_isp = (total_thrust > 0) ? (thrust_isp_sum / total_thrust) : 0;
        float tm = total_dry_mass + total_fuel;
        if (total_dry_mass > 0 && avg_isp > 0) total_delta_v = (float)(avg_isp * 9.80665 * log((double)tm / (double)total_dry_mass));
        else total_delta_v = 0;
        twr = (tm > 0) ? total_thrust / (tm * 9.80665f) : 0;
    }

    void addPart(int def_id, int parent_idx = -1, int sym = 1) {
        PlacedPart p; p.def_id = def_id; p.parent_idx = parent_idx; p.symmetry = sym;
        if (parts.empty()) {
            p.pos = Vec3(0, 0, 0); // Foundational root part anchored at origin
        } else if (parent_idx == -1) {
            p.pos = Vec3(0, total_height, 0);
        }
        parts.push_back(p); recalculate();
    }

    void removePart(int idx) {
        if (idx < 0 || idx >= (int)parts.size()) return;
        // Simple removal: just erase it. In a complex tree, we'd remove children.
        parts.erase(parts.begin() + idx);
        // Correct parent indices for shifted elements
        for (auto& p : parts) if (p.parent_idx > idx) p.parent_idx--; else if (p.parent_idx == idx) p.parent_idx = -1;
        recalculate();
    }

    struct SnapResult { int parent_idx = -1; int p_node = -1; int c_node = -1; Vec3 pos = Vec3(0,0,0); float score = 1e10f; };
    SnapResult findBestSnapNode(int d_id, Vec3 rayPos, Vec3 rayDir) const {
        SnapResult best; const PartDef& dDef = PART_CATALOG[d_id];
        for (int i = 0; i < (int)parts.size(); i++) {
            const auto& pPart = parts[i];
            const PartDef& pDef = PART_CATALOG[pPart.def_id];
            for (int pn = 0; pn < (int)pDef.snap_nodes.size(); pn++) {
                // Account for parent rotation
                Vec3 nodeLocal = pDef.snap_nodes[pn].pos;
                Vec3 nodeWorld = pPart.rot.rotate(nodeLocal);
                Vec3 wPos = pPart.pos + nodeWorld;
                
                float d = (wPos - rayPos).length();
                for (int cn = 0; cn < (int)dDef.snap_nodes.size(); cn++) {
                    if (d < 5.0f && d < best.score) {
                        best.score = d; best.parent_idx = i; best.p_node = pn; best.c_node = cn;
                        best.pos = wPos - dDef.snap_nodes[cn].pos; 
                    }
                }
            }
        }
        return best;
    }
    SnapResult findSurfaceSnap(int d_id, Vec3 rayPos, Vec3 rayDir) const {
        SnapResult best; const PartDef& dDef = PART_CATALOG[d_id];
        if (!dDef.surf_attach) return best;

        for (int i = 0; i < (int)parts.size(); i++) {
            const PartDef& pDef = PART_CATALOG[parts[i].def_id];
            // Broaden snapping: allow attachment to tanks, structural, engines, and commands
            bool valid_parent = (pDef.category == CAT_FUEL_TANK || 
                                 pDef.category == CAT_STRUCTURAL || 
                                 pDef.category == CAT_ENGINE || 
                                 pDef.category == CAT_COMMAND_POD ||
                                 pDef.category == CAT_NOSE_CONE);
            if (!valid_parent) continue;
            
            float r = pDef.diameter * 0.5f;
            // Central axis of parent cylinder
            Vec3 p0 = parts[i].pos;
            Vec3 p1 = parts[i].pos + Vec3(0, pDef.height, 0);

            // Find point on central axis closest to the ray/mouse point
            float t = (rayPos.y - p0.y) / fmaxf(0.001f, pDef.height);
            if (t >= -0.1f && t <= 1.1f) {
                t = std::max(0.0f, std::min(1.0f, t));
                Vec3 axisPt = p0 + Vec3(0, t * pDef.height, 0);
                // Vector from axis to cursor
                Vec3 diff = rayPos - axisPt; diff.y = 0; // Stick to 2D radial offset
                float dist = diff.length();
                if (dist < r * 3.5f && dist > 0.01f) {
                    float score = std::abs(dist - r);
                    if (score < best.score || best.score > 2.0f) {
                        best.score = score; best.parent_idx = i;
                        best.pos = axisPt + diff.normalized() * r;
                        // Surface snap uses a specific node index to signify surface
                        best.p_node = -2; 
                    }
                }
            }
        }
        return best;
    }


    bool hasEngine() const { for (const auto& p : parts) if (PART_CATALOG[p.def_id].thrust > 0) return true; return false; }
    
    struct RocketConfig buildRocketConfig() const {
        RocketConfig cfg; 
        cfg.dry_mass = (double)total_dry_mass; cfg.height = (double)total_height; cfg.diameter = (double)max_diameter;
        cfg.stages = 1; cfg.specific_impulse = (double)avg_isp; cfg.cosrate = (double)total_consumption;
        
        float min_y = 0;
        if (!parts.empty()) {
            min_y = 1e10f;
            for (const auto& p : parts) min_y = std::min(min_y, (float)p.pos.y);
        }
        cfg.bounds_bottom = (double)min_y;

        cfg.nozzle_area = 0.5 * (max_diameter / 3.7);
        StageManager::BuildStages(*this, cfg);
        return cfg;
    }
};

struct CenterVisualizationState {
    bool showCoM = false, showCoL = false, showCoT = false;
    Vec3 comPos = Vec3(0,0,0), colPos = Vec3(0,0,0), cotPos = Vec3(0,0,0);
    bool hasCoM = false, hasCoL = false, hasCoT = false;
    size_t lastAssemblyHash = 0;
};

struct BuilderState {
    bool is_placement_valid = true;
    RocketAssembly assembly;
    CenterVisualizationState centerViz;
    int selected_category = 0, catalog_cursor = 0, assembly_cursor = -1;
    bool in_assembly_mode = false;
    int dragging_def_id = -1, dragging_parent_idx = -1, moving_part_idx = -1;
    Vec3 dragging_pos = Vec3(0,0,0); Quat dragging_rot = Quat();
    int current_symmetry = 1, hovered_part_def_id = -1;
    float hover_timer = 0, orbit_angle = 0.0f, orbit_pitch = 0.2f, cam_dist = 15.0f, orbit_speed = 0.0f, launch_blink = 0.0f;
    float cam_y_offset = 0.0f;
    Quat placement_manual_rot = Quat(); // Manual rotation adjustment during placement

    // Context Menu State
    bool show_part_menu = false;
    int r_clicked_part_idx = -1;
    Vec2 menu_pos = Vec2(0,0);
    float last_mx = 0, last_my = 0;

    void getPartsInCategory(std::vector<int>& out) const {
        out.clear();
        for (int i = 0; i < PART_CATALOG_SIZE; i++) {
            if (PART_CATALOG[i].category == selected_category) out.push_back(i);
        }
    }
};

struct BuilderKeyState {
    bool up, down, left, right, enter, del, tab, pgup, pgdn, space;
    bool q, e, a, d, w, s; 
    float mx, my; 
    bool lmb, rmb;
};

// 7. UI Drawing & Input
inline bool builderCheckHit(float mx, float my, float cx, float cy, float w, float h) {
    return (mx >= cx - w/2.0f && mx <= cx + w/2.0f && my >= cy - h/2.0f && my <= cy + h/2.0f);
}

inline void drawBuilderUI_KSP(Renderer* r, BuilderState& bs, const AgencyState& agency, float time) {
    r->addRect(0.0f, 0.92f, 2.0f, 0.12f, 0.08f, 0.10f, 0.18f, 0.8f);
    r->drawText(-0.42f, 0.925f, "ROCKET ASSEMBLY V2", 0.028f, 0.4f, 0.9f, 1.0f);
    float pl = -0.98f, pw = 0.65f; r->addRect(pl + pw/2.0f, 0.15f, pw, 1.40f, 0.05f, 0.06f, 0.10f, 0.6f);
    float tab_y = 0.82f, tab_w = pw / CAT_COUNT;
    for (int c = 0; c < CAT_COUNT; c++) {
        float tx = pl + tab_w*c + tab_w/2.0f; bool act = (c == bs.selected_category);
        r->addRect(tx, tab_y, tab_w - 0.005f, 0.06f, act?0.2f:0.1f, act?0.5f:0.2f, act?0.3f:0.2f, 0.8f);
        char abbr[4]={0}; for(int j=0;j<3&&CATEGORY_NAMES[c][j];j++) abbr[j]=CATEGORY_NAMES[c][j];
        r->drawText(tx - 0.035f, tab_y, abbr, 0.012f, act?1.0f:0.6f, act?1.0f:0.6f, act?1.0f:0.6f);
    }
    std::vector<int> cat_p; bs.getPartsInCategory(cat_p);
    float gx = pl + 0.08f, gy = 0.70f, cw = 0.16f, ch = 0.16f;
    for (int i = 0; i < (int)cat_p.size(); i++) {
        const PartDef& d = PART_CATALOG[cat_p[i]];
        float cx = gx + (i%3)*(cw+0.02f), cy = gy - (i/3)*(ch+0.02f); bool hov = (bs.hovered_part_def_id == cat_p[i]);
        r->addRect(cx, cy, cw, ch, 0.15f, 0.18f, 0.25f, hov?0.9f:0.4f);
        r->addRect(cx, cy+0.02f, cw*0.7f, ch*0.5f, d.r, d.g, d.b, 0.8f);
        r->drawText(cx-0.07f, cy-0.05f, d.name, 0.007f, 0.8f, 0.8f, 0.8f);
    }
    if (bs.hovered_part_def_id != -1) {
        const PartDef& d = PART_CATALOG[bs.hovered_part_def_id];
        r->addRect(0.0f, 0.0f, 0.5f, 0.6f, 0.08f, 0.10f, 0.15f, 0.95f);
        r->drawText(-0.22f, 0.25f, d.name, 0.02f, 0.5f, 0.9f, 1.0f);
        r->drawText(-0.22f, 0.20f, d.description, 0.009f, 0.7f, 0.7f, 0.7f);
        char buf[64]; snprintf(buf, 64, "MASS: %.0f kg", d.dry_mass); r->drawText(-0.22f, 0.12f, buf, 0.01f, 0.8f, 0.8f, 0.8f);
        if (d.fuel_capacity > 0) { snprintf(buf, 64, "FUEL: %.0f kg", d.fuel_capacity); r->drawText(-0.22f, 0.08f, buf, 0.01f, 0.9f, 0.7f, 0.2f); }
        if (d.thrust > 0) { snprintf(buf,64,"THRUST: %.0f kN", d.thrust/1000.0f); r->drawText(-0.22f, 0.04f, buf, 0.01f, 1.0f, 0.4f, 0.2f); }
    }
    float ax = 0.62f, aw = 0.35f; r->addRect(ax, 0.15f, aw, 1.40f, 0.07f, 0.08f, 0.13f, 0.5f);
    r->addRect(0.0f, -0.78f, 2.0f, 0.30f, 0.06f, 0.07f, 0.12f, 0.8f);
    r->drawInt(-0.88f, -0.82f, (int)(bs.assembly.total_dry_mass + bs.assembly.total_fuel)/1000, 0.016f, 1,1,1, 1.0f);
    r->drawInt(-0.12f, -0.82f, (int)bs.assembly.total_delta_v, 0.016f, 0.3f, 0.9f, 0.4f, 1.0f);
    r->drawInt(0.32f, -0.82f, (int)(bs.assembly.twr * 100), 0.016f, 0.3f, 0.9f, 0.4f, 1.0f);
    auto drawBtn = [&](float x, float y, const char* label, bool en, bool act, float br, float bg, float bb) {
        r->addRect(x,y,0.3f,0.05f, act?br:0.08f, act?bg:0.12f, act?bb:0.15f, en?0.8f:0.3f);
        r->drawText(x-0.10f, y, label, 0.011f, en?1.0f:0.3f, en?1.0f:0.3f, en?1.0f:0.3f);
    };
    drawBtn(ax, 0.05f, "CoM", bs.centerViz.hasCoM, bs.centerViz.showCoM, 0.15f,0.35f,0.70f);
    drawBtn(ax, -0.01f,"CoL", bs.centerViz.hasCoL, bs.centerViz.showCoL, 0.70f,0.55f,0.15f);
    drawBtn(ax, -0.07f,"CoT", bs.centerViz.hasCoT, bs.centerViz.showCoT, 0.70f,0.15f,0.15f);

    if (bs.show_part_menu && bs.r_clicked_part_idx != -1) {
        float mx = bs.menu_pos.x, my = bs.menu_pos.y, mw = 0.32f, mh = 0.68f;
        r->addRect(mx + mw/2.0f, my - mh/2.0f, mw, mh, 0.05f, 0.06f, 0.12f, 0.98f);
        const auto& p = bs.assembly.parts[bs.r_clicked_part_idx];
        const auto& d = PART_CATALOG[p.def_id];
        r->drawText(mx + 0.02f, my - 0.04f, d.name, 0.012f, 0.4f, 0.8f, 1.0f);
        
        char s1[64], s2[64];
        if (d.category == CAT_ENGINE) {
            snprintf(s1, 64, "THRUST: %.0f kN", d.thrust/1000.0f);
            snprintf(s2, 64, "ISP: %.0f s", d.isp);
        } else if (d.category == CAT_FUEL_TANK) {
            snprintf(s1, 64, "FUELCAP: %.0f kg", d.fuel_capacity);
            snprintf(s2, 64, "WETMASS: %.1f t", (d.dry_mass + d.fuel_capacity)/1000.0f);
        } else {
            snprintf(s1, 64, "DRAG: %.2f", d.drag_coeff);
            snprintf(s2, 64, "MASS: %.0f kg", d.dry_mass);
        }
        r->drawText(mx + 0.02f, my - 0.08f, s1, 0.008f, 0.7f, 0.7f, 0.7f);
        r->drawText(mx + 0.02f, my - 0.11f, s2, 0.008f, 0.7f, 0.7f, 0.7f);

        auto drawMenuBtn = [&](float menu_y, const char* label) {
            bool hov = builderCheckHit(bs.last_mx, bs.last_my, mx + mw/2.0f, menu_y, mw - 0.02f, 0.045f);
            r->addRect(mx + mw/2.0f, menu_y, mw - 0.02f, 0.045f, hov?0.2f:0.1f, hov?0.4f:0.2f, hov?0.7f:0.3f, 0.9f);
            r->drawText(mx + 0.04f, menu_y, label, 0.010f, 1, 1, 1);
        };
        
        drawMenuBtn(my - 0.17f, "DUPLICATE");
        char s_stage[32]; snprintf(s_stage, 32, "STAGE: %d (CYCLE)", p.stage);
        drawMenuBtn(my - 0.23f, s_stage);
        drawMenuBtn(my - 0.29f, "DELETE PART");

        // Advanced controls: Translation & Rotation
        float row_y = my - 0.35f;
        float btn_w = mw / 3.2f;
        auto drawAxisRow = [&](const char* label, float y) {
            r->drawText(mx + 0.02f, y, label, 0.009f, 0.8f, 0.8f, 0.2f);
            auto drawTinyBtn = [&](float x_off, const char* txt) {
                bool hov = builderCheckHit(bs.last_mx, bs.last_my, mx + x_off, y, btn_w, 0.035f);
                r->addRect(mx + x_off, y, btn_w, 0.035f, hov?0.3f:0.15f, hov?0.3f:0.15f, hov?0.4f:0.3f, 0.9f);
                r->drawText(mx + x_off - 0.02f, y, txt, 0.009f, 1, 1, 1);
            };
            drawTinyBtn(btn_w*0.6f, "+");
            drawTinyBtn(btn_w*1.6f, "-");
        };

        drawAxisRow("POS X", row_y);
        drawAxisRow("POS Y", row_y - 0.04f);
        drawAxisRow("POS Z", row_y - 0.08f);
        drawAxisRow("ROT X", row_y - 0.14f);
        drawAxisRow("ROT Y", row_y - 0.18f);
        drawAxisRow("ROT Z", row_y - 0.22f);
    }
    r->addRect(-0.85f, -0.55f, 0.15f, 0.08f, 0.2f, 0.2f, 0.3f, 0.8f);
    char s_sym[16]; snprintf(s_sym, 16, "SYM: %dx", bs.current_symmetry); r->drawText(-0.91f,-0.55f,s_sym, 0.012f, 1,1,1);
    if (bs.assembly.hasEngine() && !bs.assembly.parts.empty()) {
        float blink = 0.5f + 0.5f*sinf(time*3.0f); r->addRect(0.0f,-0.93f,0.60f,0.08f, 0.05f*blink, 0.25f*blink, 0.05f*blink, 0.8f);
    }
}

inline bool builderHandleInput(BuilderState& bs, const BuilderKeyState& k, const BuilderKeyState& pk) {
    bs.last_mx = k.mx; bs.last_my = k.my;
    // Context Menu Interactivity (Highest Priority)
    if (bs.show_part_menu && k.lmb && !pk.lmb) {
        float mx = bs.menu_pos.x, my = bs.menu_pos.y, mw = 0.32f, mh = 0.68f;
        auto& p = bs.assembly.parts[bs.r_clicked_part_idx];
        
        // Button 1: DUPLICATE
        if (builderCheckHit(k.mx, k.my, mx + mw/2.0f, my - 0.17f, mw, 0.045f)) {
            bs.dragging_def_id = p.def_id;
            bs.moving_part_idx = -1;
            bs.show_part_menu = false; return false; 
        }
        // Button 2: CYCLE STAGE
        if (builderCheckHit(k.mx, k.my, mx + mw/2.0f, my - 0.23f, mw, 0.045f)) {
            p.stage = (p.stage + 1) % 5;
            bs.assembly.recalculate();
            return false;
        }
        // Button 3: DELETE
        if (builderCheckHit(k.mx, k.my, mx + mw/2.0f, my - 0.29f, mw, 0.045f)) {
            bs.assembly.removePart(bs.r_clicked_part_idx);
            bs.show_part_menu = false; return false;
        }

        // Advanced controls (Translation & Rotation)
        float row_y = my - 0.35f;
        float btn_w = mw / 3.2f;
        auto checkAxisBtn = [&](float x_off, float y_off, double& val, float step) {
            if (builderCheckHit(k.mx, k.my, mx + x_off, row_y + y_off, btn_w, 0.035f)) { val += step; bs.assembly.recalculate(); return true; }
            return false;
        };
        auto checkRotBtn = [&](float x_off, float y_off, Vec3 axis, float angle_deg) {
            if (builderCheckHit(k.mx, k.my, mx + x_off, row_y + y_off, btn_w, 0.035f)) {
                p.rot = p.rot * Quat::fromAxisAngle(axis, angle_deg * (float)PI / 180.0f);
                p.rot = p.rot.normalized();
                bs.assembly.recalculate(); return true;
            }
            return false;
        };

        if (checkAxisBtn(btn_w*0.6f, 0, p.pos.x, 0.1f)) return false;
        if (checkAxisBtn(btn_w*1.6f, 0, p.pos.x, -0.1f)) return false;
        if (checkAxisBtn(btn_w*0.6f, -0.04f, p.pos.y, 0.1f)) return false;
        if (checkAxisBtn(btn_w*1.6f, -0.04f, p.pos.y, -0.1f)) return false;
        if (checkAxisBtn(btn_w*0.6f, -0.08f, p.pos.z, 0.1f)) return false;
        if (checkAxisBtn(btn_w*1.6f, -0.08f, p.pos.z, -0.1f)) return false;

        if (checkRotBtn(btn_w*0.6f, -0.14f, Vec3(1,0,0), 5.0f)) return false;
        if (checkRotBtn(btn_w*1.6f, -0.14f, Vec3(1,0,0), -5.0f)) return false;
        if (checkRotBtn(btn_w*0.6f, -0.18f, Vec3(0,1,0), 5.0f)) return false;
        if (checkRotBtn(btn_w*1.6f, -0.18f, Vec3(0,1,0), -5.0f)) return false;
        if (checkRotBtn(btn_w*0.6f, -0.22f, Vec3(0,0,1), 5.0f)) return false;
        if (checkRotBtn(btn_w*1.6f, -0.22f, Vec3(0,0,1), -5.0f)) return false;

        // Project a click outside menu or specialized "Close" area
        if (!builderCheckHit(k.mx, k.my, mx + mw/2.0f, my - mh/2.0f, mw, mh)) {
            bs.show_part_menu = false; 
        }
    }
    
    int dragging_id_at_start = bs.dragging_def_id;
    bs.hovered_part_def_id = -1;
    float pl=-0.98f,pw=0.65f,gx=pl+0.08f,gy=0.70f,cw=0.16f,ch=0.16f;
    std::vector<int> cat_p; bs.getPartsInCategory(cat_p);
    
    // Check catalog hits
    bool over_catalog = (k.mx < pl + pw);
    bool over_icon = false;
    for (int i=0;i<(int)cat_p.size();i++) {
        float cx=gx+(i%3)*(cw+0.02f), cy=gy-(i/3)*(ch+0.02f);
        if (builderCheckHit(k.mx,k.my,cx,cy,cw,ch)) {
            bs.hovered_part_def_id=cat_p[i];
            over_icon = true;
            if(k.lmb && !pk.lmb) { 
                // Pick from catalog
                bs.dragging_def_id=cat_p[i]; 
                bs.moving_part_idx = -1;
                bs.in_assembly_mode = false; 
                bs.placement_manual_rot = Quat(); // Reset manual rotation
            }
        }
    }

    // Tabs
    float tab_y=0.82f, tab_w=pw/CAT_COUNT;
    for(int c=0;c<CAT_COUNT;c++) {
        float tx=pl+tab_w*c+tab_w/2.0f;
        if(builderCheckHit(k.mx,k.my,tx,tab_y,tab_w,0.06f)&&k.lmb&&!pk.lmb) bs.selected_category=c;
    }

    // Camera Sync with main.cpp: Dynamic target and distance
    float current_height = std::max(5.0f, bs.assembly.total_height);
    float look_y = current_height * 0.4f + bs.cam_y_offset;
    float view_dist = bs.cam_dist + current_height * 0.5f;
    Vec3 camTarget(0, look_y, 0);

    // Pick/Right-click from assembly (Improved Proximity/Picking Logic)
    if (bs.dragging_def_id == -1 && !over_catalog && ((k.lmb && !pk.lmb) || (k.rmb && !pk.rmb))) {
        float best_d = 0.15f; int best_idx = -1;
        float cosA = cosf(bs.orbit_angle), sinA = sinf(bs.orbit_angle);
        float cosP = cosf(bs.orbit_pitch), sinP = sinf(bs.orbit_pitch);
        Vec3 camRight(sinA, 0, -cosA);
        Vec3 camUp(-cosA * sinP, cosP, -sinA * sinP);

        for(int i=0; i<(int)bs.assembly.parts.size(); i++) {
            const auto& p = bs.assembly.parts[i];
            const auto& def = PART_CATALOG[p.def_id];
            Vec3 testPoints[] = { p.pos, p.pos + Vec3(0, def.height * 0.5f, 0) };
            for (const auto& pt : testPoints) {
                Vec3 relPos = pt - camTarget;
                float px = relPos.dot(camRight) / (view_dist * 0.5f * 1.6f);
                float py = relPos.dot(camUp) / (view_dist * 0.5f);
                float dx = px - k.mx; float dy = py - k.my;
                float d = sqrtf(dx*dx + dy*dy);
                if (d < best_d) { best_d = d; best_idx = i; }
            }
        }
        
        if (k.lmb && !pk.lmb && best_idx != -1) {
            bs.dragging_def_id = bs.assembly.parts[best_idx].def_id;
            bs.moving_part_idx = best_idx;
            bs.dragging_pos = bs.assembly.parts[best_idx].pos;
            bs.assembly.removePart(best_idx);
            dragging_id_at_start = bs.dragging_def_id; 
            bs.show_part_menu = false;
        } else if (k.rmb && !pk.rmb) {
            if (best_idx != -1) {
                bs.show_part_menu = true;
                bs.r_clicked_part_idx = best_idx;
                bs.menu_pos = Vec2(k.mx, k.my);
            } else {
                bs.show_part_menu = false;
            }
        }
    }

    // Dragging Logic (Sticky)
    if (bs.dragging_def_id != -1) {
        float cosA = cosf(bs.orbit_angle), sinA = sinf(bs.orbit_angle);
        float cosP = cosf(bs.orbit_pitch), sinP = sinf(bs.orbit_pitch);
        Vec3 camRight(sinA, 0, -cosA);
        Vec3 camUp(-cosA * sinP, cosP, -sinA * sinP);
        Vec3 camFwd(-cosA * cosP, -sinP, -sinA * cosP);

        float aspect = 1.6f; 
        float scale = view_dist * 0.5f;
        // Project relative to camTarget
        Vec3 mouseWorldPosRelative = camRight * (k.mx * scale * aspect) + camUp * (k.my * scale);
        Vec3 mouseWorldPos = camTarget + mouseWorldPosRelative;
        
        auto snap = bs.assembly.findBestSnapNode(bs.dragging_def_id, mouseWorldPos, camFwd);
        auto surf = bs.assembly.findSurfaceSnap(bs.dragging_def_id, mouseWorldPos, camFwd);
        
        if (snap.parent_idx != -1 && snap.score < 5.0f) { 
            bs.dragging_pos = snap.pos; 
            bs.dragging_parent_idx = snap.parent_idx; 
            bs.is_placement_valid = true;
            // Snapped parts inherit parent rotation + manual shift
            bs.dragging_rot = bs.assembly.parts[snap.parent_idx].rot * bs.placement_manual_rot;
        } else if (surf.parent_idx != -1 && surf.score < 2.0f) {
            bs.dragging_pos = surf.pos;
            bs.dragging_parent_idx = surf.parent_idx;
            bs.is_placement_valid = true;
            
            // Outward rotation for surface attachment
            Vec3 normal = (surf.pos - bs.assembly.parts[surf.parent_idx].pos); normal.y = 0;
            Quat surf_rot = Quat();
            if (normal.length() > 0.01f) {
                float angle = atan2(normal.z, normal.x);
                surf_rot = Quat::fromAxisAngle(Vec3(0, 1, 0), -angle + (float)PI/2.0f);
            }
            bs.dragging_rot = surf_rot * bs.placement_manual_rot;
        } else { 
            if (bs.assembly.parts.empty()) {
                bs.dragging_pos = Vec3(0, 0, 0); // Snap first part to origin
                bs.is_placement_valid = true;
            } else {
                bs.dragging_pos = mouseWorldPos; 
                bs.is_placement_valid = PART_CATALOG[bs.dragging_def_id].surf_attach;
            }
            bs.dragging_parent_idx = -1;
            bs.dragging_rot = bs.placement_manual_rot;
        }

        // --- Keyboard Rotation Controls ---
        float rot_step = 90.0f * (float)PI / 180.0f;
        if (k.q && !pk.q) bs.placement_manual_rot = bs.placement_manual_rot * Quat::fromAxisAngle(Vec3(0, 1, 0), rot_step);
        if (k.e && !pk.e) bs.placement_manual_rot = bs.placement_manual_rot * Quat::fromAxisAngle(Vec3(0, 1, 0), -rot_step);
        if (k.w && !pk.w) bs.placement_manual_rot = bs.placement_manual_rot * Quat::fromAxisAngle(Vec3(1, 0, 0), rot_step);
        if (k.s && !pk.s) bs.placement_manual_rot = bs.placement_manual_rot * Quat::fromAxisAngle(Vec3(1, 0, 0), -rot_step);
        if (k.a && !pk.a) bs.placement_manual_rot = bs.placement_manual_rot * Quat::fromAxisAngle(Vec3(0, 0, 1), rot_step);
        if (k.d && !pk.d) bs.placement_manual_rot = bs.placement_manual_rot * Quat::fromAxisAngle(Vec3(0, 0, 1), -rot_step);
        bs.placement_manual_rot = bs.placement_manual_rot.normalized();

        // Place or Delete
        if (k.lmb && !pk.lmb && dragging_id_at_start != -1) {
            bool picked_this_frame = (bs.moving_part_idx != -1 && !pk.lmb); // Simple heuristic: if we were moving and just pressed LMB, it might be the same click
            // Actually, a better check: did dragging_id_at_start just become valid?
            // Since we set bs.dragging_def_id = -1 at the end of placement, we can check if it was -1 in the previous state's handle input call.
            // But we don't have pk's BuilderState here. Let's use a more direct check.
            
            if (over_catalog) {
                bs.dragging_def_id = -1;
                bs.moving_part_idx = -1;
            } else if (bs.is_placement_valid && bs.hover_timer > 0.05f) { // Add a tiny delay (1-2 frames) before allowing placement
                bs.assembly.addPart(bs.dragging_def_id, bs.dragging_parent_idx, bs.current_symmetry);
                auto& newP = bs.assembly.parts.back();
                newP.pos = bs.dragging_pos;
                newP.rot = bs.dragging_rot; // Apply the rotated orientation
                bs.dragging_def_id = -1; 
                bs.moving_part_idx = -1;
                bs.show_part_menu = false; // Cancel selection/menu after placement
            }
        }
        bs.hover_timer += 0.016f; // Update timer while dragging
    } else {
        bs.hover_timer = 0; // Reset timer when not dragging
    }

    // Symmetry & UI Buttons
    if (k.mx < -0.7f && k.my < -0.5f && k.lmb && !pk.lmb) {
        int syms[] = {1,2,4,6,8}; static int s_idx=0; s_idx=(s_idx+1)%5; bs.current_symmetry=syms[s_idx];
    }
    float ax=0.62f;
    if (builderCheckHit(k.mx,k.my, ax, 0.05f, 0.3f, 0.05f)&&k.lmb&&!pk.lmb) bs.centerViz.showCoM=!bs.centerViz.showCoM;
    if (builderCheckHit(k.mx,k.my, ax, -0.01f, 0.3f, 0.05f)&&k.lmb&&!pk.lmb) bs.centerViz.showCoL=!bs.centerViz.showCoL;
    if (builderCheckHit(k.mx,k.my, ax, -0.07f, 0.3f, 0.05f)&&k.lmb&&!pk.lmb) bs.centerViz.showCoT=!bs.centerViz.showCoT;
    if (k.space && !pk.space && !bs.assembly.parts.empty() && bs.assembly.hasEngine()) return true;
    return false;
}
