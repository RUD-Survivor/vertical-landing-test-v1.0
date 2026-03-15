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
};

struct PlacedPart {
    int def_id;          
    int stage;
    Vec3 pos;            
    Quat rot;            
    int parent_idx;      
    int symmetry;        
    float stack_y;       
    float param0; 

    PlacedPart() : def_id(0), stage(0), pos(Vec3(0,0,0)), rot(Quat()), parent_idx(-1), symmetry(1), stack_y(0), param0(1.0f) {}
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
    {0,  "Standard Fairing", CAT_NOSE_CONE, 500.0f, 0.0f, 0.0f, 0.0f, 0.0f, 8.0f, 3.7f, 0.2f, 0.85f, 0.85f, 0.90f, "Basic aerodynamic fairing", stackNodes(0, 3.7f), false},
    {1,  "Aero Nosecone",    CAT_NOSE_CONE, 350.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 3.7f, 0.15f, 0.70f, 0.70f, 0.75f, "Low-drag pointed nosecone", stackNodes(0, 3.7f), false},
    {2,  "Payload Bay",      CAT_NOSE_CONE, 1200.0f, 0.0f, 0.0f, 0.0f, 0.0f, 12.0f, 4.0f, 0.3f, 0.90f, 0.90f, 0.85f, "Large payload enclosure", stackNodes(0, 4.0f), false},
    {3,  "Mk1 Capsule",      CAT_COMMAND_POD, 1500.0f, 0.0f, 0.0f, 0.0f, 0.0f, 4.0f, 2.5f, 0.25f, 0.30f, 0.30f, 0.35f, "Single-crew capsule", stackNodes(4.0f, 2.5f), false},
    {4,  "Probe Core",       CAT_COMMAND_POD, 200.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.1f, 0.20f, 0.25f, 0.20f, "Guidance unit", stackNodes(1.0f, 1.0f), false},
    {5,  "Small Tank 50t",   CAT_FUEL_TANK, 3000.0f, 50000.0f, 0.0f, 0.0f, 0.0f, 25.0f, 3.7f, 0.3f, 0.92f, 0.92f, 0.92f, "Small fuel storage", stackNodes(25.0f, 3.7f), false},
    {6,  "Medium Tank 100t", CAT_FUEL_TANK, 5000.0f, 100000.0f, 0.0f, 0.0f, 0.0f, 35.0f, 3.7f, 0.3f, 0.90f, 0.90f, 0.92f, "Standard fuel tank", stackNodes(35.0f, 3.7f), false},
    {7,  "Large Tank 200t",  CAT_FUEL_TANK, 8000.0f, 200000.0f, 0.0f, 0.0f, 0.0f, 50.0f, 3.7f, 0.3f, 0.88f, 0.88f, 0.92f, "High-capacity tank", stackNodes(50.0f, 3.7f), false},
    {8,  "Jumbo Tank 500t",  CAT_FUEL_TANK, 15000.0f, 500000.0f, 0.0f, 0.0f, 0.0f, 80.0f, 5.0f, 0.3f, 0.85f, 0.85f, 0.90f, "Massive reservoir", stackNodes(80.0f, 5.0f), false},
    {18, "Gold Leaf Tank",   CAT_FUEL_TANK, 1000.0f, 20000.0f, 0.0f, 0.0f, 0.0f, 12.0f, 2.0f, 0.2f, 1.0f, 0.84f, 0.0f, "Titanium-gold tank", stackNodes(12.0f, 2.0f), false},
    {9,  "Raptor",           CAT_ENGINE, 2000.0f, 0.0f, 1500.0f, 2200000.0f, 100.0f, 4.0f, 3.7f, 0.05f, 0.35f, 0.35f, 0.38f, "High-ISP engine", stackNodes(4.0f, 3.7f), false},
    {10, "Merlin 1D",        CAT_ENGINE, 1500.0f, 0.0f, 1200.0f, 845000.0f, 80.0f, 3.0f, 3.0f, 0.05f, 0.40f, 0.35f, 0.30f, "Workhorse engine", stackNodes(3.0f, 3.0f), false},
    {19, "Vacuum Engine",    CAT_ENGINE, 800.0f, 0.0f, 1800.0f, 300000.0f, 20.0f, 5.0f, 4.2f, 0.02f, 0.6f, 0.6f, 0.7f, "Deep space efficient", stackNodes(5.0f, 4.2f), false},
    {15, "Decoupler",        CAT_STRUCTURAL, 300.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 3.7f, 0.0f, 0.25f, 0.25f, 0.25f, "Stage separator", stackNodes(1.0f, 3.7f), false},
    {16, "Fin Set (x4)",     CAT_STRUCTURAL, 200.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 3.7f, -0.1f, 0.30f, 0.30f, 0.30f, "Stability fins", stackNodes(2.0f, 3.7f), true}, 
    {17, "Girder XL",        CAT_STRUCTURAL, 400.0f, 0.0f, 0.0f, 0.0f, 0.0f, 6.0f, 1.0f, 0.1f, 0.40f, 0.40f, 0.42f, "Structural beam", stackNodes(6.0f, 1.0f), false},
    {20, "Landing Leg",      CAT_STRUCTURAL, 500.0f, 0.0f, 0.0f, 0.0f, 0.0f, 8.0f, 1.0f, 0.4f, 0.7f, 0.7f, 0.75f, "Heavy landing gear", stackNodes(8.0f, 1.0f), true},
    {21, "Solar Panel V3",   CAT_STRUCTURAL, 100.0f, 0.0f, 0.0f, 0.0f, 0.0f, 4.0f, 5.0f, 0.0f, 0.1f, 0.3f, 0.8f, "Efficient power gen", stackNodes(4.0f, 5.0f), true},
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

    void recalculate() {
        total_dry_mass = 0; total_fuel = 0; total_height = 0; max_diameter = 0;
        total_thrust = 0; total_consumption = 0; total_drag = 0;
        float thrust_isp_sum = 0;

        for (size_t i = 0; i < parts.size(); i++) {
            const PartDef& def = PART_CATALOG[parts[i].def_id];
            float sym = (float)parts[i].symmetry;
            float p0 = parts[i].param0;
            
            total_dry_mass += def.dry_mass * sym;
            
            float part_fuel = def.fuel_capacity * sym;
            if (def.category == CAT_FUEL_TANK) part_fuel *= p0;
            total_fuel += part_fuel;

            float part_thrust = def.thrust * sym;
            float part_cons = def.consumption * sym;
            if (def.category == CAT_ENGINE) {
                part_thrust *= p0;
                part_cons *= p0;
            }
            
            total_thrust += part_thrust;
            total_consumption += part_cons;
            total_drag += def.drag_coeff * sym;
            
            if (part_thrust > 0) thrust_isp_sum += part_thrust * def.isp;
            
            total_height = std::max(total_height, parts[i].pos.y + def.height);
            max_diameter = std::max(max_diameter, def.diameter + sqrtf(parts[i].pos.x*parts[i].pos.x + parts[i].pos.z*parts[i].pos.z)*2.0f);
        }
        avg_isp = (total_thrust > 0) ? (thrust_isp_sum / total_thrust) : 0;
        float tm = total_dry_mass + total_fuel;
        if (total_dry_mass > 0 && avg_isp > 0) total_delta_v = (float)(avg_isp * 9.80665 * log((double)tm / (double)total_dry_mass));
        else total_delta_v = 0;
        twr = (tm > 0) ? total_thrust / (tm * 9.80665f) : 0;
    }

    void addPart(int def_id, int parent_idx = -1, int sym = 1) {
        PlacedPart p; p.def_id = def_id; p.parent_idx = parent_idx; p.symmetry = sym;
        if (parent_idx == -1 && !parts.empty()) p.pos = Vec3(0, total_height, 0);
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
            const PartDef& pDef = PART_CATALOG[parts[i].def_id];
            for (int pn = 0; pn < (int)pDef.snap_nodes.size(); pn++) {
                Vec3 wPos = parts[i].pos + pDef.snap_nodes[pn].pos;
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

    bool hasEngine() const { for (const auto& p : parts) if (PART_CATALOG[p.def_id].thrust > 0) return true; return false; }
    
    struct RocketConfig buildRocketConfig() const {
        RocketConfig cfg; 
        cfg.dry_mass = (double)total_dry_mass; cfg.height = (double)total_height; cfg.diameter = (double)max_diameter;
        cfg.stages = 1; cfg.specific_impulse = (double)avg_isp; cfg.cosrate = (double)total_consumption;
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
    float cam_pan_y = 0.0f;
    Vec2 rmb_down_pos = Vec2(0,0);
    RocketConfig cached_cfg;
    bool stats_dirty = true;
    int context_menu_part_idx = -1;

    void getPartsInCategory(std::vector<int>& out) const {
        out.clear();
        for (int i = 0; i < PART_CATALOG_SIZE; i++) {
            if (PART_CATALOG[i].category == selected_category) out.push_back(i);
        }
    }
};

struct BuilderKeyState {
    bool up, down, left, right, enter, del, tab, pgup, pgdn, space;
    bool w, s, a, d, q, e, shift;
    float mx, my; bool lmb, rmb;
};

// 7. UI Drawing & Input
inline bool builderCheckHit(float mx, float my, float cx, float cy, float w, float h) {
    return (mx >= cx - w/2.0f && mx <= cx + w/2.0f && my >= cy - h/2.0f && my <= cy + h/2.0f);
}inline void drawBuilderUI_KSP(Renderer* r, BuilderState& bs, const AgencyState& agency, float time) {
    if (bs.stats_dirty) {
        bs.cached_cfg = bs.assembly.buildRocketConfig();
        bs.stats_dirty = false;
    }

    // Top Header - Glassmorphic
    r->addRect(0.0f, 0.94f, 2.0f, 0.12f, 0.05f, 0.08f, 0.15f, 0.85f);
    r->drawText(-0.15f, 0.95f, "VEHICLE ASSEMBLY BUILDING", 0.022f, 0.6f, 0.8f, 1.0f);
    
    // Left Catalog Panel - Glassmorphic
    float pl = -1.0f, pw = 0.55f; 
    r->addRect(pl + pw/2.0f, 0.15f, pw, 1.45f, 0.04f, 0.06f, 0.12f, 0.7f);
    
    // Tabs
    float tab_y = 0.84f, tab_w = pw / CAT_COUNT;
    for (int c = 0; c < CAT_COUNT; c++) {
        float tx = pl + tab_w*c + tab_w/2.0f; bool act = (c == bs.selected_category);
        r->addRect(tx, tab_y, tab_w - 0.005f, 0.05f, act?0.15f:0.08f, act?0.4f:0.12f, act?0.6f:0.18f, 0.9f);
        char abbr[8]={0}; for(int j=0;j<4&&CATEGORY_NAMES[c][j];j++) abbr[j]=CATEGORY_NAMES[c][j];
        r->drawText(tx - 0.04f, tab_y, abbr, 0.010f, act?1.0f:0.5f, act?1.0f:0.5f, act?1.0f:0.5f);
    }

    // Parts Grid
    std::vector<int> cat_p; bs.getPartsInCategory(cat_p);
    float gx = pl + 0.10f, gy = 0.74f, cw = 0.16f, ch = 0.16f;
    for (int i = 0; i < (int)cat_p.size(); i++) {
        const PartDef& d = PART_CATALOG[cat_p[i]];
        float cx = gx + (i%2)*(cw+0.04f), cy = gy - (i/2)*(ch+0.04f); 
        bool hov = (bs.hovered_part_def_id == cat_p[i]);
        r->addRect(cx, cy, cw, ch, 0.12f, 0.15f, 0.22f, hov?1.0f:0.4f);
        r->addRect(cx, cy+0.02f, cw*0.7f, ch*0.5f, d.r, d.g, d.b, 0.8f);
        r->drawText(cx-0.075f, cy-0.06f, d.name, 0.006f, 1.0f, 1.0f, 1.0f);
    }

    // Right Staging Sidebar
    float sx = 0.75f, sw = 0.50f;
    r->addRect(sx + sw/2.0f, 0.15f, sw, 1.45f, 0.04f, 0.05f, 0.08f, 0.7f);
    r->drawText(sx + 0.05f, 0.84f, "STAGING", 0.015f, 0.5f, 0.8f, 1.0f);

    float stage_y = 0.78f;
    for (int i = 0; i < bs.cached_cfg.stages; i++) {
        const auto& sc = bs.cached_cfg.stage_configs[i];
        int stage_num = bs.cached_cfg.stages - 1 - i;
        r->addRect(sx + sw/2.0f, stage_y, sw - 0.04f, 0.12f, 0.1f, 0.12f, 0.18f, 0.6f);
        char sbuf[32]; snprintf(sbuf, 32, "STAGE %d", stage_num);
        r->drawText(sx + 0.04f, stage_y + 0.03f, sbuf, 0.012f, 0.4f, 0.7f, 1.0f);
        
        // Find parts in this stage
        float part_icon_x = sx + 0.12f;
        for (int pidx = 0; pidx < (int)bs.assembly.parts.size(); pidx++) {
            const auto& pp = bs.assembly.parts[pidx];
            if (pp.stage == stage_num) {
                const auto& pdef = PART_CATALOG[pp.def_id];
                if (pdef.category == CAT_ENGINE || (pdef.category == CAT_STRUCTURAL && pp.def_id == 15)) {
                     r->addRect(part_icon_x, stage_y - 0.02f, 0.05f, 0.05f, pdef.r, pdef.g, pdef.b, 0.8f);
                     part_icon_x += 0.07f;
                }
            }
        }

        // Compute delta-v for this stage (simplified)
        float dv = (sc.thrust > 0) ? (sc.specific_impulse * 9.80665f * logf( (sc.dry_mass + sc.fuel_capacity + 100) / (sc.dry_mass + 100)) ) : 0;
        snprintf(sbuf, 32, "dV: %.0f", dv);
        r->drawText(sx + 0.35f, stage_y + 0.02f, sbuf, 0.008f, 0.3f, 0.9f, 0.4f);
        
        stage_y -= 0.14f;
    }

    // Hover Info Card
    if (bs.hovered_part_def_id != -1) {
        const PartDef& d = PART_CATALOG[bs.hovered_part_def_id];
        r->addRect(0.0f, 0.0f, 0.6f, 0.5f, 0.06f, 0.08f, 0.12f, 0.98f);
        r->drawText(-0.25f, 0.20f, d.name, 0.022f, 0.4f, 0.8f, 1.0f);
        r->drawText(-0.25f, 0.14f, d.description, 0.009f, 0.7f, 0.7f, 0.7f);
        char buf[64]; 
        snprintf(buf, 64, "MASS: %.0f kg", d.dry_mass); r->drawText(-0.25f, 0.06f, buf, 0.01f, 0.9f, 0.9f, 0.9f);
        if (d.fuel_capacity > 0) { snprintf(buf, 64, "FUEL: %.0f kg", d.fuel_capacity); r->drawText(-0.25f, 0.02f, buf, 0.01f, 0.9f, 0.7f, 0.2f); }
        if (d.thrust > 0) { 
            snprintf(buf,64,"THRUST: %.0f kN", d.thrust/1000.0f); r->drawText(-0.25f, -0.02f, buf, 0.01f, 1.0f, 0.4f, 0.2f); 
            snprintf(buf,64,"ISP: %.0f s", d.isp); r->drawText(-0.25f, -0.06f, buf, 0.01f, 0.2f, 0.8f, 1.0f);
        }
    }

    // Bottom Stats Bar
    r->addRect(0.0f, -0.85f, 2.0f, 0.25f, 0.03f, 0.04f, 0.08f, 0.9f);
    float val_y = -0.82f;
    r->drawText(-0.90f, val_y + 0.05f, "TOTAL MASS", 0.008f, 0.5f, 0.5f, 0.5f);
    r->drawInt(-0.90f, val_y, (int)(bs.assembly.total_dry_mass + bs.assembly.total_fuel)/1000, 0.025f, 1,1,1, 1.0f);
    r->drawText(-0.82f, val_y, "t", 0.012f, 0.5f, 0.5f, 0.5f);

    r->drawText(-0.15f, val_y + 0.05f, "TOTAL DELTA-V", 0.008f, 0.3f, 0.6f, 0.3f);
    r->drawInt(-0.15f, val_y, (int)bs.assembly.total_delta_v, 0.025f, 0.3f, 0.9f, 0.4f, 1.0f);
    r->drawText(-0.06f, val_y, "m/s", 0.012f, 0.3f, 0.6f, 0.3f);

    r->drawText(0.35f, val_y + 0.05f, "START TWR", 0.008f, 0.6f, 0.5f, 0.3f);
    char twrbuf[16]; snprintf(twrbuf, 16, "%.2f", bs.assembly.twr);
    r->drawText(0.35f, val_y, twrbuf, 0.025f, 0.9f, 0.7f, 0.2f);

    if (bs.assembly.hasEngine() && !bs.assembly.parts.empty()) {
        float blink = 0.6f + 0.4f*sinf(time*4.0f); 
        r->addRect(0.0f, -0.95f, 0.4f, 0.08f, 0.05f, 0.4f*blink, 0.1f, 0.85f);
        r->drawText(-0.13f, -0.95f, "[SPACE] LAUNCH", 0.015f, 0.8f, 1.0f, 0.8f);
    }

    // Part Context Menu
    if (bs.context_menu_part_idx != -1) {
        const auto& pp = bs.assembly.parts[bs.context_menu_part_idx];
        const auto& def = PART_CATALOG[pp.def_id];
        r->addRect(0.0f, -0.2f, 0.5f, 0.35f, 0.05f, 0.07f, 0.15f, 0.95f);
        r->drawText(-0.2f, -0.08f, "ADJUST PARAMETERS", 0.012f, 0.5f, 0.8f, 1.0f);
        
        char pbuf[64];
        const char* p_name = (def.category == CAT_ENGINE) ? "THRUST LIMIT" : "FUEL LEVEL";
        snprintf(pbuf, 64, "%s: %.0f%%", p_name, pp.param0 * 100.0f);
        r->drawText(-0.2f, -0.14f, pbuf, 0.01f, 1,1,1);
        
        // Pseudo-buttons for parameter adjustment
        r->addRect(0.0f, -0.20f, 0.45f, 0.06f, 0.15f, 0.2f, 0.3f, 0.8f);
        r->drawText(-0.18f, -0.20f, "[-] DECREASE 10%", 0.01f, 1,1,1);
        r->addRect(0.0f, -0.28f, 0.45f, 0.06f, 0.15f, 0.2f, 0.3f, 0.8f);
        r->drawText(-0.18f, -0.28f, "[+] INCREASE 10%", 0.01f, 1,1,1);
        
        r->addRect(0.0f, -0.36f, 0.45f, 0.06f, 0.15f, 0.2f, 0.3f, 0.8f);
        char s_buf[32]; snprintf(s_buf, 32, "CYCLE STAGE (NOW: %d)", pp.stage);
        r->drawText(-0.18f, -0.36f, s_buf, 0.01f, 1,1,1);
    }
}

inline bool builderHandleInput(BuilderState& bs, const BuilderKeyState& k, const BuilderKeyState& pk) {
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
                bs.dragging_rot = Quat(); // Reset rotation for new part
                bs.moving_part_idx = -1;
                bs.in_assembly_mode = false; 
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
    float cur_ht = std::max(5.0f, bs.assembly.total_height);
    float look_y = cur_ht * 0.4f + bs.cam_pan_y;
    float view_dist = bs.cam_dist + cur_ht * 0.5f;
    Vec3 camTarget(0, look_y, 0);

    // Pick from assembly (Improved Proximity/Picking Logic)
    if (bs.dragging_def_id == -1 && k.lmb && !pk.lmb && !over_catalog) {
        float best_d = 0.8f; int best_idx = -1;
        float cosA = cosf(bs.orbit_angle), sinA = sinf(bs.orbit_angle);
        float cosP = cosf(bs.orbit_pitch), sinP = sinf(bs.orbit_pitch);
        Vec3 camRight(sinA, 0, -cosA);
        Vec3 camUp(-cosA * sinP, cosP, -sinA * sinP);

        for(int i=0; i<(int)bs.assembly.parts.size(); i++) {
            const auto& p = bs.assembly.parts[i];
            const auto& def = PART_CATALOG[p.def_id];
            Vec3 testPoints[] = { p.pos, p.pos + Vec3(0, def.height * 0.5f, 0) };
            for (const auto& pt : testPoints) {
                // Account for camTarget in projection
                Vec3 relPos = pt - camTarget;
                float px = relPos.dot(camRight) / (view_dist * 0.5f * 1.6f);
                float py = relPos.dot(camUp) / (view_dist * 0.5f);
                float dx = px - k.mx;
                float dy = py - k.my;
                float d = sqrtf(dx*dx + dy*dy);
                if (d < best_d) { best_d = d; best_idx = i; }
            }
        }
        if (best_idx != -1) {
            bs.dragging_def_id = bs.assembly.parts[best_idx].def_id;
            bs.moving_part_idx = best_idx;
            bs.dragging_pos = bs.assembly.parts[best_idx].pos;
            bs.assembly.removePart(best_idx);
            dragging_id_at_start = bs.dragging_def_id; 
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
        if (snap.parent_idx != -1 && snap.score < 5.0f) { 
            bs.dragging_pos = snap.pos; 
            bs.dragging_parent_idx = snap.parent_idx; 
            bs.is_placement_valid = true;
        } else { 
            bs.dragging_pos = mouseWorldPos; 
            bs.dragging_parent_idx = -1;
            bs.is_placement_valid = bs.assembly.parts.empty() || PART_CATALOG[bs.dragging_def_id].surf_attach;
        }

        // Rotation Gizmos (WSADQE)
        float step = 3.14159f / 2.0f; // 90 degrees
        if (k.w && !pk.w) bs.dragging_rot = bs.dragging_rot * Quat::fromAxisAngle(Vec3(1,0,0), step);
        if (k.s && !pk.s) bs.dragging_rot = bs.dragging_rot * Quat::fromAxisAngle(Vec3(1,0,0), -step);
        if (k.a && !pk.a) bs.dragging_rot = bs.dragging_rot * Quat::fromAxisAngle(Vec3(0,1,0), step);
        if (k.d && !pk.d) bs.dragging_rot = bs.dragging_rot * Quat::fromAxisAngle(Vec3(0,1,0), -step);
        if (k.q && !pk.q) bs.dragging_rot = bs.dragging_rot * Quat::fromAxisAngle(Vec3(0,0,1), step);
        if (k.e && !pk.e) bs.dragging_rot = bs.dragging_rot * Quat::fromAxisAngle(Vec3(0,0,1), -step);

        // Place or Delete
        if (k.lmb && !pk.lmb && dragging_id_at_start != -1) {
            if (over_catalog) {
                bs.dragging_def_id = -1;
                bs.moving_part_idx = -1;
            } else if (bs.is_placement_valid) {
                bs.assembly.addPart(bs.dragging_def_id, bs.dragging_parent_idx, bs.current_symmetry);
                bs.assembly.parts.back().pos = bs.dragging_pos;
                bs.assembly.parts.back().rot = bs.dragging_rot;
                bs.dragging_def_id = -1; 
                bs.moving_part_idx = -1;
                bs.stats_dirty = true;
            }
        }
    }

    // Context Menu (Right Click - Triggered on Release if minimal movement)
    if (bs.dragging_def_id == -1 && !over_catalog) {
        if (k.rmb && !pk.rmb) {
            bs.rmb_down_pos = Vec2(k.mx, k.my);
        }
        if (!k.rmb && pk.rmb) {
            float dist = (Vec2(k.mx, k.my) - bs.rmb_down_pos).length();
            if (dist < 0.05f) { // Only if mouse didn't move much (click, not drag)
                float best_d = 0.5f; int best_idx = -1;
                float current_ht_v2 = std::max(5.0f, bs.assembly.total_height);
                float b_view_dist = bs.cam_dist + current_ht_v2 * 0.5f;
                Vec3 b_camTarget(0, current_ht_v2 * 0.4f + bs.cam_pan_y, 0);
                float b_cosA = cosf(bs.orbit_angle), b_sinA = sinf(bs.orbit_angle);
                float b_cosP = cosf(bs.orbit_pitch), b_sinP = sinf(bs.orbit_pitch);
                Vec3 b_camRight(b_sinA, 0, -b_cosA); Vec3 b_camUp(-b_cosA * b_sinP, b_cosP, -b_sinA * b_sinP);

                for(int i=0; i<(int)bs.assembly.parts.size(); i++) {
                    const auto& p = bs.assembly.parts[i];
                    Vec3 relPos = p.pos - b_camTarget;
                    float px = relPos.dot(b_camRight) / (b_view_dist * 0.5f * 1.6f);
                    float py = relPos.dot(b_camUp) / (b_view_dist * 0.5f);
                    float d = sqrtf((px-k.mx)*(px-k.mx) + (py-k.my)*(py-k.my));
                    if (d < best_d) { best_d = d; best_idx = i; }
                }
                bs.context_menu_part_idx = best_idx;
            }
        }
    }
    if (k.lmb && !pk.lmb) {
        // Handle context menu clicks before closing
        if (bs.context_menu_part_idx != -1) {
            if (builderCheckHit(k.mx, k.my, 0.0f, -0.2f, 0.45f, 0.06f)) {
                bs.assembly.parts[bs.context_menu_part_idx].param0 = std::max(0.0f, bs.assembly.parts[bs.context_menu_part_idx].param0 - 0.1f);
                bs.stats_dirty = true;
            } else if (builderCheckHit(k.mx, k.my, 0.0f, -0.28f, 0.45f, 0.06f)) {
                bs.assembly.parts[bs.context_menu_part_idx].param0 = std::min(1.0f, bs.assembly.parts[bs.context_menu_part_idx].param0 + 0.1f);
                bs.stats_dirty = true;
            } else if (builderCheckHit(k.mx, k.my, 0.0f, -0.36f, 0.45f, 0.06f)) {
                bs.assembly.parts[bs.context_menu_part_idx].stage++;
                if (bs.assembly.parts[bs.context_menu_part_idx].stage > 4) bs.assembly.parts[bs.context_menu_part_idx].stage = 0;
                bs.stats_dirty = true;
            } else {
                bs.context_menu_part_idx = -1; 
            }
        }
    }

    // Set dirty if parts were removed (picked from assembly)
    if (dragging_id_at_start == -1 && bs.dragging_def_id != -1) {
        bs.stats_dirty = true;
    }

    // Symmetry & UI Buttons
    if (k.mx < -0.7f && k.my < -0.5f && k.lmb && !pk.lmb) {
        int syms[] = {1,2,4,6,8}; static int s_idx=0; s_idx=(s_idx+1)%5; bs.current_symmetry=syms[s_idx];
    }
    float rax=0.62f;
    if (builderCheckHit(k.mx,k.my, rax, 0.05f, 0.3f, 0.05f)&&k.lmb&&!pk.lmb) bs.centerViz.showCoM=!bs.centerViz.showCoM;
    if (builderCheckHit(k.mx,k.my, rax, -0.01f, 0.3f, 0.05f)&&k.lmb&&!pk.lmb) bs.centerViz.showCoL=!bs.centerViz.showCoL;
    if (builderCheckHit(k.mx,k.my, rax, -0.07f, 0.3f, 0.05f)&&k.lmb&&!pk.lmb) bs.centerViz.showCoT=!bs.centerViz.showCoT;

    // Staging Interaction
    if (k.mx > 0.5f && k.my > -0.7f && k.lmb && !pk.lmb) {
        float st_y = 0.78f;
        for (int i = 0; i < bs.cached_cfg.stages; i++) {
            if (k.my < st_y + 0.06f && k.my > st_y - 0.06f) {
                // Clicked on a stage. For INDUSTRIAL grade, let's just cycle the hovered part's stage
                if (bs.hovered_part_def_id != -1) {
                    // This is dummy for now, we should really pick part in assembly
                }
            }
            st_y -= 0.14f;
        }
    }
    if (k.space && !pk.space && !bs.assembly.parts.empty() && bs.assembly.hasEngine()) return true;
    return false;
}
