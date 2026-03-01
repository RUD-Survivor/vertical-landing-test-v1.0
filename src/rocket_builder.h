#pragma once
// ==========================================================
// rocket_builder.h — KSP-like Rocket Builder System
// Part catalog, assembly data, builder state & UI
// ==========================================================

#include "rocket_state.h"
#include "renderer3d.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>

// ==========================================================
// Part Categories
// ==========================================================
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
    "NOSECONES",
    "COMMAND",
    "FUEL TANKS",
    "ENGINES",
    "BOOSTERS",
    "STRUCTURAL"
};

// ==========================================================
// Part Definition — immutable blueprint
// ==========================================================
struct PartDef {
    int id;
    const char* name;
    PartCategory category;
    float dry_mass;         // kg
    float fuel_capacity;    // kg (0 for non-fuel parts)
    float isp;              // seconds (engines only)
    float thrust;           // Newtons (engines only)
    float consumption;      // kg/s fuel consumption (engines only)
    float height;           // meters
    float diameter;         // meters (visual)
    float drag_coeff;       // Cd contribution
    float r, g, b;          // render color
    const char* description;
};

// ==========================================================
// Part Catalog — all available parts
// ==========================================================
static const PartDef PART_CATALOG[] = {
    // ---- NOSECONES ----
    {0,  "Standard Fairing",   CAT_NOSE_CONE,    500,  0,     0,    0,   0,   8,  3.7f, 0.2f,  0.85f, 0.85f, 0.90f, "Basic aerodynamic fairing"},
    {1,  "Aero Nosecone",      CAT_NOSE_CONE,    350,  0,     0,    0,   0,   10, 3.7f, 0.15f, 0.70f, 0.70f, 0.75f, "Low-drag pointed nosecone"},
    {2,  "Payload Bay",        CAT_NOSE_CONE,    1200, 0,     0,    0,   0,   12, 4.0f, 0.3f,  0.90f, 0.90f, 0.85f, "Large payload enclosure"},

    // ---- COMMAND PODS ----
    {3,  "Mk1 Capsule",        CAT_COMMAND_POD,  1500, 0,     0,    0,   0,   4,  2.5f, 0.25f, 0.30f, 0.30f, 0.35f, "Single-crew capsule"},
    {4,  "Probe Core",         CAT_COMMAND_POD,  200,  0,     0,    0,   0,   1,  1.0f, 0.1f,  0.20f, 0.25f, 0.20f, "Unmanned guidance unit"},

    // ---- FUEL TANKS ----
    {5,  "Small Tank 50t",     CAT_FUEL_TANK,    3000, 50000, 0,    0,   0,   25, 3.7f, 0.3f,  0.92f, 0.92f, 0.92f, "Compact fuel storage"},
    {6,  "Medium Tank 100t",   CAT_FUEL_TANK,    5000, 100000,0,    0,   0,   35, 3.7f, 0.3f,  0.90f, 0.90f, 0.92f, "Standard fuel tank"},
    {7,  "Large Tank 200t",    CAT_FUEL_TANK,    8000, 200000,0,    0,   0,   50, 3.7f, 0.3f,  0.88f, 0.88f, 0.92f, "High-capacity fuel tank"},
    {8,  "Jumbo Tank 500t",    CAT_FUEL_TANK,    15000,500000,0,    0,   0,   80, 5.0f, 0.3f,  0.85f, 0.85f, 0.90f, "Massive fuel reservoir"},

    // ---- ENGINES ----
    {9,  "Raptor",             CAT_ENGINE,       2000, 0,     1500, 2200000, 100, 4,  3.7f, 0.05f, 0.35f, 0.35f, 0.38f, "High-ISP full-flow engine"},
    {10, "Merlin 1D",          CAT_ENGINE,       1500, 0,     1200, 845000,  80,  3,  3.0f, 0.05f, 0.40f, 0.35f, 0.30f, "Reliable workhorse engine"},
    {11, "Solid Rocket Motor", CAT_ENGINE,       3000, 30000, 250,  3500000, 200, 8,  3.7f, 0.1f,  0.50f, 0.40f, 0.30f, "Powerful solid fuel booster"},
    {12, "Ion Thruster",       CAT_ENGINE,       500,  0,     4200, 5000,    0.1, 2,  1.5f, 0.02f, 0.50f, 0.60f, 0.80f, "Ultra-efficient low-thrust"},

    // ---- BOOSTERS ----
    {13, "SRB-Small",          CAT_BOOSTER,      4000, 40000, 220,  2800000, 250, 15, 2.5f, 0.15f, 0.55f, 0.45f, 0.35f, "Strap-on solid booster"},
    {14, "SRB-Large",          CAT_BOOSTER,      8000, 100000,230,  6000000, 500, 25, 3.5f, 0.2f,  0.60f, 0.50f, 0.40f, "Heavy strap-on booster"},

    // ---- STRUCTURAL ----
    {15, "Decoupler",          CAT_STRUCTURAL,   300,  0,     0,    0,   0,   1,  3.7f, 0.0f,  0.25f, 0.25f, 0.25f, "Stage separation device"},
    {16, "Fin Set (x4)",       CAT_STRUCTURAL,   200,  0,     0,    0,   0,   2,  3.7f, -0.1f, 0.30f, 0.30f, 0.30f, "Aerodynamic stability fins"},
    {17, "Adapter 5m-3.7m",   CAT_STRUCTURAL,   500,  0,     0,    0,   0,   3,  5.0f, 0.05f, 0.40f, 0.40f, 0.42f, "Diameter transition piece"},
};

static const int PART_CATALOG_SIZE = sizeof(PART_CATALOG) / sizeof(PART_CATALOG[0]);

// ==========================================================
// Placed Part — one instance on the assembly stack
// ==========================================================
struct PlacedPart {
    int def_id;     // index into PART_CATALOG
    int stage;      // staging group (0 = last to fire, higher = fires first)
    float stack_y;  // computed Y position (meters from bottom)
};

// ==========================================================
// Rocket Assembly — the player's constructed rocket
// ==========================================================
struct RocketAssembly {
    std::vector<PlacedPart> parts; // bottom-to-top order

    // Computed aggregates
    float total_dry_mass = 0;
    float total_fuel = 0;
    float total_height = 0;
    float max_diameter = 0;
    float total_thrust = 0;
    float avg_isp = 0;
    float total_consumption = 0;
    float total_delta_v = 0;
    float twr = 0;
    float total_drag = 0;

    void recalculate() {
        total_dry_mass = 0;
        total_fuel = 0;
        total_height = 0;
        max_diameter = 0;
        total_thrust = 0;
        total_consumption = 0;
        total_drag = 0;
        float thrust_isp_sum = 0;

        float y = 0;
        for (size_t i = 0; i < parts.size(); i++) {
            const PartDef& def = PART_CATALOG[parts[i].def_id];
            parts[i].stack_y = y;
            y += def.height;

            total_dry_mass += def.dry_mass;
            total_fuel += def.fuel_capacity;
            total_height += def.height;
            if (def.diameter > max_diameter) max_diameter = def.diameter;
            total_thrust += def.thrust;
            total_consumption += def.consumption;
            total_drag += def.drag_coeff;
            if (def.thrust > 0) {
                thrust_isp_sum += def.thrust * def.isp;
            }
        }

        avg_isp = (total_thrust > 0) ? (thrust_isp_sum / total_thrust) : 0;

        float total_mass = total_dry_mass + total_fuel;
        if (total_dry_mass > 0 && avg_isp > 0) {
            total_delta_v = (float)(avg_isp * G0 * log((double)total_mass / (double)total_dry_mass));
        } else {
            total_delta_v = 0;
        }

        if (total_mass > 0) {
            twr = total_thrust / (total_mass * (float)G0);
        } else {
            twr = 0;
        }
    }

    void addPart(int def_id) {
        PlacedPart p;
        p.def_id = def_id;
        p.stage = 0;
        p.stack_y = 0;
        parts.push_back(p);
        recalculate();
    }

    void removeTop() {
        if (!parts.empty()) {
            parts.pop_back();
            recalculate();
        }
    }

    void removeAt(int index) {
        if (index >= 0 && index < (int)parts.size()) {
            parts.erase(parts.begin() + index);
            recalculate();
        }
    }

    void moveUp(int index) {
        if (index > 0 && index < (int)parts.size()) {
            std::swap(parts[index], parts[index - 1]);
            recalculate();
        }
    }

    void moveDown(int index) {
        if (index >= 0 && index < (int)parts.size() - 1) {
            std::swap(parts[index], parts[index + 1]);
            recalculate();
        }
    }

    RocketConfig buildRocketConfig() const {
        RocketConfig cfg;
        cfg.dry_mass = total_dry_mass;
        cfg.diameter = max_diameter;
        cfg.height = total_height;
        cfg.stages = 1; // simplified single-stage for physics
        cfg.specific_impulse = avg_isp;
        cfg.cosrate = total_consumption;
        cfg.nozzle_area = 0.5 * (max_diameter / 3.7); // scaled nozzle
        return cfg;
    }

    bool hasEngine() const {
        for (const auto& p : parts) {
            const PartDef& def = PART_CATALOG[p.def_id];
            if (def.thrust > 0) return true;
        }
        return false;
    }

    bool hasNosecone() const {
        if (parts.empty()) return false;
        const PartDef& top = PART_CATALOG[parts.back().def_id];
        return top.category == CAT_NOSE_CONE;
    }
};

// ==========================================================
// Builder State — UI state for the builder screen
// ==========================================================
struct BuilderState {
    RocketAssembly assembly;

    // UI navigation
    int selected_category = 0;       // which category tab is active
    int catalog_cursor = 0;          // cursor within current category's parts
    int assembly_cursor = -1;        // -1 = not in assembly editing mode, >=0 = selected part index
    bool in_assembly_mode = false;   // true = editing assembly, false = browsing catalog

    // Camera auto-orbit
    float orbit_angle = 0.0f;
    float orbit_speed = 0.3f;

    // Animation
    float launch_blink = 0.0f;

    // Get parts in the current category
    void getPartsInCategory(std::vector<int>& out) const {
        out.clear();
        for (int i = 0; i < PART_CATALOG_SIZE; i++) {
            if (PART_CATALOG[i].category == selected_category) {
                out.push_back(i);
            }
        }
    }
};

// ==========================================================
// Builder UI Drawing (using 2D Renderer overlay)
// ==========================================================
inline void drawBuilderText(Renderer* r, float x, float y, const char* text, float size,
                            float cr, float cg, float cb, float ca = 1.0f) {
    // Use the existing drawLabel for supported chars, drawNumber for digits
    // We extend with a simple uppercase block-font renderer
    float px = size * 0.22f;
    float cw = px * 4.5f;
    int idx = 0;
    while (text[idx]) {
        char c = text[idx];

        // Map character to a 3x5 bitmap. Each int = 15 bits (3 cols x 5 rows, MSB first)
        int gl = 0;
        switch (c) {
            // Letters
            case 'A': case 'a': gl = 0b010101111101101; break;
            case 'B': case 'b': gl = 0b110101110101110; break;
            case 'C': case 'c': gl = 0b011100100100011; break;
            case 'D': case 'd': gl = 0b110101101101110; break;
            case 'E': case 'e': gl = 0b111100110100111; break;
            case 'F': case 'f': gl = 0b111100110100100; break;
            case 'G': case 'g': gl = 0b011100101101011; break;
            case 'H': case 'h': gl = 0b101101111101101; break;
            case 'I': case 'i': gl = 0b111010010010111; break;
            case 'J': case 'j': gl = 0b001001001101010; break;
            case 'K': case 'k': gl = 0b101110100110101; break;
            case 'L': case 'l': gl = 0b100100100100111; break;
            case 'M': case 'm': gl = 0b101111101101101; break;
            case 'N': case 'n': gl = 0b101111111101101; break;
            case 'O': case 'o': gl = 0b010101101101010; break;
            case 'P': case 'p': gl = 0b110101110100100; break;
            case 'Q': case 'q': gl = 0b010101101011011; break;
            case 'R': case 'r': gl = 0b110101110101101; break;
            case 'S': case 's': gl = 0b011100010001110; break;
            case 'T': case 't': gl = 0b111010010010010; break;
            case 'U': case 'u': gl = 0b101101101101010; break;
            case 'V': case 'v': gl = 0b101101101010010; break;
            case 'W': case 'w': gl = 0b101101101111101; break;
            case 'X': case 'x': gl = 0b101101010101101; break;
            case 'Y': case 'y': gl = 0b101101010010010; break;
            case 'Z': case 'z': gl = 0b111001010100111; break;
            // Digits
            case '0': gl = 0b010101101101010; break;
            case '1': gl = 0b010110010010111; break;
            case '2': gl = 0b110001010100111; break;
            case '3': gl = 0b110001110001110; break;
            case '4': gl = 0b101101111001001; break;
            case '5': gl = 0b111100110001110; break;
            case '6': gl = 0b011100110101010; break;
            case '7': gl = 0b111001010010010; break;
            case '8': gl = 0b010101010101010; break;
            case '9': gl = 0b010101011001010; break;
            // Symbols
            case '/': gl = 0b001001010100100; break;
            case '.': gl = 0b000000000000010; break;
            case ':': gl = 0b000010000010000; break;
            case '-': gl = 0b000000111000000; break;
            case '(': gl = 0b010100100100010; break;
            case ')': gl = 0b010001001001010; break;
            case '<': gl = 0b001010100010001; break;
            case '>': gl = 0b100010001010100; break;
            case '[': gl = 0b110100100100110; break;
            case ']': gl = 0b011001001001011; break;
            case '%': gl = 0b101001010100101; break;
            case '+': gl = 0b000010111010000; break;
            case '=': gl = 0b000111000111000; break;
            case '!': gl = 0b010010010000010; break;
            case ' ': gl = 0; break;
            default:  gl = 0b111111111111111; break; // block for unknown
        }

        if (gl) {
            float ox = x + idx * cw;
            for (int row = 0; row < 5; row++) {
                for (int col = 0; col < 3; col++) {
                    if (gl & (1 << (14 - row * 3 - col))) {
                        r->addRect(ox + col * px, y + (2.0f - row) * px,
                                   px * 0.85f, px * 0.85f, cr, cg, cb, ca);
                    }
                }
            }
        }
        idx++;
    }
}

// Draw an integer value at position
inline void drawBuilderInt(Renderer* r, float x, float y, int value, float size,
                           float cr, float cg, float cb, float ca = 1.0f) {
    char buf[32];
    int len = 0;
    bool neg = value < 0;
    if (neg) value = -value;
    if (value == 0) {
        buf[len++] = '0';
    } else {
        while (value > 0 && len < 30) {
            buf[len++] = '0' + (value % 10);
            value /= 10;
        }
    }
    if (neg) buf[len++] = '-';
    // Reverse
    char str[32];
    for (int i = 0; i < len; i++) str[i] = buf[len - 1 - i];
    str[len] = 0;
    drawBuilderText(r, x, y, str, size, cr, cg, cb, ca);
}

// ==========================================================
// Main Builder UI Rendering
// ==========================================================
inline void drawBuilderUI_KSP(Renderer* r, BuilderState& bs, float time) {
    // ---- Background ----
    r->addRect(0.0f, 0.0f, 2.0f, 2.0f, 0.05f, 0.06f, 0.10f, 1.0f);

    // Subtle star field
    for (int i = 0; i < 120; i++) {
        float sx = hash11(i * 3917) * 2.0f - 1.0f;
        float sy = hash11(i * 7121) * 2.0f - 1.0f;
        float ss = 0.001f + hash11(i * 2131) * 0.003f;
        float sa = 0.3f + hash11(i * 991) * 0.5f;
        // Twinkle effect
        sa *= 0.7f + 0.3f * sinf(time * (1.0f + hash11(i * 443) * 2.0f) + hash11(i * 661) * 6.28f);
        r->addRect(sx, sy, ss, ss, 0.8f, 0.85f, 1.0f, sa);
    }

    // ---- Title Bar ----
    r->addRect(0.0f, 0.92f, 2.0f, 0.12f, 0.08f, 0.10f, 0.18f, 0.85f);
    drawBuilderText(r, -0.42f, 0.925f, "ROCKET ASSEMBLY", 0.030f, 0.3f, 0.85f, 1.0f);

    // ---- Left Panel: Part Catalog ----
    float panel_left = -0.98f;
    float panel_width = 0.55f;
    float panel_right = panel_left + panel_width;

    // Panel background
    r->addRect(panel_left + panel_width / 2.0f, 0.15f, panel_width, 1.40f,
               0.07f, 0.08f, 0.13f, 0.80f);

    // Category tabs
    float tab_y = 0.82f;
    float tab_w = panel_width / CAT_COUNT;
    for (int c = 0; c < CAT_COUNT; c++) {
        float tx = panel_left + tab_w * c + tab_w / 2.0f;
        bool active = (c == bs.selected_category);
        float br = active ? 0.15f : 0.08f;
        float bg = active ? 0.35f : 0.12f;
        float bb = active ? 0.20f : 0.15f;
        float ba = active ? 0.9f : 0.5f;
        r->addRect(tx, tab_y, tab_w - 0.005f, 0.06f, br, bg, bb, ba);

        // Category abbreviation (first 3 chars)
        char abbr[4] = {0};
        for (int j = 0; j < 3 && CATEGORY_NAMES[c][j]; j++) abbr[j] = CATEGORY_NAMES[c][j];
        float tr = active ? 0.5f : 0.35f;
        float tg = active ? 1.0f : 0.45f;
        float tb = active ? 0.6f : 0.45f;
        drawBuilderText(r, tx - 0.03f, tab_y, abbr, 0.012f, tr, tg, tb);
    }

    // Part list in current category
    std::vector<int> cat_parts;
    bs.getPartsInCategory(cat_parts);
    float list_y_start = 0.72f;
    float item_h = 0.10f;

    for (int i = 0; i < (int)cat_parts.size(); i++) {
        const PartDef& def = PART_CATALOG[cat_parts[i]];
        float iy = list_y_start - i * item_h;
        bool is_selected = (!bs.in_assembly_mode && bs.catalog_cursor == i);

        // Item background
        if (is_selected) {
            r->addRect(panel_left + panel_width / 2.0f, iy, panel_width - 0.02f, item_h - 0.01f,
                       0.10f, 0.30f, 0.15f, 0.65f);
            // Selection indicator
            r->addRect(panel_left + 0.015f, iy, 0.008f, item_h * 0.6f, 0.2f, 1.0f, 0.4f, 0.9f);
        }

        // Part color swatch
        r->addRect(panel_left + 0.04f, iy + 0.01f, 0.02f, 0.035f, def.r, def.g, def.b, 0.9f);

        // Part name
        float nr = is_selected ? 0.9f : 0.55f;
        float ng = is_selected ? 0.95f : 0.55f;
        float nb = is_selected ? 0.9f : 0.55f;
        drawBuilderText(r, panel_left + 0.07f, iy + 0.018f, def.name, 0.010f, nr, ng, nb);

        // Part stats line
        if (def.fuel_capacity > 0) {
            drawBuilderInt(r, panel_left + 0.07f, iy - 0.015f, (int)(def.fuel_capacity / 1000), 0.008f, 0.9f, 0.7f, 0.2f);
            drawBuilderText(r, panel_left + 0.20f, iy - 0.015f, "t fuel", 0.008f, 0.6f, 0.5f, 0.2f);
        } else if (def.thrust > 0) {
            drawBuilderInt(r, panel_left + 0.07f, iy - 0.015f, (int)(def.thrust / 1000), 0.008f, 1.0f, 0.5f, 0.2f);
            drawBuilderText(r, panel_left + 0.20f, iy - 0.015f, "kn", 0.008f, 0.7f, 0.4f, 0.2f);
            drawBuilderInt(r, panel_left + 0.30f, iy - 0.015f, (int)def.isp, 0.008f, 0.5f, 0.8f, 1.0f);
            drawBuilderText(r, panel_left + 0.40f, iy - 0.015f, "s isp", 0.008f, 0.4f, 0.5f, 0.7f);
        } else {
            drawBuilderInt(r, panel_left + 0.07f, iy - 0.015f, (int)def.dry_mass, 0.008f, 0.6f, 0.6f, 0.6f);
            drawBuilderText(r, panel_left + 0.20f, iy - 0.015f, "kg", 0.008f, 0.4f, 0.4f, 0.4f);
        }
    }

    // ---- Right Panel: Assembly Stack ----
    float asm_panel_x = 0.62f;
    float asm_panel_w = 0.35f;

    // Panel background
    r->addRect(asm_panel_x, 0.15f, asm_panel_w, 1.40f,
               0.07f, 0.08f, 0.13f, 0.80f);

    // Header
    drawBuilderText(r, asm_panel_x - 0.13f, 0.82f, "ASSEMBLY", 0.012f,
                    bs.in_assembly_mode ? 0.3f : 0.5f,
                    bs.in_assembly_mode ? 1.0f : 0.5f,
                    bs.in_assembly_mode ? 0.5f : 0.5f);

    if (bs.assembly.parts.empty()) {
        drawBuilderText(r, asm_panel_x - 0.15f, 0.45f, "ADD PARTS", 0.012f, 0.3f, 0.3f, 0.35f);
        drawBuilderText(r, asm_panel_x - 0.15f, 0.35f, "ENTER=ADD", 0.010f, 0.2f, 0.5f, 0.3f);
    } else {
        // List assembly parts (bottom-to-top, but display top-to-bottom)
        float asm_list_y = 0.72f;
        float asm_item_h = 0.065f;
        for (int i = (int)bs.assembly.parts.size() - 1; i >= 0; i--) {
            int display_idx = (int)bs.assembly.parts.size() - 1 - i;
            float iy = asm_list_y - display_idx * asm_item_h;
            const PlacedPart& pp = bs.assembly.parts[i];
            const PartDef& def = PART_CATALOG[pp.def_id];
            bool is_asm_sel = (bs.in_assembly_mode && bs.assembly_cursor == i);

            if (is_asm_sel) {
                r->addRect(asm_panel_x, iy, asm_panel_w - 0.02f, asm_item_h - 0.005f,
                           0.15f, 0.25f, 0.10f, 0.6f);
                r->addRect(asm_panel_x - asm_panel_w / 2.0f + 0.015f, iy, 0.006f, asm_item_h * 0.6f,
                           1.0f, 0.8f, 0.2f, 0.9f);
            }

            // Part color + name
            r->addRect(asm_panel_x - asm_panel_w / 2.0f + 0.03f, iy, 0.015f, 0.025f,
                       def.r, def.g, def.b, 0.85f);

            float nr = is_asm_sel ? 0.95f : 0.5f;
            float ng = is_asm_sel ? 0.95f : 0.5f;
            float nb = is_asm_sel ? 0.90f : 0.5f;
            // Truncate name to ~12 chars
            char short_name[16] = {0};
            for (int j = 0; j < 14 && def.name[j]; j++) short_name[j] = def.name[j];
            drawBuilderText(r, asm_panel_x - asm_panel_w / 2.0f + 0.06f, iy, short_name, 0.008f, nr, ng, nb);
        }
    }

    // ---- Center: 3D Rocket Preview (drawn as 2D schematic) ----
    float preview_cx = 0.08f;
    float preview_cy = 0.20f;
    float preview_scale = 0.005f; // meters to screen units

    if (!bs.assembly.parts.empty()) {
        float total_h_screen = bs.assembly.total_height * preview_scale;
        // Auto-scale if rocket is too tall
        if (total_h_screen > 1.2f) {
            preview_scale = 1.2f / bs.assembly.total_height;
        }

        float base_y = preview_cy - bs.assembly.total_height * preview_scale / 2.0f;

        for (int i = 0; i < (int)bs.assembly.parts.size(); i++) {
            const PlacedPart& pp = bs.assembly.parts[i];
            const PartDef& def = PART_CATALOG[pp.def_id];
            float pw = def.diameter * preview_scale * 0.5f;
            float ph = def.height * preview_scale;
            float py = base_y + pp.stack_y * preview_scale + ph / 2.0f;

            bool highlight = (bs.in_assembly_mode && bs.assembly_cursor == i);
            float hr = highlight ? fminf(def.r + 0.2f, 1.0f) : def.r;
            float hg = highlight ? fminf(def.g + 0.2f, 1.0f) : def.g;
            float hb = highlight ? fminf(def.b + 0.2f, 1.0f) : def.b;

            if (def.category == CAT_NOSE_CONE) {
                // Triangle (cone) shape
                r->addRotatedTri(preview_cx, py + ph * 0.15f, pw * 0.9f, ph * 0.7f, 0.0f, hr, hg, hb);
                r->addRect(preview_cx, py - ph * 0.2f, pw * 0.95f, ph * 0.3f, hr * 0.95f, hg * 0.95f, hb * 0.95f);
            } else if (def.category == CAT_ENGINE) {
                // Bell-shaped nozzle (inverted triangle + rectangle)
                r->addRect(preview_cx, py + ph * 0.15f, pw * 0.6f, ph * 0.4f, hr * 0.8f, hg * 0.8f, hb * 0.8f);
                r->addRotatedTri(preview_cx, py - ph * 0.2f, pw * 1.2f, ph * 0.5f, 3.14159f, hr, hg, hb);
            } else if (def.category == CAT_STRUCTURAL && def.drag_coeff < 0) {
                // Fins: draw as small triangles on sides
                r->addRect(preview_cx, py, pw * 0.3f, ph, hr, hg, hb);
                float fin_w = pw * 0.8f;
                r->addRotatedTri(preview_cx - pw * 0.5f, py, fin_w, ph * 0.8f, -1.57f, hr * 0.9f, hg * 0.9f, hb * 0.9f);
                r->addRotatedTri(preview_cx + pw * 0.5f, py, fin_w, ph * 0.8f, 1.57f, hr * 0.9f, hg * 0.9f, hb * 0.9f);
            } else if (def.category == CAT_BOOSTER) {
                // Boosters drawn as side-mounted cylinders
                r->addRect(preview_cx, py, pw * 0.7f, ph, hr, hg, hb);
                // Side boosters
                r->addRect(preview_cx - pw * 0.6f, py, pw * 0.3f, ph * 0.8f, hr * 0.8f, hg * 0.8f, hb * 0.8f);
                r->addRect(preview_cx + pw * 0.6f, py, pw * 0.3f, ph * 0.8f, hr * 0.8f, hg * 0.8f, hb * 0.8f);
            } else {
                // Default: rectangle (fuel tanks, adapters, decouplers, pods)
                r->addRect(preview_cx, py, pw, ph, hr, hg, hb);

                // Inter-stage rings for fuel tanks
                if (def.category == CAT_FUEL_TANK) {
                    r->addRect(preview_cx, py + ph / 2.0f, pw * 1.03f, 0.004f, 0.2f, 0.2f, 0.2f);
                    r->addRect(preview_cx, py - ph / 2.0f, pw * 1.03f, 0.004f, 0.2f, 0.2f, 0.2f);
                }
                if (def.category == CAT_STRUCTURAL && def.drag_coeff >= 0) {
                    // Decoupler: striped ring
                    r->addRect(preview_cx, py, pw * 1.05f, ph, 0.3f, 0.3f, 0.1f, 0.7f);
                }
            }

            // Highlight border for selected part
            if (highlight) {
                float blink = 0.6f + 0.4f * sinf(time * 4.0f);
                r->addRect(preview_cx, py + ph / 2.0f + 0.003f, pw * 1.1f, 0.003f, 0.2f, 1.0f, 0.4f, blink);
                r->addRect(preview_cx, py - ph / 2.0f - 0.003f, pw * 1.1f, 0.003f, 0.2f, 1.0f, 0.4f, blink);
            }
        }
    }

    // ---- Bottom Stats Panel ----
    r->addRect(0.0f, -0.78f, 2.0f, 0.30f, 0.06f, 0.07f, 0.12f, 0.85f);

    // Separator line
    r->addRect(0.0f, -0.64f, 1.8f, 0.002f, 0.2f, 0.4f, 0.6f, 0.5f);

    float stat_y = -0.72f;
    float stat_y2 = -0.82f;

    // Mass
    drawBuilderText(r, -0.88f, stat_y, "MASS", 0.009f, 0.5f, 0.5f, 0.5f);
    float total_mass = bs.assembly.total_dry_mass + bs.assembly.total_fuel;
    drawBuilderInt(r, -0.88f, stat_y2, (int)(total_mass / 1000), 0.018f, 0.9f, 0.9f, 0.9f);
    drawBuilderText(r, -0.70f, stat_y2, "t", 0.012f, 0.5f, 0.5f, 0.5f);

    // Fuel
    drawBuilderText(r, -0.50f, stat_y, "FUEL", 0.009f, 0.5f, 0.5f, 0.5f);
    drawBuilderInt(r, -0.50f, stat_y2, (int)(bs.assembly.total_fuel / 1000), 0.018f, 0.9f, 0.7f, 0.2f);
    drawBuilderText(r, -0.32f, stat_y2, "t", 0.012f, 0.6f, 0.5f, 0.2f);

    // Delta-V
    drawBuilderText(r, -0.12f, stat_y, "DELTA V", 0.009f, 0.5f, 0.5f, 0.5f);
    drawBuilderInt(r, -0.12f, stat_y2, (int)bs.assembly.total_delta_v, 0.018f, 0.3f, 0.9f, 0.4f);
    drawBuilderText(r, 0.12f, stat_y2, "m/s", 0.012f, 0.2f, 0.6f, 0.3f);

    // TWR
    drawBuilderText(r, 0.32f, stat_y, "TWR", 0.009f, 0.5f, 0.5f, 0.5f);
    int twr_int = (int)(bs.assembly.twr * 100);
    drawBuilderInt(r, 0.32f, stat_y2, twr_int, 0.018f,
                   bs.assembly.twr >= 1.0f ? 0.3f : 1.0f,
                   bs.assembly.twr >= 1.0f ? 0.9f : 0.3f,
                   bs.assembly.twr >= 1.0f ? 0.4f : 0.3f);
    drawBuilderText(r, 0.48f, stat_y2, "/100", 0.009f, 0.4f, 0.4f, 0.4f);

    // Height
    drawBuilderText(r, 0.65f, stat_y, "HEIGHT", 0.009f, 0.5f, 0.5f, 0.5f);
    drawBuilderInt(r, 0.65f, stat_y2, (int)bs.assembly.total_height, 0.018f, 0.6f, 0.7f, 0.9f);
    drawBuilderText(r, 0.82f, stat_y2, "m", 0.012f, 0.4f, 0.5f, 0.6f);

    // ---- Launch Button / Prompt ----
    bool can_launch = bs.assembly.hasEngine() && !bs.assembly.parts.empty();
    if (can_launch) {
        float blink = 0.5f + 0.5f * sinf(time * 3.0f);
        r->addRect(0.0f, -0.93f, 0.60f, 0.08f,
                   0.05f * blink, 0.25f * blink, 0.05f * blink, 0.8f);
        drawBuilderText(r, -0.18f, -0.93f, "[SPACE] LAUNCH", 0.013f,
                        0.3f, 1.0f * blink, 0.4f);
    } else {
        drawBuilderText(r, -0.22f, -0.93f, "ADD ENGINE TO LAUNCH", 0.010f, 0.5f, 0.3f, 0.3f);
    }

    // ---- Controls Help ----
    float help_y = 0.82f;
    float help_x = 0.05f;
    drawBuilderText(r, help_x - 0.07f, help_y, "TAB=MODE", 0.007f, 0.3f, 0.3f, 0.35f);

    // Mode indicator
    if (bs.in_assembly_mode) {
        drawBuilderText(r, help_x - 0.07f, help_y - 0.03f, "DEL=REMOVE", 0.007f, 0.3f, 0.3f, 0.35f);
        drawBuilderText(r, help_x - 0.07f, help_y - 0.05f, "PGUP/DN=MOVE", 0.007f, 0.3f, 0.3f, 0.35f);
    } else {
        drawBuilderText(r, help_x - 0.07f, help_y - 0.03f, "ENTER=ADD", 0.007f, 0.3f, 0.3f, 0.35f);
        drawBuilderText(r, help_x - 0.07f, help_y - 0.05f, "<>=CATEGORY", 0.007f, 0.3f, 0.3f, 0.35f);
    }
}

// ==========================================================
// Builder Input Handling
// ==========================================================
struct BuilderKeyState {
    bool up, down, left, right, enter, del, tab, pgup, pgdn, space;
};

inline bool builderHandleInput(BuilderState& bs, const BuilderKeyState& keys,
                               const BuilderKeyState& prev_keys) {
    // TAB: toggle catalog/assembly mode
    if (keys.tab && !prev_keys.tab) {
        bs.in_assembly_mode = !bs.in_assembly_mode;
        if (bs.in_assembly_mode) {
            bs.assembly_cursor = bs.assembly.parts.empty() ? -1 : (int)bs.assembly.parts.size() - 1;
        }
    }

    if (bs.in_assembly_mode) {
        // Assembly editing mode
        if (!bs.assembly.parts.empty()) {
            if (keys.up && !prev_keys.up) {
                bs.assembly_cursor = std::min(bs.assembly_cursor + 1, (int)bs.assembly.parts.size() - 1);
            }
            if (keys.down && !prev_keys.down) {
                bs.assembly_cursor = std::max(bs.assembly_cursor - 1, 0);
            }
            if (keys.del && !prev_keys.del) {
                bs.assembly.removeAt(bs.assembly_cursor);
                if (bs.assembly_cursor >= (int)bs.assembly.parts.size()) {
                    bs.assembly_cursor = (int)bs.assembly.parts.size() - 1;
                }
            }
            if (keys.pgup && !prev_keys.pgup) {
                bs.assembly.moveUp(bs.assembly_cursor);
                if (bs.assembly_cursor > 0) bs.assembly_cursor--;
            }
            if (keys.pgdn && !prev_keys.pgdn) {
                bs.assembly.moveDown(bs.assembly_cursor);
                if (bs.assembly_cursor < (int)bs.assembly.parts.size() - 1) bs.assembly_cursor++;
            }
        }
    } else {
        // Catalog browsing mode
        std::vector<int> cat_parts;
        bs.getPartsInCategory(cat_parts);

        if (keys.up && !prev_keys.up) {
            bs.catalog_cursor = std::max(bs.catalog_cursor - 1, 0);
        }
        if (keys.down && !prev_keys.down) {
            bs.catalog_cursor = std::min(bs.catalog_cursor + 1, (int)cat_parts.size() - 1);
        }
        if (keys.left && !prev_keys.left) {
            bs.selected_category = (bs.selected_category - 1 + CAT_COUNT) % CAT_COUNT;
            bs.catalog_cursor = 0;
        }
        if (keys.right && !prev_keys.right) {
            bs.selected_category = (bs.selected_category + 1) % CAT_COUNT;
            bs.catalog_cursor = 0;
        }
        if (keys.enter && !prev_keys.enter) {
            if (bs.catalog_cursor >= 0 && bs.catalog_cursor < (int)cat_parts.size()) {
                bs.assembly.addPart(cat_parts[bs.catalog_cursor]);
            }
        }
    }

    // SPACE = launch (return true)
    if (keys.space && !prev_keys.space) {
        if (bs.assembly.hasEngine() && !bs.assembly.parts.empty()) {
            return true; // signal launch
        }
    }

    return false; // not launching
}


