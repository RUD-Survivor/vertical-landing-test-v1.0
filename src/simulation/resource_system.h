#pragma once
// ==========================================================
// resource_system.h — 资源与背包系统 (Phase 1)
// ==========================================================

#include "core/agency_state.h"
#include <vector>
#include <string>
#include <cstring>

// ==========================================
// Item Metadata: name, description, icon color
// ==========================================
struct ItemInfo {
    const char* name;
    const char* category;     // "RAW", "PROCESSED", "PART"
    float icon_r, icon_g, icon_b; // display color for UI
    int stack_max;            // max stack size per slot
};

inline const ItemInfo& GetItemInfo(ItemType type) {
    static const ItemInfo INFO[] = {
        // RAW MATERIALS
        /* ITEM_IRON_ORE    */ {"IRON ORE",       "RAW",       0.6f, 0.4f, 0.3f, 999},
        /* ITEM_COPPER_ORE  */ {"COPPER ORE",     "RAW",       0.8f, 0.5f, 0.2f, 999},
        /* ITEM_COAL        */ {"COAL",            "RAW",       0.2f, 0.2f, 0.2f, 999},
        /* ITEM_SILICON     */ {"SILICON",         "RAW",       0.5f, 0.5f, 0.6f, 999},
        /* ITEM_TITANIUM_ORE*/ {"TITANIUM ORE",   "RAW",       0.7f, 0.7f, 0.8f, 999},
        // PROCESSED
        /* ITEM_STEEL       */ {"STEEL",           "PROCESSED", 0.7f, 0.7f, 0.7f, 999},
        /* ITEM_COPPER_WIRE */ {"COPPER WIRE",     "PROCESSED", 0.9f, 0.6f, 0.2f, 999},
        /* ITEM_ELECTRONICS */ {"ELECTRONICS",     "PROCESSED", 0.3f, 0.9f, 0.4f, 500},
        /* ITEM_TITANIUM    */ {"TITANIUM ALLOY",  "PROCESSED", 0.8f, 0.8f, 0.9f, 500},
        /* ITEM_FUEL        */ {"ROCKET FUEL",     "PROCESSED", 1.0f, 0.4f, 0.1f, 999},
        // ROCKET PARTS
        /* PART_ENGINE      */ {"ROCKET ENGINE",   "PART",      1.0f, 0.3f, 0.1f, 50},
        /* PART_FUEL_TANK   */ {"FUEL TANK",       "PART",      0.9f, 0.9f, 0.9f, 50},
        /* PART_COMMAND_POD */ {"COMMAND POD",     "PART",      0.2f, 0.6f, 1.0f, 10},
        /* PART_STRUCTURAL  */ {"STRUCTURAL",      "PART",      0.5f, 0.5f, 0.5f, 99},
        /* PART_NOSECONE    */ {"NOSECONE",        "PART",      0.8f, 0.8f, 0.8f, 50},
    };
    int idx = (int)type;
    if (idx < 0 || idx >= (int)(sizeof(INFO)/sizeof(INFO[0]))) {
        static const ItemInfo unknown = {"UNKNOWN", "???", 1,1,1, 1};
        return unknown;
    }
    return INFO[idx];
}

// ==========================================
// Market Pricing: defines sale value per unit
// ==========================================
inline double GetItemPrice(ItemType type) {
    switch (type) {
        // RAW MATERIALS
        case ITEM_IRON_ORE:     return 100.0;
        case ITEM_COPPER_ORE:   return 120.0;
        case ITEM_COAL:         return 80.0;
        case ITEM_SILICON:      return 150.0;
        case ITEM_TITANIUM_ORE: return 250.0;
        
        // PROCESSED
        case ITEM_STEEL:        return 400.0;
        case ITEM_COPPER_WIRE:  return 100.0;
        case ITEM_ELECTRONICS:  return 500.0;
        case ITEM_TITANIUM:     return 1000.0;
        case ITEM_FUEL:         return 50.0;
        
        // ROCKET PARTS
        case PART_STRUCTURAL:   return 1500.0;
        case PART_NOSECONE:     return 2500.0;
        case PART_FUEL_TANK:    return 6000.0;
        case PART_ENGINE:       return 8000.0;
        case PART_COMMAND_POD:  return 10000.0;
        
        default: return 0.0;
    }
}

// ==========================================
// Recipe: defines input -> output conversion
// ==========================================
struct RecipeSlot {
    ItemType item;
    int count;
};

struct Recipe {
    const char* name;
    RecipeSlot inputs[4];   // up to 4 input types (unused slots have count=0)
    int input_count;
    RecipeSlot output;
    float craft_time;       // seconds to produce one batch
};

// All available recipes
inline const Recipe* GetRecipes(int& count) {
    static const Recipe RECIPES[] = {
        // Smelting
        {"SMELT STEEL",
            {{ITEM_IRON_ORE, 2}, {ITEM_COAL, 1}}, 2,
            {ITEM_STEEL, 1}, 5.0f},
        {"DRAW WIRE",
            {{ITEM_COPPER_ORE, 1}}, 1,
            {ITEM_COPPER_WIRE, 2}, 3.0f},
        {"REFINE TITANIUM",
            {{ITEM_TITANIUM_ORE, 3}}, 1,
            {ITEM_TITANIUM, 1}, 8.0f},
        // Assembly
        {"MAKE ELECTRONICS",
            {{ITEM_COPPER_WIRE, 2}, {ITEM_SILICON, 1}}, 2,
            {ITEM_ELECTRONICS, 1}, 6.0f},
        {"REFINE FUEL",
            {{ITEM_COAL, 2}}, 1,
            {ITEM_FUEL, 5}, 4.0f},
        // Rocket Parts
        {"BUILD ENGINE",
            {{ITEM_STEEL, 5}, {ITEM_TITANIUM, 3}, {ITEM_ELECTRONICS, 2}}, 3,
            {PART_ENGINE, 1}, 30.0f},
        {"BUILD FUEL TANK",
            {{ITEM_STEEL, 8}, {ITEM_TITANIUM, 2}}, 2,
            {PART_FUEL_TANK, 1}, 20.0f},
        {"BUILD COMMAND POD",
            {{ITEM_TITANIUM, 4}, {ITEM_ELECTRONICS, 5}, {ITEM_STEEL, 3}}, 3,
            {PART_COMMAND_POD, 1}, 45.0f},
        {"BUILD STRUCTURAL",
            {{ITEM_STEEL, 3}}, 1,
            {PART_STRUCTURAL, 1}, 10.0f},
        {"BUILD NOSECONE",
            {{ITEM_STEEL, 2}, {ITEM_TITANIUM, 1}}, 2,
            {PART_NOSECONE, 1}, 12.0f},
    };
    count = (int)(sizeof(RECIPES) / sizeof(RECIPES[0]));
    return RECIPES;
}

// ==========================================
// Inventory: container that holds items
// ==========================================
struct Inventory {
    std::map<ItemType, int> items;

    int get(ItemType t) const {
        auto it = items.find(t);
        return (it != items.end()) ? it->second : 0;
    }

    void add(ItemType t, int amount) {
        items[t] += amount;
    }

    bool remove(ItemType t, int amount) {
        auto it = items.find(t);
        if (it == items.end() || it->second < amount) return false;
        it->second -= amount;
        if (it->second <= 0) items.erase(it);
        return true;
    }

    bool hasEnough(ItemType t, int amount) const {
        return get(t) >= amount;
    }

    // Check if all inputs for a recipe are available
    bool canCraft(const Recipe& recipe) const {
        for (int i = 0; i < recipe.input_count; i++) {
            if (!hasEnough(recipe.inputs[i].item, recipe.inputs[i].count))
                return false;
        }
        return true;
    }

    // Consume inputs and produce output
    bool craft(const Recipe& recipe) {
        if (!canCraft(recipe)) return false;
        for (int i = 0; i < recipe.input_count; i++) {
            remove(recipe.inputs[i].item, recipe.inputs[i].count);
        }
        add(recipe.output.item, recipe.output.count);
        return true;
    }

    // Get total number of distinct item types
    int distinctTypes() const { return (int)items.size(); }

    // Check if empty
    bool empty() const { return items.empty(); }
};
