#pragma once
// ==========================================================
// agency_state.h — 航天局全局状态 & 物品类型定义
// ==========================================================

#include <string>
#include <map>
#include <vector>

// ==========================================
// Item Types: Raw -> Processed -> Rocket Parts
// ==========================================
enum ItemType {
    ITEM_NONE = -1,
    // Raw Materials (mined)
    ITEM_IRON_ORE = 0,
    ITEM_COPPER_ORE,
    ITEM_COAL,
    ITEM_SILICON,
    ITEM_TITANIUM_ORE,
    // Processed Materials (smelted/refined)
    ITEM_STEEL,
    ITEM_COPPER_WIRE,
    ITEM_ELECTRONICS,
    ITEM_TITANIUM,
    ITEM_FUEL,
    // Rocket Parts (assembled)
    PART_ENGINE,
    PART_FUEL_TANK,
    PART_COMMAND_POD,
    PART_STRUCTURAL,
    PART_NOSECONE,
    // Sentinel
    ITEM_TYPE_COUNT
};

// Forward declaration of Inventory from resource_system.h
// Since resource_system.h depends on agency_state.h, we use a simple struct for now
// or just keep it as a map but with helpers. 
// Actually, let's keep it as is but fix the property name and helpers to match.

// ==========================================
// Agency State: global player progression
// ==========================================
struct AgencyState {
    double funds = 500000.0;
    double reputation = 50.0;
    double science = 0.0;
    
    // Using a simple map but with fixed helper names for consistency
    std::map<ItemType, int> inventory;
    
    // Time tracking
    double global_time = 0.0;

    // Helper: get item count
    int getItemCount(ItemType t) const {
        if (t == ITEM_NONE) return 0;
        auto it = inventory.find(t);
        return (it != inventory.end()) ? it->second : 0;
    }
    
    void addItem(ItemType t, int n) { 
        if (t == ITEM_NONE) return;
        inventory[t] += n; 
    }
    
    bool removeItem(ItemType t, int n) {
        if (t == ITEM_NONE) return true;
        auto it = inventory.find(t);
        if (it == inventory.end() || it->second < n) return false;
        it->second -= n;
        if (it->second <= 0) inventory.erase(it);
        return true;
    }
};
