#pragma once
// ==========================================================
// factory_system.h — 工厂建筑系统 (Phase 3: Logistics)
// ==========================================================

#include "core/agency_state.h"
#include "simulation/resource_system.h"
#include <vector>
#include <cmath>
#include <map>

// ==========================================
// Factory Node Types
// ==========================================
enum FactoryNodeType {
    NODE_MINER = 0,
    NODE_SMELTER,
    NODE_ASSEMBLER,
    NODE_STORAGE,
    NODE_POWER_PLANT,
    NODE_TYPE_COUNT
};

inline const char* NodeTypeName(FactoryNodeType t) {
    static const char* names[] = {"MINER", "SMELTER", "ASSEMBLER", "STORAGE", "POWER PLANT"};
    return names[(int)t];
}

// ==========================================
// FactoryNode: base building in the pipeline
// ==========================================
struct FactoryNode {
    FactoryNodeType type;
    int id;                     // unique ID
    int grid_x = 0, grid_y = 0; // Position

    // Internal inventory
    Inventory input_buffer;
    Inventory output_buffer;

    // Production state
    float progress = 0.0f;      
    bool is_producing = false;
    
    // Config
    ItemType mine_output = ITEM_IRON_ORE;
    int recipe_index = -1;
    
    // Limits
    int output_max = 50;
};

// ==========================================
// Item in transit on a belt
// ==========================================
struct BeltItem {
    ItemType type;
    float progress = 0.0f; // 0.0 (start) to 1.0 (end)
};

// ==========================================
// Conveyor Belt: connects two nodes
// ==========================================
struct ConveyorBelt {
    int from_node_id;
    int to_node_id;
    float transfer_timer = 0.0f; // delay between item injections
    std::vector<BeltItem> items; // items currently on the belt
};

// ==========================================
// FactorySystem: manages all factory nodes
// ==========================================
class FactorySystem {
public:
    std::vector<FactoryNode> nodes;
    std::vector<ConveyorBelt> belts;
    int next_id = 1;
    
    // Global Power Stats
    float total_power_gen = 0.0f;
    float total_power_req = 0.0f;
    float power_efficiency = 1.0f;

    int addNode(FactoryNodeType type, int gx, int gy) {
        FactoryNode node;
        node.id = next_id++;
        node.type = type;
        node.grid_x = gx;
        node.grid_y = gy;
        
        // Defaults
        if (type == NODE_MINER) {
            ItemType raws[] = {ITEM_IRON_ORE, ITEM_COPPER_ORE, ITEM_COAL, ITEM_SILICON, ITEM_TITANIUM_ORE};
            node.mine_output = raws[gx % 5];
            node.output_max = 100;
        } else if (type == NODE_SMELTER) {
            node.recipe_index = 0; // Smelt Steel
            node.output_max = 50;
        } else if (type == NODE_ASSEMBLER) {
            node.recipe_index = 5; // Build Engine
            node.output_max = 50;
        } else if (type == NODE_STORAGE) {
            node.output_max = 500;
        } else if (type == NODE_POWER_PLANT) {
            node.output_max = 10; // Not really used for output
        }

        nodes.push_back(node);
        return node.id;
    }

    void removeNode(int id) {
        for (auto it = nodes.begin(); it != nodes.end(); ++it) {
            if (it->id == id) {
                nodes.erase(it);
                break;
            }
        }
    }

    FactoryNode* findNode(int id) {
        for (auto& n : nodes) if (n.id == id) return &n;
        return nullptr;
    }

    void tick(float dt, AgencyState& agency) {
        int rcount = 0;
        const Recipe* recipes = GetRecipes(rcount);

        // 0. Global Power Calculation
        total_power_gen = 0.0f;
        total_power_req = 0.0f;
        for (auto& node : nodes) {
            if (node.type == NODE_POWER_PLANT) {
                // Power plant: if it has coal in buffer, it produces power
                // For simplicity, let's assume it always has some base production, 
                // but significantly more with coal.
                float base_gen = 10.0f;
                if (node.input_buffer.get(ITEM_COAL) > 0) {
                    node.progress += dt * 0.2f; // consume 1 coal every 5s
                    if (node.progress >= 1.0f) {
                        if (node.input_buffer.remove(ITEM_COAL, 1)) {
                            node.progress -= 1.0f;
                        } else {
                            node.progress = 0.0f;
                        }
                    }
                    total_power_gen += 100.0f;
                } else {
                    total_power_gen += base_gen;
                    node.progress = 0.0f;
                }
            } else if (node.type == NODE_MINER) {
                total_power_req += 20.0f;
            } else if (node.type == NODE_SMELTER) {
                total_power_req += 40.0f;
            } else if (node.type == NODE_ASSEMBLER) {
                total_power_req += 60.0f;
            }
        }
        power_efficiency = (total_power_req > 0) ? fminf(1.0f, total_power_gen / total_power_req) : 1.0f;

        // 1. Production Logic
        for (auto& node : nodes) {
            if (node.type == NODE_MINER) {
                node.progress += dt * 0.5f * power_efficiency; // 1 item every 2s
                if (node.progress >= 1.0f) {
                    if (node.output_buffer.get(node.mine_output) < node.output_max) {
                        node.output_buffer.add(node.mine_output, 1);
                        node.progress -= 1.0f;
                    } else {
                        node.progress = 1.0f; // Stalled
                    }
                }
            }
            else if (node.type == NODE_SMELTER || node.type == NODE_ASSEMBLER) {
                if (node.recipe_index < 0 || node.recipe_index >= rcount) continue;
                const Recipe& recipe = recipes[node.recipe_index];

                if (!node.is_producing) {
                    if (node.input_buffer.canCraft(recipe) && node.output_buffer.get(recipe.output.item) < node.output_max) {
                        // Consume from input buffer
                        for (int i = 0; i < recipe.input_count; i++) {
                            node.input_buffer.remove(recipe.inputs[i].item, recipe.inputs[i].count);
                        }
                        node.is_producing = true;
                        node.progress = 0.0f;
                    }
                }

                if (node.is_producing) {
                    node.progress += (dt / recipe.craft_time) * power_efficiency;
                    if (node.progress >= 1.0f) {
                        node.progress = 0.0f;
                        node.is_producing = false;
                        node.output_buffer.add(recipe.output.item, recipe.output.count);
                    }
                }
            }
            else if (node.type == NODE_STORAGE) {
                // Storage nodes "export" everything in input/output buffers to global warehouse
                // Actually, let's just make it export input_buffer to warehouse
                if (!node.input_buffer.empty()) {
                    for (auto const& pair : node.input_buffer.items) {
                        agency.addItem(pair.first, pair.second);
                    }
                    node.input_buffer.items.clear();
                }
                // If anything is in output_buffer (e.g. from previous phases), move it to agency too
                if (!node.output_buffer.empty()) {
                    for (auto const& pair : node.output_buffer.items) {
                        agency.addItem(pair.first, pair.second);
                    }
                    node.output_buffer.items.clear();
                }
            }
        }

        // 2. Logistics Logic (Belts)
        const float BELT_SPEED = 0.5f;    // Progress per second (takes 2s to cross)
        const float BELT_SPACING = 0.25f; // Min progress distance between items
        
        for (auto& belt : belts) {
            // A. Move items already on the belt
            bool blocked = false;
            for (int i = 0; i < (int)belt.items.size(); i++) {
                float next_pos = 1.1f; // end of belt
                if (i > 0) next_pos = belt.items[i-1].progress;
                
                if (belt.items[i].progress < 1.0f) {
                    float move = BELT_SPEED * dt;
                    if (belt.items[i].progress + move < next_pos - BELT_SPACING || i == 0) {
                        belt.items[i].progress += move;
                    }
                }
            }

            // B. Handoff items to destination building
            if (!belt.items.empty() && belt.items[0].progress >= 1.0f) {
                FactoryNode* to = findNode(belt.to_node_id);
                if (to) {
                    // Try to push into input buffer
                    to->input_buffer.add(belt.items[0].type, 1);
                    belt.items.erase(belt.items.begin());
                }
            }

            // C. Inject new items from source building
            belt.transfer_timer += dt;
            if (belt.transfer_timer >= 0.5f) { // check injection every 0.5s
                FactoryNode* from = findNode(belt.from_node_id);
                if (from && !from->output_buffer.empty()) {
                    // Can we fit a new item at the start (pos 0)?
                    bool space = true;
                    if (!belt.items.empty()) {
                        if (belt.items.back().progress < BELT_SPACING) space = false;
                    }

                    if (space) {
                        ItemType to_move = ITEM_TYPE_COUNT;
                        for (auto const& pair : from->output_buffer.items) {
                            if (pair.second > 0) { to_move = pair.first; break; }
                        }

                        if (to_move != ITEM_TYPE_COUNT) {
                            from->output_buffer.remove(to_move, 1);
                            BeltItem newItem;
                            newItem.type = to_move;
                            newItem.progress = 0.0f;
                            belt.items.push_back(newItem);
                            belt.transfer_timer = 0.0f;
                        }
                    }
                }
            }
        }
    }

    int countType(FactoryNodeType type) const {
        int c = 0;
        for (auto& n : nodes) if (n.type == type) c++;
        return c;
    }
};
