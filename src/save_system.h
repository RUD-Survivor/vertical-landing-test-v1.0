#pragma once
// ==========================================================
// save_system.h — 游戏保存/加载系统
// ==========================================================

#include "core/rocket_state.h"
#include "simulation/rocket_builder.h"
#include "core/agency_state.h"
#include "simulation/factory_system.h"
#include "simulation/resource_system.h"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>

namespace SaveSystem {

// 保存文件路径
const char* SAVE_FILE_PATH = "rocket_save.dat";

// 保存数据结构
struct SaveData {
    // 火箭装配数据
    std::vector<int> part_ids;
    
    // 火箭状态
    double fuel;
    double px, py, pz;
    double vx, vy, vz;
    double abs_px, abs_py, abs_pz;
    double angle, ang_vel;
    double angle_z, ang_vel_z;
    double sim_time;
    double altitude;
    double velocity;
    double local_vx;
    int status;
    int current_soi;
    bool auto_mode;
    
    // Multi-stage state
    int current_stage;
    int total_stages;
    double jettisoned_mass;
    std::vector<double> stage_fuels;
    
    // 控制输入
    double throttle;
    double torque_cmd;
    double torque_cmd_z;
    
    // 时间戳
    double save_timestamp;
};

// 简单的序列化函数(使用文本格式便于调试)
bool SaveGame(const RocketAssembly& assembly, const RocketState& state, 
              const ControlInput& input) {
    std::ofstream file(SAVE_FILE_PATH);
    if (!file.is_open()) {
        return false;
    }
    
    // 写入版本号
    file << "VERSION 2.0\n";
    
    // 保存火箭装配
    file << "PARTS " << assembly.parts.size() << "\n";
    for (const auto& part : assembly.parts) {
        file << part.def_id << " " << part.stage << "\n";
    }
    
    // 保存火箭状态
    file << "STATE\n";
    file << state.fuel << "\n";
    file << state.px << " " << state.py << " " << state.pz << "\n";
    file << state.vx << " " << state.vy << " " << state.vz << "\n";
    file << state.surf_px << " " << state.surf_py << " " << state.surf_pz << " " 
         << state.launch_site_px << " " << state.launch_site_py << " " << state.launch_site_pz << "\n";
    file << state.abs_px << " " << state.abs_py << " " << state.abs_pz << "\n";
    file << state.angle << " " << state.ang_vel << "\n";
    file << state.angle_z << " " << state.ang_vel_z << "\n";
    file << state.sim_time << "\n";
    file << state.altitude << "\n";
    file << state.velocity << "\n";
    file << state.local_vx << "\n";
    file << (int)state.status << "\n";
    file << current_soi_index << "\n";
    file << (state.auto_mode ? 1 : 0) << "\n";
    
    // 保存多级状态
    file << "STAGES\n";
    file << state.current_stage << " " << state.total_stages << "\n";
    file << state.jettisoned_mass << "\n";
    file << (int)state.stage_fuels.size() << "\n";
    for (int i = 0; i < (int)state.stage_fuels.size(); i++) {
        file << state.stage_fuels[i] << "\n";
    }
    
    // 保存控制输入
    file << "CONTROL\n";
    file << input.throttle << "\n";
    file << input.torque_cmd << "\n";
    file << input.torque_cmd_z << "\n";
    
    // 保存时间戳
    file << "TIMESTAMP\n";
    file << state.sim_time << "\n";
    
    file.close();
    return true;
}

// 加载游戏
bool LoadGame(RocketAssembly& assembly, RocketState& state, 
              ControlInput& input) {
    std::ifstream file(SAVE_FILE_PATH);
    if (!file.is_open()) {
        return false;
    }
    
    std::string line;
    
    // 读取版本号
    std::getline(file, line);
    if (line.find("VERSION") == std::string::npos) {
        file.close();
        return false;
    }
    
    // 读取火箭装配
    std::getline(file, line);
    if (line.find("PARTS") != std::string::npos) {
        int part_count;
        std::istringstream iss(line.substr(6));
        iss >> part_count;
        
        assembly.parts.clear();
        for (int i = 0; i < part_count; i++) {
            int def_id, stage;
            file >> def_id >> stage;
            PlacedPart p;
            p.def_id = def_id;
            p.stage = stage;
            p.stack_y = 0;
            assembly.parts.push_back(p);
        }
        assembly.recalculate();
        std::getline(file, line); // 消耗换行符
    }
    
    // 读取火箭状态
    std::getline(file, line);
    if (line.find("STATE") != std::string::npos) {
        file >> state.fuel;
        file >> state.px >> state.py >> state.pz;
        file >> state.vx >> state.vy >> state.vz;
        file >> state.surf_px >> state.surf_py >> state.surf_pz 
             >> state.launch_site_px >> state.launch_site_py >> state.launch_site_pz;
        file >> state.abs_px >> state.abs_py >> state.abs_pz;
        file >> state.angle >> state.ang_vel;
        file >> state.angle_z >> state.ang_vel_z;
        file >> state.sim_time;
        file >> state.altitude;
        file >> state.velocity;
        file >> state.local_vx;
        int status_int;
        file >> status_int;
        state.status = (MissionState)status_int;
        file >> current_soi_index;
        int auto_mode_int;
        file >> auto_mode_int;
        state.auto_mode = (auto_mode_int != 0);
        std::getline(file, line); // 消耗换行符
    }
    
    // 读取多级状态 (VERSION 2.0+)
    std::streampos pos = file.tellg();
    std::getline(file, line);
    if (line.find("STAGES") != std::string::npos) {
        file >> state.current_stage >> state.total_stages;
        file >> state.jettisoned_mass;
        int num_stage_fuels;
        file >> num_stage_fuels;
        state.stage_fuels.clear();
        for (int i = 0; i < num_stage_fuels; i++) {
            double sf;
            file >> sf;
            state.stage_fuels.push_back(sf);
        }
        std::getline(file, line); // consume newline
        
        // Read CONTROL section
        std::getline(file, line);
    } else if (line.find("CONTROL") != std::string::npos) {
        // Version 1.0: no stages section, this IS the CONTROL line
        state.current_stage = 0;
        state.total_stages = 1;
        state.jettisoned_mass = 0;
        state.stage_fuels.clear();
        state.stage_fuels.push_back(state.fuel);
    }
    
    // Read control values (CONTROL header already consumed above)
    if (line.find("CONTROL") != std::string::npos || true) {
        file >> input.throttle;
        file >> input.torque_cmd;
        file >> input.torque_cmd_z;
        std::getline(file, line); // consume newline
    }
    
    // 读取时间戳(可选)
    std::getline(file, line);
    if (line.find("TIMESTAMP") != std::string::npos) {
        double timestamp;
        file >> timestamp;
    }
    
    file.close();
    return true;
}

// ==========================================
// Save Agency and Factory data
// ==========================================
inline bool SaveAgencyFactory(const AgencyState& agency, const FactorySystem& factory) {
    std::ofstream file("agency_save.dat");
    if (!file.is_open()) return false;

    file << "AGENCY_VERSION 1.0\n";
    
    // 1. Agency Stats
    file << "STATS\n";
    file << agency.funds << " " << agency.science << " " << agency.reputation << " " << agency.global_time << "\n";

    // 2. Inventory (AgencyState::inventory is std::map)
    file << "INVENTORY " << (int)agency.inventory.size() << "\n";
    for (std::map<ItemType, int>::const_iterator it = agency.inventory.begin(); it != agency.inventory.end(); ++it) {
        file << (int)it->first << " " << it->second << "\n";
    }

    // 3. Factory Nodes
    file << "NODES " << (int)factory.nodes.size() << "\n";
    for (size_t i = 0; i < factory.nodes.size(); i++) {
        const FactoryNode& n = factory.nodes[i];
        file << (int)n.type << " " << n.id << " " << n.grid_x << " " << n.grid_y << " ";
        file << n.progress << " " << (n.is_producing ? 1 : 0) << " ";
        file << (int)n.mine_output << " " << n.recipe_index << "\n";
        
        // Internal Buffers (FactoryNode::input_buffer is Inventory struct)
        file << "IBUFFER " << (int)n.input_buffer.items.size() << "\n";
        for (std::map<ItemType, int>::const_iterator it = n.input_buffer.items.begin(); it != n.input_buffer.items.end(); ++it) {
            file << (int)it->first << " " << it->second << "\n";
        }
        file << "OBUFFER " << (int)n.output_buffer.items.size() << "\n";
        for (std::map<ItemType, int>::const_iterator it = n.output_buffer.items.begin(); it != n.output_buffer.items.end(); ++it) {
            file << (int)it->first << " " << it->second << "\n";
        }
    }

    // 4. Belts
    file << "BELTS " << (int)factory.belts.size() << "\n";
    for (size_t i = 0; i < factory.belts.size(); i++) {
        const ConveyorBelt& b = factory.belts[i];
        file << b.from_node_id << " " << b.to_node_id << " " << b.transfer_timer << "\n";
        // Belt items
        file << "ITEMS " << (int)b.items.size() << "\n";
        for (size_t j = 0; j < b.items.size(); j++) {
            file << (int)b.items[j].type << " " << b.items[j].progress << "\n";
        }
    }

    file.close();
    return true;
}

// ==========================================
// Load Agency and Factory data
// ==========================================
inline bool LoadAgencyFactory(AgencyState& agency, FactorySystem& factory) {
    std::ifstream file("agency_save.dat");
    if (!file.is_open()) return false;

    std::string line;
    if (!std::getline(file, line) || line.find("AGENCY_VERSION") == std::string::npos) {
        file.close();
        return false;
    }

    factory.nodes.clear();
    factory.belts.clear();
    agency.inventory.clear();
    factory.next_id = 1;

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        if (line == "STATS") {
            file >> agency.funds >> agency.science >> agency.reputation >> agency.global_time;
            std::getline(file, line); // finish the STATS data line
        } 
        else if (line.find("INVENTORY") == 0) {
            int count = 0;
            if (line.length() > 10) count = std::stoi(line.substr(10));
            for (int i = 0; i < count; i++) {
                int type, qty;
                if (file >> type >> qty) agency.addItem((ItemType)type, qty);
            }
            std::getline(file, line); // finish last inventory line
        }
        else if (line.find("NODES") == 0) {
            int count = 0;
            if (line.length() > 6) count = std::stoi(line.substr(6));
            for (int i = 0; i < count; i++) {
                FactoryNode n;
                int n_type, is_prod, m_out;
                if (!(file >> n_type >> n.id >> n.grid_x >> n.grid_y >> n.progress >> is_prod >> m_out >> n.recipe_index)) break;
                n.type = (FactoryNodeType)n_type;
                n.is_producing = (is_prod != 0);
                n.mine_output = (ItemType)m_out;
                std::getline(file, line); // finish node data line

                // Buffers
                for (int b = 0; b < 2; b++) {
                    if (!std::getline(file, line)) break;
                    if (line.find("IBUFFER") == 0) {
                        int icount = std::stoi(line.substr(8));
                        for (int j = 0; j < icount; j++) {
                            int t, q; if (file >> t >> q) n.input_buffer.add((ItemType)t, q);
                        }
                        if (icount > 0) std::getline(file, line); // consume remainder
                    } else if (line.find("OBUFFER") == 0) {
                        int ocount = std::stoi(line.substr(8));
                        for (int j = 0; j < ocount; j++) {
                            int t, q; if (file >> t >> q) n.output_buffer.add((ItemType)t, q);
                        }
                        if (ocount > 0) std::getline(file, line); // consume remainder
                    }
                }
                
                factory.nodes.push_back(n);
                if (n.id >= factory.next_id) factory.next_id = n.id + 1;
            }
        }
        else if (line.find("BELTS") == 0) {
            int count = 0;
            if (line.length() > 6) count = std::stoi(line.substr(6));
            for (int i = 0; i < count; i++) {
                ConveyorBelt b;
                if (!(file >> b.from_node_id >> b.to_node_id >> b.transfer_timer)) break;
                std::getline(file, line); // finish belt line
                
                if (std::getline(file, line) && line.find("ITEMS") == 0) {
                    int icount = std::stoi(line.substr(6));
                    for (int j = 0; j < icount; j++) {
                        int t; float p;
                        if (file >> t >> p) {
                            BeltItem itm; itm.type = (ItemType)t; itm.progress = p;
                            b.items.push_back(itm);
                        }
                    }
                    if (icount > 0) std::getline(file, line); // consume remainder
                }
                factory.belts.push_back(b);
            }
        }
    }
    file.close();
    return true;
}

// 检查是否存在保存文件
bool HasSaveFile() {
    std::ifstream file(SAVE_FILE_PATH);
    bool exists = file.good();
    file.close();
    return exists;
}

bool HasAgencySave() {
    std::ifstream file("agency_save.dat");
    bool exists = file.good();
    file.close();
    return exists;
}

// 删除保存文件
void DeleteSaveFile() {
    std::remove(SAVE_FILE_PATH);
}

// 获取保存文件信息(用于显示)
bool GetSaveInfo(double& sim_time, int& part_count) {
    std::ifstream file(SAVE_FILE_PATH);
    if (!file.is_open()) {
        return false;
    }
    
    std::string line;
    
    // 跳过版本号
    std::getline(file, line);
    
    // 读取零件数量
    std::getline(file, line);
    if (line.find("PARTS") != std::string::npos) {
        std::istringstream iss(line.substr(6));
        iss >> part_count;
    }
    
    // 跳过零件数据
    for (int i = 0; i < part_count; i++) {
        std::getline(file, line);
    }
    
    // 跳到STATE部分
    std::getline(file, line);
    if (line.find("STATE") != std::string::npos) {
        // 跳过前面的数据直到sim_time
        double dummy;
        file >> dummy; // fuel
        file >> dummy >> dummy >> dummy; // px py pz
        file >> dummy >> dummy >> dummy; // vx vy vz
        file >> dummy >> dummy >> dummy; // abs_px abs_py abs_pz
        file >> dummy >> dummy; // angle ang_vel
        file >> dummy >> dummy; // angle_z ang_vel_z
        file >> sim_time; // 这是我们要的
    }
    
    file.close();
    return true;
}

} // namespace SaveSystem
