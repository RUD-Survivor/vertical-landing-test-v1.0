#pragma once
// ==========================================================
// save_system.h — 游戏保存/加载系统
// ==========================================================

#include "core/rocket_state.h"
#include "simulation/rocket_builder.h"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

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
    file << "VERSION 1.0\n";
    
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
    
    // 读取控制输入
    std::getline(file, line);
    if (line.find("CONTROL") != std::string::npos) {
        file >> input.throttle;
        file >> input.torque_cmd;
        file >> input.torque_cmd_z;
        std::getline(file, line); // 消耗换行符
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

// 检查是否存在保存文件
bool HasSaveFile() {
    std::ifstream file(SAVE_FILE_PATH);
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
