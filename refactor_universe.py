import os
import glob
import re

universe_model_code = """#pragma once
#include <vector>
#include "rocket_state.h"

class UniverseModel {
public:
    static UniverseModel& getInstance() {
        static UniverseModel instance;
        return instance;
    }

    std::vector<CelestialBody> solar_system;
    int current_soi_index = 3;
};
"""
with open(r'c:\antigravity_code\RocketSim3D\src\core\universe_model.h', 'w', encoding='utf-8') as f:
    f.write(universe_model_code)

state_path = r'c:\antigravity_code\RocketSim3D\src\core\rocket_state.h'
with open(state_path, 'r', encoding='utf-8') as f:
    text = f.read()

text = re.sub(r'extern std::vector<CelestialBody> SOLAR_SYSTEM;\s*', '', text)
text = re.sub(r'extern int current_soi_index;\s*', '', text)
with open(state_path, 'w', encoding='utf-8') as f:
    f.write(text)

files = glob.glob(r'c:\antigravity_code\RocketSim3D\src\**\*.cpp', recursive=True) + glob.glob(r'c:\antigravity_code\RocketSim3D\src\**\*.h', recursive=True)

for file in files:
    with open(file, 'r', encoding='utf-8') as f:
        content = f.read()
        
    modified = False
    
    if 'SOLAR_SYSTEM' in content:
        content = content.replace('SOLAR_SYSTEM', 'UniverseModel::getInstance().solar_system')
        modified = True
        
    if 'current_soi_index' in content and 'UniverseModel::getInstance().current_soi_index' not in content:
        content = re.sub(r'\bcurrent_soi_index\b', 'UniverseModel::getInstance().current_soi_index', content)
        modified = True
        
    if modified:
        if 'universe_model.h' not in content:
            if '#pragma once' in content:
                content = content.replace('#pragma once', '#pragma once\n#include "core/universe_model.h"')
            else:
                content = '#include "core/universe_model.h"\n' + content
        with open(file, 'w', encoding='utf-8') as f:
            f.write(content)

phys_path = r'c:\antigravity_code\RocketSim3D\src\physics\physics_system.cpp'
with open(phys_path, 'r', encoding='utf-8') as f:
    content = f.read()

content = re.sub(r'std::vector<CelestialBody> UniverseModel::getInstance\(\)\.solar_system;\s*', '', content)
content = re.sub(r'int UniverseModel::getInstance\(\)\.current_soi_index = 3;\s*', '', content)

with open(phys_path, 'w', encoding='utf-8') as f:
    f.write(content)

print("Universe encapsulation complete!")
