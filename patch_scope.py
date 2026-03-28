import re
import os

MAIN_FILE = r'c:\antigravity_code\RocketSim3D\src\main.cpp'

def patch():
    with open(MAIN_FILE, 'r', encoding='utf-8') as f:
        content = f.read()

    # 1. Add definitions
    target_declare = "    Mat4 macroProjMat;"
    replacement_declare = """    Mat4 macroProjMat;
    Vec3 camEye_rel;
    float aspect;"""
    content = content.replace(target_declare, replacement_declare)

    # 2. Fix local shadowing
    content = content.replace("Vec3 camEye_rel = camResult.eye;", "camEye_rel = camResult.eye;")
    content = content.replace("float aspect = (float)ww / (float)wh;", "aspect = (float)ww / (float)wh;")

    with open(MAIN_FILE, 'w', encoding='utf-8') as f:
        f.write(content)

    print("Patched main.cpp variable scoping")

if __name__ == '__main__':
    patch()
