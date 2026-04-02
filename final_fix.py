import os

def final_fix():
    file_path = r'c:\antigravity_code\RocketSim3D\src\scene\flight_scene.h'
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
            enc = 'utf-8'
    except UnicodeDecodeError:
        with open(file_path, 'r', encoding='gbk') as f:
            content = f.read()
            enc = 'gbk'

    if "FlightHUD hud;" not in content:
        content = content.replace("CameraDirector cam;", "CameraDirector cam;\n    FlightHUD hud;")
    if '#include "render/HUD_system.h"' not in content:
        content = '#include "render/HUD_system.h"\n' + content

    if "Mesh earthMesh;" not in content:
        content = content.replace("CameraDirector cam;", "CameraDirector cam;\n    Mesh earthMesh;\n    Mesh ringMesh;\n    Mesh rocketBody;\n    Mesh rocketNose;\n    Mesh rocketBox;\n    Mesh launchPadMesh;\n    bool has_launch_pad;")

    if "earthMesh = " not in content.split("void onEnter()")[1][:1000]:
        content = content.replace("void onEnter() override {", """void onEnter() override {
        GameContext& ctx = GameContext::getInstance();
        earthMesh = MeshGen::sphere(256, 512, 1.0f);
        ringMesh = MeshGen::ring(128, 1.11f, 2.35f);
        rocketBody = MeshGen::cylinder(32, 1.0f, 1.0f);
        rocketNose = MeshGen::cone(32, 1.0f, 1.0f);
        rocketBox = MeshGen::box(1.0f, 1.0f, 1.0f);
        launchPadMesh = ModelLoader::loadOBJ("assets/launch_pad.obj");
        has_launch_pad = (launchPadMesh.indexCount > 0);
""")

    content = content.replace("orbit_predictor.", "GameContext::getInstance().orbit_predictor->")
    content = content.replace("Report_Status(", "// Report_Status(")

    with open(file_path, 'w', encoding=enc) as f:
        f.write(content)

final_fix()
print("Fixed FlightScene issues")
