import os

def fix_all():
    file_path = r'c:\antigravity_code\RocketSim3D\src\scene\game_context.h'
    with open(file_path, 'r', encoding='gbk') as f:
        content = f.read()
        
    if "AsyncOrbitPredictor* orbit_predictor" not in content:
        content = content.replace("FactorySystem factory;", 
                                  "FactorySystem factory;\n    class Simulation::AsyncOrbitPredictor* orbit_predictor = nullptr;")

    # Include predictor
    if "predictor.h" not in content:
        content = '#include "simulation/predictor.h"\n' + content
        
    with open(file_path, 'w', encoding='gbk') as f:
        f.write(content)

    # Now main.cpp predictor
    main_path = r'c:\antigravity_code\RocketSim3D\src\main.cpp'
    with open(main_path, 'r', encoding='utf-8') as f:
        m_content = f.read()
    
    if "ctx.orbit_predictor = &orbit_predictor;" not in m_content:
        m_content = m_content.replace("orbit_predictor.Start();", 
                                      "orbit_predictor.Start();\n  ctx.orbit_predictor = &orbit_predictor;")
    
    with open(main_path, 'w', encoding='utf-8') as f:
        f.write(m_content)

    # FlightScene fixes
    flight_path = r'c:\antigravity_code\RocketSim3D\src\scene\flight_scene.h'
    with open(flight_path, 'r', encoding='gbk') as f:
        fl_content = f.read()

    # Meshes
    if "Mesh earthMesh;" not in fl_content:
        fl_content = fl_content.replace("CameraDirector cam;", 
            "CameraDirector cam;\n    Mesh earthMesh;\n    Mesh ringMesh;\n    Mesh rocketBody;\n    Mesh rocketNose;\n    Mesh rocketBox;\n    Mesh launchPadMesh;\n    bool has_launch_pad;")

    if "earthMesh = " not in fl_content.split("void onEnter")[1][:1000]:
        fl_content = fl_content.replace("void onEnter() override {", """void onEnter() override {
        GameContext& ctx = GameContext::getInstance();
        earthMesh = MeshGen::sphere(256, 512, 1.0f);
        ringMesh = MeshGen::ring(128, 1.11f, 2.35f);
        rocketBody = MeshGen::cylinder(32, 1.0f, 1.0f);
        rocketNose = MeshGen::cone(32, 1.0f, 1.0f);
        rocketBox = MeshGen::box(1.0f, 1.0f, 1.0f);
        launchPadMesh = ModelLoader::loadOBJ("assets/launch_pad.obj");
        has_launch_pad = (launchPadMesh.indexCount > 0);
""")

    if "orbit_predictor" in fl_content and "ctx.orbit_predictor" not in fl_content:
        fl_content = fl_content.replace("orbit_predictor", "GameContext::getInstance().orbit_predictor")
        
    if "Report_Status(" in fl_content:
        fl_content = fl_content.replace("Report_Status(", "// Report_Status(")

    with open(flight_path, 'w', encoding='gbk') as f:
        f.write(fl_content)

if __name__ == '__main__':
    fix_all()
