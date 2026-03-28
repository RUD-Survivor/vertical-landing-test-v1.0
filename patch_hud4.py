import os

HUD_FILE = r'c:\antigravity_code\RocketSim3D\src\render\HUD_system.h'
MAIN_FILE = r'c:\antigravity_code\RocketSim3D\src\main.cpp'

def patch():
    # 1. Update HUD_system.h
    with open(HUD_FILE, 'r', encoding='utf-8') as f:
        hud = f.read()

    ctx_old =    "    const Vec3* localNorth;\n"
    ctx_new = """    const Vec3* localNorth;
    double ws_d;
    const float* global_best_ang;
"""
    hud = hud.replace(ctx_old, ctx_new)

    unpack_old = "        const Vec3& localNorth = *ctx.localNorth;\n"
    unpack_new = """        const Vec3& localNorth = *ctx.localNorth;
        double ws_d = ctx.ws_d;
        float global_best_ang = *ctx.global_best_ang;
"""
    hud = hud.replace(unpack_old, unpack_new)

    with open(HUD_FILE, 'w', encoding='utf-8') as f:
        f.write(hud)


    # 2. Update main.cpp
    with open(MAIN_FILE, 'r', encoding='utf-8') as f:
        main_cpp = f.read()

    ctx_assign_old = "    hud_ctx.localNorth = &localNorth;\n"
    ctx_assign_new = """    hud_ctx.localNorth = &localNorth;
    hud_ctx.ws_d = ws_d;
    hud_ctx.global_best_ang = &global_best_ang;
"""
    main_cpp = main_cpp.replace(ctx_assign_old, ctx_assign_new)

    with open(MAIN_FILE, 'w', encoding='utf-8') as f:
        f.write(main_cpp)
        
    print("Patched global_best_ang and ws_d")

if __name__ == '__main__':
    patch()
