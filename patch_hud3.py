import os

HUD_FILE = r'c:\antigravity_code\RocketSim3D\src\render\HUD_system.h'
MAIN_FILE = r'c:\antigravity_code\RocketSim3D\src\main.cpp'

def patch():
    # 1. Update HUD_system.h
    with open(HUD_FILE, 'r', encoding='utf-8') as f:
        hud = f.read()

    # Context fields
    ctx_old = "    int frame;\n"
    ctx_new = """    int frame;
    const Quat* rocketQuat;
    const Vec3* localRight;
    const Vec3* rocketUp;
    const Vec3* localNorth;
"""
    hud = hud.replace(ctx_old, ctx_new)

    # Class member type fix
    hud = hud.replace("TransferResult transfer_result;", "PorkchopResult transfer_result;")

    # render() unpack
    unpack_old = "        int frame = ctx.frame;\n"
    unpack_new = """        int frame = ctx.frame;
        const Quat& rocketQuat = *ctx.rocketQuat;
        const Vec3& localRight = *ctx.localRight;
        const Vec3& rocketUp = *ctx.rocketUp;
        const Vec3& localNorth = *ctx.localNorth;
"""
    hud = hud.replace(unpack_old, unpack_new)

    with open(HUD_FILE, 'w', encoding='utf-8') as f:
        f.write(hud)


    # 2. Update main.cpp
    with open(MAIN_FILE, 'r', encoding='utf-8') as f:
        main_cpp = f.read()

    ctx_assign_old = "    hud_ctx.frame = frame;\n"
    ctx_assign_new = """    hud_ctx.frame = frame;
    hud_ctx.rocketQuat = &rocketQuat;
    hud_ctx.localRight = &localRight;
    hud_ctx.rocketUp = &rocketUp;
    hud_ctx.localNorth = &localNorth;
"""
    main_cpp = main_cpp.replace(ctx_assign_old, ctx_assign_new)

    with open(MAIN_FILE, 'w', encoding='utf-8') as f:
        f.write(main_cpp)
        
    print("Patched attitude var injections")

if __name__ == '__main__':
    patch()
