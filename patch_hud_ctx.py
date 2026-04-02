import codecs

path = r'c:\antigravity_code\RocketSim3D\src\scene\flight_scene.h'
with codecs.open(path, 'r', 'utf-8') as f:
    text = f.read()

# Remove local declarations
text = text.replace("Quat rocketQuat;", "")
text = text.replace("Vec3 rocketUp, localNorth, localRight;", "")

# Add to Core Members
if "Quat rocketQuat;" not in text:
    text = text.replace("// === Core Members ===", "// === Core Members ===\n    Quat rocketQuat;\n    Vec3 rocketUp, localNorth, localRight;")

# Add to hud_ctx in render()
if "hud_ctx.rocketQuat =" not in text:
    target = "hud_ctx.ws_d = 1.0; // placeholder for ws_d"
    replacement = target + "\n        hud_ctx.rocketQuat = &rocketQuat;\n        hud_ctx.rocketUp = &rocketUp;\n        hud_ctx.localNorth = &localNorth;\n        hud_ctx.localRight = &localRight;"
    text = text.replace(target, replacement)

with codecs.open(path, 'w', 'utf-8') as f:
    f.write(text)

print("Applied patch to flight_scene.h")
