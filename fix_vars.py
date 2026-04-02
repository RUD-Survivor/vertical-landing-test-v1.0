import codecs

path = r'c:\antigravity_code\RocketSim3D\src\scene\flight_scene.h'
with codecs.open(path, 'r', 'utf-8') as f:
    text = f.read()

# Add assembly to update
if "const RocketAssembly& assembly =" not in text.split("void update(double")[1][:200]:
    text = text.replace("double real_dt = dt;", "double real_dt = dt;\n        const RocketAssembly& assembly = GameContext::getInstance().launch_assembly;")

# Add assembly to render
if "const RocketAssembly& assembly =" not in text.split("void render()")[1][:200]:
    text = text.replace("void render() override {", "void render() override {\n        const RocketAssembly& assembly = GameContext::getInstance().launch_assembly;")

# Add r3d missing in update if previously missed
if "Renderer3D* r3d =" not in text.split("void update(double")[1][:200]:
    text = text.replace("double real_dt = dt;", "double real_dt = dt;\n        Renderer3D* r3d = GameContext::getInstance().renderer3d;")

with codecs.open(path, 'w', 'utf-8') as f:
    f.write(text)

print("Added assembly and r3d locally to update/render methods.")
