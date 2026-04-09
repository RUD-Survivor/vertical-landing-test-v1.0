import os
import re

def process_flight_input():
    path = r'c:\antigravity_code\RocketSim3D\src\scene\flight_input_system.h'
    with open(path, 'r', encoding='utf-8') as f:
        content = f.read()

    # setup signature
    old_sig = 'void setup(InputRouter& router, RocketState& rocket_state, RocketConfig& rocket_config,'
    new_sig = 'void setup(InputRouter& router, RocketConfig& rocket_config,'
    content = content.replace(old_sig, new_sig)

    # replace captures: [&rocket_state] -> [&world, rocket_entity]
    # some might be [&rocket_state, &hud] -> [&world, rocket_entity, &hud]
    content = re.sub(r'\[(.*?)rocket_state(.*?)\]', lambda m: '[' + m.group(1).replace('&', '') + '&world, rocket_entity' + m.group(2) + ']', content)
    # let's be more precise
    
    # Actually just add `auto& guid = world.get<GuidanceComponent>(rocket_entity);` etc. in lambdas
    def inject_components(match):
        body = match.group(0)
        prefix = '{\n            auto& guid = world.get<GuidanceComponent>(rocket_entity);\n            auto& mnv = world.get<ManeuverComponent>(rocket_entity);\n            auto& tele = world.get<TelemetryComponent>(rocket_entity);\n'
        return body.replace('{', prefix, 1)

    # We can inject components in every router.registerKey usage
    # Actually simpler: any lambda that uses guid, mnv, etc needs them.
    # We can just replace all [..., &world, ...] { with [..., &world, ...] { auto& guid = ...
    
    # Let me just rewrite the file content manually for flight_input_system.h with a regex or simple replaces.
    # Even better, since it's just a few lines of lambdas:
    
    lines = content.split('\n')
    out = []
    for line in lines:
        if 'router.on_ignition' in line:
            line = line.replace('&rocket_state', '&world, rocket_entity')
        elif 'router.registerKey' in line:
            # fix capture
            if '&rocket_state' in line:
                if '&world' in line:
                    line = line.replace('&rocket_state, ', '').replace('&rocket_state', '')
                else:
                    line = line.replace('&rocket_state', '&world, rocket_entity')
            # if we need to add &world, rocket_entity but it's not there and uses guid:
            elif 'hud' in line or 'cam' in line or 'show_clouds' in line or '&' in line:
                if '&world' not in line:
                    line = line.replace('[', '[&world, rocket_entity, ')
                    line = line.replace('[, ', '[')

        out.append(line)
        if '{' in line and ('router.on_ignition' in line or 'router.registerKey' in line):
            out.append('            auto& guid = world.get<GuidanceComponent>(rocket_entity);')
            out.append('            auto& mnv = world.get<ManeuverComponent>(rocket_entity);')
            out.append('            auto& tele = world.get<TelemetryComponent>(rocket_entity);')
            out.append('            auto& trans = world.get<TransformComponent>(rocket_entity);')
    
    content = '\n'.join(out)
    
    # poll signature
    content = content.replace('void poll(GLFWwindow* window, InputRouter& router, RocketState& rocket_state, Renderer3D* r3d) {', 
                              'void poll(GLFWwindow* window, InputRouter& router, entt::registry& world, entt::entity rocket_entity, Renderer3D* r3d) {\n        auto& guid = world.get<GuidanceComponent>(rocket_entity);\n        auto& tele = world.get<TelemetryComponent>(rocket_entity);\n        auto& trans = world.get<TransformComponent>(rocket_entity);')

    with open(path, 'w', encoding='utf-8') as f:
        f.write(content)

def process_flight_scene():
    path = r'c:\antigravity_code\RocketSim3D\src\scene\flight_scene.h'
    with open(path, 'r', encoding='utf-8') as f:
        content = f.read()

    # update setup call
    content = content.replace('inputSystem.setup(input, rocket_state, rocket_config', 'inputSystem.setup(input, rocket_config')
    
    # update poll call
    content = content.replace('inputSystem.poll(GameContext::getInstance().window, input, rocket_state, r3d);', 'inputSystem.poll(GameContext::getInstance().window, input, world, rocket_entity, r3d);')
    
    # remove lingering rocket_state decls: auto& rocket_state = world.get<RocketState>(rocket_entity);
    # EXCEPT we might need them if they are still used.
    # Just fix the compiler errors first.

    with open(path, 'w', encoding='utf-8') as f:
        f.write(content)

process_flight_input()
process_flight_scene()
print("Done")
