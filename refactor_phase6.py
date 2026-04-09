"""
Phase 6: Fix the ECS data synchronization gap.

Problem:
  - Input lambdas write to `rocket_state` (legacy)
  - Physics/Control systems read/write ECS components
  - Rendering reads `rocket_state` (legacy)
  => No bidirectional sync => white world, no ignition

Solution: Add two sync functions called every frame:
  1. syncLegacyToECS() — before physics (picks up input changes like SPACE=ignite)
  2. syncECSToLegacy() — after physics (propagates physics results to rendering)
"""

def read(p):
    with open(p, 'r', encoding='utf-8') as f: return f.read()
def write(p, d):
    with open(p, 'w', encoding='utf-8') as f: f.write(d)

FS = r'c:\antigravity_code\RocketSim3D\src\scene\flight_scene.h'
fs = read(FS)

# =================================================================
# 1. Add the two sync helper methods to FlightScene class
#    Insert them just before the update() function
# =================================================================

sync_methods = r'''
    // ===================================================================
    // ECS Bridge: Bidirectional Synchronization
    // -------------------------------------------------------------------
    // During the transition period, some systems write to RocketState
    // (e.g., input lambdas) while others write to ECS components
    // (e.g., PhysicsSystem). These two functions keep them in sync.
    // ===================================================================
    
    void syncLegacyToECS() {
        auto& rs = world.get<RocketState>(rocket_entity);
        auto& trans = world.get<TransformComponent>(rocket_entity);
        auto& vel   = world.get<VelocityComponent>(rocket_entity);
        auto& att   = world.get<AttitudeComponent>(rocket_entity);
        auto& prop  = world.get<PropulsionComponent>(rocket_entity);
        auto& tele  = world.get<TelemetryComponent>(rocket_entity);
        auto& guid  = world.get<GuidanceComponent>(rocket_entity);
        auto& mnv   = world.get<ManeuverComponent>(rocket_entity);

        trans.px = rs.px; trans.py = rs.py; trans.pz = rs.pz;
        trans.abs_px = rs.abs_px; trans.abs_py = rs.abs_py; trans.abs_pz = rs.abs_pz;
        trans.surf_px = rs.surf_px; trans.surf_py = rs.surf_py; trans.surf_pz = rs.surf_pz;
        trans.launch_site_px = rs.launch_site_px; trans.launch_site_py = rs.launch_site_py; trans.launch_site_pz = rs.launch_site_pz;
        trans.launch_latitude = rs.launch_latitude; trans.launch_longitude = rs.launch_longitude;

        vel.vx = rs.vx; vel.vy = rs.vy; vel.vz = rs.vz;
        vel.abs_vx = rs.abs_vx; vel.abs_vy = rs.abs_vy; vel.abs_vz = rs.abs_vz;
        vel.acceleration = rs.acceleration;

        att.attitude = rs.attitude; att.initialized = rs.attitude_initialized;
        att.angle = rs.angle; att.ang_vel = rs.ang_vel;
        att.angle_z = rs.angle_z; att.ang_vel_z = rs.ang_vel_z;
        att.angle_roll = rs.angle_roll; att.ang_vel_roll = rs.ang_vel_roll;

        prop.fuel = rs.fuel; prop.current_stage = rs.current_stage;
        prop.total_stages = rs.total_stages; prop.stage_fuels = rs.stage_fuels;
        prop.jettisoned_mass = rs.jettisoned_mass; prop.fuel_consumption_rate = rs.fuel_consumption_rate;
        prop.thrust_power = rs.thrust_power;

        tele.sim_time = rs.sim_time; tele.altitude = rs.altitude;
        tele.velocity = rs.velocity; tele.local_vx = rs.local_vx;
        tele.terrain_altitude = rs.terrain_altitude; tele.solar_occlusion = rs.solar_occlusion;

        guid.status = rs.status; guid.mission_msg = rs.mission_msg;
        guid.mission_phase = rs.mission_phase; guid.mission_timer = rs.mission_timer;
        guid.auto_mode = rs.auto_mode; guid.sas_active = rs.sas_active;
        guid.rcs_active = rs.rcs_active; guid.sas_mode = rs.sas_mode;
        guid.sas_target_vec = rs.sas_target_vec; guid.leg_deploy_progress = rs.leg_deploy_progress;
        guid.suicide_burn_locked = rs.suicide_burn_locked; guid.show_absolute_time = rs.show_absolute_time;
        guid.pid_vert = rs.pid_vert; guid.pid_pos = rs.pid_pos;
        guid.pid_att = rs.pid_att; guid.pid_att_z = rs.pid_att_z; guid.pid_att_roll = rs.pid_att_roll;

        mnv.maneuvers = rs.maneuvers; mnv.selected_maneuver_index = rs.selected_maneuver_index;
    }

    void syncECSToLegacy() {
        auto& rs = world.get<RocketState>(rocket_entity);
        auto& trans = world.get<TransformComponent>(rocket_entity);
        auto& vel   = world.get<VelocityComponent>(rocket_entity);
        auto& att   = world.get<AttitudeComponent>(rocket_entity);
        auto& prop  = world.get<PropulsionComponent>(rocket_entity);
        auto& tele  = world.get<TelemetryComponent>(rocket_entity);
        auto& guid  = world.get<GuidanceComponent>(rocket_entity);
        auto& mnv   = world.get<ManeuverComponent>(rocket_entity);
        auto& vfx   = world.get<VFXComponent>(rocket_entity);

        rs.px = trans.px; rs.py = trans.py; rs.pz = trans.pz;
        rs.abs_px = trans.abs_px; rs.abs_py = trans.abs_py; rs.abs_pz = trans.abs_pz;
        rs.surf_px = trans.surf_px; rs.surf_py = trans.surf_py; rs.surf_pz = trans.surf_pz;
        rs.launch_site_px = trans.launch_site_px; rs.launch_site_py = trans.launch_site_py; rs.launch_site_pz = trans.launch_site_pz;
        rs.launch_latitude = trans.launch_latitude; rs.launch_longitude = trans.launch_longitude;

        rs.vx = vel.vx; rs.vy = vel.vy; rs.vz = vel.vz;
        rs.abs_vx = vel.abs_vx; rs.abs_vy = vel.abs_vy; rs.abs_vz = vel.abs_vz;
        rs.acceleration = vel.acceleration;

        rs.attitude = att.attitude; rs.attitude_initialized = att.initialized;
        rs.angle = att.angle; rs.ang_vel = att.ang_vel;
        rs.angle_z = att.angle_z; rs.ang_vel_z = att.ang_vel_z;
        rs.angle_roll = att.angle_roll; rs.ang_vel_roll = att.ang_vel_roll;

        rs.fuel = prop.fuel; rs.current_stage = prop.current_stage;
        rs.total_stages = prop.total_stages; rs.stage_fuels = prop.stage_fuels;
        rs.jettisoned_mass = prop.jettisoned_mass; rs.fuel_consumption_rate = prop.fuel_consumption_rate;
        rs.thrust_power = prop.thrust_power;

        rs.sim_time = tele.sim_time; rs.altitude = tele.altitude;
        rs.velocity = tele.velocity; rs.local_vx = tele.local_vx;
        rs.terrain_altitude = tele.terrain_altitude; rs.solar_occlusion = tele.solar_occlusion;

        rs.status = guid.status; rs.mission_msg = guid.mission_msg;
        rs.mission_phase = guid.mission_phase; rs.mission_timer = guid.mission_timer;
        rs.auto_mode = guid.auto_mode; rs.sas_active = guid.sas_active;
        rs.rcs_active = guid.rcs_active; rs.sas_mode = guid.sas_mode;
        rs.sas_target_vec = guid.sas_target_vec; rs.leg_deploy_progress = guid.leg_deploy_progress;
        rs.suicide_burn_locked = guid.suicide_burn_locked; rs.show_absolute_time = guid.show_absolute_time;
        rs.pid_vert = guid.pid_vert; rs.pid_pos = guid.pid_pos;
        rs.pid_att = guid.pid_att; rs.pid_att_z = guid.pid_att_z; rs.pid_att_roll = guid.pid_att_roll;

        rs.maneuvers = mnv.maneuvers; rs.selected_maneuver_index = mnv.selected_maneuver_index;

        rs.smoke_particles = vfx.smoke_particles;
    }

'''

# Insert before update()
fs = fs.replace(
    '    void update(double dt) override {',
    sync_methods + '    void update(double dt) override {')

# =================================================================
# 2. Call syncLegacyToECS() BEFORE physics (after input polling)
#    Call syncECSToLegacy() AFTER physics (after sim_ctrl.update)
# =================================================================

# Insert syncLegacyToECS before sim_ctrl calls
fs = fs.replace(
    '        // --- Simulation & Physics Update ---\n        sim_ctrl.handleInput(GameContext::getInstance().window, world, rocket_entity);',
    '        // --- Simulation & Physics Update ---\n        syncLegacyToECS(); // Bridge: input changes -> ECS\n        sim_ctrl.handleInput(GameContext::getInstance().window, world, rocket_entity);')

# Insert syncECSToLegacy after sim_ctrl.update
fs = fs.replace(
    '        sim_ctrl.update(real_dt, world, rocket_entity, hudManager.hud, GameContext::getInstance().window, cam.mode);\n        // Update mouse state for HUD',
    '        sim_ctrl.update(real_dt, world, rocket_entity, hudManager.hud, GameContext::getInstance().window, cam.mode);\n        syncECSToLegacy(); // Bridge: physics results -> legacy rendering\n        // Update mouse state for HUD')

write(FS, fs)
print('[OK] flight_scene.h — bidirectional sync injected')

# =================================================================
# 3. Verify TelemetryComponent has velocity & local_vx fields
# =================================================================
RSH = r'c:\antigravity_code\RocketSim3D\src\core\rocket_state.h'
rsh = read(RSH)
if 'double velocity' not in rsh.split('struct TelemetryComponent')[1].split('};')[0] if 'struct TelemetryComponent' in rsh else '':
    print('[WARN] TelemetryComponent may be missing velocity/local_vx fields!')
else:
    print('[OK] TelemetryComponent has velocity field')

print('\n=== Phase 6 Data Sync Bridge COMPLETE ===')
