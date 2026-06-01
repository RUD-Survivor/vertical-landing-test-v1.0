#pragma once
// ==========================================================
// maneuver_exec_system.h — 变轨自动执行系统
//
// 只作用于 ctx.focused_entity（玩家当前操控的火箭）
// 签名已升级为 update(registry, ctx)
// ==========================================================

#include "../system_scheduler.h"
#include "core/rocket_state.h"
#include "core/universe_model.h"
#include "simulation/maneuver_system.h"
#include "simulation/orbit_physics.h"
#include "physics/physics_system.h"
#include "render/HUD_system.h"
#include <cmath>

struct ManeuverExecSystem : ISystem {
    ManeuverExecSystem() : ISystem("ManeuverExec") {}

    bool update(entt::registry& registry, SystemContext& ctx) override {
        if (!registry.valid(ctx.focused_entity)) return true;

        auto& rocket_config = registry.get<RocketConfig>(ctx.focused_entity);
        auto& control_input = registry.get<ControlInput>(ctx.focused_entity);
        auto& prop   = registry.get<PropulsionComponent>(ctx.focused_entity);
        auto& tele   = registry.get<TelemetryComponent>(ctx.focused_entity);
        auto& guid   = registry.get<GuidanceComponent>(ctx.focused_entity);
        auto& mnv    = registry.get<ManeuverComponent>(ctx.focused_entity);
        auto& trans  = registry.get<TransformComponent>(ctx.focused_entity);
        auto& vel    = registry.get<VelocityComponent>(ctx.focused_entity);
        auto& att    = registry.get<AttitudeComponent>(ctx.focused_entity);

        ctx.mnv_autopilot_active = false;
        if (mnv.maneuvers.empty()) return true;
        if (!(guid.status == ASCEND || guid.status == DESCEND)) return true;

        FlightHUD* hud = static_cast<FlightHUD*>(ctx.hud);
        if (!hud) return true;

        auto& node = mnv.maneuvers[0];

        // 1. 估算点火时长
        double total_dv_mag = node.delta_v.length();
        double current_mass = prop.fuel + rocket_config.dry_mass + rocket_config.upper_stages_mass;
        double max_thrust = 0;
        if (prop.current_stage < (int)rocket_config.stage_configs.size())
            max_thrust = rocket_config.stage_configs[prop.current_stage].thrust;
        double ve = rocket_config.specific_impulse * 9.80665;

        if (max_thrust > 0 && ve > 0 && total_dv_mag > 0) {
            double m_0 = current_mass;
            double m_f = m_0 * exp(-total_dv_mag / ve);
            node.burn_duration = (m_0 - m_f) / (max_thrust / ve);
        }

        double time_to_burn_start = node.sim_time - tele.sim_time;

        // 2. 快照目标状态
        if (!node.snap_valid && time_to_burn_start < 5.0) {
            snapshotTarget(registry, ctx.focused_entity, node);
        }

        if (!hud->auto_exec_mnv) return true;

        // 3. 剩余 Delta-V
        double remaining_dv = total_dv_mag;
        if (node.snap_valid) {
            remaining_dv = calcRemainingDv(registry, ctx.focused_entity, node);
        }

        // 4. 姿态控制
        if (time_to_burn_start < 60.0) {
            ctx.mnv_autopilot_active = true;
            steerTowardBurnDir(registry, ctx.focused_entity, node);
        }

        // 5. 油门
        if (time_to_burn_start <= 0 && remaining_dv > 0.5) {
            control_input.throttle = 1.0;
            ctx.mnv_autopilot_active = true;
        } else if (node.snap_valid && remaining_dv <= 0.5) {
            control_input.throttle = 0;
            guid.mission_msg = "MNV COMPLETE";
            mnv.maneuvers.erase(mnv.maneuvers.begin());
            if (mnv.selected_maneuver_index >= 0) mnv.selected_maneuver_index--;
            if (mnv.maneuvers.empty()) hud->auto_exec_mnv = false;
        } else if (!ctx.mnv_autopilot_active) {
            control_input.throttle = 0;
        }

        return true;
    }

private:
    static void snapshotTarget(entt::registry& reg, entt::entity e, ManeuverNode& node) {
        auto& trans = reg.get<TransformComponent>(e);
        auto& vel   = reg.get<VelocityComponent>(e);
        auto& tele  = reg.get<TelemetryComponent>(e);
        int ref_idx = (node.ref_body >= 0) ? node.ref_body : UniverseModel::getInstance().current_soi_index;
        CelestialBody& ref_b = UniverseModel::getInstance().solar_system[ref_idx];
        CelestialBody& cur_b = UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index];
        double srvx = vel.vx + cur_b.vx - ref_b.vx;
        double srvy = vel.vy + cur_b.vy - ref_b.vy;
        double srvz = vel.vz + cur_b.vz - ref_b.vz;
        double srpx = trans.px + cur_b.px - ref_b.px;
        double srpy = trans.py + cur_b.py - ref_b.py;
        double srpz = trans.pz + cur_b.pz - ref_b.pz;
        double mu_ref = 6.67430e-11 * ref_b.mass;
        double npx, npy, npz, nvx, nvy, nvz;
        get3DStateAtTime(srpx, srpy, srpz, srvx, srvy, srvz, mu_ref, node.sim_time - tele.sim_time,
                        npx, npy, npz, nvx, nvy, nvz);
        ManeuverFrame frame = ManeuverSystem::getFrame(Vec3((float)npx,(float)npy,(float)npz), Vec3((float)nvx,(float)nvy,(float)nvz));
        Vec3 tdv = frame.prograde*node.delta_v.x + frame.normal*node.delta_v.y + frame.radial*node.delta_v.z;
        node.locked_burn_dir = tdv.normalized();
        node.snap_rel_px=npx; node.snap_rel_py=npy; node.snap_rel_pz=npz;
        node.snap_rel_vx=nvx+tdv.x; node.snap_rel_vy=nvy+tdv.y; node.snap_rel_vz=nvz+tdv.z;
        double rbpx,rbpy,rbpz,rbvx,rbvy,rbvz;
        PhysicsSystem::GetCelestialStateAt(ref_idx, node.sim_time, rbpx,rbpy,rbpz,rbvx,rbvy,rbvz);
        node.snap_px=rbpx+npx; node.snap_py=rbpy+npy; node.snap_pz=rbpz+npz;
        node.snap_vx=rbvx+node.snap_rel_vx; node.snap_vy=rbvy+node.snap_rel_vy; node.snap_vz=rbvz+node.snap_rel_vz;
        node.snap_time=node.sim_time; node.snap_valid=true;
    }

    static double calcRemainingDv(entt::registry& reg, entt::entity e, ManeuverNode& node) {
        auto& vel  = reg.get<VelocityComponent>(e);
        auto& tele = reg.get<TelemetryComponent>(e);
        int ref_idx = (node.ref_body >= 0) ? node.ref_body : UniverseModel::getInstance().current_soi_index;
        CelestialBody& ref_b = UniverseModel::getInstance().solar_system[ref_idx];
        CelestialBody& cur_b = UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index];
        double crvx=vel.vx+cur_b.vx-ref_b.vx, crvy=vel.vy+cur_b.vy-ref_b.vy, crvz=vel.vz+cur_b.vz-ref_b.vz;
        double mu_ref=6.67430e-11*ref_b.mass, dt_snap=tele.sim_time-node.snap_time;
        double tpx,tpy,tpz,tvx,tvy,tvz;
        get3DStateAtTime(node.snap_rel_px,node.snap_rel_py,node.snap_rel_pz,
                        node.snap_rel_vx,node.snap_rel_vy,node.snap_rel_vz, mu_ref, dt_snap,
                        tpx,tpy,tpz,tvx,tvy,tvz);
        Vec3 rem_v((float)(tvx-crvx),(float)(tvy-crvy),(float)(tvz-crvz));
        double rdv=(double)rem_v.dot(node.locked_burn_dir);
        if(rem_v.length()<0.05) rdv=0;
        return rdv;
    }

    static void steerTowardBurnDir(entt::registry& reg, entt::entity e, ManeuverNode& node) {
        auto& cfg = reg.get<RocketConfig>(e);
        auto& ctrl= reg.get<ControlInput>(e);
        auto& prop= reg.get<PropulsionComponent>(e);
        auto& att = reg.get<AttitudeComponent>(e);
        auto& trans=reg.get<TransformComponent>(e);
        auto& vel  =reg.get<VelocityComponent>(e);
        auto& tele =reg.get<TelemetryComponent>(e);

        Vec3 burn_dir = node.snap_valid ? node.locked_burn_dir : Vec3(0,0,0);
        if(burn_dir.length()<0.1f){
            double mu=6.67430e-11*UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index].mass;
            double npx,npy,npz,nvx,nvy,nvz;
            get3DStateAtTime(trans.px,trans.py,trans.pz,vel.vx,vel.vy,vel.vz,mu,node.sim_time-tele.sim_time,npx,npy,npz,nvx,nvy,nvz);
            ManeuverFrame f=ManeuverSystem::getFrame(Vec3((float)npx,(float)npy,(float)npz),Vec3((float)nvx,(float)nvy,(float)nvz));
            burn_dir=(f.prograde*node.delta_v.x+f.normal*node.delta_v.y+f.radial*node.delta_v.z).normalized();
        }
        Vec3 fwd=att.attitude.forward();
        float dp=fwd.dot(burn_dir);
        Vec3 ea=fwd.cross(burn_dir); float em=ea.length();
        if(dp<-0.999f&&em<0.01f){ea=att.attitude.right();em=1.0f;}
        double tm=cfg.dry_mass+prop.fuel+cfg.upper_stages_mass;
        float moi=(float)(50000.0*(tm/50000.0));
        if(em>0.001f){
            ea=ea/em; float eang=std::asin(std::min(em,1.0f));
            if(dp<0) eang=(float)PI-eang;
            float kp=moi*32.0f,kd=moi*12.0f;
            Vec3 ra=att.attitude.right(),ua=att.attitude.up();
            ctrl.torque_cmd=kp*ea.dot(ua)*eang-kd*(float)att.ang_vel;
            ctrl.torque_cmd_z=kp*ea.dot(ra)*eang-kd*(float)att.ang_vel_z;
        }else{
            ctrl.torque_cmd=-moi*8.0f*(float)att.ang_vel;
            ctrl.torque_cmd_z=-moi*8.0f*(float)att.ang_vel_z;
        }
        ctrl.torque_cmd_roll=-moi*8.0f*(float)att.ang_vel_roll;
    }
};
