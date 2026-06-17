#include "ground_collision_system.h"

#include "physics/voxel/vessel_voxel_model.h"
#include "scene/game_context.h"
#include "simulation/rocket_builder.h"
#include <algorithm>
#include <cmath>
#include <string>

namespace GroundCollisionSystem {
namespace {

constexpr double kContactSlop = 0.02;          // meters
constexpr double kRestitution = 0.08;          // mostly inelastic soil/landing-leg contact
constexpr double kStaticFriction = 0.75;
constexpr double kDynamicFriction = 0.55;
constexpr double kMaxDepenetrationSpeed = 35.0; // m/s, prevents deep-hit solver explosions
constexpr double kCrushImpactSpeed = 35.0;      // m/s; above this plastic absorption ramps in

static double restitutionForClosingSpeed(double closingSpeed, bool hardContact) {
    double e0 = hardContact ? (kRestitution * 0.65) : kRestitution;
    double decay = hardContact ? 45.0 : 90.0;
    double e = e0 * std::exp(-std::max(0.0, closingSpeed) / decay);
    return std::clamp(e, 0.0, 0.28);
}

static void applyPlasticCrush(Vec3& linVel,
                              Vec3& omega,
                              const Vec3& groundVel,
                              const Vec3& normal,
                              double closingSpeed) {
    if (closingSpeed <= kCrushImpactSpeed) return;
    double t = std::clamp((closingSpeed - kCrushImpactSpeed) / (kCrushImpactSpeed * 2.5), 0.0, 0.90);
    Vec3 rel = linVel - groundVel;
    double outbound = rel.dot(normal);
    if (outbound > 0.0) linVel -= normal * (outbound * t);
    linVel = groundVel + (linVel - groundVel) * (1.0 - t * 0.70);
    omega *= (1.0 - t * 0.60);
}

static double chunkInertiaScalar(const RigidChunkComponent& chunk) {
    Vec3 size = chunk.local_bounds_max - chunk.local_bounds_min;
    double rx = std::max(0.25, size.x * 0.5);
    double ry = std::max(0.25, size.y * 0.5);
    double rz = std::max(0.25, size.z * 0.5);
    double ixx = chunk.mass * (ry * ry + rz * rz) / 3.0;
    double iyy = chunk.mass * (rx * rx + rz * rz) / 3.0;
    double izz = chunk.mass * (rx * rx + ry * ry) / 3.0;
    double avg = (ixx + iyy + izz) / 3.0;
    return std::max(1.0, avg);
}

struct GroundFrame {
    Quat bodyToWorld;
    Quat worldToBody;
    Vec3 normal;
    Vec3 omegaWorld;
};

static GroundFrame makeGroundFrame(const CelestialBody& body, double simTime, const Vec3& referencePos) {
    double theta = body.prime_meridian_epoch + (simTime * 2.0 * PI / body.rotation_period);
    Quat rot = Quat::fromAxisAngle(Vec3(0, 0, 1), theta);
    Quat tilt = Quat::fromAxisAngle(Vec3(1, 0, 0), body.axial_tilt);
    GroundFrame frame;
    frame.bodyToWorld = tilt * rot;
    frame.worldToBody = frame.bodyToWorld.conjugate();
    frame.normal = referencePos.lengthSq() > 1e-9 ? referencePos.normalized() : Vec3(0, 1, 0);
    double omega = (2.0 * PI) / body.rotation_period;
    frame.omegaWorld = frame.bodyToWorld.rotate(Vec3(0, 0, omega));
    return frame;
}

static Vec3 surfaceVelocityAt(const GroundFrame& frame, const Vec3& worldPos) {
    return frame.omegaWorld.cross(worldPos);
}

static Vec3 angularVelocityWorld(const AttitudeComponent& att) {
    return att.attitude.rotate(Vec3(att.ang_vel_z, att.ang_vel_roll, att.ang_vel));
}

static void addAngularVelocityWorld(AttitudeComponent& att, const Vec3& deltaWorld) {
    Vec3 local = att.attitude.conjugate().rotate(deltaWorld);
    att.ang_vel_z += local.x;
    att.ang_vel_roll += local.y;
    att.ang_vel += local.z;
}

static bool finiteVec(const Vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

static bool finiteQuat(const Quat& q) {
    return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}

static void sanitizeAttitude(AttitudeComponent& att) {
    if (!finiteQuat(att.attitude)) {
        att.attitude = Quat();
        att.initialized = false;
    } else {
        att.attitude = att.attitude.normalized();
        if (!finiteQuat(att.attitude)) {
            att.attitude = Quat();
            att.initialized = false;
        }
    }

    Vec3 omega(att.ang_vel_z, att.ang_vel_roll, att.ang_vel);
    if (!finiteVec(omega)) {
        att.ang_vel_z = 0.0;
        att.ang_vel_roll = 0.0;
        att.ang_vel = 0.0;
    }
}

static int activePartStart(const RocketConfig& config, const PropulsionComponent& prop) {
    if (prop.current_stage >= 0 && prop.current_stage < (int)config.stage_configs.size()) {
        return config.stage_configs[prop.current_stage].part_start_index;
    }
    return 0;
}

static void addSymmetricContactSamples(std::vector<Vec3>& out,
                                       const PlacedPart& part,
                                       const PartDef& def,
                                       int symmetryIndex) {
    constexpr double kTwoPi = 6.28318530717958647692;
    double symAngle = (part.symmetry > 1) ? (symmetryIndex * kTwoPi / part.symmetry) : 0.0;
    Vec3 localPos = part.pos;
    if (part.symmetry > 1) {
        double dist = std::sqrt(part.pos.x * part.pos.x + part.pos.z * part.pos.z);
        if (dist > 0.01) {
            double ca = std::atan2(part.pos.z, part.pos.x);
            localPos.x = std::cos(ca + symAngle) * dist;
            localPos.z = std::sin(ca + symAngle) * dist;
        }
    }

    Quat symRot = Quat::fromAxisAngle(Vec3(0, 1, 0), symAngle);
    Quat localRot = part.rot * symRot;
    bool isLandingLeg = (std::string(def.name) == "Landing Leg");
    if (isLandingLeg) {
        out.push_back(localPos);
        return;
    }

    double r = std::max(0.2, (double)def.diameter * 0.5);
    const double ys[] = { 0.0, std::max(0.0, (double)def.height) };
    for (double y : ys) {
        out.push_back(localPos + localRot.rotate(Vec3(0, y, 0)));
        for (int i = 0; i < 6; i++) {
            double a = kTwoPi * (double)i / 6.0;
            Vec3 ringPoint(std::cos(a) * r, y, std::sin(a) * r);
            out.push_back(localPos + localRot.rotate(ringPoint));
        }
    }
}

static std::vector<Vec3> collectContactSamples(const RocketConfig& config,
                                               const PropulsionComponent& prop,
                                               const StructuralStateComponent* structural) {
    std::vector<Vec3> localSamples;
    const RocketAssembly& assembly = GameContext::getInstance().launch_assembly;
    int renderStart = activePartStart(config, prop);

    if (!assembly.parts.empty()) {
        for (int pi = renderStart; pi < (int)assembly.parts.size(); pi++) {
            if (!isPartStructurallyActive(structural, pi)) continue;
            const PlacedPart& part = assembly.parts[pi];
            if (part.def_id < 0 || part.def_id >= PART_CATALOG_SIZE) continue;
            const PartDef& def = PART_CATALOG[part.def_id];
            int symmetry = std::max(1, part.symmetry);
            for (int s = 0; s < symmetry; s++) {
                addSymmetricContactSamples(localSamples, part, def, s);
            }
        }
    }

    if (localSamples.empty()) {
        double r = std::max(0.5, config.diameter * 0.5);
        double y = config.bounds_bottom;
        localSamples.push_back(Vec3(0, y, 0));
        for (int i = 0; i < 8; i++) {
            double a = 2.0 * PI * (double)i / 8.0;
            localSamples.push_back(Vec3(std::cos(a) * r, y, std::sin(a) * r));
        }
    }

    return localSamples;
}

static double lowestAltitudeAt(const Vec3& com,
                               const AttitudeComponent& att,
                               const std::vector<Vec3>& localSamples,
                               const TelemetryComponent& tele,
                               const CelestialBody& body) {
    double lowest = 1e30;
    for (const Vec3& local : localSamples) {
        Vec3 world = com + att.attitude.rotate(local);
        Vec3 normal = world.lengthSq() > 1e-9 ? world.normalized() : Vec3(0, 1, 0);
        double terrainH = SampleTerrainHeightMeters(normal, tele);
        double pointAlt = world.length() - body.radius - terrainH;
        lowest = std::min(lowest, pointAlt);
    }
    return lowest;
}

static bool applyHighSpeedTerrainTOI(const RocketConfig& config,
                                     const PropulsionComponent& prop,
                                     TransformComponent& trans,
                                     VelocityComponent& vel,
                                     AttitudeComponent& att,
                                     const GuidanceComponent& guid,
                                     const TelemetryComponent& tele,
                                     const CelestialBody& body,
                                     double totalMass,
                                     double simTime,
                                     double dt,
                                     const Vec3& previousCom,
                                     const std::vector<Vec3>& samples,
                                     ContactResult& result) {
    if (guid.status == PRE_LAUNCH || dt <= 0.0 || samples.empty()) return false;
    Vec3 currentCom(trans.px, trans.py, trans.pz);
    Vec3 sweep = currentCom - previousCom;
    if (sweep.lengthSq() < 1e-12) return false;

    double bestAlpha = 2.0;
    Vec3 bestLocal;
    Vec3 bestNormal;
    bool found = false;

    for (const Vec3& local : samples) {
        Vec3 prevWorld = previousCom + att.attitude.rotate(local);
        Vec3 currWorld = currentCom + att.attitude.rotate(local);
        Vec3 prevNormal = prevWorld.lengthSq() > 1e-9 ? prevWorld.normalized() : Vec3(0, 1, 0);
        Vec3 currNormal = currWorld.lengthSq() > 1e-9 ? currWorld.normalized() : prevNormal;
        double prevAlt = prevWorld.length() - body.radius - SampleTerrainHeightMeters(prevNormal, tele);
        double currAlt = currWorld.length() - body.radius - SampleTerrainHeightMeters(currNormal, tele);
        if (prevAlt > kContactSlop && currAlt <= kContactSlop) {
            double denom = std::max(1e-6, prevAlt - currAlt);
            double alpha = std::max(0.0, std::min(1.0, (prevAlt - kContactSlop) / denom));
            if (alpha < bestAlpha) {
                bestAlpha = alpha;
                bestLocal = local;
                bestNormal = (prevNormal * (1.0 - alpha) + currNormal * alpha).normalized();
                if (bestNormal.lengthSq() < 1e-9) bestNormal = currNormal;
                found = true;
            }
        }
    }

    if (!found) {
        double prevLowest = lowestAltitudeAt(previousCom, att, samples, tele, body);
        double currLowest = lowestAltitudeAt(currentCom, att, samples, tele, body);
        if (!(currLowest < -std::max(1.0, sweep.length() * 0.25))) return false;
        bestAlpha = 0.0;
        bestLocal = Vec3(0, config.bounds_bottom, 0);
        Vec3 normal = currentCom.lengthSq() > 1e-9 ? currentCom.normalized() : Vec3(0, 1, 0);
        bestNormal = normal;
    }

    Vec3 hitCom = previousCom + sweep * bestAlpha;
    Vec3 contactWorld = hitCom + att.attitude.rotate(bestLocal);
    Vec3 normal = bestNormal;
    double terrainH = SampleTerrainHeightMeters(normal, tele);
    double contactAlt = contactWorld.length() - body.radius - terrainH;
    hitCom += normal * (kContactSlop - contactAlt);

    trans.px = hitCom.x;
    trans.py = hitCom.y;
    trans.pz = hitCom.z;

    contactWorld = hitCom + att.attitude.rotate(bestLocal);
    GroundFrame frame = makeGroundFrame(body, simTime, contactWorld);
    Vec3 groundVel = surfaceVelocityAt(frame, contactWorld);
    Vec3 comVel(vel.vx, vel.vy, vel.vz);
    Vec3 omega = angularVelocityWorld(att);
    Vec3 r = contactWorld - hitCom;
    Vec3 relVel = (comVel + omega.cross(r)) - groundVel;
    double vn = relVel.dot(normal);
    if (vn < 0.0) {
        double closingSpeed = -vn;
        result.hardImpact = true;
        result.maxClosingSpeed = std::max(result.maxClosingSpeed, closingSpeed);
        result.hasImpactLocalPoint = true;
        result.impactLocalPoint = bestLocal;

        double radius = std::max(0.5, config.diameter * 0.5);
        double height = std::max(1.0, config.height);
        double inertia = std::max(1.0, totalMass * (3.0 * radius * radius + height * height) / 12.0);
        double invMass = 1.0 / std::max(1.0, totalMass);
        double invInertia = 1.0 / inertia;
        Vec3 rn = r.cross(normal);
        double denom = invMass + rn.lengthSq() * invInertia;
        double restitution = restitutionForClosingSpeed(closingSpeed, closingSpeed > 8.0);
        double jn = -(1.0 + restitution) * vn / std::max(1e-6, denom);
        Vec3 impulseN = normal * jn;
        comVel += impulseN * invMass;
        omega += r.cross(impulseN) * invInertia;

        Vec3 postRel = (comVel + omega.cross(r)) - groundVel;
        Vec3 tangent = postRel - normal * postRel.dot(normal);
        double tangentLen = tangent.length();
        if (tangentLen > 1e-5) {
            Vec3 tdir = tangent / tangentLen;
            Vec3 rt = r.cross(tdir);
            double denomT = invMass + rt.lengthSq() * invInertia;
            double jtIdeal = -tangentLen / std::max(1e-6, denomT);
            double jtMax = kDynamicFriction * jn;
            double jt = std::max(-jtMax, std::min(jtIdeal, jtMax));
            Vec3 impulseT = tdir * jt;
            comVel += impulseT * invMass;
            omega += r.cross(impulseT) * invInertia;
        }

        applyPlasticCrush(comVel, omega, groundVel, normal, closingSpeed);

        if (!finiteVec(comVel) || !finiteVec(omega)) {
            comVel = groundVel;
            omega = Vec3(0, 0, 0);
        }

        vel.vx = comVel.x;
        vel.vy = comVel.y;
        vel.vz = comVel.z;
        Vec3 deltaOmega = omega - angularVelocityWorld(att);
        addAngularVelocityWorld(att, deltaOmega);
    }

    return true;
}

static std::vector<ContactPoint> buildContactManifold(const RocketConfig& config,
                                                      const PropulsionComponent& prop,
                                                      const TransformComponent& trans,
                                                      const VelocityComponent& vel,
                                                      const AttitudeComponent& att,
                                                      const TelemetryComponent& tele,
                                                      const CelestialBody& body,
                                                      double simTime,
                                                      const std::vector<Vec3>& localSamples) {
    const RocketAssembly& assembly = GameContext::getInstance().launch_assembly;
    int renderStart = activePartStart(config, prop);

    Vec3 com(trans.px, trans.py, trans.pz);
    Vec3 linVel(vel.vx, vel.vy, vel.vz);
    Vec3 omega = angularVelocityWorld(att);
    std::vector<ContactPoint> contacts;
    contacts.reserve(localSamples.size());

    for (const Vec3& local : localSamples) {
        ContactPoint cp;
        cp.local = local;
        cp.world = com + att.attitude.rotate(local);
        cp.normal = cp.world.lengthSq() > 1e-9 ? cp.world.normalized() : Vec3(0, 1, 0);
        double terrainH = SampleTerrainHeightMeters(cp.normal, tele);
        double pointAlt = cp.world.length() - body.radius;
        cp.penetration = terrainH - pointAlt;
        if (cp.penetration <= -kContactSlop) continue;

        GroundFrame frame = makeGroundFrame(body, simTime, cp.world);
        cp.groundVelocity = surfaceVelocityAt(frame, cp.world);
        Vec3 r = cp.world - com;
        Vec3 contactVel = linVel + omega.cross(r);
        Vec3 relVel = contactVel - cp.groundVelocity;
        cp.normalSpeed = relVel.dot(cp.normal);
        Vec3 tangent = relVel - cp.normal * cp.normalSpeed;
        cp.tangentSpeed = tangent.length();
        cp.isLandingLeg = false;

        if (!assembly.parts.empty()) {
            for (int pi = renderStart; pi < (int)assembly.parts.size(); pi++) {
                const PlacedPart& p = assembly.parts[pi];
                if (p.def_id >= 0 && p.def_id < PART_CATALOG_SIZE && std::string(PART_CATALOG[p.def_id].name) == "Landing Leg") {
                    if ((local - p.pos).length() < 0.5) {
                        cp.isLandingLeg = true;
                        break;
                    }
                }
            }
        }
        contacts.push_back(cp);
    }
    return contacts;
}

static ContactResult solveGroundContacts(std::vector<ContactPoint>& contacts,
                                         TransformComponent& trans,
                                         VelocityComponent& vel,
                                         AttitudeComponent& att,
                                         GuidanceComponent& guid,
                                         const RocketConfig& config,
                                         double totalMass,
                                         double dt) {
    ContactResult result;
    if (contacts.empty()) return result;

    Vec3 com(trans.px, trans.py, trans.pz);
    Vec3 linVel(vel.vx, vel.vy, vel.vz);
    Vec3 omega = angularVelocityWorld(att);
    double radius = std::max(0.5, config.diameter * 0.5);
    double height = std::max(1.0, config.height);
    double inertia = std::max(1.0, totalMass * (3.0 * radius * radius + height * height) / 12.0);
    double invMass = 1.0 / std::max(1.0, totalMass);
    double invInertia = 1.0 / inertia;

    Vec3 correctionNormal(0, 0, 0);
    double worstClosing = 0.0;
    Vec3 worstImpactLocal;
    for (ContactPoint& cp : contacts) {
        if (cp.penetration <= 0.0 && cp.normalSpeed >= 0.0) continue;

        result.contactCount++;
        if (cp.isLandingLeg) result.legContactCount++;
        result.maxPenetration = std::max(result.maxPenetration, cp.penetration);
        result.maxClosingSpeed = std::max(result.maxClosingSpeed, -cp.normalSpeed);
        result.maxTangentSpeed = std::max(result.maxTangentSpeed, cp.tangentSpeed);
        if (-cp.normalSpeed > worstClosing) {
            worstClosing = -cp.normalSpeed;
            worstImpactLocal = cp.local;
        }

        double hardNormalLimit = cp.isLandingLeg ? 14.0 : 8.0;
        double hardTangentLimit = cp.isLandingLeg ? 22.0 : 14.0;
        bool hardContact = (-cp.normalSpeed > hardNormalLimit || cp.tangentSpeed > hardTangentLimit);
        if (hardContact) {
            result.hardImpact = true;
            if (!cp.isLandingLeg) result.anyNonLegHardContact = true;
        }

        Vec3 r = cp.world - com;
        Vec3 relVel = (linVel + omega.cross(r)) - cp.groundVelocity;
        double vn = relVel.dot(cp.normal);
        if (vn < 0.0) {
            Vec3 rn = r.cross(cp.normal);
            double denom = invMass + rn.lengthSq() * invInertia;
            double closingSpeed = -cp.normalSpeed;
            double restitution = restitutionForClosingSpeed(closingSpeed, hardContact);
            double jn = -(1.0 + restitution) * vn / std::max(1e-6, denom);
            Vec3 impulseN = cp.normal * jn;
            linVel += impulseN * invMass;
            omega += r.cross(impulseN) * invInertia;

            Vec3 relAfterN = (linVel + omega.cross(r)) - cp.groundVelocity;
            Vec3 tangent = relAfterN - cp.normal * relAfterN.dot(cp.normal);
            double tangentLen = tangent.length();
            if (tangentLen > 1e-5) {
                Vec3 tdir = tangent / tangentLen;
                Vec3 rt = r.cross(tdir);
                double denomT = invMass + rt.lengthSq() * invInertia;
                double jtIdeal = -tangentLen / std::max(1e-6, denomT);
                double jtMax = (tangentLen < 0.25 ? kStaticFriction : kDynamicFriction) * jn;
                double jt = std::max(-jtMax, std::min(jtIdeal, jtMax));
                Vec3 impulseT = tdir * jt;
                linVel += impulseT * invMass;
                omega += r.cross(impulseT) * invInertia;
            }
        }

        if (cp.penetration > kContactSlop) {
            correctionNormal += cp.normal * (cp.penetration - kContactSlop);
        }
    }

    if (result.contactCount > 0) {
        if (correctionNormal.lengthSq() > 1e-10) {
            Vec3 correction = correctionNormal / (double)result.contactCount;
            correction *= 0.85;
            double maxCorrection = std::max(0.05, kMaxDepenetrationSpeed * std::max(0.001, dt));
            double correctionLen = correction.length();
            if (correctionLen > maxCorrection) {
                correction = correction * (maxCorrection / correctionLen);
            }
            trans.px += correction.x;
            trans.py += correction.y;
            trans.pz += correction.z;
        }

        Vec3 pos(trans.px, trans.py, trans.pz);
        Vec3 normal = pos.lengthSq() > 1e-9 ? pos.normalized() : Vec3(0, 1, 0);

        if (result.hardImpact) {
            Vec3 groundVel = contacts.front().groundVelocity;
            applyPlasticCrush(linVel, omega, groundVel, normal, result.maxClosingSpeed);
        }

        if (!finiteVec(linVel) || !finiteVec(omega)) {
            linVel = contacts.front().groundVelocity;
            omega = Vec3(0, 0, 0);
        }

        vel.vx = linVel.x;
        vel.vy = linVel.y;
        vel.vz = linVel.z;
        Vec3 deltaOmega = omega - angularVelocityWorld(att);
        addAngularVelocityWorld(att, deltaOmega);

        double angularSpeed = omega.length();
        double tiltCos = att.attitude.forward().normalized().dot(Vec3(trans.px, trans.py, trans.pz).normalized());
        bool uprightEnough = tiltCos > (result.legContactCount > 0 ? std::cos(50.0 * PI / 180.0) : std::cos(25.0 * PI / 180.0));
        result.stable =
            !result.hardImpact &&
            result.contactCount >= 1 &&
            result.maxClosingSpeed < 1.5 &&
            result.maxTangentSpeed < 1.0 &&
            angularSpeed < 0.08 &&
            uprightEnough;

        if (!uprightEnough && result.maxClosingSpeed > 2.5) {
            result.hardImpact = true;
        }

        if (result.hardImpact) {
            guid.mission_msg = result.anyNonLegHardContact
                ? "HARD IMPACT: STRUCTURE HIT TERRAIN"
                : "HARD IMPACT: LANDING LOAD EXCEEDED";
            if (worstClosing > 0.0) {
                result.hasImpactLocalPoint = true;
                result.impactLocalPoint = worstImpactLocal;
            }
        }
    }
    return result;
}

} // namespace

double SampleTerrainHeightMeters(const Vec3& surfaceNormal, const TelemetryComponent& tele) {
    Terrain::QuadtreeTerrain* terrain = GameContext::getInstance().terrain;
    if (terrain && surfaceNormal.lengthSq() > 1e-9) {
        Quat unalign = Quat::fromAxisAngle(Vec3(1, 0, 0), PI / 2.0);
        Vec3 terrainNormal = unalign.rotate(surfaceNormal.normalized());
        return (double)terrain->getHeight(terrainNormal) * 1000.0;
    }
    return tele.terrain_altitude;
}

Vec3 SurfaceVelocityAt(const CelestialBody& body, double simTime, const Vec3& worldPos) {
    GroundFrame frame = makeGroundFrame(body, simTime, worldPos);
    return surfaceVelocityAt(frame, worldPos);
}

void FreezeSurfacePoint(TransformComponent& trans, const CelestialBody& body, double simTime) {
    Vec3 worldPos(trans.px, trans.py, trans.pz);
    GroundFrame frame = makeGroundFrame(body, simTime, worldPos);
    Vec3 localRel = frame.worldToBody.rotate(worldPos);
    trans.surf_px = localRel.x;
    trans.surf_py = localRel.y;
    trans.surf_pz = localRel.z;
}

std::vector<Vec3> BuildOwnedVoxelContactSamples(const VoxelPhysics::VesselVoxelModel& model,
                                                 const std::vector<VoxelPhysics::VoxelId>& owned,
                                                 const Vec3& chunkCom,
                                                 int maxSamples) {
    std::vector<Vec3> out;
    if (owned.empty() || maxSamples <= 0) return out;
    const auto& cells = model.getCells();
    double half = model.voxelSize() * 0.5;
    std::vector<Vec3> candidates;
    candidates.reserve(owned.size() * 8);
    for (VoxelPhysics::VoxelId id : owned) {
        if (id < 0 || id >= (VoxelPhysics::VoxelId)cells.size()) continue;
        const auto& cell = cells[(size_t)id];
        for (int dx = -1; dx <= 1; dx += 2) {
            for (int dy = -1; dy <= 1; dy += 2) {
                for (int dz = -1; dz <= 1; dz += 2) {
                    Vec3 corner = cell.center + Vec3(dx * half, dy * half, dz * half);
                    candidates.push_back(corner - chunkCom);
                }
            }
        }
    }
    if (candidates.empty()) return out;
    int stride = std::max(1, (int)candidates.size() / maxSamples);
    out.reserve(std::min(maxSamples, (int)candidates.size()));
    for (int i = 0; i < (int)candidates.size() && (int)out.size() < maxSamples; i += stride) {
        out.push_back(candidates[(size_t)i]);
    }
    if (out.empty()) out.push_back(Vec3(0, 0, 0));
    return out;
}

std::vector<Vec3> BuildChunkBoundsContactSamples(const RigidChunkComponent& chunk) {
    std::vector<Vec3> out;
    Vec3 com = chunk.local_center_of_mass;
    Vec3 mn = chunk.local_bounds_min;
    Vec3 mx = chunk.local_bounds_max;
    for (int ix = 0; ix <= 1; ix++) {
        for (int iy = 0; iy <= 1; iy++) {
            for (int iz = 0; iz <= 1; iz++) {
                Vec3 p(mn.x + (mx.x - mn.x) * ix,
                       mn.y + (mx.y - mn.y) * iy,
                       mn.z + (mx.z - mn.z) * iz);
                out.push_back(p - com);
            }
        }
    }
    return out;
}

std::vector<Vec3> ResolveStructureContactSamples(const RocketConfig& config,
                                                 const PropulsionComponent& prop,
                                                 const VoxelBodyComponent* voxelBody,
                                                 const RigidChunkComponent* chunkComp,
                                                 const StructuralStateComponent* structural) {
    if (voxelBody && chunkComp && voxelBody->structure_fractured && voxelBody->model) {
        if (!chunkComp->owned_voxels.empty()) {
            auto samples = BuildOwnedVoxelContactSamples(*voxelBody->model,
                                                         chunkComp->owned_voxels,
                                                         chunkComp->local_center_of_mass,
                                                         96);
            if (!samples.empty()) return samples;
        }
        auto boundsSamples = BuildChunkBoundsContactSamples(*chunkComp);
        if (!boundsSamples.empty()) return boundsSamples;
    }
    return collectContactSamples(config, prop, structural);
}

std::vector<Vec3> BuildVoxelContactSamples(const VoxelBodyComponent& voxel,
                                           const RigidChunkComponent& chunk,
                                           int maxSamples) {
    if (!voxel.model || maxSamples <= 0) return {};
    if (!chunk.owned_voxels.empty()) {
        return BuildOwnedVoxelContactSamples(*voxel.model,
                                             chunk.owned_voxels,
                                             chunk.local_center_of_mass,
                                             maxSamples);
    }
    std::vector<Vec3> bounds = BuildChunkBoundsContactSamples(chunk);
    if (!bounds.empty()) return bounds;

    const auto& surfaces = voxel.model->getSurfaceSamples();
    const auto& chunks = voxel.model->getChunks();
    if (chunk.chunk_id < 0 || chunk.chunk_id >= (VoxelPhysics::ChunkId)chunks.size()) return bounds;
    const auto& summary = chunks[(size_t)chunk.chunk_id];
    if (summary.surface_indices.empty()) return bounds;

    int stride = std::max(1, (int)summary.surface_indices.size() / maxSamples);
    bounds.reserve(std::min(maxSamples, (int)summary.surface_indices.size()));
    for (int i = 0; i < (int)summary.surface_indices.size() && (int)bounds.size() < maxSamples; i += stride) {
        int si = summary.surface_indices[(size_t)i];
        if (si < 0 || si >= (int)surfaces.size()) continue;
        bounds.push_back(surfaces[(size_t)si].local_pos - chunk.local_center_of_mass);
    }
    if (bounds.empty()) bounds.push_back(Vec3(0, 0, 0));
    return bounds;
}

static bool applyChunkTerrainTOI(TransformComponent& trans,
                               VelocityComponent& vel,
                               AttitudeComponent& att,
                               const CelestialBody& body,
                               const TelemetryComponent& tele,
                               double mass,
                               double inertia,
                               double simTime,
                               double dt,
                               const Vec3& previousCom,
                               const std::vector<Vec3>& samples,
                               ContactResult& result) {
    if (dt <= 0.0 || samples.empty()) return false;

    Vec3 currentCom(trans.px, trans.py, trans.pz);
    Vec3 sweep = currentCom - previousCom;
    if (sweep.lengthSq() < 1e-12) return false;

    double bestAlpha = 2.0;
    Vec3 bestLocal;
    Vec3 bestNormal;
    bool found = false;

    for (const Vec3& local : samples) {
        Vec3 prevWorld = previousCom + att.attitude.rotate(local);
        Vec3 currWorld = currentCom + att.attitude.rotate(local);
        Vec3 prevNormal = prevWorld.lengthSq() > 1e-9 ? prevWorld.normalized() : Vec3(0, 1, 0);
        Vec3 currNormal = currWorld.lengthSq() > 1e-9 ? currWorld.normalized() : prevNormal;
        double prevAlt = prevWorld.length() - body.radius - SampleTerrainHeightMeters(prevNormal, tele);
        double currAlt = currWorld.length() - body.radius - SampleTerrainHeightMeters(currNormal, tele);
        if (prevAlt > kContactSlop && currAlt <= kContactSlop) {
            double denom = std::max(1e-6, prevAlt - currAlt);
            double alpha = std::max(0.0, std::min(1.0, (prevAlt - kContactSlop) / denom));
            if (alpha < bestAlpha) {
                bestAlpha = alpha;
                bestLocal = local;
                bestNormal = (prevNormal * (1.0 - alpha) + currNormal * alpha).normalized();
                if (bestNormal.lengthSq() < 1e-9) bestNormal = currNormal;
                found = true;
            }
        }
    }

    if (!found) {
        double prevLowest = lowestAltitudeAt(previousCom, att, samples, tele, body);
        double currLowest = lowestAltitudeAt(currentCom, att, samples, tele, body);
        if (!(currLowest < -std::max(1.0, sweep.length() * 0.25))) return false;
        bestAlpha = 0.0;
        bestLocal = samples.front();
        bestNormal = currentCom.lengthSq() > 1e-9 ? currentCom.normalized() : Vec3(0, 1, 0);
    }

    Vec3 hitCom = previousCom + sweep * bestAlpha;
    Vec3 contactWorld = hitCom + att.attitude.rotate(bestLocal);
    Vec3 normal = bestNormal;
    double terrainH = SampleTerrainHeightMeters(normal, tele);
    double contactAlt = contactWorld.length() - body.radius - terrainH;
    hitCom += normal * (kContactSlop - contactAlt);

    trans.px = hitCom.x;
    trans.py = hitCom.y;
    trans.pz = hitCom.z;

    contactWorld = hitCom + att.attitude.rotate(bestLocal);
    Vec3 groundVel = SurfaceVelocityAt(body, simTime, contactWorld);
    Vec3 comVel(vel.vx, vel.vy, vel.vz);
    Vec3 omega = angularVelocityWorld(att);
    Vec3 r = contactWorld - hitCom;
    Vec3 relVel = (comVel + omega.cross(r)) - groundVel;
    double vn = relVel.dot(normal);
    if (vn >= 0.0) return found;

    double closingSpeed = -vn;
    result.hardImpact = true;
    result.maxClosingSpeed = std::max(result.maxClosingSpeed, closingSpeed);
    result.hasImpactLocalPoint = true;
    result.impactLocalPoint = bestLocal;

    double invMass = 1.0 / std::max(1.0, mass);
    double invInertia = 1.0 / std::max(1.0, inertia);
    Vec3 rn = r.cross(normal);
    double denom = invMass + rn.lengthSq() * invInertia;
    double restitution = restitutionForClosingSpeed(closingSpeed, closingSpeed > 6.0);
    double jn = -(1.0 + restitution) * vn / std::max(1e-6, denom);
    Vec3 impulseN = normal * jn;
    comVel += impulseN * invMass;
    omega += r.cross(impulseN) * invInertia;

    Vec3 postRel = (comVel + omega.cross(r)) - groundVel;
    Vec3 tangent = postRel - normal * postRel.dot(normal);
    double tangentLen = tangent.length();
    if (tangentLen > 1e-5) {
        Vec3 tdir = tangent / tangentLen;
        Vec3 rt = r.cross(tdir);
        double denomT = invMass + rt.lengthSq() * invInertia;
        double jtIdeal = -tangentLen / std::max(1e-6, denomT);
        double jtMax = kDynamicFriction * jn;
        double jt = std::max(-jtMax, std::min(jtIdeal, jtMax));
        Vec3 impulseT = tdir * jt;
        comVel += impulseT * invMass;
        omega += r.cross(impulseT) * invInertia;
    }

    applyPlasticCrush(comVel, omega, groundVel, normal, closingSpeed);

    vel.vx = comVel.x;
    vel.vy = comVel.y;
    vel.vz = comVel.z;
    Vec3 deltaOmega = omega - angularVelocityWorld(att);
    addAngularVelocityWorld(att, deltaOmega);
    return true;
}

static ContactResult solveChunkGroundContacts(std::vector<ContactPoint>& contacts,
                                              TransformComponent& trans,
                                              VelocityComponent& vel,
                                              AttitudeComponent& att,
                                              double mass,
                                              double inertia,
                                              double dt) {
    ContactResult result;
    if (contacts.empty()) return result;

    Vec3 com(trans.px, trans.py, trans.pz);
    Vec3 linVel(vel.vx, vel.vy, vel.vz);
    Vec3 omega = angularVelocityWorld(att);
    double invMass = 1.0 / std::max(1.0, mass);
    double invInertia = 1.0 / std::max(1.0, inertia);

    Vec3 correctionNormal(0, 0, 0);
    for (ContactPoint& cp : contacts) {
        if (cp.penetration <= 0.0 && cp.normalSpeed >= 0.0) continue;
        result.contactCount++;
        result.maxPenetration = std::max(result.maxPenetration, cp.penetration);
        result.maxClosingSpeed = std::max(result.maxClosingSpeed, -cp.normalSpeed);
        result.maxTangentSpeed = std::max(result.maxTangentSpeed, cp.tangentSpeed);
        if (-cp.normalSpeed > 6.0) result.hardImpact = true;

        Vec3 r = cp.world - com;
        Vec3 relVel = (linVel + omega.cross(r)) - cp.groundVelocity;
        double vn = relVel.dot(cp.normal);
        if (vn < 0.0) {
            double closingSpeed = -vn;
            Vec3 rn = r.cross(cp.normal);
            double denom = invMass + rn.lengthSq() * invInertia;
            double restitution = restitutionForClosingSpeed(closingSpeed, closingSpeed > 6.0);
            double jn = -(1.0 + restitution) * vn / std::max(1e-6, denom);
            Vec3 impulseN = cp.normal * jn;
            linVel += impulseN * invMass;
            omega += r.cross(impulseN) * invInertia;

            Vec3 relAfterN = (linVel + omega.cross(r)) - cp.groundVelocity;
            Vec3 tangent = relAfterN - cp.normal * relAfterN.dot(cp.normal);
            double tangentLen = tangent.length();
            if (tangentLen > 1e-5) {
                Vec3 tdir = tangent / tangentLen;
                Vec3 rt = r.cross(tdir);
                double denomT = invMass + rt.lengthSq() * invInertia;
                double jtIdeal = -tangentLen / std::max(1e-6, denomT);
                double jtMax = kDynamicFriction * jn;
                double jt = std::max(-jtMax, std::min(jtIdeal, jtMax));
                Vec3 impulseT = tdir * jt;
                linVel += impulseT * invMass;
                omega += r.cross(impulseT) * invInertia;
            }
        }

        if (cp.penetration > kContactSlop) {
            correctionNormal += cp.normal * (cp.penetration - kContactSlop);
        }
    }

    if (result.contactCount > 0) {
        if (correctionNormal.lengthSq() > 1e-10) {
            Vec3 correction = correctionNormal / (double)result.contactCount;
            correction *= 0.85;
            double maxCorrection = std::max(0.05, kMaxDepenetrationSpeed * std::max(0.001, dt));
            double correctionLen = correction.length();
            if (correctionLen > maxCorrection) {
                correction = correction * (maxCorrection / correctionLen);
            }
            trans.px += correction.x;
            trans.py += correction.y;
            trans.pz += correction.z;
        }

        Vec3 normal = Vec3(trans.px, trans.py, trans.pz).normalized();
        applyPlasticCrush(linVel, omega, contacts.front().groundVelocity, normal, result.maxClosingSpeed);

        vel.vx = linVel.x;
        vel.vy = linVel.y;
        vel.vz = linVel.z;
        Vec3 deltaOmega = omega - angularVelocityWorld(att);
        addAngularVelocityWorld(att, deltaOmega);
    }
    return result;
}

ContactResult ResolveChunkGroundContacts(TransformComponent& trans,
                                         VelocityComponent& vel,
                                         AttitudeComponent& att,
                                         const VoxelBodyComponent& voxel,
                                         const RigidChunkComponent& chunk,
                                         const CelestialBody& body,
                                         const TelemetryComponent& tele,
                                         double simTime,
                                         double dt,
                                         const Vec3& previousCom) {
    sanitizeAttitude(att);
    ContactResult result;
    if (!voxel.model || chunk.mass <= 0.0 || dt <= 0.0) return result;

    std::vector<Vec3> samples = BuildVoxelContactSamples(voxel, chunk, 64);
    if (samples.empty()) return result;

    double mass = std::max(1.0, chunk.mass);
    double inertia = chunkInertiaScalar(chunk);
    applyChunkTerrainTOI(trans, vel, att, body, tele, mass, inertia, simTime, dt, previousCom, samples, result);

    RocketConfig pseudoConfig{};
    pseudoConfig.bounds_bottom = 0.0;
    PropulsionComponent pseudoProp{};
    std::vector<ContactPoint> contacts = buildContactManifold(
        pseudoConfig, pseudoProp, trans, vel, att, tele, body, simTime, samples);
    ContactResult solved = solveChunkGroundContacts(contacts, trans, vel, att, mass, inertia, dt);
    result.contactCount = std::max(result.contactCount, solved.contactCount);
    result.maxPenetration = std::max(result.maxPenetration, solved.maxPenetration);
    result.maxClosingSpeed = std::max(result.maxClosingSpeed, solved.maxClosingSpeed);
    result.maxTangentSpeed = std::max(result.maxTangentSpeed, solved.maxTangentSpeed);
    result.hardImpact = result.hardImpact || solved.hardImpact;
    return result;
}

ContactResult ResolveGroundContacts(const RocketConfig& config,
                                    const PropulsionComponent& prop,
                                    TransformComponent& trans,
                                    VelocityComponent& vel,
                                    AttitudeComponent& att,
                                    GuidanceComponent& guid,
                                    const TelemetryComponent& tele,
                                    const CelestialBody& body,
                                    double totalMass,
                                    double simTime,
                                    double dt,
                                    const Vec3& previousCom,
                                    const VoxelBodyComponent* fracturedVoxel,
                                    const RigidChunkComponent* fracturedChunk,
                                    const StructuralStateComponent* structural) {
    sanitizeAttitude(att);
    std::vector<Vec3> contactSamples = ResolveStructureContactSamples(
        config, prop, fracturedVoxel, fracturedChunk, structural);

    ContactResult sweepResult;
    applyHighSpeedTerrainTOI(config, prop, trans, vel, att, guid, tele, body, totalMass,
                             simTime, dt, previousCom, contactSamples, sweepResult);

    std::vector<ContactPoint> contacts = buildContactManifold(
        config, prop, trans, vel, att, tele, body, simTime, contactSamples);
    ContactResult result = solveGroundContacts(contacts, trans, vel, att, guid, config, totalMass, dt);
    if (sweepResult.hardImpact) {
        result.hardImpact = true;
        result.maxClosingSpeed = std::max(result.maxClosingSpeed, sweepResult.maxClosingSpeed);
        if (sweepResult.hasImpactLocalPoint) {
            result.hasImpactLocalPoint = true;
            result.impactLocalPoint = sweepResult.impactLocalPoint;
        }
        if (contacts.empty()) {
            result.contactCount = std::max(1, result.contactCount);
        }
        if (guid.mission_msg.empty() || guid.mission_msg.find("HARD IMPACT") == std::string::npos) {
            guid.mission_msg = "HARD IMPACT: TERRAIN CONTACT";
        }
    }
    return result;
}

} // namespace GroundCollisionSystem
