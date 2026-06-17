#include "chunk_body_collision.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ChunkBodyCollision {
namespace {

constexpr double kSlop = 0.04;
constexpr double kRestitution = 0.12;
constexpr double kFriction = 0.35;
constexpr double kMinMass = 1.0;

struct Body {
    entt::entity entity = entt::null;
    TransformComponent* trans = nullptr;
    VelocityComponent* vel = nullptr;
    AttitudeComponent* att = nullptr;
    RigidChunkComponent* chunk = nullptr;
    Vec3 center;
    Vec3 aabbMin;
    Vec3 aabbMax;
    double mass = 1.0;
    double invMass = 1.0;
};


static void addAngularVelocityWorld(AttitudeComponent& att, const Vec3& deltaWorld) {
    Vec3 local = att.attitude.conjugate().rotate(deltaWorld);
    att.ang_vel_z += local.x;
    att.ang_vel_roll += local.y;
    att.ang_vel += local.z;
}

static bool finiteVec(const Vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

static void computeWorldAabb(Body& body) {
    body.center = Vec3(body.trans->px, body.trans->py, body.trans->pz);
    Vec3 mn = body.chunk->local_bounds_min - body.chunk->local_center_of_mass;
    Vec3 mx = body.chunk->local_bounds_max - body.chunk->local_center_of_mass;
    if (mx.x < mn.x || mx.y < mn.y || mx.z < mn.z) {
        double r = 0.5;
        body.aabbMin = body.center - Vec3(r, r, r);
        body.aabbMax = body.center + Vec3(r, r, r);
        return;
    }

    body.aabbMin = Vec3(1e30, 1e30, 1e30);
    body.aabbMax = Vec3(-1e30, -1e30, -1e30);
    const Quat& q = body.att->attitude;
    for (int ix = 0; ix <= 1; ix++) {
        for (int iy = 0; iy <= 1; iy++) {
            for (int iz = 0; iz <= 1; iz++) {
                Vec3 local(mn.x + (mx.x - mn.x) * ix,
                           mn.y + (mx.y - mn.y) * iy,
                           mn.z + (mx.z - mn.z) * iz);
                Vec3 world = body.center + q.rotate(local);
                body.aabbMin.x = std::min(body.aabbMin.x, world.x);
                body.aabbMin.y = std::min(body.aabbMin.y, world.y);
                body.aabbMin.z = std::min(body.aabbMin.z, world.z);
                body.aabbMax.x = std::max(body.aabbMax.x, world.x);
                body.aabbMax.y = std::max(body.aabbMax.y, world.y);
                body.aabbMax.z = std::max(body.aabbMax.z, world.z);
            }
        }
    }
}

static bool buildBody(entt::entity e,
                      entt::registry& registry,
                      Body& out) {
    if (!registry.all_of<RigidChunkComponent, TransformComponent, VelocityComponent, AttitudeComponent>(e)) {
        return false;
    }
    auto& chunk = registry.get<RigidChunkComponent>(e);
    if (chunk.mass <= 0.0) return false;

    out.entity = e;
    out.trans = &registry.get<TransformComponent>(e);
    out.vel = &registry.get<VelocityComponent>(e);
    out.att = &registry.get<AttitudeComponent>(e);
    out.chunk = &chunk;
    out.mass = std::max(kMinMass, chunk.mass);
    out.invMass = 1.0 / out.mass;
    computeWorldAabb(out);
    return true;
}

static bool aabbOverlap(const Body& a, const Body& b, Vec3& normal, double& penetration) {
    double overlapX = std::min(a.aabbMax.x, b.aabbMax.x) - std::max(a.aabbMin.x, b.aabbMin.x);
    double overlapY = std::min(a.aabbMax.y, b.aabbMax.y) - std::max(a.aabbMin.y, b.aabbMin.y);
    double overlapZ = std::min(a.aabbMax.z, b.aabbMax.z) - std::max(a.aabbMin.z, b.aabbMin.z);
    if (overlapX <= 0.0 || overlapY <= 0.0 || overlapZ <= 0.0) return false;

    penetration = overlapX;
    normal = Vec3((a.center.x >= b.center.x) ? 1.0 : -1.0, 0.0, 0.0);
    if (overlapY < penetration) {
        penetration = overlapY;
        normal = Vec3(0.0, (a.center.y >= b.center.y) ? 1.0 : -1.0, 0.0);
    }
    if (overlapZ < penetration) {
        penetration = overlapZ;
        normal = Vec3(0.0, 0.0, (a.center.z >= b.center.z) ? 1.0 : -1.0);
    }
    return penetration > 0.0;
}

static void resolvePair(Body& a, Body& b, double dt) {
    Vec3 normal;
    double penetration = 0.0;
    if (!aabbOverlap(a, b, normal, penetration)) return;

    double invSum = a.invMass + b.invMass;
    if (invSum <= 1e-12) return;

    double correction = std::max(0.0, penetration - kSlop);
    if (correction > 0.0) {
        double maxPush = std::max(0.25, 20.0 * std::max(0.001, dt));
        correction = std::min(correction, maxPush);
        Vec3 corr = normal * correction;
        a.trans->px += corr.x * (a.invMass / invSum);
        a.trans->py += corr.y * (a.invMass / invSum);
        a.trans->pz += corr.z * (a.invMass / invSum);
        b.trans->px -= corr.x * (b.invMass / invSum);
        b.trans->py -= corr.y * (b.invMass / invSum);
        b.trans->pz -= corr.z * (b.invMass / invSum);
        a.center = Vec3(a.trans->px, a.trans->py, a.trans->pz);
        b.center = Vec3(b.trans->px, b.trans->py, b.trans->pz);
        computeWorldAabb(a);
        computeWorldAabb(b);
    }

    Vec3 va(a.vel->vx, a.vel->vy, a.vel->vz);
    Vec3 vb(b.vel->vx, b.vel->vy, b.vel->vz);
    Vec3 relVel = va - vb;
    double vn = relVel.dot(normal);
    if (vn >= 0.0) return;

    double jn = -(1.0 + kRestitution) * vn / invSum;
    Vec3 impulseN = normal * jn;
    va += impulseN * a.invMass;
    vb -= impulseN * b.invMass;

    Vec3 tangent = relVel - normal * vn;
    double tangentLen = tangent.length();
    if (tangentLen > 1e-5) {
        Vec3 tdir = tangent / tangentLen;
        double jt = std::max(-kFriction * jn, std::min(-tangentLen / invSum, kFriction * jn));
        va += tdir * (jt * a.invMass);
        vb -= tdir * (jt * b.invMass);
    }

    a.vel->vx = va.x;
    a.vel->vy = va.y;
    a.vel->vz = va.z;
    b.vel->vx = vb.x;
    b.vel->vy = vb.y;
    b.vel->vz = vb.z;

    if (finiteVec(impulseN)) {
        Vec3 torqueA = (b.center - a.center).cross(impulseN * 0.5);
        addAngularVelocityWorld(*a.att, torqueA * (a.invMass * 0.12));
        addAngularVelocityWorld(*b.att, torqueA * (-b.invMass * 0.12));
    }
}

} // namespace

void resolveRegistryPairs(entt::registry& registry, double dt, int iterations) {
    if (dt <= 0.0 || iterations <= 0) return;

    std::vector<Body> bodies;
    auto view = registry.view<RigidChunkComponent, TransformComponent, VelocityComponent, AttitudeComponent>(
        entt::exclude<PendingDestroy>);
    for (auto e : view) {
        Body body;
        if (buildBody(e, registry, body)) {
            bodies.push_back(body);
        }
    }
    if (bodies.size() < 2) return;

    iterations = std::clamp(iterations, 1, 4);
    for (int iter = 0; iter < iterations; iter++) {
        for (size_t i = 0; i < bodies.size(); i++) {
            for (size_t j = i + 1; j < bodies.size(); j++) {
                resolvePair(bodies[i], bodies[j], dt);
            }
        }
    }
}

} // namespace ChunkBodyCollision
