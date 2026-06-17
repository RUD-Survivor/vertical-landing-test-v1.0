#include "vessel_voxelizer.h"
#include "simulation/rocket_builder.h"
#include <algorithm>
#include <cmath>

namespace VoxelPhysics {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;

static Vec3 symmetricPos(const PlacedPart& part, int s) {
    Vec3 localPos = part.pos;
    int symmetry = std::max(1, part.symmetry);
    if (symmetry > 1) {
        double dist = std::sqrt(part.pos.x * part.pos.x + part.pos.z * part.pos.z);
        if (dist > 0.01) {
            double ca = std::atan2(part.pos.z, part.pos.x);
            double symAngle = s * kTwoPi / symmetry;
            localPos.x = std::cos(ca + symAngle) * dist;
            localPos.z = std::sin(ca + symAngle) * dist;
        }
    }
    return localPos;
}

static Quat symmetricRot(const PlacedPart& part, int s) {
    int symmetry = std::max(1, part.symmetry);
    double symAngle = symmetry > 1 ? s * kTwoPi / symmetry : 0.0;
    return part.rot * Quat::fromAxisAngle(Vec3(0, 1, 0), symAngle);
}

static double radiusAt(const PartDef& def, double localY) {
    double h = std::max(0.01f, def.height);
    double t = std::max(0.0, std::min(1.0, localY / h));
    double r = std::max(0.05f, def.diameter * 0.5f);
    if (def.category == CAT_NOSE_CONE) return std::max(0.05 * r, r * (1.0 - 0.92 * t));
    if (def.category == CAT_ENGINE) return r * (0.75 + 0.25 * t);
    if (def.category == CAT_STRUCTURAL && def.id == 13) return r * 0.65;
    if (def.category == CAT_STRUCTURAL && def.id == 17) return r * 0.25;
    return r;
}

static bool pointInsidePart(const PartDef& def, const Vec3& partLocal) {
    if (partLocal.y < 0.0 || partLocal.y > def.height) return false;
    double r = radiusAt(def, partLocal.y);
    if (def.category == CAT_STRUCTURAL && def.id == 17) {
        return std::abs(partLocal.x) <= def.diameter * 0.5 &&
               std::abs(partLocal.z) <= std::max(0.08, def.height * 0.04);
    }
    double radialSq = partLocal.x * partLocal.x + partLocal.z * partLocal.z;
    return radialSq <= r * r;
}

static void partBounds(const PlacedPart& part,
                       const PartDef& def,
                       int symIndex,
                       Vec3& outMin,
                       Vec3& outMax) {
    Vec3 pos = symmetricPos(part, symIndex);
    Quat rot = symmetricRot(part, symIndex);
    double r = std::max(0.05f, def.diameter * 0.5f);
    outMin = Vec3(1e30, 1e30, 1e30);
    outMax = Vec3(-1e30, -1e30, -1e30);
    for (int ix = -1; ix <= 1; ix += 2) {
        for (int iy = 0; iy <= 1; iy++) {
            for (int iz = -1; iz <= 1; iz += 2) {
                Vec3 local(ix * r, iy * (double)def.height, iz * r);
                Vec3 world = pos + rot.rotate(local);
                outMin.x = std::min(outMin.x, world.x);
                outMin.y = std::min(outMin.y, world.y);
                outMin.z = std::min(outMin.z, world.z);
                outMax.x = std::max(outMax.x, world.x);
                outMax.y = std::max(outMax.y, world.y);
                outMax.z = std::max(outMax.z, world.z);
            }
        }
    }
}

} // namespace

VesselVoxelModelPtr VoxelizeAssembly(const RocketAssembly& assembly,
                                     const VoxelizationSettings& settings) {
    auto model = std::make_shared<VesselVoxelModel>(settings.voxel_size);
    if (assembly.parts.empty()) return model;

    int start = std::max(0, settings.part_start_index);
    int end = settings.part_end_index < 0
        ? (int)assembly.parts.size()
        : std::min(settings.part_end_index, (int)assembly.parts.size());
    if (start >= end) return model;

    for (int pi = start; pi < end; pi++) {
        const PlacedPart& part = assembly.parts[(size_t)pi];
        if (part.def_id < 0 || part.def_id >= PART_CATALOG_SIZE) continue;
        const PartDef& def = PART_CATALOG[part.def_id];
        int symmetry = std::max(1, part.symmetry);
        double partCount = (double)symmetry;
        double partMass = std::max(0.1f, def.dry_mass + def.fuel_capacity) / partCount;
        VoxelMaterial material = MaterialFromPartCategory((int)def.category,
                                                          def.fuel_capacity > 0.0f,
                                                          def.thrust > 0.0f);

        for (int s = 0; s < symmetry; s++) {
            Vec3 pos = symmetricPos(part, s);
            Quat rot = symmetricRot(part, s);
            Quat invRot = rot.conjugate();
            Vec3 bmin, bmax;
            partBounds(part, def, s, bmin, bmax);
            double pad = settings.voxel_size;
            VoxelCoord cmin = model->localToCoord(bmin - Vec3(pad, pad, pad));
            VoxelCoord cmax = model->localToCoord(bmax + Vec3(pad, pad, pad));

            std::vector<VoxelCoord> occupied;
            for (int z = cmin.z; z <= cmax.z; z++) {
                for (int y = cmin.y; y <= cmax.y; y++) {
                    for (int x = cmin.x; x <= cmax.x; x++) {
                        Vec3 center = model->coordToCenter({ x, y, z });
                        Vec3 partLocal = invRot.rotate(center - pos);
                        if (pointInsidePart(def, partLocal)) occupied.push_back({ x, y, z });
                    }
                }
            }

            if (occupied.empty()) continue;
            double massPerCell = partMass / (double)occupied.size();
            for (const VoxelCoord& coord : occupied) {
                model->addCell(coord, material, pi, massPerCell);
                if ((int)model->getCells().size() > settings.max_cells) {
                    model->rebuildDerivedData();
                    return model;
                }
            }
        }
    }

    model->rebuildDerivedData();
    return model;
}

} // namespace VoxelPhysics
