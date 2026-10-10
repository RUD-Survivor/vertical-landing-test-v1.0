#ifndef WATER_SURFACE_GLSL
#define WATER_SURFACE_GLSL
// Height-map values are dimensionless; geometry is km, optical depth is meters.
// Keep the bed BEFORE any flood fill. Lakes use filledHeight; oceans use sea level.
const float WATER_SEA_LEVEL = 0.45;
float waterBedKm(float h, float maxElevationKm) {
    return h < WATER_SEA_LEVEL
        ? (h - WATER_SEA_LEVEL) / WATER_SEA_LEVEL * 6.0
        : (h - WATER_SEA_LEVEL) / (1.0 - WATER_SEA_LEVEL) * maxElevationKm;
}
float waterLevelKm(float bed, float filled, float maxElevationKm) {
    return bed < WATER_SEA_LEVEL ? 0.0
        : waterBedKm(max(bed, filled), maxElevationKm);
}
float waterDepthMeters(float bed, float filled, float maxElevationKm) {
    return max(0.0, waterLevelKm(bed, filled, maxElevationKm)
                    - waterBedKm(bed, maxElevationKm)) * 1000.0;
}
#endif
