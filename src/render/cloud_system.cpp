#include "cloud_system.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>

// ─── Shared CPU noise helpers ─────────────────────────────────────────────────
// All functions are pure (no global state); declared static to keep them TU-local.

static float cpu_fract(float x) { return x - floorf(x); }

static float cpu_hash(float px, float py, float pz) {
    px = cpu_fract(px * 443.897f);
    py = cpu_fract(py * 441.423f);
    pz = cpu_fract(pz * 437.195f);
    float d = px*(py+19.19f) + py*(pz+19.19f) + pz*(px+19.19f);
    px += d; py += d; pz += d;
    return cpu_fract((px + py) * pz);
}

static float cpu_noise3d(float px, float py, float pz) {
    float ix=floorf(px), iy=floorf(py), iz=floorf(pz);
    float fx=px-ix, fy=py-iy, fz=pz-iz;
    fx = fx*fx*fx*(fx*(fx*6.0f-15.0f)+10.0f);
    fy = fy*fy*fy*(fy*(fy*6.0f-15.0f)+10.0f);
    fz = fz*fz*fz*(fz*(fz*6.0f-15.0f)+10.0f);
    float n000=cpu_hash(ix,  iy,  iz  ), n100=cpu_hash(ix+1,iy,  iz  );
    float n010=cpu_hash(ix,  iy+1,iz  ), n110=cpu_hash(ix+1,iy+1,iz  );
    float n001=cpu_hash(ix,  iy,  iz+1), n101=cpu_hash(ix+1,iy,  iz+1);
    float n011=cpu_hash(ix,  iy+1,iz+1), n111=cpu_hash(ix+1,iy+1,iz+1);
    float nx00=n000+fx*(n100-n000), nx10=n010+fx*(n110-n010);
    float nx01=n001+fx*(n101-n001), nx11=n011+fx*(n111-n011);
    float nxy0=nx00+fy*(nx10-nx00), nxy1=nx01+fy*(nx11-nx01);
    return nxy0+fz*(nxy1-nxy0);
}

static float cpu_fbm(float px, float py, float pz, int octaves) {
    float v=0.0f, amp=0.5f;
    for (int i=0; i<octaves; i++) {
        v += cpu_noise3d(px, py, pz) * amp;
        px=px*2.07f+0.131f; py=py*2.07f-0.217f; pz=pz*2.07f+0.344f;
        amp *= 0.48f;
    }
    return v;
}

// Non-tileable Worley (kept for bakeDetail where tileability is not required).
static float cpu_worley(float px, float py, float pz) {
    float ix=floorf(px), iy=floorf(py), iz=floorf(pz);
    float fx=px-ix, fy=py-iy, fz=pz-iz;
    float md = 1.0f;
    for (int x=-1; x<=1; x++) for (int y=-1; y<=1; y++) for (int z=-1; z<=1; z++) {
        float bx=ix+x, by=iy+y, bz=iz+z;
        float hx=cpu_fract(sinf(bx*127.1f+by*311.7f+bz*74.7f)*43758.5453f);
        float hy=cpu_fract(sinf(bx*269.5f+by*183.3f+bz*246.1f)*43758.5453f);
        float hz=cpu_fract(sinf(bx*113.5f+by*271.9f+bz*124.6f)*43758.5453f);
        float dx=fx-(x+hx), dy=fy-(y+hy), dz=fz-(z+hz);
        float dd=dx*dx+dy*dy+dz*dz;
        if (dd < md) md = dd;
    }
    return sqrtf(md);
}

// Tileable Worley with period T=8 — C0-continuous at UV integer boundaries,
// so GL_REPEAT sampling produces no seam cracks.
static float cpu_worley_tile(float px, float py, float pz) {
    const float T = 8.0f;
    float wpx = px - floorf(px/T)*T;
    float wpy = py - floorf(py/T)*T;
    float wpz = pz - floorf(pz/T)*T;
    float ix=floorf(wpx), iy=floorf(wpy), iz=floorf(wpz);
    float fx=wpx-ix, fy=wpy-iy, fz=wpz-iz;
    float md = 1.0f;
    for (int x=-1; x<=1; x++) for (int y=-1; y<=1; y++) for (int z=-1; z<=1; z++) {
        float bx=ix+x, by=iy+y, bz=iz+z;
        float wbx=bx-floorf(bx/T)*T;
        float wby=by-floorf(by/T)*T;
        float wbz=bz-floorf(bz/T)*T;
        float hx=cpu_fract(sinf(wbx*127.1f+wby*311.7f+wbz*74.7f)*43758.5453f);
        float hy=cpu_fract(sinf(wbx*269.5f+wby*183.3f+wbz*246.1f)*43758.5453f);
        float hz=cpu_fract(sinf(wbx*113.5f+wby*271.9f+wbz*124.6f)*43758.5453f);
        float dx=fx-((float)x+hx), dy=fy-((float)y+hy), dz=fz-((float)z+hz);
        float dd=dx*dx+dy*dy+dz*dz;
        if (dd < md) md = dd;
    }
    return sqrtf(md);
}

static uint8_t clamp01_u8(float v) {
    int i = (int)(v * 255.0f + 0.5f);
    return (uint8_t)(i < 0 ? 0 : i > 255 ? 255 : i);
}

// ─── CloudSystem public API ───────────────────────────────────────────────────

bool CloudSystem::loadGLSL(const char* path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "[CloudSystem] Cannot open GLSL file: " << path << std::endl;
        return false;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    m_glsl = ss.str();
    std::cout << "[CloudSystem] Loaded cloud.glsl (" << m_glsl.size() << " bytes)" << std::endl;
    return true;
}

void CloudSystem::init(GLuint prog) {
    ua_time       = glGetUniformLocation(prog, "uCloudTime");
    ua_phaseSin   = glGetUniformLocation(prog, "uCloudPhaseSin");
    ua_phaseCos   = glGetUniformLocation(prog, "uCloudPhaseCos");
    ua_frameIdx   = glGetUniformLocation(prog, "uFrameIndex");
    ua_cSteps     = glGetUniformLocation(prog, "uCloudSteps");
    ua_lSteps     = glGetUniformLocation(prog, "uLightSteps");
    ua_covLo      = glGetUniformLocation(prog, "uTuneCovLo");
    ua_covHi      = glGetUniformLocation(prog, "uTuneCovHi");
    ua_threshLo   = glGetUniformLocation(prog, "uTuneThreshLo");
    ua_threshHi   = glGetUniformLocation(prog, "uTuneThreshHi");
    ua_erosion    = glGetUniformLocation(prog, "uTuneErosion");
    ua_density    = glGetUniformLocation(prog, "uTuneDensity");
    ua_extinction = glGetUniformLocation(prog, "uTuneExtinction");
    ua_minAlt     = glGetUniformLocation(prog, "uTuneMinAlt");
    ua_maxAlt     = glGetUniformLocation(prog, "uTuneMaxAlt");
    ua_debug      = glGetUniformLocation(prog, "uDebugCoverage");
    ua_noiseT     = glGetUniformLocation(prog, "uNoiseTex3D");
    ua_coverT     = glGetUniformLocation(prog, "uCoverTex3D");
    ua_detailT    = glGetUniformLocation(prog, "uDetailTex3D");
    ua_weatherT   = glGetUniformLocation(prog, "uWeatherMap");

    bakeNoise();
    bakeCover();
    bakeDetail();
    bakeWeather();
}

void CloudSystem::destroy() {
    if (noiseT)   { glDeleteTextures(1, &noiseT);   noiseT   = 0; }
    if (coverT)   { glDeleteTextures(1, &coverT);   coverT   = 0; }
    if (detailT)  { glDeleteTextures(1, &detailT);  detailT  = 0; }
    if (weatherT) { glDeleteTextures(1, &weatherT); weatherT = 0; }
}

void CloudSystem::bind(GLuint prog, double time, int frameIdx, float camAltKm) {
    (void)prog; // uniform locations already cached in init()

    // Time animation
    float wrappedTime = (float)std::fmod(time, 10000.0);
    glUniform1f(ua_time, wrappedTime);
    double phase = time * 0.004 * -0.2;
    glUniform1f(ua_phaseSin, (float)std::sin(phase));
    glUniform1f(ua_phaseCos, (float)std::cos(phase));
    glUniform1i(ua_frameIdx, frameIdx);

    // Tune params
    glUniform1f(ua_covLo,      tuneParams.covLo);
    glUniform1f(ua_covHi,      tuneParams.covHi);
    glUniform1f(ua_threshLo,   tuneParams.threshLo);
    glUniform1f(ua_threshHi,   tuneParams.threshHi);
    glUniform1f(ua_erosion,    tuneParams.erosion);
    glUniform1f(ua_density,    tuneParams.density);
    glUniform1f(ua_extinction, tuneParams.extinction);
    glUniform1f(ua_minAlt,     tuneParams.minAlt);
    glUniform1f(ua_maxAlt,     tuneParams.maxAlt);
    glUniform1f(ua_debug,      (float)tuneParams.debugMode);

    // Distance-based LOD step counts
    int cSteps, lSteps;
    if      (camAltKm < 20.f)   { cSteps = 128; lSteps = 16; }
    else if (camAltKm < 500.f)  { cSteps = 48;  lSteps = 12; }
    else if (camAltKm < 5000.f) { cSteps = 32;  lSteps =  8; }
    else                        { cSteps = 16;  lSteps =  4; }
    glUniform1i(ua_cSteps, cSteps);
    glUniform1i(ua_lSteps, lSteps);

    // Bind textures to fixed slots 8–11
    glActiveTexture(GL_TEXTURE8);
    glBindTexture(GL_TEXTURE_3D, noiseT);
    glUniform1i(ua_noiseT, 8);

    glActiveTexture(GL_TEXTURE9);
    glBindTexture(GL_TEXTURE_3D, coverT);
    glUniform1i(ua_coverT, 9);

    glActiveTexture(GL_TEXTURE10);
    glBindTexture(GL_TEXTURE_3D, detailT);
    glUniform1i(ua_detailT, 10);

    glActiveTexture(GL_TEXTURE11);
    glBindTexture(GL_TEXTURE_2D, weatherT);
    glUniform1i(ua_weatherT, 11);
}

// ─── Bake: 128³ noise texture ─────────────────────────────────────────────────
// R=PerlinWorley (probe+base shape)  G=Worley (erosion)
// B=fbm(3oct) (domain warp)          A=fbm(5oct) (coverage large-scale)
void CloudSystem::bakeNoise() {
    const int N = 128;
    std::vector<uint8_t> data(N*N*N*4);
    std::cout << "[Cloud] Baking noise texture (" << N << "^3)..." << std::flush;
    for (int z=0; z<N; z++)
    for (int y=0; y<N; y++)
    for (int x=0; x<N; x++) {
        float s = (float)x/N*8, t8 = (float)y/N*8, u = (float)z/N*8;
        float perlin = cpu_fbm(s, t8, u, 4);
        float w_inv  = 1.0f - cpu_worley_tile(s, t8, u);
        float r = w_inv + perlin * (1.0f - w_inv);  // Perlin-Worley screen blend
        float g = cpu_worley_tile(s, t8, u);         // tileable, no seam cracks
        float b = cpu_fbm(s, t8, u, 3);
        float a = cpu_fbm(s, t8, u, 5);
        int idx = (z*N*N + y*N + x)*4;
        data[idx+0]=clamp01_u8(r); data[idx+1]=clamp01_u8(g);
        data[idx+2]=clamp01_u8(b); data[idx+3]=clamp01_u8(a);
    }
    if (noiseT) glDeleteTextures(1, &noiseT);
    glGenTextures(1, &noiseT);
    glBindTexture(GL_TEXTURE_3D, noiseT);
    glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA8, N, N, N, 0, GL_RGBA, GL_UNSIGNED_BYTE, data.data());
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_REPEAT);
    glBindTexture(GL_TEXTURE_3D, 0);
    std::cout << " done." << std::endl;
}

// ─── Bake: 128³ coverage texture ─────────────────────────────────────────────
// R=curl_lo(2oct)  G=curl_hi(1oct)  B=latNoise(3oct)  A=cov_med(3oct)
void CloudSystem::bakeCover() {
    const int N = 128;
    std::vector<uint8_t> data(N*N*N*4);
    std::cout << "[Cloud] Baking coverage noise texture (" << N << "^3)..." << std::flush;
    for (int z=0; z<N; z++)
    for (int y=0; y<N; y++)
    for (int x=0; x<N; x++) {
        float px=(float)x/N, py=(float)y/N, pz=(float)z/N;
        float r = cpu_fbm(px*8,       py*8, pz*8, 2);  // curl lo
        float g = cpu_fbm(px*8,       py*8, pz*8, 1);  // curl hi
        float b = cpu_fbm(px*8+12.3f, py*8, pz*8, 3);  // latNoise (independent seed)
        float a = cpu_fbm(px*8+47.1f, py*8, pz*8, 3);  // cov_medium (independent seed)
        int idx=(z*N*N+y*N+x)*4;
        data[idx+0]=clamp01_u8(r); data[idx+1]=clamp01_u8(g);
        data[idx+2]=clamp01_u8(b); data[idx+3]=clamp01_u8(a);
    }
    if (coverT) glDeleteTextures(1, &coverT);
    glGenTextures(1, &coverT);
    glBindTexture(GL_TEXTURE_3D, coverT);
    glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA8, N, N, N, 0, GL_RGBA, GL_UNSIGNED_BYTE, data.data());
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_REPEAT);
    glBindTexture(GL_TEXTURE_3D, 0);
    std::cout << " done." << std::endl;
}

// ─── Bake: 32³ detail erosion texture ────────────────────────────────────────
// R/G/B = Worley at 50m/25m/12.5m cells (200m world tile).
// A = weighted blend used for combined surface erosion.
void CloudSystem::bakeDetail() {
    const int N = 32;
    std::vector<uint8_t> data(N*N*N*4);
    std::cout << "[Cloud] Baking detail noise texture (" << N << "^3, multi-freq Worley)..." << std::flush;
    for (int z=0; z<N; z++)
    for (int y=0; y<N; y++)
    for (int x=0; x<N; x++) {
        float px=(float)x/N, py=(float)y/N, pz=(float)z/N;
        float r = cpu_worley(px*4.0f  + 0.13f, py*4.0f  + 0.47f, pz*4.0f  + 0.31f);
        float g = cpu_worley(px*8.0f  + 7.31f, py*8.0f  + 5.17f, pz*8.0f  + 2.89f);
        float b = cpu_worley(px*16.0f + 11.8f, py*16.0f + 9.44f, pz*16.0f + 14.3f);
        float a = r * 0.625f + g * 0.25f + b * 0.125f;
        int idx=(z*N*N+y*N+x)*4;
        data[idx+0]=clamp01_u8(r); data[idx+1]=clamp01_u8(g);
        data[idx+2]=clamp01_u8(b); data[idx+3]=clamp01_u8(a);
    }
    if (detailT) glDeleteTextures(1, &detailT);
    glGenTextures(1, &detailT);
    glBindTexture(GL_TEXTURE_3D, detailT);
    glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA8, N, N, N, 0, GL_RGBA, GL_UNSIGNED_BYTE, data.data());
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_REPEAT);
    glBindTexture(GL_TEXTURE_3D, 0);
    std::cout << " done." << std::endl;
}

// ─── Bake: 2048×1024 weather map ─────────────────────────────────────────────
// Equirectangular (lon/lat) climate texture for Earth (planetIdx==3).
// R=coverage_bias  G=cloud_type  B=humidity  A=land_mask
void CloudSystem::bakeWeather() {
    std::cout << "[Cloud] Baking weather map (2048x1024)..." << std::flush;
    const int W = 2048, H = 1024;
    std::vector<uint8_t> data(W * H * 4);

    // Smooth ellipse membership [0,1] centered at (cLon,cLat) degrees.
    auto ellipse = [](float lon, float lat, float cLon, float cLat, float hW, float hH) -> float {
        float dlon = lon - cLon;
        if (dlon >  180.f) dlon -= 360.f;
        if (dlon < -180.f) dlon += 360.f;
        float dlat = lat - cLat;
        float r = std::sqrt((dlon/hW)*(dlon/hW) + (dlat/hH)*(dlat/hH));
        if (r >= 1.3f) return 0.f;
        if (r <= 0.7f) return 1.f;
        float tt = (r - 0.7f) / 0.6f;
        return 1.f - tt * tt * (3.f - 2.f * tt);
    };
    auto gauss = [](float x, float center, float sigma) -> float {
        float d = (x - center) / sigma;
        return std::exp(-0.5f * d * d);
    };

    for (int y = 0; y < H; y++) {
        float lat = ((float)y / (float)H - 0.5f) * 180.f;
        for (int x = 0; x < W; x++) {
            float lon = ((float)x / (float)W - 0.5f) * 360.f;

            // Continental land mask
            float land = 0.f;
            land = std::fmax(land, ellipse(lon, lat, -98.f,  52.f, 42.f, 30.f)); // N. America
            land = std::fmax(land, ellipse(lon, lat, -90.f,  18.f, 13.f, 12.f)); // Cent. America
            land = std::fmax(land, ellipse(lon, lat, -60.f, -14.f, 22.f, 27.f)); // S. America
            land = std::fmax(land, ellipse(lon, lat,  12.f,  52.f, 22.f, 18.f)); // Europe
            land = std::fmax(land, ellipse(lon, lat,  17.f,  20.f, 38.f, 22.f)); // N. Africa
            land = std::fmax(land, ellipse(lon, lat,  22.f, -10.f, 28.f, 26.f)); // Sub-Saharan Africa
            land = std::fmax(land, ellipse(lon, lat,  45.f,  23.f, 18.f, 13.f)); // Arabian Peninsula
            land = std::fmax(land, ellipse(lon, lat,  73.f,  28.f, 18.f, 14.f)); // South Asia
            land = std::fmax(land, ellipse(lon, lat, 108.f,  36.f, 30.f, 28.f)); // East Asia
            land = std::fmax(land, ellipse(lon, lat,  80.f,  62.f, 65.f, 17.f)); // Siberia/Russia
            land = std::fmax(land, ellipse(lon, lat, 102.f,  15.f, 15.f, 12.f)); // SE Asia
            land = std::fmax(land, ellipse(lon, lat, 118.f,  -3.f, 20.f,  7.f)); // Indonesia
            land = std::fmax(land, ellipse(lon, lat, 134.f, -26.f, 22.f, 16.f)); // Australia
            land = std::fmax(land, ellipse(lon, lat, -42.f,  72.f, 22.f, 12.f)); // Greenland
            { float tt = std::min(1.f, std::max(0.f, (-lat - 70.f) / 14.f));
              land = std::fmax(land, tt * tt * (3.f - 2.f * tt)); } // Antarctica
            land = std::min(land, 1.f);

            // Desert aridity mask
            float desert = 0.f;
            desert = std::fmax(desert, ellipse(lon, lat,  17.f,  24.f, 42.f, 14.f) * land); // Sahara
            desert = std::fmax(desert, ellipse(lon, lat,  47.f,  22.f, 18.f, 10.f) * land); // Arabia
            desert = std::fmax(desert, ellipse(lon, lat, 134.f, -27.f, 18.f, 12.f) * land); // C. Australia
            desert = std::fmax(desert, ellipse(lon, lat, -70.f, -22.f,  6.f, 14.f) * land); // Atacama
            desert = std::fmax(desert, ellipse(lon, lat,  62.f,  38.f, 20.f,  9.f) * land); // C. Asia
            desert = std::fmax(desert, ellipse(lon, lat,-113.f,  33.f, 10.f,  6.f) * land); // US SW

            // Tropical rainforest mask
            float rainforest = 0.f;
            rainforest = std::fmax(rainforest, ellipse(lon, lat, -60.f, -4.f, 20.f, 10.f) * land); // Amazon
            rainforest = std::fmax(rainforest, ellipse(lon, lat,  23.f, -1.f, 10.f,  8.f) * land); // Congo
            rainforest = std::fmax(rainforest, ellipse(lon, lat, 110.f,  2.f, 18.f,  8.f) * land); // SE Asia rain

            // Climate zone weights by absolute latitude
            float absLat = std::abs(lat);
            float wITCZ      = gauss(absLat,  2.f,  6.f);
            float wSubDry    = gauss(absLat, 27.f, 10.f);
            float wTemperate = gauss(absLat, 52.f, 13.f);
            float wPolar     = gauss(absLat, 75.f,  9.f);
            float wSum = wITCZ + wSubDry + wTemperate + wPolar + 1e-6f;
            wITCZ /= wSum; wSubDry /= wSum; wTemperate /= wSum; wPolar /= wSum;

            // Ocean climate
            float cov_oc  = wITCZ*0.82f + wSubDry*0.35f + wTemperate*0.68f + wPolar*0.52f;
            float type_oc = wITCZ*0.84f + wSubDry*0.15f + wTemperate*0.48f + wPolar*0.12f;
            float hum_oc  = wITCZ*0.88f + wSubDry*0.52f + wTemperate*0.72f + wPolar*0.58f;

            // Land climate (modified by desert/rainforest)
            float itczLandCov  = 0.55f + rainforest * 0.20f;
            float itczLandType = 0.65f + rainforest * 0.15f;
            float itczLandHum  = 0.62f + rainforest * 0.20f;
            float subLandCov   = std::fmax(0.04f, 0.32f - desert * 0.28f);
            float subLandType  = std::fmax(0.06f, 0.35f - desert * 0.26f);
            float subLandHum   = std::fmax(0.06f, 0.32f - desert * 0.26f);
            float cov_la  = wITCZ*itczLandCov  + wSubDry*subLandCov  + wTemperate*0.48f + wPolar*0.38f;
            float type_la = wITCZ*itczLandType + wSubDry*subLandType + wTemperate*0.38f + wPolar*0.08f;
            float hum_la  = wITCZ*itczLandHum  + wSubDry*subLandHum  + wTemperate*0.52f + wPolar*0.42f;

            float coverage  = cov_oc  + (cov_la  - cov_oc)  * land;
            float cloudType = type_oc + (type_la - type_oc) * land;
            float humidity  = hum_oc  + (hum_la  - hum_oc)  * land;

            int idx = (y * W + x) * 4;
            data[idx+0] = (uint8_t)std::min(255, (int)(coverage  * 255.f + 0.5f));
            data[idx+1] = (uint8_t)std::min(255, (int)(cloudType * 255.f + 0.5f));
            data[idx+2] = (uint8_t)std::min(255, (int)(humidity  * 255.f + 0.5f));
            data[idx+3] = (uint8_t)std::min(255, (int)(land      * 255.f + 0.5f));
        }
    }

    if (!weatherT) glGenTextures(1, &weatherT);
    glBindTexture(GL_TEXTURE_2D, weatherT);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, W, H, 0, GL_RGBA, GL_UNSIGNED_BYTE, data.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
    std::cout << " done." << std::endl;
}
