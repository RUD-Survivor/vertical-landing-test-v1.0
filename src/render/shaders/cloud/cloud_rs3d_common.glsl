// =============================================================================
// cloud_rs3d_common.glsl
// -----------------------------------------------------------------------------
// 用途：RocketSim3D 体积云公共定义与工具函数库（binding 无关）。
//
// 提供内容：
//   • 常量：PI、CLOUD_STEPS_MAX
//   • 全局大气/云参数（g* 系列），由 setupPlanetProfile() 按 planetIdx 填充
//   • 坐标系工具：
//       planetSph()         — 世界点 → 行星径向单位向量（全球 weather UV 的基础）
//       worldNoiseKm()      — 世界点 → 稳定的 km 尺度噪声坐标（不随浮点原点/相机漂移）
//       worldDetailUV()     — 世界点 → 细节噪声 3D 采样坐标
//       useLocalCloudSampling() — 判断是否进入「相机附近局部高细节」采样路径
//   • setupPlanetProfile()  — 按 planetIdx 设置 Rayleigh/Mie/Ozone/Cloud 系数
//   • intersectSphere()     — 相机射线与行星球壳求交（统一以 pc.planetCenter 为球心）
//   • ozoneDensity()        — 高斯型臭氧密度
//
// 设计原则：
//   1. 不声明任何 binding / uniform —— 由 include 它的 .frag / .comp 提供布局。
//   2. 全部依赖 frame.* / cloud.* / pc.* 三个外部 uniform 块，签名与 cloud.frag 原版一致。
//   3. 球心统一使用 pc.planetCenter.xyz（浮点原点），不假设 (0,0,0)。
//   4. 高度一律 = length(p - planetCenter) - surfaceRadius，与 atmo.frag 一致。
//
// 来源：cloud.frag 第 6–7、65–137 行抽取。
// =============================================================================

#ifndef CLOUD_RS3D_COMMON_GLSL
#define CLOUD_RS3D_COMMON_GLSL

// ── 常量 ──────────────────────────────────────────────────────────────────────
#define PI 3.14159265359
// march 循环安全上限；实际步长由 adaptStep 动态决定，循环内 break 提前退出。
#define CLOUD_STEPS_MAX 128

// ── 大气/云全局参数（g*）──────────────────────────────────────────────────────
// 由 setupPlanetProfile() 按行星索引填充；在 cloudDensity / getOpticalDepth / 光照组装中使用。
vec3  gRayleighCoeff;       // Rayleigh 散射系数 (1/km)，RGB 不同导致天空蓝
float gMieCoeff;            // Mie 散射系数
float gHRayleigh, gHMie;    // Rayleigh/Mie 高度尺度 (km)，越大衰减越慢
float gGMie;                // Mie 相位各向异性 g
vec3  gOzoneCoeff;          // 臭氧吸收系数 (1/km)
float gHOzoneCenter;        // 臭氧层中心高度 (km)
float gHOzoneWidth;         // 臭氧层宽度 (km)
float gCloudMinAlt, gCloudMaxAlt;  // 云带 [min, max] 高度 (km, 海平面以上)
vec3  gCloudExtinction;     // 云消光系数 σ_t (RGB 同值，便于多散射阶调色)

// 调试通道：cloudDensity 写入，main 读取用于 uDebug 可视化
float gDbgCoverage = 0.0;
float gDbgProfile  = 0.0;
float gDbgBase     = 0.0;
float gDbgEroded   = 0.0;

// ── 坐标系工具 ────────────────────────────────────────────────────────────────

// 世界点 p → 行星径向单位向量。
// 用于：
//   • 全球 weather map 的经纬度 UV（不随相机/浮点原点漂移，关键于轨道视角）
//   • 球面 curl 噪声采样
//   • coneAO 的局部坐标系
vec3 planetSph(vec3 p) {
    return normalize(p - pc.planetCenter.xyz);
}

// 世界点 p → 稳定的 km 尺度噪声坐标。
// lon/lat 弧度 → 弧长 km：eastKm = lon * R * cos(lat)，northKm = lat * R。
// 关键：用「地理方向」而非「世界 XZ」，相机移动时云不动 —— 这是 RocketSim3D
// 相对 flower 的核心优势（flower 用 posKm.xz 会随浮点原点乱飘）。
// 输出 .y = 云带内高度 h - gCloudMinAlt，方便 3D 噪声垂直采样。
vec3 worldNoiseKm(vec3 p, float h) {
    vec3 sph = planetSph(p);
    float lon = atan(sph.y, sph.x);
    float lat = asin(clamp(sph.z, -1.0, 1.0));
    float cosLat = max(cos(lat), 0.01);          // 防极点除零
    float eastKm  = lon * pc.surfaceRadius * cosLat;
    float northKm = lat * pc.surfaceRadius;
    return vec3(eastKm, h - gCloudMinAlt, northKm);
}

// 世界点 p → 细节噪声 3D UV（用于近距 uDetailTex3D.a 侵蚀）。
// 用球面方向 fract + 高度偏移，避免随距离重复采样。
vec3 worldDetailUV(vec3 p, float h) {
    vec3 sph = planetSph(p);
    return fract(sph * 47.0 + vec3(h * 0.031, h * 0.017, h * 0.023)
               + vec3(0.137, 0.421, 0.073));
}

// 是否进入「相机附近局部高细节」采样路径。
// 条件：调参 uLocalRadius > 0 且相机低空（< 30 km）且距离相机够近。
// 局部路径用 cloudMapLocalCore（worldNoiseKm + 高密度 3D 噪声），
// 远处用球面低频采样（cloudDensity 的 spherical 分支）以保性能。
bool useLocalCloudSampling(vec3 p, float camAltKm) {
    if (cloud.uLocalRadius <= 0.0 || camAltKm > 30.0) return false;
    return length(p - frame.viewPos) <= cloud.uLocalRadius;
}

// ── 行星 profile 初始化 ───────────────────────────────────────────────────────
// 按 pc.planetIdx 设置大气与云带参数。每帧 main 起始调用一次。
//   idx 3 = 地球类（强 Rayleigh、薄臭氧）
//   idx 2 = 厚大气类（强 Mie、厚臭氧）
//   其他  = 默认地球类（无臭氧）
void setupPlanetProfile() {
    int idx = pc.planetIdx;
    if (idx == 3) {
        gRayleighCoeff = vec3(5.8, 13.5, 33.1) * 1.2e-3;
        gMieCoeff = 5.0e-3; gHRayleigh = 8.0; gHMie = 1.0; gGMie = 0.8;
        gOzoneCoeff = vec3(0.35, 0.85, 0.09) * 1e-3; gHOzoneCenter = 25.0; gHOzoneWidth = 15.0;
    } else if (idx == 2) {
        gRayleighCoeff = vec3(5.0, 12.0, 28.0) * 1e-3;
        gMieCoeff = 0.04; gHRayleigh = 15.0; gHMie = 5.0; gGMie = 0.76;
        gOzoneCoeff = vec3(0.5, 5.0, 20.0) * 1e-3; gHOzoneCenter = 50.0; gHOzoneWidth = 20.0;
    } else {
        gRayleighCoeff = vec3(5.8, 13.5, 33.1) * 1e-3;
        gMieCoeff = 0.005; gHRayleigh = 8.0; gHMie = 1.0; gGMie = 0.8;
        gOzoneCoeff = vec3(0.0); gHOzoneCenter = 1.0; gHOzoneWidth = 1.0;
    }
    gCloudMinAlt = cloud.uMinAlt;
    gCloudMaxAlt = cloud.uMaxAlt;
    gCloudExtinction = vec3(cloud.uExtinction);
}

// ── 球壳求交 ──────────────────────────────────────────────────────────────────
// 相机射线 (ro, rd) 与以 planetCenter 为心、半径 radius 的球壳求交。
// 返回是否有交，t0 = 近交点（可能 <0），t1 = 远交点。
// 调用方负责 max(t0, 0.0) 截取。
// 用于：
//   • 云带内/外球壳求 tStart/tEnd
//   • 行星表面截断（避免云画到地下）
//   • getOpticalDepth 中大气壳/云壳求交
bool intersectSphere(vec3 ro, vec3 rd, float radius, out float t0, out float t1) {
    vec3  L = ro - pc.planetCenter.xyz;
    float b = dot(L, rd), disc = b*b - dot(L,L) + radius*radius;
    if (disc < 0.0) return false;
    float sq = sqrt(disc); t0 = -b - sq; t1 = -b + sq;
    return true;
}

// ── 臭氧密度 ──────────────────────────────────────────────────────────────────
// 高斯型：以 gHOzoneCenter 为中心，gHOzoneWidth 为标准差。
// 在 getOpticalDepth 沿太阳路积分时累加 dO。
float ozoneDensity(float h) {
    float d = (h - gHOzoneCenter) / gHOzoneWidth;
    return exp(-0.5 * d * d);
}

#endif // CLOUD_RS3D_COMMON_GLSL
