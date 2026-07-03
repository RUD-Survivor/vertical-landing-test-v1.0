// =============================================================================
// cloud_rs3d_lighting.glsl
// -----------------------------------------------------------------------------
// 用途：RocketSim3D 体积云光照辅助库（binding 无关）。
//
// 提供内容：
//   • getOpticalDepth()   — 沿太阳方向积分大气 + 云自阴影光学深度
//   • cloudPhase()        — 双瓣 Henyey-Greenstein 云相位
//   • powderEffectNew()   — 粉末/银边效应（背光厚云提亮）
//   • coneAO()            — 锥形追踪云底 AO
//   • screenHash()        — 屏幕空间蓝噪声抖动（march 起点偏移）
//
// 设计原则：
//   1. 不声明任何 binding —— 纹理由调用方提供。
//   2. 大气光学深度实时算（不用 LUT），与 atmo.frag 一致。
//   3. 云自阴影用 cloudDensityLow（低频），避免与主 march 重复高细节计算。
//
// 来源：cloud.frag 第 270–312、315–318、531–566、184–188 行抽取。
// =============================================================================

#ifndef CLOUD_RS3D_LIGHTING_GLSL
#define CLOUD_RS3D_LIGHTING_GLSL

// ── 大气 + 云自阴影光学深度 ───────────────────────────────────────────────────
// 沿太阳方向 lDir 从 p 出发积分：
//   dR = ∫ Rayleigh 密度 ds   → gRayleighCoeff * dR 给天空蓝衰减
//   dM = ∫ Mie 密度 ds        → gMieCoeff * dM 给霾衰减
//   dO = ∫ 臭氧密度 ds         → gOzoneCoeff * dO 给 UV 吸收
//   dC = ∫ 云密度 ds           → gCloudExtinction * dC 给云影
//
// 若射线撞行星表面（innerRadius），返回 1e6 表示完全遮挡。
// 大气壳用 outerRadius 截断；云壳用 [surfaceRadius+min, surfaceRadius+max] 截断。
//
// 性能：uLightSteps 控制大气积分步数（默认 4~16）；云自阴影固定 10 步指数增长步长。
// 调用方：main 主 march 每步调一次 → sunTint / cloudShadow；light march 不递归调用本函数。
void getOpticalDepth(vec3 p, vec3 lDir, out float dR, out float dM, out float dO, out float dC) {
    dR = 0.0; dM = 0.0; dO = 0.0; dC = 0.0;
    float tS0, tS1;
    // 撞行星表面 → 完全遮挡
    if (intersectSphere(p, lDir, pc.innerRadius, tS0, tS1) && tS0 > 0.0) {
        dR = dM = dO = 1e6; return;
    }
    float t0, t1;
    // 大气壳求交
    if (!intersectSphere(p, lDir, pc.outerRadius, t0, t1) || t1 <= 0.0) return;
    float start = max(t0, 0.0);
    float stepSz = (t1 - start) / float(max(cloud.uLightSteps, 4));
    for (int i = 0; i < 64; i++) {
        if (i >= cloud.uLightSteps) break;
        vec3  pos = p + lDir * (start + (float(i) + 0.5) * stepSz);
        float h = length(pos - pc.planetCenter.xyz) - pc.surfaceRadius;
        if (h < -1.0) { dR = dM = dO = 1e6; return; }  // 撞地
        dR += exp(-h / gHRayleigh) * stepSz;
        dM += exp(-h / gHMie)      * stepSz;
        dO += ozoneDensity(h)       * stepSz;
    }

    // ── 云自阴影：低频形状，指数增长步长 ────────────────────────────────────
    // 用 cloudDensityLow（不做高细节侵蚀），沿太阳方向 10 步积分。
    // 步长 cStep 从 lightLen/10 起步，每步 *1.35 → 近处密、远处疏。
    float cloudInnerR = pc.surfaceRadius + gCloudMinAlt;
    float cloudOuterR = pc.surfaceRadius + gCloudMaxAlt;
    float ct0, ct1;
    if (intersectSphere(p, lDir, cloudOuterR, ct0, ct1) && ct1 > 0.0) {
        float cStart = max(ct0, 0.0);
        float cEnd = ct1;
        float lightLen = max(cEnd - cStart, 0.0);
        float d = min(lightLen * 0.04, 0.20);     // 起始偏移，避免自交
        float cStep = max(lightLen / 10.0, 0.08);
        for (int j = 0; j < 10; j++) {
            if (d >= lightLen) break;
            vec3 cp = p + lDir * (cStart + d);
            float ch = length(cp - pc.planetCenter.xyz) - pc.surfaceRadius;
            if (ch >= gCloudMinAlt && ch <= gCloudMaxAlt) {
                dC += cloudDensityLow(cp, ch) * cStep;
            }
            d += cStep;
            cStep *= 1.35;
        }
    }
}

// ── 双瓣 Henyey-Greenstein 云相位 ─────────────────────────────────────────────
// mix(HG(g2=-0.3), HG(g1=0.85), 0.7)：
//   g1=0.85 强前向 → 朝太阳亮边（银边）
//   g2=-0.3 后向 → 背光面散射
// cosTheta = dot(rayDir, lightDir)
float cloudPhase(float cosTheta) {
    float g1=0.85, num1=1.0-g1*g1, denom1=pow(max(1.0+g1*g1-2.0*g1*cosTheta,1e-5),1.5);
    float phase1=(1.0/(4.0*PI))*(num1/denom1);
    float g2=-0.3, num2=1.0-g2*g2, denom2=pow(max(1.0+g2*g2-2.0*g2*cosTheta,1e-5),1.5);
    float phase2=(1.0/(4.0*PI))*(num2/denom2);
    return mix(phase2,phase1,0.7);
}

// ── 粉末/银边效应 ─────────────────────────────────────────────────────────────
// 让背光、厚、云底不太黑。
//   depth  — 厚度因子（密度×8 + 光学深度×0.25）
//   height — 垂直位置概率（云底 7%~22% 高度带最强）
//   VoL    — 视线·光向 cos；背光时 height 权重大，顺光时减弱
float powderEffectNew(float depth, float height, float VoL) {
    float r = clamp(VoL * 0.5 + 0.5, 0.0, 1.0);  // VoL → [0,1]
    r *= r;                                        // 背光端权重更大
    return clamp(depth * mix(height, 1.0, r), 0.0, 2.0);
}

// ── 锥形追踪云底 AO ──────────────────────────────────────────────────────────
// 在 p 处沿「向下 + 两个切向」短距采样 uNoiseTex3D，估算云底遮挡。
// 返回 [0,1]：1=无遮挡，0=全遮挡。
// 仅在近距（< 60 km）且每 8 步算一次（性能控制）。
float coneAO(vec3 p, vec3 sph, float h) {
    if (h<gCloudMinAlt+0.3) return 0.08;  // 太靠近云底，直接给暗值
    vec3 down=-sph, tan1=normalize(cross(sph,vec3(0.371,0.629,0.683)));
    vec3 tan2=cross(sph,tan1);
    float occ=0.0, wSum=0.0;
    float dists[3]=float[3](0.12,0.28,0.50), wgts[3]=float[3](0.50,0.30,0.20);
    for (int d=0; d<3; d++) {
        // 向下采样（权重 0.50）
        { vec3 pp=p+down*dists[d]; float ph=length(pp-pc.planetCenter.xyz)-pc.surfaceRadius;
          if (ph>=gCloudMinAlt&&ph<=gCloudMaxAlt) {
              vec3 ps=normalize(pp-pc.planetCenter.xyz);
              float dC=1.0-texture(uNoiseTex3D,ps*INV8).g; occ+=dC*wgts[d]*0.50; }
          wSum+=wgts[d]*0.50; }
        // 切向 1 采样（权重 0.25）
        { vec3 pp=p+tan1*dists[d]*1.5; float ph=length(pp-pc.planetCenter.xyz)-pc.surfaceRadius;
          if (ph>=gCloudMinAlt&&ph<=gCloudMaxAlt) {
              vec3 ps=normalize(pp-pc.planetCenter.xyz);
              float dC=1.0-texture(uNoiseTex3D,ps*INV8).g; occ+=dC*wgts[d]*0.25; }
          wSum+=wgts[d]*0.25; }
        // 切向 2 采样（权重 0.25）
        { vec3 pp=p+tan2*dists[d]*1.5; float ph=length(pp-pc.planetCenter.xyz)-pc.surfaceRadius;
          if (ph>=gCloudMinAlt&&ph<=gCloudMaxAlt) {
              vec3 ps=normalize(pp-pc.planetCenter.xyz);
              float dC=1.0-texture(uNoiseTex3D,ps*INV8).g; occ+=dC*wgts[d]*0.25; }
          wSum+=wgts[d]*0.25; }
    }
    float avgOc=(wSum>0.001)?occ/wSum:0.0;
    return 1.0-avgOc*1.3;
}

// ── 屏幕空间抖动 ─────────────────────────────────────────────────────────────
// Interleaved Gradient Noise + 帧索引黄金分割偏移。
// 用于 march 起点抖动，减轻带状分层（banding）。
float screenHash(vec2 uv) {
    float ign = fract(52.9829189 * fract(dot(uv, vec2(0.06711056, 0.00583715))));
    return fract(ign + float(pc.frameIndex % 32) * 0.6180339887);
}

#endif // CLOUD_RS3D_LIGHTING_GLSL
