// =============================================================================
// cloud_rs3d_density.glsl
// -----------------------------------------------------------------------------
// 用途：RocketSim3D 体积云密度采样库（binding 无关）。
//
// 提供内容：
//   • cloudMapLocalCore() — 相机附近局部高细节密度核心（worldNoiseKm + 多尺度噪声）
//   • cloudDensityLow()   — 低频密度（getOpticalDepth 中 light march 用，省细节）
//   • cloudDensity()      — 主密度入口，按 planetIdx / 高度分层选择路径
//
// 设计原则：
//   1. 不声明任何 binding —— uNoiseTex3D / uCoverTex3D / uDetailTex3D / uWeatherMap
//      由调用方提供。
//   2. 全球 weather UV 一律用经纬度（planetSph → atan/asin），不随相机漂移。
//   3. 局部路径（cloudMapLocalCore）用 worldNoiseKm（地理 km 坐标），稳定且高细节。
//   4. 输出 gDbg* 调试通道，main 据此可视化。
//
// 来源：cloud.frag 第 190–529 行抽取。
//   cloudMapLocalCore   ← 190–250
//   cloudDensityLow     ← 252–268
//   cloudDensity        ← 320–529（含 cyclone / cirrus / 其它行星 fallback）
// =============================================================================

#ifndef CLOUD_RS3D_DENSITY_GLSL
#define CLOUD_RS3D_DENSITY_GLSL

// ── 相机附近局部高细节密度核心 ───────────────────────────────────────────────
// 在 useLocalCloudSampling() 为 true 时调用。比球面路径密度更高、更稳定。
//
// 流程（HZD/flower 顺序）：
//   coverage(天气图) → heightProfile(类型×高度) → baseShape(Perlin-Worley)
//   → detail erosion(Worley) → 近距 uDetailTex3D.a 侵蚀 → threshold remap → 密度
//
// 输入：
//   p           — 世界点
//   h           — 海拔高度 km
//   highDetail  — 是否启用近距高细节侵蚀（main 主 march=true，light march=false）
// 输出（out）：
//   coverage, heightProfile, baseShape, erodedShape — 调试用
// 返回：该点云密度标量（0=无云）
float cloudMapLocalCore(vec3 p, float h, bool highDetail,
                        out float coverage, out float heightProfile,
                        out float baseShape, out float erodedShape) {
    coverage = 0.0;
    heightProfile = 0.0;
    baseShape = 0.0;
    erodedShape = 0.0;

    float t = cloud.uTime * 0.004;

    // ── 1. 全球 weather map → coverage ──────────────────────────────────────
    // 关键：用经纬度 UV，不随相机漂移（RocketSim3D 行星/轨道视角必需）。
    vec3 sph = planetSph(p);
    vec2 wUV = vec2(atan(sph.y, sph.x) * (0.5 / PI) + 0.5,
                    asin(clamp(sph.z, -1.0, 1.0)) / PI + 0.5);
    vec4 weather = texture(uWeatherMap, wUV);
    coverage = remapClamped(weather.r, cloud.uCovLo, cloud.uCovHi, 0.0, 1.0);

    // ── 2. 高度 profile：stratus / cumulus 按云类型混合 ─────────────────────
    // altNorm 限定在 0~7 km 低层云带内归一化。
    float altNorm = clamp((h - gCloudMinAlt) / max(7.0 - gCloudMinAlt, 0.1), 0.0, 1.0);
    float altBase = smoothstep(0.0, 0.02, altNorm);          // 锐利云底（~110m）
    float cloudType = weather.g;                              // 天气图 G 通道
    float stratusProfile = altBase * (1.0 - smoothstep(0.82, 1.0, altNorm));
    float cumulusCore = exp(-((altNorm - 0.35) * (altNorm - 0.35)) / (0.25 * 0.25));
    float cumulusProfile = altBase * cumulusCore;
    heightProfile = mix(stratusProfile, cumulusProfile, smoothstep(0.0, 0.5, cloudType)) * 1.5;

    // ── 3. 基础形状：worldNoiseKm + 风 ──────────────────────────────────────
    // worldNoiseKm 用地理 km 坐标，浮点原点/相机移动时云不动。
    vec3 windKm = vec3(cloud.uPhaseCos * t * 2.0, 0.0, cloud.uPhaseSin * t * 2.0);
    const float kmScale = 1.6;
    vec3 posKm = worldNoiseKm(p, h);
    vec3 tc = posKm * kmScale + windKm + vec3(t * 0.6, t * 0.15, t * 0.4);
    baseShape = sampleBakedN(tc);

    // ── 4. 细节侵蚀：Worley G 通道，云顶反相 ────────────────────────────────
    float detailNoise = texture(uNoiseTex3D, tc * 0.2875 + vec3(-t * 0.04, t * 0.03, 0.0)).g;
    float topFraction = smoothstep(0.55, 0.95, altNorm);
    float detailMix = mix(detailNoise, 1.0 - detailNoise, topFraction) * cloud.uErosion;

    // HZD 顺序：coverage × remap(baseShape, 1-cov, 1, 0, 1) - detailMix
    float basicWithCov = coverage * remapClamped(baseShape, 1.0 - coverage, 1.0, 0.0, 1.0);
    erodedShape = basicWithCov - detailMix;

    // ── 5. 近距高细节侵蚀：uDetailTex3D.a ───────────────────────────────────
    if (highDetail) {
        float viewDist = length(frame.viewPos - p);
        if (viewDist < 8.0) {
            float dW1 = 1.0 - smoothstep(0.5, 8.0, viewDist);
            vec3 uvA = worldDetailUV(p, h);
            float da = texture(uDetailTex3D, uvA).a;
            float tFr1 = smoothstep(0.5, 0.85, altNorm);
            float maskLoVal = min(cloud.uThreshLo, cloud.uThreshHi);
            float surfW = 1.0 - smoothstep(maskLoVal, maskLoVal + 0.10, erodedShape);
            erodedShape -= mix(da, 1.0 - da, tFr1) * (0.10 + 0.30 * surfW) * dW1;
        }
    }

    // ── 6. threshold remap → 密度 ────────────────────────────────────────────
    // formPotential：coverage 高时用更紧的阈值（更厚实），低时放松（更碎）。
    float maskLo = min(cloud.uThreshLo, cloud.uThreshHi);
    float maskHi = max(cloud.uThreshLo, cloud.uThreshHi);
    float formPotential = smoothstep(0.18, 0.42, coverage);
    float localMaskLo = mix(0.58, maskLo, formPotential);
    float localMaskHi = mix(0.76, maskHi, formPotential);
    float shaped = remapClamped(erodedShape, localMaskLo, localMaskHi, 0.0, 1.0);
    float densityScale = cloud.uDensity * (0.30 + 0.70 * coverage);
    return shaped * heightProfile * densityScale;
}

// ── 低频密度（light march 用）────────────────────────────────────────────────
// 用于 getOpticalDepth 沿太阳路积分云自阴影。比 cloudDensity 省细节：
//   • 不做 cyclone / meso / 高细节侵蚀
//   • 只取 weather + 球面低频采样
// 注意：内部仍走 useLocalCloudSampling 分支，保证与主路径一致。
float cloudDensityLow(vec3 p, float h) {
    if (h < gCloudMinAlt || h > gCloudMaxAlt) return 0.0;
    float camAltKm = length(frame.viewPos - pc.planetCenter.xyz) - pc.surfaceRadius;
    if (useLocalCloudSampling(p, camAltKm)) {
        float cov, prof, base, eroded;
        return cloudMapLocalCore(p, h, false, cov, prof, base, eroded);
    }
    // 球面低频路径：球面方向 *3 + 时间漂移，再 weather coverage。
    vec3 sph = normalize(p - pc.planetCenter.xyz);
    float altN = clamp((h - gCloudMinAlt) / max(gCloudMaxAlt - gCloudMinAlt, 0.1), 0.0, 1.0);
    float profile = smoothstep(0.0, 0.08, altN) * smoothstep(1.0, 0.82, altN);
    vec2 wUV = vec2(atan(sph.y, sph.x) * (0.5 / PI) + 0.5,
                    asin(clamp(sph.z, -1.0, 1.0)) / PI + 0.5);
    vec4 weather = texture(uWeatherMap, wUV);
    float coverage = remapClamped(weather.r, cloud.uCovLo, cloud.uCovHi, 0.0, 1.0);
    float baseShape = sampleBakedN(sph * 3.0 + vec3(cloud.uTime * 0.0004));
    return remapClamped(baseShape * coverage * profile, 0.18, 0.85, 0.0, 1.0) * cloud.uDensity;
}

// ── 主密度入口 ───────────────────────────────────────────────────────────────
// main 主 march 调用。按 planetIdx 与高度分层：
//   idx 3（地球类）：
//     • 低层（gCloudMinAlt ~ 7 km）：局部路径 or 球面 curl+coverage+cyclone 高细节
//     • 高层（8 ~ gCloudMaxAlt km）：cirrus 卷云（球面噪声）
//   其它行星：单层球面噪声 fallback
//
// 输入：
//   p           — 世界点
//   h           — 海拔高度 km
//   highDetail  — 是否启用近距高细节（main=true, light march=false）
// 返回：密度标量；同时写 gDbg* 调试通道
float cloudDensity(vec3 p, float h, bool highDetail) {
    if (pc.showClouds < 0.5) return 0.0;
    if (h < gCloudMinAlt || h > gCloudMaxAlt) return 0.0;

    // 调试：强制均匀密度，隔离形状与光照
    if (cloud.uDebug == 1) return 2.0;

    gDbgCoverage = 0.0;
    gDbgProfile = 0.0;
    gDbgBase = 0.0;
    gDbgEroded = 0.0;

    float camAltKm = length(frame.viewPos - pc.planetCenter.xyz) - pc.surfaceRadius;

    // ── idx 3 + 局部路径：直接走 cloudMapLocalCore + cirrus ────────────────
    if (pc.planetIdx == 3 && useLocalCloudSampling(p, camAltKm)) {
        float cov, prof, base, eroded;
        float density = 0.0;
        if (h >= gCloudMinAlt && h <= 7.0) {
            density = cloudMapLocalCore(p, h, highDetail, cov, prof, base, eroded);
            gDbgCoverage = cov;
            gDbgProfile = prof;
            gDbgBase = base;
            gDbgEroded = eroded;
        }
        if (h >= 8.0 && h <= gCloudMaxAlt) {
            // Cirrus：球面路径，轻量
            vec3 sph = normalize(p - pc.planetCenter.xyz);
            float t = cloud.uTime * 0.004;
            float altNorm = (h - 8.0) / (gCloudMaxAlt - 8.0);
            float altShape = smoothstep(0.0, 0.08, altNorm) * exp(-altNorm * 3.2);
            float angle2 = t * -0.5;
            float latWarp2 = sph.z * sph.z * sph.z * 0.15;
            float s2 = sin(angle2 + latWarp2), c2 = cos(angle2 + latWarp2);
            vec3 windSph2 = vec3(sph.x * c2 - sph.y * s2, sph.x * s2 + sph.y * c2, sph.z);
            vec3 tc2 = windSph2 * 50.0 + vec3(t * 0.04);
            float n2 = texture(uNoiseTex3D, tc2 * INV8).b;
            n2 -= texture(uNoiseTex3D, tc2 * 0.3125).g * 0.18;
            float mask2 = smoothstep(0.42 + altNorm * 0.10, 0.82, n2) * altShape;
            density = max(density, mask2 * 0.30);
            gDbgCoverage = max(gDbgCoverage, mask2);
            gDbgProfile = max(gDbgProfile, altShape);
            gDbgBase = max(gDbgBase, n2);
            gDbgEroded = max(gDbgEroded, n2);
        }
        return density;
    }

    // ── idx 3 球面高细节路径（远距 / 非 local）──────────────────────────────
    vec3 sph = normalize(p - pc.planetCenter.xyz);
    float t = cloud.uTime * 0.004;
    float density = 0.0;

    if (pc.planetIdx == 3) {
        // 低层云带（gCloudMinAlt ~ 7 km）：cyclone + curl coverage + meso + 多阶段形状
        if (h >= gCloudMinAlt && h <= 7.0) {
            float altNorm = (h - gCloudMinAlt) / (7.0 - gCloudMinAlt);
            float altBase = smoothstep(0.0, 0.02, altNorm);  // 锐利云底

            // 风向旋转（latWarp 让赤道附近风更强）
            float latWarp = sph.z*sph.z*sph.z*0.3;
            float sB=sin(latWarp), cB=cos(latWarp);
            float s1=cloud.uPhaseSin*cB+cloud.uPhaseCos*sB;
            float c1=cloud.uPhaseCos*cB-cloud.uPhaseSin*sB;
            vec3 windSph1 = vec3(sph.x*c1-sph.y*s1, sph.x*s1+sph.y*c1, sph.z);

            // 气旋扭曲：随时间移动的 cycCenter，附近 windSph1 被 Rodrigues 旋转
            float cycAngle = t * -0.1;
            vec3 cycCenter = normalize(vec3(0.6*cos(cycAngle),0.6*sin(cycAngle),0.35));
            float cycDist = distance(sph, cycCenter);
            float cycInfluence = smoothstep(0.22, 0.0, cycDist);
            if (cycInfluence > 0.001) {
                float twist = cycInfluence*1.8;
                float ct=cos(twist), st=sin(twist);
                vec3 cycCenterW = normalize(vec3(cycCenter.x*c1-cycCenter.y*s1,
                                                 cycCenter.x*s1+cycCenter.y*c1,cycCenter.z));
                vec3 warped = windSph1*ct+cross(cycCenterW,windSph1)*st
                            + cycCenterW*dot(cycCenterW,windSph1)*(1.0-ct);
                windSph1 = mix(windSph1, warped, cycInfluence);
            }

            // Curl 噪声：用 uCoverTex3D 的相邻采样差分得到 curl，扭曲 coverage 采样方向
            vec3 coverageSph = vec3(sph.x*c1-sph.y*s1, sph.x*s1+sph.y*c1, sph.z);
            vec3 wAxis = normalize(cross(coverageSph,vec3(0,0,1)+coverageSph*0.01));
            vec3 vAxis = cross(coverageSph,wAxis);
            const float EPS=0.06;

            vec3 seedLo = vec3(t*0.003,0,t*0.002);
            float pLo_pu=texture(uCoverTex3D,(coverageSph*1.5+wAxis*EPS+seedLo)*INV8).r;
            float pLo_mu=texture(uCoverTex3D,(coverageSph*1.5-wAxis*EPS+seedLo)*INV8).r;
            float pLo_pv=texture(uCoverTex3D,(coverageSph*1.5+vAxis*EPS+seedLo)*INV8).r;
            float pLo_mv=texture(uCoverTex3D,(coverageSph*1.5-vAxis*EPS+seedLo)*INV8).r;
            float curlU_lo=-(pLo_pv-pLo_mv), curlV_lo=(pLo_pu-pLo_mu);

            vec3 seedHi=vec3(91.2+t*0.005,5.1,22.0);
            float pHi_pu=texture(uCoverTex3D,(coverageSph*6.0+wAxis*EPS+seedHi)*INV8).g;
            float pHi_mu=texture(uCoverTex3D,(coverageSph*6.0-wAxis*EPS+seedHi)*INV8).g;
            float pHi_pv=texture(uCoverTex3D,(coverageSph*6.0+vAxis*EPS+seedHi)*INV8).g;
            float pHi_mv=texture(uCoverTex3D,(coverageSph*6.0-vAxis*EPS+seedHi)*INV8).g;
            float curlU_hi=-(pHi_pv-pHi_mv), curlV_hi=(pHi_pu-pHi_mu);

            float warpU=curlU_lo*2.5+curlU_hi*1.0, warpV=curlV_lo*2.5+curlV_hi*1.0;
            vec3 warpedSph=normalize(coverageSph+wAxis*warpU+vAxis*warpV);
            float cov_large=texture(uCoverTex3D,(warpedSph*0.8+vec3(t*0.004))*INV8).b;
            float cov_medium=texture(uCoverTex3D,(warpedSph*6.0+vec3(t*0.011,-t*0.008,t*0.013))*INV8).a;
            float coverage=cov_large*0.65+cov_medium*0.35;
            vec2 wUV=vec2(atan(warpedSph.y,warpedSph.x)*(0.5/PI)+0.5,
                         asin(clamp(warpedSph.z,-1.0,1.0))/PI+0.5);
            vec4 weather=(pc.planetIdx==3)?texture(uWeatherMap,wUV):vec4(0.65,0.5,0.6,0.0);
            coverage=clamp(coverage*weather.r,0.0,1.0);
            float covRange=max(cloud.uCovHi-cloud.uCovLo,0.001);
            coverage=clamp((coverage-cloud.uCovLo)/covRange,0.0,1.0);

            // 气旋眼：cycInfluence 高时加密 coverage 并挖眼
            if (cycInfluence>0.001) {
                coverage=mix(coverage,clamp(coverage*1.9,0.0,1.0),cycInfluence*0.7);
                float eyeNoise=texture(uCoverTex3D,(sph*4.0+vec3(t*0.02))*INV8).r*0.009;
                coverage*=smoothstep((0.017+eyeNoise)*0.35,0.017+eyeNoise,cycDist);
            }

            // HZD/flower 顺序：coverage → height profile → base shape → detail erosion → density
            float cloudType=mix(texture(uCoverTex3D,(warpedSph*2.5+vec3(31.4,17.8,42.1))*INV8).r,weather.g,0.55);
            float stratusProfile=altBase*(1.0-smoothstep(0.82,1.0,altNorm));
            float cumulusCore=exp(-((altNorm-0.35)*(altNorm-0.35))/(0.25*0.25));
            float topNoise=texture(uCoverTex3D,(windSph1*8.0+vec3(53.2,29.7,71.4+t*0.01))*INV8).r;
            float topHeight=mix(0.55,1.00,topNoise);
            float cumulusTop=1.0-smoothstep(topHeight-0.08,topHeight+0.08,altNorm);
            float cumulusProfile=altBase*cumulusCore*cumulusTop;
            float cbTower=smoothstep(0.0,0.50,altNorm)*(1.0-smoothstep(0.50,0.72,altNorm));
            float cbAnvil=smoothstep(0.68,0.80,altNorm)*(1.0-smoothstep(0.88,1.00,altNorm))*0.35;
            float cumulonimbusProfile=altBase*max(cbTower,cbAnvil);
            float heightGradient=mix(mix(stratusProfile,cumulusProfile,smoothstep(0.0,0.5,cloudType)),
                                    cumulonimbusProfile,smoothstep(0.5,1.0,cloudType));
            float heightProfile=heightGradient*1.5;

            vec3 tc=windSph1*(80.0+h*0.45)+vec3(t*0.6,t*0.4,t*0.15);
            float viewDist=length(frame.viewPos-p);

            // 基础形状：warp 扰动 + sampleBakedN
            float warp=texture(uNoiseTex3D,tc*0.0375).b;
            vec3 warpedTc=tc+vec3(warp*0.4,warp*0.4,warp*0.2)+vec3(t*0.02);
            float baseShape=sampleBakedN(warpedTc);

            // 细节侵蚀
            float ero=texture(uNoiseTex3D,tc*0.2875+vec3(-t*0.04,t*0.03,0.0)).g;
            float topFraction=smoothstep(topHeight*0.5,topHeight*0.95,altNorm);
            float detailErosion=mix(ero,1.0-ero,topFraction)*cloud.uErosion;
            float erodedShape=baseShape-detailErosion;

            // Meso 尺度扰动：再叠一层低频细节
            {
                vec3 mesoSph = normalize(windSph1 + wAxis * (warp - 0.5) * 0.0028
                                              + vAxis * (warp * 1.4 - 0.7) * 0.0028);
                vec3 mesoTc = mesoSph * (80.0 + h * 0.45) + vec3(t * 0.6, t * 0.4, t * 0.15);
                float c1=1.0-texture(uNoiseTex3D,mesoTc*0.005+vec3(0.914+t*0.00225,0.521,0.341+t*0.00150)).g;
                float c2=1.0-texture(uNoiseTex3D,mesoTc*0.00975+vec3(0.039,0.440+t*0.00275,0.091)).g;
                erodedShape+=(c1*(0.4+0.6*c2))*0.30-0.16;
            }

            // 近距高细节侵蚀
            if (viewDist<8.0) {
                float dW1=1.0-smoothstep(0.5,8.0,viewDist);
                vec3 uvA=worldDetailUV(p,h);
                float da=texture(uDetailTex3D,uvA).a;
                float tFr1=smoothstep(0.5,0.85,altNorm);
                float maskLoVal=min(cloud.uThreshLo,cloud.uThreshHi);
                float surfW=1.0-smoothstep(maskLoVal,maskLoVal+0.10,erodedShape);
                erodedShape-=mix(da,1.0-da,tFr1)*(0.10+0.30*surfW)*dW1;
            }

            // threshold remap（meso 尺度变化阈值，让厚度有区域差异）
            float maskLo=min(cloud.uThreshLo,cloud.uThreshHi);
            float maskHi=max(cloud.uThreshLo,cloud.uThreshHi);
            float formPotential=smoothstep(0.18,0.42,coverage);
            float localMaskLo=mix(0.58,maskLo,formPotential);
            float localMaskHi=mix(0.76,maskHi,formPotential);
            float mesoField=texture(uCoverTex3D,(windSph1*1.2+vec3(71.3,39.7,14.2+t*0.002))*INV8).r;
            float thickVar=(mesoField-0.45)*0.28;
            float mesoLo=localMaskLo+thickVar, mesoHi=localMaskHi+thickVar*0.5;
            float coveredShape=remapClamped(erodedShape, mesoLo, mesoHi, 0.0, 1.0);
            float densityScale=cloud.uDensity*(0.30+0.70*coverage);
            density=max(density,coveredShape*heightProfile*densityScale);
            gDbgCoverage = coverage;
            gDbgProfile = heightProfile;
            gDbgBase = baseShape;
            gDbgEroded = erodedShape;
        }

        // 高层 cirrus（8 ~ gCloudMaxAlt km）
        if (h>=8.0 && h<=gCloudMaxAlt) {
            float altNorm=(h-8.0)/(gCloudMaxAlt-8.0);
            float cirrusBase=smoothstep(0.0,0.08,altNorm);
            float cirrusWisp=exp(-altNorm*3.2);
            float altShape=cirrusBase*cirrusWisp;
            float angle2=t*-0.5;
            float latWarp2=sph.z*sph.z*sph.z*0.15;
            float s2=sin(angle2+latWarp2),c2=cos(angle2+latWarp2);
            vec3 windSph2=vec3(sph.x*c2-sph.y*s2,sph.x*s2+sph.y*c2,sph.z);
            vec3 tc2=windSph2*50.0+vec3(t*0.04);
            float n2=texture(uNoiseTex3D,tc2*INV8).b;
            n2-=texture(uNoiseTex3D,tc2*0.3125).g*0.18;
            float mask2=smoothstep(0.42+altNorm*0.10,0.82,n2)*altShape;
            density=max(density,mask2*0.30);
            gDbgCoverage=max(gDbgCoverage,mask2);
            gDbgProfile=max(gDbgProfile,altShape);
            gDbgBase=max(gDbgBase,n2);
            gDbgEroded=max(gDbgEroded,n2);
        }
    } else {
        // 其它行星：单层球面噪声 fallback
        float altNorm=(h-gCloudMinAlt)/(gCloudMaxAlt-gCloudMinAlt);
        float altShape=4.0*altNorm*(1.0-altNorm);  // 抛物线高度包络
        float angle=cloud.uTime*0.004*-0.1;
        float s1s=sin(angle),c1s=cos(angle);
        vec3 windSph=vec3(sph.x*c1s-sph.y*s1s,sph.x*s1s+sph.y*c1s,sph.z);
        float n=sampleBakedN(windSph*8.0+vec3(cloud.uTime*0.0002));
        density=(0.3+0.7*n)*altShape*0.8;
        gDbgCoverage=n;
        gDbgProfile=altShape;
        gDbgBase=n;
        gDbgEroded=n;
    }
    return density;
}

#endif // CLOUD_RS3D_DENSITY_GLSL
