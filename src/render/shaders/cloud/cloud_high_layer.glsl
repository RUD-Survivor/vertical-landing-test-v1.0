#ifndef CLOUD_HIGH_LAYER_GLSL
#define CLOUD_HIGH_LAYER_GLSL

// ==========================================================================
// cloud_high_layer.glsl — 高空云（第二层，Ac/As）密度
//
// 与低云共用 cloudMap → 主 march（τ = σ×Δs_km×1000），不再独立 march。
// 密度单位与低云 cloudMap 返回值相同（Flower 米制 σ 标量），经 max() 合成。
// ==========================================================================

vec2 highCloudWeatherUv(vec3 posKm)
{
    const vec3 sphDir=normalize(posKm);
    const float lonRad=atan(sphDir.y,sphDir.x);
    const float latRad=asin(clamp(sphDir.z,-1.0,1.0));
    vec2 uv=vec2(lonRad*(1.0/(2.0*kPI))+0.5,latRad*(1.0/kPI)+0.5);
    uv*=frameData.sky.atmosphereConfig.cloudWeatherUVScale;

    const float kEqKm=2.0*kPI*frameData.sky.atmosphereConfig.bottomRadius;
    const vec2 windDir=vec2(frameData.sky.atmosphereConfig.cloudDirection.x,
                            frameData.sky.atmosphereConfig.cloudDirection.z);
    const vec2 windUv=windDir/vec2(kEqKm,kEqKm*0.5);
    uv+=windUv*frameData.appTime.x*frameData.sky.atmosphereConfig.cloudSpeed;
    return uv;
}

// 高空云密度（与低云同单位）：纯 2D Hi weather，带 [hiCloudBottom, hiCloudTop]。
float highCloudDensity(vec3 posKm, float normalizedHeight)
{
    const float hiGlobal=frameData.sky.atmosphereConfig.hiCloudCoverage;
    if(hiGlobal<=0.0){ return 0.0; }

    const vec4 hiW=texture(sampler2D(inCloudHiWeather,linearRepeatSampler),highCloudWeatherUv(posKm));
    const float hiCov=hiW.r*hiGlobal;
    if(hiCov<1e-3){ return 0.0; }

    const float hiType=mix(frameData.sky.atmosphereConfig.hiCloudType,hiW.g,0.5);
    const float hb=frameData.sky.atmosphereConfig.hiCloudBottom;
    const float ht=frameData.sky.atmosphereConfig.hiCloudTop;
    const float range=max(ht-hb,1e-3);
    const float top=hb+range*pow(saturate(hiCov),0.7+0.5*hiType);

    const float thr=frameData.sky.atmosphereConfig.hiCloudThreshold;
    const float soft=max(frameData.sky.atmosphereConfig.hiCloudSoftness,0.001);
    const float baseDensity=hiCov*smoothstep(thr,thr+2.0*soft,hiCov);

    const float bandSoft=0.04;
    const float bandMask=smoothstep(hb-bandSoft,hb+bandSoft,normalizedHeight)
                        *(1.0-smoothstep(top-bandSoft,top+bandSoft,normalizedHeight));

    return saturate(baseDensity*bandMask)*frameData.sky.atmosphereConfig.hiCloudDensity;
}

#endif
