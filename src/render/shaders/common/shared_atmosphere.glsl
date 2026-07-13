#ifndef SHARED_ATMOSPHERE_GLSL
#define SHARED_ATMOSPHERE_GLSL

#include "shared_struct.glsl"
#include "shared_functions.glsl"

// All distance units in kilometers

struct AtmosphereParameters
{
    float atmospherePreExposure;

	// Radius of the planet (center to ground)
	float bottomRadius;

	// Maximum considered atmosphere height (center to atmosphere top)
	float topRadius;

	// Rayleigh scattering exponential distribution scale in the atmosphere
	float rayleighDensityExpScale;

	// Rayleigh scattering coefficients
	vec3 rayleighScattering;

	// Mie scattering exponential distribution scale in the atmosphere
	float mieDensityExpScale;

	// Mie scattering coefficients
	vec3 mieScattering;

	// Mie extinction coefficients
	vec3 mieExtinction;

	// Mie absorption coefficients
	vec3 mieAbsorption;

	// Mie phase function excentricity
	float miePhaseG;

	// Another medium type in the atmosphere
	float absorptionDensity0LayerWidth;
	float absorptionDensity0ConstantTerm;
	float absorptionDensity0LinearTerm;
	float absorptionDensity1ConstantTerm;
	float absorptionDensity1LinearTerm;

	// This other medium only absorb light, e.g. useful to represent ozone in the earth atmosphere
	vec3 absorptionExtinction;

	// The albedo of the ground.
	vec3 groundAlbedo;

	float multipleScatteringFactor; 

	uint viewRayMarchMinSPP;
	uint viewRayMarchMaxSPP;

	float cloudAreaStartHeight; // km
    float cloudAreaThickness;
    mat4 cloudShadowViewProj;
    mat4 cloudShadowViewProjInverse;
};

// Build atmosphere parameters from frame data.
AtmosphereParameters getAtmosphereParameters(in const PerFrameData frameData)
{
	AtmosphereParameters parameters;

    const AtmosphereConfig config = frameData.sky.atmosphereConfig;

	parameters.absorptionExtinction = config.absorptionColor * config.absorptionLength;

    // Copy parameters.
    parameters.groundAlbedo             = config.groundAlbedo;
	parameters.bottomRadius             = config.bottomRadius;
	parameters.topRadius                = config.topRadius;
    parameters.viewRayMarchMinSPP       = config.viewRayMarchMinSPP;
	parameters.viewRayMarchMaxSPP       = config.viewRayMarchMaxSPP;
	parameters.miePhaseG                = config.miePhaseFunctionG;
    parameters.atmospherePreExposure    = config.atmospherePreExposure;
	parameters.multipleScatteringFactor = config.multipleScatteringFactor;

	// Traslation from Bruneton2017 parameterisation.
	parameters.rayleighDensityExpScale        = config.rayleighDensity[1].w;
	parameters.mieDensityExpScale             = config.mieDensity[1].w;
	parameters.absorptionDensity0LayerWidth   = config.absorptionDensity[0].x;
	parameters.absorptionDensity0ConstantTerm = config.absorptionDensity[1].x;
	parameters.absorptionDensity0LinearTerm   = config.absorptionDensity[0].w;
	parameters.absorptionDensity1ConstantTerm = config.absorptionDensity[2].y;
	parameters.absorptionDensity1LinearTerm   = config.absorptionDensity[2].x;

	parameters.rayleighScattering = config.rayleighScatteringColor * config.rayleighScatterLength;
    parameters.mieAbsorption      = config.mieAbsorption;
	parameters.mieScattering      = config.mieScatteringColor * config.mieScatteringLength;
	parameters.mieExtinction      = parameters.mieScattering + config.mieAbsColor * config.mieAbsLength;

	parameters.cloudAreaStartHeight       = config.cloudAreaStartHeight;
    parameters.cloudAreaThickness         = config.cloudAreaThickness;
    parameters.cloudShadowViewProj        = config.cloudSpaceViewProject;
    parameters.cloudShadowViewProjInverse = config.cloudSpaceViewProjectInverse;

	return parameters;
}

// https://github.com/sebh/UnrealEngineSkyAtmosphere
// Transmittance LUT function parameterisation from Bruneton 2017 https://github.com/ebruneton/precomputed_atmospheric_scattering
// Detail also in video https://www.youtube.com/watch?v=y-oBGzDCZKI at 08:35.
void lutTransmittanceParamsToUv(
    in const AtmosphereParameters atmosphere, 
    in float viewHeight, // [bottomRAdius, topRadius]
    in float viewZenithCosAngle, // [-1,1]
    out vec2 uv) // [0,1]
{
	float H = sqrt(max(0.0f, atmosphere.topRadius * atmosphere.topRadius - atmosphere.bottomRadius * atmosphere.bottomRadius));
	float rho = sqrt(max(0.0f, viewHeight * viewHeight - atmosphere.bottomRadius * atmosphere.bottomRadius));

	uv.y = rho / H;

	// Distance to atmosphere boundary
	float discriminant = viewHeight * viewHeight * (viewZenithCosAngle * viewZenithCosAngle - 1.0) + atmosphere.topRadius * atmosphere.topRadius;
	float d = max(0.0, (-viewHeight * viewZenithCosAngle + sqrt(discriminant))); 

	float dMin = atmosphere.topRadius - viewHeight;
	float dMax = rho + H;

	uv.x = (d - dMin) / (dMax - dMin);
}

void uvToLutTransmittanceParams(
    in const AtmosphereParameters atmosphere, 
    out float viewHeight, // [bottomRAdius, topRadius]
    out float viewZenithCosAngle, // [-1,1]
    in vec2 uv) // [0,1]
{ 
	float H = sqrt(atmosphere.topRadius * atmosphere.topRadius - atmosphere.bottomRadius * atmosphere.bottomRadius);
	float rho = H * uv.y;
	viewHeight = sqrt(rho * rho + atmosphere.bottomRadius * atmosphere.bottomRadius);

	float dMin = atmosphere.topRadius - viewHeight;
	float dMax = rho + H;

	// Distance to atmosphere boundary
	float d = dMin + uv.x * (dMax - dMin);

	viewZenithCosAngle = (d == 0.0) ? 1.0f : (H * H - rho * rho - d * d) / (2.0 * viewHeight * d);
	viewZenithCosAngle = clamp(viewZenithCosAngle, -1.0, 1.0);
}

float fromUnitToSubUvs(float u, float resolution) { return (u + 0.5f / resolution) * (resolution / (resolution + 1.0f)); }
float fromSubUvsToUnit(float u, float resolution) { return (u - 0.5f / resolution) * (resolution / (resolution - 1.0f)); }

void skyViewLutParamsToUv(
	in const AtmosphereParameters atmosphere, 
	in bool  bIntersectGround, 
	in float viewZenithCosAngle, 
	in float lightViewCosAngle, 
	in float viewHeight, 
    in vec2 lutSize,
	out vec2 uv)
{
	float vHorizon = sqrt(viewHeight * viewHeight - atmosphere.bottomRadius * atmosphere.bottomRadius);

	// Ground to horizon cos.
	float cosBeta = vHorizon / viewHeight;		

	float beta = acos(cosBeta);
	float zenithHorizonAngle = kPI - beta;

	if (!bIntersectGround)
	{
		float coord = acos(viewZenithCosAngle) / zenithHorizonAngle;
		coord = 1.0 - coord;
		coord = sqrt(coord); // Non-linear sky view lut.

		coord = 1.0 - coord;
		uv.y = coord * 0.5f;
	}
	else
	{
		float coord = (acos(viewZenithCosAngle) - zenithHorizonAngle) / beta;
		coord = sqrt(coord); // Non-linear sky view lut.

		uv.y = coord * 0.5f + 0.5f;
	}

	// UV x remap.
	{
		float coord = -lightViewCosAngle * 0.5f + 0.5f;
		coord = sqrt(coord);
		uv.x = coord;
	}

	// Constrain uvs to valid sub texel range (avoid zenith derivative issue making LUT usage visible)
	uv = vec2(fromUnitToSubUvs(uv.x, lutSize.x), fromUnitToSubUvs(uv.y, lutSize.y));
}

// Air perspective.
const float kAirPerspectiveKmPerSlice = 4.0f; // total 32 * 4 = 128 km.
float aerialPerspectiveDepthToSlice(float depth) { return depth * (1.0f / kAirPerspectiveKmPerSlice); }
float aerialPerspectiveSliceToDepth(float slice) { return slice * kAirPerspectiveKmPerSlice; }

// NOTE: When offset height is big, this value should be more bigger.
// TODO: Compute this by camera height.
const float kPlanetRadiusOffset = 0.001f; // Offset 1 m.

// 相机相对行星中心位置 (km) → Sky-View 烘焙/采样观察点：沿径向抬高 0.5 km，
// 避免世界 +Y 偏移在非极点纬度扭曲当地天顶方向（贴地白色楔形）。
const float kAtmoCameraOffsetKm = 0.5f;

vec3 cameraPosToAtmosphereObserverKm(vec3 camRelPlanetKm)
{
	float len = length(camRelPlanetKm);
	vec3 radial = (len > 1e-4f) ? (camRelPlanetKm / len) : vec3(0.0f, 0.0f, 1.0f);
	return camRelPlanetKm + radial * kAtmoCameraOffsetKm;
}

// Camera unit to atmosphere unit convert. meter -> kilometers.
vec3 convertToAtmosphereUnit(vec3 o, in const PerFrameData frame)
{
	return cameraPosToAtmosphereObserverKm(o * 0.001f);
}  

// atmosphere unit to camera unit convert. kilometers -> meter.
vec3 convertToCameraUnit(vec3 o, in const PerFrameData frame)
{
	float len = length(o);
	vec3 radial = (len > 1e-4f) ? (o / len) : vec3(0.0f, 0.0f, 1.0f);
	return (o - radial * kAtmoCameraOffsetKm) * 1000.0f;
}  

// Participating media functions and struct.
// From https://github.com/sebh/UnrealEngineSkyAtmosphere
struct MediumSampleRGB
{
	vec3 scattering;
	vec3 absorption;
	vec3 extinction;

	vec3 scatteringMie;
	vec3 absorptionMie;
	vec3 extinctionMie;

	vec3 scatteringRay;
	vec3 absorptionRay;
	vec3 extinctionRay;

	vec3 scatteringOzo;
	vec3 absorptionOzo;
	vec3 extinctionOzo;

	vec3 albedo;
};

float getAlbedo(float scattering, float extinction)
{
	return scattering / max(0.001, extinction);
}

vec3 getAlbedo(vec3 scattering, vec3 extinction)
{
	return scattering / max(vec3(0.001), extinction);
}

float getViewHeight(vec3 worldPos, in const AtmosphereParameters atmosphere)
{
	// Current default set planet center is (0, 0, 0).
    // And we start from horizontal plane which treat as 0 plane on height.
	return length(worldPos) - atmosphere.bottomRadius;
}

bool moveToTopAtmosphere(inout vec3 worldPos, in const vec3 worldDir, in const float atmosphereTopRadius)
{
	float viewHeight = length(worldPos);
	if (viewHeight > atmosphereTopRadius)
	{
		float tTop = raySphereIntersectNearest(worldPos, worldDir, vec3(0.0), atmosphereTopRadius);
		if (tTop >= 0.0f)
		{
			vec3 upVector = worldPos / viewHeight;
			vec3 upOffset = upVector * -kPlanetRadiusOffset;
			worldPos = worldPos + worldDir * tTop + upOffset;
		}
		else
		{
			// Ray is not intersecting the atmosphere
			return false;
		}
	}
	return true; // ok to start tracing
}

MediumSampleRGB sampleMediumRGB(in vec3 worldPos, in const AtmosphereParameters atmosphere)
{
	const float viewHeight = getViewHeight(worldPos, atmosphere);

    // Get mie and ray density.
	const float densityMie = exp(atmosphere.mieDensityExpScale * viewHeight);
	const float densityRay = exp(atmosphere.rayleighDensityExpScale * viewHeight);

    // Get ozone density.
	const float densityOzo = saturate(viewHeight < atmosphere.absorptionDensity0LayerWidth ?
		atmosphere.absorptionDensity0LinearTerm * viewHeight + atmosphere.absorptionDensity0ConstantTerm :
		atmosphere.absorptionDensity1LinearTerm * viewHeight + atmosphere.absorptionDensity1ConstantTerm);

    // Build medium sample.
	MediumSampleRGB s;

    // Mie term.
	s.scatteringMie = densityMie * atmosphere.mieScattering;
	s.absorptionMie = densityMie * atmosphere.mieAbsorption;
	s.extinctionMie = densityMie * atmosphere.mieExtinction;

    // Ray term.
	s.scatteringRay = densityRay * atmosphere.rayleighScattering;
	s.absorptionRay = vec3(0.0);
	s.extinctionRay = s.scatteringRay + s.absorptionRay;

    // Ozone term.
	s.scatteringOzo = vec3(0.0);
	s.absorptionOzo = densityOzo * atmosphere.absorptionExtinction;
	s.extinctionOzo = s.scatteringOzo + s.absorptionOzo;

    // Composite.
	s.scattering = s.scatteringMie + s.scatteringRay + s.scatteringOzo;
	s.absorption = s.absorptionMie + s.absorptionRay + s.absorptionOzo;
	s.extinction = s.extinctionMie + s.extinctionRay + s.extinctionOzo;
	s.albedo = getAlbedo(s.scattering, s.extinction);
	return s;
}

// ── 实时（非 LUT）大气透射率 ───────────────────────────────────────────────────
// 用于云管线放弃 Transmittance LUT 之后的替代方案：从 posKm（行星中心为原点，km）
// 沿 dir 方向短程数值积分到大气顶，累积 Rayleigh+Mie+臭氧消光，返回 exp(-光学深度)。
// 直接复用 sampleMediumRGB 的指数密度模型——和 fillPlanetAtmosphereProfile() 喂给
// AtmosphereConfig 的是同一套系数，天然和 cloud_rs3d_common.glsl 的实时大气一致，
// 不需要另外维护一张预烘焙表，也没有"该用相机高度还是采样点高度"的参数化问题
// （因为直接从 posKm 本身出发积分，没有 LUT 那种和位置脱钩的查表环节）。
vec3 realTimeTransmittance(vec3 posKm, vec3 dir, in const AtmosphereParameters atmosphere, int steps)
{
	float tTop = raySphereIntersectNearest(posKm, dir, vec3(0.0), atmosphere.topRadius);
	if (tTop < 0.0)
	{
		return vec3(1.0); // 方向不朝大气内（已经在大气外侧背向），视为无遮挡
	}

	float dt = tTop / float(steps);
	vec3 opticalDepth = vec3(0.0);
	float t = dt * 0.5;
	for (int i = 0; i < steps; i++)
	{
		vec3 p = posKm + dir * t;
		MediumSampleRGB medium = sampleMediumRGB(p, atmosphere);
		opticalDepth += medium.extinction * dt;
		t += dt;
	}
	return exp(-opticalDepth);
}

// ── 实时天空环境色近似 ─────────────────────────────────────────────────────────
// 不做完整大气积分（那样等于把 Transmittance/MultiScatter/SkyView LUT 的活又干一遍，
// 只是换成每像素实时算，没有省掉复杂度），改用和 cloud_rs3d_lighting.glsl 里
// cloud.frag 同一套手调的日/昏/夜三色混合——艺术近似，但和现有 rs3d 云在视觉上一致，
// 参数只需要"当地太阳高度角"一个输入。
vec3 analyticSkyAmbient(float sunElevation) // sunElevation = dot(localUp, sunDir)
{
	vec3 skyDay   = vec3(0.62, 0.70, 0.92);
	vec3 skyDusk  = vec3(1.05, 0.40, 0.12);
	vec3 skyNight = vec3(0.02, 0.025, 0.04);

	float skyVis = smoothstep(-0.5, 0.15, sunElevation);
	float twilightLocal = smoothstep(0.22, -0.06, sunElevation);

	return mix(mix(skyNight, skyDay, skyVis), skyDusk, twilightLocal * skyVis);
}

// ── Inscatter 4D LUT（Phase B：Bruneton 非线性映射）────────────────────────────
// 参数 (r, mu, mu_s, nu) 打包进 3D（与 Bruneton SCATTERING 表同构）：
//   width  = RES_MU_S * RES_NU
//   height = RES_MU   （上半：不击地；下半：击地）
//   depth  = RES_R
// 映射对齐 GetScatteringTextureUvwzFromRMuMuSNu /
// GetRMuMuSNuFromScatteringTextureUvwz（functions.glsl）。
const int kInscatterResR    = 32;
const int kInscatterResMu   = 128;
const int kInscatterResMuS  = 32;
const int kInscatterResNu   = 8;
// Bruneton AtmosphereParameters::mu_s_min：太阳天顶角下限（略低于地平线）。
const float kInscatterMuSMin = -0.2;

float inscatterTexCoordFromUnit(float x, int size)
{
	return 0.5 / float(size) + x * (1.0 - 1.0 / float(size));
}

float inscatterUnitFromTexCoord(float u, int size)
{
	return (u - 0.5 / float(size)) / (1.0 - 1.0 / float(size));
}

float inscatterSafeSqrt(float a)
{
	return sqrt(max(a, 0.0));
}//返回根号a，如果a小于0就返回0

// DistanceToTopAtmosphereBoundary(r, mu) — Bruneton
float inscatterDistanceToTop(float topRadius, float r, float mu)
{
	float discriminant = r * r * (mu * mu - 1.0) + topRadius * topRadius;
	return max(0.0, -r * mu + inscatterSafeSqrt(discriminant));
}

bool inscatterRayIntersectsGround(in const AtmosphereParameters atmosphere, float r, float mu)
{
	return mu < 0.0 &&
		(r * r * (mu * mu - 1.0) + atmosphere.bottomRadius * atmosphere.bottomRadius) >= 0.0;
}

// (r,mu,mu_s,nu) → 归一化 4D 坐标 (u_nu, u_mu_s, u_mu, u_r)
// 对齐 Bruneton GetScatteringTextureUvwzFromRMuMuSNu。
vec4 inscatterParamsToUvwz(in const AtmosphereParameters atmosphere,
	float r, float mu, float mu_s, float nu, bool rayIntersectsGround)
{
	float H = sqrt(max(0.0,
		atmosphere.topRadius * atmosphere.topRadius -
		atmosphere.bottomRadius * atmosphere.bottomRadius));
	float rho = inscatterSafeSqrt(r * r - atmosphere.bottomRadius * atmosphere.bottomRadius);
	float u_r = inscatterTexCoordFromUnit(clamp(rho / max(H, 1e-6), 0.0, 1.0), kInscatterResR);

	float r_mu = r * mu;
	float discriminant = r_mu * r_mu - r * r + atmosphere.bottomRadius * atmosphere.bottomRadius;
	float u_mu;
	if (rayIntersectsGround)
	{
		float d = -r_mu - inscatterSafeSqrt(discriminant);
		float d_min = r - atmosphere.bottomRadius;
		float d_max = rho;
		float x = (d_max == d_min) ? 0.0 : clamp((d - d_min) / max(d_max - d_min, 1e-6), 0.0, 1.0);
		u_mu = 0.5 - 0.5 * inscatterTexCoordFromUnit(x, kInscatterResMu / 2);
	}
	else
	{
		float d = -r_mu + inscatterSafeSqrt(discriminant + H * H);
		float d_min = atmosphere.topRadius - r;
		float d_max = rho + H;
		float x = clamp((d - d_min) / max(d_max - d_min, 1e-6), 0.0, 1.0);
		u_mu = 0.5 + 0.5 * inscatterTexCoordFromUnit(x, kInscatterResMu / 2);
	}

	// mu_s：到大气顶距离参数化，地平线附近加密
	float d = inscatterDistanceToTop(atmosphere.topRadius, atmosphere.bottomRadius, mu_s);
	float d_min = atmosphere.topRadius - atmosphere.bottomRadius;
	float d_max = H;
	float a = (d - d_min) / max(d_max - d_min, 1e-6);
	float D = inscatterDistanceToTop(atmosphere.topRadius, atmosphere.bottomRadius, kInscatterMuSMin);
	float A = (D - d_min) / max(d_max - d_min, 1e-6);
	float u_mu_s = inscatterTexCoordFromUnit(
		clamp(max(1.0 - a / max(A, 1e-6), 0.0) / (1.0 + a), 0.0, 1.0),
		kInscatterResMuS);

	float u_nu = clamp(nu * 0.5 + 0.5, 0.0, 1.0);
	return vec4(u_nu, u_mu_s, u_mu, u_r);
}

// 归一化 4D 坐标 → (r,mu,mu_s,nu)；对齐 GetRMuMuSNuFromScatteringTextureUvwz。
void inscatterUvwzToParams(in const AtmosphereParameters atmosphere,
	vec4 uvwz,
	out float r, out float mu, out float mu_s, out float nu,
	out bool rayIntersectsGround)
{
	float H = sqrt(max(0.0,
		atmosphere.topRadius * atmosphere.topRadius -
		atmosphere.bottomRadius * atmosphere.bottomRadius));
	float rho = H * inscatterUnitFromTexCoord(uvwz.w, kInscatterResR);
	r = sqrt(rho * rho + atmosphere.bottomRadius * atmosphere.bottomRadius);

	if (uvwz.z < 0.5)
	{
		float d_min = r - atmosphere.bottomRadius;
		float d_max = rho;
		float d = d_min + (d_max - d_min) * inscatterUnitFromTexCoord(
			1.0 - 2.0 * uvwz.z, kInscatterResMu / 2);
		mu = (d == 0.0) ? -1.0 :
			clamp(-(rho * rho + d * d) / (2.0 * r * d), -1.0, 1.0);
		rayIntersectsGround = true;
	}
	else
	{
		float d_min = atmosphere.topRadius - r;
		float d_max = rho + H;
		float d = d_min + (d_max - d_min) * inscatterUnitFromTexCoord(
			2.0 * uvwz.z - 1.0, kInscatterResMu / 2);
		mu = (d == 0.0) ? 1.0 :
			clamp((H * H - rho * rho - d * d) / (2.0 * r * d), -1.0, 1.0);
		rayIntersectsGround = false;
	}

	float x_mu_s = inscatterUnitFromTexCoord(uvwz.y, kInscatterResMuS);
	float d_min = atmosphere.topRadius - atmosphere.bottomRadius;
	float d_max = H;
	float D = inscatterDistanceToTop(atmosphere.topRadius, atmosphere.bottomRadius, kInscatterMuSMin);
	float A = (D - d_min) / max(d_max - d_min, 1e-6);
	float a = (A - x_mu_s * A) / (1.0 + x_mu_s * A);
	float d = d_min + min(a, A) * (d_max - d_min);
	mu_s = (d == 0.0) ? 1.0 :
		clamp((H * H - d * d) / (2.0 * atmosphere.bottomRadius * d), -1.0, 1.0);

	nu = clamp(uvwz.x * 2.0 - 1.0, -1.0, 1.0);
}

// 从 texel 索引解码 (r, mu, mu_s, nu)。r 为地心距 km。
// 对齐 Bruneton GetRMuMuSNuFromScatteringTextureFragCoord（半 texel 中心）。
void inscatterTexelToParams(in const AtmosphereParameters atmosphere,
	ivec3 texel, ivec3 lutSize,
	out float r, out float mu, out float mu_s, out float nu,
	out bool rayIntersectsGround)
{
	float fx = float(texel.x) + 0.5;
	float fy = float(texel.y) + 0.5;
	float fz = float(texel.z) + 0.5;
	float frag_nu   = floor(fx / float(kInscatterResMuS));
	float frag_mu_s = mod(fx, float(kInscatterResMuS));
	vec4 uvwz = vec4(frag_nu, frag_mu_s, fy, fz) /
		vec4(float(max(kInscatterResNu - 1, 1)), float(kInscatterResMuS),
			float(kInscatterResMu), float(kInscatterResR));

	inscatterUvwzToParams(atmosphere, uvwz, r, mu, mu_s, nu, rayIntersectsGround);

	// nu 与 (mu, mu_s) 几何相容
	float muBound = mu * mu_s + sqrt(max(0.0, (1.0 - mu * mu) * (1.0 - mu_s * mu_s)));
	nu = clamp(nu, -muBound, muBound);
}

// 4D 归一化坐标打包为 3D 采样 UV（nu 维双线性）；对齐 Bruneton GetScattering。
void inscatterUvwzToPackedUvw(vec4 uvwz, out vec3 uvw0, out vec3 uvw1, out float nuLerp)
{
	float tex_coord_x = uvwz.x * float(kInscatterResNu - 1);
	float tex_x = floor(tex_coord_x);
	nuLerp = tex_coord_x - tex_x;
	uvw0 = vec3((tex_x + uvwz.y) / float(kInscatterResNu), uvwz.z, uvwz.w);
	uvw1 = vec3((tex_x + 1.0 + uvwz.y) / float(kInscatterResNu), uvwz.z, uvwz.w);
}

void inscatterParamsToUvw(in const AtmosphereParameters atmosphere,
	float r, float mu, float mu_s, float nu, bool rayIntersectsGround,
	out vec3 uvw0, out vec3 uvw1, out float nuLerp)
{
	vec4 uvwz = inscatterParamsToUvwz(atmosphere, r, mu, mu_s, nu, rayIntersectsGround);
	inscatterUvwzToPackedUvw(uvwz, uvw0, uvw1, nuLerp);
}

// 由 (mu, mu_s, nu) 构造视线/太阳方向（zenith=+Y）。sunDir 指向太阳。
void inscatterDirsFromParams(float mu, float mu_s, float nu, out vec3 viewDir, out vec3 sunDir)
{
	float sinM = sqrt(max(0.0, 1.0 - mu * mu));
	viewDir = vec3(sinM, mu, 0.0);

	float sinMs = sqrt(max(0.0, 1.0 - mu_s * mu_s));
	float cosPhi = 0.0;
	float denom = max(sinM * sinMs, 1e-4);
	cosPhi = clamp((nu - mu * mu_s) / denom, -1.0, 1.0);
	float sinPhi = sqrt(max(0.0, 1.0 - cosPhi * cosPhi));
	sunDir = vec3(sinMs * cosPhi, mu_s, sinMs * sinPhi);
}

vec3 sampleInscatterLut(texture3D lut, sampler s,
	in const AtmosphereParameters atmosphere,
	float r, float mu, float mu_s, float nu)
{
	r = clamp(r, atmosphere.bottomRadius, atmosphere.topRadius);
	mu = clamp(mu, -1.0, 1.0);
	mu_s = clamp(mu_s, kInscatterMuSMin, 1.0);
	float muBound = mu * mu_s + sqrt(max(0.0, (1.0 - mu * mu) * (1.0 - mu_s * mu_s)));
	nu = clamp(nu, -muBound, muBound);

	bool hitGround = inscatterRayIntersectsGround(atmosphere, r, mu);
	vec3 uvw0, uvw1;
	float nuLerp;
	inscatterParamsToUvw(atmosphere, r, mu, mu_s, nu, hitGround, uvw0, uvw1, nuLerp);
	return texture(sampler3D(lut, s), uvw0).rgb * (1.0 - nuLerp) +
		texture(sampler3D(lut, s), uvw1).rgb * nuLerp;
}

#endif