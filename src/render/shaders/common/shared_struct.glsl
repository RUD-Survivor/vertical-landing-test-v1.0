#ifndef SHARED_STRUCT_GLSL
#define SHARED_STRUCT_GLSL

#define kPI 3.141592653589793

const float kLog2 = log(2.0);

#define kPositionStrip 3
#define kNormalStrip   3
#define kUv0Strip      2
#define kTangentStrip  4

const float kMaxHalfFloat   = 65504.0f;
const float kMax11BitsFloat = 65024.0f;
const float kMax10BitsFloat = 64512.0f;
const vec3  kMax111110BitsFloat3 = vec3(kMax11BitsFloat, kMax11BitsFloat, kMax10BitsFloat);

struct CascadeShadowConfig
{
    int cascadeCount;
    int percascadeDimXY;
    float cascadeSplitLambda;
    float maxDrawDepthDistance;

    float shadowBiasConst; 
    float shadowBiasSlope; 
    float shadowFilterSize;
    float maxFilterSize;

    float cascadeBorderAdopt;
    float cascadeEdgeLerpThreshold;
    float pad0;
    float pad1;
};

// All units in kilometers
struct AtmosphereConfig
{
    float atmospherePreExposure;
    float pad0;
    float pad1;
    float pad2;

    vec3 absorptionColor;
    float absorptionLength;

	vec3 rayleighScatteringColor;
    float rayleighScatterLength;

    float multipleScatteringFactor;  
	float miePhaseFunctionG; 
    float bottomRadius; 
	float topRadius;

    vec3 mieScatteringColor;
    float mieScatteringLength;

    vec3 mieAbsColor;
    float mieAbsLength;

	vec3 mieAbsorption;  
	int viewRayMarchMinSPP;

	vec3 groundAlbedo;  
	int viewRayMarchMaxSPP;

	vec4 rayleighDensity[3];
	vec4 mieDensity[3]; 
	vec4 absorptionDensity[3];

    // Clout infos.
    float cloudAreaStartHeight; // km
    float cloudAreaThickness;
    float cloudGodRayScale;
    float cloudShadowExtent; // x4

    vec3 camWorldPos; // cameraworld Position, in atmosphere space unit.
    uint updateFaceIndex; // flower: cubemap face; RS3D Cloud Tuner: 1=1/4 Bayer, 0=full-res raymarch

    // World space to cloud space view project matrix. Unit also is km.
    mat4 cloudSpaceViewProject;
    mat4 cloudSpaceViewProjectInverse;

    // Cloud settings.
    vec2  cloudWeatherUVScale;
    float cloudCoverage;
    float cloudDensity;

    float cloudShadingSunLightScale;
    float cloudFogFade; 
    float cloudMaxTraceingDistance; 
    float cloudTracingStartMaxDistance; 

    vec3 cloudDirection;
    float cloudSpeed;

    float cloudMultiScatterExtinction;
    float cloudMultiScatterScatter;
    float cloudBasicNoiseScale;
    float cloudDetailNoiseScale;

    vec3  cloudAlbedo;
    float cloudPhaseForward;

    float cloudPhaseBackward;
    float cloudPhaseMixFactor;
    float cloudPowderScale;
    float cloudPowderPow;

    // Powder 模式：0=Off 1=Legacy(powderEffectNew) 2=Sugar(堆糖 2·Beer·(1-e^{-2τ}))
    int   cloudPowderMode;
    int   cloudPowderSugarAllMs;        // Sugar：0=仅 ms0，1=每阶都乘
    float cloudPowderSugarGain;         // 原文 ×2；可调
    float cloudPowderSugarViewWeight;   // 0=按 -VoL 门控，1=全向套用原文

    float cloudLightStepMul;
    float cloudLightBasicStep;
    int  cloudLightStepNum;
    int cloudEnableGroundContribution;

    int cloudMarchingStepNum;
    int cloudSunLitMapOctave;
    float cloudNoiseScale;
    int cloudGodRay;

    // RS3D Cloud Tuner：第一层自适应步长（cloud_common.glsl cloudColorCompute）
    float cloudMarchStepMinKm;
    float cloudMarchStepMaxKm;
    int   cloudMarchStepHardMax;
    // 第二层：壳内真空跳步（仅 density==0 时大步；薄云不省光照）
    int   cloudMarchEmptySkip;
    float cloudMarchEmptyStepKm;
    float cloudMarchEmptyTauMax; // 空步长蓝噪声抖动幅度 ±(0~0.5)
    // 第三层：云缘密度梯度缩步（见 cloud_common.glsl 主 march 循环）
    int   cloudMarchEdgeRefine;     // 0=关，非0=开
    float cloudMarchEdgeScaleMin;   // 边缘处步长 = stepT×此值（下限 stepTMin）
    float cloudMarchEdgeGradLow;    // τ 跳变 smoothstep 下界（density×km 量纲，归一化 per km），小于它认为不是云的边缘
    float cloudMarchEdgeGradHigh;   // τ 跳变 smoothstep 上界，大于它认为是边缘，中间平滑取值

    // Froxel 空气透视（LUT 为单位太阳；运行时 × L_sun × SunScale）
    int   cloudAirPerspectiveEnabled; // 0=跳过空气透视
    float cloudAirPerspectiveScale;   // AP 强度 [0,1]（1=全物理 L'=L_air+T_air×L）
    float cloudAirPerspectiveSunScale;// 相对 L_sun 的 L_air 倍率（默认 1）
    float cloudAmbientExposure;       // 环境光相对倍率（LUT×L_sun 之后再乘）
    float cloudAmbientAnalyticBlend;  // 与 analyticSkyAmbient 混合 [0,1]
    int   cloudIrradianceBakeAtShell; // 保留布局；SkyView 烘焙高度（atmo 路径）
    // Curl 假资产 localCoverage（云管线；复用原 padAmbient 槽位，保 632B）
    float cloudCurlUVScale;           // UV = km * scale（旧硬编码 0.001）
    float cloudCurlStrength;          // 叠到 weather.x；0=关
    float cloudCurlContrast;          // saturate(x*c - c*0.25)*strength

    // 噪声频带阶梯（cloud_common.glsl::cloudMap）。形状链 Basic/Meso/Fine 共用同一张
    // 128³ inBasicNoise，只换世界频率（单位 1/km，纹理平铺周期 = 1/scale km）；
    // 侵蚀链用 32³ inDetailNoise。Basic 幅度恒为 1，其余是相对它的权重。
    // 幅度全为 0 时退化成"单档 Basic + 无侵蚀"，可当作频带 LOD 的关闭档。
    float cloudMesoNoiseScale;
    float cloudMesoAmplitude;
    float cloudFineNoiseScale;
    float cloudFineAmplitude;

    float cloudDetailAmplitude;       // 侵蚀幅度：remap 的下限阈值上界
    // 频带 LOD（cloud_noise_lod.glsl）：按采样点到相机距离的屏幕足迹淡出高频档。
    // 与自适应步长无关——步长由 empty skip / edge refine 单独管，两套不互相挂钩。
    float cloudNoiseLodNMin;             // 一个特征周期至少要占几个像素才保留该档
    float cloudNoiseLodShadowDistScale;  // 阴影侧的有效距离倍率（>1 = 阴影更早关细档）
    int   cloudNoiseLodEnabled;          // 0=权重恒 1，退化成"全档常开"（回归开关）

    // ── phi_fwd 物理漫射场（cloud_common.glsl::volumetricShadow / cloudColorCompute）──
    // 扩散近似（τ≫1、g_eff→0）下的各向同性多重散射，沿太阳光步用 1D 格林函数累加，
    // 作为与 Frostbite MsCount 假 MS 正交的「加性」散射项。Intensity=0 时整条路径跳过。
    float cloudPhiFwdIntensity;          // 标定强度（非 0~1）；0=关。吸收 1/(4πD) 等，常用 10~1000
    float cloudPhiFwdMsBuildScale;       // C_iso：各向同性 MS 建立速度（越大越早转漫射）
    float cloudPhiFwdCompress;           // 软饱和压缩；0=线性不压缩
    float cloudPhiFwdDepthPow;           // C_bottom：底部置信度恢复速度；0=禁用底部软化
    float cloudPhiFwdDepthBias;          // C_bottom：底部锚点偏移（>0 云底保留更多漫射）
    float cloudPhiFwdBoundaryConfidence; // C_top：云顶受光置信度强度（0=禁用，1=全量）

    // ── 双层云：Profile LUT 类型轴 + 高空云（第二层） ────────────────────────
    // Profile LUT（inCloudProfileLut，binding 31）：U=归一化高度，V=径向距离，
    // RGB=Cu/Tcu/Cb 三条垂直廓形（近似 LWC 钟形，峰值中上部、顶部归零）。
    // 本组把 weather.g 的 cloudType 重新映射到 LUT 类型轴（0=Cu,0.5=Tcu,1=Cb）。
    // 廓形即 LUT 曲线本身，不设运行时倍率（旧 cloudProfileScale 乘在 coverage
    // remap 之前会悄悄改变云体覆盖率，已移除；提亮/压暗请用 cloudDensity）。
    float cloudProfileTypeBias;      // cloudType 偏移（先减 0.5 再乘对比度后加回 0.5+此值）
    float cloudProfileTypeContrast;  // cloudType 对比度（>1 拉开类型、<1 收敛到中部）
    float cloudProfilePad0;
    float cloudProfilePad1;

    // 高空云（inCloudHiWeather，binding 32）：R=HiMask（高空云覆盖）G=HiType(As/Ac)。
    // 密度纯 2D，与低云同 σ 单位，max() 进 cloudMap 共用主 march（见 cloud_high_layer.glsl）。
    float hiCloudCoverage;           // 高空云全局覆盖倍率（0=关闭第二层）
    float hiCloudBottom;             // 高空云带底（slab 归一化高度 0~1）
    float hiCloudTop;                // 高空云带顶（slab 归一化高度 0~1）
    float hiCloudDensity;            // 高空云密度（与低云 cloudMap 同单位）

    float hiCloudThreshold;          // hi 覆盖阈值（低于此值密度=0）
    float hiCloudSoftness;           // 阈值过渡宽度
    float hiCloudType;               // 高空云类型偏好（0=As，1=Ac；与天气图 G 各取一半）
    // 原 hiCloudPad0 + cloudProfilePad2/3/4（16B）已挪到 cloudPowderMode 块，保 752B。
    // 本段 11×float=44B；前段含 powder 模式块 708B → 752B（=47×16）。
};

struct SkyInfo
{
    vec3  color;
    float intensity;

    vec3  direction;
    int  shadowType; // Shadow type of this sky light.

    int rayTraceShadow; // = 0 is false, = 1 is true;
    int pad0;
    int pad1;
    int pad2;

    CascadeShadowConfig cacsadeConfig;
    AtmosphereConfig atmosphereConfig;
};

struct PerFrameData
{
    // .x is app runtime, .y is sin(.x), .z is cos(.x), .w is pad
    vec4 appTime;

    // .x is frame count, .y is frame count % 8, .z is frame count % 16, .w is frame count % 32
    uvec4 frameIndex;

    // Camera world space position.
    vec4 camWorldPos;

    // .xyz is camera forward.
    vec4 camForward;

    // .x fovy, .y aspectRatio, .z nearZ, .w farZ
    vec4 camInfo;
    
    // prev-frame's cam info.
    vec4 camInfoPrev;

    // Camera matrixs.
    mat4 camView;
    mat4 camProj;
    mat4 camViewProj;

    // Camera inverse matrixs.
    mat4 camInvertView;
    mat4 camInvertProj;
    mat4 camInvertViewProj;

    // Camera matrix remove jitter effects.
    mat4 camProjNoJitter;
    mat4 camViewProjNoJitter;

    // Camera invert matrixs no jitter effects.
    mat4 camInvertProjNoJitter;
    mat4 camInvertViewProjNoJitter;

    // Prev-frame camera infos.
    mat4 camViewProjPrev;
    mat4 camViewProjPrevNoJitter;

    // Camera frustum planes for culling.
    vec4 frustumPlanes[6];

    // Halton sequence jitter data, .xy is current frame jitter data, .zw is prev frame jitter data.
    vec4 jitterData;
    
    uint  jitterPeriod;        // jitter period for jitter data.
    uint  bEnableJitter;       // Is main camera enable jitter in this frame.
    float basicTextureLODBias; // Lod basic texture bias when render mesh, used when upscale need.
    uint  bCameraCut;          // Camera cut in this frame or not.

    uint skyValid; // sky is valid.
    uint skySDSMValid;
    float fixExposure;
    uint bAutoExposure;

    float renderWidth;
    float renderHeight;
    float displayWidth;
    float displayHeight;

    SkyInfo sky;
};

// See GPUStaticMeshStandardPBR in shader_struct.h
struct MaterialStandardPBR
{
    uint baseColorId;
    uint baseColorSampler;
    uint normalTexId;   
    uint normalSampler;

    uint specTexId; 
    uint specSampler;
    uint occlusionTexId; 
    uint occlusionSampler;

    uint emissiveTexId; 
    uint emissiveSampler;
    float cutoff;
    // > 1.0f is backface cut, < -1.0f is frontface cut, [-1.0f, 1.0f] is no face cut.
    float faceCut; 

    // Base color operator.
    vec4 baseColorMul;
    vec4 baseColorAdd;

    // Metal operator.
    float metalMul;
    float metalAdd;
    // Roughness operator.
    float roughnessMul;
    float roughnessAdd;

    // Emissive operator.
    vec4 emissiveMul;
    vec4 emissiveAdd;

    float shadeingModel;
    float pad0;
    float pad1;
    float pad2;
};

// Memory layout same with mesh_misc.h
// Raw data of one static mesh vertex.
struct StaticMeshVertexRaw
{
    vec4 x0;
    vec4 x1;
    vec4 x2;
};
// Static mesh vertex.
struct StaticMeshVertex
{
    vec3 position;
    vec3 normal;
    vec4 tangent;
    vec2 uv0;
};
StaticMeshVertex buildVertex(in const StaticMeshVertexRaw raw)
{
    StaticMeshVertex result;
    result.position = raw.x0.xyz;

    result.normal.x  = raw.x0.w;
    result.normal.yz = raw.x1.xy;

    result.tangent.xy = raw.x1.zw;
    result.tangent.zw = raw.x2.xy;
    
    result.uv0 = raw.x2.zw;
    return result;
}

// Same with cpp.
#define SMT_StaticMesh    0
#define SMT_PMXStaticMesh 1

struct StaticMeshPerObjectData
{
    // Material for static mesh.
    MaterialStandardPBR material;

    // Current-frame model matrix.
    mat4 modelMatrix;

    // Prev-frame model matrix.
    mat4 modelMatrixPrev; 
   
    uint uv0sArrayId;    // Vertices buffer in bindless buffer id.    
    uint positionsArrayId;   // Positions buffer in bindless buffer id.
    uint indicesArrayId;     // Indices buffer in bindless buffer id.
    uint indexStartPosition; // Index start offset position.

    vec4 sphereBounds;

    vec3 extents;
    uint indexCount;         // Mesh object info, used to build draw calls.

    uint sceneNodeId; // Object id of scene node.
    uint bSelected; // Can pack with sceneNodeId like shader.
    uint tangentsArrayId;
    uint normalsArrayId;

    uint positionsPrevArrayId;
    uint objectType; // == 0 is static mesh, == 1 is pmx static mesh.
    uint smoothNormalArrayId;
    uint pad1;
};

/**
*   typedef struct VkDrawIndexedIndirectCommand {
*       uint32_t    indexCount;
*       uint32_t    instanceCount;
*       uint32_t    firstIndex;
*       int32_t     vertexOffset;
*       uint32_t    firstInstance;
*   } VkDrawIndexedIndirectCommand;
*
*   typedef struct VkDrawIndirectCommand {
*       uint32_t    vertexCount;
*       uint32_t    instanceCount;
*       uint32_t    firstVertex;
*       uint32_t    firstInstance;
*   } VkDrawIndirectCommand;
**/
struct StaticMeshDrawCommand
{
    // VkDrawIndirectCommand

    // Build draw call data.
    uint vertexCount;
    uint instanceCount;
    uint firstVertex;
    uint firstInstance;

    // Object id for StaticMeshPerObjectData array indexing.
    uint objectId;
    uint pad0;
    uint pad1;
    uint pad2;
};
StaticMeshDrawCommand defaultStaticMeshDrawCommand()
{
    StaticMeshDrawCommand result;

    result.vertexCount   = 0;
    result.instanceCount = 0;
    result.firstVertex   = 0;
    result.firstInstance = 0;
    result.objectId      = 0;

    return result;
}
// We use vkDrawIndirect to draw all vertex.
// Instead of binding index buffer, we load vertex data from index buffer.
// So the index count same with vertex count.

struct CascadeInfo
{
    mat4 viewProj;
    vec4 frustumPlanes[6];
    vec4 cascadeScale;
};

struct Ray
{
	vec3 o;
	vec3 d;
};

Ray createRay(in vec3 p, in vec3 d)
{
	Ray r;
	r.o = p;
	r.d = d;
	return r;
}

struct ScreenSpaceRay 
{
    vec3 ssRayStart;
    vec3 ssRayEnd;
    vec3 ssViewRayEnd;
    vec3 uvRayStart;
    vec3 uvRay;
};

struct DispatchIndirectCommand 
{
    uint x;
    uint y;
    uint z;
    uint pad;
};

struct DepthRange
{
    uint minDepth;
    uint maxDepth;
};

#endif