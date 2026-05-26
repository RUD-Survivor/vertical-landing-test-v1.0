#pragma once
// ============================================================================
// scene_snapshot.h — 渲染快照：Scene → Renderer 的中间表示
//
// 设计原则：
//   1. Scene 只提取数据，不调用任何 GL/Vulkan API
//   2. Renderer 只消费 Snapshot，不访问 ECS/GameContext
//   3. 同一个 Snapshot 可被 OpenGL 和 Vulkan 路径共享
//   4. 纯数据（POD + vector），可 copy、可序列化、可 mock 测试
// ============================================================================

#include "math/math3d.h"
#include <vector>
#include <cstring>

// ============================================================================
// 单个天体的渲染数据（行星/卫星/太阳）
// ============================================================================
struct CelestialDraw {
    float model[16];      // 世界矩阵（列主序，Mat4::toFloatArray 格式）
    float center[3];      // 球心世界坐标（大气着色器用）
    float radius;         // 渲染单位半径（用于大气厚度计算 + ring 归一化）
    float r, g, b;        // 基础颜色
    const char* shaderKey; // "earth"|"jupiter"|"saturn"|"uranus"|"neptune"|
                           // "mercury"|"venus"|"moon"|"mars"|"gas_giant"|"barren"
    int   bodyIdx;        // 1~9（对应 UniverseModel 索引，0=太阳）
    int   atmoIdx;        // 0=无大气，2=Venus, 3=Earth, 5=Mars, 6=Jupiter, 7=Saturn, 8=Uranus, 9=Neptune
    bool  hasRing;        // 土星环
    bool  showClouds;      // 是否渲染大气云层
};

// ============================================================================
// 单个排气羽流的渲染数据
// ============================================================================
struct ExhaustDraw {
    float model[16];      // TRS 矩阵（位置=喷管中点，旋转=喷管朝向，缩放=dia×len×dia）
    float throttle;       // 0~1
    float expansion;      // 大气膨胀系数（0=真空, 1=海平面）
    float groundDist;     // 到地面的距离
    float plumeLen;       // 羽流长度
};

// ============================================================================
// 轨迹丝带数据
// ============================================================================
struct RibbonSegment {
    std::vector<Vec3> points;   // 世界空间点
    std::vector<Vec4> colors;   // per-vertex 颜色
    float width;                // 丝带宽度（渲染单位）
};

// ============================================================================
// 广告牌数据（变轨节点手柄等）
// ============================================================================
struct BillboardDraw {
    float pos[3];       // 世界空间位置
    float size;         // 大小
    float r, g, b, a;   // 颜色
};

// ============================================================================
// 发射台渲染数据
// ============================================================================
struct LaunchPadDraw {
    float model[16];       // TRS 矩阵
    bool  useObjMesh = false;  // true=使用 "launchPad" OBJ 网格；false=使用 rocketBox 程序网格
};

// ============================================================================
// 太阳球体数据
// ============================================================================
struct SunBodyDraw {
    float model[16];    // TRS 矩阵
    float radius;       // 渲染半径
};

// ============================================================================
// 云调试面板状态
// ============================================================================
struct CloudTunerState {
    bool visible;
    float covLo, covHi, threshLo, threshHi;
    float erosion, density, extinction, minAlt, maxAlt;
};

// ============================================================================
// SVO 区块数据 (Block D)
// ============================================================================
struct SVOChunkDraw {
    float svoMat[16];     // 局部到相机相对坐标的变换矩阵
    uint32_t indexCount;  // EBO 索引数
    // 注意：实际 VBO/EBO 由 renderer 持有，此处只存标识
};

// ============================================================================
// 地形补丁数据 (Block D)
// ============================================================================
struct TerrainPatchDraw {
    float model[16];           // 世界矩阵（含行星旋转，translation = 行星世界位置）
    float planetRadius;        // 星球半径（shader 归一化用）
    float maxElev;             // 最大海拔（km）
    float time;                // 时间
    // 以下字段供 Vulkan terrain shader 使用
    float nodeCenter[3];       // node->center（单位立方体面中心）
    float nodeSideA[3];        // node->sideA（水平跨度向量）
    float nodeSideB[3];        // node->sideB（垂直跨度向量）
    float planetCenterRel[3];  // 行星中心相对相机位置（world center - cam pos）
    int   nodeLevel;           // LOD 层级
};

// ============================================================================
// 火箭零件渲染数据
// ============================================================================
struct RocketPartDraw {
    float model[16];      // TRS 矩阵
    float r, g, b;        // 颜色
    int   meshType;       // 0=body cylinder, 1=nose cone, 2=box（meshId 为空时使用）
    std::string meshId;   // 非空时优先使用此 Vulkan mesh ID（OBJ 模型路径）
};

// ============================================================================
// 每帧的完整渲染快照
// ============================================================================
struct SceneSnapshot {
    // ---- 相机 ----
    float view[16];
    float proj[16];
    float camPos[3];
    float lightDir[3];    // 归一化光照方向
    float time;

    // ---- 天体 ----
    std::vector<CelestialDraw> celestials;

    // ---- 火箭 ----
    float rocketBodyModel[16];  // 保留兜底
    float rocketNoseModel[16];
    std::vector<RocketPartDraw> rocketParts;  // 全零件渲染

    // ---- 排气羽流 ----
    std::vector<ExhaustDraw> plumes;

    // ---- 天空 ----
    float skyVibrancy;
    int   frameIndex;

    // ---- 太阳 ----
    float sunScreen[2];
    float sunIntensity;
    float sunWorldPos[3];  // 镜头光晕遮挡测试用
    float aspect;
    float dayBlend;         // 大气 sunVisibility（含遮挡）
    int   cameraMode;       // 0=Orbit 1=Chase 2=Panorama 3=Free

    // ---- 镜头光晕遮挡（每个星球中心+半径） ----
    struct Occluder { float cx, cy, cz, radius; };
    std::vector<Occluder> occluders;

    // ---- 轨迹丝带 (Block A) ----
    std::vector<RibbonSegment> ribbons;

    // ---- 广告牌 (Block A: 变轨节点) ----
    std::vector<BillboardDraw> billboards;

    // ---- 发射台 (Block C) ----
    LaunchPadDraw launchPad;
    bool hasLaunchPad = false;

    // ---- 太阳球体 (Block E) ----
    SunBodyDraw sunBody;
    bool drawSunBody = false;

    // ---- 云调试面板 (Block F) ----
    CloudTunerState cloudTuner;

    // ---- SVO 区块 (Block D) ----
    std::vector<SVOChunkDraw> svoChunks;

    // ---- 地形补丁 (Block D) ----
    std::vector<TerrainPatchDraw> terrainPatches;

    // ---- 植被实例计数 (Block D) ----
    uint32_t vegInstanceCount = 0;
    std::vector<float> vegInstanceData;  // 每实例 5 floats: pos(3) + scale(1) + rot(1)

    // ========================================================================
    // 工厂方法
    // ========================================================================

    SceneSnapshot() {
        memset(view, 0, sizeof(view)); memset(proj, 0, sizeof(proj));
        memset(camPos, 0, sizeof(camPos)); memset(lightDir, 0, sizeof(lightDir));
        memset(rocketBodyModel, 0, sizeof(rocketBodyModel));
        memset(rocketNoseModel, 0, sizeof(rocketNoseModel));
        memset(sunScreen, 0, sizeof(sunScreen));
        sunScreen[0] = -3.0f; // 哨兵：默认隐藏光晕，直到计算出有效屏幕坐标
        memset(sunWorldPos, 0, sizeof(sunWorldPos));
        time=0; skyVibrancy=1.f; frameIndex=0; sunIntensity=1.f; aspect=1.f; dayBlend=1.f;
        rocketBodyModel[0]=rocketBodyModel[5]=rocketBodyModel[10]=rocketBodyModel[15]=1.f;
        rocketNoseModel[0]=rocketNoseModel[5]=rocketNoseModel[10]=rocketNoseModel[15]=1.f;
        memset(launchPad.model, 0, sizeof(launchPad.model));
        memset(sunBody.model, 0, sizeof(sunBody.model));
        launchPad.model[0]=launchPad.model[5]=launchPad.model[10]=launchPad.model[15]=1.f;
        sunBody.model[0]=sunBody.model[5]=sunBody.model[10]=sunBody.model[15]=1.f;
        cloudTuner.visible = false;
    }
};
