#pragma once
// ==========================================================================
// vk_game_ui.h — Vulkan 游戏 UI 状态机（完整版）
//
// 状态流程：
//   MENU ──→ AGENCY ──→ WORKSHOP ──→ FLIGHT
//               └──→ FACTORY
//   MENU ──→ SETTINGS
//
// 3D 工坊预览：
//   每帧调用 updateWorkshopInput() 处理鼠标轨道相机
//   调用 buildWorkshopSnapshot() 获取渲染快照（由 main.cpp 提交给 VkRenderer3D）
// ==========================================================================
#include "imgui.h"
#include "simulation/rocket_builder.h"
#include "simulation/center_calculator.h"
#include "simulation/factory_system.h"
#include "core/agency_state.h"
#include "scene/game_context.h"
#include "scene/flight_scene.h"
#include "render/scene_snapshot.h"
#include "save_system.h"
#include "simulation/transfer_calculator.h"
#include "simulation/maneuver_system.h"
#include "simulation/orbit_physics.h"
#include <GLFW/glfw3.h>
#include <cstdio>
#include <string>
#include <cmath>
#include <algorithm>

enum class VkGameState { MENU, SETTINGS, AGENCY, FACTORY, WORKSHOP, FLIGHT };

// ─── 辅助格式化 ──────────────────────────────────────────────────────────────
static std::string _fmtAlt(double m) {
    char b[32];
    if (m >= 1e6) snprintf(b,32,"%.2f Mm",m/1e6);
    else if (m >= 1e3) snprintf(b,32,"%.2f km",m/1e3);
    else snprintf(b,32,"%.0f m",m);
    return b;
}
static std::string _fmtSpd(double ms) { char b[32]; snprintf(b,32,"%.1f m/s",ms); return b; }
static std::string _fmtFunds(double f) {
    char b[32];
    if (f >= 1e9) snprintf(b,32,"%.2f G¥",f/1e9);
    else if (f >= 1e6) snprintf(b,32,"%.2f M¥",f/1e6);
    else snprintf(b,32,"%.0f ¥",f);
    return b;
}

// ─── 物品类型名称 ─────────────────────────────────────────────────────────────
static const char* kItemNames[] = {
    "铁矿石","铜矿石","煤炭","硅","钛矿石",
    "钢铁","铜线","电子元件","钛合金","燃料",
    "发动机","燃料箱","指令舱","结构件","整流罩"
};
static const char* itemName(ItemType t) {
    int i = (int)t;
    if (i < 0 || i >= ITEM_TYPE_COUNT) return "?";
    return kItemNames[i];
}

// ==========================================================================
struct VkGameUI {
    VkGameState state = VkGameState::MENU;

    // ── 状态转换标志（main.cpp 读取后清零）──
    bool goToAgency   = false;
    bool goToWorkshop = false;
    bool goToFlight   = false;
    bool goToExit     = false;

    // ── Workshop 工作数据 ──────────────────────────────────────────────────
    RocketAssembly assembly;
    int  selectedCat  = 0;
    int  selectedPart = -1;
    int  removePart   = -1;
    bool workshopInit = false;

    // Workshop 轨道相机
    float ws_angle   =  0.5f;
    float ws_pitch   =  0.25f;
    float ws_dist    =  8.0f;
    float ws_yOff    =  0.0f;
    bool  ws_rmbPrev =  false;
    double ws_prevMX = 0, ws_prevMY = 0;
    int   ws_frameCount = 0;

    // ── Workshop 附加状态 ─────────────────────────────────────────────
    CenterVisualizationState ws_centerViz;
    int  ws_symmetry  = 1;    // 对称模式: 1/2/4/8
    bool ws_ctxOpen   = false; // 右键上下文菜单打开
    int  ws_ctxIdx    = -1;    // 上下文菜单对应的零件索引

    // ── 3D 拖拽状态 ─────────────────────────────────────────────────
    bool ws_dragging      = false;  // 是否正在拖拽零件（幽灵模式）
    int  ws_dragDefId     = -1;     // 正在拖拽的零件定义 ID
    int  ws_dragParentIdx = -1;     // 吸附到的父零件索引 (-1=无)
    Vec3 ws_dragPos       = Vec3(0,0,0); // 拖拽位置
    Quat ws_dragRot       = Quat(); // 拖拽旋转
    bool ws_dragValid     = true;   // 当前位置是否有效
    int  ws_dragMovingIdx = -1;     // 正在移动的已放置零件索引 (-1=新建)
    int  ws_hoveredPartIdx = -1;    // 鼠标悬停的已放置零件索引（用于高亮）
    // 键盘旋转累积
    Quat ws_manualRot     = Quat(); // Q/E/W/S/A/D 手动旋转累积
    float ws_manualRotTimer = 0.0f; // 键盘旋转计时器
    // 右键点击跟踪
    double ws_rmbClickX = 0, ws_rmbClickY = 0;
    int    ws_rmbClickPartIdx = -1; // 右键点击时命中的零件

    // ── Flight HUD 附加状态 ───────────────────────────────────────────────
    bool hud_show_galaxy = false;  // 星系信息覆盖层
    int  hud_sel_body    = -1;     // 选中的天体索引
    int  hud_exp_planet  = -1;     // 展开卫星的行星索引
    bool hud_adv_orbit   = false;  // 高级轨道面板
    bool hud_flight_asst = false;  // 飞行辅助面板
    bool hud_transfer_win= false;  // 转移窗口面板
    int  hud_transfer_target = 4;  // 转移目标天体索引（默认火星）
    PorkchopResult hud_transfer_result;      // 转移窗口计算结果
    bool hud_transfer_valid = false;         // 转移结果是否有效
    int  hud_transfer_hover_dep = -1;        // 鼠标悬停 departure 轴索引
    int  hud_transfer_hover_tof = -1;        // 鼠标悬停 TOF 轴索引

    // ── Factory UI 状态 ─────────────────────────────────────────────────
    int factorySelectedNode = -1;

    // 网格相机
    float fac_camX = 0.0f, fac_camY = 0.0f;
    float fac_zoom = 1.0f;
    static constexpr int FAC_GRID_W = 30;
    static constexpr int FAC_GRID_H = 20;
    float fac_cellSize = 32.0f;  // 像素（在 zoom=1 时每格像素数）

    // 放置模式
    bool fac_placing = false;
    FactoryNodeType fac_placeType = NODE_MINER;

    // 传送带模式
    bool fac_beltMode = false;
    int  fac_beltFromId = -1;

    // 删除模式
    bool fac_deleteMode = false;

    // 鼠标交互
    bool  fac_rmbDown = false;
    float fac_lastMX = 0, fac_lastMY = 0;
    float fac_dragStartX = 0, fac_dragStartY = 0;
    int   fac_hoverGX = -1, fac_hoverGY = -1;

    // ── Settings ─────────────────────────────────────────────────────────
    bool  settingsVSync    = true;
    int   settingsLanguage = 0;   // 0=中文  1=English

    // ─────────────────────────────────────────────────────────────────────
    // 主入口
    void draw(FlightScene* flightScene, int ww, int wh) {
        switch (state) {
            case VkGameState::MENU:     _drawMenu(ww, wh);    break;
            case VkGameState::SETTINGS: _drawSettings(ww, wh); break;
            case VkGameState::AGENCY:   _drawAgency(ww, wh);  break;
            case VkGameState::FACTORY:  _drawFactory(ww, wh); break;
            case VkGameState::WORKSHOP: _drawWorkshop(ww, wh); break;
            case VkGameState::FLIGHT:   _drawFlightHUD(flightScene, ww, wh); break;
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Workshop 相机 + 交互：每帧从 main.cpp 调用（在 ImGui 之前）
    void updateWorkshopInput(GLFWwindow* window) {
        if (state != VkGameState::WORKSHOP) return;
        ws_frameCount++;

        double mx, my;
        glfwGetCursorPos(window, &mx, &my);
        bool rmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        bool lmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;

        int ww, wh;
        glfwGetWindowSize(window, &ww, &wh);
        bool overViewport = (mx > 310 && mx < ww - 320);

        // ── 每帧：更新悬停零件索引 ──
        if (overViewport && !rmb) {
            ws_hoveredPartIdx = _wsPickPart(ww, wh, (float)mx, (float)my);
        } else if (!overViewport) {
            ws_hoveredPartIdx = -1;
        }

        // ── 右键：轨道相机旋转 OR 零件上下文菜单 ──
        if (rmb) {
            if (!ws_rmbPrev) {
                // 右键刚按下：记录起始位置和命中零件
                ws_rmbClickX = mx; ws_rmbClickY = my;
                ws_rmbClickPartIdx = overViewport ? _wsPickPart(ww, wh, (float)mx, (float)my) : -1;
            }
            if (ws_rmbPrev) {
                float dx = (float)(mx - ws_prevMX) * 0.005f;
                float dy = (float)(my - ws_prevMY) * 0.005f;
                ws_angle -= dx;
                ws_pitch  = std::max(-0.45f, std::min(1.4f, ws_pitch + dy));
            }
            ws_prevMX = mx; ws_prevMY = my;
        } else if (ws_rmbPrev) {
            // 右键刚松开：检查是否是短点击（未大幅移动）→ 触发上下文菜单
            float clickDist = (float)sqrt((mx-ws_rmbClickX)*(mx-ws_rmbClickX) + (my-ws_rmbClickY)*(my-ws_rmbClickY));
            if (clickDist < 5.0f && ws_rmbClickPartIdx >= 0 && ws_rmbClickPartIdx < (int)assembly.parts.size()) {
                ws_ctxIdx  = ws_rmbClickPartIdx;
                ws_ctxOpen = true;
                // 通知 ImGui 打开弹窗（在 _drawWorkshop 中处理）
            }
            ws_rmbClickPartIdx = -1;
        }
        ws_rmbPrev = rmb;

        // 不自动旋转（用户明确要求取消）
        // if (!rmb && !ws_dragging) ws_angle += 0.002f;

        // ── 左键交互 ──
        static bool lmbPrev = false;
        static bool lmbWasOverViewport = false;

        if (lmb && !lmbPrev) {
            // 左键刚按下
            lmbWasOverViewport = overViewport;
            if (overViewport && !rmb) {
                if (ws_dragging) {
                    // 拖拽中点击视口 → 放置零件
                    _wsHandleRelease();
                } else if (ws_hoveredPartIdx >= 0) {
                    // 点击已建造的零件 → 拆解为幽灵部件
                    _wsStartDetachDrag(ws_hoveredPartIdx);
                }
            } else if (!overViewport && ws_dragging) {
                // 点击 UI 区域 → 删除幽灵（不归还拆解零件）
                _wsClearDragState(false);
            }
        }

        // 拖拽中：持续更新幽灵位置（无论鼠标在视口还是 UI 上）
        if (ws_dragging) {
            _wsUpdateDrag(ww, wh, (float)mx, (float)my);
        }

        lmbPrev = lmb;

        // ── ESC 取消拖拽 ──
        if (ws_dragging && glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            _wsCancelDrag();
        }

        // ── 键盘旋转累积 ──
        float rotStep = 90.0f * 3.14159f / 180.0f;
        ws_manualRotTimer += 0.016f;
        if (ws_manualRotTimer > 0.05f) {
            bool rotated = false;
            auto& mr = ws_manualRot;
            if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) { mr = mr * Quat::fromAxisAngle(Vec3(0,1,0), -rotStep*ws_manualRotTimer); rotated = true; }
            if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) { mr = mr * Quat::fromAxisAngle(Vec3(0,1,0),  rotStep*ws_manualRotTimer); rotated = true; }
            if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) { mr = mr * Quat::fromAxisAngle(Vec3(1,0,0), -rotStep*ws_manualRotTimer); rotated = true; }
            if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) { mr = mr * Quat::fromAxisAngle(Vec3(1,0,0),  rotStep*ws_manualRotTimer); rotated = true; }
            if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) { mr = mr * Quat::fromAxisAngle(Vec3(0,0,1), -rotStep*ws_manualRotTimer); rotated = true; }
            if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) { mr = mr * Quat::fromAxisAngle(Vec3(0,0,1),  rotStep*ws_manualRotTimer); rotated = true; }
            if (rotated) { mr = mr.normalized(); ws_manualRotTimer = 0.0f; }
        }
    }

    void handleWorkshopScroll(float dy) {
        if (state != VkGameState::WORKSHOP) return;
        if (glfwGetKey(GameContext::getInstance().window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
            ws_dist = std::max(2.0f, std::min(80.0f, ws_dist * powf(0.85f, dy)));
        else
            ws_yOff += dy * (ws_dist * 0.05f);
    }

    // ── 获取工坊相机参数 ──────────────────────────────────────────────
    struct WsCamera {
        Vec3 eye, target;
        Mat4 view, proj;
        float aspect;
    };
    WsCamera _wsGetCamera(int ww, int wh) const {
        WsCamera cam;
        float rocketH = std::max(5.0f, assembly.total_height);
        float lookY   = rocketH * 0.4f + ws_yOff;
        float dist    = ws_dist + rocketH * 0.5f;
        float cy      = sinf(ws_pitch) * dist;
        float cx      = cosf(ws_pitch) * cosf(ws_angle) * dist;
        float cz      = cosf(ws_pitch) * sinf(ws_angle) * dist;
        cam.eye    = Vec3(cx, cy + lookY, cz);
        cam.target = Vec3(0.0f, lookY, 0.0f);
        cam.aspect = (ww > 0 && wh > 0) ? (float)ww / (float)wh : 1.0f;
        cam.view   = Mat4::lookAt(cam.eye, cam.target, Vec3(0,1,0));
        cam.proj   = Mat4::perspective(1.0f, cam.aspect, 0.1f, 1000.0f);
        return cam;
    }

    // ── 屏幕坐标 → 3D 世界射线 ──────────────────────────────────────
    struct WsRay { Vec3 origin, dir; };
    WsRay _wsScreenToRay(int ww, int wh, float mx, float my) const {
        WsCamera cam = _wsGetCamera(ww, wh);

        // NDC 坐标
        float ndcX = (2.0f * mx) / (float)ww - 1.0f;
        float ndcY = 1.0f - (2.0f * my) / (float)wh; // Y 翻转

        // 逆投影：NDC → 视空间
        float tanHalfFov = tanf(0.5f); // FOV=1.0 rad
        float vsX = ndcX * tanHalfFov * cam.aspect;
        float vsY = ndcY * tanHalfFov;
        Vec3 vsNear(vsX, vsY, -1.0f); // 近平面上的点（视空间）

        // 视空间 → 世界空间
        Mat4 invView = cam.view.inverse();
        Vec3 worldNear = invView.transformPoint(vsNear);

        WsRay ray;
        ray.origin = cam.eye;
        ray.dir    = (worldNear - cam.eye).normalized();
        return ray;
    }

    // ── 射线 vs 圆柱体碰撞检测 ──────────────────────────────────────
    // 返回 t 值（射线参数），<0 表示未命中
    static float _wsRayHitCylinder(const WsRay& ray, Vec3 cylBase, Vec3 cylTop, float radius) {
        Vec3 ba = cylTop - cylBase;
        float h = ba.length();
        if (h < 0.001f) return -1.0f;
        Vec3 baDir = ba / h;

        // 射线与圆柱轴线的最近点（无限圆柱）
        Vec3 oc = ray.origin - cylBase;
        float dDotBa = ray.dir.dot(baDir);
        float ocDotBa = oc.dot(baDir);

        float a = 1.0f - dDotBa * dDotBa;
        float b = 2.0f * (ray.dir.dot(oc) - dDotBa * ocDotBa);
        float c = oc.dot(oc) - ocDotBa * ocDotBa - radius * radius;

        float disc = b*b - 4.0f*a*c;
        if (disc < 0.0f) return -1.0f;

        float t = (-b - sqrtf(disc)) / (2.0f * a);
        if (t < 0.0f) t = (-b + sqrtf(disc)) / (2.0f * a);
        if (t < 0.0f) return -1.0f;

        // 检查命中点是否在有限高度内
        Vec3 hitPt = ray.origin + ray.dir * t;
        float proj = (hitPt - cylBase).dot(baDir);
        if (proj < -0.1f * h || proj > 1.1f * h) return -1.0f;

        return t;
    }

    // ── 拾取：找到射线命中的最近零件 ──────────────────────────────────
    // 返回零件索引，-1 表示未命中
    int _wsPickPart(int ww, int wh, float mx, float my) const {
        if (assembly.parts.empty()) return -1;
        WsRay ray = _wsScreenToRay(ww, wh, mx, my);

        int bestIdx = -1;
        float bestT = 1e10f;

        for (int i = 0; i < (int)assembly.parts.size(); i++) {
            const PlacedPart& pp = assembly.parts[i];
            const PartDef& def = PART_CATALOG[pp.def_id];

            float r = def.diameter * 0.5f;
            Vec3 base = pp.pos;
            Vec3 top  = pp.pos + pp.rot.rotate(Vec3(0, def.height, 0));

            float t = _wsRayHitCylinder(ray, base, top, r * 1.15f); // 略微放大便于点击
            if (t > 0.0f && t < bestT) {
                bestT = t;
                bestIdx = i;
            }
        }
        return bestIdx;
    }

    // ── 吸附计算：KSP 风格 —— 双向节点匹配 + 方向对齐 ──────────────
    // 每个零件有两个 stack node：顶部 (pos=h, dir=+Y) 和底部 (pos=0, dir=-Y)
    // 吸附条件：父节点方向与子节点方向相反（面对面）
    void _wsCalcSnap(int defId, Vec3 worldPos, int& outParent, Vec3& outPos, Quat& outRot) {
        outParent = -1;
        outPos = worldPos;
        outRot = ws_manualRot;
        if (defId < 0 || defId >= PART_CATALOG_SIZE) return;
        const PartDef& dDef = PART_CATALOG[defId];

        // ── 1. Snap node 吸附（堆叠）──
        float bestScore = 8.0f;
        for (int i = 0; i < (int)assembly.parts.size(); i++) {
            const auto& pPart = assembly.parts[i];
            const PartDef& pDef = PART_CATALOG[pPart.def_id];
            for (int pn = 0; pn < (int)pDef.snap_nodes.size(); pn++) {
                Vec3 pNodeLocal = pDef.snap_nodes[pn].pos;
                Vec3 pDirLocal  = pDef.snap_nodes[pn].dir;
                Vec3 pNodeWorld = pPart.pos + pPart.rot.rotate(pNodeLocal);
                Vec3 pDirWorld  = pPart.rot.rotate(pDirLocal);

                for (int cn = 0; cn < (int)dDef.snap_nodes.size(); cn++) {
                    Vec3 cNodeLocal = dDef.snap_nodes[cn].pos;
                    Vec3 cDirLocal  = dDef.snap_nodes[cn].dir;

                    // 方向匹配：子节点在父节点旋转下的世界方向应与父节点方向相反
                    // 使用父零件旋转作为子零件的基础旋转
                    Quat baseRot = pPart.rot * ws_manualRot;
                    Vec3 cDirWorld = baseRot.rotate(cDirLocal);
                    float dirDot = cDirWorld.dot(pDirWorld);
                    // 面对面 (dot < -0.5 即角度 < 120°) 或同向 (dot > 0.5) 都算匹配
                    bool dirMatch = (dirDot < -0.5f);

                    // 计算子零件应在的位置
                    Vec3 targetPos = pNodeWorld - baseRot.rotate(cNodeLocal);
                    float d = (targetPos - worldPos).length();

                    if (d < bestScore && dirMatch) {
                        bestScore = d;
                        outParent = i;
                        outPos = targetPos;
                        outRot = baseRot;
                    }
                }
            }
        }
        if (outParent >= 0) return;

        // ── 2. 表面贴装 ──
        if (!dDef.surf_attach) return;
        for (int i = 0; i < (int)assembly.parts.size(); i++) {
            const PartDef& pDef = PART_CATALOG[assembly.parts[i].def_id];
            bool valid = (pDef.category == CAT_FUEL_TANK || pDef.category == CAT_STRUCTURAL ||
                          pDef.category == CAT_ENGINE || pDef.category == CAT_COMMAND_POD ||
                          pDef.category == CAT_NOSE_CONE);
            if (!valid) continue;

            float r = pDef.diameter * 0.5f;
            Vec3 p0 = assembly.parts[i].pos;
            float t = (worldPos.y - p0.y) / fmaxf(0.001f, pDef.height);
            if (t >= -0.1f && t <= 1.1f) {
                t = std::max(0.0f, std::min(1.0f, t));
                Vec3 axisPt = p0 + Vec3(0, t * pDef.height, 0);
                Vec3 diff = worldPos - axisPt; diff.y = 0;
                float dist = diff.length();
                if (dist < r * 3.5f && dist > 0.01f) {
                    float score = fabsf(dist - r);
                    if (score < bestScore || bestScore > 2.0f) {
                        bestScore = score;
                        outParent = i;
                        outPos = axisPt + diff.normalized() * r;
                        Vec3 normal = (outPos - assembly.parts[i].pos); normal.y = 0;
                        if (normal.length() > 0.01f) {
                            float angle = atan2f(normal.z, normal.x);
                            outRot = Quat::fromAxisAngle(Vec3(0,1,0), -angle + 3.14159f/2.0f) * ws_manualRot;
                        }
                    }
                }
            }
        }
    }

    // ── 清空拖拽状态（可选是否归还拆解的零件）────────────────────────
    void _wsClearDragState(bool restoreDetached) {
        if (restoreDetached && ws_dragMovingIdx >= 0 && ws_dragDefId >= 0) {
            PlacedPart pp;
            pp.def_id   = ws_dragDefId;
            pp.symmetry = 1;
            pp.pos      = ws_dragPos;
            pp.rot      = ws_dragRot;
            assembly.parts.push_back(pp);
            assembly.recalculate();
        }
        ws_dragging      = false;
        ws_dragDefId     = -1;
        ws_dragParentIdx = -1;
        ws_dragMovingIdx = -1;
        ws_manualRot     = Quat();
        ws_manualRotTimer = 0.0f;
        ws_hoveredPartIdx = -1;
        ws_ctxOpen       = false;
        selectedPart     = -1;
        ws_centerViz.lastAssemblyHash = 0;
    }

    // ── 从目录启动拖拽（由 _drawWorkshop 中目录点击调用）─────────────
    void _wsStartCatalogDrag(int defId) {
        if (defId < 0 || defId >= PART_CATALOG_SIZE) return;
        // 直接清空旧状态（不归还拆解的零件——用户主动点击目录=放弃旧幽灵）
        _wsClearDragState(false);

        ws_dragDefId     = defId;
        ws_dragMovingIdx = -1;
        ws_dragging      = true;
        ws_dragValid     = true;
        ws_dragParentIdx = -1;
        ws_manualRot     = Quat();
        ws_manualRotTimer = 0.0f;
        ws_dragPos       = Vec3(0, assembly.total_height > 0 ? assembly.total_height : 2.0f, 0);
        ws_dragRot       = Quat();
        ws_ctxOpen       = false;
    }

    // ── 拆解已建造零件为幽灵 ─────────────────────────────────────────
    void _wsStartDetachDrag(int partIdx) {
        if (partIdx < 0 || partIdx >= (int)assembly.parts.size()) return;
        if (ws_dragging) return; // 已经在拖拽，不能再拆

        ws_dragMovingIdx = partIdx;
        ws_dragDefId     = assembly.parts[partIdx].def_id;
        ws_dragPos       = assembly.parts[partIdx].pos;
        ws_dragRot       = assembly.parts[partIdx].rot;
        assembly.removePart(partIdx);
        ws_dragging      = true;
        ws_dragValid     = true;
        ws_dragParentIdx = -1;
        ws_manualRot     = Quat();
        ws_manualRotTimer = 0.0f;
        ws_centerViz.lastAssemblyHash = 0;
        ws_ctxOpen       = false;
        ws_hoveredPartIdx = -1;
    }

    // ── 拖拽中：更新位置（KSP 风格固定深度）───────────────────────
    void _wsUpdateDrag(int ww, int wh, float mx, float my) {
        if (!ws_dragging || ws_dragDefId < 0) return;

        WsRay ray = _wsScreenToRay(ww, wh, mx, my);
        WsCamera cam = _wsGetCamera(ww, wh);

        // ── KSP 风格：固定深度 = 相机到火箭中心的距离 ──
        float rocketMidY = assembly.total_height > 0 ? assembly.total_height * 0.45f : 2.0f;
        Vec3 rocketCenter(0, rocketMidY, 0);
        float depth = fmaxf(1.0f, (float)(rocketCenter - cam.eye).length());
        Vec3 worldPos = ray.origin + ray.dir * depth;

        // 首个零件固定在原点
        if (assembly.parts.empty() && ws_dragMovingIdx < 0) {
            ws_dragPos = Vec3(0, 0, 0);
            ws_dragValid = true;
            ws_dragParentIdx = -1;
            ws_dragRot = ws_manualRot;
            return;
        }

        // 吸附计算
        int parentIdx = -1;
        Vec3 snapPos = worldPos;
        Quat snapRot = ws_manualRot;
        _wsCalcSnap(ws_dragDefId, worldPos, parentIdx, snapPos, snapRot);

        ws_dragPos = snapPos;
        ws_dragRot = snapRot;
        ws_dragParentIdx = parentIdx;
        ws_dragValid = (parentIdx >= 0) || PART_CATALOG[ws_dragDefId].surf_attach || assembly.parts.empty();
    }

    // ── 放置零件 ────────────────────────────────────────────────────
    void _wsHandleRelease() {
        if (!ws_dragging || ws_dragDefId < 0) { ws_dragging = false; return; }

        // 放置到 assembly
        PlacedPart pp;
        pp.def_id    = ws_dragDefId;
        pp.parent_idx = ws_dragParentIdx;
        pp.symmetry  = ws_symmetry;
        pp.pos       = ws_dragPos;
        pp.rot       = ws_dragRot;
        assembly.parts.push_back(pp);
        assembly.recalculate();
        ws_centerViz.lastAssemblyHash = 0;

        // 彻底清空幽灵状态
        ws_dragging      = false;
        ws_dragDefId     = -1;
        ws_dragParentIdx = -1;
        ws_dragMovingIdx = -1;
        ws_manualRot     = Quat();
        ws_manualRotTimer = 0.0f;
        selectedPart     = -1;  // 清空目录选中
        ws_hoveredPartIdx = -1;
    }

    // ── 取消拖拽 ──────────────────────────────────────────────────────
    void _wsCancelDrag() {
        if (!ws_dragging) return;
        // 如果是从火箭上拾取的零件，归还回去
        if (ws_dragMovingIdx >= 0 && ws_dragDefId >= 0) {
            PlacedPart pp;
            pp.def_id   = ws_dragDefId;
            pp.symmetry = 1;
            pp.pos      = ws_dragPos;
            pp.rot      = ws_dragRot;
            assembly.parts.push_back(pp);
            assembly.recalculate();
        }
        ws_dragging      = false;
        ws_dragDefId     = -1;
        ws_dragParentIdx = -1;
        ws_dragMovingIdx = -1;
        ws_manualRot     = Quat();
        ws_manualRotTimer = 0.0f;
        ws_centerViz.lastAssemblyHash = 0;
        selectedPart     = -1;
        ws_hoveredPartIdx = -1;
    }

    // ─────────────────────────────────────────────────────────────────────
    // 构建 Workshop 3D 快照（main.cpp 每帧调用，用于 VkRenderer3D 渲染）
    // fs: 可选的 FlightScene 指针，用于获取已加载的 OBJ 网格数据
    SceneSnapshot buildWorkshopSnapshot(int ww, int wh, const FlightScene* fs = nullptr) const {
        SceneSnapshot snap;
        snap.skyVibrancy = 0.0f;   // 纯黑背景
        snap.dayBlend    = 0.0f;
        snap.frameIndex  = ws_frameCount;
        snap.time        = 0.0f;
        snap.aspect      = (ww > 0 && wh > 0) ? (float)ww / (float)wh : 1.0f;
        snap.sunScreen[0]  = -3.0f;  // 不渲染镜头光晕
        snap.sunIntensity  = 0.0f;

        // ── 相机 ──
        float rocketH = std::max(5.0f, assembly.total_height);
        float lookY   = rocketH * 0.4f + ws_yOff;
        float dist    = ws_dist + rocketH * 0.5f;
        float cy      = sinf(ws_pitch) * dist;
        float cx      = cosf(ws_pitch) * cosf(ws_angle) * dist;
        float cz      = cosf(ws_pitch) * sinf(ws_angle) * dist;
        Vec3 camEye(cx, cy + lookY, cz);
        Vec3 camTarget(0.0f, lookY, 0.0f);

        Mat4 viewMat = Mat4::lookAt(camEye, camTarget, Vec3(0,1,0));
        Mat4 projMat = Mat4::perspective(1.0f, snap.aspect, 0.1f, 1000.0f);
        viewMat.toFloatArray(snap.view);
        projMat.toFloatArray(snap.proj);
        snap.camPos[0] = camEye.x; snap.camPos[1] = camEye.y; snap.camPos[2] = camEye.z;

        // 三点光照（主光+填充光+背光）
        snap.lightDir[0] =  0.577f;
        snap.lightDir[1] =  0.577f;
        snap.lightDir[2] =  0.577f;

        // ── 地台（用 rocketBox mesh 铺底板） ──
        {
            float s = 20.0f;
            float pad[16]={s,0,0,0, 0,0.15f,0,0, 0,0,s,0, 0,-0.08f,0,1};
            snap.hasLaunchPad = true;
            memcpy(snap.launchPad.model, pad, 64);
        }

        // ── 火箭零件 ──
        constexpr float k2Pi = 6.28318530718f;
        // OBJ 路径解析（与 PartRenderer 和 extractRenderSnapshot 使用相同逻辑）
        auto wsObjPath = [](const PartDef& def) -> std::string {
            if (def.model_path) return def.model_path;
            std::string tmp = def.name;
            for (char& c : tmp) { c=(char)std::tolower((unsigned char)c); if(c==' ')c='_'; }
            return "assets/models/" + tmp + ".obj";
        };
        for (int i = 0; i < (int)assembly.parts.size(); i++) {
            const PlacedPart& pp  = assembly.parts[i];
            const PartDef&    def = PART_CATALOG[pp.def_id];
            // 查找 OBJ 模型
            const Mesh* objMesh = nullptr;
            std::string objPath;
            if (fs) {
                objPath = wsObjPath(def);
                auto it = fs->partObjMeshes.find(objPath);
                if (it != fs->partObjMeshes.end() && !it->second.cpuIndices.empty())
                    objMesh = &it->second;
            }
            for (int s = 0; s < pp.symmetry; s++) {
                float symAngle = (s * k2Pi) / pp.symmetry;
                Vec3 localPos = pp.pos;
                if (pp.symmetry > 1) {
                    float r2 = sqrtf(pp.pos.x*pp.pos.x + pp.pos.z*pp.pos.z);
                    if (r2 > 0.01f) {
                        float ca = atan2f(pp.pos.z, pp.pos.x);
                        localPos.x = cosf(ca+symAngle)*r2;
                        localPos.z = sinf(ca+symAngle)*r2;
                    }
                }
                Quat wr = pp.rot * Quat::fromAxisAngle(Vec3(0,1,0), symAngle);
                RocketPartDraw rp;
                // 悬停高亮：被悬停的零件用金色调
                bool isHovered = (i == ws_hoveredPartIdx);
                rp.r = isHovered ? std::min(1.0f, def.r + 0.4f) : def.r;
                rp.g = isHovered ? std::min(1.0f, def.g + 0.5f) : def.g;
                rp.b = isHovered ? def.b * 0.5f : def.b;
                if (objMesh) {
                    // OBJ 模型：按物理尺寸缩放 + 中心偏移对齐（工坊空间无 ws_d 缩放）
                    float sx = def.diameter / objMesh->width;
                    float sy = def.height   / objMesh->height;
                    float sz = def.diameter / objMesh->depth;
                    Mat4 m = Mat4::translate(localPos) * Mat4::fromQuat(wr)
                           * Mat4::scale(Vec3(sx,sy,sz))
                           * Mat4::translate(Vec3(-objMesh->centerX, -objMesh->minY, -objMesh->centerZ));
                    m.toFloatArray(rp.model);
                    rp.meshType = 0;
                    rp.meshId   = objPath;
                } else {
                    float h = def.height, d = def.diameter;
                    Mat4 m = Mat4::TRS(localPos, wr, Vec3(d,h,d));
                    m.toFloatArray(rp.model);
                    rp.meshType = (def.category==CAT_NOSE_CONE||def.category==CAT_COMMAND_POD) ? 1 :
                                  (def.category==CAT_STRUCTURAL) ? 2 : 0;
                }
                snap.rocketParts.push_back(rp);
            }
        }

        // ── 拖拽幽灵零件 ──
        if (ws_dragging && ws_dragDefId >= 0 && ws_dragDefId < PART_CATALOG_SIZE) {
            const PartDef& gDef = PART_CATALOG[ws_dragDefId];
            float ghostAlpha = 0.45f + 0.15f * sinf((float)ws_frameCount * 0.15f); // 闪烁

            // 查找 OBJ 模型
            std::string gObjPath;
            const Mesh* gObjMesh = nullptr;
            if (fs) {
                gObjPath = wsObjPath(gDef);
                auto it = fs->partObjMeshes.find(gObjPath);
                if (it != fs->partObjMeshes.end() && !it->second.cpuIndices.empty())
                    gObjMesh = &it->second;
            }

            for (int s = 0; s < ws_symmetry; s++) {
                float symAngle = (s * k2Pi) / ws_symmetry;
                Vec3 gLocalPos = ws_dragPos;
                if (ws_symmetry > 1) {
                    float r2 = sqrtf(ws_dragPos.x*ws_dragPos.x + ws_dragPos.z*ws_dragPos.z);
                    if (r2 > 0.01f) {
                        float ca = atan2f(ws_dragPos.z, ws_dragPos.x);
                        gLocalPos.x = cosf(ca+symAngle)*r2;
                        gLocalPos.z = sinf(ca+symAngle)*r2;
                    }
                }
                Quat gWr = ws_dragRot * Quat::fromAxisAngle(Vec3(0,1,0), symAngle);
                RocketPartDraw grp;
                // 有效=绿色调，无效=红色调
                grp.r = ws_dragValid ? 0.5f : 1.2f;
                grp.g = ws_dragValid ? 1.2f : 0.5f;
                grp.b = ws_dragValid ? 0.5f : 0.5f;

                if (gObjMesh) {
                    float sx = gDef.diameter / gObjMesh->width;
                    float sy = gDef.height   / gObjMesh->height;
                    float sz = gDef.diameter / gObjMesh->depth;
                    Mat4 m = Mat4::translate(gLocalPos) * Mat4::fromQuat(gWr)
                           * Mat4::scale(Vec3(sx,sy,sz))
                           * Mat4::translate(Vec3(-gObjMesh->centerX, -gObjMesh->minY, -gObjMesh->centerZ));
                    m.toFloatArray(grp.model);
                    grp.meshType = 0;
                    grp.meshId   = gObjPath;
                } else {
                    float h = gDef.height, d = gDef.diameter;
                    Mat4 m = Mat4::TRS(gLocalPos, gWr, Vec3(d,h,d));
                    m.toFloatArray(grp.model);
                    grp.meshType = (gDef.category==CAT_NOSE_CONE||gDef.category==CAT_COMMAND_POD) ? 1 :
                                   (gDef.category==CAT_STRUCTURAL) ? 2 : 0;
                }
                snap.rocketParts.push_back(grp);
            }
        }

        // ── 质心/升力中心/推力中心标记 ──
        // 每次哈希变化时重算
        {
            size_t h = CenterCalculator::hashAssembly(assembly);
            if (h != ws_centerViz.lastAssemblyHash) {
                const_cast<CenterVisualizationState&>(ws_centerViz).lastAssemblyHash = h;
                const_cast<CenterVisualizationState&>(ws_centerViz).hasCoM = !assembly.parts.empty();
                const_cast<CenterVisualizationState&>(ws_centerViz).hasCoL = false;
                const_cast<CenterVisualizationState&>(ws_centerViz).hasCoT = assembly.hasEngine();
                if (ws_centerViz.hasCoM)
                    const_cast<CenterVisualizationState&>(ws_centerViz).comPos = CenterCalculator::calculateCenterOfMass(assembly);
                const_cast<CenterVisualizationState&>(ws_centerViz).colPos = CenterCalculator::calculateCenterOfLift(assembly);
                const_cast<CenterVisualizationState&>(ws_centerViz).hasCoL = (ws_centerViz.colPos.y > 0.0f);
                if (ws_centerViz.hasCoT)
                    const_cast<CenterVisualizationState&>(ws_centerViz).cotPos = CenterCalculator::calculateCenterOfThrust(assembly);
            }
        }
        auto addMarker = [&](Vec3 pos, float r, float g, float b, int mtype) {
            float s = 0.55f;
            Mat4 m = Mat4::TRS(pos, Quat(), Vec3(s, s, s));
            RocketPartDraw rp; m.toFloatArray(rp.model);
            rp.r = r; rp.g = g; rp.b = b; rp.meshType = mtype;
            snap.rocketParts.push_back(rp);
        };
        if (ws_centerViz.showCoM && ws_centerViz.hasCoM)
            addMarker(ws_centerViz.comPos, 0.2f, 0.5f, 1.0f, 0); // 蓝色 CoM 球
        if (ws_centerViz.showCoL && ws_centerViz.hasCoL)
            addMarker(ws_centerViz.colPos, 1.0f, 0.8f, 0.2f, 1); // 黄色 CoL 锥
        if (ws_centerViz.showCoT && ws_centerViz.hasCoT)
            addMarker(ws_centerViz.cotPos, 1.0f, 0.2f, 0.2f, 2); // 红色 CoT 方块

        return snap;
    }

    // 进入 Workshop 时初始化默认装配
    void beginWorkshop() {
        if (!workshopInit) {
            assembly = RocketAssembly{};
            assembly.addPart(9);  // Raptor
            assembly.addPart(6);  // Medium Tank
            assembly.addPart(0);  // Fairing
            ws_dist = 8.0f; ws_yOff = 0.0f;
            ws_angle = 0.5f; ws_pitch = 0.25f;
            workshopInit = true;
        }
    }

private:
    // ================================================================
    // NAVBALL — 基于四元数真3D球面投影的人工智能地平仪
    // 与 OpenGL 版本使用相同的数学逻辑：
    //   1. 经纬度 → 球面3D坐标
    //   2. 地理参考系(localRight/Up/North) → 世界空间
    //   3. 火箭姿态四元数共轭 → 箭体局部空间
    //   4. pR.y < 0 → 背面剔除; 否则映射到2D圆盘
    // ================================================================
    static void _drawNavball(ImDrawList* dl, ImVec2 ctr, float R,
                             const Quat& rocketQuat, const Vec3& localRight,
                             const Vec3& localUp, const Vec3& localNorth) {
        const float kPI = 3.14159265358979f;

        // ── 辅助：经纬度 → 球面3D点 ──────────────────────────────────
        auto getSpherePt = [&](float lat, float lon) -> Vec3 {
            float phi = lat * (kPI / 180.f), theta = lon * (kPI / 180.f);
            return Vec3((double)(cosf(phi) * sinf(theta)),
                        (double)sinf(phi),
                        (double)(cosf(phi) * cosf(theta)));
        };

        // ── 辅助：球面3D点 → 2D屏幕坐标（四元数投影）────────────────
        // 返回 {999,999} 表示在球背面，不绘制
        auto project = [&](const Vec3& spherePt) -> ImVec2 {
            Vec3 pW = localRight * spherePt.x + localUp * spherePt.y + localNorth * spherePt.z;
            Vec3 pR = rocketQuat.conjugate().rotate(pW);
            if (pR.y < 0) return {999.f, 999.f};  // 背面剔除
            return {ctr.x + (float)(pR.x * R), ctr.y - (float)(pR.z * R)};
        };

        // ── 辅助：世界方向向量 → 2D屏幕坐标 ─────────────────────────
        auto projectWorld = [&](const Vec3& wv) -> ImVec2 {
            Vec3 pR = rocketQuat.conjugate().rotate(wv);
            if (pR.y < 0.05) return {999.f, 999.f};
            return {ctr.x + (float)(pR.x * R), ctr.y - (float)(pR.z * R)};
        };

        // ── 安全检查：仅当顶点在正面时才绘制 ──────────────────────
        auto safeTri = [&](ImVec2 a, ImVec2 b, ImVec2 c, ImU32 col) {
            if (a.x > 900 || b.x > 900 || c.x > 900) return;
            dl->AddTriangleFilled(a, b, c, col);
        };
        auto safeLine = [&](ImVec2 a, ImVec2 b, ImU32 col, float thick) {
            if (a.x > 900 || b.x > 900) return;
            dl->AddLine(a, b, col, thick);
        };

        // ════════════════════════════════════════════════════════════════
        // 1. 底座圆盘
        // ════════════════════════════════════════════════════════════════
        dl->AddCircleFilled(ctr, R * 1.05f, IM_COL32(25, 25, 30, 230));
        dl->AddCircleFilled(ctr, R, IM_COL32(50, 50, 56, 255));

        // ════════════════════════════════════════════════════════════════
        // 2. 球面镶嵌（天空/地面色块） — 24×48 经纬网格
        // ════════════════════════════════════════════════════════════════
        const int latSteps = 24, lonSteps = 48;
        const ImU32 colSky    = IM_COL32(20, 115, 224, 255);
        const ImU32 colGround = IM_COL32(148, 82, 38, 255);

        for (int i = 0; i < latSteps; i++) {
            float lat1 = -90.f + (float)i       / latSteps * 180.f;
            float lat2 = -90.f + (float)(i + 1) / latSteps * 180.f;
            ImU32 col = (lat1 + lat2 > 0) ? colSky : colGround;

            for (int j = 0; j < lonSteps; j++) {
                float lon1 = (float)j       / lonSteps * 360.f;
                float lon2 = (float)(j + 1) / lonSteps * 360.f;

                ImVec2 p1 = project(getSpherePt(lat1, lon1));
                ImVec2 p2 = project(getSpherePt(lat1, lon2));
                ImVec2 p3 = project(getSpherePt(lat2, lon2));
                ImVec2 p4 = project(getSpherePt(lat2, lon1));

                if (p1.x < 900 || p2.x < 900 || p3.x < 900 || p4.x < 900) {
                    safeTri(p1, p2, p3, col);
                    safeTri(p1, p3, p4, col);
                }
            }
        }

        // ════════════════════════════════════════════════════════════════
        // 3. 地平线（纬度=0，128段）────────────────────────────────────
        // ════════════════════════════════════════════════════════════════
        for (int j = 0; j < 128; j++) {
            float lon1 = (float)j       / 128.f * 360.f;
            float lon2 = (float)(j + 1) / 128.f * 360.f;
            safeLine(project(getSpherePt(0, lon1)),
                     project(getSpherePt(0, lon2)),
                     IM_COL32(255, 255, 255, 230), 1.8f);
        }

        // ════════════════════════════════════════════════════════════════
        // 4. 纬线（俯仰刻度线）-80°~+80°，每10° ─────────────────────
        // ════════════════════════════════════════════════════════════════
        for (int lat = -80; lat <= 80; lat += 10) {
            if (lat == 0) continue;  // 0°即地平线，已在上面绘制
            bool isMain = (lat % 30 == 0);
            ImU32 lc = isMain ? IM_COL32(255, 255, 220, 150)
                              : IM_COL32(255, 255, 220, 75);
            float thick = isMain ? 1.1f : 0.7f;

            for (int j = 0; j < 96; j++) {
                float lon1 = (float)j       / 96.f * 360.f;
                float lon2 = (float)(j + 1) / 96.f * 360.f;
                safeLine(project(getSpherePt((float)lat, lon1)),
                         project(getSpherePt((float)lat, lon2)),
                         lc, thick);
            }

            // 度数标注（主刻度，4个方位）—— 带边缘淡出
            if (isMain) {
                for (int deg = 0; deg < 360; deg += 90) {
                    Vec3 vL = getSpherePt((float)lat, (float)deg);
                    Vec3 pW = localRight * vL.x + localUp * vL.y + localNorth * vL.z;
                    Vec3 pR = rocketQuat.conjugate().rotate(pW);
                    if (pR.y > 0.1) {
                        float alpha = (float)fmin(1.0, pR.y * 3.0);
                        ImVec2 pl = {ctr.x + (float)(pR.x * R), ctr.y - (float)(pR.z * R)};
                        char buf[8]; snprintf(buf, 8, "%d", lat);
                        dl->AddText({pl.x - 10.f, pl.y - 7.f},
                                    IM_COL32(255, 255, 200, (int)(200.f * alpha)), buf);
                    }
                }
            }
        }

        // ════════════════════════════════════════════════════════════════
        // 5. 经线（垂直经线/子午线）每30° + 航向标签 ──────────────────
        // ════════════════════════════════════════════════════════════════
        const char* headMarks[] = {"N","30","60","E","120","150",
                                   "S","210","240","W","300","330"};
        for (int lon = 0; lon < 360; lon += 30) {
            // 经线：从南极到北极，每5°一段
            for (int i = 0; i < 36; i++) {
                float la1 = -90.f + (float)i * 5.f;
                float la2 = la1 + 5.f;
                safeLine(project(getSpherePt(la1, (float)lon)),
                         project(getSpherePt(la2, (float)lon)),
                         IM_COL32(255, 255, 255, 70), 0.7f);
            }

            // 航向标签（在赤道 lat=0 处）
            Vec3 vM = getSpherePt(0, (float)lon);
            Vec3 pW_m = localRight * vM.x + localUp * vM.y + localNorth * vM.z;
            Vec3 pR_m = rocketQuat.conjugate().rotate(pW_m);
            if (pR_m.y > 0.05) {
                float alpha = (float)fmin(1.0, pR_m.y * 4.0);
                ImVec2 pm = {ctr.x + (float)(pR_m.x * R), ctr.y - (float)(pR_m.z * R)};
                dl->AddText({pm.x - 9.f, pm.y - 8.f},
                            IM_COL32(255, 255, 25, (int)(230.f * alpha)),
                            headMarks[lon / 30]);
            }
        }

        // ════════════════════════════════════════════════════════════════
        // 6. 边缘滚转刻度环（在外圈内侧）───────────────────────────────
        // ════════════════════════════════════════════════════════════════
        int rollMarks[] = {0,30,60,90,120,150,180,-30,-60,-90,-120,-150};
        for (int rm : rollMarks) {
            // 使用经线投影：经线 lon=rm 在赤道(lat=0)处的方向
            Vec3 vm = getSpherePt(0, (float)rm);
            Vec3 pW = localRight * vm.x + localUp * vm.y + localNorth * vm.z;
            Vec3 pR = rocketQuat.conjugate().rotate(pW);
            // 投影到圆边缘
            if (pR.y > -0.5) {
                float alpha = (float)(pR.y > 0 ? 1.0 : fmax(0.3, (pR.y + 0.5) * 2.0));
                float ang = (float)atan2(-pR.z, pR.x);
                bool isZero = (rm == 0);
                bool is90   = (rm == 90 || rm == -90 || rm == 180);
                float tLen = isZero ? R * 0.13f : (is90 ? R * 0.09f : R * 0.06f);
                float th   = isZero ? 2.5f : 1.5f;
                ImU32 rc   = isZero
                    ? IM_COL32(255, 225, 45, (int)(245.f * alpha))
                    : (is90
                        ? IM_COL32(220, 220, 220, (int)(190.f * alpha))
                        : IM_COL32(180, 180, 180, (int)(150.f * alpha)));
                float ri = R - tLen - 1.f;
                float ro = R - 1.f;
                float ca = cosf(ang), sa = sinf(ang);
                dl->AddLine({ctr.x + ri * ca, ctr.y + ri * sa},
                            {ctr.x + ro * ca, ctr.y + ro * sa}, rc, th);
            }
        }

        // ════════════════════════════════════════════════════════════════
        // 7. 框架指针（固定顶部，黄色三角）─────────────────────────────
        // ════════════════════════════════════════════════════════════════
        {
            float ty = ctr.y - R - 4.f;
            dl->AddTriangleFilled(
                {ctr.x,       ty + 11.f},
                {ctr.x - 7.f, ty},
                {ctr.x + 7.f, ty},
                IM_COL32(255, 225, 45, 240));
            dl->AddTriangle(
                {ctr.x,       ty + 11.f},
                {ctr.x - 7.f, ty},
                {ctr.x + 7.f, ty},
                IM_COL32(0, 0, 0, 120), 1.f);
        }

        // ════════════════════════════════════════════════════════════════
        // 8. 外圈边框 ──────────────────────────────────────────────────
        // ════════════════════════════════════════════════════════════════
        dl->AddCircle(ctr, R, IM_COL32(160, 160, 160, 220), 64, 2.2f);

        // ════════════════════════════════════════════════════════════════
        // 9. 中心固定指示器（橙色"机翼"） ─────────────────────────────
        // ════════════════════════════════════════════════════════════════
        {
            // 半透明黑色底座
            dl->AddCircleFilled(ctr, R * 0.12f, IM_COL32(0, 0, 0, 102));
            float iw = R * 0.45f;
            float ih = R * 0.045f;
            // 左侧机翼
            dl->AddRectFilled({ctr.x - iw * 0.7f - iw * 0.52f, ctr.y - ih * 1.5f},
                              {ctr.x - iw * 0.7f,               ctr.y + ih * 1.5f},
                              IM_COL32(0, 0, 0, 200));
            dl->AddRectFilled({ctr.x - iw * 0.7f - iw * 0.5f, ctr.y - ih},
                              {ctr.x - iw * 0.7f,              ctr.y + ih},
                              IM_COL32(255, 153, 0, 255));
            // 右侧机翼
            dl->AddRectFilled({ctr.x + iw * 0.7f,               ctr.y - ih * 1.5f},
                              {ctr.x + iw * 0.7f + iw * 0.52f, ctr.y + ih * 1.5f},
                              IM_COL32(0, 0, 0, 200));
            dl->AddRectFilled({ctr.x + iw * 0.7f,              ctr.y - ih},
                              {ctr.x + iw * 0.7f + iw * 0.5f, ctr.y + ih},
                              IM_COL32(255, 153, 0, 255));
            // 中心方块
            dl->AddRectFilled({ctr.x - ih * 2.5f, ctr.y - ih * 2.5f},
                              {ctr.x + ih * 2.5f, ctr.y + ih * 2.5f},
                              IM_COL32(0, 0, 0, 200));
            dl->AddRectFilled({ctr.x - ih * 2.5f, ctr.y - ih * 2.5f},
                              {ctr.x + ih * 2.5f, ctr.y + ih * 2.5f},
                              IM_COL32(255, 153, 0, 255));
        }
    }

    // ================================================================
    // MENU
    // ================================================================
    void _drawMenu(int ww, int wh) {
        ImGui::SetNextWindowPos(ImVec2(ww*0.35f, wh*0.20f), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(ww*0.30f, wh*0.60f), ImGuiCond_Always);
        ImGui::Begin("##MainMenu", nullptr,
            ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove);

        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f,0.85f,1.0f,1.0f));
        ImGui::SetWindowFontScale(1.8f);
        float tw = ImGui::CalcTextSize("RocketSim 3D").x;
        ImGui::SetCursorPosX((ImGui::GetWindowWidth()-tw)*0.5f);
        ImGui::Text("RocketSim 3D");
        ImGui::SetWindowFontScale(1.0f);
        ImGui::PopStyleColor();
        ImGui::Separator(); ImGui::Dummy(ImVec2(0,12));

        float bw = ImGui::GetContentRegionAvail().x * 0.72f;
        float bx = (ImGui::GetContentRegionAvail().x - bw) * 0.5f;
        auto btn = [&](const char* label, ImVec4 c, ImVec4 ch, ImVec4 ca) -> bool {
            ImGui::SetCursorPosX(bx);
            ImGui::PushStyleColor(ImGuiCol_Button, c);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ch);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ca);
            bool r = ImGui::Button(label, ImVec2(bw, 46));
            ImGui::PopStyleColor(3);
            ImGui::Dummy(ImVec2(0,6));
            return r;
        };

        if (btn("新任务", {0.10f,0.55f,0.20f,0.9f},{0.20f,0.75f,0.35f,1},{0.05f,0.40f,0.15f,1})) {
            goToAgency = true;
        }

        bool hasSave = SaveSystem::HasSaveFile();
        if (hasSave) {
            if (btn("继续游戏", {0.15f,0.40f,0.60f,0.9f},{0.25f,0.55f,0.80f,1},{0.10f,0.30f,0.50f,1})) {
                GameContext::getInstance().skip_builder = true;
                goToFlight = true;
            }
        }

        if (btn("设置", {0.20f,0.20f,0.30f,0.9f},{0.30f,0.30f,0.45f,1},{0.15f,0.15f,0.25f,1})) {
            state = VkGameState::SETTINGS;
        }

        if (btn("退出", {0.55f,0.10f,0.10f,0.9f},{0.75f,0.20f,0.20f,1},{0.40f,0.05f,0.05f,1})) {
            goToExit = true;
        }

        ImGui::Dummy(ImVec2(0,10));
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f,0.4f,0.4f,1));
        ImGui::SetWindowFontScale(0.82f);
        ImGui::TextWrapped("ESC 退出 | 右键拖拽旋转视角");
        ImGui::SetWindowFontScale(1.0f);
        ImGui::PopStyleColor();
        ImGui::End();
    }

    // ================================================================
    // SETTINGS
    // ================================================================
    void _drawSettings(int ww, int wh) {
        ImGui::SetNextWindowPos(ImVec2(ww*0.30f, wh*0.20f), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(ww*0.40f, wh*0.60f), ImGuiCond_Always);
        ImGui::Begin("设置", nullptr, ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove);

        ImGui::SeparatorText("语言 / Language");
        {
            static const char* langOpts[] = { "中文", "English" };
            ImGui::Combo("界面语言", &settingsLanguage, langOpts, 2);
            ImGui::TextDisabled("重启后部分文本生效");
        }

        ImGui::SeparatorText("图形");
        ImGui::Checkbox("垂直同步 (VSync)", &settingsVSync);
        ImGui::TextDisabled("分辨率: %d × %d", ww, wh);

        ImGui::SeparatorText("渲染器");
        ImGui::TextDisabled("后端: Vulkan 1.3 动态渲染");
        ImGui::TextDisabled("AA: TAA (时序抗锯齿)");
        ImGui::TextDisabled("HDR: RGBA16F ping-pong");

        ImGui::SeparatorText("关于");
        ImGui::TextDisabled("RocketSim 3D — Vulkan 渲染模式");
        ImGui::TextDisabled("使用 Dear ImGui 绘制所有 UI");

        ImGui::Separator(); ImGui::Dummy(ImVec2(0,8));
        if (ImGui::Button("返回主菜单", ImVec2(-1, 40))) {
            state = VkGameState::MENU;
        }
        ImGui::End();
    }

    // ================================================================
    // AGENCY HUB — 航天中心
    // ================================================================
    void _drawAgency(int ww, int wh) {
        GameContext& gc = GameContext::getInstance();
        AgencyState& ag = gc.agency_state;
        FactorySystem& factory = gc.factory;

        // ── 左侧：总览 ──────────────────────────────────────────────
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(300, (float)wh-20), ImGuiCond_Always);
        ImGui::Begin("航天局总部", nullptr, ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove);

        // 资源仪表板
        ImGui::TextColored({0.4f,0.85f,1.0f,1}, "── 资源概况 ──");
        ImGui::Separator();

        auto statRow = [](const char* label, const std::string& val, ImVec4 col={1,0.9f,0.5f,1}) {
            ImGui::Text("%-10s", label); ImGui::SameLine(110);
            ImGui::TextColored(col, "%s", val.c_str());
        };
        statRow("资金:", _fmtFunds(ag.funds), {0.3f,1.0f,0.5f,1});
        {
            ImGui::Text("%-10s", "声望:"); ImGui::SameLine(110);
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.3f,0.7f,1.0f,1));
            char repBuf[16]; snprintf(repBuf,16,"%.0f/100",ag.reputation);
            ImGui::ProgressBar((float)(ag.reputation/100.0), ImVec2(160,14), repBuf);
            ImGui::PopStyleColor();
        }
        statRow("科学点:", std::to_string((int)ag.science)+" pts", {0.8f,0.5f,1.0f,1});

        ImGui::Dummy(ImVec2(0,10));
        ImGui::TextColored({0.4f,0.85f,1.0f,1}, "── 库存 ──");
        ImGui::Separator();

        // 分类显示库存
        auto showInvGroup = [&](const char* title, int from, int to) {
            bool any = false;
            for (int i = from; i <= to && i < ITEM_TYPE_COUNT; i++) {
                int cnt = ag.getItemCount((ItemType)i);
                if (cnt > 0) any = true;
            }
            if (!any) return;
            ImGui::TextDisabled("%s", title);
            for (int i = from; i <= to && i < ITEM_TYPE_COUNT; i++) {
                int cnt = ag.getItemCount((ItemType)i);
                if (cnt <= 0) continue;
                ImGui::Text("  %-12s", itemName((ItemType)i));
                ImGui::SameLine(140); ImGui::TextColored({1,0.9f,0.5f,1}, "x%d", cnt);
            }
        };
        showInvGroup("原材料", 0, 4);
        showInvGroup("加工材料", 5, 9);
        showInvGroup("火箭零件", 10, ITEM_TYPE_COUNT-1);
        if (ag.inventory.empty())
            ImGui::TextDisabled("  （库存为空）");

        ImGui::Dummy(ImVec2(0,10));
        ImGui::TextColored({0.4f,0.85f,1.0f,1}, "── 工厂 ──");
        ImGui::Separator();
        ImGui::Text("节点数: %d", (int)factory.nodes.size());
        ImGui::Text("传送带: %d", (int)factory.belts.size());
        ImGui::Text("发电: %.1f / %.1f kW", factory.total_power_gen, factory.total_power_req);

        ImGui::End();

        // ── 右侧：操作按钮 ────────────────────────────────────────────
        ImGui::SetNextWindowPos(ImVec2((float)ww-320, 10), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(310, (float)wh-20), ImGuiCond_Always);
        ImGui::Begin("##AgencyActions", nullptr,
            ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove);

        ImGui::TextColored({0.4f,0.85f,1.0f,1}, "行动中心");
        ImGui::Separator(); ImGui::Dummy(ImVec2(0,8));

        float abw = ImGui::GetContentRegionAvail().x;

        // 前往工坊
        ImGui::PushStyleColor(ImGuiCol_Button,        {0.10f,0.55f,0.20f,0.9f});
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, {0.20f,0.75f,0.35f,1.0f});
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  {0.05f,0.40f,0.15f,1.0f});
        if (ImGui::Button("进入组装车间 (VAB)", ImVec2(abw, 52))) {
            goToWorkshop = true;
            workshopInit = false;
        }
        ImGui::PopStyleColor(3);
        ImGui::TextDisabled("在此设计并组装火箭");
        ImGui::Dummy(ImVec2(0,10));

        // 前往工厂
        ImGui::PushStyleColor(ImGuiCol_Button,        {0.40f,0.25f,0.10f,0.9f});
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, {0.60f,0.40f,0.15f,1.0f});
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  {0.30f,0.20f,0.05f,1.0f});
        if (ImGui::Button("工厂管理", ImVec2(abw, 52))) {
            state = VkGameState::FACTORY;
        }
        ImGui::PopStyleColor(3);
        ImGui::TextDisabled("管理生产线和资源采集");
        ImGui::Dummy(ImVec2(0,10));

        // 简单任务：购买零件
        ImGui::SeparatorText("市场");
        struct MarketItem { const char* name; ItemType it; double cost; int qty; };
        static const MarketItem market[] = {
            {"购买铁矿石 ×10",  ITEM_IRON_ORE,   500.0,  10},
            {"购买钢铁 ×5",    ITEM_STEEL,      2000.0,  5},
            {"购买燃料 ×1t",   ITEM_FUEL,       5000.0,  1},
            {"购买发动机 ×1",  PART_ENGINE,    50000.0,  1},
        };
        for (auto& mi : market) {
            bool canBuy = ag.funds >= mi.cost;
            if (!canBuy) ImGui::BeginDisabled();
            if (ImGui::Button(mi.name, ImVec2(abw*0.72f, 30))) {
                ag.funds -= mi.cost;
                ag.addItem(mi.it, mi.qty);
            }
            if (!canBuy) ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::TextDisabled("%s", _fmtFunds(mi.cost).c_str());
        }

        ImGui::Dummy(ImVec2(0,20));
        ImGui::Separator();
        if (ImGui::Button("返回主菜单", ImVec2(abw, 36))) {
            SaveSystem::SaveAgencyFactory(ag, factory);
            state = VkGameState::MENU;
        }

        ImGui::End();
    }

    // ================================================================
    // FACTORY — 工厂流水线（网格视图 + 建筑放置 + 传送带）
    // ================================================================
    void _drawFactory(int ww, int wh) {
        GameContext& gc = GameContext::getInstance();
        AgencyState& ag = gc.agency_state;
        FactorySystem& factory = gc.factory;

        // ── 辅助：网格 ↔ 屏幕坐标转换 ────────────────────────────
        float cs = fac_cellSize * fac_zoom;
        float gridW_px = FAC_GRID_W * cs;
        float gridH_px = FAC_GRID_H * cs;
        // 网格左上角在屏幕空间的位置
        float gridOriginX = (float)ww * 0.5f - gridW_px * 0.5f + fac_camX;
        float gridOriginY = (float)wh * 0.5f - gridH_px * 0.5f + fac_camY;

        auto gridToScreen = [&](int gx, int gy, float& sx, float& sy) {
            sx = gridOriginX + (gx + 0.5f) * cs;
            sy = gridOriginY + (gy + 0.5f) * cs;
        };
        auto screenToGrid = [&](float mx, float my, int& gx, int& gy) {
            gx = (int)floorf((mx - gridOriginX) / cs);
            gy = (int)floorf((my - gridOriginY) / cs);
        };
        auto gridValid = [&](int gx, int gy) -> bool {
            return gx >= 0 && gx < FAC_GRID_W && gy >= 0 && gy < FAC_GRID_H;
        };
        auto findNodeAtGrid = [&](int gx, int gy) -> int {
            for (auto& n : factory.nodes) if (n.grid_x == gx && n.grid_y == gy) return n.id;
            return -1;
        };

        // ── 节点颜色 ────────────────────────────────────────────
        auto nodeColor = [](FactoryNodeType t, float& r, float& g, float& b) {
            switch (t) {
                case NODE_MINER:       r=0.6f;g=0.4f;b=0.2f;break;
                case NODE_SMELTER:     r=0.8f;g=0.5f;b=0.1f;break;
                case NODE_ASSEMBLER:   r=0.2f;g=0.7f;b=0.3f;break;
                case NODE_STORAGE:     r=0.4f;g=0.4f;b=0.6f;break;
                case NODE_POWER_PLANT: r=0.9f;g=0.9f;b=0.2f;break;
                case NODE_MARKET:      r=1.0f;g=0.8f;b=0.2f;break;
                default:               r=0.5f;g=0.5f;b=0.5f;break;
            }
        };

        // ── 鼠标交互 ────────────────────────────────────────────
        ImGuiIO& io = ImGui::GetIO();
        float mx = io.MousePos.x, my = io.MousePos.y;
        bool lmb = ImGui::IsMouseDown(0);
        bool rmb = ImGui::IsMouseDown(1);
        bool lmbClicked = ImGui::IsMouseClicked(0);
        bool rmbClicked = ImGui::IsMouseClicked(1);
        fac_lastMX = mx; fac_lastMY = my;

        // 计算悬停网格坐标
        screenToGrid(mx, my, fac_hoverGX, fac_hoverGY);
        if (!gridValid(fac_hoverGX, fac_hoverGY)) { fac_hoverGX = -1; fac_hoverGY = -1; }

        // 右键拖拽平移
        if (rmb && !fac_rmbDown) {
            fac_dragStartX = mx; fac_dragStartY = my;
        }
        if (rmb && fac_rmbDown) {
            fac_camX += mx - fac_dragStartX;
            fac_camY += my - fac_dragStartY;
            fac_dragStartX = mx; fac_dragStartY = my;
        }
        fac_rmbDown = rmb;

        // 滚轮缩放
        float wheel = io.MouseWheel;
        if (wheel != 0.0f) {
            // 以鼠标位置为中心缩放
            float oldZoom = fac_zoom;
            fac_zoom = std::max(0.3f, std::min(4.0f, fac_zoom * powf(1.15f, wheel)));
            float ratio = fac_zoom / oldZoom;
            fac_camX = mx - (mx - fac_camX) * ratio;
            fac_camY = my - (my - fac_camY) * ratio;
        }

        // ── 左键交互 ────────────────────────────────────────────
        if (lmbClicked && gridValid(fac_hoverGX, fac_hoverGY)) {
            int nodeAt = findNodeAtGrid(fac_hoverGX, fac_hoverGY);

            if (fac_placing) {
                // 放置模式：在空格子上放置建筑
                if (nodeAt < 0) {
                    factory.addNode(fac_placeType, fac_hoverGX, fac_hoverGY);
                }
            } else if (fac_beltMode) {
                // 传送带模式
                if (nodeAt >= 0) {
                    if (fac_beltFromId < 0) {
                        fac_beltFromId = nodeAt;
                    } else if (nodeAt != fac_beltFromId) {
                        // 创建传送带
                        ConveyorBelt belt;
                        belt.from_node_id = fac_beltFromId;
                        belt.to_node_id = nodeAt;
                        factory.belts.push_back(belt);
                        fac_beltFromId = -1;
                    }
                }
            } else if (fac_deleteMode) {
                // 删除模式
                if (nodeAt >= 0) {
                    factory.removeNode(nodeAt);
                    if (factorySelectedNode == nodeAt) factorySelectedNode = -1;
                    // 也删除相关传送带
                    factory.belts.erase(
                        std::remove_if(factory.belts.begin(), factory.belts.end(),
                            [&](const ConveyorBelt& b) { return b.from_node_id == nodeAt || b.to_node_id == nodeAt; }),
                        factory.belts.end());
                }
            } else {
                // 选择模式
                if (nodeAt >= 0) {
                    factorySelectedNode = nodeAt;
                    // 退出其他模式
                    fac_placing = false;
                    fac_beltMode = false;
                    fac_deleteMode = false;
                    fac_beltFromId = -1;
                } else {
                    factorySelectedNode = -1;
                }
            }
        }

        // ── 绘制主画布 ──────────────────────────────────────────
        ImDrawList* dl = ImGui::GetForegroundDrawList();

        // 背景
        dl->AddRectFilled({0,0}, {(float)ww,(float)wh}, IM_COL32(10,12,18,255));

        // 网格线
        ImU32 gridCol = IM_COL32(30,34,44,180);
        ImU32 gridColMajor = IM_COL32(45,50,62,220);
        for (int x = 0; x <= FAC_GRID_W; x++) {
            float sx = gridOriginX + x * cs;
            ImU32 col = (x % 5 == 0) ? gridColMajor : gridCol;
            dl->AddLine({sx, gridOriginY}, {sx, gridOriginY + gridH_px}, col, 1.0f);
        }
        for (int y = 0; y <= FAC_GRID_H; y++) {
            float sy = gridOriginY + y * cs;
            ImU32 col = (y % 5 == 0) ? gridColMajor : gridCol;
            dl->AddLine({gridOriginX, sy}, {gridOriginX + gridW_px, sy}, col, 1.0f);
        }

        // ── 绘制传送带 ──────────────────────────────────────────
        float time = (float)ImGui::GetTime();
        for (auto& belt : factory.belts) {
            const FactoryNode* from = factory.findNode(belt.from_node_id);
            const FactoryNode* to   = factory.findNode(belt.to_node_id);
            if (!from || !to) continue;

            float fx, fy, tx, ty;
            gridToScreen(from->grid_x, from->grid_y, fx, fy);
            gridToScreen(to->grid_x, to->grid_y, tx, ty);

            float pulse = 0.7f + 0.3f * sinf(time * 6.0f + (float)belt.from_node_id);
            ImU32 beltCol = IM_COL32((int)(80*pulse),(int)(140*pulse),(int)(100*pulse),200);

            // L 形路径：先水平再垂直
            dl->AddLine({fx, fy}, {tx, fy}, beltCol, 3.0f);
            dl->AddLine({tx, fy}, {tx, ty}, beltCol, 3.0f);

            // 在途物品
            for (auto const& item : belt.items) {
                float ix, iy;
                if (item.progress < 0.5f) {
                    float t2 = item.progress * 2.0f;
                    ix = fx + (tx - fx) * t2;
                    iy = fy;
                } else {
                    float t2 = (item.progress - 0.5f) * 2.0f;
                    ix = tx;
                    iy = fy + (ty - fy) * t2;
                }
                const ItemInfo& info = GetItemInfo(item.type);
                float isz = cs * 0.15f;
                ImU32 itemCol = IM_COL32((int)(info.icon_r*255),(int)(info.icon_g*255),(int)(info.icon_b*255),240);
                dl->AddRectFilled({ix-isz, iy-isz}, {ix+isz, iy+isz}, itemCol);
                dl->AddRect({ix-isz, iy-isz}, {ix+isz, iy+isz}, IM_COL32(255,255,255,80));
            }
        }

        // 传送带绘制中的预览线
        if (fac_beltMode && fac_beltFromId >= 0) {
            const FactoryNode* from = factory.findNode(fac_beltFromId);
            if (from && gridValid(fac_hoverGX, fac_hoverGY)) {
                float fx, fy, hx, hy;
                gridToScreen(from->grid_x, from->grid_y, fx, fy);
                gridToScreen(fac_hoverGX, fac_hoverGY, hx, hy);
                float p2 = 0.5f + 0.5f * sinf(time * 10.0f);
                ImU32 prevCol = IM_COL32((int)(70*p2),(int)(220*p2),(int)(90*p2),160);
                dl->AddLine({fx, fy}, {hx, fy}, prevCol, 2.5f);
                dl->AddLine({hx, fy}, {hx, hy}, prevCol, 2.5f);
            }
        }

        // ── 绘制建筑 ────────────────────────────────────────────
        for (auto& node : factory.nodes) {
            float sx, sy;
            gridToScreen(node.grid_x, node.grid_y, sx, sy);

            float nr, ng, nb;
            nodeColor(node.type, nr, ng, nb);
            float bsz = cs * 0.42f;

            bool isSel = (node.id == factorySelectedNode);
            bool isHov = (fac_hoverGX == node.grid_x && fac_hoverGY == node.grid_y);

            // 建筑主体
            ImU32 bodyCol = IM_COL32((int)(nr*200),(int)(ng*200),(int)(nb*200),230);
            dl->AddRectFilled({sx-bsz, sy-bsz}, {sx+bsz, sy+bsz}, bodyCol, 4.0f);

            // 悬停/选中高亮
            if (isSel) {
                float p3 = 0.6f + 0.4f * sinf(time * 5.0f);
                dl->AddRect({sx-bsz-2, sy-bsz-2}, {sx+bsz+2, sy+bsz+2},
                    IM_COL32((int)(180*p3),(int)(180*p3),255,255), 4.0f, 0, 3.0f);
            } else if (isHov) {
                dl->AddRect({sx-bsz-1, sy-bsz-1}, {sx+bsz+1, sy+bsz+1},
                    IM_COL32(200,200,220,180), 4.0f, 0, 2.0f);
            }

            // 进度条（非仓库类建筑）
            if (node.progress > 0.0f && node.type != NODE_STORAGE && node.type != NODE_MARKET) {
                float barW = bsz * 1.6f, barH = bsz * 0.28f;
                float barY = sy - bsz * 0.55f;
                dl->AddRectFilled({sx-barW/2, barY-barH/2}, {sx+barW/2, barY+barH/2}, IM_COL32(20,20,20,200));
                float fillW = barW * std::min(1.0f, node.progress);
                dl->AddRectFilled({sx-barW/2, barY-barH/2}, {sx-barW/2+fillW, barY+barH/2},
                    IM_COL32(50,220,70,220));
            }

            // 标签
            const char* nlabel = NodeTypeName(node.type);
            float textW = ImGui::CalcTextSize(nlabel).x * 0.65f;
            dl->AddText(nullptr, 11.0f * std::min(1.5f, fac_zoom),
                {sx - textW * 0.5f, sy + bsz + 2.0f},
                IM_COL32(220,220,230,230), nlabel);

            // 配方标签（熔炉/组装机）
            if ((node.type == NODE_SMELTER || node.type == NODE_ASSEMBLER) && node.recipe_index >= 0) {
                int rc = 0;
                const Recipe* recipes = GetRecipes(rc);
                if (node.recipe_index < rc) {
                    const char* rname = recipes[node.recipe_index].name;
                    float rtw = ImGui::CalcTextSize(rname).x * 0.55f;
                    dl->AddText(nullptr, 9.0f * std::min(1.5f, fac_zoom),
                        {sx - rtw * 0.5f, sy - bsz - 12.0f},
                        IM_COL32(255,220,120,200), rname);
                }
            }
            // 矿机标签
            if (node.type == NODE_MINER) {
                const char* mname = itemName(node.mine_output);
                float mtw = ImGui::CalcTextSize(mname).x * 0.55f;
                dl->AddText(nullptr, 9.0f * std::min(1.5f, fac_zoom),
                    {sx - mtw * 0.5f, sy - bsz - 12.0f},
                    IM_COL32(120,200,255,200), mname);
            }
        }

        // ── 放置预览幽灵 ────────────────────────────────────────
        if (fac_placing && gridValid(fac_hoverGX, fac_hoverGY)) {
            int existing = findNodeAtGrid(fac_hoverGX, fac_hoverGY);
            float gsx, gsy;
            gridToScreen(fac_hoverGX, fac_hoverGY, gsx, gsy);
            float nr2, ng2, nb2;
            nodeColor(fac_placeType, nr2, ng2, nb2);
            float gbsz = cs * 0.42f;
            float alpha = (existing >= 0) ? 0.25f : 0.55f;
            float p4 = 0.7f + 0.3f * sinf(time * 7.0f);
            ImU32 ghostCol = IM_COL32((int)(nr2*255*p4),(int)(ng2*255*p4),(int)(nb2*255*p4),(int)(alpha*255));
            dl->AddRectFilled({gsx-gbsz, gsy-gbsz}, {gsx+gbsz, gsy+gbsz}, ghostCol, 4.0f);
            dl->AddRect({gsx-gbsz, gsy-gbsz}, {gsx+gbsz, gsy+gbsz}, IM_COL32(255,255,255,(int)(alpha*180)), 4.0f, 0, 2.0f);
        }

        // ── 删除模式悬停高亮 ────────────────────────────────────
        if (fac_deleteMode && gridValid(fac_hoverGX, fac_hoverGY)) {
            int delNode = findNodeAtGrid(fac_hoverGX, fac_hoverGY);
            if (delNode >= 0) {
                float dsx, dsy;
                gridToScreen(fac_hoverGX, fac_hoverGY, dsx, dsy);
                float dbsz = cs * 0.45f;
                dl->AddRect({dsx-dbsz, dsy-dbsz}, {dsx+dbsz, dsy+dbsz},
                    IM_COL32(255,40,40,200), 4.0f, 0, 3.0f);
                dl->AddLine({dsx-dbsz*0.6f, dsy-dbsz*0.6f}, {dsx+dbsz*0.6f, dsy+dbsz*0.6f},
                    IM_COL32(255,60,60,220), 2.5f);
                dl->AddLine({dsx+dbsz*0.6f, dsy-dbsz*0.6f}, {dsx-dbsz*0.6f, dsy+dbsz*0.6f},
                    IM_COL32(255,60,60,220), 2.5f);
            }
        }

        // ── 底部状态栏 ──────────────────────────────────────────
        float sbY = (float)wh - 52.0f;
        dl->AddRectFilled({0, sbY}, {(float)ww, (float)wh}, IM_COL32(12,14,20,240));
        dl->AddLine({0, sbY}, {(float)ww, sbY}, IM_COL32(60,120,200,150), 2.0f);

        char statusBuf[256];
        const char* modeStr = fac_placing ? "放置" : (fac_beltMode ? "传送带" : (fac_deleteMode ? "删除" : "选择"));
        snprintf(statusBuf, sizeof(statusBuf),
            "建筑: %d  |  传送带: %d  |  发电: %.0f/%.0f kW (%.0f%%)  |  资金: %.0f ¥  |  模式: %s",
            (int)factory.nodes.size(), (int)factory.belts.size(),
            factory.total_power_gen, factory.total_power_req,
            factory.power_efficiency * 100.0f, ag.funds, modeStr);
        dl->AddText(nullptr, 14.0f, {16.0f, sbY + 16.0f},
            IM_COL32(180,190,210,230), statusBuf);

        // ── 顶部工具栏 ──────────────────────────────────────────
        float tbY = 8.0f;
        float tbH = 48.0f;
        float tbW = (float)ww - 16.0f;
        dl->AddRectFilled({8.0f, tbY}, {8.0f + tbW, tbY + tbH}, IM_COL32(15,17,25,235), 6.0f);
        dl->AddRect({8.0f, tbY}, {8.0f + tbW, tbY + tbH}, IM_COL32(50,100,180,120), 6.0f);

        dl->AddText(nullptr, 16.0f, {20.0f, tbY + 14.0f},
            IM_COL32(230,180,60,255), "工厂流水线");

        // 工具栏按钮
        struct ToolBtn { const char* label; FactoryNodeType type; float r,g,b; bool isBelt; bool isDel; };
        ToolBtn btns[] = {
            {"矿机",   NODE_MINER,       0.6f,0.4f,0.2f, false,false},
            {"熔炉",   NODE_SMELTER,     0.8f,0.5f,0.1f, false,false},
            {"组装",   NODE_ASSEMBLER,   0.2f,0.7f,0.3f, false,false},
            {"仓库",   NODE_STORAGE,     0.4f,0.4f,0.6f, false,false},
            {"电厂",   NODE_POWER_PLANT, 0.9f,0.9f,0.2f, false,false},
            {"市场",   NODE_MARKET,      1.0f,0.8f,0.2f, false,false},
            {"传送带", NODE_MINER,       0.3f,0.8f,0.4f, true, false},
            {"删除",   NODE_MINER,       0.9f,0.2f,0.1f, false,true},
        };
        int numBtns = 8;
        float btnStartX = 180.0f;
        float btnSpacing = 72.0f;
        float btnW = 62.0f, btnH = 34.0f;

        for (int i = 0; i < numBtns; i++) {
            float bx = btnStartX + i * btnSpacing;
            float by = tbY + (tbH - btnH) * 0.5f;

            bool active = false;
            if (btns[i].isBelt) active = fac_beltMode;
            else if (btns[i].isDel) active = fac_deleteMode;
            else active = fac_placing && fac_placeType == btns[i].type;

            // 鼠标检测
            bool hover = (mx >= bx && mx <= bx + btnW && my >= by && my <= by + btnH);

            ImU32 btnCol;
            if (active) {
                float p = 0.7f + 0.3f * sinf(time * 5.0f);
                btnCol = IM_COL32((int)(btns[i].r*255*p),(int)(btns[i].g*255*p),(int)(btns[i].b*255*p),240);
            } else if (hover) {
                btnCol = IM_COL32((int)(btns[i].r*180),(int)(btns[i].g*180),(int)(btns[i].b*180),200);
            } else {
                btnCol = IM_COL32(35,38,48,200);
            }
            dl->AddRectFilled({bx, by}, {bx+btnW, by+btnH}, btnCol, 4.0f);

            float tw2 = ImGui::CalcTextSize(btns[i].label).x * 0.7f;
            dl->AddText(nullptr, 13.0f,
                {bx + (btnW - tw2) * 0.5f, by + (btnH - 14.0f) * 0.5f},
                active ? IM_COL32(255,255,255,255) : IM_COL32(200,200,210,220),
                btns[i].label);

            // 点击处理
            if (hover && lmbClicked) {
                if (btns[i].isBelt) {
                    fac_beltMode = !fac_beltMode;
                    fac_placing = false;
                    fac_deleteMode = false;
                    fac_beltFromId = -1;
                    factorySelectedNode = -1;
                } else if (btns[i].isDel) {
                    fac_deleteMode = !fac_deleteMode;
                    fac_placing = false;
                    fac_beltMode = false;
                    fac_beltFromId = -1;
                    factorySelectedNode = -1;
                } else {
                    if (fac_placing && fac_placeType == btns[i].type) {
                        fac_placing = false; // 再次点击取消
                    } else {
                        fac_placing = true;
                        fac_placeType = btns[i].type;
                        fac_beltMode = false;
                        fac_deleteMode = false;
                        fac_beltFromId = -1;
                    }
                }
            }
        }

        // ── 右侧详情面板 ────────────────────────────────────────
        if (factorySelectedNode >= 0) {
            const FactoryNode* selNode = factory.findNode(factorySelectedNode);
            if (selNode) {
                float panelW = 240.0f;
                float panelX = (float)ww - panelW - 12.0f;
                float panelY = tbY + tbH + 16.0f;
                float panelH = std::min(480.0f, (float)wh - panelY - sbY - 20.0f);

                dl->AddRectFilled({panelX, panelY}, {panelX+panelW, panelY+panelH},
                    IM_COL32(16,18,28,240), 6.0f);
                dl->AddRect({panelX, panelY}, {panelX+panelW, panelY+panelH},
                    IM_COL32(50,100,180,100), 6.0f);

                float py = panelY + 10.0f;

                // 标题
                float nr3, ng3, nb3;
                nodeColor(selNode->type, nr3, ng3, nb3);
                dl->AddText(nullptr, 15.0f, {panelX + panelW*0.5f - 40.0f, py},
                    IM_COL32((int)(nr3*255),(int)(ng3*255),(int)(nb3*255),255),
                    NodeTypeName(selNode->type));
                py += 22.0f;

                dl->AddText(nullptr, 12.0f, {panelX + 10.0f, py},
                    IM_COL32(180,180,190,200), "状态: 运行中");
                py += 16.0f;

                char pbuf[32];
                snprintf(pbuf, 32, "进度: %.0f%%", selNode->progress * 100.0f);
                dl->AddText(nullptr, 12.0f, {panelX + 10.0f, py},
                    IM_COL32(200,200,210,200), pbuf);
                py += 20.0f;

                // 进度条
                dl->AddRectFilled({panelX+10, py}, {panelX+panelW-10, py+8}, IM_COL32(30,30,38,200));
                dl->AddRectFilled({panelX+10, py}, {panelX+10+(panelW-20)*selNode->progress, py+8},
                    IM_COL32(50,200,80,220));
                py += 20.0f;

                // 类型特定信息
                if (selNode->type == NODE_MINER) {
                    dl->AddText(nullptr, 12.0f, {panelX + 10.0f, py},
                        IM_COL32(120,200,255,200), "开采资源:");
                    py += 14.0f;
                    dl->AddText(nullptr, 13.0f, {panelX + 10.0f, py},
                        IM_COL32(220,180,60,220), itemName(selNode->mine_output));
                    py += 20.0f;
                } else if (selNode->type == NODE_SMELTER || selNode->type == NODE_ASSEMBLER) {
                    dl->AddText(nullptr, 12.0f, {panelX + 10.0f, py},
                        IM_COL32(120,200,255,200), "当前配方:");
                    py += 14.0f;
                    int rc2 = 0;
                    const Recipe* recipes2 = GetRecipes(rc2);
                    if (selNode->recipe_index >= 0 && selNode->recipe_index < rc2) {
                        dl->AddText(nullptr, 13.0f, {panelX + 10.0f, py},
                            IM_COL32(255,200,80,220), recipes2[selNode->recipe_index].name);
                    }
                    py += 24.0f;
                }

                // 输出缓冲
                if (!selNode->output_buffer.items.empty()) {
                    dl->AddText(nullptr, 12.0f, {panelX + 10.0f, py},
                        IM_COL32(150,150,160,200), "输出缓冲:");
                    py += 16.0f;
                    for (auto& [it, cnt] : selNode->output_buffer.items) {
                        char obuf[64];
                        snprintf(obuf, 64, "  %s x%d", itemName(it), cnt);
                        dl->AddText(nullptr, 11.0f, {panelX + 10.0f, py},
                            IM_COL32(200,200,210,200), obuf);
                        py += 14.0f;
                    }
                    py += 8.0f;
                }

                // 输入缓冲
                if (!selNode->input_buffer.items.empty()) {
                    dl->AddText(nullptr, 12.0f, {panelX + 10.0f, py},
                        IM_COL32(150,150,160,200), "输入缓冲:");
                    py += 16.0f;
                    for (auto& [it, cnt] : selNode->input_buffer.items) {
                        char ibuf[64];
                        snprintf(ibuf, 64, "  %s x%d", itemName(it), cnt);
                        dl->AddText(nullptr, 11.0f, {panelX + 10.0f, py},
                            IM_COL32(200,200,210,200), ibuf);
                        py += 14.0f;
                    }
                }
            }
        }

        // ── 操作提示 ────────────────────────────────────────────
        float hintY = sbY - 20.0f;
        dl->AddText(nullptr, 11.0f, {12.0f, hintY},
            IM_COL32(100,110,130,180),
            "左键: 放置/选择  |  右键拖拽: 平移  |  滚轮: 缩放  |  点击工具栏切换模式");

        // ── 返回按钮（左下角）───────────────────────────────────
        float backX = 12.0f, backY = sbY - 44.0f, backW = 180.0f, backH = 28.0f;
        bool backHover = (mx >= backX && mx <= backX+backW && my >= backY && my <= backY+backH);
        ImU32 backCol = backHover ? IM_COL32(50,60,80,230) : IM_COL32(30,35,50,200);
        dl->AddRectFilled({backX, backY}, {backX+backW, backY+backH}, backCol, 4.0f);
        dl->AddRect({backX, backY}, {backX+backW, backY+backH}, IM_COL32(80,100,140,150), 4.0f);
        float btw = ImGui::CalcTextSize("← 保存并返回航天中心").x * 0.65f;
        dl->AddText(nullptr, 12.0f,
            {backX + (backW - btw) * 0.5f, backY + (backH - 14.0f) * 0.5f},
            IM_COL32(180,200,230,230), "← 保存并返回航天中心");

        if (backHover && lmbClicked) {
            SaveSystem::SaveAgencyFactory(ag, factory);
            state = VkGameState::AGENCY;
        }

        // ── 工厂 tick ───────────────────────────────────────────
        factory.tick(io.DeltaTime, ag);
    }


    // ================================================================
    // ================================================================
    // WORKSHOP — 组装车间 (对标 OpenGL 时代 drawBuilderUI_KSP)
    // ================================================================
    void _drawWorkshop(int ww, int wh) {
        static const char* catNames[] = {"整流罩","指令舱","燃料箱","发动机","助推器","结构件"};
        static const char* catNamesFull[] = {
            "整流罩 / 鼻锥", "指令舱 / 探测器", "燃料箱", "发动机", "助推器", "结构件"
        };
        const float catColor[][3] = {
            {0.7f,0.7f,0.8f}, {0.3f,0.8f,0.6f}, {0.3f,0.6f,0.9f},
            {0.9f,0.5f,0.2f}, {0.9f,0.3f,0.3f}, {0.6f,0.6f,0.6f}
        };

        // ── 左侧：零件目录 ─────────────────────────────────────────────
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(300, (float)wh - 20), ImGuiCond_Always);
        ImGui::Begin("零件目录##ws", nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

        // 分类标签栏
        if (ImGui::BeginTabBar("##WsCats")) {
            for (int c = 0; c < CAT_COUNT; c++) {
                if (ImGui::BeginTabItem(catNames[c])) {
                    selectedCat = c;
                    ImGui::EndTabItem();
                }
            }
            ImGui::EndTabBar();
        }
        ImGui::TextColored({0.5f,0.6f,0.7f,1}, "%s", catNamesFull[selectedCat]);
        ImGui::Separator();

        // 零件网格（2列）
        float cellW = (ImGui::GetContentRegionAvail().x - 8) * 0.5f;
        const float cellH = 58.f;
        int col = 0;
        for (int i = 0; i < PART_CATALOG_SIZE; i++) {
            const PartDef& def = PART_CATALOG[i];
            if ((int)def.category != selectedCat) continue;

            ImGui::PushID(i);
            bool sel = (selectedPart == i);

            // 背景框颜色
            ImVec4 bgCol = sel
                ? ImVec4(0.15f, 0.40f, 0.70f, 0.95f)
                : ImVec4(0.12f, 0.14f, 0.20f, 0.80f);
            ImGui::PushStyleColor(ImGuiCol_ChildBg, bgCol);

            // 每个零件用一个子窗口绘制，以支持边框 + 点击
            ImGui::BeginChild(i + 1000, ImVec2(cellW, cellH), true,
                ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

            ImDrawList* cdl = ImGui::GetWindowDrawList();
            ImVec2 cp = ImGui::GetWindowPos();

            // 左侧彩色色块代表零件类型
            float swSz = 36.f;
            cdl->AddRectFilled({cp.x+4, cp.y+4}, {cp.x+4+swSz, cp.y+4+swSz},
                IM_COL32((int)(def.r*255),(int)(def.g*255),(int)(def.b*255),220), 4.f);
            cdl->AddRect({cp.x+4, cp.y+4}, {cp.x+4+swSz, cp.y+4+swSz},
                IM_COL32(200,200,200,120), 4.f);

            // 零件名
            ImGui::SetCursorPos({swSz + 10.f, 4.f});
            ImGui::SetWindowFontScale(0.88f);
            ImGui::TextColored(sel ? ImVec4{1,1,1,1} : ImVec4{0.9f,0.9f,0.9f,1}, "%s", def.name);
            // 简短规格
            ImGui::SetCursorPos({swSz + 10.f, 22.f});
            ImGui::SetWindowFontScale(0.72f);
            if (def.thrust > 0)
                ImGui::TextColored({0.9f,0.6f,0.2f,1}, "%.0f kN / ISP %.0fs", def.thrust/1000.f, def.isp);
            else if (def.fuel_capacity > 0)
                ImGui::TextColored({0.4f,0.8f,0.9f,1}, "%.0f t", def.fuel_capacity/1000.f);
            else
                ImGui::TextColored({0.6f,0.6f,0.6f,1}, "%.0f kg", def.dry_mass);
            ImGui::SetWindowFontScale(1.0f);

            // 点击：立即启动幽灵拖拽（左键点击目录 → 幽灵跟随鼠标）
            if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(0)) {
                selectedPart = i;
                _wsStartCatalogDrag(i);
            }

            // 悬停提示
            if (ImGui::IsWindowHovered()) {
                ImGui::BeginTooltip();
                ImGui::TextColored({0.4f,0.9f,1,1}, "%s", def.name);
                ImGui::SetWindowFontScale(0.85f);
                ImGui::Separator();
                ImGui::TextWrapped("%s", def.description);
                ImGui::Separator();
                ImGui::Text("空重:  %.0f kg", def.dry_mass);
                if (def.fuel_capacity > 0) ImGui::Text("燃料容量: %.0f kg", def.fuel_capacity);
                if (def.thrust > 0) {
                    ImGui::Text("推力:  %.0f kN", def.thrust / 1000.f);
                    ImGui::Text("ISP:   %.0f s",  def.isp);
                }
                ImGui::Text("高度: %.1f m  直径: %.1f m", def.height, def.diameter);
                ImGui::SetWindowFontScale(1.0f);
                ImGui::EndTooltip();
            }

            ImGui::EndChild();
            ImGui::PopStyleColor();

            if (col == 0) { ImGui::SameLine(0, 8); col = 1; }
            else { col = 0; }
            ImGui::PopID();
        }
        if (col == 1) { /* odd count, force newline */ }

        ImGui::Separator();
        // 添加零件按钮（含对称数量）
        {
            char addLabel[48];
            if (selectedPart >= 0)
                snprintf(addLabel, 48, "添加: %s  (x%d)", PART_CATALOG[selectedPart].name, ws_symmetry);
            else
                snprintf(addLabel, 48, "添加零件");
            bool canAdd = selectedPart >= 0;
            if (!canAdd) ImGui::BeginDisabled();
            ImGui::PushStyleColor(ImGuiCol_Button, {0.12f,0.45f,0.75f,0.9f});
            if (ImGui::Button(addLabel, ImVec2(-1, 34))) {
                _wsCancelDrag(); // 清除可能存在的拖拽状态
                assembly.addPart(selectedPart, -1, ws_symmetry);
                assembly.recalculate();
                ws_centerViz.lastAssemblyHash = 0;
            }
            ImGui::PopStyleColor();
            if (!canAdd) ImGui::EndDisabled();
        }

        // 对称模式切换
        {
            static const int symCycle[] = {1, 2, 4, 8};
            char symLabel[16]; snprintf(symLabel, 16, "对称: %dx##sym", ws_symmetry);
            if (ImGui::Button(symLabel, ImVec2(-1, 24))) {
                int cur = 0;
                for (int k = 0; k < 4; k++) if (symCycle[k] == ws_symmetry) { cur = k; break; }
                ws_symmetry = symCycle[(cur + 1) % 4];
            }
        }
        ImGui::End();

        // ── 右侧：组装清单 + 统计 + 发射 ────────────────────────────
        ImGui::SetNextWindowPos(ImVec2((float)ww - 315.f, 10), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(305, (float)wh - 20), ImGuiCond_Always);
        ImGui::Begin("火箭组装##ws", nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

        // ─ 组装清单 ─
        ImGui::TextColored({0.4f,0.85f,1,1}, "组装清单  (%d 件)", (int)assembly.parts.size());
        ImGui::SameLine(ImGui::GetWindowWidth() - 58.f);
        if (ImGui::SmallButton("清空##clr") && !assembly.parts.empty()) {
            assembly = RocketAssembly{};
            assembly.recalculate();
            ws_centerViz.lastAssemblyHash = 0;
        }
        ImGui::Separator();

        removePart = -1;
        float listH = std::max(80.f, (float)wh * 0.36f);
        ImGui::BeginChild("##WsParts", ImVec2(0, listH), true);

        int partCtxTrigger = -1;
        // 从顶端（最后添加）到底部（最先添加）显示
        for (int i = (int)assembly.parts.size() - 1; i >= 0; i--) {
            const PlacedPart& pp  = assembly.parts[i];
            const PartDef&    def = PART_CATALOG[pp.def_id];
            ImGui::PushID(i);

            // 分级标签
            ImVec4 stageCol[] = {
                {0.3f,0.8f,0.3f,1},{0.8f,0.7f,0.1f,1},{0.9f,0.4f,0.1f,1},
                {0.8f,0.2f,0.5f,1},{0.5f,0.2f,0.8f,1}
            };
            int sg = std::min(pp.stage, 4);
            ImGui::PushStyleColor(ImGuiCol_Button, stageCol[sg]);
            char slbl[8]; snprintf(slbl, 8, "S%d", pp.stage);
            if (ImGui::SmallButton(slbl)) {
                assembly.parts[i].stage = (pp.stage + 1) % 5;
                assembly.recalculate();
            }
            ImGui::PopStyleColor();
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("点击切换分级 (0=最先分离)");

            ImGui::SameLine();

            // 零件名（含对称数量提示）
            if (pp.symmetry > 1)
                ImGui::TextColored({0.85f,0.85f,1,1}, "%s x%d", def.name, pp.symmetry);
            else
                ImGui::TextUnformatted(def.name);

            // 悬停提示：零件详情
            if (ImGui::IsItemHovered()) {
                ImGui::BeginTooltip();
                ImGui::TextColored({0.4f,0.9f,1,1}, "%s", def.name);
                ImGui::Text("质量: %.0f kg  H=%.1fm  D=%.1fm", def.dry_mass, def.height, def.diameter);
                if (def.fuel_capacity > 0) ImGui::Text("燃料容量: %.0f kg", def.fuel_capacity);
                if (def.thrust > 0) ImGui::Text("推力: %.0f kN  ISP: %.0f s", def.thrust/1000.f, def.isp);
                ImGui::Text("位置: (%.2f, %.2f, %.2f)", pp.pos.x, pp.pos.y, pp.pos.z);
                ImGui::EndTooltip();
            }

            // 右键菜单触发
            if (ImGui::IsItemClicked(1) || (ImGui::IsItemHovered() && ImGui::IsMouseClicked(1)))
                partCtxTrigger = i;

            ImGui::SameLine(ImGui::GetContentRegionAvail().x - 44);
            // 复制按钮
            if (ImGui::SmallButton("+##dup")) {
                assembly.addPart(pp.def_id, -1, pp.symmetry);
                assembly.recalculate();
                ws_centerViz.lastAssemblyHash = 0;
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("复制此零件");
            ImGui::SameLine();
            // 删除按钮
            ImGui::PushStyleColor(ImGuiCol_Button, {0.6f,0.1f,0.1f,0.8f});
            if (ImGui::SmallButton("×")) removePart = i;
            ImGui::PopStyleColor();

            ImGui::PopID();
        }
        ImGui::EndChild();

        // 右键上下文弹出（来源：右侧列表 OR 3D 视口右键点击）
        if (partCtxTrigger >= 0) {
            ws_ctxIdx  = partCtxTrigger;
            ws_ctxOpen = true;
        }
        if (ws_ctxOpen && ws_ctxIdx >= 0 && ws_ctxIdx < (int)assembly.parts.size()) {
            ImGui::OpenPopup("##WsPartCtx");
            if (ImGui::BeginPopup("##WsPartCtx")) {
                auto& cp2 = assembly.parts[ws_ctxIdx];
                const PartDef& cpDef = PART_CATALOG[cp2.def_id];
                ImGui::TextColored({0.4f,0.9f,1,1}, "%s", cpDef.name);
                ImGui::Separator();
                if (ImGui::MenuItem("复制零件")) {
                    assembly.addPart(cp2.def_id, -1, cp2.symmetry);
                    assembly.recalculate();
                    ws_centerViz.lastAssemblyHash = 0;
                    ImGui::CloseCurrentPopup(); ws_ctxOpen = false;
                }
                char sg_lbl[32]; snprintf(sg_lbl, 32, "分级: S%d  (点击循环)", cp2.stage);
                if (ImGui::MenuItem(sg_lbl)) {
                    cp2.stage = (cp2.stage + 1) % 5;
                    assembly.recalculate();
                }
                ImGui::Separator();
                // 精调位置
                ImGui::TextColored({0.7f,0.7f,0.3f,1}, "精调位置:");
                auto adjRow = [&](const char* lbl, double& val, float step) {
                    ImGui::Text("  %s", lbl); ImGui::SameLine(80);
                    char pid[32]; snprintf(pid, 32, "+##p%s", lbl);
                    if (ImGui::SmallButton(pid)) { val += step; assembly.recalculate(); ws_centerViz.lastAssemblyHash = 0; }
                    ImGui::SameLine();
                    char mid[32]; snprintf(mid, 32, "-##p%s", lbl);
                    if (ImGui::SmallButton(mid)) { val -= step; assembly.recalculate(); ws_centerViz.lastAssemblyHash = 0; }
                    ImGui::SameLine(); ImGui::Text("%.2f", val);
                };
                adjRow("X", cp2.pos.x, 0.25);
                adjRow("Y", cp2.pos.y, 0.25);
                adjRow("Z", cp2.pos.z, 0.25);

                // 精调旋转
                ImGui::Separator();
                ImGui::TextColored({0.3f,0.8f,0.6f,1}, "精调旋转:");
                const float kDeg2Rad = 3.14159f / 180.0f;
                auto adjRot = [&](const char* lbl, Vec3 axis, float deg) {
                    ImGui::Text("  %s", lbl); ImGui::SameLine(80);
                    char pid[32]; snprintf(pid, 32, "+##r%s", lbl);
                    if (ImGui::SmallButton(pid)) {
                        cp2.rot = cp2.rot * Quat::fromAxisAngle(axis, deg * kDeg2Rad);
                        cp2.rot = cp2.rot.normalized();
                        assembly.recalculate(); ws_centerViz.lastAssemblyHash = 0;
                    }
                    ImGui::SameLine();
                    char mid[32]; snprintf(mid, 32, "-##r%s", lbl);
                    if (ImGui::SmallButton(mid)) {
                        cp2.rot = cp2.rot * Quat::fromAxisAngle(axis, -deg * kDeg2Rad);
                        cp2.rot = cp2.rot.normalized();
                        assembly.recalculate(); ws_centerViz.lastAssemblyHash = 0;
                    }
                    ImGui::SameLine(); ImGui::TextDisabled("%.0f deg", deg);
                };
                adjRot("绕Y(偏航)", Vec3(0,1,0), 15.0f);
                adjRot("绕X(俯仰)", Vec3(1,0,0), 15.0f);
                adjRot("绕Z(滚转)", Vec3(0,0,1), 15.0f);

                ImGui::Separator();
                if (ImGui::MenuItem("删除零件")) {
                    removePart = ws_ctxIdx;
                    ImGui::CloseCurrentPopup(); ws_ctxOpen = false;
                }
                ImGui::EndPopup();
            } else { ws_ctxOpen = false; }
        }

        if (removePart >= 0) {
            assembly.removePart(removePart);
            assembly.recalculate();
            ws_centerViz.lastAssemblyHash = 0;
        }

        // ─ 质心/升力心/推力心 切换按钮 ─
        ImGui::Separator();
        {
            auto togBtn = [&](const char* lbl, bool& show, bool has, ImVec4 onCol) {
                if (!has) ImGui::BeginDisabled();
                if (show) ImGui::PushStyleColor(ImGuiCol_Button, onCol);
                else      ImGui::PushStyleColor(ImGuiCol_Button, {0.2f,0.2f,0.25f,0.7f});
                if (ImGui::Button(lbl, {60, 22})) show = !show;
                ImGui::PopStyleColor();
                if (!has) ImGui::EndDisabled();
            };
            togBtn("● CoM", ws_centerViz.showCoM, ws_centerViz.hasCoM, {0.15f,0.35f,0.85f,0.9f});
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("质心 (Center of Mass)");
            ImGui::SameLine();
            togBtn("▲ CoL", ws_centerViz.showCoL, ws_centerViz.hasCoL, {0.75f,0.65f,0.10f,0.9f});
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("升力中心 (Center of Lift)");
            ImGui::SameLine();
            togBtn("▮ CoT", ws_centerViz.showCoT, ws_centerViz.hasCoT, {0.80f,0.15f,0.15f,0.9f});
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("推力中心 (Center of Thrust)");
        }

        // ─ 性能统计 ─
        ImGui::Separator();
        ImGui::TextColored({0.4f,0.85f,1,1}, "性能统计");
        ImGui::Separator();
        auto sRow = [](const char* lbl, const char* val) {
            ImGui::Text("%-12s", lbl); ImGui::SameLine(130);
            ImGui::TextColored({1,0.9f,0.5f,1}, "%s", val);
        };
        char b[64];
        double totalMass = assembly.total_dry_mass + assembly.total_fuel;
        snprintf(b, 64, "%.1f t",  totalMass / 1000.0);          sRow("起飞质量:", b);
        snprintf(b, 64, "%.1f t",  assembly.total_fuel / 1000.0);sRow("燃料:", b);
        snprintf(b, 64, "%.0f kN", assembly.total_thrust / 1000.0);sRow("总推力:", b);
        snprintf(b, 64, "%.0f s",  assembly.avg_isp);             sRow("ISP:", b);
        snprintf(b, 64, "%.0f m/s",assembly.total_delta_v);       sRow("ΔV:", b);
        snprintf(b, 64, "%.1f m",  assembly.total_height);        sRow("高度:", b);

        // TWR 条目（带颜色提示）
        ImGui::Text("%-12s", "TWR:"); ImGui::SameLine(130);
        ImVec4 twrCol = assembly.twr >= 1.2f ? ImVec4{0.2f,0.9f,0.2f,1} :
                        assembly.twr >= 1.0f ? ImVec4{1,0.8f,0.2f,1}   : ImVec4{1,0.3f,0.3f,1};
        const char* twrNote = assembly.twr < 1.0f ? " ✗ 无法起飞" : assembly.twr >= 1.2f ? " ✓ 良好" : " △ 勉强";
        ImGui::TextColored(twrCol, "%.2f%s", assembly.twr, twrNote);

        // ─ TWR 进度条 ─
        {
            float twrN = std::min(1.f, assembly.twr / 2.0f);
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, twrCol);
            ImGui::ProgressBar(twrN, ImVec2(-1, 8), "");
            ImGui::PopStyleColor();
        }

        // ─ 相机 / 操作提示 ─
        ImGui::Separator();
        ImGui::PushStyleColor(ImGuiCol_Text, {0.45f,0.45f,0.50f,1});
        ImGui::SetWindowFontScale(0.78f);
        if (ws_dragging) {
            ImGui::TextColored({1.0f,0.85f,0.3f,1}, "拖拽中: %s (x%d)  |  左键视口=放置",
                PART_CATALOG[ws_dragDefId].name, ws_symmetry);
            ImGui::TextColored({1.0f,0.6f,0.3f,1}, "ESC 取消  |  QWEASD 旋转  |  点击菜单=取消");
        } else {
            ImGui::TextWrapped("左键点击零件目录 → 拖拽 | 视口左键放置");
            ImGui::TextWrapped("右键拖拽旋转 | Shift+滚轮缩放 | 滚轮上下");
            ImGui::TextWrapped("左键点击火箭零件 = 拆解 | 悬停高亮");
        }
        ImGui::SetWindowFontScale(1.0f);
        ImGui::PopStyleColor();
        ImGui::Separator();

        // ─ 发射按钮 ─
        float bwL = ImGui::GetContentRegionAvail().x;
        bool canLaunch = !assembly.parts.empty() && assembly.twr >= 0.5f;
        if (!canLaunch) {
            ImGui::TextColored({1,0.4f,0.4f,1}, "需至少 1 个零件且 TWR ≥ 0.5");
            ImGui::BeginDisabled();
        }
        ImGui::PushStyleColor(ImGuiCol_Button,        {0.08f,0.52f,0.18f,0.95f});
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, {0.15f,0.70f,0.30f,1.00f});
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  {0.04f,0.38f,0.12f,1.00f});
        if (ImGui::Button(">> 发射! <<", ImVec2(bwL, 52))) {
            GameContext::getInstance().launch_assembly = assembly;
            GameContext::getInstance().skip_builder    = false;
            goToFlight = true;
        }
        ImGui::PopStyleColor(3);
        if (!canLaunch) ImGui::EndDisabled();

        ImGui::Dummy(ImVec2(0, 5));
        if (ImGui::Button("← 返回航天中心", ImVec2(bwL, 30)))
            state = VkGameState::AGENCY;

        ImGui::End();
    }

    // ================================================================
    // FLIGHT HUD
    // ================================================================
    void _drawFlightHUD(FlightScene* fs, int ww, int wh) {
        if (!fs) return;
        auto& trans = fs->world.get<TransformComponent>(fs->rocket_entity);
        auto& vel   = fs->world.get<VelocityComponent>(fs->rocket_entity);
        auto& prop  = fs->world.get<PropulsionComponent>(fs->rocket_entity);
        auto& tele  = fs->world.get<TelemetryComponent>(fs->rocket_entity);
        auto& guid  = fs->world.get<GuidanceComponent>(fs->rocket_entity);
        auto& att   = fs->world.get<AttitudeComponent>(fs->rocket_entity);
        auto& orb   = fs->world.get<OrbitComponent>(fs->rocket_entity);
        auto& ctrl  = fs->world.get<ControlInput>(fs->rocket_entity);
        auto& mnv   = fs->world.get<ManeuverComponent>(fs->rocket_entity);
        const RocketAssembly& assembly = GameContext::getInstance().launch_assembly;

        double speed = sqrt(vel.vx*vel.vx + vel.vy*vel.vy + vel.vz*vel.vz);
        float pitchRad = (float)(PI / 2.0 - att.angle_z);
        float rollRad  = (float)att.angle_roll;
        bool inPanorama = (fs->cam.mode == 2);

        // ── 预计算物理量 ─────────────────────────────────────────────
        int soi_idx = UniverseModel::getInstance().current_soi_index;
        const CelestialBody& soi_body = UniverseModel::getInstance().solar_system[soi_idx];
        double planet_r = soi_body.radius;
        double g_local = G0 * (planet_r*planet_r) / ((planet_r + tele.altitude)*(planet_r + tele.altitude));
        double total_mass = (double)assembly.total_dry_mass + prop.fuel;
        double twr = (total_mass > 0.01 && g_local > 0.001) ? prop.thrust_power / (total_mass * g_local) : 0.0;
        double dv_rem = 0.0;
        if ((double)assembly.total_dry_mass > 0.01 && total_mass > (double)assembly.total_dry_mass && assembly.avg_isp > 0)
            dv_rem = (double)assembly.avg_isp * G0 * log(total_mass / (double)assembly.total_dry_mass);
        double lat = (planet_r > 0) ? asin(std::max(-1.0, std::min(1.0, trans.surf_pz / planet_r))) * 180.0 / PI : 0.0;
        double lon = atan2(trans.surf_py, trans.surf_px) * 180.0 / PI;

        // 轨道根数
        const double mu = 3.986e14;
        double r_mag = sqrt(trans.px*trans.px + trans.py*trans.py + trans.pz*trans.pz);
        double v_sq  = vel.vx*vel.vx + vel.vy*vel.vy + vel.vz*vel.vz;
        double spec_e = v_sq * 0.5 - mu / r_mag;
        double a_val  = (spec_e < -1.0) ? (-mu / (2.0 * spec_e)) : 0.0;
        Vec3 rv(trans.px, trans.py, trans.pz), vv(vel.vx, vel.vy, vel.vz);
        Vec3 hvec = rv.cross(vv);
        double h_mag = hvec.length();
        double e_val = (h_mag > 1.0) ? sqrt(std::max(0.0, 1.0 + 2.0*spec_e*h_mag*h_mag/(mu*mu))) : 0.0;
        double inc_deg = (h_mag > 1.0) ? acos(std::max(-1.0, std::min(1.0, hvec.z / h_mag))) * 180.0 / PI : 0.0;

        // 遥测面板 Y 偏移（星系栏存在时下移）
        float telY = (inPanorama && hud_show_galaxy) ? (hud_exp_planet >= 0 ? 155.f : 100.f) : 10.f;

        // ── 主遥测（左上角）──────────────────────────────────────────
        ImGui::SetNextWindowPos(ImVec2(10, telY), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(290, 0), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.82f);
        ImGui::Begin("遥测", nullptr,
            ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove|
            ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_AlwaysAutoResize);

        const char* stStr = "飞行中";
        switch(guid.status) {
            case PRE_LAUNCH: stStr="待发射"; break; case ASCEND:  stStr="上升";  break;
            case DESCEND:    stStr="下降";   break; case LANDED:  stStr="着陆";  break;
            case CRASHED:    stStr="坠毁";   break; default: break;
        }
        ImGui::TextColored({0.4f,1.0f,0.6f,1}, "[%s]", stStr);
        {
            double met = tele.sim_time;
            int hh=(int)(met/3600), mm=(int)(met/60)%60, ss=(int)met%60;
            ImGui::SameLine(160);
            ImGui::TextColored({0.7f,0.9f,0.7f,1}, "MET %02d:%02d:%02d", hh, mm, ss);
        }
        ImGui::Separator();

        // ── 存档按钮（独占一行，发射前禁用）────────────────────────
        {
            bool canSave = (guid.status != PRE_LAUNCH);
            if (!canSave) ImGui::BeginDisabled();
            ImGui::PushStyleColor(ImGuiCol_Button, {0.08f,0.45f,0.25f,0.90f});
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, {0.15f,0.65f,0.38f,1.0f});
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  {0.05f,0.32f,0.18f,1.0f});
            if (ImGui::Button("💾 保存进度", ImVec2(-1, 22))) {
                SaveSystem::SaveGame(assembly, fs->world, fs->rocket_entity, ctrl);
                fs->last_save_frame = fs->frame;
            }
            ImGui::PopStyleColor(3);
            if (!canSave) ImGui::EndDisabled();

            // 存档后闪烁指示器（覆盖在按钮区域下方）
            int frames_since = fs->frame - fs->last_save_frame;
            if (frames_since < 180 && frames_since >= 0) {
                float alpha = 1.0f - (float)frames_since / 180.0f;
                float pulse = 0.5f + 0.5f * sinf((float)frames_since * 0.3f);
                ImGui::SetWindowFontScale(0.78f);
                ImGui::TextColored({0.3f*alpha, 1.0f*alpha*pulse, 0.4f*alpha, alpha}, "  ✔ 已保存");
                ImGui::SetWindowFontScale(1.0f);
            }
        }

        ImGui::Separator();

        auto row = [](const char* lbl, const std::string& val, ImVec4 c={1,0.9f,0.5f,1}) {
            ImGui::TextUnformatted(lbl); ImGui::SameLine(115);
            ImGui::TextColored(c, "%s", val.c_str());
        };

        { char buf[36]; snprintf(buf,36,"%.4f, %.4f", lat, lon);
          row("LAT/LON:", buf, {0.4f,0.8f,1,1}); }
        row("高度:", _fmtAlt(tele.altitude));
        row("速度:", _fmtSpd(speed));
        { float vv2=(float)vel.vertical_velocity;
          ImVec4 vc = (vv2 < -50.f) ? ImVec4{1,0.3f,0.3f,1} : (vv2 < 0.f ? ImVec4{1,0.8f,0.3f,1} : ImVec4{0.5f,1,0.5f,1});
          char buf[24]; snprintf(buf,24,"%.1f m/s", vv2);
          row("垂直速度:", buf, vc); }
        row("水平速度:", _fmtSpd(vel.horizontal_velocity));
        { char buf[48]; snprintf(buf,48,"%.1f kN  (TWR:%.2f)", prop.thrust_power/1000.0, twr);
          row("推力:", buf, {1,0.6f,0.2f,1}); }
        { char buf[48]; snprintf(buf,48,"%.1f t  (燃料:%.1f t)", total_mass/1000.0, prop.fuel/1000.0);
          row("质量:", buf, {0.85f,0.85f,0.85f,1}); }
        { char buf[32]; snprintf(buf,32,"%.0f m/s", dv_rem);
          row("剩余ΔV:", buf, {0.3f,1,0.5f,1}); }

        // 燃料条
        ImGui::TextUnformatted("燃料:"); ImGui::SameLine(115);
        { double maxF = 1.0;
          if (prop.current_stage<(int)prop.stage_fuels.size() && prop.stage_fuels[prop.current_stage]>0)
              maxF = prop.stage_fuels[prop.current_stage];
          float fp = std::max(0.f, std::min(1.f, (float)(prop.fuel/maxF)));
          ImGui::PushStyleColor(ImGuiCol_PlotHistogram, fp>0.25f ? ImVec4{0.2f,0.85f,0.4f,1} : ImVec4{1,0.3f,0.3f,1});
          char fl[32]; snprintf(fl,32,"%.0f kg",(float)prop.fuel);
          ImGui::ProgressBar(fp, ImVec2(155,14), fl);
          ImGui::PopStyleColor(); }
        // 节流阀
        ImGui::TextUnformatted("节流:"); ImGui::SameLine(115);
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, {1.0f,0.6f,0.1f,1});
        { char tl[16]; snprintf(tl,16,"%.0f%%",ctrl.throttle*100.0);
          ImGui::ProgressBar((float)ctrl.throttle, ImVec2(155,12), tl); }
        ImGui::PopStyleColor();

        ImGui::Separator();
        { char stg[24]; snprintf(stg,24,"第 %d / %d 级",prop.current_stage+1,prop.total_stages);
          row("级别:", stg); }
        { float pd = pitchRad * (180.f / 3.14159f);
          ImVec4 pc = (fabsf(pd)<5.f) ? ImVec4{0.3f,1,0.3f,1} : (fabsf(pd)<20.f ? ImVec4{1,0.9f,0.3f,1} : ImVec4{1,0.4f,0.4f,1});
          char buf[16]; snprintf(buf,16,"%.1f°", pd);
          row("俯仰:", buf, pc); }

        // 轨道拱点
        double ap=-1, pe=-1;
        for (auto& a : orb.predicted_apsides) {
            if (a.is_apoapsis) ap=a.altitude; else pe=a.altitude;
        }
        if (ap>0 || pe>0) {
            ImGui::Separator();
            ImGui::TextColored({0.6f,0.8f,1,1}, "轨道");
            if (pe>=0) row("近地点:", _fmtAlt(pe));
            if (ap>=0) row("远地点:", _fmtAlt(ap));
            if (a_val > 0) {
                char buf[48]; snprintf(buf,48,"%.1f km  e=%.4f", a_val/1000.0, e_val);
                row("半长轴:", buf, {0.85f,0.85f,1,1});
                snprintf(buf,32,"%.2f°", inc_deg);
                row("倾角:", buf, {0.85f,0.85f,1,1});
            }
        }
        if (!guid.mission_msg.empty()) {
            ImGui::Separator();
            ImGui::SetWindowFontScale(0.85f);
            ImGui::TextWrapped("%s", guid.mission_msg.c_str());
            ImGui::SetWindowFontScale(1.0f);
        }
        ImGui::End();

        // ── 底部中央：导航球 + 控制面板 ─────────────────────────────
        const float navR  = 78.f;
        const float ctrlW = 162.f;
        const float panW  = navR*2.f + 16.f + ctrlW;
        const float panH  = navR*2.f + 54.f;  // 上留20px给框架指针，下留16px给文字
        float panX = (float)ww * 0.5f - panW * 0.5f;
        float panY = (float)wh - panH - 10.f;

        ImGui::SetNextWindowPos(ImVec2(panX, panY), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(panW, panH), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.85f);
        ImGui::Begin("##NavCtrl", nullptr,
            ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
            ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar);

        ImVec2 wp = ImGui::GetWindowPos();
        ImDrawList* dl = ImGui::GetWindowDrawList();
        ImVec2 navCtr = { wp.x + navR + 8.f, wp.y + navR + 24.f };  // 24px 顶边给框架指针三角

        // ── 使用 OpenGL 同款四元数球面投影 ───────────────────────────
        const Quat& rq = fs->ctx.rocketQuat;
        const Vec3& lr = fs->ctx.localRight;
        const Vec3& lu = fs->ctx.rocketUp;
        const Vec3& ln = fs->ctx.localNorth;
        _drawNavball(dl, navCtr, navR, rq, lr, lu, ln);

        // ── 轨道矢量标记（与导航球使用相同的四元数投影）──────────────
        if (speed > 1.0) {
            Quat rqInv = rq.conjugate();
            // 世界方向向量 → 导航球2D坐标（与 _drawNavball 内部 projectWorld 逻辑一致）
            auto projectNav = [&](const Vec3& wv, float& nx, float& ny) -> bool {
                Vec3 body = rqInv.rotate(wv);
                if (body.y < 0.05) return false;  // 背面剔除
                nx = navCtr.x + (float)(body.x * navR);
                ny = navCtr.y - (float)(body.z * navR);  // ImGui Y-down
                // 钳制在圆内
                float dx = nx - navCtr.x, dy = ny - navCtr.y;
                float d2 = dx*dx + dy*dy;
                if (d2 > (navR-3.f)*(navR-3.f)) {
                    float s = (navR-3.f) / sqrtf(d2);
                    nx = navCtr.x + dx * s;
                    ny = navCtr.y + dy * s;
                }
                return true;
            };
            auto drawMarker = [&](float x, float y, ImU32 col, bool filled, float mr=5.f) {
                if (filled) {
                    dl->AddCircleFilled({x,y}, mr, col, 12);
                    dl->AddCircle({x,y}, mr+1.f, IM_COL32(0,0,0,150), 12, 1.f);
                } else {
                    dl->AddLine({x-mr,y},{x+mr,y}, col, 1.5f);
                    dl->AddLine({x,y-mr},{x,y+mr}, col, 1.5f);
                    dl->AddCircle({x,y}, mr+1.5f, col, 12, 1.5f);
                }
            };
            const Vec3& vPro = fs->ctx.prograde_rel;
            const Vec3& vNrm = fs->ctx.orbit_normal_rel;
            const Vec3& vRad = fs->ctx.radial_rel;
            Vec3 nPro(-vPro.x,-vPro.y,-vPro.z);
            Vec3 nNrm(-vNrm.x,-vNrm.y,-vNrm.z);
            Vec3 nRad(-vRad.x,-vRad.y,-vRad.z);
            float mx2, my2;
            if (projectNav(nRad, mx2,my2))  drawMarker(mx2,my2, IM_COL32(100,220,255,200), false, 4.5f);
            if (projectNav(vRad, mx2,my2))  drawMarker(mx2,my2, IM_COL32(100,220,255,200), true,  4.5f);
            if (projectNav(nNrm, mx2,my2))  drawMarker(mx2,my2, IM_COL32(210,100,255,200), false, 4.5f);
            if (projectNav(vNrm, mx2,my2))  drawMarker(mx2,my2, IM_COL32(210,100,255,200), true,  4.5f);
            if (projectNav(nPro, mx2,my2))  drawMarker(mx2,my2, IM_COL32(255,210,0,220),   false, 5.5f);
            if (projectNav(vPro, mx2,my2))  drawMarker(mx2,my2, IM_COL32(255,210,0,220),   true,  5.5f);
        }

        // 俯仰/滚转文本
        {
            float pitchDeg = pitchRad * (180.f / 3.14159f);
            float rollDeg  = rollRad  * (180.f / 3.14159f);
            ImGui::SetCursorPos({8.f, navR*2.f + 24.f});
            ImGui::SetWindowFontScale(0.75f);
            ImGui::TextColored({0.8f,0.9f,1,1}, "P:%.0f° R:%.0f°", pitchDeg, rollDeg);
            ImGui::SetWindowFontScale(1.0f);
        }

        // 右侧控制列
        float colX = navR*2.f + 16.f;
        ImGui::SetCursorPos({colX, 4.f});
        { bool sas = guid.sas_active;
          if (sas) ImGui::PushStyleColor(ImGuiCol_Button, {0.1f,0.7f,0.2f,0.9f});
          else     ImGui::PushStyleColor(ImGuiCol_Button, {0.4f,0.4f,0.4f,0.7f});
          if (ImGui::Button("SAS", {50.f,22.f})) guid.sas_active = !guid.sas_active;
          ImGui::PopStyleColor(); }
        ImGui::SameLine();
        { bool rcs = guid.rcs_active;
          if (rcs) ImGui::PushStyleColor(ImGuiCol_Button, {0.1f,0.5f,0.9f,0.9f});
          else     ImGui::PushStyleColor(ImGuiCol_Button, {0.4f,0.4f,0.4f,0.7f});
          if (ImGui::Button("RCS", {50.f,22.f})) guid.rcs_active = !guid.rcs_active;
          ImGui::PopStyleColor(); }
        ImGui::SameLine();
        { bool am = guid.auto_mode;
          if (am) ImGui::PushStyleColor(ImGuiCol_Button, {0.7f,0.4f,0.0f,0.9f});
          else    ImGui::PushStyleColor(ImGuiCol_Button, {0.4f,0.4f,0.4f,0.7f});
          if (ImGui::Button(am?"自动":"手动", {50.f,22.f})) guid.auto_mode = !guid.auto_mode;
          ImGui::PopStyleColor(); }

        // SAS 模式 4×2 网格
        ImGui::SetCursorPos({colX, 32.f});
        ImGui::SetWindowFontScale(0.72f);
        static const char* sasLabels[] = {"稳","顺行","逆行","法+","法-","径内","径外","变轨"};
        static const int   sasOrder[]  = {0,1,2,7, 3,4,5,6};
        for (int i = 0; i < 8; i++) {
            int mode = sasOrder[i];
            bool active = guid.sas_active && ((int)guid.sas_mode == mode);
            if (active) ImGui::PushStyleColor(ImGuiCol_Button, {0.2f,0.6f,0.9f,0.9f});
            else        ImGui::PushStyleColor(ImGuiCol_Button, {0.25f,0.25f,0.3f,0.8f});
            char btnId[16]; snprintf(btnId,16,"%s##s%d", sasLabels[mode], mode);
            if (ImGui::Button(btnId, {36.f,20.f})) {
                guid.sas_mode   = (SASMode)mode;
                guid.sas_active = true;
            }
            ImGui::PopStyleColor();
            if ((i%4) != 3) ImGui::SameLine(0, 3.f);
        }
        ImGui::SetWindowFontScale(1.0f);
        ImGui::End();

        // ── 时间加速提示（顶部中央）──────────────────────────────────
        if (fs->sim_ctrl.time_warp > 1) {
            ImGui::SetNextWindowPos(ImVec2((float)ww*0.5f-90, 10), ImGuiCond_Always);
            ImGui::SetNextWindowBgAlpha(0.75f);
            ImGui::Begin("##TW", nullptr,
                ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                ImGuiWindowFlags_NoMove|ImGuiWindowFlags_AlwaysAutoResize);
            ImGui::TextColored({1,0.8f,0.2f,1}, ">> x%d 时间加速 <<", fs->sim_ctrl.time_warp);
            ImGui::End();
        }

        // ── 自由相机速度显示 ─────────────────────────────────────
        if (fs->cam.mode == 3) {
            ImGui::SetNextWindowPos(ImVec2((float)ww*0.5f-100, 45.f), ImGuiCond_Always);
            ImGui::SetNextWindowBgAlpha(0.75f);
            ImGui::Begin("##FreeCam", nullptr,
                ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                ImGuiWindowFlags_NoMove|ImGuiWindowFlags_AlwaysAutoResize);
            float spd = fs->cam.free_move_speed;
            if (spd > 1000000.f)
                ImGui::TextColored({0.4f,1.0f,1.0f,1}, "FREE CAM: %.2f km/s", spd / 1000.f);
            else
                ImGui::TextColored({0.4f,1.0f,1.0f,1}, "FREE CAM: %.1f m/s", spd);
            ImGui::End();
        }

        // ── Panorama（轨道俯瞰）模式 ─────────────────────────────────
        if (inPanorama) {

            // ── 星系信息栏 ─────────────────────────────────────────────
            if (hud_show_galaxy) {
                auto& ss = UniverseModel::getInstance().solar_system;
                const float barH = 90.f;
                ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
                ImGui::SetNextWindowSize(ImVec2((float)ww, barH), ImGuiCond_Always);
                ImGui::SetNextWindowBgAlpha(0.88f);
                ImGui::Begin("##GalaxyBar", nullptr,
                    ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                    ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar);
                ImDrawList* gdl = ImGui::GetWindowDrawList();
                const float iconR2 = 26.f, spacing2 = 70.f, startX = 20.f, cy_bar = barH * 0.5f;
                int top_idx = 0;
                for (int i = 0; i < (int)ss.size(); i++) {
                    if (ss[i].parent_index != -1) continue;
                    float sc_x = startX + top_idx * spacing2;
                    ImGui::SetCursorScreenPos({sc_x, cy_bar - iconR2});
                    bool clicked = ImGui::InvisibleButton(("##pb"+std::to_string(i)).c_str(), {iconR2*2.f, iconR2*2.f});
                    if (clicked) {
                        hud_sel_body  = (hud_sel_body  == i) ? -1 : i;
                        hud_exp_planet= (hud_exp_planet == i) ? -1 : i;
                    }
                    float cx_s = sc_x + iconR2, cy_s = cy_bar;
                    ImU32 col = IM_COL32((int)(ss[i].r*255),(int)(ss[i].g*255),(int)(ss[i].b*255),255);
                    if (hud_sel_body==i||hud_exp_planet==i)
                        gdl->AddCircle({cx_s,cy_s}, iconR2+3.f, IM_COL32(255,255,100,220), 32, 2.f);
                    gdl->AddCircleFilled({cx_s,cy_s}, iconR2, col, 32);
                    bool hasMoon = false;
                    for (int m=0; m<(int)ss.size(); m++) if(ss[m].parent_index==i){hasMoon=true;break;}
                    if (hasMoon) {
                        ImVec2 tp={cx_s-6.f, cy_s+iconR2+2.f};
                        gdl->AddTriangleFilled(tp,{tp.x+12.f,tp.y},{tp.x+6.f,tp.y+8.f},
                            (hud_exp_planet==i) ? IM_COL32(255,220,60,220) : IM_COL32(160,160,160,160));
                    }
                    ImGui::SetCursorScreenPos({cx_s-iconR2, cy_s+iconR2+3.f});
                    ImGui::SetWindowFontScale(0.65f);
                    ImGui::TextUnformatted(ss[i].name.c_str());
                    ImGui::SetWindowFontScale(1.0f);
                    top_idx++;
                }
                ImGui::End();

                // 卫星子栏
                if (hud_exp_planet >= 0) {
                    const float moonBarH = 52.f;
                    ImGui::SetNextWindowPos(ImVec2(0, barH), ImGuiCond_Always);
                    ImGui::SetNextWindowSize(ImVec2((float)ww, moonBarH), ImGuiCond_Always);
                    ImGui::SetNextWindowBgAlpha(0.78f);
                    ImGui::Begin("##MoonBar", nullptr,
                        ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                        ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar);
                    ImDrawList* mdl = ImGui::GetWindowDrawList();
                    const float mr2 = 16.f, moon_cy = moonBarH*0.5f;
                    float mx_off = 20.f;
                    for (int i = 0; i < (int)ss.size(); i++) {
                        if (ss[i].parent_index != hud_exp_planet) continue;
                        ImGui::SetCursorScreenPos({mx_off, moon_cy - mr2});
                        bool clicked2 = ImGui::InvisibleButton(("##mb"+std::to_string(i)).c_str(), {mr2*2.f, mr2*2.f});
                        if (clicked2) hud_sel_body = (hud_sel_body==i) ? -1 : i;
                        float cx_m = mx_off + mr2;
                        ImU32 mc = IM_COL32((int)(ss[i].r*255),(int)(ss[i].g*255),(int)(ss[i].b*255),255);
                        if (hud_sel_body==i) mdl->AddCircle({cx_m,moon_cy}, mr2+2.f, IM_COL32(255,255,100,220), 24, 2.f);
                        mdl->AddCircleFilled({cx_m,moon_cy}, mr2, mc, 24);
                        ImGui::SetCursorScreenPos({cx_m-mr2, moon_cy+mr2+2.f});
                        ImGui::SetWindowFontScale(0.62f);
                        ImGui::TextUnformatted(ss[i].name.c_str());
                        ImGui::SetWindowFontScale(1.0f);
                        mx_off += mr2*2.f + 12.f;
                    }
                    ImGui::End();
                }

                // 天体详情面板
                if (hud_sel_body >= 0 && hud_sel_body < (int)ss.size()) {
                    const CelestialBody& bd = ss[hud_sel_body];
                    float detY = (hud_exp_planet >= 0) ? (barH+52.f+5.f) : (barH+5.f);
                    ImGui::SetNextWindowPos(ImVec2(10, detY), ImGuiCond_Always);
                    ImGui::SetNextWindowSize(ImVec2(285, 0), ImGuiCond_Always);
                    ImGui::SetNextWindowBgAlpha(0.87f);
                    ImGui::Begin("##BodyDet", nullptr,
                        ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                        ImGuiWindowFlags_NoMove|ImGuiWindowFlags_AlwaysAutoResize);
                    ImGui::TextColored({0.8f,0.9f,1,1}, "%s", bd.name.c_str());
                    ImGui::Separator();
                    static const char* btype[] = {"恒星","类地","气态巨星","卫星","环状巨星"};
                    ImGui::Text("类型: %s", btype[std::min((int)bd.type,4)]);
                    ImGui::Text("半径: %.0f km", bd.radius/1000.0);
                    double sg = G_const * bd.mass / (bd.radius*bd.radius);
                    ImGui::Text("表面重力: %.2f m/s²", sg);
                    ImGui::Text("轨道倾角: %.2f°", bd.inc_base*180.0/PI);
                    ImGui::Text("离心率: %.4f", bd.ecc_base);
                    if (bd.rotation_period > 0) {
                        int rph=(int)(bd.rotation_period/3600), rpm=(int)(bd.rotation_period/60)%60;
                        ImGui::Text("自转周期: %dh %02dm", rph, rpm);
                    }
                    ImGui::Text("表面气压: %.0f hPa", bd.surface_pressure);
                    ImGui::Text("平均温度: %.0f K", bd.average_temp);
                    int mc_n=0; for(int m=0;m<(int)ss.size();m++) if(ss[m].parent_index==hud_sel_body) mc_n++;
                    if (mc_n>0) ImGui::Text("卫星数: %d", mc_n);
                    ImGui::End();
                }
            }

            // ── 轨道信息（右侧面板）──────────────────────────────────
            ImGui::SetNextWindowPos(ImVec2((float)ww - 285.f, 10.f), ImGuiCond_Always);
            ImGui::SetNextWindowSize(ImVec2(275.f, 0), ImGuiCond_Always);
            ImGui::SetNextWindowBgAlpha(0.82f);
            ImGui::Begin("轨道信息", nullptr,
                ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove|
                ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_AlwaysAutoResize);
            auto rowR = [](const char* lbl, const std::string& val) {
                ImGui::TextUnformatted(lbl); ImGui::SameLine(115);
                ImGui::TextColored({1,0.9f,0.5f,1}, "%s", val.c_str());
            };

            double apAlt=-1, peAlt=-1, tAP=-1, tPE=-1;
            for (auto& a : orb.predicted_apsides) {
                double dt = a.sim_time - tele.sim_time;
                if (a.is_apoapsis) { apAlt=a.altitude; tAP=dt; }
                else               { peAlt=a.altitude; tPE=dt; }
            }
            ImGui::TextColored({0.6f,0.8f,1,1}, "轨道拱点");
            ImGui::Separator();
            if (apAlt>=0) {
                rowR("远地点 (AP):", _fmtAlt(apAlt));
                if (tAP>=0) { int tmm=(int)(tAP/60), tss=(int)tAP%60;
                    char buf[24]; snprintf(buf,24,"%dm %02ds",tmm,tss); rowR("到 AP:", buf); }
            }
            if (peAlt>=0) {
                rowR("近地点 (PE):", _fmtAlt(peAlt));
                if (tPE>=0) { int tmm=(int)(tPE/60), tss=(int)tPE%60;
                    char buf[24]; snprintf(buf,24,"%dm %02ds",tmm,tss); rowR("到 PE:", buf); }
            }
            if (apAlt>=0 && peAlt>=0) {
                double a2 = planet_r + (apAlt+peAlt)*0.5;
                double T  = 6.28318530718 * sqrt(a2*a2*a2 / mu);
                int th=(int)(T/3600), tm=(int)(T/60)%60, ts=(int)T%60;
                char tbuf[32]; snprintf(tbuf,32,"%dh %02dm %02ds",th,tm,ts);
                ImGui::Separator();
                rowR("轨道周期:", tbuf);
            }
            ImGui::Separator();
            rowR("总速度:", _fmtSpd(speed));
            rowR("高度:",   _fmtAlt(tele.altitude));
            if (a_val > 0) {
                ImGui::Separator();
                ImGui::TextColored({0.6f,0.8f,1,1}, "轨道根数");
                { char buf[32]; snprintf(buf,32,"%.1f km", a_val/1000.0); rowR("半长轴:", buf); }
                { char buf[16]; snprintf(buf,16,"%.4f", e_val);           rowR("离心率:", buf); }
                { char buf[16]; snprintf(buf,16,"%.2f°", inc_deg);        rowR("倾角:", buf); }
            }
            ImGui::Separator();
            // 防止在"高级轨道"面板区域内点击时误关面板
            static float s_advOrbitLeft = 0, s_advOrbitTop = 0, s_advOrbitRight = 0, s_advOrbitBottom = 0;
            bool blockAdvToggle = hud_adv_orbit &&
                ImGui::GetMousePos().x >= s_advOrbitLeft && ImGui::GetMousePos().x <= s_advOrbitRight &&
                ImGui::GetMousePos().y >= s_advOrbitTop && ImGui::GetMousePos().y <= s_advOrbitBottom;
            if (ImGui::Button(hud_adv_orbit  ?"[ 高级轨道 ]":"高级轨道",  {120.f,20.f}) && !blockAdvToggle) hud_adv_orbit   = !hud_adv_orbit;
            ImGui::SameLine();
            if (ImGui::Button(hud_flight_asst?"[ 飞行辅助 ]":"飞行辅助", {120.f,20.f})) hud_flight_asst = !hud_flight_asst;
            ImGui::Separator();
            if (ImGui::Button(hud_show_galaxy?"[ 星系信息 ]":"星系信息",  {120.f,20.f})) hud_show_galaxy = !hud_show_galaxy;
            ImGui::Separator();
            // ── 气候视图 ──────────────────────────────────────────
            static const char* climateNames[] = {"普通", "温度", "降水", "气压", "水文"};
            Renderer3D* r3d_hud = GameContext::getInstance().renderer3d;
            int curClimate = r3d_hud ? r3d_hud->climateViewMode : 0;
            if (ImGui::Button("CLIMATE VIEW", {120.f,20.f})) {
                if (r3d_hud) r3d_hud->setClimateViewMode((curClimate + 1) % 5);
            }
            ImGui::SameLine();
            ImGui::TextColored({0.7f,0.9f,1.0f,1}, "%s", climateNames[curClimate % 5]);
            ImGui::SetWindowFontScale(0.80f);
            ImGui::TextColored({0.6f,0.6f,0.6f,1}, "右键拖 旋转 | 滚轮 缩放\n[O] 切换视角");
            ImGui::SetWindowFontScale(1.0f);
            ImGui::End();

            // ── 高级轨道面板 ─────────────────────────────────────────
            if (hud_adv_orbit) {
                auto& hudRef = fs->hudManager.hud;
                auto& ss2 = UniverseModel::getInstance().solar_system;
                int nBodies = (int)ss2.size();

                ImGui::SetNextWindowPos(ImVec2((float)ww - 565.f, 10.f), ImGuiCond_Always);
                ImGui::SetNextWindowSize(ImVec2(290.f, 0), ImGuiCond_Always);
                ImGui::SetNextWindowBgAlpha(0.82f);
                ImGui::Begin("高级轨道##adv", nullptr,
                    ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove|
                    ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_AlwaysAutoResize);

                // ── 轨道模式切换 ──────────────────────────────────────
                ImGui::TextColored({0.8f,0.8f,0.4f,1}, "轨道模式");
                bool advEnabled = hudRef.adv_orbit_enabled;
                if (ImGui::Checkbox("SYM-LMM4 (数值预测)", &advEnabled))
                    hudRef.adv_orbit_enabled = advEnabled;
                ImGui::SameLine(); ImGui::SetWindowFontScale(0.72f);
                ImGui::TextColored({0.6f,0.6f,0.6f,1}, advEnabled ? "(高精度)" : "(Kepler)");
                ImGui::SetWindowFontScale(1.0f);
                ImGui::Separator();

                // ── 参考系选择 ──────────────────────────────────────
                static const char* frameNames[] = {"惯性系", "共转系", "地表系"};
                ImGui::TextColored({0.6f,0.8f,1,1}, "参考系");
                int curFrame = hudRef.adv_orbit_ref_mode;
                if (ImGui::Combo("##frame", &curFrame, frameNames, 3))
                    hudRef.adv_orbit_ref_mode = curFrame;
                ImGui::Separator();

                // ── 主参考天体 ───────────────────────────────────────
                ImGui::TextColored({0.6f,0.8f,1,1}, "主参考天体");
                if (nBodies > 0) {
                    int curBody = hudRef.adv_orbit_ref_body;
                    if (curBody < 0) curBody = 0;
                    if (curBody >= nBodies) curBody = nBodies - 1;
                    if (ImGui::Button("<##priL", {24.f,20.f}) && curBody > 0)
                        hudRef.adv_orbit_ref_body = curBody - 1;
                    ImGui::SameLine();
                    ImGui::Text("%s", ss2[curBody].name.c_str());
                    ImGui::SameLine();
                    if (ImGui::Button(">##priR", {24.f,20.f}) && curBody < nBodies - 1)
                        hudRef.adv_orbit_ref_body = curBody + 1;
                }
                ImGui::Separator();

                // ── 副参考天体（仅共转系）───────────────────────────
                if (hudRef.adv_orbit_ref_mode == 1) {
                    ImGui::TextColored({0.5f,0.9f,0.5f,1}, "副参考天体");
                    if (nBodies > 0) {
                        int curSec = hudRef.adv_orbit_secondary_ref_body;
                        if (curSec < 0) curSec = 0;
                        if (curSec >= nBodies) curSec = nBodies - 1;
                        if (ImGui::Button("<##secL", {24.f,20.f}) && curSec > 0)
                            hudRef.adv_orbit_secondary_ref_body = curSec - 1;
                        ImGui::SameLine();
                        ImGui::Text("%s", ss2[curSec].name.c_str());
                        ImGui::SameLine();
                        if (ImGui::Button(">##secR", {24.f,20.f}) && curSec < nBodies - 1)
                            hudRef.adv_orbit_secondary_ref_body = curSec + 1;
                    }
                    ImGui::Separator();
                }

                // ── 预测天数滑块 ────────────────────────────────────
                ImGui::TextColored({0.6f,0.8f,1,1}, "预测时长");
                float predDays = hudRef.adv_orbit_pred_days;
                // 对数滑块: 1 ~ 3650 天
                float predLog = logf(predDays);
                float predLogMin = 0.f, predLogMax = logf(3650.f);
                float predT = (predLog - predLogMin) / (predLogMax - predLogMin);
                if (ImGui::SliderFloat("##predT", &predT, 0.f, 1.f, ""))
                    hudRef.adv_orbit_pred_days = expf(predLogMin + predT * (predLogMax - predLogMin));
                ImGui::SameLine();
                if (predDays < 1.5f) ImGui::Text("%.0f 天", predDays);
                else if (predDays < 365.f) ImGui::Text("%.0f 天", predDays);
                else ImGui::Text("%.1f 年", predDays / 365.25f);
                ImGui::Separator();

                // ── 迭代次数 ────────────────────────────────────────
                ImGui::TextColored({0.6f,0.8f,1,1}, "迭代次数");
                static const int iterOpts[] = {500, 1000, 2000, 4000, 8000, 16000, 32000};
                static const char* iterNames[] = {"500","1000","2000","4000","8000","16000","32000"};
                int curIter = hudRef.adv_orbit_iters;
                int curIterIdx = 3; // default 4000
                for (int j=0; j<7; j++) if (curIter == iterOpts[j]) curIterIdx = j;
                if (ImGui::Combo("##iters", &curIterIdx, iterNames, 7))
                    hudRef.adv_orbit_iters = iterOpts[curIterIdx];
                ImGui::Separator();

                // ── 创建变轨节点 ────────────────────────────────────
                ImGui::TextColored({0.4f,0.8f,0.4f,1}, "变轨操作");
                if (ImGui::Button("创建变轨节点", {260.f, 22.f})) {
                    ManeuverNode node;
                    node.sim_time = tele.sim_time + 600.0;
                    node.delta_v = Vec3(0, 0, 0);
                    node.active = true;
                    node.ref_body = UniverseModel::getInstance().current_soi_index;
                    mnv.maneuvers.clear();
                    mnv.maneuvers.push_back(node);
                    mnv.selected_maneuver_index = 0;
                }
                ImGui::Separator();

                // ── 快速跃迁到节点 ──────────────────────────────────
                bool hasMnv = !mnv.maneuvers.empty();
                if (!hasMnv) ImGui::BeginDisabled();
                bool warpOn = hudRef.adv_warp_to_node;
                if (warpOn) {
                    ImGui::PushStyleColor(ImGuiCol_Button, {1.0f,0.5f,0.1f,0.8f});
                    if (ImGui::Button("[ 跃迁中... ]", {260.f, 24.f}))
                        hudRef.adv_warp_to_node = false;
                    ImGui::PopStyleColor();
                } else {
                    if (ImGui::Button("跃迁到变轨节点", {260.f, 24.f}))
                        hudRef.adv_warp_to_node = true;
                }
                if (!hasMnv) ImGui::EndDisabled();
                ImGui::Separator();

                // ── 变轨节点列表 + 编辑 ──────────────────────────────
                ImGui::TextColored({0.6f,0.8f,1,1}, "变轨节点");
                ImGui::Separator();
                auto& mnvs = mnv.maneuvers;
                if (mnvs.empty()) {
                    ImGui::TextColored({0.6f,0.6f,0.6f,1}, "无变轨节点");
                } else {
                    int selIdx = mnv.selected_maneuver_index;
                    for (int i=0; i<(int)mnvs.size(); i++) {
                        double dt2 = mnvs[i].sim_time - tele.sim_time;
                        bool sel2 = (selIdx == i);
                        if (sel2) ImGui::PushStyleColor(ImGuiCol_Text, {1,0.9f,0.3f,1});
                        double dv_mag = mnvs[i].delta_v.length();
                        ImGui::Text("节点 %d  T%+.0fs  ΔV=%.0f m/s", i+1, dt2, dv_mag);
                        if (sel2) ImGui::PopStyleColor();
                        ImGui::SameLine();
                        if (ImGui::SmallButton(("选##mn"+std::to_string(i)).c_str()))
                            mnv.selected_maneuver_index = i;
                    }

                    // ── 选中节点的编辑面板 ──────────────────────────
                    if (selIdx >= 0 && selIdx < (int)mnvs.size()) {
                        auto& node = mnvs[selIdx];
                        ImGui::Separator();
                        ImGui::TextColored({1.0f,0.9f,0.3f,1}, "编辑节点 %d", selIdx+1);
                        ImGui::SameLine();
                        // 关闭按钮 — 点击取消选中即可关闭编辑面板
                        if (ImGui::SmallButton("X##closeEdit"))
                            mnv.selected_maneuver_index = -1;

                        // 参考天体
                        if (node.ref_body >= 0 && node.ref_body < (int)ss2.size())
                            ImGui::TextColored({0.5f,0.7f,0.9f,1}, "参考: %s", ss2[node.ref_body].name.c_str());

                        // 时间偏移
                        double timeOffset = node.sim_time - tele.sim_time;
                        ImGui::TextColored({0.6f,1.0f,0.6f,1}, "时间偏移");
                        float tSlide = (float)timeOffset;
                        float tMin = -300.f, tMax = 3600.f;
                        if (ImGui::SliderFloat("##mnvTime", &tSlide, tMin, tMax, "T%+.0f s"))
                            node.sim_time = tele.sim_time + (double)tSlide;
                        // 微调按钮
                        ImGui::SameLine();
                        if (ImGui::SmallButton("+10s##t")) node.sim_time += 10.0;
                        ImGui::SameLine();
                        if (ImGui::SmallButton("-10s##t")) node.sim_time -= 10.0;
                        ImGui::SameLine();
                        if (ImGui::SmallButton("+60s##t")) node.sim_time += 60.0;
                        ImGui::SameLine();
                        if (ImGui::SmallButton("-60s##t")) node.sim_time -= 60.0;

                        // ΔV 分量滑块 (Prograde / Normal / Radial)
                        float dvPro = (float)node.delta_v.x;
                        float dvNrm = (float)node.delta_v.y;
                        float dvRad = (float)node.delta_v.z;
                        float dvRange = 500.f;
                        float dvStep = 1.f;

                        ImGui::TextColored({1.0f,0.9f,0.1f,1}, "顺向 ΔV (Prograde)");
                        if (ImGui::SliderFloat("##dvPro", &dvPro, -dvRange, dvRange, "%.0f m/s"))
                            node.delta_v.x = (double)dvPro;
                        ImGui::SameLine(); ImGui::SetWindowFontScale(0.7f);
                        if (ImGui::SmallButton("0##p0")) node.delta_v.x = 0.0;
                        ImGui::SetWindowFontScale(1.0f);

                        ImGui::TextColored({1.0f,0.3f,1.0f,1}, "法向 ΔV (Normal)");
                        if (ImGui::SliderFloat("##dvNrm", &dvNrm, -dvRange, dvRange, "%.0f m/s"))
                            node.delta_v.y = (double)dvNrm;
                        ImGui::SameLine(); ImGui::SetWindowFontScale(0.7f);
                        if (ImGui::SmallButton("0##n0")) node.delta_v.y = 0.0;
                        ImGui::SetWindowFontScale(1.0f);

                        ImGui::TextColored({0.2f,0.8f,1.0f,1}, "径向 ΔV (Radial)");
                        if (ImGui::SliderFloat("##dvRad", &dvRad, -dvRange, dvRange, "%.0f m/s"))
                            node.delta_v.z = (double)dvRad;
                        ImGui::SameLine(); ImGui::SetWindowFontScale(0.7f);
                        if (ImGui::SmallButton("0##r0")) node.delta_v.z = 0.0;
                        ImGui::SetWindowFontScale(1.0f);

                        // 总 ΔV 显示
                        double totalDv = node.delta_v.length();
                        ImGui::TextColored({0.8f,1.0f,0.5f,1}, "总 ΔV = %.0f m/s", totalDv);

                        // 删除节点
                        ImGui::Separator();
                        if (ImGui::Button("删除此节点", {260.f, 22.f})) {
                            mnv.maneuvers.erase(mnv.maneuvers.begin() + selIdx);
                            if (mnv.selected_maneuver_index >= (int)mnv.maneuvers.size())
                                mnv.selected_maneuver_index = (int)mnv.maneuvers.size() - 1;
                            if (mnv.maneuvers.empty()) hudRef.adv_warp_to_node = false;
                        }
                    }
                }
                ImGui::Separator();

                // ── 当前轨道根数 ────────────────────────────────────
                ImGui::TextColored({0.5f,0.8f,0.5f,1}, "当前轨道根数");
                if (a_val > 0) {
                    ImGui::Text("a = %.1f km  e = %.4f  i = %.2f°", a_val/1000.0, e_val, inc_deg);
                } else {
                    ImGui::TextColored({0.6f,0.6f,0.6f,1}, "轨道数据不足（速度过低）");
                }
                ImGui::End();
                // 记住面板位置，用于防止误关 + 动态定位飞行辅助面板
                { ImVec2 p = ImGui::GetWindowPos(); ImVec2 s = ImGui::GetWindowSize();
                  s_advOrbitLeft = p.x; s_advOrbitTop = p.y;
                  s_advOrbitRight = p.x + s.x; s_advOrbitBottom = p.y + s.y; }

                // ── WARP TO NODE 实际跃迁逻辑 ────────────────────
                auto& hudRefWarp = fs->hudManager.hud;
                if (hudRefWarp.adv_warp_to_node && !mnv.maneuvers.empty()) {
                    double target_t = mnv.maneuvers[0].sim_time;
                    double time_to_start = target_t - tele.sim_time;
                    if (time_to_start <= 60.0) {
                        fs->sim_ctrl.time_warp = 1;
                        hudRefWarp.adv_warp_to_node = false;
                    } else if (time_to_start > 86400.0 * 10) {
                        fs->sim_ctrl.time_warp = 10000000;
                    } else if (time_to_start > 86400.0) {
                        fs->sim_ctrl.time_warp = 1000000;
                    } else if (time_to_start > 3600.0 * 4) {
                        fs->sim_ctrl.time_warp = 100000;
                    } else if (time_to_start > 3600.0) {
                        fs->sim_ctrl.time_warp = 10000;
                    } else if (time_to_start > 600.0) {
                        fs->sim_ctrl.time_warp = 1000;
                    } else if (time_to_start > 120.0) {
                        fs->sim_ctrl.time_warp = 100;
                    } else {
                        fs->sim_ctrl.time_warp = 10;
                    }
                }
            }

            // ── 飞行辅助面板 ─────────────────────────────────────────
            if (hud_flight_asst) {
                auto& hudRef2 = fs->hudManager.hud;
                float asst_y = hud_adv_orbit ? (s_advOrbitBottom + 5.f) : 10.f;
                ImGui::SetNextWindowPos(ImVec2((float)ww - 565.f, asst_y), ImGuiCond_Always);
                ImGui::SetNextWindowSize(ImVec2(268.f, 0), ImGuiCond_Always);
                ImGui::SetNextWindowBgAlpha(0.82f);
                ImGui::Begin("飞行辅助##asst", nullptr,
                    ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove|
                    ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_AlwaysAutoResize);

                // ── 自动操作 ──────────────────────────────────────
                ImGui::TextColored({1.0f,0.7f,0.3f,1}, "自动操作");
                ImGui::Separator();

                // AUTO EXECUTE MNV
                bool autoExec = hudRef2.auto_exec_mnv;
                if (ImGui::Checkbox("自动执行变轨", &autoExec))
                    hudRef2.auto_exec_mnv = autoExec;

                // AUTO ORBIT
                bool is_ascent = (guid.status == ASCEND);
                bool temp_ascent = is_ascent;
                if (ImGui::Checkbox("自动入轨", &temp_ascent)) {
                    if (temp_ascent) {
                        guid.status = ASCEND;
                        guid.mission_phase = 0;
                        guid.auto_mode = true;
                        guid.mission_msg = "AUTOPILOT: INITIATING ASCENT...";
                    }
                }

                // AUTO LANDING
                bool is_descent = (guid.status == DESCEND);
                bool temp_descent = is_descent;
                if (ImGui::Checkbox("自动着陆", &temp_descent)) {
                    if (temp_descent) {
                        guid.status = DESCEND;
                        guid.auto_mode = true;
                        guid.mission_msg = "AUTOPILOT: INITIATING LANDING...";
                    }
                }

                ImGui::Separator();

                // ── SAS 快速模式 ──────────────────────────────────
                ImGui::TextColored({0.8f,1.0f,0.5f,1}, "SAS 快速模式");
                ImGui::Separator();
                auto sasBtn = [&](const char* lbl, SASMode m) {
                    bool act = guid.sas_active && guid.sas_mode == m;
                    if (act) ImGui::PushStyleColor(ImGuiCol_Button, {0.2f,0.6f,0.9f,0.9f});
                    else     ImGui::PushStyleColor(ImGuiCol_Button, {0.25f,0.25f,0.3f,0.8f});
                    if (ImGui::Button(lbl, {120.f,22.f})) { guid.sas_mode=m; guid.sas_active=true; }
                    ImGui::PopStyleColor();
                };
                sasBtn("顺行", SAS_PROGRADE);   ImGui::SameLine();
                sasBtn("逆行", SAS_RETROGRADE);
                sasBtn("法向+", SAS_NORMAL);    ImGui::SameLine();
                sasBtn("法向-", SAS_ANTINORMAL);
                sasBtn("径向外", SAS_RADIAL_OUT); ImGui::SameLine();
                sasBtn("径向内", SAS_RADIAL_IN);
                sasBtn("变轨节点", SAS_MANEUVER); ImGui::SameLine();
                sasBtn("稳定", SAS_STABILITY);

                ImGui::Separator();

                // ── 转移窗口 ──────────────────────────────────────
                ImGui::TextColored({0.3f,0.8f,1.0f,1}, "行星际工具");
                if (ImGui::Button(hud_transfer_win ? "[ 转移窗口 ]" : "转移窗口", {250.f, 22.f}))
                    hud_transfer_win = !hud_transfer_win;

                ImGui::End();
            }

            // ── 转移窗口面板 ─────────────────────────────────────
            if (hud_transfer_win && hud_flight_asst && inPanorama) {
                auto& ss3 = UniverseModel::getInstance().solar_system;
                int nBodies3 = (int)ss3.size();

                float tw_w = 520.f;
                float tw_h = 580.f;
                float tw_x = (float)ww - 560.f - tw_w - 20.f;  // 放到高级轨道面板左边
                float tw_y = 10.f;

                ImGui::SetNextWindowPos(ImVec2(tw_x, tw_y), ImGuiCond_Always);
                ImGui::SetNextWindowSize(ImVec2(tw_w, tw_h), ImGuiCond_Always);
                ImGui::SetNextWindowBgAlpha(0.90f);
                ImGui::Begin("转移窗口计算器##tw", nullptr,
                    ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove|
                    ImGuiWindowFlags_NoCollapse);

                ImGui::TextColored({0.4f,0.8f,1.0f,1}, "TRANSFER WINDOW CALCULATOR");
                ImGui::Separator();

                // ── 目标天体选择 ────────────────────────────────
                ImGui::TextColored({0.8f,0.8f,0.8f,1}, "目标天体:");
                ImGui::SameLine();
                int origin = TransferCalculator::getTransferOriginBody();
                if (ImGui::Button("<##tgtL", {24.f,20.f})) {
                    do { hud_transfer_target--; if (hud_transfer_target < 1) hud_transfer_target = nBodies3-1; }
                    while (hud_transfer_target == origin || hud_transfer_target == 4 || hud_transfer_target == 0);
                    hud_transfer_valid = false;
                }
                ImGui::SameLine();
                if (hud_transfer_target >= 0 && hud_transfer_target < nBodies3)
                    ImGui::TextColored({1.0f,0.9f,0.3f,1}, "%s", ss3[hud_transfer_target].name.c_str());
                ImGui::SameLine();
                if (ImGui::Button(">##tgtR", {24.f,20.f})) {
                    do { hud_transfer_target++; if (hud_transfer_target >= nBodies3) hud_transfer_target = 1; }
                    while (hud_transfer_target == origin || hud_transfer_target == 4 || hud_transfer_target == 0);
                    hud_transfer_valid = false;
                }
                ImGui::SameLine();
                ImGui::TextColored({0.5f,0.5f,0.5f,1}, "  出发: %s", ss3[origin].name.c_str());

                ImGui::Separator();

                // ── CALCULATE 按钮 ──────────────────────────────
                if (ImGui::Button("计算转移窗口", {tw_w - 20.f, 28.f})) {
                    hud_transfer_result = TransferCalculator::computePorkchop(origin, hud_transfer_target, tele.sim_time, 40);
                    hud_transfer_valid = hud_transfer_result.computed;
                    hud_transfer_hover_dep = -1;
                    hud_transfer_hover_tof = -1;
                }

                ImGui::Separator();

                // ── Porkchop 热力图 ────────────────────────────
                if (hud_transfer_valid && hud_transfer_result.computed) {
                    int gn = hud_transfer_result.n_dep;
                    const float plotW = tw_w - 80.f;   // 留出 Y 轴标签空间
                    const float plotH = 380.f;
                    const float plotX = 60.f;           // 左侧边距给标签
                    float plotY = ImGui::GetCursorPosY();

                    ImDrawList* tdl = ImGui::GetWindowDrawList();
                    ImVec2 winPos = ImGui::GetWindowPos();

                    float cellW = plotW / gn;
                    float cellH = plotH / gn;

                    double dvMinPlot = hud_transfer_result.min_dv;
                    double dvMaxPlot = dvMinPlot * 5.0;
                    if (dvMaxPlot < dvMinPlot + 1000.0) dvMaxPlot = dvMinPlot + 1000.0;

                    // 绘制网格
                    hud_transfer_hover_dep = -1;
                    hud_transfer_hover_tof = -1;
                    for (int gi = 0; gi < gn; gi++) {
                        for (int gj = 0; gj < gn; gj++) {
                            int idx = gi * gn + gj;
                            if (idx >= (int)hud_transfer_result.grid.size()) continue;
                            const PorkchopPoint& pt = hud_transfer_result.grid[idx];

                            float cx = winPos.x + plotX + (gi + 0.5f) * cellW;
                            float cy = winPos.y + plotY + plotH - (gj + 0.5f) * cellH;

                            // 鼠标悬停检测
                            ImVec2 mp = ImGui::GetMousePos();
                            if (mp.x >= cx - cellW*0.5f && mp.x <= cx + cellW*0.5f &&
                                mp.y >= cy - cellH*0.5f && mp.y <= cy + cellH*0.5f) {
                                hud_transfer_hover_dep = gi;
                                hud_transfer_hover_tof = gj;
                            }

                            if (!pt.valid) {
                                tdl->AddRectFilled({cx-cellW*0.45f, cy-cellH*0.45f},
                                                   {cx+cellW*0.45f, cy+cellH*0.45f},
                                                   IM_COL32(20,20,20,180));
                                continue;
                            }

                            float t = (float)((pt.dv_total - dvMinPlot) / (dvMaxPlot - dvMinPlot));
                            t = (t < 0.f) ? 0.f : (t > 1.f ? 1.f : t);
                            ImU32 col;
                            if (t < 0.33f) {
                                col = IM_COL32((int)(t*3*255), 255, 0, 220);
                            } else if (t < 0.66f) {
                                float s = (t-0.33f)*3;
                                col = IM_COL32(255, (int)((1.f-s)*255), 0, 220);
                            } else {
                                float s = (t-0.66f)*3;
                                col = IM_COL32(255, 0, (int)(s*128), 220);
                            }
                            tdl->AddRectFilled({cx-cellW*0.45f, cy-cellH*0.45f},
                                               {cx+cellW*0.45f, cy+cellH*0.45f}, col);
                        }
                    }

                    // 标记最小 ΔV 点
                    if (hud_transfer_result.min_dv_index >= 0) {
                        int mi = hud_transfer_result.min_dv_index / gn;
                        int mj = hud_transfer_result.min_dv_index % gn;
                        float mx = winPos.x + plotX + (mi + 0.5f) * cellW;
                        float my = winPos.y + plotY + plotH - (mj + 0.5f) * cellH;
                        tdl->AddRect({mx-cellW*0.6f, my-cellH*0.6f},
                                     {mx+cellW*0.6f, my+cellH*0.6f},
                                     IM_COL32(255,255,255,255), 0.f, 0, 2.f);
                        tdl->AddLine({mx-cellW*0.8f, my}, {mx+cellW*0.8f, my}, IM_COL32(255,255,255,200), 1.5f);
                        tdl->AddLine({mx, my-cellH*0.8f}, {mx, my+cellH*0.8f}, IM_COL32(255,255,255,200), 1.5f);
                    }

                    // 轴标签
                    ImGui::SetCursorPosY(plotY + plotH + 4.f);
                    ImGui::SetWindowFontScale(0.72f);
                    ImGui::TextColored({0.6f,0.6f,0.6f,1}, "Departure (days from now)  -->");
                    ImGui::SetWindowFontScale(1.0f);
                    for (int ti = 0; ti <= 4; ti++) {
                        float fx = (float)ti / 4.f;
                        double depDay = (hud_transfer_result.dep_start + fx * (hud_transfer_result.dep_end - hud_transfer_result.dep_start) - tele.sim_time) / 86400.0;
                        char tick[16]; snprintf(tick, 16, "%.0f", depDay);
                        float tx = winPos.x + plotX + fx * plotW;
                        tdl->AddText({tx - 10.f, winPos.y + plotY + plotH + 2.f}, IM_COL32(160,160,160,200), tick);
                    }

                    // ── 信息显示 ────────────────────────────────
                    ImGui::SetCursorPosY(plotY + plotH + 22.f);
                    if (hud_transfer_result.min_dv_index >= 0) {
                        const PorkchopPoint& best = hud_transfer_result.grid[hud_transfer_result.min_dv_index];
                        ImGui::TextColored({0.2f,1.0f,0.4f,1},
                            "MIN ΔV: %.2f km/s  (Dep: %.2f  Arr: %.2f)",
                            best.dv_total/1000.0, best.dv_departure/1000.0, best.dv_arrival/1000.0);
                        double depDays = (best.departure_time - tele.sim_time) / 86400.0;
                        double tofDays = best.tof / 86400.0;
                        ImGui::TextColored({0.7f,0.7f,0.7f,1},
                            "Depart: T+%.1f days | Travel: %.1f days", depDays, tofDays);
                    }

                    // 悬停 tooltip
                    if (hud_transfer_hover_dep >= 0 && hud_transfer_hover_tof >= 0) {
                        int hidx = hud_transfer_hover_dep * gn + hud_transfer_hover_tof;
                        if (hidx >= 0 && hidx < (int)hud_transfer_result.grid.size()) {
                            const PorkchopPoint& hpt = hud_transfer_result.grid[hidx];
                            if (hpt.valid) {
                                ImGui::SetCursorPosY(plotY + plotH + 52.f);
                                ImGui::TextColored({0.5f,0.8f,1.0f,1},
                                    "悬停: ΔV=%.2f km/s | T+%.0fd / %.0fd flight",
                                    hpt.dv_total/1000.0,
                                    (hpt.departure_time - tele.sim_time)/86400.0,
                                    hpt.tof/86400.0);
                            }
                        }
                    }

                    // ── CREATE MANEUVER NODE ──────────────────
                    ImGui::SetCursorPosY(plotY + plotH + 72.f);
                    bool hasBest = (hud_transfer_result.min_dv_index >= 0);
                    if (!hasBest) ImGui::BeginDisabled();
                    if (ImGui::Button("从最优解创建变轨节点", {tw_w - 20.f, 28.f})) {
                        const PorkchopPoint& best = hud_transfer_result.grid[hud_transfer_result.min_dv_index];
                        ManeuverNode node;
                        node.sim_time = best.departure_time;
                        node.active = true;
                        node.ref_body = UniverseModel::getInstance().current_soi_index;

                        double mu_soi = G_const * UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index].mass;
                        double npx, npy, npz, nvx, nvy, nvz;
                        get3DStateAtTime(trans.px, trans.py, trans.pz,
                                        vel.vx, vel.vy, vel.vz,
                                        mu_soi, best.departure_time - tele.sim_time,
                                        npx, npy, npz, nvx, nvy, nvz);
                        ManeuverFrame frame = ManeuverSystem::getFrame(
                            Vec3((float)npx, (float)npy, (float)npz),
                            Vec3((float)nvx, (float)nvy, (float)nvz));
                        Vec3 dv_world = best.departure_dv_vec;
                        float proComp = dv_world.dot(frame.prograde);
                        float nrmComp = dv_world.dot(frame.normal);
                        float radComp = dv_world.dot(frame.radial);
                        node.delta_v = Vec3(proComp, nrmComp, radComp);

                        mnv.maneuvers.clear();
                        mnv.maneuvers.push_back(node);
                        mnv.selected_maneuver_index = 0;
                        guid.mission_msg = "TRANSFER NODE CREATED";
                    }
                    if (!hasBest) ImGui::EndDisabled();
                }
                ImGui::End();
            }
        }

        // ── 帮助浮窗 ──────────────────────────────────────────────────
        static bool showHelp = false;
        ImGui::SetNextWindowPos(ImVec2((float)ww-215, (float)wh-34), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.65f);
        ImGui::Begin("##HelpBtn", nullptr,
            ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
            ImGuiWindowFlags_NoMove|ImGuiWindowFlags_AlwaysAutoResize);
        if (ImGui::SmallButton(showHelp?"隐藏帮助":"? 帮助")) showHelp=!showHelp;
        ImGui::End();

        if (showHelp) {
            ImGui::SetNextWindowPos(ImVec2((float)ww*0.5f-200, (float)wh-225), ImGuiCond_Always);
            ImGui::SetNextWindowSize(ImVec2(400, 210), ImGuiCond_Always);
            ImGui::SetNextWindowBgAlpha(0.78f);
            ImGui::Begin("##Help", nullptr,
                ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove);
            ImGui::SetWindowFontScale(0.9f);
            ImGui::TextColored({0.8f,0.8f,1,1}, "飞行操作指南");
            ImGui::Separator();
            ImGui::Columns(2, nullptr, false);
            ImGui::Text("SPACE  发射/分级");  ImGui::NextColumn(); ImGui::Text("Z/X    全油门/熄火");  ImGui::NextColumn();
            ImGui::Text("WASD   姿态控制");   ImGui::NextColumn(); ImGui::Text("TAB    自动/手动");    ImGui::NextColumn();
            ImGui::Text("1-4    时间加速");   ImGui::NextColumn(); ImGui::Text("O      切换视角");     ImGui::NextColumn();
            ImGui::Text("T      SAS 开关");   ImGui::NextColumn(); ImGui::Text("R      RCS 开关");     ImGui::NextColumn();
            ImGui::Text("右键拖 旋转相机");   ImGui::NextColumn(); ImGui::Text("滚轮   缩放");         ImGui::NextColumn();
            ImGui::Columns(1);
            ImGui::End();
        }
    }
};
