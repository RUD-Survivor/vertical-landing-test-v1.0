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
#include "scene/flight_scene.h"
#include "render/scene_snapshot.h"
#include "save_system.h"
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

    // ── Flight HUD 附加状态 ───────────────────────────────────────────────
    bool hud_show_galaxy = false;  // 星系信息覆盖层
    int  hud_sel_body    = -1;     // 选中的天体索引
    int  hud_exp_planet  = -1;     // 展开卫星的行星索引
    bool hud_adv_orbit   = false;  // 高级轨道面板
    bool hud_flight_asst = false;  // 飞行辅助面板

    // ── Factory UI 状态 ─────────────────────────────────────────────────
    int factorySelectedNode = -1;

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
    // Workshop 相机：每帧从 main.cpp 调用（在 ImGui 之前）
    void updateWorkshopInput(GLFWwindow* window) {
        if (state != VkGameState::WORKSHOP) return;
        ws_frameCount++;

        double mx, my;
        glfwGetCursorPos(window, &mx, &my);
        bool rmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

        if (rmb) {
            if (ws_rmbPrev) {
                float dx = (float)(mx - ws_prevMX) * 0.005f;
                float dy = (float)(my - ws_prevMY) * 0.005f;
                ws_angle -= dx;
                ws_pitch  = std::max(-0.45f, std::min(1.4f, ws_pitch + dy));
            }
            ws_prevMX = mx; ws_prevMY = my;
        }
        ws_rmbPrev = rmb;

        // 自动慢速旋转（未右键时）
        if (!rmb) ws_angle += 0.002f;
    }

    void handleWorkshopScroll(float dy) {
        if (state != VkGameState::WORKSHOP) return;
        if (glfwGetKey(GameContext::getInstance().window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
            ws_dist = std::max(2.0f, std::min(80.0f, ws_dist * powf(0.85f, dy)));
        else
            ws_yOff += dy * (ws_dist * 0.05f);
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
                rp.r = def.r; rp.g = def.g; rp.b = def.b;
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
    // NAVBALL — ImDrawList 绘制的人工地平仪
    // pitch: 从水平面算起的仰角 (rad, 正=抬头)
    // roll:  滚转角 (rad)
    // ================================================================
    static void _drawNavball(ImDrawList* dl, ImVec2 ctr, float R, float pitch, float roll) {
        const float kPI  = 3.14159265358979f;
        const float kDEG = kPI / 180.f;

        // ── 1. 天空填充 ────────────────────────────────────────────────
        dl->AddCircleFilled(ctr, R, IM_COL32(18, 58, 148, 228));

        // ── 2. 地面多边形 ──────────────────────────────────────────────
        float hOff = sinf(pitch) * R;
        float hw   = sqrtf(std::max(0.f, R*R - hOff*hOff));
        bool allGround = (hOff <= -(R - 0.5f));
        bool allSky    = (hOff >=  (R - 0.5f));

        if (allGround) {
            dl->AddCircleFilled(ctr, R, IM_COL32(100, 62, 18, 228));
        } else if (!allSky && hw > 1.f) {
            float hcx = ctr.x + hOff * sinf(roll);
            float hcy = ctr.y + hOff * cosf(roll);
            float hdx = cosf(roll), hdy = -sinf(roll);
            float h1x = hcx - hw*hdx, h1y = hcy - hw*hdy;
            float h2x = hcx + hw*hdx, h2y = hcy + hw*hdy;

            float a1 = atan2f(h1y - ctr.y, h1x - ctr.x);
            float a2 = atan2f(h2y - ctr.y, h2x - ctr.x);
            float ag = atan2f(cosf(roll), sinf(roll));
            float cw12 = a2 - a1; if (cw12 < 0) cw12 += 2*kPI;
            float cw1g = ag - a1; if (cw1g < 0) cw1g += 2*kPI;
            bool goCW = (cw1g < cw12);

            const int N = 48;
            ImVec2 poly[N + 2]; int np = 0;
            poly[np++] = {h1x, h1y};
            float dAng = (goCW ? cw12 : cw12 - 2*kPI) / N;
            for (int s = 1; s <= N; s++) {
                float a = a1 + s*dAng;
                poly[np++] = {ctr.x + R*cosf(a), ctr.y + R*sinf(a)};
            }
            if (np >= 3)
                dl->AddConvexPolyFilled(poly, np, IM_COL32(100, 62, 18, 228));

            // 地平线（粗白线）
            dl->AddLine({h1x,h1y},{h2x,h2y}, IM_COL32(255,255,255,230), 2.5f);
            // 中央间隔缺口
            float gi = R*0.12f, go = R*0.28f;
            dl->AddLine({hcx-hdx*go,hcy-hdy*go},{hcx-hdx*gi,hcy-hdy*gi}, IM_COL32(255,255,255,190), 2.f);
            dl->AddLine({hcx+hdx*gi,hcy+hdy*gi},{hcx+hdx*go,hcy+hdy*go}, IM_COL32(255,255,255,190), 2.f);
        }

        // ── 3. 俯仰刻度尺（每10°，±80°范围，随滚转旋转）─────────────
        bool showLbl = (fabsf(roll) < 40.f * kDEG); // 滚转小时显示文字标注
        for (int deg = -80; deg <= 80; deg += 10) {
            if (deg == 0) continue;
            float p2  = pitch + (float)deg * kDEG;
            float ho2 = sinf(p2) * R;
            if (fabsf(ho2) >= R - 4.f) continue;

            float hc2x = ctr.x + ho2 * sinf(roll);
            float hc2y = ctr.y + ho2 * cosf(roll);
            float hd2x = cosf(roll), hd2y = -sinf(roll);

            bool is30 = (deg % 30 == 0);
            bool is20 = (deg % 20 == 0 && !is30);
            float tHalf = is30 ? R*0.30f : (is20 ? R*0.20f : R*0.14f);
            float thick = is30 ? 1.9f : 1.3f;
            ImU32 lc = is30 ? IM_COL32(255,255,195,185) : IM_COL32(255,255,180,130);

            float ex1 = hc2x - hd2x*tHalf, ey1 = hc2y - hd2y*tHalf;
            float ex2 = hc2x + hd2x*tHalf, ey2 = hc2y + hd2y*tHalf;

            // 两端点都在圆内才画
            auto inC = [&](float x, float y){ float dx=x-ctr.x,dy=y-ctr.y; return dx*dx+dy*dy<(R-3.f)*(R-3.f); };
            if (!inC(ex1,ey1) || !inC(ex2,ey2)) continue;

            dl->AddLine({ex1,ey1},{ex2,ey2}, lc, thick);

            // 度数标注（仅主刻度）
            if (showLbl && is30) {
                char lbl[6]; snprintf(lbl,6,"%d", deg);
                float lx = ex1 - hd2x*4.f - 14.f;
                float ly = ey1 - hd2y*4.f -  6.f;
                if (inC(lx+7.f, ly+6.f))
                    dl->AddText({lx, ly}, IM_COL32(255,255,195,210), lbl);
            }
        }

        // ── 4. 滚转刻度环（球体上，随滚转旋转）───────────────────────
        // screen_angle = -π/2 + roll + mark_radians
        int rollMarkDegs[] = {0,30,60,90,120,150,180,-30,-60,-90,-120,-150};
        for (int rm : rollMarkDegs) {
            float sa = -kPI*0.5f + roll + (float)rm*kDEG;
            float ca = cosf(sa), sb = sinf(sa);
            bool  isZero = (rm == 0);
            bool  is90   = (rm == 90 || rm == -90 || rm == 180);
            float tLen = isZero ? R*0.15f : (is90 ? R*0.10f : R*0.07f);
            float th   = isZero ? 2.5f : 1.5f;
            ImU32 rc   = isZero ? IM_COL32(255,225,45,245) : (is90 ? IM_COL32(220,220,220,190) : IM_COL32(180,180,180,150));
            float ri = R - tLen - 1.f;
            float ro = R - 1.f;
            dl->AddLine({ctr.x+ri*ca, ctr.y+ri*sb}, {ctr.x+ro*ca, ctr.y+ro*sb}, rc, th);
        }

        // ── 5. 框架指针（固定顶部，黄色三角）─────────────────────────
        // 三角尖端指向圆内，固定不随球体旋转
        {
            float ty = ctr.y - R - 4.f;
            dl->AddTriangleFilled(
                {ctr.x,       ty + 11.f},   // 尖端（向下，指向球面）
                {ctr.x - 7.f, ty},
                {ctr.x + 7.f, ty},
                IM_COL32(255,225,45,240));
            dl->AddTriangle(
                {ctr.x,       ty + 11.f},
                {ctr.x - 7.f, ty},
                {ctr.x + 7.f, ty},
                IM_COL32(0,0,0,120), 1.f);
        }

        // ── 6. 外圈边框 ────────────────────────────────────────────────
        dl->AddCircle(ctr, R, IM_COL32(160,160,160,220), 64, 2.2f);

        // ── 7. 中心准星（固定黄色，表示火箭鼻锥方向）─────────────────
        float g = R * 0.11f, l = R * 0.34f;
        dl->AddLine({ctr.x-l-g, ctr.y}, {ctr.x-g, ctr.y},     IM_COL32(255,210,0,245), 2.5f);
        dl->AddLine({ctr.x+g,   ctr.y}, {ctr.x+l+g, ctr.y},   IM_COL32(255,210,0,245), 2.5f);
        dl->AddCircle(ctr, g,                                    IM_COL32(255,210,0,245), 16, 2.f);
        dl->AddLine({ctr.x, ctr.y-g},   {ctr.x, ctr.y-g*1.9f}, IM_COL32(255,210,0,245), 2.5f);
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
    // FACTORY — 工厂管理
    // ================================================================
    void _drawFactory(int ww, int wh) {
        GameContext& gc = GameContext::getInstance();
        AgencyState& ag = gc.agency_state;
        FactorySystem& factory = gc.factory;

        // ── 工厂节点列表 ──────────────────────────────────────────────
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(280, (float)wh-20), ImGuiCond_Always);
        ImGui::Begin("工厂节点", nullptr, ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove);

        ImGui::TextColored({0.4f,0.85f,1.0f,1}, "建筑列表 (%d)", (int)factory.nodes.size());
        ImGui::Separator();

        // 新增节点按钮
        static const char* nodeTypeLabels[] = {"矿机","熔炉","组装机","仓库","发电机","市场"};
        if (ImGui::BeginCombo("添加建筑", "选择类型...")) {
            for (int i = 0; i < NODE_TYPE_COUNT; i++) {
                if (ImGui::Selectable(nodeTypeLabels[i])) {
                    factory.addNode((FactoryNodeType)i, (int)factory.nodes.size() % 5, (int)factory.nodes.size() / 5);
                }
            }
            ImGui::EndCombo();
        }
        ImGui::Separator();

        // 节点列表
        ImGui::BeginChild("##NodeList", ImVec2(0, (float)wh*0.55f), true);
        for (int i = 0; i < (int)factory.nodes.size(); i++) {
            const FactoryNode& n = factory.nodes[i];
            ImGui::PushID(n.id);
            bool sel = (factorySelectedNode == n.id);

            char label[64];
            snprintf(label,64,"[%d] %s  %.0f%%", n.id, NodeTypeName(n.type), n.progress*100.f);
            if (ImGui::Selectable(label, sel)) factorySelectedNode = n.id;

            // 进度条
            ImGui::SameLine(200);
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, {0.25f,0.65f,1.0f,1});
            ImGui::ProgressBar(n.progress, ImVec2(60,10), "");
            ImGui::PopStyleColor();
            ImGui::PopID();
        }
        ImGui::EndChild();

        // 删除选中节点
        if (factorySelectedNode >= 0) {
            if (ImGui::Button("删除选中节点", ImVec2(-1,30))) {
                factory.removeNode(factorySelectedNode);
                factorySelectedNode = -1;
            }
        }

        ImGui::End();

        // ── 右侧：详情 + 传送带 ───────────────────────────────────────
        ImGui::SetNextWindowPos(ImVec2(300, 10), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2((float)ww-320, (float)wh-20), ImGuiCond_Always);
        ImGui::Begin("工厂详情", nullptr, ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove);

        // 能源总览
        ImGui::SeparatorText("能源");
        float pwr = factory.total_power_req > 0 ?
            factory.total_power_gen / factory.total_power_req : 1.0f;
        pwr = std::min(1.0f, pwr);
        ImGui::Text("发电量:"); ImGui::SameLine(100);
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, pwr>=1 ? ImVec4{0.2f,0.9f,0.3f,1} : ImVec4{1,0.4f,0.2f,1});
        char pwrBuf[32]; snprintf(pwrBuf,32,"%.1f/%.1f kW",factory.total_power_gen,factory.total_power_req);
        ImGui::ProgressBar(pwr, ImVec2(200,16), pwrBuf);
        ImGui::PopStyleColor();
        ImGui::TextDisabled("效率: %.0f%%", factory.power_efficiency*100.f);

        // 选中节点详情
        ImGui::SeparatorText("节点详情");
        FactoryNode* sel = (factorySelectedNode>=0) ? factory.findNode(factorySelectedNode) : nullptr;
        if (!sel) {
            ImGui::TextDisabled("（点击左侧列表选中节点）");
        } else {
            ImGui::Text("类型: %s", NodeTypeName(sel->type));
            ImGui::Text("位置: (%d, %d)", sel->grid_x, sel->grid_y);
            ImGui::Text("进度: %.1f%%", sel->progress*100.f);
            ImGui::Text("生产中: %s", sel->is_producing ? "是" : "否");

            // 矿机：选择矿种
            if (sel->type == NODE_MINER) {
                ImGui::SeparatorText("矿机配置");
                static const ItemType rawTypes[] = {ITEM_IRON_ORE,ITEM_COPPER_ORE,ITEM_COAL,ITEM_SILICON,ITEM_TITANIUM_ORE};
                if (ImGui::BeginCombo("矿种", itemName(sel->mine_output))) {
                    for (auto it : rawTypes) {
                        if (ImGui::Selectable(itemName(it), sel->mine_output==it))
                            sel->mine_output = it;
                    }
                    ImGui::EndCombo();
                }
            }

            // 输出缓冲
            if (!sel->output_buffer.items.empty()) {
                ImGui::SeparatorText("输出缓冲");
                for (auto& [it, cnt] : sel->output_buffer.items) {
                    ImGui::Text("  %s x%d", itemName(it), cnt);
                }
                if (ImGui::SmallButton("转入库存")) {
                    for (auto& [it, cnt] : sel->output_buffer.items)
                        ag.addItem(it, cnt);
                    sel->output_buffer.items.clear();
                }
            }
        }

        // 传送带列表
        ImGui::SeparatorText("传送带");
        if (factory.belts.empty()) {
            ImGui::TextDisabled("（暂无传送带）");
        } else {
            for (auto& b : factory.belts) {
                ImGui::Text("节点 %d → %d  (%d 件在途)", b.from_node_id, b.to_node_id, (int)b.items.size());
            }
        }

        ImGui::Separator();
        if (ImGui::Button("保存并返回航天中心", ImVec2(-1, 36))) {
            SaveSystem::SaveAgencyFactory(ag, factory);
            state = VkGameState::AGENCY;
        }

        ImGui::End();
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

            // 点击选中
            if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(0))
                selectedPart = i;

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
                assembly.addPart(selectedPart, -1, ws_symmetry);
                assembly.recalculate();
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

        // 右键上下文弹出
        if (partCtxTrigger >= 0) {
            ws_ctxIdx  = partCtxTrigger;
            ws_ctxOpen = true;
            ImGui::OpenPopup("##WsPartCtx");
        }
        if (ws_ctxOpen && ws_ctxIdx >= 0 && ws_ctxIdx < (int)assembly.parts.size()) {
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
                    char pid[32]; snprintf(pid, 32, "+##%s", lbl);
                    if (ImGui::SmallButton(pid)) { val += step; assembly.recalculate(); ws_centerViz.lastAssemblyHash = 0; }
                    ImGui::SameLine();
                    char mid[32]; snprintf(mid, 32, "-##%s", lbl);
                    if (ImGui::SmallButton(mid)) { val -= step; assembly.recalculate(); ws_centerViz.lastAssemblyHash = 0; }
                    ImGui::SameLine(); ImGui::Text("%.2f", val);
                };
                adjRow("X", cp2.pos.x, 0.25);
                adjRow("Y", cp2.pos.y, 0.25);
                adjRow("Z", cp2.pos.z, 0.25);
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

        // ─ 相机操作提示 ─
        ImGui::Separator();
        ImGui::PushStyleColor(ImGuiCol_Text, {0.45f,0.45f,0.50f,1});
        ImGui::SetWindowFontScale(0.78f);
        ImGui::TextWrapped("右键拖动 旋转 | Shift+滚轮 缩放 | 滚轮 上下移动");
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
        _drawNavball(dl, navCtr, navR, pitchRad, rollRad);

        // ── 轨道矢量标记 ──────────────────────────────────────────────
        if (speed > 1.0) {
            Quat rqInv = fs->ctx.rocketQuat.conjugate();
            auto projectNav = [&](const Vec3& wv, float& nx, float& ny) -> bool {
                Vec3 body = rqInv.rotate(wv);
                float cosA = (float)std::max(-1.0, std::min(1.0, body.y));
                float theta = acosf(cosA);
                float r2d = sinf(theta) * navR;
                float l2d = sqrtf((float)(body.x*body.x + body.z*body.z));
                if (l2d > 0.001f) {
                    nx = navCtr.x + (float)(body.x / l2d) * r2d;
                    ny = navCtr.y - (float)(body.z / l2d) * r2d;
                } else { nx = navCtr.x; ny = navCtr.y; }
                float dx=nx-navCtr.x, dy=ny-navCtr.y, d=sqrtf(dx*dx+dy*dy);
                if (d > navR-3.f) { float s=(navR-3.f)/d; nx=navCtr.x+dx*s; ny=navCtr.y+dy*s; }
                return cosA > -0.5f;
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
        static const char* sasLabels[] = {"稳","顺行","逆行","变轨","法+","法-","径内","径外"};
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
            if (ImGui::Button(hud_adv_orbit  ?"[ 高级轨道 ]":"高级轨道",  {120.f,20.f})) hud_adv_orbit   = !hud_adv_orbit;
            ImGui::SameLine();
            if (ImGui::Button(hud_flight_asst?"[ 飞行辅助 ]":"飞行辅助", {120.f,20.f})) hud_flight_asst = !hud_flight_asst;
            ImGui::Separator();
            if (ImGui::Button(hud_show_galaxy?"[ 星系信息 ]":"星系信息",  {120.f,20.f})) hud_show_galaxy = !hud_show_galaxy;
            ImGui::SetWindowFontScale(0.80f);
            ImGui::TextColored({0.6f,0.6f,0.6f,1}, "右键拖 旋转 | 滚轮 缩放\n[O] 切换视角");
            ImGui::SetWindowFontScale(1.0f);
            ImGui::End();

            // ── 高级轨道面板 ─────────────────────────────────────────
            if (hud_adv_orbit) {
                ImGui::SetNextWindowPos(ImVec2((float)ww - 565.f, 10.f), ImGuiCond_Always);
                ImGui::SetNextWindowSize(ImVec2(268.f, 0), ImGuiCond_Always);
                ImGui::SetNextWindowBgAlpha(0.82f);
                ImGui::Begin("高级轨道##adv", nullptr,
                    ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove|
                    ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_AlwaysAutoResize);
                ImGui::TextColored({0.6f,0.8f,1,1}, "变轨节点");
                ImGui::Separator();
                auto& mnvs = mnv.maneuvers;
                if (mnvs.empty()) {
                    ImGui::TextColored({0.6f,0.6f,0.6f,1}, "无变轨节点");
                } else {
                    for (int i=0; i<(int)mnvs.size(); i++) {
                        double dt2 = mnvs[i].sim_time - tele.sim_time;
                        bool sel2 = (mnv.selected_maneuver_index == i);
                        if (sel2) ImGui::PushStyleColor(ImGuiCol_Text, {1,0.9f,0.3f,1});
                        double dv_mag = mnvs[i].delta_v.length();
                        ImGui::Text("节点 %d  T%+.0fs  ΔV=%.0f m/s", i+1, dt2, dv_mag);
                        if (sel2) ImGui::PopStyleColor();
                        ImGui::SameLine();
                        if (ImGui::SmallButton(("选##mn"+std::to_string(i)).c_str()))
                            mnv.selected_maneuver_index = i;
                    }
                }
                ImGui::Separator();
                ImGui::TextColored({0.5f,0.8f,0.5f,1}, "当前轨道根数");
                if (a_val > 0) {
                    ImGui::Text("a = %.1f km  e = %.4f  i = %.2f°", a_val/1000.0, e_val, inc_deg);
                } else {
                    ImGui::TextColored({0.6f,0.6f,0.6f,1}, "轨道数据不足（速度过低）");
                }
                ImGui::End();
            }

            // ── 飞行辅助面板 ─────────────────────────────────────────
            if (hud_flight_asst) {
                float asst_y = 10.f + (hud_adv_orbit ? 160.f : 0.f);
                ImGui::SetNextWindowPos(ImVec2((float)ww - 565.f, asst_y), ImGuiCond_Always);
                ImGui::SetNextWindowSize(ImVec2(268.f, 0), ImGuiCond_Always);
                ImGui::SetNextWindowBgAlpha(0.82f);
                ImGui::Begin("飞行辅助##asst", nullptr,
                    ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove|
                    ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_AlwaysAutoResize);
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
