#pragma once
// ==========================================================================
// imgui_cloud_tuner.h — Dear ImGui 体积云参数调试面板
// 用法: CloudTunerImGui(&open, params);
// ==========================================================================
#include "../cloud_system.h"  // CloudTuneParams
#include "imgui.h"

inline void CloudTunerImGui(bool* p_open, CloudTuneParams& p) {
    // F2 toggles the window
    if (ImGui::IsKeyPressed(ImGuiKey_F2, false)) *p_open = !*p_open;

    if (!*p_open) return;

    ImGui::SetNextWindowSize(ImVec2(380, 480), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("体积云调参", p_open, ImGuiWindowFlags_NoResize)) {
        ImGui::End();
        return;
    }

    ImGui::Text("密度与消光");
    ImGui::SliderFloat("Density",      &p.density,    0.10f, 4.00f, "%.2f");
    ImGui::SliderFloat("Extinction",   &p.extinction, 0.02f, 0.40f, "%.3f");

    ImGui::Separator();
    ImGui::Text("覆盖率");
    ImGui::SliderFloat("CovLo",        &p.covLo,      0.10f, 0.70f, "%.3f");
    ImGui::SliderFloat("CovHi",        &p.covHi,      0.30f, 0.90f, "%.3f");

    ImGui::Separator();
    ImGui::Text("逾渗阈值（碎块程度）");
    ImGui::SliderFloat("ThreshLo",     &p.threshLo,   0.05f, 0.55f, "%.3f");
    ImGui::SliderFloat("ThreshHi",     &p.threshHi,   0.40f, 0.92f, "%.3f");

    ImGui::Separator();
    ImGui::Text("形状");
    ImGui::SliderFloat("Erosion",      &p.erosion,    0.00f, 0.50f, "%.3f");
    ImGui::SliderFloat("MinAlt (km)",  &p.minAlt,     0.50f, 6.00f, "%.2f");
    ImGui::SliderFloat("MaxAlt (km)",  &p.maxAlt,     8.00f, 22.00f, "%.2f");
    ImGui::SliderFloat("LocalRadius (km)", &p.localRadiusKm, 0.f, 500.f, "%.0f");
    ImGui::TextDisabled("LocalRadius: Flower-style tangent cloudMap near camera (0=off)");

    ImGui::Separator();
    ImGui::Text("调试");
    const char* debugItems[] = {
        "Off",
        "Force Density",
        "Density",
        "Coverage/Profile/Shape",
        "Alpha/Trans/Depth",
        "Sun/Sky/Ground",
        "Step Optical Depth"
    };
    ImGui::Combo("DebugMode", &p.debugMode, debugItems, IM_ARRAYSIZE(debugItems));

    ImGui::Separator();
    ImGui::TextDisabled("F2: 开关面板  |  Shift+P: 开关云");

    ImGui::End();
}
