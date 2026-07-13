#pragma once
// ==========================================================================
// imgui_perf_overlay.h — Dear ImGui 帧率 / 帧时间监视面板
// 用法: PerfOverlayImGui(&open, ww, wh, presentModeName);
// F1 开关。计时使用 ImGui::GetIO().DeltaTime（须在 newFrame 之后调用）。
// ==========================================================================
#include "imgui.h"
#include <algorithm>

inline void PerfOverlayImGui(bool* p_open, int ww, int wh, const char* presentModeName) {
    if (ImGui::IsKeyPressed(ImGuiKey_F1, false)) *p_open = !*p_open;
    if (!*p_open) return;

    constexpr int kCapacity = 120;
    static float s_ms[kCapacity] = {};
    static int   s_count = 0;
    static int   s_head  = 0;  // next write index

    const ImGuiIO& io = ImGui::GetIO();
    float dt = io.DeltaTime;
    if (dt > 0.0f && dt < 1.0f) {
        s_ms[s_head] = dt * 1000.0f;
        s_head = (s_head + 1) % kCapacity;
        if (s_count < kCapacity) s_count++;
    }

    float curMs = (dt > 0.0f) ? dt * 1000.0f : 0.0f;
    float instFps = (dt > 1e-6f) ? (1.0f / dt) : 0.0f;
    float smoothFps = io.Framerate;

    ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(280.0f, 260.0f), ImGuiCond_FirstUseEver);
    ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNav;
    if (!ImGui::Begin("性能监视", p_open, flags)) {
        ImGui::End();
        return;
    }

    if (s_count <= 0) {
        ImGui::Text("采集中…");
        ImGui::TextDisabled("F1: 开关面板");
        ImGui::End();
        return;
    }

    float chron[kCapacity];
    float sorted[kCapacity];
    int start = (s_count < kCapacity) ? 0 : s_head;
    for (int i = 0; i < s_count; i++) {
        chron[i] = s_ms[(start + i) % kCapacity];
        sorted[i] = chron[i];
    }

    float minMs = chron[0], maxMs = chron[0];
    double sum = 0.0;
    for (int i = 0; i < s_count; i++) {
        sum += chron[i];
        if (chron[i] < minMs) minMs = chron[i];
        if (chron[i] > maxMs) maxMs = chron[i];
    }
    float avgMs = (float)(sum / s_count);

    std::sort(sorted, sorted + s_count);  // ascending ms
    int worstN = (std::max)(1, (s_count + 99) / 100);  // ceil(n * 0.01)
    double worstSum = 0.0;
    for (int i = s_count - worstN; i < s_count; i++)
        worstSum += sorted[i];
    float worstAvgMs = (float)(worstSum / worstN);
    float onePctLowFps = (worstAvgMs > 1e-6f) ? (1000.0f / worstAvgMs) : 0.0f;

    ImGui::Text("FPS  %.0f  (平滑 %.0f)", instFps, smoothFps);
    ImGui::Text("帧时间  %.2f ms", curMs);
    ImGui::Separator();
    ImGui::Text("窗口 (%d 帧)", s_count);
    ImGui::Text("  avg  %.2f ms", avgMs);
    ImGui::Text("  min  %.2f ms", minMs);
    ImGui::Text("  max  %.2f ms", maxMs);
    ImGui::Text("  1%% low  %.0f FPS", onePctLowFps);

    ImGui::Separator();
    ImGui::Text("帧时间 (ms)");
    ImGui::PlotLines("##ft", chron, s_count, 0, nullptr, 0.0f,
                     maxMs > 0.0f ? maxMs * 1.1f : 33.3f, ImVec2(-1.0f, 64.0f));

    ImGui::Separator();
    ImGui::TextDisabled("分辨率: %d × %d", ww, wh);
    ImGui::TextDisabled("Present: %s", presentModeName ? presentModeName : "?");
    ImGui::TextDisabled("F1: 开关面板");

    ImGui::End();
}
