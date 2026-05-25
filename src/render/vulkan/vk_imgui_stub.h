#pragma once
// Stub: ImGui not available
struct VkImGuiSystem {
    bool init(void*, void*, unsigned) { return true; }
    void shutdown(void*) {}
    void newFrame() {}
    void renderToSwapchain(void*, void*, void*, void*) {}
};
