#pragma once
#include "renderer_2d.h"
#include "renderer3d.h"
#include <cstdio>
#include <algorithm>

// Real-time cloud parameter slider panel.
// Toggle visibility with Tab key (handled by caller).
// Use LMB to drag sliders. Reads/writes Renderer3D::cloudTuneParams directly.
struct CloudTuner {
    bool visible = false;
    int  dragging = -1;
    float dbgFloat = 0.0f;  // float bridge for debugMode int slider

    void toggle() { visible = !visible; }

    // Call every frame from flight scene render pass (after 3D, before swap).
    // mx/my: NDC mouse position (-1..1). lmb: left button held.
    void render(Renderer* r2d, Renderer3D* r3d, float mx, float my, bool lmb) {
        if (!visible) return;

        auto& p = r3d->cloudSystem.tuneParams;

        // Sync int debugMode ↔ float slider
        dbgFloat = (float)p.debugMode;

        // Panel layout (NDC)
        const float PX  = -0.99f;  // left edge
        const float PY  =  0.96f;  // top edge
        const float PW  =  0.62f;  // panel width
        const float RH  =  0.068f; // row height
        const int   N   =  10;

        const float panH = RH * N + 0.04f;
        const float cx   = PX + PW * 0.5f;
        const float cy   = PY - panH * 0.5f;

        // Background
        r2d->addRect(cx, cy, PW, panH, 0.04f, 0.04f, 0.10f, 0.88f);
        r2d->addRectOutline(cx, cy, PW, panH, 0.4f, 0.8f, 1.0f, 0.7f, 0.003f);

        // Title
        r2d->drawText(cx, PY - 0.025f, "Cloud Tuner  [F2] to close",
                      0.030f, 0.5f, 0.9f, 1.0f, 1.0f, false, Renderer::CENTER);

        // Slider descriptors: { label, ptr, min, max }
        struct SliderDef { const char* label; float* val; float lo; float hi; };
        SliderDef sliders[N] = {
            { "CovLo   (coverage smoothstep lo)", &p.covLo,     0.10f, 0.70f },
            { "CovHi   (coverage smoothstep hi)", &p.covHi,     0.30f, 0.90f },
            { "ThreshLo (clear-sky threshold)",   &p.threshLo,  0.40f, 0.92f },
            { "ThreshHi (overcast threshold)",    &p.threshHi,  0.05f, 0.55f },
            { "Erosion",                          &p.erosion,   0.00f, 0.50f },
            { "Density",                          &p.density,   0.50f,12.00f },
            { "Extinction",                       &p.extinction,0.01f, 0.60f },
            { "MinAlt (km)",                      &p.minAlt,    0.50f, 6.00f },
            { "MaxAlt (km)",                      &p.maxAlt,    8.00f,22.00f },
            { "Debug (0=off 1=force density)",    &p.dbgFloat,  0.00f, 1.00f },
        };

        const float LABEL_X  = PX + 0.01f;
        const float SLIDER_X = PX + 0.24f;   // slider bar left edge (NDC)
        const float SLIDER_W = PW - 0.26f;   // slider bar width

        for (int i = 0; i < N; i++) {
            float rowY = PY - 0.04f - RH * (i + 0.5f);

            // Label
            r2d->drawText(LABEL_X, rowY, sliders[i].label,
                          0.026f, 0.75f, 0.90f, 1.0f, 1.0f, false, Renderer::LEFT);

            // Slider track
            float trackCx = SLIDER_X + SLIDER_W * 0.5f;
            r2d->addRect(trackCx, rowY, SLIDER_W, 0.016f, 0.15f, 0.15f, 0.25f, 1.0f);

            // Filled portion
            float t = (*sliders[i].val - sliders[i].lo) / (sliders[i].hi - sliders[i].lo);
            t = std::max(0.0f, std::min(1.0f, t));
            float fillW  = SLIDER_W * t;
            float fillCx = SLIDER_X + fillW * 0.5f;
            r2d->addRect(fillCx, rowY, fillW, 0.016f, 0.25f, 0.65f, 1.0f, 1.0f);

            // Handle
            float handleX = SLIDER_X + SLIDER_W * t;
            r2d->addRect(handleX, rowY, 0.018f, 0.034f, 0.9f, 0.95f, 1.0f, 1.0f);

            // Value text
            char buf[16];
            snprintf(buf, sizeof(buf), "%.3f", *sliders[i].val);
            r2d->drawText(SLIDER_X + SLIDER_W + 0.025f, rowY, buf,
                          0.028f, 1.0f, 1.0f, 0.5f, 1.0f, false, Renderer::LEFT);

            // Hit detection: start drag when clicking on the row
            float rowTop = rowY + RH * 0.5f;
            float rowBot = rowY - RH * 0.5f;
            bool inRow = (my >= rowBot && my <= rowTop &&
                          mx >= SLIDER_X - 0.01f && mx <= SLIDER_X + SLIDER_W + 0.01f);

            if (lmb && inRow && dragging == -1)
                dragging = i;
        }

        // Update dragged slider value
        if (!lmb) {
            dragging = -1;
        } else if (dragging >= 0) {
            float t = (mx - SLIDER_X) / SLIDER_W;
            t = std::max(0.0f, std::min(1.0f, t));
            *sliders[dragging].val = sliders[dragging].lo +
                                     t * (sliders[dragging].hi - sliders[dragging].lo);
        }

        // Sync float slider → int debugMode
        p.debugMode = (int)(dbgFloat + 0.5f);

        // Print current values hint at bottom
        r2d->drawText(cx, PY - panH + 0.018f,
                      "Drag sliders, then tell Claude the values",
                      0.024f, 0.5f, 0.5f, 0.6f, 0.8f, false, Renderer::CENTER);
    }
};
