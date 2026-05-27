// ==========================================================
// center_visualizer.cpp — Center Point Visualization Implementation
// ==========================================================

#include "center_visualizer.h"

// Mesh creation functions from renderer3d.h (MeshGen)
#include "render/renderer3d.h"
#include "render/renderer_2d.h"

namespace CenterVisualizer {
    
    void renderMarker(Renderer3D* renderer, const Vec3& pos, 
                     MarkerType type, const Mat4& rocketTransform) {
        if (!renderer) return;
        
        // Create transformation matrix: translate to position
        Mat4 translation = Mat4::translate(pos);
        
        // Combine rocket transform with marker position
        Mat4 modelMatrix = rocketTransform * translation;
        
        // Select mesh and color based on marker type
        switch (type) {
            case MARKER_COM: {
                // Blue sphere for Center of Mass
                Mesh sphereMesh = MeshGen::sphere(16, 16, COM_RADIUS);
                renderer->drawMesh(sphereMesh, modelMatrix, 
                                 COM_COLOR[0], COM_COLOR[1], COM_COLOR[2], 0.8f);
                break;
            }
            case MARKER_COL: {
                // Yellow cone for Center of Lift
                // Cone is created with base at y=0 and tip at y=height
                // We need to offset it so it's centered at the position
                Mat4 coneOffset = Mat4::translate(Vec3(0, -COL_HEIGHT * 0.5f, 0));
                Mat4 coneModel = modelMatrix * coneOffset;
                Mesh coneMesh = MeshGen::cone(16, COL_RADIUS, COL_HEIGHT);
                renderer->drawMesh(coneMesh, coneModel,
                                 COL_COLOR[0], COL_COLOR[1], COL_COLOR[2], 0.8f);
                break;
            }
            case MARKER_COT: {
                // Red cylinder for Center of Thrust
                // Cylinder is already centered at origin along Y axis
                Mesh cylinderMesh = MeshGen::cylinder(16, COT_RADIUS, COT_HEIGHT);
                renderer->drawMesh(cylinderMesh, modelMatrix,
                                 COT_COLOR[0], COT_COLOR[1], COT_COLOR[2], 0.8f);
                break;
            }
        }
    }
    
    void renderLabel(Renderer* renderer2d, const Vec3& worldPos, 
                    const char* text, const Mat4& viewProj) {
        if (!renderer2d || !text) return;
        
        // Project 3D world position to clip space using view-projection matrix
        Vec3 clipPos = viewProj.transformPoint(worldPos);
        
        // Check if the point is behind the camera (negative z in clip space)
        // or outside the view frustum
        if (clipPos.z < -1.0f || clipPos.z > 1.0f) return;
        if (clipPos.x < -1.0f || clipPos.x > 1.0f) return;
        if (clipPos.y < -1.0f || clipPos.y > 1.0f) return;
        
        // Convert from clip space [-1, 1] to screen space coordinates
        // The 2D renderer uses normalized coordinates where:
        // - (0, 0) is the center of the screen
        // - x ranges from approximately -1 to 1
        // - y ranges from approximately -1 to 1
        float screenX = clipPos.x;
        float screenY = clipPos.y;
        
        // Offset the label slightly above and to the right of the marker
        // to avoid overlapping with the marker itself
        screenX += 0.05f;
        screenY += 0.05f;
        
        // Render the text label with white color and slight transparency
        // Use a small font size appropriate for labels
        float labelSize = 0.015f;
        renderer2d->drawText(screenX, screenY, text, labelSize, 
                           1.0f, 1.0f, 1.0f, 0.9f, true, Renderer::LEFT);
    }
    
}
