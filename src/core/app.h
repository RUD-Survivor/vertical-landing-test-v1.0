#ifndef APP_H
#define APP_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <memory>
#include <string>

#include "render/renderer_2d.h"
#include "render/renderer3d.h"
#include "core/agency_state.h"
#include "simulation/factory_system.h"
#include "simulation/predictor.h"
#include "menu_system.h"

namespace RocketSim {

class App {
public:
    App();
    ~App();

    bool Init(int width, int height, const std::string& title);
    void Run();
    void Shutdown();

private:
    // Core Loop Phases
    void HandleMenu();
    void HandleAgencyHub();
    void HandleFactory();
    void HandleVAB();
    void HandleFlight();

    // Internal Helpers
    void ProcessInput();
    void Update(float dt);
    void Render();

    // GLFW Callbacks
    static void FramebufferSizeCallback(GLFWwindow* window, int width, int height);
    static void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset);

private:
    GLFWwindow* m_window = nullptr;
    int m_width, m_height;
    
    // Core Systems
    std::unique_ptr<Renderer> m_renderer2d;
    std::unique_ptr<Renderer3D> m_renderer3d;
    
    // Game State
    AgencyState m_agencyState;
    FactorySystem m_factory;
    Simulation::AsyncOrbitPredictor m_orbitPredictor;
    MenuSystem::MenuState m_menuState;
    MenuSystem::MenuChoice m_currentChoice = MenuSystem::MENU_NONE;
    
    // 3D State
    std::unique_ptr<Renderer3D> m_r3d;
    BuilderState m_builderState;
    Mesh m_rocketBody, m_rocketNose, m_rocketBox;
    
    // Control variables
    bool m_shouldClose = false;
    float m_lastFrameTime = 0.0f;
    static float s_scrollY;
};

} // namespace RocketSim

#endif // APP_H
