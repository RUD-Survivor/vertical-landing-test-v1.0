#include "core/app.h"
#include "physics/physics_system.h"
#include "save_system.h"
#include <iostream>
#include <chrono>
#include <thread>

namespace RocketSim {

float App::s_scrollY = 0.0f;

App::App() : m_width(1000), m_height(800) {}

App::~App() {
    Shutdown();
}

bool App::Init(int width, int height, const std::string& title) {
    m_width = width;
    m_height = height;

    if (!glfwInit()) return false;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    m_window = glfwCreateWindow(m_width, m_height, title.c_str(), NULL, NULL);
    if (!m_window) {
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_window);
    glfwSetWindowUserPointer(m_window, this);
    glfwSetFramebufferSizeCallback(m_window, FramebufferSizeCallback);
    glfwSetScrollCallback(m_window, ScrollCallback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) return false;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_MULTISAMPLE);

    m_renderer2d = std::make_unique<Renderer>();
    // m_renderer3d will be initialized per-phase or globally as needed
    // In original main.cpp, r3d was created before flight/builder phase

    PhysicsSystem::InitSolarSystem();
    m_orbitPredictor.Start();

    return true;
}

void App::Run() {
    while (!glfwWindowShouldClose(m_window) && !m_shouldClose) {
        // High-level State Machine 
        // 1. Menu Phase
        HandleMenu();
        
        if (m_shouldClose || glfwWindowShouldClose(m_window)) break;

        // 2. Hub / Factory Phase
        HandleAgencyHub();

        if (m_shouldClose || glfwWindowShouldClose(m_window)) break;

        // 3. VAB Phase
        HandleVAB();

        if (m_shouldClose || glfwWindowShouldClose(m_window)) break;

        // 4. Flight Phase
        HandleFlight();
    }
}

void App::Shutdown() {
    if (m_window) {
        glfwDestroyWindow(m_window);
        glfwTerminate();
        m_window = nullptr;
    }
}

void App::HandleMenu() {
    using namespace MenuSystem;
    m_menuState.has_save = SaveSystem::HasSaveFile() || SaveSystem::HasAgencySave();
    if (SaveSystem::HasSaveFile()) {
        SaveSystem::GetSaveInfo(m_menuState.save_time, m_menuState.save_parts);
    }

    m_currentChoice = MENU_NONE;
    while (!glfwWindowShouldClose(m_window) && m_currentChoice == MENU_NONE) {
        glfwPollEvents();
        
        double mx, my;
        glfwGetCursorPos(m_window, &mx, &my);
        float mouse_x = (float)(mx / m_width * 2.0 - 1.0);
        float mouse_y = (float)(1.0 - my / m_height * 2.0);
        bool mouse_left = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;

        bool up = false, down = false, enter = false; // Dummy for now
        m_currentChoice = HandleMenuInput(m_window, m_menuState, up, down, enter, mouse_x, mouse_y, mouse_left);

        if (m_currentChoice == MENU_EXIT) {
            m_shouldClose = true;
            return;
        }

        glClearColor(0.02f, 0.03f, 0.08f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        m_renderer2d->beginFrame();
        DrawMainMenu(m_renderer2d.get(), m_menuState, (float)glfwGetTime());
        m_renderer2d->endFrame();
        glfwSwapBuffers(m_window);
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
        
        if (m_currentChoice != MENU_NONE && m_currentChoice != MENU_SETTINGS) break; 
    }

    // New Game / Continue transition
    if (m_currentChoice == MENU_NEW_GAME) {
        if (SaveSystem::HasSaveFile()) SaveSystem::DeleteSaveFile();
        m_agencyState = AgencyState();
        m_factory = FactorySystem();
        // Starter resources (copied from main.cpp)
        m_agencyState.funds = 100000.0;
        m_agencyState.addItem(ITEM_IRON_ORE, 50);
        m_agencyState.addItem(ITEM_COPPER_ORE, 30);
        m_agencyState.addItem(ITEM_COAL, 40);
        m_agencyState.addItem(ITEM_STEEL, 10);
        m_agencyState.addItem(PART_ENGINE, 2);
        m_agencyState.addItem(PART_FUEL_TANK, 4);
        m_agencyState.addItem(PART_NOSECONE, 2);
        m_agencyState.addItem(PART_STRUCTURAL, 5);
        m_agencyState.addItem(PART_COMMAND_POD, 1);
        m_currentChoice = MENU_AGENCY_HUB;
    } else if (m_currentChoice == MENU_CONTINUE) {
        if (SaveSystem::HasAgencySave()) {
            SaveSystem::LoadAgencyFactory(m_agencyState, m_factory);
        }
    }
}

void App::HandleAgencyHub() {
    using namespace MenuSystem;
    while (m_currentChoice == MENU_AGENCY_HUB && !glfwWindowShouldClose(m_window)) {
        glfwPollEvents();
        double mx, my; glfwGetCursorPos(m_window, &mx, &my);
        float mouse_x = (float)(mx / m_width * 2.0 - 1.0);
        float mouse_y = (float)(1.0 - my / m_height * 2.0);
        bool mouse_left = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;

        bool up = false, down = false, enter = false;
        m_currentChoice = HandleAgencyHubInput(m_window, m_menuState, up, down, enter, mouse_x, mouse_y, mouse_left);
        m_factory.tick(0.016f, m_agencyState);
        
        glClearColor(0.02f, 0.03f, 0.08f, 1.0f); glClear(GL_COLOR_BUFFER_BIT);
        m_renderer2d->beginFrame();
        DrawAgencyHub(m_renderer2d.get(), m_menuState, m_agencyState, (float)glfwGetTime(), m_factory);
        m_renderer2d->endFrame();
        glfwSwapBuffers(m_window);
        std::this_thread::sleep_for(std::chrono::milliseconds(16));

        if (m_currentChoice == MENU_FACTORY) {
            HandleFactory();
            m_currentChoice = MENU_AGENCY_HUB;
        }
    }
    SaveSystem::SaveAgencyFactory(m_agencyState, m_factory);
}

void App::HandleFactory() {
    FactoryUIState factory_ui;
    s_scrollY = 0.0f; 
    bool in_factory = true;
    while (in_factory && !glfwWindowShouldClose(m_window)) {
        glfwPollEvents();
        double fmx, fmy; glfwGetCursorPos(m_window, &fmx, &fmy);
        float mouse_x = (float)(fmx / m_width * 2.0 - 1.0);
        float mouse_y = (float)(1.0 - fmy / m_height * 2.0);
        bool mouse_left = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        float scroll = s_scrollY; s_scrollY = 0.0f;

        in_factory = HandleFactoryInput(m_window, factory_ui, m_factory, m_agencyState, mouse_x, mouse_y, mouse_left, scroll);
        m_factory.tick(0.016f, m_agencyState);
        
        glClearColor(0.04f, 0.05f, 0.07f, 1.0f); glClear(GL_COLOR_BUFFER_BIT);
        m_renderer2d->beginFrame(); 
        DrawFactoryUI(m_renderer2d.get(), factory_ui, m_factory, m_agencyState, (float)glfwGetTime()); 
        m_renderer2d->endFrame();
        glfwSwapBuffers(m_window); 
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
}

void App::HandleVAB() {
    // Implementation will follow from main.cpp logic
}

void App::HandleFlight() {
    // Implementation will follow from main.cpp logic
}

void App::FramebufferSizeCallback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
    App* app = (App*)glfwGetWindowUserPointer(window);
    if (app) {
        app->m_width = width;
        app->m_height = height;
    }
}

void App::ScrollCallback(GLFWwindow* /*window*/, double /*xoffset*/, double yoffset) {
    s_scrollY += (float)yoffset;
}

} // namespace RocketSim
