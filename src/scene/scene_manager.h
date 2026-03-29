#pragma once
#include <memory>
#include "scene.h"

/**
 * 场景管理器 (SceneManager)
 * 职责：存储当前活动的 Scene，并且在不立即销毁当前帧环境的情况下安全切换场景。
 */
class SceneManager {
public:
    static SceneManager& getInstance() {
        static SceneManager instance;
        return instance;
    }

    // 请求切换到新场景，会在下一帧 update 时正式切换
    void changeScene(std::unique_ptr<IScene> newScene) {
        nextScene = std::move(newScene);
    }
    
    // 主循环驱动
    void update(double dt) {
        // 如果有挂起的场景切换请求
        if (nextScene) {
            if (currentScene) {
                currentScene->onExit();
            }
            currentScene = std::move(nextScene);
            currentScene->onEnter();
        }

        if (currentScene) {
            currentScene->update(dt);
        }
    }
    
    // 主渲染提交
    void render() {
        if (currentScene) {
            currentScene->render();
        }
    }

    IScene* getCurrentScene() const {
        return currentScene.get();
    }

private:
    SceneManager() = default;
    ~SceneManager() = default;

    std::unique_ptr<IScene> currentScene;
    std::unique_ptr<IScene> nextScene;
};
