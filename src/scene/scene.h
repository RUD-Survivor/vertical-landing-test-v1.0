#pragma once

/**
 * IScene 接口
 * 所有的场景 (Menu, Agency, Workshop, Flight) 都必须继承这个接口。
 */
class IScene {
public:
    virtual ~IScene() = default;
    
    // 场景变活跃时调用：加载模型、绑定输入
    virtual void onEnter() {}
    
    // 场景切出时调用：释放资源、保存状态
    virtual void onExit() {}
    
    // 逻辑更新 (物理推进、输入处理)
    virtual void update(double dt) = 0;
    
    // 渲染管线提交 (2D 和 3D 渲染)
    virtual void render() = 0;
};
