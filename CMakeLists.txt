# 1. 声明要求的最低 CMake 版本
cmake_minimum_required(VERSION 3.10)

# 2. 定义项目名称和版本
project(RocketSim3D VERSION 1.0)

# 3. 设置 C++ 标准（推荐使用 C++17 或更高，对现代特性支持更好）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 4. 查找系统中的依赖库 (OpenGL 和 GLFW)
# 注意：你需要确保你的电脑上已经安装了 GLFW 的开发包
find_package(OpenGL REQUIRED)

# 引入 FetchContent 模块
include(FetchContent)

# 告诉 CMake 去哪里下载 GLFW
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.3.8 # 你可以指定版本
)

# 关闭 GLFW 自带的测试和例子，加快编译速度
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)

# 让 CMake 拉取并让 glfw 目标可用
FetchContent_MakeAvailable(glfw)

# ---------------------------------------------

# 5. 告诉编译器去哪里找头文件 (.h)
# 这样在代码里就可以直接写 #include "math/math3d.h" 或 #include "glad/glad.h"
include_directories(src)
include_directories(vendor)

# 6. 把所有需要编译的源文件 (.cpp 和 .c) 收集起来
# 这里不需要列出 .h 文件和 shader 文件，因为它们不直接参与编译生成对象
set(SOURCES
    src/main.cpp
    src/physics/physics_system.cpp
    src/control/control_system.cpp
    vendor/glad/glad.c
)

# 7. 告诉 CMake：我要用上面这些源文件，生成一个叫 RocketSim3D 的可执行程序
add_executable(RocketSim3D ${SOURCES})

# 8. 将第三方库链接到我们的程序上
target_link_libraries(RocketSim3D PRIVATE OpenGL::GL glfw)