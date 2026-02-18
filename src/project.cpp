#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm> 

using namespace std;

// ==========================================
// Part 0: 数学常量与辅助工具
// ==========================================
const double PI = 3.14159265358979323846;
const double G0 = 9.80665;              // 海平面标准重力
const double EARTH_RADIUS = 6371000.0;  // 地球半径 (米)

// 简单的二维向量工具
struct Vec2 { double x, y; };
Vec2 rotateVec(double x, double y, double angle) {
    double s = sin(angle); double c = cos(angle);
    return { x * c - y * s, x * s + y * c };
}
float my_lerp(float a, float b, float t) { return a + t * (b - a); }

// PID 控制器结构体
// 修改 Part 0 中的 PID 结构体
struct PID {
    double kp, ki, kd;
    double integral = 0;
    double prev_error = 0;

    // 增加积分限幅，防止油门锁死
    double integral_limit = 50.0;

    double update(double target, double current, double dt) {
        double error = target - current;
        integral += error * dt;

        // --- 抗饱和 (Anti-Windup) ---
        if (integral > integral_limit) integral = integral_limit;
        if (integral < -integral_limit) integral = -integral_limit;
        // ---------------------------

        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return kp * error + ki * integral + kd * derivative;
    }

    // 增加重置函数，着陆瞬间清空误差
    void reset() { integral = 0; prev_error = 0; }
};

// ==========================================
// Part 1: 现代 OpenGL 渲染引擎 (升级版：支持旋转)
// ==========================================
const char* vertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec2 aPos;
    layout (location = 1) in vec3 aColor;
    out vec3 ourColor;
    void main() {
        gl_Position = vec4(aPos.x, aPos.y, 0.0, 1.0);
        ourColor = aColor;
    }
)";

const char* fragmentShaderSource = R"(
    #version 330 core
    out vec4 FragColor;
    in vec3 ourColor;
    void main() {
        FragColor = vec4(ourColor, 1.0);
    }
)";

class Renderer {
private:
    unsigned int shaderProgram, VBO, VAO;
    vector<float> vertices;
    unsigned int compileShader(unsigned int type, const char* source) {
        unsigned int id = glCreateShader(type);
        glShaderSource(id, 1, &source, NULL);
        glCompileShader(id);
        return id;
    }
public:
    Renderer() {
        unsigned int v = compileShader(GL_VERTEX_SHADER, vertexShaderSource);
        unsigned int f = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSource);
        shaderProgram = glCreateProgram();
        glAttachShader(shaderProgram, v); glAttachShader(shaderProgram, f);
        glLinkProgram(shaderProgram); glDeleteShader(v); glDeleteShader(f);

        glGenVertexArrays(1, &VAO); glGenBuffers(1, &VBO);
        glBindVertexArray(VAO); glBindBuffer(GL_ARRAY_BUFFER, VBO);
        // 预分配更大的显存以支持圆形地球绘制
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 20000, NULL, GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(2 * sizeof(float)));
        glEnableVertexAttribArray(1);
    }
    void beginFrame() { vertices.clear(); }
    void addVertex(float x, float y, float r, float g, float b) {
        vertices.insert(vertices.end(), { x, y, r, g, b });
    }
    // 普通矩形
    void addRect(float x, float y, float w, float h, float r, float g, float b) {
        addVertex(x - w / 2, y - h / 2, r, g, b); addVertex(x + w / 2, y - h / 2, r, g, b); addVertex(x + w / 2, y + h / 2, r, g, b);
        addVertex(x + w / 2, y + h / 2, r, g, b); addVertex(x - w / 2, y + h / 2, r, g, b); addVertex(x - w / 2, y - h / 2, r, g, b);
    }
    // 【新增】旋转矩形
    void addRotatedRect(float cx, float cy, float w, float h, float angle, float r, float g, float b) {
        Vec2 p1 = { -w / 2, -h / 2 }, p2 = { w / 2, -h / 2 }, p3 = { w / 2, h / 2 }, p4 = { -w / 2, h / 2 };
        p1 = rotateVec(p1.x, p1.y, -angle); p2 = rotateVec(p2.x, p2.y, -angle);
        p3 = rotateVec(p3.x, p3.y, -angle); p4 = rotateVec(p4.x, p4.y, -angle);
        addVertex(cx + p1.x, cy + p1.y, r, g, b); addVertex(cx + p2.x, cy + p2.y, r, g, b); addVertex(cx + p3.x, cy + p3.y, r, g, b);
        addVertex(cx + p3.x, cy + p3.y, r, g, b); addVertex(cx + p4.x, cy + p4.y, r, g, b); addVertex(cx + p1.x, cy + p1.y, r, g, b);
    }
    // 【新增】旋转三角形
    void addRotatedTri(float cx, float cy, float w, float h, float angle, float r, float g, float b) {
        Vec2 p1 = { 0, h / 2 }, p2 = { -w / 2, -h / 2 }, p3 = { w / 2, -h / 2 };
        p1 = rotateVec(p1.x, p1.y, -angle); p2 = rotateVec(p2.x, p2.y, -angle); p3 = rotateVec(p3.x, p3.y, -angle);
        addVertex(cx + p1.x, cy + p1.y, r, g, b); addVertex(cx + p2.x, cy + p2.y, r, g, b); addVertex(cx + p3.x, cy + p3.y, r, g, b);
    }
    void endFrame() {
        if (vertices.empty()) return;
        glUseProgram(shaderProgram); glBindVertexArray(VAO); glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(float), vertices.data());
        glDrawArrays(GL_TRIANGLES, 0, vertices.size() / 5);
    }
    ~Renderer() { glDeleteVertexArrays(1, &VAO); glDeleteBuffers(1, &VBO); glDeleteProgram(shaderProgram); }
};

// ==========================================
// Part 2: Explorer 类 (集成 2D 物理与控制)
// ==========================================
const double SLP = 1013.25;

class Explorer
{
private:
    // --- 原有参数保留 ---
    double fuel;
    double mass; // 干重
    double diameter;
    double height;
    bool suicide_burn_locked = false;
    double altitude; // 兼容旧逻辑：代表海拔高度
    double velocity; // 兼容旧逻辑：代表垂直速度 (径向速度)
    double specific_impulse;
    double fuel_consumption_rate; // 当前油门下的消耗率
    double thrust_power;
    double acceleration;
    double cosrate; // 满油门消耗率
    double nozzle_area;

    // --- 新增 2D 物理参数 ---
    // 坐标系：地心为原点 (0,0)。发射点(北极)为 (0, EARTH_RADIUS)
public:
    double px, py;       // 2D 位置
    double vx, vy;       // 2D 速度
    double angle;        // 姿态角 (0=垂直向上/径向, 正=向左倾斜, 负=向右倾斜)
    double ang_vel;      // 角速度
    double throttle;     // 油门 (0.0 - 1.0)
    double torque_cmd;   // 力矩指令

    // PID 控制器实例
    PID pid_vert = { 0.5, 0.001, 1.2 };  // 高度环
    PID pid_pos = { 0.001, 0.0, 0.2 };  // 位置环 (参数很小因为距离很大)
    PID pid_att = { 40000.0, 0.0, 100000.0 };   // 姿态环 (需要快速响应)

public:
    enum State { PRE_LAUNCH, ASCEND, DESCEND, LANDED, CRASHED } status;
    int stages;

    Explorer(double fuel, double mass, double diameter, double height, int stages, double specific_impulse, double fuel_consumption_rate, double nozzle_area)
    {
        // 2D 初始化：放在地球北极
        px = 0;
        py = EARTH_RADIUS;
        vx = 0; vy = 0;
        angle = 0; ang_vel = 0;

        // 兼容旧变量初始化
        altitude = 0;
        velocity = 0;

        this->fuel = fuel;
        this->mass = mass;
        this->diameter = diameter;
        this->height = height;
        this->stages = stages;
        this->specific_impulse = specific_impulse;
        // 注意：传入的 fuel_consumption_rate 视为“满油门消耗率”
        this->cosrate = fuel_consumption_rate;
        this->nozzle_area = nozzle_area;

        this->fuel_consumption_rate = 0; // 初始油门为0
        this->thrust_power = 0;
        this->throttle = 0;
        this->torque_cmd = 0;

        status = PRE_LAUNCH;
    }

    // --- Getters ---
    double getAltitude() const { return altitude; }
    double getThrust() const { return thrust_power; }
    double getVelocityMag() const { return sqrt(vx * vx + vy * vy); }

    // --- 物理计算 ---
    // 1. 重力随高度变化 (平方反比定律)
    double get_gravity(double r) {
        return G0 * pow(EARTH_RADIUS / r, 2);
    }

    // 2. 气压/密度模型
    double get_pressure(double h) {
        if (h > 100000) return 0;
        if (h < 0) return SLP;
        return SLP * exp(-h / 7000.0); // 简化指数大气
    }
    double get_air_density(double h) {
        if (h > 100000) return 0;
        return 1.225 * exp(-h / 7000.0);
    }

    void Report_Status()
    {
        // 增加水平数据的显示
        cout << "----------------------------------" << endl;
        cout << "[Alt]: " << altitude << " m | [Vert_Vel]: " << velocity << " m/s" << endl;
        cout << "[Pos_X]: " << px << " m | [Horz_Vel]: " << vx << " m/s" << endl;
        cout << "[Angle]: " << angle * 180.0 / PI << " deg | [Throttle]: " << throttle * 100 << "%" << endl;
        cout << "[Thrust]: " << thrust_power / 1000 << " kN | [Fuel]: " << fuel << " kg" << endl;
        cout << "[Status]: " << status << endl;
    }

    // 核心物理引擎 (2D 矢量版)
    void Burn(double dt)
    {
        if (status == PRE_LAUNCH) {
            px = 0; py = EARTH_RADIUS; vx = 0; vy = 0; altitude = 0;
            return;
        }
        if (status == LANDED || status == CRASHED) return;

        // A. 基础状态计算
        double r = sqrt(px * px + py * py); // 距地心距离
        altitude = r - EARTH_RADIUS;    // 更新旧变量

        // B. 受力分析
        double total_mass = mass + fuel;

        // 1. 重力 (万有引力，指向地心)
        double g_curr = get_gravity(r);
        double Fg_x = -g_curr * (px / r) * total_mass;
        double Fg_y = -g_curr * (py / r) * total_mass;

        // 2. 推力 (基于角度和油门)
        thrust_power = 0;
        if (fuel > 0) {
            // 计算推力 (真空推力 - 背压损失)
            double max_thrust = specific_impulse * G0 * cosrate;
            double pressure_loss = 100 * get_pressure(altitude) * nozzle_area;
            double current_thrust = throttle * max_thrust - pressure_loss;
            if (current_thrust < 0) current_thrust = 0;
            thrust_power = current_thrust;

            // 消耗燃料
            double m_dot = thrust_power / (specific_impulse * G0);
            fuel -= m_dot * dt;
            fuel_consumption_rate = m_dot; // 记录当前消耗率供显示
        }
        else {
            thrust_power = 0;
            fuel_consumption_rate = 0;
        }

        // 推力方向：假设 angle=0 是沿径向向上。
        // 计算局部垂直方向 (Local Vertical) 的角度
        double local_up_angle = atan2(py, px);
        // 实际推力方向 = 局部垂直 + 火箭偏角
        double thrust_dir = local_up_angle + angle;

        double Ft_x = thrust_power * cos(thrust_dir);
        double Ft_y = thrust_power * sin(thrust_dir);

        // 3. 空气阻力 & 气动扭矩
        double v_sq = vx * vx + vy * vy;
        double v_mag = sqrt(v_sq);
        double Fd_x = 0, Fd_y = 0;
        double aero_torque = 0;

        if (v_mag > 0.1 && altitude < 80000) {
            double rho = get_air_density(altitude);

            // 阻力 (F = 0.5 * rho * v^2 * Cd * A)
            // 假设 Cd = 0.5, A = 10 (简化)
            double drag_mag = 0.5 * rho * v_sq * 0.5 * 10.0;
            Fd_x = -drag_mag * (vx / v_mag);
            Fd_y = -drag_mag * (vy / v_mag);

            // 气动扭矩 (风向标效应)
            // 攻角 Alpha = 火箭指向 - 速度方向
            double vel_angle = atan2(vy, vx);
            // 这里比较复杂，简化为：由于 angle 是相对于局部垂直的，我们需要绝对角度
            // 绝对火箭角 = local_up_angle + angle
            double rocket_abs_angle = local_up_angle + angle;
            double alpha = rocket_abs_angle - vel_angle;

            // 归一化到 -PI ~ PI
            while (alpha > PI) alpha -= 2 * PI;
            while (alpha < -PI) alpha += 2 * PI;

            // 恢复力矩：屁股朝下飞时(alpha接近PI)，如果不稳定会翻转
            // 我们假设有栅格翼，提供强大的稳定性 (Stability Factor)
            // 扭矩尝试减小 alpha
            double stability = 10.0; // 稳定性系数
            // 特殊情况：如果倒飞 (alpha ~ 180度)，我们需要它稳定在倒飞状态
            // 简化处理：仅仅施加阻尼和控制力矩，假设栅格翼工作良好

            // 这里的扭矩我们主要模拟“旋转阻力”，防止转个不停
            aero_torque -= ang_vel * 0.1 * v_mag * rho;
        }

        // C. 积分 (牛顿第二定律)
        // 线运动
        double ax = (Fg_x + Ft_x + Fd_x) / total_mass;
        double ay = (Fg_y + Ft_y + Fd_y) / total_mass;

        vx += ax * dt;
        vy += ay * dt;
        px += vx * dt;
        py += vy * dt;

        // 角运动 (I * alpha = Torque)
        double moment_of_inertia = 50000.0; // 假定转动惯量
        double ang_accel = (torque_cmd + aero_torque) / moment_of_inertia;
        ang_vel += ang_accel * dt;
        angle += ang_vel * dt;

        // D. 更新兼容旧变量
        // 垂直速度 = 速度在局部垂直方向的投影
        velocity = vx * cos(local_up_angle) + vy * sin(local_up_angle);
        acceleration = sqrt(ax * ax + ay * ay); // 标量加速度

        // E. 碰撞检测 (双重保险版)
        double current_r = sqrt(px * px + py * py);
        double current_alt = current_r - EARTH_RADIUS;

        if (current_alt <= 0.0) {
            // 撞地了，但要看看是不是在发射
            if (status == ASCEND) {
                // 如果是上升状态，说明只是推力还没把火箭推起来 (TWR < 1)
                // 此时强制锁在地面，不判输
                px = 0; py = EARTH_RADIUS;
                vx = 0; vy = 0;
                altitude = 0;
                // 不改变 status，继续积攒推力
            }
            else if (velocity < 0.1) {
                // 只有非上升状态，且没有明显向上速度时，才算着陆/坠毁
                altitude = 0; px = 0; py = EARTH_RADIUS;

                if (status != PRE_LAUNCH) {
                    if (abs(velocity) > 10 || abs(vx) > 10) {
                        status = CRASHED;
                        cout << ">> CRASH! Impact Vel: " << velocity << endl;
                    }
                    else {
                        status = LANDED;
                        cout << ">> LANDED! Smoothly." << endl;
                    }
                    vx = 0; vy = 0; throttle = 0; ang_vel = 0; angle = 0;
                    suicide_burn_locked = false; // 重置标志位
                }
            }
            else {
                
                altitude = current_alt;
            }
        }
        else {
            altitude = current_alt;
        }
    }

    bool is_Flying() {
        return(status == PRE_LAUNCH || status == ASCEND || status == DESCEND);
    }
    bool is_IntoSpace() { return (altitude > 100000); }
    bool is_IntoOrbit() {
    
        return(altitude > 100000 && abs(vx) > 7000);
    }

  
    // --- 完美着陆版 AutoPilot (动态倾角限制 + 效率预判) ---
    void AutoPilot(double dt)
    {
        if (status == PRE_LAUNCH || status == LANDED || status == CRASHED) return;

        // 1. 获取物理参数
        double max_thrust_vac = specific_impulse * G0 * cosrate;
        double current_total_mass = mass + fuel;
        double current_g = get_gravity(sqrt(px * px + py * py));

        // 关键改动 A：计算刹车距离时，假设引擎效率打折 (0.75)
        // 因为我们预判接下来会大幅侧身，垂直分力不足 100%
        double conservative_thrust = max_thrust_vac * 0.75;
        double max_accel_avail = (conservative_thrust / current_total_mass) - current_g;

        // 2. 状态机逻辑

        // --- 上升阶段 (保持不变) ---
        if (status == ASCEND) {
            throttle = 1.0;
            double launch_tilt = -0.3; // 17度发射
            torque_cmd = pid_att.update(launch_tilt, angle, dt);

            if (altitude > 10000 || velocity < -10) {
                status = DESCEND;
                pid_vert.reset();
            }
            return;
        }

        // --- 下落阶段 ---
        status = DESCEND;

        // 计算刹车距离
        double stop_dist = 0;
        if (velocity < 0) {
            // 使用打折后的加速度计算，这会让火箭更早点火！
            stop_dist = (velocity * velocity) / (2 * max_accel_avail);
        }

        // 触发逻辑
        bool need_burn = false;
        if (suicide_burn_locked) need_burn = true;
        // 安全系数可以回调到 1.1 了，因为 max_accel 已经很保守了
        else if (altitude < stop_dist * 1.1 + 30.0 && velocity < -10) {
            suicide_burn_locked = true;
            need_burn = true;
            cout << ">> [AUTOPILOT] IGNITION! Dist: " << altitude << " (StopDist: " << stop_dist << ")" << endl;
        }

        if (!need_burn) {
            throttle = 0;
            // 自由落体时预瞄准：屁股对准速度反方向
            double vel_angle = atan2(vy, vx);
            double align_angle = (vel_angle + PI) - atan2(py, px);
            while (align_angle > PI) align_angle -= 2 * PI;
            while (align_angle < -PI) align_angle += 2 * PI;
            torque_cmd = pid_att.update(align_angle, angle, dt);
        }
        else {
            // === 核心着陆逻辑 ===

            // --- 1. 垂直油门 (能量制导) ---
            if (altitude > 10.0) {
                double target_depth = 2.0;
                double required_decel = (velocity * velocity) / (2.0 * (altitude + target_depth));
                // 这里恢复用 100% 推力去执行，如果还需要更猛，PID会补救
                double required_thrust = current_total_mass * (current_g * 1.05 + required_decel);
                throttle = required_thrust / max_thrust_vac;
            }
            else {
                // 末端软着陆
                double target_vel = -2.0;
                if (altitude < 0.2) { throttle = 0; target_vel = 0; }
                else {
                    double hover_throttle = (current_total_mass * current_g) / max_thrust_vac;
                    double error = target_vel - velocity;
                    throttle = hover_throttle + (error * 2.0);
                }
            }

            // --- 2. 水平姿态 (动态限制) ---
            double target_angle = 0;

            if (altitude > 1.0) {
                // 计算逆向角
                double vel_angle = atan2(vy, vx);
                double raw_target = (vel_angle + PI) - atan2(py, px);
                while (raw_target > PI) raw_target -= 2 * PI;
                while (raw_target < -PI) raw_target += 2 * PI;

                // 关键改动 B：动态最大倾角 (Linear Tapering)
                // 高度 200m 以上：允许 45度 (0.8)
                // 高度 0m：允许 0度
                // 中间线性过渡
                double max_tilt_limit = 0.8; // 默认最大
                if (altitude < 200.0) {
                    // 随着高度降低，像漏斗一样收紧角度限制
                    // alt=200 -> limit=0.8
                    // alt=0   -> limit=0.0
                    max_tilt_limit = (altitude / 200.0) * 0.8;
                }

                // 确保至少留一点点控制权 (5度)，否则没法最后修正
                if (max_tilt_limit < 0.08 && altitude > 2.0) max_tilt_limit = 0.08;

                // 应用限制
                target_angle = raw_target;
                if (target_angle > max_tilt_limit) target_angle = max_tilt_limit;
                if (target_angle < -max_tilt_limit) target_angle = -max_tilt_limit;

                // 速度极小时回正
                if (abs(vx) < 0.2) target_angle = 0;

                // 推力补偿
                if (abs(throttle) > 0.01) throttle /= max(0.5, cos(target_angle));
            }
            else {
                target_angle = 0;
            }

            torque_cmd = pid_att.update(target_angle, angle, dt);

            if (throttle > 1.0) throttle = 1.0;
            if (throttle < 0.0) throttle = 0.0;
        }
    } 
    

    // 手动发射
    void ManualLaunch() { if (status == PRE_LAUNCH) { status = ASCEND; } }
};

// ==========================================
// Part 3: 主函数 
// ==========================================

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

Renderer* renderer;

int main()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(1000, 800, "2D Rocket Suicide Burn Sim", NULL, NULL);
    if (!window) { glfwTerminate(); return -1; }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) return -1;
    renderer = new Renderer();

    // 初始化：放在地球北极，干重10吨，燃料50吨
    Explorer baba1(50000, 10000, 3.7, 50, 1, 300, 500, 0.5);

    cout << ">> SYSTEM READY." << endl;
    cout << ">> PRESS [SPACE] TO LAUNCH!" << endl;
    cout << ">> SIT BACK AND WATCH THE AUTO-LANDING." << endl;

    double dt = 0.02; // 50Hz 物理步长

    while (baba1.is_Flying() && !glfwWindowShouldClose(window))
    {
        glfwPollEvents();
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, true);
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
            baba1.ManualLaunch();

        // --- 物理更新 (多次迭代增加稳定性) ---
        // 自动驾驶和物理计算分离，PID计算可以慢一点，物理积分要快
        baba1.AutoPilot(dt);
        baba1.Burn(dt);

        // 只有每隔一定帧数才打印，防止控制台看不清
        static int frame = 0;
        if (frame++ % 10 == 0) baba1.Report_Status();

        this_thread::sleep_for(chrono::milliseconds(20)); // 限制帧率

        // --- 渲染 ---
        // 天空颜色随高度变化
        float t = (float)min(baba1.getAltitude() / 50000.0, 1.0);
        float r = my_lerp(0.5f, 0.0f, t);
        float g = my_lerp(0.7f, 0.0f, t);
        float b = my_lerp(1.0f, 0.0f, t);
        glClearColor(r, g, b, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        renderer->beginFrame();

        // 坐标转换：世界坐标 -> 屏幕坐标
        // 相机始终对准火箭，但火箭保持在屏幕下方 1/3 处
        // 随着高度增加，视野缩放 (Zoom out)
        double scale = 0.005; // 基础缩放 
        if (baba1.getAltitude() > 2000) scale = 0.001; // 高空视野变大

        float cx = 0.0f; // 屏幕中心 X
        float cy = -0.4f; // 屏幕中心 Y 偏下

        auto toScreenX = [&](double wx) { return (float)((wx - baba1.px) * scale + cx); };
        auto toScreenY = [&](double wy) { return (float)((wy - baba1.py) * scale + cy); };

        // 1. 画地面 (一段巨大的圆弧，模拟地球表面)
        // 在局部范围内，可以用一堆矩形或者一个大矩形模拟
        // 这里简单画一个跟随 x 坐标的大绿色块
        renderer->addRect(toScreenX(baba1.px), toScreenY(EARTH_RADIUS) - 200.0f, 1000.0f, 400.0f, 0.2f, 0.6f, 0.2f);
        // 发射台标记
        renderer->addRect(toScreenX(0), toScreenY(EARTH_RADIUS), 0.1f, 0.05f, 0.8f, 0.8f, 0.8f);

        // 2. 画火箭 (旋转)
        float w = 0.1f; float h = 0.4f;
        // 注意：addRotatedRect 需要屏幕坐标，且角度是火箭的 angle
        renderer->addRotatedRect(cx, cy, w, h, (float)baba1.angle, 0.9f, 0.9f, 0.9f);
        // 鼻锥
        // 鼻锥位置需要根据角度手动旋转一点偏移
        Vec2 nose_offset = { 0, h / 2 + w / 2 }; // 局部偏移
        nose_offset = rotateVec(nose_offset.x, nose_offset.y, -baba1.angle);
        renderer->addRotatedTri(cx + nose_offset.x, cy + nose_offset.y, w, w, (float)baba1.angle, 1.0f, 0.2f, 0.2f);

        // 3. 画火焰
        if (baba1.getThrust() > 1000) {
            float flameLen = (baba1.getThrust() / 3000000.0) * 0.3f;
            Vec2 flame_offset = { 0, -h / 2 - flameLen / 2 };
            flame_offset = rotateVec(flame_offset.x, flame_offset.y, -baba1.angle);
            renderer->addRotatedTri(cx + flame_offset.x, cy + flame_offset.y, w * 0.8f, flameLen, (float)baba1.angle, 1.0f, 0.6f, 0.0f);
        }

        renderer->endFrame();
        glfwSwapBuffers(window);
    }

    delete renderer;
    glfwTerminate();
    if (baba1.status == Explorer::LANDED || baba1.status == Explorer::CRASHED) {
        cout << "\n>> SIMULATION ENDED. PRESS ENTER." << endl;
        cin.get();
    }
    return 0;
}