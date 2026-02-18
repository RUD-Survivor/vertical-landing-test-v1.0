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
    // 旋转矩形
    void addRotatedRect(float cx, float cy, float w, float h, float angle, float r, float g, float b) {
        Vec2 p1 = { -w / 2, -h / 2 }, p2 = { w / 2, -h / 2 }, p3 = { w / 2, h / 2 }, p4 = { -w / 2, h / 2 };
        p1 = rotateVec(p1.x, p1.y, angle); p2 = rotateVec(p2.x, p2.y, angle);
        p3 = rotateVec(p3.x, p3.y, angle); p4 = rotateVec(p4.x, p4.y, angle);
        addVertex(cx + p1.x, cy + p1.y, r, g, b); addVertex(cx + p2.x, cy + p2.y, r, g, b); addVertex(cx + p3.x, cy + p3.y, r, g, b);
        addVertex(cx + p3.x, cy + p3.y, r, g, b); addVertex(cx + p4.x, cy + p4.y, r, g, b); addVertex(cx + p1.x, cy + p1.y, r, g, b);
    }
    // 旋转三角形
    void addRotatedTri(float cx, float cy, float w, float h, float angle, float r, float g, float b) {
        Vec2 p1 = { 0, h / 2 }, p2 = { -w / 2, -h / 2 }, p3 = { w / 2, -h / 2 };
        p1 = rotateVec(p1.x, p1.y, angle); p2 = rotateVec(p2.x, p2.y, angle); p3 = rotateVec(p3.x, p3.y, angle);
        addVertex(cx + p1.x, cy + p1.y, r, g, b); addVertex(cx + p2.x, cy + p2.y, r, g, b); addVertex(cx + p3.x, cy + p3.y, r, g, b);
    }
    // 画圆形地球
    void addCircle(float cx, float cy, float radius, float r, float g, float b) {
        int segments = 120; // 120个多边形拟合圆
        for (int i = 0; i < segments; i++) {
            float theta1 = 2.0f * PI * float(i) / float(segments);
            float theta2 = 2.0f * PI * float(i + 1) / float(segments);
            addVertex(cx, cy, r, g, b); // 圆心
            addVertex(cx + radius * cos(theta1), cy + radius * sin(theta1), r, g, b);
            addVertex(cx + radius * cos(theta2), cy + radius * sin(theta2), r, g, b);
        }
    }

    void addEarthWithContinents(float cx, float cy, float radius, float cam_angle) {
        int segments = 120;
        for (int i = 0; i < segments; i++) {
            // 2. 在这里加上 cam_angle 偏移！
            float theta1 = 2.0f * PI * float(i) / float(segments) + cam_angle;
            float theta2 = 2.0f * PI * float(i + 1) / float(segments) + cam_angle;

            float r, g, b;
            if ((i / 10) % 2 == 0) { r = 0.1f; g = 0.4f; b = 0.8f; } // 海洋
            else { r = 0.2f; g = 0.6f; b = 0.2f; } // 大陆

            addVertex(cx, cy, r, g, b);
            addVertex(cx + radius * cos(theta1), cy + radius * sin(theta1), r, g, b);
            addVertex(cx + radius * cos(theta2), cy + radius * sin(theta2), r, g, b);
        }
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
    
    double fuel;
    double mass; // 干重
    double diameter;
    double height;
    bool suicide_burn_locked = false;
    double altitude; // 海拔高度
    double velocity; // 垂直速度 (径向速度)
    double specific_impulse;
    double fuel_consumption_rate; // 当前油门下的消耗率
    double thrust_power;
    double acceleration;
    double cosrate; // 满油门消耗率
    double nozzle_area;

   
    // 坐标系：地心为原点 (0,0)。发射点(北极)为 (0, EARTH_RADIUS)
public:
    double local_vx;     // 真实的相对地表水平速度 (切向速度)
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
    string mission_msg = "SYSTEM READY"; // 新增：持久化任务消息
    int stages;
    int mission_phase = 0; // 记录当前太空任务阶段
    double mission_timer = 0; // 任务计时器
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

   
    double getAltitude() const { return altitude; }
    double getThrust() const { return thrust_power; }
    double getVelocityMag() const { return sqrt(vx * vx + vy * vy); }
    // 实时计算远地点(Apoapsis)和近地点(Periapsis)
    void getOrbitParams(double& apoapsis, double& periapsis) {
        double r = sqrt(px * px + py * py);
        double v_sq = vx * vx + vy * vy;
        double mu = G0 * EARTH_RADIUS * EARTH_RADIUS; // 标准引力参数

        double energy = v_sq / 2.0 - mu / r; // 轨道比能
        double h = px * vy - py * vx;        // 比角动量

        double e_sq = 1.0 + 2.0 * energy * h * h / (mu * mu); // 离心率平方
        double e = (e_sq > 0) ? sqrt(e_sq) : 0;

        if (energy >= 0) { // 逃逸轨道 (已经飞出地球引力了)
            apoapsis = 999999999;
            periapsis = (h * h / mu) / (1.0 + e) - EARTH_RADIUS;
        }
        else { // 闭合椭圆轨道
            double a = -mu / (2.0 * energy); // 半长轴
            apoapsis = a * (1.0 + e) - EARTH_RADIUS;
            periapsis = a * (1.0 - e) - EARTH_RADIUS;
        }
    }
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
        double apo, peri;
        getOrbitParams(apo, peri);
        cout << "\n----------------------------------" << endl;
        cout << ">>> [MISSION CONTROL]: " << mission_msg << " <<<" << endl;
        cout << "----------------------------------" << endl;
        cout << "[Alt]: " << altitude << " m | [Vert_Vel]: " << velocity << " m/s" << endl;
        cout << "[Pos_X]: " << px << " m | [Horz_Vel]: " << vx << " m/s" << endl;
        cout << "[Angle]: " << angle * 180.0 / PI << " deg | [Throttle]: " << throttle * 100 << "%" << endl;
        cout << "[Ground_Horz_Vel]: " << local_vx << " m/s | [Orbit_Vel]: " << getVelocityMag() << " m/s" << endl;
        cout << "[Thrust]: " << thrust_power / 1000 << " kN | [Fuel]: " << fuel << " kg" << endl;
        cout << "[Apoapsis]: " << apo / 1000.0 << " km | [Periapsis]: " << peri / 1000.0 << " km" << endl;
        cout << "[Ground_Horz_Vel]: " << local_vx << " m/s | [Orbit_Vel]: " << getVelocityMag() << " m/s" << endl;
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
        altitude = r - EARTH_RADIUS;   

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
       
   
        // 1. 真实垂直速度 (径向，正为向上)
        velocity = vx * cos(local_up_angle) + vy * sin(local_up_angle);

        // 2. 真实水平速度 (切向，正为顺时针飞行)
        // 利用向量点乘切向单位向量 (-sin, cos) 算出
        local_vx = -vx * sin(local_up_angle) + vy * cos(local_up_angle);

        acceleration = sqrt(ax * ax + ay * ay);

        // E. 碰撞检测 
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
                    if (abs(velocity) > 10 || abs(local_vx) > 10) {
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
      

        // 角运动 (I * alpha = Torque)
         moment_of_inertia = 50000.0;

        // 【物理升级】力矩不再是凭空产生的，而是推力矢量产生的
        // 假设喷管最大能偏转 5度 (0.087 rad)
        // PID 输出的 torque_cmd 现在代表“喷管偏转百分比 (-1.0 到 1.0)”

        // 如果引擎没开 (thrust_power=0)，就没有控制力矩 (RCS除外)
        // 这里模拟：主引擎推力 * 力臂(20米) * sin(喷管角度)
        double gimbal_torque = 0;
        if (thrust_power > 0) {
            // 限制 PID 输出在 -1 到 1 之间作为喷管指令
            double gimbal_cmd = max(-1.0, min(1.0, torque_cmd / 10000.0)); // 归一化
            double max_gimbal_angle = 0.1; // 约 5.7度

            // 力矩 = 推力 * sin(偏转角) * 质心距离
            gimbal_torque = thrust_power * sin(gimbal_cmd * max_gimbal_angle) * (height / 2);
        }

        // 如果想保留原来的简单逻辑，直接用下面这一行覆盖上面的 if 块：
        // double final_torque = torque_cmd; 

        // 使用简单逻辑（直接改大PID）是最快见效的：
        double final_torque = torque_cmd;

        ang_accel = (final_torque + aero_torque) / moment_of_inertia;
        ang_vel += ang_accel * dt;
        angle += ang_vel * dt;
    }

    bool is_Flying() {
        return(status == PRE_LAUNCH || status == ASCEND || status == DESCEND);
    }
    bool is_IntoSpace() { return (altitude > 100000); }
    bool is_IntoOrbit() {
    
        return(altitude > 100000 && abs(vx) > 7000);
    }

  
    // ---  AutoPilot  ---
    void AutoPilot(double dt)
    {
        if (status == PRE_LAUNCH || status == LANDED || status == CRASHED) return;

        // 1. 获取物理参数
        double max_thrust_vac = specific_impulse * G0 * cosrate;
        double current_total_mass = mass + fuel;
        double current_g = get_gravity(sqrt(px * px + py * py));

        // 因为我们预判接下来会大幅侧身，垂直分力不足 100%
        double conservative_thrust = max_thrust_vac * 0.95;
        double max_accel_avail = (conservative_thrust / current_total_mass) - current_g;

        // 2. 状态机逻辑



        if (status == ASCEND) {
            double apo, peri;
            getOrbitParams(apo, peri); // 获取实时开普勒轨道参数

            if (mission_phase == 0) {
                // 阶段 0：上升与重力转弯 (抬高远地点)
                throttle = 1.0;
                // 平滑重力转弯
                double target_pitch = 0;
                if (altitude > 1000) {
                    target_pitch = -min(1.2, (altitude - 1000) / 80000.0 * 1.57);
                }
                torque_cmd = pid_att.update(target_pitch, angle, dt);

                // 目标远地点 150km 达成，立刻关机！
                if (apo > 150000) {
                    mission_phase = 1;
                    mission_msg = "MECO! COASTING TO APOAPSIS.";
                }
            }
            else if (mission_phase == 1) {
                // 阶段 1：无动力滑行至远地点
                throttle = 0;
                torque_cmd = pid_att.update(-PI / 2.0, angle, dt); // 飞船改平

                // 当火箭爬升到接近远地点 (垂直速度极小) 时，准备圆轨
                if (altitude > 130000 && velocity < 50) {
                    mission_phase = 2;
                    mission_msg = "CIRCULARIZATION BURN STARTED!";
                }
                // 保底：如果还没到 130km 就往下掉了，强行切入圆轨
                if (velocity < -50) mission_phase = 2;
            }
            else if (mission_phase == 2) {
                // 阶段 2：远地点圆轨点火 (拉高近地点)
                throttle = 1.0;
                torque_cmd = pid_att.update(-PI / 2.0, angle, dt); // 死死按住水平方向喷射

                // 当近地点突破 140km，说明轨道完美变圆！
                if (peri > 140000) {
                    mission_phase = 3;
                    mission_timer = 0;
                    mission_msg = "ORBIT CIRCULARIZED! CRUISING.";
                }
            }
            else if (mission_phase == 3) {
                // 阶段 3：在轨巡航
                throttle = 0;
                torque_cmd = pid_att.update(-PI / 2.0, angle, dt);

                mission_timer += dt;
                // 绕轨巡航 6000 秒 
                if (mission_timer > 6000.0) {
                    mission_phase = 4;
                    mission_msg = "DE-ORBIT SEQUENCE START.";
                }
            }
            else if (mission_phase == 4) {
                // 阶段 4：脱轨点火 (降低近地点)
                double vel_angle = atan2(vy, vx);
                double align_angle = (vel_angle + PI) - atan2(py, px); // 逆向对准
                while (align_angle > PI) align_angle -= 2 * PI;
                while (align_angle < -PI) align_angle += 2 * PI;

                torque_cmd = pid_att.update(align_angle, angle, dt);

                if (abs(angle - align_angle) < 0.1) throttle = 1.0;
                else throttle = 0.0;

                // 目标脱轨轨道：让近地点砸进 30km 的浓密大气层
                if (peri < 30000) {
                    throttle = 0;
                    status = DESCEND; // 移交着陆系统
                    pid_vert.reset();
                    mission_msg = ">> RE-ENTRY BURN COMPLETE. AEROBRAKING INITIATED.";
                }
            }
            return;
        }
        // --- 下落阶段 ---
        status = DESCEND;

        // 1. 独立计算 Y 轴绝对保命推力 (最低限度抗重力 + 垂直减速)
        double req_a_y = (velocity * velocity) / (2.0 * max(1.0, altitude));
        if (velocity > 0) req_a_y = 0; // 防反弹
        double req_F_y = current_total_mass * (current_g + req_a_y);

   
        // 2. 独立计算 X 轴消除速度所需推力 (【核心升级】：时间同步刹车)
        // 让水平和垂直速度在触地时“同时”归零！
        // 假设落地时间 T = 2 * 高度 / 垂直下落速度
        // 需要的水平加速度 a_x = v_x / T = (v_x * v_y) / (2 * 高度)
        double ref_vel = max(2.0, abs(velocity));
        double req_a_x = (-local_vx * ref_vel) / (2.0 * max(1.0, altitude));
        req_a_x += -local_vx * 0.5;
        double req_F_x = current_total_mass * req_a_x;

        // 3. 矢量合成与点火判断
        double req_thrust = sqrt(req_F_x * req_F_x + req_F_y * req_F_y);
        bool need_burn = suicide_burn_locked;

        // 3. 触发逻辑：当总需求推力达到引擎上限的 95%，且高度逼近低空时，极限点火！
        if (!need_burn && req_thrust > max_thrust_vac * 0.95 && altitude < 4000) {
            suicide_burn_locked = true;
            need_burn = true;
            mission_msg = ">> SYNCHRONIZED HOVERSLAM IGNITION!";
        }

        // 4. 防悬停锁：如果空气阻力极其给力，导致需求推力甚至连重力的 80% 都不到了
        // 坚决关机，白嫖自由落体！不到低空绝不悬停！
        if (suicide_burn_locked && altitude > 100.0 && req_thrust < current_total_mass * current_g * 0.8) {
            suicide_burn_locked = false;
            need_burn = false;
            mission_msg = ">> AEROBRAKING COAST... WAITING.";
        }

        if (!need_burn) {
            throttle = 0;
            // 自由落体时，保持逆气流方向减阻
            double vel_angle = atan2(vy, vx);
            double align_angle = (vel_angle + PI) - atan2(py, px);
            while (align_angle > PI) align_angle -= 2 * PI;
            while (align_angle < -PI) align_angle += 2 * PI;
            torque_cmd = pid_att.update(align_angle, angle, dt);
        }
        else {
           

        // 理论最完美的侧滑角
            double target_angle = atan2(req_F_x, req_F_y);

            // 防撞地绝对极限倾角
            double max_safe_tilt = 1.5;
            if (req_F_y < max_thrust_vac) {
                max_safe_tilt = acos(req_F_y / max_thrust_vac);
            }
            else {
                max_safe_tilt = 0.0;
            }

            
            if (altitude < 100.0) {
                // 基础允许倾角随高度降低而逐渐减小
                double base_tilt = (altitude / 100.0) * 0.5;
                // 只要横向速度没消完，就允许借用倾角去疯狂刹车！(每1m/s侧滑借用约1.5度)
                double rescue_tilt = abs(local_vx) * 0.025;
                max_safe_tilt = min(max_safe_tilt, base_tilt + rescue_tilt);
            }

            // 只有当高度极低 (<2米) 且横向速度已经被杀干净 (<1m/s) 时，才真正立正触地！
            if (altitude < 2.0 && abs(local_vx) < 1.0) max_safe_tilt = 0.0;

            if (target_angle > max_safe_tilt) target_angle = max_safe_tilt;
            if (target_angle < -max_safe_tilt) target_angle = -max_safe_tilt;

            torque_cmd = pid_att.update(target_angle, angle, dt);

            // 最终油门计算
            if (altitude > 2.0) {
                double final_thrust = req_F_y / max(0.1, cos(target_angle));
                throttle = final_thrust / max_thrust_vac;
            }
            else {
                // 最后 2 米触地段，同样需要姿态补偿！
                double hover_throttle = (current_total_mass * current_g) / max_thrust_vac;
                throttle = hover_throttle + ((-1.5 - velocity) * 2.0);
                throttle /= max(0.8, cos(target_angle));
            }
        }

        if (throttle > 1.0) throttle = 1.0;
        if (throttle < 0.0) throttle = 0.0;
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

// 根据当前状态，计算并画出预测轨道
void drawOrbit(Renderer* renderer, double px, double py, double vx, double vy, double scale, float cx, float cy, double cam_angle) {
    double r_mag = sqrt(px * px + py * py);
    double mu = 9.80665 * 6371000.0 * 6371000.0; // 地球标准引力参数

    double h = px * vy - py * vx; // 轨道角动量
    if (abs(h) < 1.0) return;

    // 计算偏心率矢量 (e)
    double ex = (vy * h) / mu - px / r_mag;
    double ey = (-vx * h) / mu - py / r_mag;
    double e_mag = sqrt(ex * ex + ey * ey);

    double p = (h * h) / mu; // 半通径
    double omega = atan2(ey, ex); // 近地点幅角

    int segments = 200; // 轨道分段渲染
    float prev_x = 0, prev_y = 0;
    bool first = true;

    double sin_c = sin(cam_angle);
    double cos_c = cos(cam_angle);

    for (int i = 0; i <= segments; i++) {
        double nu = 2.0 * PI * i / segments; // 真近点角
        double denom = 1.0 + e_mag * cos(nu - omega);
        if (denom <= 0.05) continue; // 忽略逃逸轨道的无限远端

        double r_nu = p / denom;
        if (r_nu > EARTH_RADIUS * 5) continue; // 太远的就不画了

        double x_orb = r_nu * cos(nu);
        double y_orb = r_nu * sin(nu);

        // 旋转映射到摄像机屏幕
        double rx = x_orb * cos_c - y_orb * sin_c;
        double ry = x_orb * sin_c + y_orb * cos_c;
        float screen_x = (float)(rx * scale + cx);
        float screen_y = (float)((ry - r_mag) * scale + cy);

        if (!first) {
            // 大气层内或地下显示红色警告，安全轨道显示为黄色
            float r_col = 1.0f, g_col = 0.8f, b_col = 0.0f;
            if (r_nu < EARTH_RADIUS + 80000.0) { r_col = 1.0f; g_col = 0.0f; b_col = 0.0f; }

            // 画出预测轨迹线
            float thickness = max(0.00001f, (float)(2000.0 * scale));
            float dx = screen_x - prev_x, dy = screen_y - prev_y;
            float len = sqrt(dx * dx + dy * dy);
            if (len > 0) {
                float nx = -dy / len * thickness, ny = dx / len * thickness;
                renderer->addVertex(prev_x + nx, prev_y + ny, r_col, g_col, b_col);
                renderer->addVertex(prev_x - nx, prev_y - ny, r_col, g_col, b_col);
                renderer->addVertex(screen_x + nx, screen_y + ny, r_col, g_col, b_col);

                renderer->addVertex(screen_x + nx, screen_y + ny, r_col, g_col, b_col);
                renderer->addVertex(prev_x - nx, prev_y - ny, r_col, g_col, b_col);
                renderer->addVertex(screen_x - nx, screen_y - ny, r_col, g_col, b_col);
            }
        }
        prev_x = screen_x; prev_y = screen_y; first = false;
    }
}
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
    Explorer baba1(100000, 10000, 3.7, 50, 1, 1500, 100, 0.5);

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

        // --- 时间加速逻辑 ---
        static int time_warp = 1;
        if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS) time_warp = 1;     // 正常速度
        if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS) time_warp = 10;    // 10 倍速
        if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS) time_warp = 100;   // 100 倍速
        if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS) time_warp = 1000;  // 1000倍速 (轨道用)

        // --- 物理更新 (循环执行实现加速) ---
        for (int i = 0; i < time_warp; i++) {
            baba1.AutoPilot(dt);
            baba1.Burn(dt);

            // 如果撞地了，立刻打断时间加速，防止越界
            if (!baba1.is_Flying()) break;
        }

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
       


        // 动态无极缩放公式
        double scale = 1.0 / (baba1.getAltitude() * 1.5 + 200.0);
        float cx = 0.0f;
        float cy = 0.0f; // 火箭永远在屏幕正中央

        // 计算局部垂直角度
        double rocket_r = sqrt(baba1.px * baba1.px + baba1.py * baba1.py);
        double rocket_theta = atan2(baba1.py, baba1.px);

        // 让相机旋转，使得火箭永远朝上 (屏幕正上方)
        double cam_angle = PI / 2.0 - rocket_theta;
        double sin_c = sin(cam_angle);
        double cos_c = cos(cam_angle);

        // 带有旋转矩阵的坐标映射器
        auto toScreenX = [&](double wx, double wy) {
            double rx = wx * cos_c - wy * sin_c; // 旋转
            return (float)(rx * scale + cx);
            };
        auto toScreenY = [&](double wx, double wy) {
            double ry = wx * sin_c + wy * cos_c; // 旋转
            return (float)((ry - rocket_r) * scale + cy); // 相对火箭平移
            };

       
        renderer->addEarthWithContinents(toScreenX(0, 0), toScreenY(0, 0), EARTH_RADIUS * scale, cam_angle);
        drawOrbit(renderer, baba1.px, baba1.py, baba1.vx, baba1.vy, scale, cx, cy, cam_angle);
        // 2. 画局部平坦地表 (解决低空多边形间隙，永远在火箭正下方)
        float ground_y = (float)((EARTH_RADIUS - rocket_r) * scale + cy);
        renderer->addRect(cx, ground_y - 2000.0f * scale, 100000.0f * scale, 4000.0f * scale, 0.2f, 0.6f, 0.2f);

        // 3. 画原发射台 (注意：它现在会绕着地球转动，且自身也会倾斜)
        renderer->addRotatedRect(toScreenX(0, EARTH_RADIUS), toScreenY(0, EARTH_RADIUS),
            100.0f * scale, 50.0f * scale, (float)-cam_angle, 0.8f, 0.8f, 0.8f);

        // 4. 画火箭主体 (尺寸动态限制，防止高空缩成原子)
        float w = max(0.015f, (float)(10.0 * scale));
        float h = max(0.06f, (float)(40.0 * scale));

        // 因为宇宙已经旋转了，火箭的局部姿态角 (angle) 就可以直接用于屏幕渲染！
        renderer->addRotatedRect(cx, cy, w, h, (float)baba1.angle, 0.9f, 0.9f, 0.9f);

        // 5. 鼻锥
        Vec2 nose_offset = { 0, h / 2 + w / 2 };
        nose_offset = rotateVec(nose_offset.x, nose_offset.y, baba1.angle);
        renderer->addRotatedTri(cx + nose_offset.x, cy + nose_offset.y, w, w, (float)baba1.angle, 1.0f, 0.2f, 0.2f);

  
        // 6. 画火焰
        if (baba1.getThrust() > 1000) {
            float flameLen = (baba1.getThrust() / 3000000.0) * 0.3f;
            Vec2 flame_offset = { 0, -h / 2 - flameLen / 2 };
            flame_offset = rotateVec(flame_offset.x, flame_offset.y, baba1.angle);
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