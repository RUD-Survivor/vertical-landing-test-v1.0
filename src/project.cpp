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
// Part 1: 现代 OpenGL 渲染引擎 
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
		// 预分配足够的显存
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 5000, NULL, GL_DYNAMIC_DRAW);
		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(2 * sizeof(float)));
		glEnableVertexAttribArray(1);
	}
	void beginFrame() { vertices.clear(); }
	void addVertex(float x, float y, float r, float g, float b) {
		vertices.insert(vertices.end(), { x, y, r, g, b });
	}
	// 画矩形 (中心点 x,y, 宽 w, 高 h, 颜色 r,g,b)
	void addRect(float x, float y, float w, float h, float r, float g, float b) {
		addVertex(x - w / 2, y - h / 2, r, g, b); addVertex(x + w / 2, y - h / 2, r, g, b); addVertex(x + w / 2, y + h / 2, r, g, b);
		addVertex(x + w / 2, y + h / 2, r, g, b); addVertex(x - w / 2, y + h / 2, r, g, b); addVertex(x - w / 2, y - h / 2, r, g, b);
	}
	// 画三角形 (底边中心 x,y, 宽 w, 高 h, 颜色 r,g,b)
	void addTri(float x, float y, float w, float h, float r, float g, float b) {
		addVertex(x, y + h, r, g, b); addVertex(x - w / 2, y, r, g, b); addVertex(x + w / 2, y, r, g, b);
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
// Part 2:  Explorer 类 
// ==========================================
const double GRAVITY = 9.8;
const double SLP = 1013.25; //Sea-level Pressure
class Explorer
{
private:
	double fuel;
	double mass;
	double diameter;
	double height;
	bool suicide_burn_locked = false; // 记录是否已经进入决战时刻
	double altitude;
	double velocity;
	double specific_impulse;
	double fuel_consumption_rate;
	double thrust_power;
	double acceleration;
	double cosrate;
	double nozzle_area;
public:
	enum State
	{
		PRE_LAUNCH,
		ASCEND,
		DESCEND,
		LANDED,
		CRASHED

	}status;
	int stages;
	Explorer(double fuel, double mass, double diameter, double height, int stages, double specific_impulse, double fuel_consumption_rate, double nozzle_area)
	{
		altitude = 0;
		velocity = 0;
		this->fuel = fuel;
		this->mass = mass;
		this->diameter = diameter;
		this->height = height;
		this->stages = stages;
		this->specific_impulse = specific_impulse;
		this->fuel_consumption_rate = fuel_consumption_rate;
		this->nozzle_area = nozzle_area;
		cosrate = fuel_consumption_rate;
		thrust_power = specific_impulse * GRAVITY * fuel_consumption_rate;
		status = PRE_LAUNCH;
	}

	// --- 为了绘图需要，添加两个只读接口 ---
	double getAltitude() const { return altitude; }
	double getThrust() const { return thrust_power; }
	// -------------------------------------------

	double get_pressure(double h)
	{
		if (h > 0 && h < 100000)
		{
			return SLP * pow(0.5, h / 5000);
		}
		else if (h <= 0)
		{
			return SLP;
		}
		else
		{
			return 0;
		}
	}
	void Report_Status()
	{

		cout << "----------------------------------" << endl;
		cout << "[Altitude]: " << altitude << " m" << endl;
		cout << "[Velocity]:" << velocity << " m/s" << endl;
		cout << "[Thrust_Power]:" << thrust_power / 1000 << " kN" << endl;
		cout << "[Accleration]:" << acceleration << "m/(s^2)" << endl;
		cout << "[Fuel_Mass]:" << fuel << "kg" << endl;
		cout << "[Total_Mass]:" << fuel + mass << "kg" << endl;
		cout << "[Atmospheric Pressure]:" << get_pressure(altitude) << endl;
		cout << "[estimated_impact_time]:" << altitude / abs(velocity + 0.001) << "s" << endl;
		cout << "[Thrust-to-weight ratio]:" << thrust_power / ((fuel + mass) * GRAVITY) << endl;
		cout << "[Status]: " << status << "(0.PRE_LAUNCH 1.ASCEND 2.DESCEND 3.LANDED 4.CRASHED)" << endl;
	}
	void Burn(double duration)
	{

		if (fuel <= 0)
		{
			cosrate = fuel_consumption_rate;
			cout << ">>WARNING:OUT OF FUEL! ENGINE CUTOFF." << endl;
			thrust_power = 0;

			fuel_consumption_rate = 0;
		}

		double current_mass = mass + fuel;
		thrust_power = specific_impulse * GRAVITY * fuel_consumption_rate - 100 * get_pressure(altitude) * nozzle_area;
		if (thrust_power < 0)
		{
			thrust_power = 0;
		}
		double fuel_spent = fuel_consumption_rate * duration;
		if (fuel_spent > fuel)
		{
			fuel_spent = fuel;
		}
		fuel -= fuel_spent;

		double net_force = thrust_power - (current_mass * GRAVITY);

		acceleration = net_force / current_mass;

		velocity += acceleration * duration;

		altitude += velocity * duration;

		if (altitude < 0)
		{
			altitude = 0;
			if (velocity < -10)
			{
				cout << ">> [CRITICAL FAILURE] RAPID UNSCHEDULED DISASSEMBLY!" << endl;
				cout << ">> IMPACT VELOCITY: " << velocity << " m/s. CREW LOST." << endl;
				status = CRASHED;
			}
			else
			{
				cout << ">> [SUCCESS] SMOOTH LANDING. WELCOME HOME." << endl;
				cout << ">> TOUCHDOWN VELOCITY: " << velocity << " m/s." << endl;
				status = LANDED;
			}
			velocity = 0;
			acceleration = 0;
			thrust_power = 0;
			fuel_consumption_rate = 0;
			return;

		}

	}
	bool is_Flying()
	{
		return(status == PRE_LAUNCH || status == ASCEND || status == DESCEND);
	}
	bool is_IntoSpace()
	{
		return (altitude > 100000);

	}
	bool is_IntoOrbit()
	{
		return(altitude > 100000 && velocity > 7000);
	}

	void Stage()
	{

	}

	//
	void AutoPilot()
	{
		// --- 1. 物理计算  ---

		// 强制使用海平面气压 (SLP) 来计算“保底推力”
		// 这样计算出来的刹车距离是最长的
		double min_thrust = specific_impulse * GRAVITY * cosrate - 100 * SLP * nozzle_area;
		double min_accel = min_thrust / (mass + fuel);
		double net_accel = min_accel - GRAVITY; // 净减速能力 

		// 刹车距离公式：d = v^2 / 2a
		double stop_dist = 0;
		if (velocity < 0) {
			// 增加 20% 的安全距离 (1.2)
			stop_dist = (velocity * velocity) / (2 * (net_accel + 0.001)) * 1.2;
		}

		// 悬停推力 (Hover Thrust)
		double hover_thrust = (mass + fuel) * GRAVITY;
		double hover_fuel_rate = hover_thrust / (specific_impulse * GRAVITY);

		// --- 2. 状态机决策 ---

		// A. 太空 / 上升
		if (altitude > 10000) {
			status = DESCEND;
			fuel_consumption_rate = 0;
		}
		else if (velocity > 0 && altitude < 10000) {
			status = ASCEND;
			fuel_consumption_rate = cosrate;
		}

		// B. 下降逻辑 (核心修改)
		else
		{
			status = DESCEND;

			// 阶段一：自杀点火 (Suicide Burn)
			// 只有当速度非常快，且高度进入危险区时触发
			if (suicide_burn_locked || (velocity < -30 && altitude < stop_dist))
			{
				suicide_burn_locked = true; // 锁定

				// 只有当速度真的降下来了 (比如小于 30m/s)，才解锁转入平滑着陆
				if (velocity > -30) {
					suicide_burn_locked = false; // 解锁
					cout << "[AUTOPILOT] >> HANDOFF TO FINE LANDING..." << endl;
				}
				else {
					fuel_consumption_rate = cosrate; // 焊死油门，全功率
					cout << "[AUTOPILOT] >> SUICIDE BURN! Dist Remain: " << altitude << " / Stop Dist: " << stop_dist << endl;
				}
			}

			// 阶段二：平滑着陆 (Fine Landing / PID Control)
			else if (velocity < 0) // 只要还在掉，且没触发自杀点火
			{
				// 目标速度算法：高度越低，越要慢
				// 500米 -> -52 m/s
				// 100米 -> -12 m/s
				// 0米   -> -2  m/s
				double target_velocity = -(altitude * 0.1 + 2.0);

				double error = target_velocity - velocity;
				double kp = 0.8; // 反应灵敏度

				// 基础推力是悬停，根据误差进行增减
				double throttle_cmd = hover_fuel_rate + (error * kp * 100);

				// 限制范围
				if (throttle_cmd > cosrate) throttle_cmd = cosrate;
				if (throttle_cmd < 0) throttle_cmd = 0;

				fuel_consumption_rate = throttle_cmd;

				cout << "[AUTOPILOT] FINE LANDING. Tgt: " << target_velocity << " | Real: " << velocity << endl;
			}
			else {
				// 还没开始掉，或者极其缓慢
				fuel_consumption_rate = 0;
			}
		}

		// 计算实际推力 
		thrust_power = specific_impulse * GRAVITY * fuel_consumption_rate - 100 * get_pressure(altitude) * nozzle_area;
		if (thrust_power < 0) thrust_power = 0;
	}
};

// ==========================================
// Part 3: 主函数 
// ==========================================

// 窗口大小改变时的回调
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
	glViewport(0, 0, width, height);
}

// 工具函数：线性插值颜色
float my_lerp(float a, float b, float t) {
	return a + t * (b - a);
}

int main()
{
	// --- 1. 初始化图形环境 ---
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // 使用现代核心模式

	GLFWwindow* window = glfwCreateWindow(800, 600, "NPU Rocket Flight Data Recorder", NULL, NULL);
	if (!window) { glfwTerminate(); return -1; }
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		cout << "Failed to initialize GLAD" << endl; return -1;
	}

	Renderer* renderer = new Renderer();

	// --- 2. 初始化火箭  ---
	Explorer baba1(2560000, 1000000, 10, 120, 1, 380, 10000, 20);
	cout << ">>SYSTEM STARTUP..." << endl;
	cout << ">>EXPLORER BABA1 READY FOR LAUNCH." << endl;

	double time_step = 0.05;

	// --- 3. 主循环 (图形 + 物理) ---
	while (baba1.is_Flying() && !glfwWindowShouldClose(window))
	{
		// 处理窗口事件
		glfwPollEvents();
		if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
			glfwSetWindowShouldClose(window, true);

		// --- A. 核心物理逻辑  ---
		baba1.Burn(time_step);
		baba1.Report_Status(); // 监视数据依然会在控制台刷屏

		if (baba1.is_IntoSpace())
		{
			cout << "YOU ARE INTO SPACE" << endl;
		}
		if (baba1.is_IntoOrbit())
		{
			cout << "YOU ARE INTO ORBIT" << endl;
		}
		this_thread::sleep_for(chrono::milliseconds(50));

		baba1.AutoPilot();

		// --- B. 图形渲染逻辑 ---

		// 1. 计算天空颜色 (随高度渐变)
		double alt = baba1.getAltitude();
		float t = (float)min(alt / 50000.0, 1.0); // 50km 高空完全变黑
		// 从浅蓝色 (0.5, 0.7, 1.0) 渐变到黑色 (0, 0, 0)
		float skyR = my_lerp(0.5f, 0.0f, t);
		float skyG = my_lerp(0.7f, 0.0f, t);
		float skyB = my_lerp(1.0f, 0.0f, t);
		glClearColor(skyR, skyG, skyB, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		renderer->beginFrame();

		// 2. 设置相机 (简单的垂直跟随)
		float scale = 0.002f; // 缩放比例：1屏幕单位 = 500米
		float cameraY = (float)alt;
		// 保持火箭在屏幕偏下位置
		auto toScreenY = [&](float y) {
			return (y - cameraY) * scale - 0.5f;
			};

		// 3. 画地面 (绿色大矩形)
		float groundY = toScreenY(0);
		if (groundY > -1.2f) {
			renderer->addRect(0.0f, groundY - 50.0f, 10.0f, 100.0f, 0.2f, 0.6f, 0.2f);
		}

		// 4. 火箭 (白色矩形 + 三角形鼻锥)
		float rocketScreenY = toScreenY((float)alt);
		float rocketW = 0.1f; // 屏幕上的宽度
		float rocketH = 0.4f; // 屏幕上的高度
		//箭体
		renderer->addRect(0.0f, rocketScreenY + rocketH / 2, rocketW, rocketH, 0.9f, 0.9f, 0.9f);
		//鼻锥
		renderer->addTri(0.0f, rocketScreenY + rocketH, rocketW, rocketW, 0.9f, 0.9f, 0.9f);

		// 5. 火焰 (橙色三角形，根据推力大小变化)
		double thrust = baba1.getThrust();
		if (thrust > 0) {
			// 最大推力约 37,000,000 N
			float flameH = (float)(thrust / 37000000.0) * 0.3f;
			renderer->addTri(0.0f, rocketScreenY, rocketW * 0.8f, -flameH, 1.0f, 0.5f, 0.0f);
		}

		renderer->endFrame();
		glfwSwapBuffers(window);
	}

	// 清理资源
	delete renderer;
	glfwTerminate();

	// 保持控制台窗口不立刻关闭，以便查看最后的状态
	if (baba1.status == Explorer::LANDED || baba1.status == Explorer::CRASHED) {
		cout << "\n>> SIMULATION FINISHED. PRESS ENTER TO EXIT." << endl;
		cin.get();
	}

	return 0;
}