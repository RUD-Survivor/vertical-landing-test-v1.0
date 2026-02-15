#include <iostream>
#include<thread>
#include<chrono>
#include<string>
#include<math.h>
using namespace std;

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
		cout << "[Status]: " << status<<"         (0.PRE_LAUNCH 1.ASCEND 2.DESCEND 3.LANDED 4.CRASHED)" << endl;
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
	/*void ExecuteCommond()
	{
		string str ;

		cout << "WAIT FOR YOUR COMMOND.(0.Continue 1.Stage 2.Cut_off 3.Burn)" << endl;
		cin >> str;
		if (str==(string) "1")
		{
			cout << ">>Stage!" << endl;
			stages -= 1;
		}
		else if (str == (string)"0")
		{

		}
		 else if (str == (string)"2")
		{
			cout << ">>WARNING: ENGINE CUTOFF." << endl;
			thrust_power = 0;
			fuel_consumption_rate = 0;
		}
		else if (str == (string)"3")
		{
			cout << ">>ENGINE BURN." << endl;
			fuel_consumption_rate = cosrate;
			thrust_power = specific_impulse * GRAVITY * fuel_consumption_rate;
		}
		else
		{
			cout << "UNDEFINED COMMOND" << endl;
		}*/
	//
	void AutoPilot()
	{
		// --- 1. 物理计算 (悲观模式) ---

		// 我们强制使用海平面气压 (SLP) 来计算“保底推力”
		// 这样计算出来的刹车距离是最长的(最安全的)
		double min_thrust = specific_impulse * GRAVITY * cosrate - 100 * SLP * nozzle_area;
		double min_accel = min_thrust / (mass + fuel);
		double net_accel = min_accel - GRAVITY; // 净减速能力 (只有 0.5g 左右)

		// 刹车距离公式：d = v^2 / 2a
		double stop_dist = 0;
		if (velocity < 0) {
			// 增加 20% 的安全距离 (1.2)，防止运算延迟导致的误差
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
				suicide_burn_locked = true; // 锁定！不到低速绝不松手

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
				// 0米   -> -2  m/s
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

		// 计算实际推力 (这是给物理引擎用的，不用改)
		thrust_power = specific_impulse * GRAVITY * fuel_consumption_rate - 100 * get_pressure(altitude) * nozzle_area;
		if (thrust_power < 0) thrust_power = 0;
	}

	
		
	
};
int main()
{
	Explorer baba1(2560000, 1000000, 10, 120, 1, 380,10000,20);
	cout << ">>SYSTEM STARTUP..." <<endl;
	cout << ">>EXPLORER BABA1 READY FOR LAUNCH." << endl;

	double time_step = 0.05;

	while (baba1.is_Flying())
	{
		baba1.Burn(time_step);
		baba1.Report_Status();

		if (baba1.is_IntoSpace())
		{
			cout << "YOU ARE INTO SPACE" << endl;
		}
		if (baba1.is_IntoOrbit())
		{
			cout << "YOU ARE INTO ORBIT" << endl;
		}
		this_thread::sleep_for(chrono::milliseconds(50));
		
		//baba1.ExecuteCommond();
		baba1.AutoPilot();
	
		
	}
	
	return 0;

}
