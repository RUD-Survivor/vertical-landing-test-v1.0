# Vertical-Lander-GNC 

A C++ based physics simulation and flight control system for vertical rocket landing, inspired by SpaceX Starship.

##  Project Overview
This project simulates the **Guidance, Navigation, and Control (GNC)** logic required to land a rocket booster. It implements:
* **Physics Engine:** Simulates gravity, atmospheric pressure, variable mass (fuel consumption), and thrust.
* **PID Control:** Uses Proportional Control loop for the "Fine Landing" phase.
* **Suicide Burn Algorithm:** Calculates real-time stopping distance for efficient deceleration.
* **State Machine:** Manages flight stages (Ascend, Coast, Suicide Burn, Soft Landing).

## 🛠️ Technology Stack
* **Language:** C++ (Standard 11/17)
* **Core Concepts:** Kinematics, Control Theory, OOP.
* **Zero Dependencies:** Pure C++ implementation, runs on any console.

## 📉 The Journey (Failures & Fixes)
Development wasn't a straight line. Here is how the algorithm evolved:

### Attempt 1: The "Too Late" Burn 
* **Issue:** Calculated braking distance without considering thrust-to-weight ratio changes due to pressure.
* **Result:** Impact at -415 m/s (RUD).

### Attempt 2: The "Oscillation" 
* **Issue:** Bang-Bang control (Full throttle / Zero throttle) caused severe oscillation near the ground.
* **Result:** Impact at -32 m/s.

### Final Success: The "Kiss Landing" 
* **Solution:** Implemented continuous throttle modulation (P-Control) targeting specific descent velocities based on altitude.
* **Result:** Touchdown velocity **-0.49 m/s**.

##  How to Run
```bash
g++ main.cpp -o starship
./starship
