#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include "rocket_state.h"

namespace ControlSystem {
    // Updates the ControlInput (throttle & torque) based on Autopilot logic
    void UpdateAutoPilot(RocketState& state, const RocketConfig& config, ControlInput& input, double dt);
    
    // Updates ControlInput based on Keyboard rules mapped to a simplified interface
    // (Actual key pressing checks will happen in project.cpp and map to bools here to keep GLFW out)
    struct ManualInputs {
        bool throttle_up;
        bool throttle_down;
        bool throttle_max;
        bool throttle_min;
        
        bool pitch_left;
        bool pitch_right;
        bool pitch_forward;
        bool pitch_backward;
    };
    
    void UpdateManualControl(RocketState& state, ControlInput& input, const ManualInputs& manual, double dt);
}

#endif // CONTROL_SYSTEM_H
