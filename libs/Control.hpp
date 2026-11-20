#ifndef CONTROL_HPP_
#define CONTROL_HPP_

#include "pico/stdlib.h"
#include <errno.h>
#include <cmath>
#include "CoordProcess.hpp"
#include <zsl/zsl.h>
#include "zsl/consts.h"
#include "zsl/matrices.h"
/*
* This library is where the control is being actually implemented.
* Prior to the execution of the control, 
* the data from the sensors must be collected and processed and the current position and velocity must be determined.
* This library is using namespace because the final control is not a PID and is instead a coupled state space controller.
* The point of this PID is to provide a simple and easy way to implement basic control logic.
*/
namespace Control {

    struct PIDConfig{
        float Kp;
        float Ki;
        float Kd;
        float outMin;
        float outMax;
    };

    struct PIDState{
		float integral;
		float prevError;
		float prevMeasurement;
    };

	float updatePID(PIDConfig &configuration, PIDState &state, float setpoint, float measurement, float dt);
	
	void computeMecanumOutput(float target_x, float target_y, float target_yaw,
                            		float curr_x, float curr_y, float curr_yaw, float gyro_wz,
                            		float dt, float wheel_pwm[4]);

}
#endif