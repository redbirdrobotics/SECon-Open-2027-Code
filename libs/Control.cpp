#include "Control.hpp"
namespace Control {
    static PIDConfig cfg_x   = { 0.75f, 0.11f, 0.0f, -0.4f, 0.4f };
    static PIDConfig cfg_y   = { 0.75f, 0.11f, 0.0f, -0.4f, 0.4f };
    static PIDConfig cfg_yaw = { 0.65f, 0.08f, 0.1f, -0.4f, 0.4f };

    static PIDState state_x   = { 0.0f, 0.0f, 0.0f };
    static PIDState state_y   = { 0.0f, 0.0f, 0.0f };
    static PIDState state_yaw = { 0.0f, 0.0f, 0.0f };
    float updatePID(PIDConfig &configuration, PIDState &state, float setpoint, float measurement, float dt)
    {
        float err = setpoint - measurement;
        //This is the P in PI
        float P = configuration.Kp * err;
        //This is the I in PI
        //This new integral is the integral (0->t) based the previous integral (0->t-dt) and the new integral (t-dt->t)
        float Theoretical_I = state.integral + 0.5f * configuration.Ki * dt * (err + state.prevError);
        float Theoretical_Output = P + Theoretical_I;
        /*
        *We now need to know if the output is too agressive.
        *If the output is too agressive, we will cap the integral and limit the output
        *and will only allow changes when the change is actually negative
        *This is to prevent integral windup.
        */
        if(Theoretical_Output > configuration.outMax)
        {
            Theoretical_Output = configuration.outMax;
            if(err < 0.0f ) state.integral = Theoretical_I;
        }
        else if(Theoretical_Output < configuration.outMin)
        {
            Theoretical_Output = configuration.outMin;
            if(err > 0.0f) state.integral = Theoretical_I;
        }
        else
        {
            state.integral = Theoretical_I;
        }
        state.prevError = err;
        state.prevMeasurement = measurement;
        return Theoretical_Output;
    }
	
    void computeMecanumOutput(float target_x, float target_y, float target_yaw, float curr_x, float curr_y, float curr_yaw, float gyro_wz, float dt, float wheel_pwm[4])
    {
        const float l = 0.1;//thse should probably be turned into a macro later on
        const float w = 0.1;//Im just trying to get a working code. not an efficient one right now
        const float sum_l_w = l+w;
        float v_x_Global = updatePID(cfg_x, state_x, target_x, curr_x, dt);
        float v_y_Global = updatePID(cfg_y, state_y, target_y, curr_y, dt);
        
        float yaw_err = target_yaw - curr_yaw;
        while(yaw_err > ZSL_PI) yaw_err -= 2*ZSL_PI;
        while(yaw_err < -ZSL_PI) yaw_err += 2*ZSL_PI;
        // Now that we know the error in the yaw, we perform its PI
        float Theoretical_Yaw_Output = updatePID(cfg_yaw, state_yaw, yaw_err, 0.0, dt);
        /*
        * The target angle is constant the entire time right now, so this part is somewhat
        * of an approximation that works very well here.
        * We will add a damper coefficient to this value
        */
        float True_Yaw_Output = Theoretical_Yaw_Output - cfg_yaw.Kd * gyro_wz;
        if(True_Yaw_Output > cfg_yaw.outMax) True_Yaw_Output = cfg_yaw.outMax;
        if(True_Yaw_Output < cfg_yaw.outMin) True_Yaw_Output = cfg_yaw.outMin;
        /*
        * From here, we will convert the required mechanics of the body
        * for the correction to the frequency at which the motors need to spin
        */
        ZSL_MATRIX_DEF(Transform_Matrix, 4, 3);
        ZSL_MATRIX_DEF(Global_Signal_Velocities, 3, 1);
        ZSL_MATRIX_DEF(Local_Signal_Velocities, 3, 1);
        ZSL_MATRIX_DEF(Output_Angular_Frequencies, 4, 1);
        CoordProcess.setup_conversion_matrices(curr_yaw);
        zsl_real_t transform_data[12] = {-sum_l_w, 1, -1, sum_l_w, 1, 1, sum_l_w, 1, -1, -sum_l_w, 1, 1};
        zsl_real_t global_signal_velocities[3] = {True_Yaw_Output, v_x_Global, v_y_Global};
        zsl_mtx_from_arr(&Transform_Matrix, transform_data);
        zsl_mtx_from_arr(&Global_Signal_Velocities, global_signal_velocities);
        zsl_mtx_mult(&CoordProcess.inverse_conversion_matrix, &Global_Signal_Velocities, &Local_Signal_Velocities);
        zsl_mtx_mult(&Transform_Matrix, &Local_Signal_Velocities, &Output_Angular_Frequencies);
        /*
        * So now we have the actual signals the motors need.
        * Now we place them into the array wheel_pwm to store them.
        * Since we are doing a linear transformation, we need to make sure the values are over 1 since the
        * PWM output is normalized (It will somewhat make more sense when we make the main file)
        */
        float maximum_normalized_output = 1.0f;
        for(int i = 0; i < 4; i++){
            wheel_pwm[i] = (float)Output_Angular_Frequencies.data[i];
            if (fabsf(wheel_pwm[i]) > maximum_normalized_output) maximum_normalized_output = fabsf(wheel_pwm[i]);
        }
        for(int i = 0; i < 4; i++){
            wheel_pwm[i] /= maximum_normalized_output;
        }
    }
}