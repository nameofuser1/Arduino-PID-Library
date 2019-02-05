/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * Modified by Kamnev Yuriy <kamnev.u1969@gmail.com>
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#include "pid.hpp"


PID::PID(pid_config_t &cfg) :
    inAuto(false),
    last_input(0),
    integration_sum(0),
    setpoint(false)
{
    config = cfg;
}


/**
 * @brief       Compute PID output
 *
 * @param[in]   input   -   currently measured input
 * @param[in]   dinput  -   currently measured input derivative
 *
 * @return      Output signal
 */
double PID::compute(double input, double dinput)
{
    /*Compute all the working error variables*/
    double output = compute_output(input, dinput);
    return output;
}


/**
 * @brief       Compute PID output.
 *              In this function derivative of the input is approximated by first difference.
 *
 * @param[in]   input   -   currently measured input
 *
 * @return      Output signal
 */
double PID::compute(double input)
{
    double dinput = (input - last_input);
    return compute_output(input, dinput);
}


/**
 * @brief       Update setpoing
 *
 * @param[in]   _setpoint   -   new setpoint
 */
void PID::update_setpoint(double _setpoint)
{
    setpoint = _setpoint;
}


/**
 * @brief       Compute PID output
 *
 * @param[in]   input   -   currently measured input
 * @param[in]   dinput  -   currently measured input derivative
 *
 * @return      Output signal
 */
double PID::compute_output(double input, double dinput)
{
    double error = (setpoint - input);
    integration_sum += (config.Ki * error);
    if(integration_sum > config.out_max)
    {
        integration_sum = config.out_max;
    }
    else if(integration_sum < config.out_min)
    {
        integration_sum = config.out_min;
    }

    double output = config.Kp*error - config.Kd*dinput + integration_sum;

    if(output > config.out_max)
    {
        output = config.out_max;
    }
    else if(output < config.out_min)
    {
        output = config.out_min;
    }

    last_input = input;
    return output;
}


/**
 *  @brief      Update PID coefficients
 *
 *  @param[in]  Kp  -   proportional coefficient
 *  @param[in]  Ki  -   integration coefficient
 *  @param[in]  Kd  -   differential coefficient
 */
void PID::set_tunings(double Kp, double Ki, double Kd)
{
    if(config.ControllerDirection == PID_BACKWARD)
    {
        Kp *= -1;
        Ki *= -1;
        Kd *= -1;
    }

    config.Kp = Kp;
    config.Kd = Kd;
    config.Ki = Ki;
}



/**
 * @brief       Update PID output limits
 *
 * @param[in]   min - minimum output
 * @param[in]   max - maximum output
 *
 */
void PID::set_limits(double min, double max)
{
    if(min >= max)
    {
        return;
    }

    config.out_min = min;
    config.out_max = max;
}





/**
 * @brief       Set controller direction
 *
 * @param[in]   dir     -   new direction
 */
void PID::set_direction(pid_direction_t dir)
{
    if(dir != config.ControllerDirection)
    {
        config.Kp *= -1;
        config.Ki *= -1;
        config.Kd *= -1;

        config.ControllerDirection = dir;
    }
}


/**
 * @brief       Retrieve current config
 *
 * @param[in]   cfg     -   reference to where to store config
 */
void PID::get_config(pid_config_t &cfg)
{
    cfg = config;
}
