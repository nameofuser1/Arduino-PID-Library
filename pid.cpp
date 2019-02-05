/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#include "pid.hpp"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(pid_config_t &cfg) :
    inAuto(false),
    last_input(0),
    integration_sum(0),
    setpoint(false)
{
    config = cfg;
}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
double PID::compute(double input, double dinput)
{
    /*Compute all the working error variables*/
    double output = compute_output(input, dinput);
    return output;
}


double PID::compute(double input)
{
    double dinput = (input - last_input);
    return compute_output(input, dinput);
}


void PID::update_setpoint(double _setpoint)
{
    setpoint = _setpoint;
}


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

    /*Add Proportional on Error, if P_ON_E is specified*/
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


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
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



/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::set_limits(double min, double max)
{
    if(min >= max)
    {
        return;
    }

    config.out_min = min;
    config.out_max = max;
}





/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
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


void PID::get_config(pid_config_t &cfg)
{
    cfg = config;
}
