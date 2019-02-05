/*
 * simple_example.cpp
 *
 *  Created on: Feb 5, 2019
 *      Author: Kamnev Yuriy
 */
#include "pid.hpp"
#include <vector>
#include <iostream>


int main(int argc, char **argv)
{
    PID::pid_config_t config;
    config.ControllerDirection = PID::PID_FORWARD;
    config.Kd = 1;
    config.Ki = 0.5;
    config.Kp = 2;
    config.out_max = 20;
    config.out_min = 0;

    PID pid(config);
    pid.update_setpoint(5);

    std::vector<double> outputs;

    outputs.push_back(pid.compute(0, 0));
    outputs.push_back(pid.compute(1, 1));
    outputs.push_back(pid.compute(3, 2));
    outputs.push_back(pid.compute(8, 5));
    outputs.push_back(pid.compute(5, 2));
    outputs.push_back(pid.compute(5, 0));

    for(double o : outputs)
    {
        std::cout << o << std::endl;
    }

    return 0;
}
