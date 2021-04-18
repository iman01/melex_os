#pragma once

#include <ros/ros.h>
#include <pigpiod_if2.h>
#include <vector>

namespace melex_os
{
    

    class Speed
    {
        double wheelR = 0.28; //radius of wheel in meter
        double wheelCirFer= 2*M_PI*wheelR;
        double pulsesPerRotation = 48.0; 
        double rpm = 0.0;
        double pulse = 0.0; 
        double dt;
        double t1 = ros::Time::now().toSec();
        std::vector<double> dtArray = {0.0, 0.0, 0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0 , 0.0};
        double vel = 0.0;
        int pigpioId_;
        unsigned int speedPin_;

    public:
        Speed(int pigpioId, unsigned int speedPin) : pigpioId_(pigpioId), speedPin_(speedPin)
        {
            if(set_mode(pigpioId_, speedPin_, PI_INPUT) < 0)
            {
                throw std::runtime_error("Failed to set up speed sensor pin");
            }

            if(set_pull_up_down(pigpioId_, speedPin_, PI_PUD_DOWN) < 0)
            {
                throw std::runtime_error("Failed to set pull up/down speed sensor");
            }
        }

        void calculate_dt()
        {
            pulse += 1;
            dt = ros::Time::now().toSec() - t1;
            t1 = ros::Time::now().toSec();
            dtArray.pop_back();
            dtArray.insert(dtArray.begin(), dt);
        }

        void calculate_velocity(double& vel)
        {
            double currentVel = 0;

            for(auto & dtElement : dtArray) 
            {
                if(dtElement != 0)
                {
                    rpm = (1/(dtElement*pulsesPerRotation)) * 60;
                    vel = (wheelCirFer/pulsesPerRotation)/dtElement;
                    currentVel += vel/2;
                }	
            }
            vel = currentVel/dtArray.size();
        }

        double update()
        {
            if(!wait_for_edge(pigpioId_, speedPin_, RISING_EDGE, 500))
            {
                calculate_dt();
                calculate_velocity(vel);
            }else
            {
                dtArray = {0.0, 0.0, 0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0 , 0.0};
                rpm = 0.0;
                vel = 0.0;
            }

            return vel;            
        }
    };













}