#pragma once

#include <stdexcept>
#include <cmath>
#include <pigpiod_if2.h>
#include <ros/ros.h>


namespace melex_os
{
    class PWM
    {
        int pigpioId_;   
        unsigned int pwmPin_;
        unsigned int dirPin_;
    public:
        PWM(int pigpioId, unsigned int dirPin, unsigned int pwmPin, unsigned int pwmFrequency) : pigpioId_(pigpioId), pwmPin_(pwmPin), dirPin_(dirPin)
        {
            if(set_mode(pigpioId, dirPin, PI_OUTPUT) < 0)
            {
                throw std::runtime_error("Failed to set up steering direction pin");
            }

            if(set_pull_up_down(pigpioId_, dirPin_, PI_PUD_OFF) < 0)
            {
                throw std::runtime_error("Failed to set pull up/down");
            }

            if(set_pull_up_down(pigpioId_, pwmPin_, PI_PUD_OFF) < 0)
            {
                throw std::runtime_error("Failed to set pull up/down");
            }

            if(set_mode(pigpioId, pwmPin, PI_OUTPUT) < 0)
            {
                throw std::runtime_error("Failed to set up  steering pwm pin");
            }

            if(set_PWM_frequency(pigpioId, pwmPin, pwmFrequency) < 0)
            {
                throw std::runtime_error("Failed to set up  steering pwm  frequency");
            }
        }

        void setSignedDutyCycle(double dc)
        {
            unsigned int scaledDc = (unsigned int) fabs(dc)*255;
            if(set_PWM_dutycycle(pigpioId_, pwmPin_, scaledDc) < 0)
            {
                ROS_ERROR("Cannot set duty cycle");
            }

            if(gpio_write(pigpioId_, dirPin_, scaledDc >= 0 ? 1 : 0) < 0)
            {
                ROS_ERROR("Failed to set direction");
            }
            
        }
            
    };
}