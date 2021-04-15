#pragma once

#include <stdexcept>
#include <cmath>
#include <pigpiod_if2.h>
#include <ros/ros.h>


namespace melex_os
{
    class ModeManager
    {
        int pigpioId_;   
        unsigned int modePin1_;
        unsigned int modePin2_;
        unsigned int switchPin_;
        int switchState;
        int lastSwitchState = 10; // non-valid default value to check the switch at start 
        bool lastModeMsg;
        unsigned int modeState;
    public:
        ModeManager(int pigpioId, unsigned int modePin1, unsigned int modePin2, unsigned int switchPin) : 
            pigpioId_(pigpioId), modePin1_(modePin1), modePin2_(modePin2), switchPin_(switchPin)
        {
           

            if(set_mode(pigpioId_, switchPin_, PI_INPUT) < 0)
            {
                throw std::runtime_error("Failed to set up mode switch pin");
            }

            if(set_pull_up_down(pigpioId_, switchPin_, PI_PUD_DOWN) < 0)
            {
                throw std::runtime_error("Failed to set pull up/down mode switch");
            }

            if(set_mode(pigpioId, modePin1_, PI_OUTPUT) < 0)
            {
                throw std::runtime_error("Failed to set up mode pin1");
            }

            if(set_mode(pigpioId, modePin2_, PI_OUTPUT) < 0)
            {
                throw std::runtime_error("Failed to set up mode pin2");
            }
        }

       

        void setMode(bool modeMsg, bool&  currentMode) // 
        { 
            // 0 - manual
            // 1 - stop
            // 2 - auto
            bool newSwitchFlag = false;

            switchState = gpio_read(pigpioId_, switchPin_);
            if(switchState != lastSwitchState)
            {
                ROS_INFO_STREAM("Mode switch state changed! ");
                ROS_INFO_STREAM("Switch value: " << switchState);
                if (switchState == 0)
                    modeState = 0;
                else
                    modeState = 2;
                lastSwitchState = switchState;
                newSwitchFlag = true;
            }

            if(modeMsg != lastModeMsg)
            {
                ROS_INFO_STREAM("New mode msg recieved! ");
                ROS_INFO_STREAM("Msg value: " << modeMsg);
                modeState = modeMsg ? 2 : 0;

                lastModeMsg = modeMsg;
                newSwitchFlag = true;
            }
            

            if(newSwitchFlag)
            {
                if(modeState >= 0 && modeState <= 2)
                {
                    if(modeState == 0)
                    {
                        gpio_write(pigpioId_, modePin1_, 0);
                        gpio_write(pigpioId_, modePin2_, 0);
                        ROS_INFO("Setting mode manual");
                        currentMode = false;
                    }
                    if(modeState == 1)
                    {
                        gpio_write(pigpioId_, modePin1_, 1);
                        gpio_write(pigpioId_, modePin2_, 0);
                        ROS_INFO("Setting mode stop");
                        currentMode = false;
                    }
                    if(modeState == 2)
                    {
                        gpio_write(pigpioId_, modePin1_, 0);
                        gpio_write(pigpioId_, modePin2_, 1);
                        ROS_INFO("Setting mode auto");
                        currentMode = true;
                    }
                }else
                {
                    ROS_WARN("Invalid mode value. Choose frome: 0 - manual, 1 - stop, 2 - auto");
                }

                newSwitchFlag = false;
            }

        }

            
    };
}